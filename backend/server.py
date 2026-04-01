#!/usr/bin/env python3
"""
O'CELL BMS CAN Dashboard — Backend Server
==========================================
Reads ESP32 serial output, decodes O'CELL 19S1P CAN frames,
streams live JSON to the browser via WebSocket.

Battery: IFS60.8-500-F-E3  19S1P LiFePO4  60.8V / 50Ah
CAN baud: 250 kbps

Requirements:  pip install fastapi uvicorn pyserial

Usage:
    python server.py                      # auto-detect USB port
    python server.py --port COM3
    python server.py --port /dev/ttyUSB0
    python server.py --baud 115200        # default
    python server.py --web-port 8765      # default
"""

import re, sys, time, asyncio, argparse, threading
from collections import deque
from datetime import datetime
from typing import Optional

try:
    import serial, serial.tools.list_ports
except ImportError:
    print("ERROR: pip install pyserial"); sys.exit(1)

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
except ImportError:
    print("ERROR: pip install fastapi uvicorn"); sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# O'CELL Frame Decoder
#
# Frame ID: 0x98_FUNC_SUB_F4  (29-bit extended)
#
# 0x98C8–CC28F4  Cell voltages 1-19   BE uint16, mV
# 0x98B428F4     Temperatures         byte − 40 = °C
# 0x98FFE5F4     SOC + chg current    LE uint16 ×10
# 0x98FF28F4     Pack V / disch lim / SOC   LE uint16
# 0x98FE28F4     Min/Max cell + disch lim   LE uint16
# ─────────────────────────────────────────────────────────────────────────────

FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:\s+([0-9A-Fa-f\s]+)'
)

def parse_line(line):
    m = FRAME_RE.search(line)
    if not m: return None
    return (int(m.group(1)), int(m.group(2), 16), int(m.group(3)),
            bytes(int(b, 16) for b in m.group(4).split()))

def u16be(d, o): return (d[o] << 8) | d[o+1]
def u16le(d, o): return d[o] | (d[o+1] << 8)


class BMSState:
    CELL_OV       = 3750
    CELL_UV       = 2500
    CELL_FULL     = 3650
    CELL_NOMINAL  = 3200
    CELL_BAL_THR  = 3300
    CELL_BAL_DELT = 20
    PACK_FULL_V   = 69.35
    PACK_EMPTY_V  = 47.50
    PACK_NOM_V    = 60.80
    MAX_CHARGE_A  = 25.0
    MAX_DISCH_A   = 50.0
    NUM_CELLS     = 19

    def __init__(self):
        self.lock         = threading.Lock()
        self.cell_mv      = {}
        self.temp_c       = {}
        self.pack_v       = None
        self.soc          = None
        self.soc_ffe5     = None
        self.chg_i_req    = None
        self.disch_i_lim  = None
        self.cell_max_mv  = None
        self.cell_min_mv  = None
        self.frame_count  = 0
        self.connected    = False
        self.port_name    = ""
        self.last_update  = None
        self.raw_log      = deque(maxlen=300)

    def decode(self, ts, can_id, dlc, data):
        func = (can_id >> 16) & 0xFF
        sub  = (can_id >>  8) & 0xFF

        with self.lock:
            self.frame_count += 1
            self.last_update  = datetime.now()

            if 0xC8 <= func <= 0xCC and dlc == 8:
                group = func - 0xC8
                base  = group * 4 + 1
                for i in range(4):
                    o = i * 2
                    if o + 1 < len(data):
                        v = u16be(data, o)
                        if v != 0:
                            self.cell_mv[base + i] = v

            elif func == 0xB4 and dlc == 8:
                for i in range(3):
                    raw = data[i]
                    if raw != 0:
                        self.temp_c[i + 1] = float(raw) - 40.0

            elif func == 0xFF and sub == 0xE5 and dlc == 8 and len(data) >= 4:
                self.soc_ffe5  = u16le(data, 0) / 10.0
                self.chg_i_req = u16le(data, 2) / 10.0

            elif func == 0xFF and sub == 0x28 and dlc == 8 and len(data) >= 6:
                self.pack_v      = u16le(data, 0) / 100.0
                self.disch_i_lim = u16le(data, 2) / 100.0
                self.soc         = u16le(data, 4) / 10.0

            elif func == 0xFE and sub == 0x28 and dlc == 8 and len(data) >= 8:
                self.cell_max_mv = u16le(data, 0)
                self.cell_min_mv = u16le(data, 2)
                for i in range(2):
                    raw = data[4 + i]
                    if raw != 0:
                        self.temp_c[i + 1] = float(raw) - 40.0
                self.disch_i_lim = u16le(data, 6) / 10.0

    def cell_status(self, mv):
        if mv >= self.CELL_OV:      return "overvoltage"
        if mv >= self.CELL_FULL:    return "full"
        if mv >= self.CELL_BAL_THR: return "good"
        if mv >= self.CELL_NOMINAL: return "normal"
        if mv >= self.CELL_UV:      return "low"
        return "undervoltage"

    def to_dict(self):
        with self.lock:
            cv   = dict(self.cell_mv)
            temp = dict(self.temp_c)
            soc  = self.soc_ffe5 if self.soc_ffe5 is not None else self.soc
            cells_out = {str(k): {"mv": cv[k], "status": self.cell_status(cv[k])}
                         for k in sorted(cv)}
            return {
                "connected":      self.connected,
                "port":           self.port_name,
                "frame_count":    self.frame_count,
                "timestamp":      self.last_update.isoformat() if self.last_update else None,
                "soc":            round(soc, 1)               if soc              is not None else None,
                "pack_v":         round(self.pack_v, 2)        if self.pack_v      is not None else None,
                "chg_i_req":      round(self.chg_i_req, 1)     if self.chg_i_req   is not None else None,
                "disch_i_lim":    round(self.disch_i_lim, 1)   if self.disch_i_lim is not None else None,
                "cells":          cells_out,
                "cell_count":     len(cv),
                "cell_max_mv":    self.cell_max_mv,
                "cell_min_mv":    self.cell_min_mv,
                "cell_spread_mv": (max(cv.values()) - min(cv.values())) if len(cv) > 1 else None,
                "cell_avg_mv":    round(sum(cv.values()) / len(cv))     if cv else None,
                "temps":          {str(k): round(v, 1) for k, v in sorted(temp.items())},
                "avg_temp":       round(sum(temp.values()) / len(temp), 1) if temp else None,
                "thresh": {
                    "cell_ov":    self.CELL_OV,
                    "cell_full":  self.CELL_FULL,
                    "cell_nom":   self.CELL_NOMINAL,
                    "cell_bal":   self.CELL_BAL_THR,
                    "cell_uv":    self.CELL_UV,
                    "pack_full":  self.PACK_FULL_V,
                    "pack_empty": self.PACK_EMPTY_V,
                    "max_chg_a":  self.MAX_CHARGE_A,
                    "max_dch_a":  self.MAX_DISCH_A,
                },
                "log": list(self.raw_log)[-80:],
            }


# ─────────────────────────────────────────────────────────────────────────────
# Serial reader
# ─────────────────────────────────────────────────────────────────────────────

def auto_detect_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc + hwid for k in ["cp210", "ch340", "ftdi", "esp32", "silicon labs", "uart"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


_start_time = time.time()
def millis() -> int:
    return int((time.time() - _start_time) * 1000)


def normalize_line(line: str) -> str:
    """
    Normalize any CAN log line into the standard format the parser expects:
      [Nms] ID: 0x1FFFFFFF DLC: 8 Data: AA BB CC DD EE FF 00 11

    Handles two variants:
      A — ESP32 logger (already correct):
            [1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 ...
      B — other CAN tools (needs fixing):
            Extended ID: 0x18FE28F4  DLC: 8  Data: 0x43 0x0D 0x34 ...
    """
    # Already in correct format
    if line.startswith("[") and "ID:" in line and "Data:" in line:
        return line

    # Variant B: strip "Extended " prefix
    line = line.replace("Extended ID:", "ID:").replace("extended ID:", "ID:")

    if "ID:" in line and "Data:" in line:
        head, data_part = line.split("Data:", 1)
        # Remove 0x prefix from each byte in the data section only
        clean_bytes = " ".join(
            tok.replace("0x", "").replace("0X", "")
            for tok in data_part.split()
            if tok.strip()
        )
        return f"[{millis()}ms] {head.strip()} Data: {clean_bytes}"

    return line


def serial_reader(port, baud, state, stop):
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                state.connected = True
                state.port_name = port
                state.raw_log.append(f"[INFO] Connected to {port} @ {baud}")
                while not stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    line = normalize_line(line)
                    state.raw_log.append(line)
                    result = parse_line(line)
                    if result:
                        state.decode(*result)
        except serial.SerialException as e:
            state.connected = False
            state.raw_log.append(f"[ERR] {e}")
            time.sleep(2)
        except Exception as e:
            state.raw_log.append(f"[ERR] {e}")
            time.sleep(1)


# ─────────────────────────────────────────────────────────────────────────────
# FastAPI
# ─────────────────────────────────────────────────────────────────────────────

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

bms   = BMSState()
_stop = threading.Event()


@app.get("/", response_class=HTMLResponse)
async def index():
    with open("index.html", encoding="utf-8") as f:
        return f.read()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(bms.to_dict())
            await asyncio.sleep(0.1)   # 10 Hz
    except (WebSocketDisconnect, Exception):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",     "-p")
    parser.add_argument("--baud",     "-b", type=int, default=115200)
    parser.add_argument("--host",           default="0.0.0.0")
    parser.add_argument("--web-port",       type=int, default=8765, dest="web_port")
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("❌  No serial port found. Use --port to specify one.")
        for p in serial.tools.list_ports.comports():
            print(f"    {p.device}  {p.description}")
        sys.exit(1)

    print(f"🔌  Serial : {port} @ {args.baud} baud")
    print(f"🌐  Open   : http://localhost:{args.web_port}")

    threading.Thread(target=serial_reader, args=(port, args.baud, bms, _stop),
                     daemon=True).start()
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()