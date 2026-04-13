#!/usr/bin/env python3
"""
O'CELL BMS CAN Dashboard — Backend Server
==========================================
Reads ESP32 + MCP2515 output over **WiFi TCP**, decodes O'CELL IFS60.8-500
19S1P LiFePO4 CAN frames, streams live JSON to the browser via WebSocket.

Battery: IFS60.8-500-F-E3  19S1P LiFePO4  60.8V / 50Ah
CAN baud: 250 kbps

Requirements:  pip install fastapi uvicorn

Usage:
    python server.py                        # listens on 0.0.0.0:9000 (ESP32 TCP)
    python server.py --esp-port 9000        # default ESP32 TCP port
    python server.py --web-port 8765        # default browser WebSocket port
    python server.py --host 0.0.0.0        # default bind address

ESP32 side: connect to <this PC's IP>:<esp-port> and send the same text
lines it used to print over Serial, e.g.:
    [1234ms] ID: 0x18C8C828F4 DLC: 8 Data: 0D 05 0D 06 0D 07 0D 08

VERIFIED FRAME MAP  (confirmed from 3 real car logs + O'CELL spec)
-------------------------------------------------------------------
0x18C8-CC28F4  Cell voltages — 5 frames, 4 cells each, big-endian uint16 mV
               0xC8=cells 1-4  0xC9=cells 5-8  0xCA=cells 9-12
               0xCB=cells 13-16  0xCC=cells 17-19  (last slot padded 0x0000)

0x18B428F4     Temperatures — up to 4 sensors
               bytes 0-3: raw - 40 = deg C   0xFF = sensor not connected

0x18FFE5F4     SOC + charge request  (every ~500 ms)
               bytes 0-1 LE uint16 / 10  = SOC %  (BMS coulomb counter, can drift)
               bytes 2-3 LE uint16 / 10  = charge current request A

0x18FF28F4     Pack summary  (every ~100 ms)
               bytes 0-1 LE uint16        = UNKNOWN — not decodable as pack voltage
               bytes 2-3 LE uint16 / 100 = discharge current limit A
               bytes 4-5 LE uint16 / 10  = BMS secondary SOC %

0x18FE28F4     Min/Max cell + temps + disch limit  (every ~100 ms)
               bytes 0-1 LE uint16        = max cell voltage mV
               bytes 2-3 LE uint16        = min cell voltage mV
               byte  4                    = temp sensor 1, raw - 40 = deg C
               byte  5                    = temp sensor 2, raw - 40 = deg C
               bytes 6-7 LE uint16 / 10  = discharge current limit A (BMS internal)

SOC CALIBRATION — matched to car display
-----------------------------------------
Calibrated from 4 real car logs:
  2026-03-31 10:06  avg=3301.6 mV  car=89.0%  solved_top=3400.7 mV
  2026-03-31 10:57  avg=3306.2 mV  car=89.0%  solved_top=3405.8 mV
  2026-04-01 13:03  avg=3284.8 mV  car=89.4%  solved_top=3377.9 mV
  2026-04-01 16:04  avg=3285.0 mV  car=88.4%  solved_top=3387.4 mV  ← new
  Average -> SOC_CAR_TOP_MV = 3387 mV  = 64.35 V pack

Formula:  SOC = (avg_cell_mV - 2500) / (3387 - 2500) * 100
  2500 mV/cell = 47.50 V pack = 0%
  3387 mV/cell = 64.35 V pack = 100%
  Values above 3387 mV (during charging) are clamped to 100%.

Pack voltage: sum of 19 cell voltages / 1000 — matches car display to 0.01V.
Battery capacity: 44.7 Ah actual / 50.0 Ah rated -> SOH 89.4%
"""

import re, sys, time, asyncio, argparse, threading, socket
from collections import deque
from datetime import datetime

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
except ImportError:
    print("ERROR: pip install fastapi uvicorn"); sys.exit(1)


# ---------------------------------------------------------------------------
# Frame parser
# ---------------------------------------------------------------------------

FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:\s+([0-9A-Fa-f\s]+)'
)


def parse_line(line):
    m = FRAME_RE.search(line)
    if not m:
        return None
    return (
        int(m.group(1)),
        int(m.group(2), 16),
        int(m.group(3)),
        bytes(int(b, 16) for b in m.group(4).split()),
    )


def u16be(d, o):
    return (d[o] << 8) | d[o + 1]


def u16le(d, o):
    return d[o] | (d[o + 1] << 8)


# ---------------------------------------------------------------------------
# BMS State
# ---------------------------------------------------------------------------

class BMSState:

    # Protection thresholds (O'CELL spec pages 3, 10-11)
    CELL_OV        = 3750   # mV  over-voltage trigger (TYP)
    CELL_OV_REL    = 3500   # mV  over-voltage release
    CELL_FULL      = 3650   # mV  absolute charge cut-off
    CELL_BAL_THR   = 3300   # mV  cell balancing start threshold
    CELL_BAL_DELTA = 20     # mV  balancing delta
    CELL_NOMINAL   = 3200   # mV  nominal mid-charge voltage
    CELL_UV        = 2500   # mV  under-voltage protection / SOC 0% reference
    CELL_UV_REL    = 2800   # mV  under-voltage release

    PACK_FULL_V    = 69.35  # V  = 3.65V x 19
    PACK_EMPTY_V   = 47.50  # V  = 2.50V x 19  (SOC 0%)
    PACK_NOM_V     = 60.80  # V  nominal

    # SOC calibration — matches car display
    SOC_CAR_TOP_MV = 3387   # mV  = 64.35V pack = 100% on car display

    # Battery capacity
    CAP_RATED_AH   = 50.0   # Ah  spec rated
    CAP_ACTUAL_AH  = 44.7   # Ah  measured  (SOH = 89.4%)

    MAX_CHARGE_A   = 25.0   # A
    MAX_DISCH_A    = 50.0   # A
    NUM_CELLS      = 19

    def __init__(self):
        self.lock         = threading.Lock()
        self.cell_mv      = {}
        self.temp_c       = {}
        self.soc_coulomb  = None
        self.soc_bms      = None
        self.chg_i_req    = None
        self.disch_i_lim  = None
        self.cell_max_mv  = None
        self.cell_min_mv  = None
        self.frame_count  = 0
        self.connected    = False
        self.client_addr  = ""
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
                        mv = u16be(data, o)
                        if mv != 0:
                            self.cell_mv[base + i] = mv

            elif func == 0xB4 and dlc == 8:
                for i in range(4):
                    raw = data[i]
                    if raw != 0xFF and raw != 0x00:
                        self.temp_c[i + 1] = float(raw) - 40.0

            elif func == 0xFF and sub == 0xE5 and dlc == 8 and len(data) >= 4:
                self.soc_coulomb = u16le(data, 0) / 10.0
                self.chg_i_req   = u16le(data, 2) / 10.0

            elif func == 0xFF and sub == 0x28 and dlc == 8 and len(data) >= 6:
                self.disch_i_lim = u16le(data, 2) / 100.0
                self.soc_bms     = u16le(data, 4) / 10.0

            elif func == 0xFE and sub == 0x28 and dlc == 8 and len(data) >= 8:
                self.cell_max_mv = u16le(data, 0)
                self.cell_min_mv = u16le(data, 2)
                for i in range(2):
                    raw = data[4 + i]
                    if raw != 0xFF and raw != 0x00:
                        self.temp_c[i + 1] = float(raw) - 40.0

    def cell_status(self, mv):
        if mv >= self.CELL_OV:        return "ov"
        if mv >= self.CELL_FULL:      return "full"
        if mv >= self.CELL_BAL_THR:   return "bal"
        if mv >= self.CELL_NOMINAL:   return "ok"
        if mv >= self.CELL_UV_REL:    return "low"
        if mv >= self.CELL_UV:        return "uv_warn"
        return "uv"

    def _soc_display(self, cv):
        if not cv:
            return None
        avg = sum(cv.values()) / len(cv)
        soc = (avg - self.CELL_UV) / (self.SOC_CAR_TOP_MV - self.CELL_UV) * 100.0
        return round(min(max(soc, 0.0), 100.0), 1)

    def _remaining_ah(self, soc):
        if soc is None:
            return None
        return round(self.CAP_ACTUAL_AH * soc / 100.0, 1)

    def to_dict(self):
        with self.lock:
            cv   = dict(self.cell_mv)
            temp = dict(self.temp_c)

        pack_v   = round(sum(cv.values()) / 1000.0, 2) if cv else None
        soc_disp = self._soc_display(cv)

        cells_out = {
            str(k): {"mv": cv[k], "status": self.cell_status(cv[k])}
            for k in sorted(cv)
        }

        return {
            "connected":      self.connected,
            "client":         self.client_addr,
            "frame_count":    self.frame_count,
            "timestamp":      self.last_update.isoformat() if self.last_update else None,

            "soc":            soc_disp,
            "soc_coulomb":    round(self.soc_coulomb, 1) if self.soc_coulomb is not None else None,
            "soc_bms":        round(self.soc_bms,     1) if self.soc_bms     is not None else None,

            "remaining_ah":   self._remaining_ah(soc_disp),
            "capacity_ah":    self.CAP_ACTUAL_AH,
            "capacity_rated": self.CAP_RATED_AH,
            "soh":            round(self.CAP_ACTUAL_AH / self.CAP_RATED_AH * 100, 1),

            "pack_v":         pack_v,
            "chg_i_req":      round(self.chg_i_req,   1) if self.chg_i_req   is not None else None,
            "disch_i_lim":    round(self.disch_i_lim, 1) if self.disch_i_lim is not None else None,

            "cells":          cells_out,
            "cell_count":     len(cv),
            "cell_max_mv":    self.cell_max_mv,
            "cell_min_mv":    self.cell_min_mv,
            "cell_spread_mv": (max(cv.values()) - min(cv.values())) if len(cv) > 1 else None,
            "cell_avg_mv":    round(sum(cv.values()) / len(cv))     if cv else None,

            "temps":          {str(k): round(v, 1) for k, v in sorted(temp.items())},
            "avg_temp":       round(sum(temp.values()) / len(temp), 1) if temp else None,

            "thresh": {
                "cell_ov":      self.CELL_OV,
                "cell_ov_rel":  self.CELL_OV_REL,
                "cell_full":    self.CELL_FULL,
                "cell_soc_top": self.SOC_CAR_TOP_MV,
                "cell_bal":     self.CELL_BAL_THR,
                "cell_bal_d":   self.CELL_BAL_DELTA,
                "cell_nom":     self.CELL_NOMINAL,
                "cell_uv":      self.CELL_UV,
                "cell_uv_rel":  self.CELL_UV_REL,
                "pack_full":    self.PACK_FULL_V,
                "pack_soc_top": round(self.SOC_CAR_TOP_MV * 19 / 1000, 2),
                "pack_empty":   self.PACK_EMPTY_V,
                "max_chg_a":    self.MAX_CHARGE_A,
                "max_dch_a":    self.MAX_DISCH_A,
            },

            "log": list(self.raw_log)[-80:],
        }


# ---------------------------------------------------------------------------
# Normalise incoming text lines (same as before)
# ---------------------------------------------------------------------------

_start_time = time.time()


def millis():
    return int((time.time() - _start_time) * 1000)


def normalize_line(line):
    if line.startswith("[") and "ID:" in line and "Data:" in line:
        return line
    line = line.replace("Extended ID:", "ID:").replace("extended ID:", "ID:")
    if "ID:" in line and "Data:" in line:
        head, data_part = line.split("Data:", 1)
        clean = " ".join(
            t.replace("0x", "").replace("0X", "")
            for t in data_part.split() if t.strip()
        )
        return f"[{millis()}ms] {head.strip()} Data: {clean}"
    return line


# ---------------------------------------------------------------------------
# WiFi TCP reader  (replaces serial_reader)
# ---------------------------------------------------------------------------

def handle_client(conn, addr, state):
    """Process one ESP32 TCP connection until it drops."""
    state.connected  = True
    state.client_addr = f"{addr[0]}:{addr[1]}"
    state.raw_log.append(f"[INFO] ESP32 connected from {state.client_addr}")
    buf = b""
    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break                       # ESP32 closed connection
            buf += chunk
            while b"\n" in buf:
                raw_line, buf = buf.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                line = normalize_line(line)
                state.raw_log.append(line)
                result = parse_line(line)
                if result:
                    state.decode(*result)
    except OSError as e:
        state.raw_log.append(f"[ERR] {e}")
    finally:
        conn.close()
        state.connected   = False
        state.client_addr = ""
        state.raw_log.append(f"[INFO] ESP32 disconnected")


def tcp_reader(tcp_port, state, stop):
    """
    Listens on *tcp_port* and accepts one ESP32 at a time.
    Reconnections are handled automatically.
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", tcp_port))
    srv.listen(1)
    srv.settimeout(1.0)             # so we can check stop event
    state.raw_log.append(f"[INFO] Waiting for ESP32 on TCP port {tcp_port} …")

    while not stop.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError as e:
            state.raw_log.append(f"[ERR] TCP accept: {e}")
            time.sleep(1)
            continue
        # Handle the client in the same thread (one ESP32 at a time is enough)
        handle_client(conn, addr, state)

    srv.close()


# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

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
            await asyncio.sleep(0.1)    # 10 Hz
    except (WebSocketDisconnect, Exception):
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="O'CELL BMS CAN dashboard server")
    parser.add_argument("--esp-port",  "-e", type=int, default=9000,
                        dest="esp_port",
                        help="TCP port the ESP32 connects to (default 9000)")
    parser.add_argument("--host",            default="0.0.0.0",
                        help="Bind address for the web server (default 0.0.0.0)")
    parser.add_argument("--web-port",  "-w", type=int, default=8765,
                        dest="web_port",
                        help="HTTP/WebSocket port for the browser (default 8765)")
    args = parser.parse_args()

    top_v = round(BMSState.SOC_CAR_TOP_MV * 19 / 1000, 2)
    cap   = BMSState.CAP_ACTUAL_AH
    soh   = round(cap / BMSState.CAP_RATED_AH * 100, 1)

    print("-" * 50)
    print("  O'CELL BMS Dashboard  (WiFi mode)")
    print("-" * 50)
    print(f"  ESP32 TCP  : 0.0.0.0:{args.esp_port}  ← point your ESP32 here")
    print(f"  Browser    : http://localhost:{args.web_port}")
    print(f"  SOC range  : {BMSState.PACK_EMPTY_V} V = 0%  ->  {top_v} V = 100%")
    print(f"  Capacity   : {cap} Ah  (rated {BMSState.CAP_RATED_AH} Ah, SOH {soh}%)")
    print("-" * 50)

    threading.Thread(target=tcp_reader,
                     args=(args.esp_port, bms, _stop),
                     daemon=True).start()

    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
