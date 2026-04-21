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
    python server_wifi.py                        # listens on 0.0.0.0:9000 (ESP32 TCP)
    python server_wifi.py --replay ../data/raw/bms_log_2026-03-10T10-20-02.txt
    python server_wifi.py --replay <file> --speed 5   # 5× real-time

VERIFIED FRAME MAP  (BAKO Motors CAN Protocol v1.0 — confirmed from captured logs)
------------------------------------------------------------------------------------
0x18C8-CC28F4  Cell voltages — 5 frames, 4 cells each, BIG-ENDIAN uint16 mV
               0xC8=cells 1-4  0xC9=cells 5-8  0xCA=cells 9-12
               0xCB=cells 13-16  0xCC=cells 17-19  (last slot padded 0x0000)

0x18B428F4     Temperatures — up to 8 probes, uint8 raw−40 = °C, 0xFF = not connected

0x18FF28F4     BMS Basic Message 1 (100 ms)
               byte  0       = status bit-field  (bit0=chg_cable, 1=charging, 2=discharging,
                                                   3=ready, 4=disch_contactor, 5=chg_contactor)
               byte  1       = SOC %  (BMS coulomb counter, 0-100 integer)
               bytes 2-3 LE int16 offset-5000 × 0.1 A = pack current (+discharge/-charge)
               bytes 4-5 LE uint16 × 0.1 V  = pack total voltage
               byte  6       = fault level  (0=ok, 1=serious fault)
               byte  7       = error code   (see FAULT_NAMES table)

0x18FE28F4     BMS Basic Message 2 (100 ms)
               bytes 0-1 LE uint16 mV = max cell voltage
               bytes 2-3 LE uint16 mV = min cell voltage
               byte  4    uint8 −40   = max cell temperature (°C)
               byte  5    uint8 −40   = min cell temperature (°C)
               bytes 6-7 LE uint16 × 0.1 A = max allowable discharge current

0x18FFE5F4     BMS Charging Request (1000 ms, to on-board charger)
               bytes 0-1 LE uint16 × 0.1 V = max allowable charge terminal voltage
               bytes 2-3 LE uint16 × 0.1 A = max allowable charge current
               byte  4 bit0 = charger start signal (0 = start charging)

SOC CALIBRATION — matched to car display
-----------------------------------------
Calibrated from 4 real car logs:
  Formula:  SOC = (avg_cell_mV - 2500) / (3387 - 2500) × 100
  2500 mV/cell = 47.50 V pack = 0%
  3387 mV/cell = 64.35 V pack = 100%
  Values above 3387 mV (during charging) are clamped to 100%.
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
# Fault code lookup  (BAKO CAN Protocol doc — section 4)
# ---------------------------------------------------------------------------

FAULT_NAMES = {
    0x00: "ok",
    0x01: "over_temp_severe",
    0x02: "total_voltage_high",
    0x03: "total_voltage_low",
    0x04: "discharge_overcurrent",
    0x05: "cell_voltage_high",
    0x06: "cell_voltage_low",
}


# ---------------------------------------------------------------------------
# BMS State
# ---------------------------------------------------------------------------

class BMSState:

    # Protection thresholds (O'CELL spec)
    CELL_OV        = 3750   # mV  over-voltage trigger
    CELL_OV_REL    = 3500   # mV  over-voltage release
    CELL_FULL      = 3650   # mV  charge cut-off
    CELL_BAL_THR   = 3300   # mV  balancing start
    CELL_BAL_DELTA = 20     # mV  balancing delta
    CELL_NOMINAL   = 3200   # mV  nominal mid-charge voltage
    CELL_UV        = 2500   # mV  under-voltage / SOC 0% reference
    CELL_UV_REL    = 2800   # mV  under-voltage release

    PACK_FULL_V    = 69.35  # V  = 3.65 V × 19
    PACK_EMPTY_V   = 47.50  # V  = 2.50 V × 19
    PACK_NOM_V     = 60.80  # V  nominal

    SOC_CAR_TOP_MV = 3387   # mV  = 64.35 V pack = 100% on car display
    CAP_RATED_AH   = 50.0
    CAP_ACTUAL_AH  = 44.7
    MAX_CHARGE_A   = 25.0
    MAX_DISCH_A    = 50.0
    NUM_CELLS      = 19

    def __init__(self):
        self.lock         = threading.Lock()

        # Cell voltages & temperatures
        self.cell_mv      = {}          # {1..19: mV}
        self.temp_c       = {}          # {1..8: °C}  from 0x18B428F4

        # 0x18FF28F4 — BMS Basic Message 1
        self.status_flags = 0           # bit-field byte 0
        self.soc_bms      = None        # byte 1: coulomb-counter SOC 0-100
        self.pack_current = None        # A (+discharge / -charge)
        self.pack_v_bms   = None        # V  direct BMS reading
        self.fault_level  = 0           # 0=ok, 1=serious
        self.error_code   = 0           # fault code (0x00-0x06)

        # 0x18FE28F4 — BMS Basic Message 2
        self.cell_max_mv  = None
        self.cell_min_mv  = None
        self.temp_max_c   = None        # max cell temp from BMS
        self.temp_min_c   = None        # min cell temp from BMS
        self.max_disch_a  = None        # max allowable discharge current

        # 0x18FFE5F4 — BMS Charging Request
        self.charge_max_v = None        # max allowable charge terminal voltage (V)
        self.charge_max_a = None        # max allowable charge current (A)
        self.charger_start = False      # True = BMS commands charger ON

        # Housekeeping
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

            # ── 0x18C8-CC28F4  Cell voltages (500 ms) ──────────────────────
            # Big-endian uint16 pairs, 4 cells per frame
            if 0xC8 <= func <= 0xCC and dlc == 8:
                group = func - 0xC8
                base  = group * 4 + 1
                for i in range(4):
                    o = i * 2
                    if o + 1 < len(data):
                        mv = u16be(data, o)
                        if mv != 0:
                            self.cell_mv[base + i] = mv

            # ── 0x18B428F4  Temperature probes (500 ms) ────────────────────
            # Up to 8 probes, uint8 raw−40 = °C, 0xFF = not connected
            elif func == 0xB4 and dlc >= 1:
                self.temp_c.clear()
                for i in range(min(dlc, 8)):
                    raw = data[i]
                    if raw != 0xFF and raw != 0x00:
                        self.temp_c[i + 1] = float(raw) - 40.0

            # ── 0x18FF28F4  BMS Basic Message 1 (100 ms) ───────────────────
            elif func == 0xFF and sub == 0x28 and dlc >= 8:
                self.status_flags = data[0]
                self.soc_bms      = data[1]                           # 0-100 integer
                raw_i = u16le(data, 2)
                self.pack_current = round((raw_i - 5000) * 0.1, 1)   # +discharge/-charge
                self.pack_v_bms   = round(u16le(data, 4) * 0.1, 1)   # V
                self.fault_level  = data[6]
                self.error_code   = data[7]

            # ── 0x18FFE5F4  BMS Charging Request (1000 ms) ─────────────────
            elif func == 0xFF and sub == 0xE5 and dlc >= 5:
                self.charge_max_v  = round(u16le(data, 0) * 0.1, 1)
                self.charge_max_a  = round(u16le(data, 2) * 0.1, 1)
                self.charger_start = not bool(data[4] & 0x01)

            # ── 0x18FE28F4  BMS Basic Message 2 (100 ms) ───────────────────
            elif func == 0xFE and sub == 0x28 and dlc >= 8:
                self.cell_max_mv = u16le(data, 0)
                self.cell_min_mv = u16le(data, 2)
                if data[4] != 0xFF:
                    self.temp_max_c = float(data[4]) - 40.0
                if data[5] != 0xFF:
                    self.temp_min_c = float(data[5]) - 40.0
                self.max_disch_a = round(u16le(data, 6) * 0.1, 1)

    def cell_status(self, mv):
        if mv >= self.CELL_OV:        return "overvoltage"
        if mv >= self.CELL_FULL:      return "full"
        if mv >= self.CELL_BAL_THR:   return "good"
        if mv >= self.CELL_NOMINAL:   return "normal"
        if mv >= self.CELL_UV_REL:    return "low"
        if mv >= self.CELL_UV:        return "uv_warn"
        return "undervoltage"

    def _soc_display(self, cv):
        if not cv:
            return None
        avg = sum(cv.values()) / len(cv)
        soc = (avg - self.CELL_UV) / (self.SOC_CAR_TOP_MV - self.CELL_UV) * 100.0
        return round(min(max(soc, 0.0), 100.0), 1)

    def to_dict(self):
        with self.lock:
            cv   = dict(self.cell_mv)
            temp = dict(self.temp_c)

        pack_v_cells = round(sum(cv.values()) / 1000.0, 2) if cv else None
        soc_disp     = self._soc_display(cv)

        # Prefer direct BMS pack voltage when available
        pack_v = self.pack_v_bms if self.pack_v_bms else pack_v_cells

        cells_out = {
            str(k): {"mv": cv[k], "status": self.cell_status(cv[k])}
            for k in sorted(cv)
        }

        # Compute spread / avg from live cell readings when available
        cell_max = max(cv.values())         if cv else self.cell_max_mv
        cell_min = min(cv.values())         if cv else self.cell_min_mv
        cell_avg = round(sum(cv.values()) / len(cv)) if cv else None

        result = {
            "connected":      self.connected,
            "client":         self.client_addr,
            "frame_count":    self.frame_count,
            "timestamp":      self.last_update.isoformat() if self.last_update else None,

            # SOC
            "soc":            soc_disp,
            "soc_bms":        self.soc_bms,       # BMS coulomb counter (integer %)

            # Pack
            "pack_v":         pack_v,
            "pack_current_a": self.pack_current,  # +discharge / -charge

            # Capacity
            "remaining_ah":   round(self.CAP_ACTUAL_AH * soc_disp / 100.0, 1) if soc_disp is not None else None,
            "capacity_ah":    self.CAP_ACTUAL_AH,
            "capacity_rated": self.CAP_RATED_AH,
            "soh":            round(self.CAP_ACTUAL_AH / self.CAP_RATED_AH * 100, 1),

            # Cells
            "cells":          cells_out,
            "cell_count":     len(cv),
            "cell_max_mv":    cell_max,
            "cell_min_mv":    cell_min,
            "cell_avg_mv":    cell_avg,
            "cell_spread_mv": (cell_max - cell_min) if (cell_max is not None and cell_min is not None) else None,

            # Temperatures
            "temps":          {str(k): round(v, 1) for k, v in sorted(temp.items())},
            "avg_temp":       round(sum(temp.values()) / len(temp), 1) if temp else None,
            "temp_max_c":     self.temp_max_c,
            "temp_min_c":     self.temp_min_c,

            # Discharge limit
            "max_disch_a":    self.max_disch_a,

            # Fault
            "fault_level":    self.fault_level,
            "error_code":     self.error_code,
            "fault_name":     FAULT_NAMES.get(self.error_code, "unknown") if self.error_code else None,

            # Status bit-field (0x18FF28F4 byte 0)
            "status": {
                "charge_cable":     bool(self.status_flags & 0x01),
                "charging":         bool(self.status_flags & 0x02),
                "discharging":      bool(self.status_flags & 0x04),
                "ready":            bool(self.status_flags & 0x08),
                "disch_contactor":  bool(self.status_flags & 0x10),
                "charge_contactor": bool(self.status_flags & 0x20),
            },

            # Charging request (0x18FFE5F4) — only present when frame seen
            "charger": {
                "max_charge_v":  self.charge_max_v,
                "max_charge_a":  self.charge_max_a,
                "start_signal":  self.charger_start,
            } if self.charge_max_v is not None else None,

            # Thresholds (for dashboard gauge rendering)
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
        return result


# ---------------------------------------------------------------------------
# Normalise incoming text lines
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
# WiFi TCP reader
# ---------------------------------------------------------------------------

def handle_client(conn, addr, state):
    state.connected   = True
    state.client_addr = f"{addr[0]}:{addr[1]}"
    state.raw_log.append(f"[INFO] ESP32 connected from {state.client_addr}")
    buf = b""
    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
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
        state.raw_log.append("[INFO] ESP32 disconnected")


def tcp_reader(tcp_port, state, stop):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", tcp_port))
    srv.listen(1)
    srv.settimeout(1.0)
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
        handle_client(conn, addr, state)

    srv.close()


# ---------------------------------------------------------------------------
# Log file replay
# ---------------------------------------------------------------------------

def replay_reader(log_path, speed, state, stop):
    """
    Replay a raw CAN log file into BMSState at real-time speed (×speed).
    Loops the file so the dashboard keeps refreshing.
    """
    print(f"[REPLAY] Loading {log_path}  speed={speed}×")
    state.connected   = True
    state.client_addr = f"replay:{log_path}"

    # Pre-parse all frames and timestamps
    frames = []
    with open(log_path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            state.raw_log.append(line)
            result = parse_line(line)
            if result:
                frames.append(result)

    if not frames:
        print("[REPLAY] No parseable frames found in log file")
        return

    print(f"[REPLAY] {len(frames)} frames — starting playback …")
    loop_count = 0

    while not stop.is_set():
        loop_count += 1
        t0_log  = frames[0][0]          # first frame timestamp in log (ms)
        t0_wall = time.time()

        for ts, can_id, dlc, data in frames:
            if stop.is_set():
                break
            # Sleep until the right wall-clock moment relative to log time
            elapsed_log  = (ts - t0_log) / 1000.0 / speed
            elapsed_wall = time.time() - t0_wall
            sleep_s = elapsed_log - elapsed_wall
            if sleep_s > 0:
                time.sleep(sleep_s)

            state.decode(ts, can_id, dlc, data)

        print(f"[REPLAY] Loop {loop_count} complete — restarting …")
        # Brief pause between loops so the dashboard doesn't freeze between cycles
        time.sleep(1.0 / speed)

    state.connected = False


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
    with open("../frontend/index.html", encoding="utf-8") as f:
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
    parser.add_argument("--replay",    "-r", type=str, default=None,
                        help="Replay a raw CAN log file instead of waiting for ESP32")
    parser.add_argument("--speed",     "-s", type=float, default=1.0,
                        help="Replay speed multiplier (default 1.0 = real-time)")
    args = parser.parse_args()

    top_v = round(BMSState.SOC_CAR_TOP_MV * 19 / 1000, 2)

    print("-" * 55)
    print("  O'CELL BMS Dashboard")
    print("-" * 55)
    if args.replay:
        print(f"  Mode       : REPLAY  ({args.replay})")
        print(f"  Speed      : {args.speed}×")
    else:
        print(f"  Mode       : WiFi TCP  (port {args.esp_port})")
    print(f"  Browser    : http://localhost:{args.web_port}")
    print(f"  SOC range  : {BMSState.PACK_EMPTY_V} V = 0%  ->  {top_v} V = 100%")
    print("-" * 55)

    if args.replay:
        threading.Thread(
            target=replay_reader,
            args=(args.replay, args.speed, bms, _stop),
            daemon=True,
        ).start()
    else:
        threading.Thread(
            target=tcp_reader,
            args=(args.esp_port, bms, _stop),
            daemon=True,
        ).start()

    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
