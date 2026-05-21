#!/usr/bin/env python3
"""
BAKO SMU — Combined Backend Server (Serial + WiFi AP, runtime-switchable)
==========================================================================
Merges the work of three teammates into a single backend:

  • Mahdi   — original `server.py` : flat-state CAN parser, SENSOR-line parser,
              FastAPI WebSocket dashboard, robust serial reader.
  • Hana    — `server_AP.py`        : ESP32-as-AP TCP client mode, BMSState
              with O'CELL thresholds + SOC calibration formula.
  • Selima  — `esp32_simulator_*.ino`: 7-scenario test vehicle, 9 extra
              sensors (solar, MPPT, motor, cabin, GNSS, handbrake), all
              encoded as additional CAN frame IDs (0x98D001-D004F4) plus
              a human-readable SENSOR text line.

KEY FEATURE — RUNTIME MODE SWITCHING
------------------------------------
The transport mode (Serial vs WiFi) is selected from the dashboard UI,
not from the command line. On startup the server loads the last-saved
mode from `mode_config.json`. Sending `POST /api/mode` with a JSON body
swaps the active reader thread without restarting the process.

REST API
--------
  GET  /                  → dashboard (index.html)
  GET  /api/mode          → {"mode": "serial"|"wifi"|"off",
                             "serial":{"port","baud"},
                             "wifi":{"esp_ip","esp_port"},
                             "available_ports":[...]}
  POST /api/mode          → body: {"mode":"serial"|"wifi"|"off",
                                   "serial":{...}, "wifi":{...}}
                            → swaps the reader and persists choice
  WS   /ws                → 10 Hz JSON snapshot of all state

Requirements: pip install fastapi uvicorn pyserial
"""

import asyncio, json, os, re, socket, threading, time, copy
from collections import deque
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────────────────
#  Persistent mode config
# ─────────────────────────────────────────────────────────────────────────────
CONFIG_FILE = "mode_config.json"

DEFAULT_CONFIG = {
    "mode": "serial",                # "serial" | "wifi" | "off"
    "serial": {"port": None, "baud": 115200},
    "wifi":   {"esp_ip": "192.168.4.1", "esp_port": 9000},
}

def load_config() -> dict:
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                cfg = json.load(f)
            for k, v in DEFAULT_CONFIG.items():
                if k not in cfg:
                    cfg[k] = v
                elif isinstance(v, dict):
                    for kk, vv in v.items():
                        cfg[k].setdefault(kk, vv)
            return cfg
        except Exception:
            pass
    return copy.deepcopy(DEFAULT_CONFIG)

def save_config(cfg: dict) -> None:
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception as e:
        print(f"[CFG] Could not save config: {e}")


# ─────────────────────────────────────────────────────────────────────────────
#  Shared state — single source of truth, deep-copied for the WebSocket payload
# ─────────────────────────────────────────────────────────────────────────────
state = {
    # BMS / pack
    "soc":            None,
    "soc_coulomb":    None,
    "soc_bms":        None,
    "pack_v":         None,
    "capacity_ah":    44.7,
    "capacity_rated": 50.0,
    "soh":            None,
    "chg_i_req":      None,
    "disch_i_lim":    None,
    "remaining_ah":   None,

    # Cells:  {1: {"mv": 3300, "status": "ok"}, …}
    "cells":          {},
    "cell_count":     0,
    "cell_min_mv":    None,
    "cell_max_mv":    None,
    "cell_spread_mv": None,
    "cell_avg_mv":    None,

    # BMS internal temps  {1: 24.5, 2: 24.0, …}
    "temps":          {},
    "avg_temp":       None,

    # Extended sensors (Selima) — flat keys for the existing dashboard
    "ext": {
        "aux12v":    None,    # DC/DC 12V output
        "hv_iso_v":  None,    # DC/DC HV input = pack voltage
        "mppt_i1":   None,    # solar panel current (before MPPT)
        "mppt_i2":   None,    # MPPT output current (into battery)
        "bat_t1":    None,    # cabin temp (mapped from SENSOR)
        "bat_t2":    None,    # motor temp (mapped from SENSOR)
        "mppt_t":    None,    # MPPT heatsink temp
        "dcdc_t":    None,
        "handbrake": None,
    },
    "ext_thresh": {"mppt_max_a": 22.0},

    # Raw sensor values (some panels read these directly)
    "solar_v":     None,
    "solar_i_in":  None,
    "solar_i_out": None,
    "hv64v":       None,
    "motor_t":     None,
    "cabin_t":     None,

    # GNSS
    "gnss": {"lat": None, "lon": None, "alt": None, "speed": None, "fix": 0},

    # Scenario info (only meaningful when ESP32 runs the simulator firmware)
    "scenario_name":        "—",
    "scenario_countdown_s": None,

    # Thresholds for UI colour coding (Hana's spec values)
    "thresh": {
        "cell_ov":      3750,
        "cell_ov_rel":  3500,
        "cell_full":    3650,
        "cell_soc_top": 3387,
        "cell_bal":     3300,
        "cell_bal_d":   20,
        "cell_nom":     3200,
        "cell_uv":      2500,
        "cell_uv_rel":  2800,
        "pack_full":    69.35,
        "pack_soc_top": round(3387 * 19 / 1000, 2),
        "pack_empty":   47.50,
        "max_chg_a":    25.0,
        "max_dch_a":    50.0,
    },

    # Connection status
    "connected":   False,
    "mode":        "off",
    "port":        "—",
    "frame_count": 0,
    "last_error":  "",
}

# SOC calibration constants (Hana)
SOC_CAR_TOP_MV = 3387
CELL_UV_MV     = 2500
CAP_RATED_AH   = 50.0
CAP_ACTUAL_AH  = 44.7

log_buffer  = deque(maxlen=300)
frame_count = 0
state_lock  = threading.Lock()


# ─────────────────────────────────────────────────────────────────────────────
#  Reader manager — owns the single active background thread.
#  Switching modes asks the current reader to stop, then spawns the new one.
# ─────────────────────────────────────────────────────────────────────────────
class ReaderManager:
    def __init__(self):
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.config = load_config()
        self.lock = threading.Lock()

    def get_config(self) -> dict:
        return copy.deepcopy(self.config)

    def update_config(self, new_cfg: dict) -> dict:
        with self.lock:
            for k, v in new_cfg.items():
                if isinstance(v, dict) and isinstance(self.config.get(k), dict):
                    self.config[k].update(v)
                else:
                    self.config[k] = v
            save_config(self.config)
            return copy.deepcopy(self.config)

    def stop(self):
        if self.thread and self.thread.is_alive():
            self.stop_event.set()
            self.thread.join(timeout=4.0)
        self.thread = None
        self.stop_event = threading.Event()
        with state_lock:
            state["connected"] = False
            state["mode"]      = "off"
            state["port"]      = "—"

    def start(self):
        self.stop()
        mode = self.config["mode"]

        if mode == "serial":
            if not SERIAL_AVAILABLE:
                with state_lock:
                    state["last_error"] = "pyserial not installed"
                return
            port = self.config["serial"]["port"]
            baud = self.config["serial"]["baud"]
            self.thread = threading.Thread(
                target=serial_reader_loop,
                args=(port, baud, self.stop_event),
                daemon=True,
            )
            self.thread.start()

        elif mode == "wifi":
            ip   = self.config["wifi"]["esp_ip"]
            port = self.config["wifi"]["esp_port"]
            self.thread = threading.Thread(
                target=wifi_reader_loop,
                args=(ip, port, self.stop_event),
                daemon=True,
            )
            self.thread.start()

        elif mode == "off":
            with state_lock:
                state["connected"] = False
                state["mode"]      = "off"
                state["port"]      = "—"

    def switch_to(self, new_cfg: dict) -> dict:
        cfg = self.update_config(new_cfg)
        self.start()
        return cfg


reader_mgr = ReaderManager()


# ─────────────────────────────────────────────────────────────────────────────
#  Byte helpers
# ─────────────────────────────────────────────────────────────────────────────
def u16le(d, o): return d[o] | (d[o + 1] << 8)
def u16be(d, o): return (d[o] << 8) | d[o + 1]
def s16le(d, o):
    v = u16le(d, o); return v - 65536 if v >= 32768 else v


# ─────────────────────────────────────────────────────────────────────────────
#  Cell status + SOC helpers (Hana)
# ─────────────────────────────────────────────────────────────────────────────
def cell_status(mv: int) -> str:
    t = state["thresh"]
    if mv >= t["cell_ov"]:     return "ov"
    if mv >= t["cell_full"]:   return "full"
    if mv >= t["cell_bal"]:    return "bal"
    if mv >= t["cell_nom"]:    return "ok"
    if mv >= t["cell_uv_rel"]: return "low"
    if mv >= t["cell_uv"]:     return "uv_warn"
    return "uv"

def soc_from_cells(cell_mv_dict: dict) -> Optional[float]:
    if not cell_mv_dict:
        return None
    avg = sum(cell_mv_dict.values()) / len(cell_mv_dict)
    soc = (avg - CELL_UV_MV) / (SOC_CAR_TOP_MV - CELL_UV_MV) * 100.0
    return round(min(100.0, max(0.0, soc)), 1)


# ─────────────────────────────────────────────────────────────────────────────
#  CAN frame parser  (your originals + Selima's D001-D004 frames)
# ─────────────────────────────────────────────────────────────────────────────
FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:((?:\s+[0-9A-Fa-f]{2})+)'
)

def parse_can(id_hex: int, d: bytes) -> None:
    global frame_count
    frame_count += 1
    state["frame_count"] = frame_count

    grp = (id_hex >> 16) & 0xFF

    # ── Cell voltages — 0x98C8..0xCC28F4
    if 0xC8 <= grp <= 0xCC:
        g = grp - 0xC8
        for i in range(4):
            idx = g * 4 + i + 1
            if idx <= 19 and i * 2 + 1 < len(d):
                mv = u16be(d, i * 2)
                if mv > 0:
                    state["cells"][idx] = {"mv": mv, "status": cell_status(mv)}
        mvs = [v["mv"] for v in state["cells"].values()]
        if mvs:
            state["cell_count"]     = len(mvs)
            state["cell_min_mv"]    = min(mvs)
            state["cell_max_mv"]    = max(mvs)
            state["cell_spread_mv"] = max(mvs) - min(mvs)
            state["cell_avg_mv"]    = round(sum(mvs) / len(mvs))
            if len(mvs) >= 5:
                state["pack_v"] = round(sum(mvs) / 1000.0, 2)
            soc_v = soc_from_cells({k: v["mv"] for k, v in state["cells"].items()})
            if soc_v is not None:
                state["soc"] = soc_v
        return

    # ── BMS internal temperatures — 0x98B428F4
    if id_hex == 0x98B428F4:
        for i in range(min(4, len(d))):
            raw = d[i]
            if raw not in (0x00, 0xFF):
                state["temps"][i + 1] = round(raw - 40.0, 1)
        if state["temps"]:
            state["avg_temp"] = round(
                sum(state["temps"].values()) / len(state["temps"]), 1
            )
        return

    # ── SOC + charge request — 0x98FFE5F4
    if id_hex == 0x98FFE5F4 and len(d) >= 4:
        state["soc_coulomb"] = round(u16le(d, 0) / 10.0, 1)
        state["chg_i_req"]   = round(u16le(d, 2) / 10.0, 1)
        return

    # ── Pack summary — 0x98FF28F4
    if id_hex == 0x98FF28F4 and len(d) >= 6:
        if state["pack_v"] is None:
            state["pack_v"] = round(u16le(d, 0) / 100.0, 2)
        state["disch_i_lim"] = round(u16le(d, 2) / 100.0, 1)
        state["soc_bms"]     = round(u16le(d, 4) / 10.0, 1)

        soc = state["soc"] if state["soc"] is not None else state["soc_bms"]
        if soc is not None:
            state["remaining_ah"] = round(CAP_ACTUAL_AH * soc / 100.0, 1)
        state["soh"] = round(CAP_ACTUAL_AH / CAP_RATED_AH * 100.0, 1)
        return

    # ── Min/Max + temps + disch — 0x98FE28F4
    if id_hex == 0x98FE28F4 and len(d) >= 8:
        state["cell_max_mv"] = u16le(d, 0)
        state["cell_min_mv"] = u16le(d, 2)
        if state["cell_max_mv"] and state["cell_min_mv"]:
            state["cell_spread_mv"] = state["cell_max_mv"] - state["cell_min_mv"]
        for i in range(2):
            raw = d[4 + i]
            if raw not in (0x00, 0xFF):
                state["temps"][i + 1] = round(raw - 40.0, 1)
        return

    # ── DC/DC voltages — 0x98D001F4 (Selima)
    if id_hex == 0x98D001F4 and len(d) >= 4:
        state["ext"]["aux12v"]   = round(u16le(d, 0) / 100.0, 2)
        state["ext"]["hv_iso_v"] = round(u16le(d, 2) / 100.0, 2)
        state["hv64v"]           = state["ext"]["hv_iso_v"]
        return

    # ── Solar / MPPT currents — 0x98D002F4 (Selima)
    if id_hex == 0x98D002F4 and len(d) >= 4:
        state["ext"]["mppt_i1"] = round(s16le(d, 0) / 100.0, 2)
        state["ext"]["mppt_i2"] = round(s16le(d, 2) / 100.0, 2)
        state["solar_i_in"]     = state["ext"]["mppt_i1"]
        state["solar_i_out"]    = state["ext"]["mppt_i2"]
        return

    # ── External temperatures — 0x98D003F4 (Selima)
    if id_hex == 0x98D003F4 and len(d) >= 3:
        state["cabin_t"]       = round(d[0] - 40.0, 1)
        state["motor_t"]       = round(d[1] - 40.0, 1)
        state["ext"]["bat_t1"] = state["cabin_t"]
        state["ext"]["bat_t2"] = state["motor_t"]
        state["ext"]["mppt_t"] = round(d[2] - 40.0, 1)
        if len(d) >= 4 and d[3] > 0:
            state["ext"]["dcdc_t"] = round(d[3] - 40.0, 1)
        return

    # ── Discrete inputs — 0x98D004F4 (Selima)
    if id_hex == 0x98D004F4 and len(d) >= 1:
        state["ext"]["handbrake"] = int(d[0])
        return


# ─────────────────────────────────────────────────────────────────────────────
#  SENSOR text-line parser
# ─────────────────────────────────────────────────────────────────────────────
_NUM  = re.compile(r'(\w+)=([-\d.]+)')
_NAME = re.compile(r'scenario_name=([A-Z_]+)')
_CD   = re.compile(r'scenario_countdown=(\d+)')

def parse_sensor(line: str) -> None:
    kv = {m.group(1): float(m.group(2)) for m in _NUM.finditer(line)}
    def f(k, dec=2): return round(kv[k], dec) if k in kv else None

    if 'solar_v'     in kv: state["solar_v"]         = f("solar_v")
    if 'solar_i_in'  in kv:
        state["solar_i_in"]      = f("solar_i_in")
        state["ext"]["mppt_i1"]  = f("solar_i_in")
    if 'solar_i_out' in kv:
        state["solar_i_out"]     = f("solar_i_out")
        state["ext"]["mppt_i2"]  = f("solar_i_out")
    if 'hv64v'       in kv:
        state["hv64v"]           = f("hv64v")
        state["ext"]["hv_iso_v"] = f("hv64v")
    if 'aux12v'      in kv: state["ext"]["aux12v"]   = f("aux12v")
    if 'motor_t'     in kv:
        state["motor_t"]         = f("motor_t", 1)
        state["ext"]["bat_t2"]   = f("motor_t", 1)
    if 'mppt_t'      in kv: state["ext"]["mppt_t"]   = f("mppt_t", 1)
    if 'dcdc_t'      in kv: state["ext"]["dcdc_t"]   = f("dcdc_t", 1)
    if 'cabin_t'     in kv:
        state["cabin_t"]         = f("cabin_t", 1)
        state["ext"]["bat_t1"]   = f("cabin_t", 1)
    if 'handbrake'   in kv: state["ext"]["handbrake"] = int(kv["handbrake"])

    if 'gnss_lat'    in kv: state["gnss"]["lat"]   = round(kv["gnss_lat"],   5)
    if 'gnss_lon'    in kv: state["gnss"]["lon"]   = round(kv["gnss_lon"],   5)
    if 'gnss_alt'    in kv: state["gnss"]["alt"]   = round(kv["gnss_alt"],   1)
    if 'gnss_speed'  in kv: state["gnss"]["speed"] = round(kv["gnss_speed"], 1)
    if 'gnss_fix'    in kv: state["gnss"]["fix"]   = int(kv["gnss_fix"])

    m = _NAME.search(line)
    if m: state["scenario_name"] = m.group(1).replace('_', ' ')
    m = _CD.search(line)
    if m: state["scenario_countdown_s"] = int(m.group(1))


# ─────────────────────────────────────────────────────────────────────────────
#  Common line dispatcher
# ─────────────────────────────────────────────────────────────────────────────
_start_time = time.time()
def millis(): return int((time.time() - _start_time) * 1000)

def normalize_line(line: str) -> str:
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

def process_line(line: str) -> None:
    """Caller must hold state_lock."""
    log_buffer.append(line)
    norm = normalize_line(line)
    m = FRAME_RE.match(norm)
    if m:
        id_hex = int(m.group(2), 16)
        data   = bytes(int(x, 16) for x in m.group(4).split())
        parse_can(id_hex, data)
    elif line.startswith("SENSOR:"):
        parse_sensor(line)


# ─────────────────────────────────────────────────────────────────────────────
#  Serial reader
# ─────────────────────────────────────────────────────────────────────────────
def auto_detect_serial_port() -> Optional[str]:
    if not SERIAL_AVAILABLE:
        return None
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid        or "").lower()
        if any(k in desc + hwid for k in
               ["cp210","ch340","ftdi","esp32","silicon labs","uart","usb serial"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

def list_serial_ports() -> list:
    if not SERIAL_AVAILABLE:
        return []
    return [
        {"device": p.device, "description": p.description or ""}
        for p in serial.tools.list_ports.comports()
    ]

def serial_reader_loop(port_arg: Optional[str], baud: int, stop_event: threading.Event):
    while not stop_event.is_set():
        target = port_arg or auto_detect_serial_port()
        if not target:
            with state_lock:
                state["connected"] = False
                state["last_error"] = "No serial port found"
            print("[Serial] No port found — retry in 3s")
            if stop_event.wait(3.0): break
            continue

        try:
            print(f"[Serial] Connecting to {target} @ {baud}")
            with serial.Serial(target, baud, timeout=1) as ser:
                with state_lock:
                    state["connected"]  = True
                    state["mode"]       = "serial"
                    state["port"]       = target
                    state["last_error"] = ""
                while not stop_event.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        line = raw.decode("utf-8", errors="replace").strip()
                    except Exception:
                        continue
                    if not line:
                        continue
                    with state_lock:
                        process_line(line)
        except serial.SerialException as e:
            with state_lock:
                state["connected"]  = False
                state["last_error"] = f"Serial: {e}"
            print(f"[Serial] {e}")
            if stop_event.wait(3.0): break
        except Exception as e:
            with state_lock:
                state["connected"]  = False
                state["last_error"] = f"Serial: {e}"
            print(f"[Serial] {e}")
            if stop_event.wait(2.0): break

    with state_lock:
        state["connected"] = False
    print("[Serial] Reader stopped")


# ─────────────────────────────────────────────────────────────────────────────
#  WiFi reader  (server.py is TCP CLIENT, ESP32 AP is server)
# ─────────────────────────────────────────────────────────────────────────────
def wifi_reader_loop(esp_ip: str, esp_port: int, stop_event: threading.Event):
    print(f"[WiFi] Will connect to ESP32 at {esp_ip}:{esp_port}")
    while not stop_event.is_set():
        conn = None
        try:
            conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            conn.settimeout(4.0)
            conn.connect((esp_ip, esp_port))
            conn.settimeout(1.0)   # short timeout so stop_event reacts quickly
            with state_lock:
                state["connected"]  = True
                state["mode"]       = "wifi"
                state["port"]       = f"{esp_ip}:{esp_port}"
                state["last_error"] = ""
            print(f"[WiFi] Connected to {esp_ip}:{esp_port}")

            buf = b""
            while not stop_event.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    with state_lock:
                        process_line(line)

        except (OSError, socket.timeout) as e:
            with state_lock:
                state["connected"]  = False
                state["last_error"] = f"WiFi: {e}"
            print(f"[WiFi] {e} — retry in 2s")
            if stop_event.wait(2.0): break
        except Exception as e:
            with state_lock:
                state["connected"]  = False
                state["last_error"] = f"WiFi: {e}"
            print(f"[WiFi] {e}")
            if stop_event.wait(2.0): break
        finally:
            try:
                if conn: conn.close()
            except Exception:
                pass

    with state_lock:
        state["connected"] = False
    print("[WiFi] Reader stopped")


# ─────────────────────────────────────────────────────────────────────────────
#  FastAPI
# ─────────────────────────────────────────────────────────────────────────────
app = FastAPI(title="BAKO SMU Server")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

class ModeRequest(BaseModel):
    mode:   Optional[str]  = None
    serial: Optional[dict] = None
    wifi:   Optional[dict] = None

@app.get("/")
async def root():
    return FileResponse("../frontend/index_love.html")

@app.get("/api/mode")
async def get_mode():
    cfg = reader_mgr.get_config()
    cfg["available_ports"]  = list_serial_ports()
    cfg["serial_available"] = SERIAL_AVAILABLE
    return JSONResponse(cfg)

@app.post("/api/mode")
async def set_mode(req: ModeRequest):
    new_cfg = {}
    if req.mode is not None:
        if req.mode not in ("serial", "wifi", "off"):
            return JSONResponse({"error": "mode must be serial|wifi|off"},
                                status_code=400)
        new_cfg["mode"] = req.mode
    if req.serial is not None:
        new_cfg["serial"] = req.serial
    if req.wifi is not None:
        new_cfg["wifi"] = req.wifi

    cfg = reader_mgr.switch_to(new_cfg)
    cfg["available_ports"]  = list_serial_ports()
    cfg["serial_available"] = SERIAL_AVAILABLE
    return JSONResponse(cfg)

@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with state_lock:
                payload = copy.deepcopy(state)
                payload["log"] = list(log_buffer)[-80:]
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.1)   # 10 Hz
    except (WebSocketDisconnect, Exception):
        pass


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    import argparse
    p = argparse.ArgumentParser(description="BAKO SMU Combined Server")
    p.add_argument("--host",     default="0.0.0.0")
    p.add_argument("--web-port", default=8787, type=int, dest="web_port")
    args = p.parse_args()

    reader_mgr.start()

    cfg = reader_mgr.get_config()
    print("=" * 64)
    print("  BAKO SMU Combined Server")
    print(f"  Active mode : {cfg['mode'].upper()}")
    if cfg["mode"] == "serial":
        print(f"  Serial port : {cfg['serial']['port'] or 'auto-detect'}"
              f"  @ {cfg['serial']['baud']} baud")
    elif cfg["mode"] == "wifi":
        print(f"  ESP32 WiFi  : {cfg['wifi']['esp_ip']}:{cfg['wifi']['esp_port']}")
        print(f"  (Connect your PC to the 'BAKO_SMU' WiFi network first)")
    print(f"  Dashboard   : http://localhost:{args.web_port}")
    print(f"  Mode switch : use the dropdown in the dashboard header")
    print("  Ctrl+C to stop")
    print("=" * 64)

    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
