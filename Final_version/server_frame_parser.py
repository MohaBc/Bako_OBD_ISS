#!/usr/bin/env python3
"""
BAKO SMU — Combined Backend Server (Serial + WiFi AP, runtime-switchable)
==========================================================================
CAN frame parsing updated to match BAKO CAN Protocol Rev 1.0:

BMS frames (SA=0xF4):
  0x18FF28F4  BMS Basic Msg 1 — status flags + SOC + pack current + pack voltage + fault
  0x18FE28F4  BMS Basic Msg 2 — max/min cell V + temps + max disch current
  0x18C8..CC28F4  Cell voltages (big-endian uint16 pairs)
  0x18B428F4  Temperature probes (up to 8, uint8 offset-40, 0xFF=NC)
  0x18FFE5F4  BMS charging request

New ESP32 sensor frames (SA=0xAA):
  0x18D001AA  Solar panel current before MPPT
  0x18D101AA  Solar panel voltage before MPPT
  0x18D201AA  MPPT output current + mode + efficiency
  0x18D301AA  DC/DC 12V output voltage
  0x18D401AA  Motor temperature (winding + housing)
  0x18D501AA  MPPT heatsink temperature
  0x18D601AA  Cabin interior temperature + humidity
  0x18D701AA  Handbrake position

REST API:
  GET  /          → dashboard (index.html)
  GET  /api/mode  → current mode + config
  POST /api/mode  → switch mode
  WS   /ws        → 10 Hz JSON snapshot

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
    "mode": "serial",
    "serial": {"port": None, "baud": 115200},
    "local":  {"listen_host": "0.0.0.0", "listen_port": 9000},
    "cloud":  {"ws_url": "ws://62.169.24.172:8787/ws"},
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
#  Shared state
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
    "fault_level":    0,
    "error_code":     0,

    # Cells  {1: {"mv": 3300, "status": "ok"}, …}
    "cells":          {},
    "cell_count":     0,
    "cell_min_mv":    None,
    "cell_max_mv":    None,
    "cell_spread_mv": None,
    "cell_avg_mv":    None,

    # BMS internal temps  {1: 24.5, …}
    "temps":          {},
    "avg_temp":       None,

    # ESP32 sensor readings (from SENSOR_JSON lines)
    "ext": {
        "aux12v":    None,   # 12V DC out
        "hv_iso_v":  None,   # 72V DC in
        "mppt_i1":   None,   # current in  (before MPPT)
        "mppt_i2":   None,   # current out (after MPPT)
        "bat_t1":    None,   # MPPT heatsink temp
        "bat_t2":    None,   # DC/DC heatsink temp
        "mppt_t":    None,   # motor temp
        "dcdc_t":    None,   # 72V MPPT in
        "handbrake": None,
    },
    "ext_thresh": {"mppt_max_a": 22.0},

    # Raw sensor values
    "v12_dc_out":    None,
    "v12_handbrake": None,
    "v72_dc_in":     None,
    "v72_mppt_in":   None,
    "current_in":    None,
    "current_out":   None,
    "temp_mppt":     None,
    "temp_dcdc":     None,
    "temp_motor":    None,
    "handbrake_raw": None,

    # Thresholds for UI colour coding
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

# SOC calibration constants (BAKO doc section 5.2)
SOC_CAR_TOP_MV = 3387   # 100% SOC cell voltage
CELL_UV_MV     = 2500   # 0% SOC cell voltage
CAP_RATED_AH   = 50.0
CAP_ACTUAL_AH  = 44.7

log_buffer  = deque(maxlen=300)
frame_count = 0
state_lock  = threading.Lock()


# ─────────────────────────────────────────────────────────────────────────────
#  Reader manager
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
        elif mode == "local":
            host = self.config["local"]["listen_host"]
            port = self.config["local"]["listen_port"]
            self.thread = threading.Thread(
                target=local_tcp_server_loop,
                args=(host, port, self.stop_event),
                daemon=True,
            )
            self.thread.start()
        elif mode in ("off", "cloud"):
            with state_lock:
                state["connected"] = False
                state["mode"]      = mode
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
#  Cell status + SOC helpers
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

def decode_temp(raw: int) -> Optional[float]:
    """Decode BMS-convention temperature byte (offset -40). 0xFF = not connected."""
    if raw == 0xFF:
        return None
    return round(raw - 40.0, 1)


# ─────────────────────────────────────────────────────────────────────────────
#  CAN frame parser  — all frame IDs per BAKO Protocol Rev 1.0
# ─────────────────────────────────────────────────────────────────────────────
FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:((?:\s+[0-9A-Fa-f]{2})+)'
)

def parse_can(id_hex: int, d: bytes) -> None:
    global frame_count
    frame_count += 1
    state["frame_count"] = frame_count

    # Extract PF (PDU Format byte = bits 23:16 of 29-bit ID)
    pf = (id_hex >> 16) & 0xFF

    # ── BMS: Cell voltages — 0x18C8..CC28F4 ─────────────────────────────────
    # PF = 0xC8..0xCC, PS=0x28, SA=0xF4
    # Big-endian uint16 pairs per BAKO doc section 3.3
    if 0xC8 <= pf <= 0xCC and (id_hex & 0xFF) == 0xF4:
        g = pf - 0xC8
        for i in range(4):
            idx = g * 4 + i + 1
            byte_off = i * 2
            if idx <= 19 and byte_off + 1 < len(d):
                mv = u16be(d, byte_off)
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

    # ── BMS: Temperature probes — 0x18B428F4 ────────────────────────────────
    # PF=0xB4, PS=0x28, SA=0xF4
    # Up to 8 bytes, each uint8 offset-40 °C, 0xFF = not connected
    if id_hex == 0x18B428F4:
        for i in range(min(8, len(d))):
            t = decode_temp(d[i])
            if t is not None:
                state["temps"][i + 1] = t
        if state["temps"]:
            state["avg_temp"] = round(
                sum(state["temps"].values()) / len(state["temps"]), 1
            )
        return

    # ── BMS: Basic Message 1 — 0x18FF28F4 ───────────────────────────────────
    # Byte 1: status bitfield
    # Byte 2: SOC % (low byte, scale 1)
    # Bytes 3-4: pack current LE uint16, offset -5000, scale 0.1 A/bit
    # Bytes 5-6: pack voltage LE uint16, scale 0.1 V/bit
    # Byte 7: fault level
    # Byte 8: error code
    if id_hex == 0x18FF28F4 and len(d) >= 8:
        soc_raw = d[1]
        if 0 <= soc_raw <= 100:
            state["soc_bms"] = float(soc_raw)
            if state["soc"] is None:
                state["soc"] = float(soc_raw)

        current_raw = u16le(d, 2)
        pack_current = round((current_raw - 5000) * 0.1, 1)   # A, + = discharge

        voltage_raw = u16le(d, 4)
        pack_voltage = round(voltage_raw * 0.1, 1)             # V

        if pack_voltage > 0 and state["pack_v"] is None:
            state["pack_v"] = pack_voltage

        state["fault_level"] = d[6]
        state["error_code"]  = d[7]

        soc = state["soc"] if state["soc"] is not None else state["soc_bms"]
        if soc is not None:
            state["remaining_ah"] = round(CAP_ACTUAL_AH * soc / 100.0, 1)
        state["soh"] = round(CAP_ACTUAL_AH / CAP_RATED_AH * 100.0, 1)
        return

    # ── BMS: Basic Message 2 — 0x18FE28F4 ───────────────────────────────────
    # Bytes 1-2: max cell voltage LE uint16, 1 mV/bit
    # Bytes 3-4: min cell voltage LE uint16, 1 mV/bit
    # Byte  5  : max cell temp uint8, offset -40 °C
    # Byte  6  : min cell temp uint8, offset -40 °C
    # Bytes 7-8: max discharge current LE uint16, 0.1 A/bit
    if id_hex == 0x18FE28F4 and len(d) >= 8:
        max_mv = u16le(d, 0)
        min_mv = u16le(d, 2)
        if max_mv > 0:
            state["cell_max_mv"] = max_mv
        if min_mv > 0:
            state["cell_min_mv"] = min_mv
        if max_mv and min_mv:
            state["cell_spread_mv"] = max_mv - min_mv

        t_max = decode_temp(d[4])
        t_min = decode_temp(d[5])
        if t_max is not None:
            state["temps"][1] = t_max
        if t_min is not None:
            state["temps"][2] = t_min
        if state["temps"]:
            state["avg_temp"] = round(
                sum(state["temps"].values()) / len(state["temps"]), 1
            )

        disch_lim_raw = u16le(d, 6)
        state["disch_i_lim"] = round(disch_lim_raw * 0.1, 1)
        return

    # ── BMS: Charging request — 0x18FFE5F4 ──────────────────────────────────
    # Bytes 1-2: max charge voltage LE uint16, 0.1 V/bit
    # Bytes 3-4: max charge current LE uint16, 0.1 A/bit
    # Byte  5  : control byte
    if id_hex == 0x18FFE5F4 and len(d) >= 4:
        chg_v_raw = u16le(d, 0)
        chg_i_raw = u16le(d, 2)
        # Bytes 1-2 in this frame encode SOC coulomb in some BMS variants
        # Per BAKO doc section 5.1: use as charge voltage limit
        state["chg_i_req"] = round(chg_i_raw * 0.1, 1)
        return



# ─────────────────────────────────────────────────────────────────────────────
#  ESP32 JSON sensor parser
# ─────────────────────────────────────────────────────────────────────────────
def parse_sensor_json(line: str) -> None:
    """Parse SENSOR_JSON:{...} lines emitted by the ESP32 firmware."""
    try:
        data = json.loads(line[len("SENSOR_JSON:"):])
    except Exception:
        return

    def get(k):
        return data.get(k)

    state["v12_dc_out"]    = get("v12_dc_out")
    state["v12_handbrake"] = get("v12_handbrake")
    state["v72_dc_in"]     = get("v72_dc_in")
    state["v72_mppt_in"]   = get("v72_mppt_in")
    state["current_in"]    = get("current_in")
    state["current_out"]   = get("current_out")
    state["temp_mppt"]     = get("temp_mppt")
    state["temp_dcdc"]     = get("temp_dcdc")
    state["temp_motor"]    = get("temp_motor")
    state["handbrake_raw"] = get("handbrake")

    # Map into the ext dict that the dashboard reads
    state["ext"]["aux12v"]    = get("v12_dc_out")    # 12V DC out
    state["ext"]["hv_iso_v"]  = get("v72_dc_in")     # 72V DC in
    state["ext"]["mppt_i1"]   = get("current_in")    # current in
    state["ext"]["mppt_i2"]   = get("current_out")   # current out
    state["ext"]["bat_t1"]    = get("temp_mppt")     # MPPT heatsink temp
    state["ext"]["bat_t2"]    = get("temp_dcdc")     # DC/DC heatsink temp
    state["ext"]["mppt_t"]    = get("temp_motor")    # motor temp
    state["ext"]["dcdc_t"]    = get("v72_mppt_in")   # 72V MPPT in
    state["ext"]["handbrake"] = get("handbrake")



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
    elif line.startswith("SENSOR_JSON:"):
        parse_sensor_json(line)


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
        except Exception as e:
            with state_lock:
                state["connected"]  = False
                state["last_error"] = f"Serial: {e}"
            print(f"[Serial] {e}")
            if stop_event.wait(3.0): break
    with state_lock:
        state["connected"] = False
    print("[Serial] Reader stopped")


# ─────────────────────────────────────────────────────────────────────────────
#  Local TCP server — listens for ESP32 to connect over LAN
# ─────────────────────────────────────────────────────────────────────────────
def local_tcp_server_loop(host: str, port: int, stop_event: threading.Event):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((host, port))
        srv.listen(2)
    except OSError as e:
        with state_lock:
            state["last_error"] = f"Local TCP bind failed: {e}"
        print(f"[Local] Bind error: {e}")
        return
    srv.settimeout(1.0)
    with state_lock:
        state["connected"]  = False
        state["mode"]       = "local"
        state["port"]       = f"listening :{port}"
        state["last_error"] = ""
    print(f"[Local] Listening for ESP32 on {host}:{port}")
    while not stop_event.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Local] Accept error: {e}")
            break
        print(f"[Local] ESP32 connected from {addr[0]}:{addr[1]}")
        with state_lock:
            state["connected"]  = True
            state["port"]       = f"esp32@{addr[0]}"
            state["last_error"] = ""
        conn.settimeout(1.0)
        buf = b""
        try:
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
        except Exception as e:
            print(f"[Local] Connection error: {e}")
        finally:
            conn.close()
        with state_lock:
            state["connected"] = False
            state["port"]      = f"listening :{port}"
        print("[Local] ESP32 disconnected — waiting for new connection")
    srv.close()
    with state_lock:
        state["connected"] = False
    print("[Local] TCP server stopped")


# ─────────────────────────────────────────────────────────────────────────────
#  FastAPI
# ─────────────────────────────────────────────────────────────────────────────
app = FastAPI(title="BAKO SMU Server")
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

class ModeRequest(BaseModel):
    mode:   Optional[str]  = None
    serial: Optional[dict] = None
    wifi:   Optional[dict] = None   # kept for backward compat, maps to local
    local:  Optional[dict] = None
    cloud:  Optional[dict] = None

@app.get("/")
async def root():
    return FileResponse("../Final_version/index.html")

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
        if req.mode not in ("serial", "local", "cloud", "off"):
            return JSONResponse({"error": "mode must be serial|local|cloud|off"},
                                status_code=400)
        new_cfg["mode"] = req.mode
    if req.serial is not None:
        new_cfg["serial"] = req.serial
    if req.local is not None:
        new_cfg["local"] = req.local
    elif req.wifi is not None:
        new_cfg["local"] = req.wifi   # backward compat
    if req.cloud is not None:
        new_cfg["cloud"] = req.cloud
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
    p.add_argument("--web-port", default=8765, type=int, dest="web_port")
    args = p.parse_args()

    reader_mgr.start()

    cfg = reader_mgr.get_config()
    print("=" * 64)
    print("  BAKO SMU Combined Server")
    print(f"  Active mode : {cfg['mode'].upper()}")
    if cfg["mode"] == "serial":
        print(f"  Serial port : {cfg['serial']['port'] or 'auto-detect'}"
              f"  @ {cfg['serial']['baud']} baud")
    elif cfg["mode"] == "local":
        port = cfg.get("local", {}).get("listen_port", 9000)
        print(f"  Local TCP   : listening on :{port} (ESP32 connects to this machine)")
    elif cfg["mode"] == "cloud":
        print(f"  Cloud mode  : dashboard connects to VPS directly")
    print(f"  Dashboard   : http://localhost:{args.web_port}")
    print("  Ctrl+C to stop")
    print("=" * 64)
    print(f"  >> Open http://localhost:{args.web_port} in your browser <<")
    print("  (terminal will appear frozen — that is normal, server is running)")
    print()

    uvicorn.run(app, host=args.host, port=args.web_port, log_level="info")


if __name__ == "__main__":
    main()
