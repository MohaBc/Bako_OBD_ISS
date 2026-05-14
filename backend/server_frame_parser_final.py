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
  0x18D601AA  DC/DC heatsink temperature
  0x18D701AA  Handbrake position

REST API:
  GET  /          → dashboard (index.html)
  GET  /api/mode  → current mode + config
  POST /api/mode  → switch mode
  WS   /ws        → 10 Hz JSON snapshot

Requirements: pip install fastapi uvicorn pyserial

ESP32 also serves a WebSocket on port 81 (direct browser connection);
This server connects as TCP client to ESP32 port 9000 in wifi mode.
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
    "wifi":   {"listen_host": "0.0.0.0", "listen_port": 9000},
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
    "pack_current":    None,   # A, positive = discharge
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

    # Extended sensors — mapped to dashboard keys
    "ext": {
        "aux12v":    None,   # DC/DC 12V output voltage
        "hv_iso_v":  None,   # solar panel voltage (mapped to HV reference)
        "mppt_i1":   None,   # solar panel current (before MPPT)
        "mppt_i2":   None,   # MPPT output current (after MPPT, before battery)
        "dcdc_t":    None,   # DC/DC heatsink temperature
        "bat_t2":    None,   # motor winding temperature
        "mppt_t":    None,   # MPPT heatsink temperature
        "dcdc_t":    None,   # DC/DC heatsink temperature
        "handbrake": None,
    },
    "ext_thresh": {"mppt_max_a": 22.0},

    # New sensor detail fields (full resolution)
    "solar_v":          None,   # solar panel voltage V
    "solar_voc":        None,   # open-circuit voltage V
    "solar_i_raw":      None,   # solar current A
    "solar_i_avg":      None,   # 5-cycle moving average A
    "mppt_out_i":       None,   # MPPT output current A
    "mppt_mode":        None,   # 0=off 1=tracking 2=CV 3=float
    "mppt_efficiency":  None,   # % 0-100
    "dcdc_v":           None,   # DC/DC output voltage V
    "dcdc_i":           None,   # DC/DC output current A (None = not fitted)
    "dcdc_status":      None,
    "motor_t_winding":  None,   # motor winding temp °C
    "motor_t_housing":  None,   # motor housing temp °C (None = not fitted)
    "motor_t_status":   None,
    "mppt_t_raw":       None,   # MPPT heatsink temp °C
    "mppt_t_status":    None,
    "dcdc_t":           None,   # DC/DC heatsink temp °C  (DS18B20 #2)
    "dcdc_t_status":    None,
    "handbrake_raw":    None,   # 0=released 1=engaged 0xFF=fault
    "hb_debounce":      None,   # 0=stable 1=transitioning

    # GNSS (for future use)
    "gnss": {"lat": None, "lon": None, "alt": None, "speed": None, "fix": 0},

    # Scenario info (from simulator firmware)
    "scenario_name":        "—",
    "scenario_countdown_s": None,

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

# MPPT mode labels
MPPT_MODE_LABELS = {0: "off", 1: "tracking", 2: "CV_charge", 3: "float"}

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
        elif mode == "wifi":
            host = self.config["wifi"].get("listen_host", "0.0.0.0")
            port = self.config["wifi"].get("listen_port", 9000)
            self.thread = threading.Thread(
                target=wifi_reader_loop,
                args=(host, port, self.stop_event),
                daemon=True,
            )
            self.thread.start()
        elif mode == "ws":
            # ws mode: listen for ESP32 WebSocket connection
            # (currently handled via the ws_reader_loop)
            ip      = self.config["wifi"].get("listen_host", "0.0.0.0")
            ws_port = self.config["wifi"].get("listen_port", 81)
            self.thread = threading.Thread(
                target=ws_reader_loop,
                args=(ip, ws_port, self.stop_event),
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
    # d[0]: status bitfield
    # d[1]: SOC % raw
    # d[2-3]: pack current LE uint16, (raw-5000)×0.1 A  (+ = discharge)
    # d[4-5]: pack voltage LE uint16, ×0.1 V
    # d[6]: fault level
    # d[7]: error code
    if id_hex == 0x18FF28F4 and len(d) >= 8:
        soc_raw = d[1]
        if 0 <= soc_raw <= 100:
            state["soc_bms"] = float(soc_raw)
            if state["soc"] is None:
                state["soc"] = float(soc_raw)

        current_raw = u16le(d, 2)
        pack_current = round((current_raw - 5000) * 0.1, 1)   # A, + = discharge
        state["pack_current"] = pack_current

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

    # ─────────────────────────────────────────────────────────────────────────
    #  ESP32 Sensor Gateway frames (SA=0xAA)
    # ─────────────────────────────────────────────────────────────────────────

    # ── 0x18D001AA — Solar panel current (before MPPT) ───────────────────────
    # Bytes 1-2: LE uint16 raw current,  0.1 A/bit
    # Bytes 3-4: LE uint16 filtered avg, 0.1 A/bit
    # Byte  5  : sensor status
    if id_hex == 0x18D001AA and len(d) >= 5:
        raw_i = round(u16le(d, 0) * 0.1, 2)
        avg_i = round(u16le(d, 2) * 0.1, 2)
        state["solar_i_raw"]     = raw_i
        state["solar_i_avg"]     = avg_i
        state["ext"]["mppt_i1"]  = raw_i   # maps to dashboard solar-in current
        return

    # ── 0x18D101AA — Solar panel voltage (before MPPT) ───────────────────────
    # Bytes 1-2: LE uint16 panel voltage,    0.1 V/bit
    # Bytes 3-4: LE uint16 open-circuit Voc, 0.1 V/bit
    # Byte  5  : sensor status
    if id_hex == 0x18D101AA and len(d) >= 5:
        v_panel = round(u16le(d, 0) * 0.1, 2)
        v_oc    = round(u16le(d, 2) * 0.1, 2)
        state["solar_v"]          = v_panel
        state["solar_voc"]        = v_oc
        state["ext"]["hv_iso_v"]  = v_panel  # maps to HV voltage reference
        return

    # ── 0x18D201AA — MPPT output current + mode + efficiency ─────────────────
    # Bytes 1-2: LE uint16 output current, 0.1 A/bit
    # Byte  3  : MPPT mode (0=off 1=track 2=CV 3=float)
    # Byte  4  : efficiency % (0-100)
    # Byte  5  : sensor status
    if id_hex == 0x18D201AA and len(d) >= 5:
        out_i    = round(u16le(d, 0) * 0.1, 2)
        mode_raw = d[2]
        eff      = d[3]
        state["mppt_out_i"]       = out_i
        state["mppt_mode"]        = MPPT_MODE_LABELS.get(mode_raw, str(mode_raw))
        state["mppt_efficiency"]  = eff
        state["ext"]["mppt_i2"]   = out_i  # maps to MPPT output current slot
        return

    # ── 0x18D301AA — DC/DC 12V output voltage ────────────────────────────────
    # Bytes 1-2: LE uint16 output voltage, 0.01 V/bit
    # Bytes 3-4: LE uint16 output current, 0.1 A/bit (0xFFFF = not fitted)
    # Byte  5  : DC/DC status
    if id_hex == 0x18D301AA and len(d) >= 5:
        v_raw = u16le(d, 0)
        i_raw = u16le(d, 2)
        v = round(v_raw * 0.01, 3)
        i = None if i_raw == 0xFFFF else round(i_raw * 0.1, 2)
        state["dcdc_v"]          = v
        state["dcdc_i"]          = i
        state["dcdc_status"]     = d[4]
        state["ext"]["aux12v"]   = round(v, 2)  # maps to 12V aux voltage
        return

    # ── 0x18D401AA — Motor temperature ───────────────────────────────────────
    # Byte 1: motor winding temp  (uint8, offset -40, 0xFF=NC)
    # Byte 2: motor housing temp  (uint8, offset -40, 0xFF=NC)
    # Byte 3: sensor status
    if id_hex == 0x18D401AA and len(d) >= 3:
        t_winding = decode_temp(d[0])
        t_housing = decode_temp(d[1])
        status    = d[2]
        state["motor_t_winding"]  = t_winding
        state["motor_t_housing"]  = t_housing
        state["motor_t_status"]   = status
        state["ext"]["bat_t2"]    = t_winding  # maps to motor temp display slot
        return

    # ── 0x18D501AA — MPPT heatsink temperature ───────────────────────────────
    # Byte 1: MPPT heatsink temp (uint8, offset -40)
    # Byte 2: sensor status
    if id_hex == 0x18D501AA and len(d) >= 2:
        t = decode_temp(d[0])
        state["mppt_t_raw"]     = t
        state["mppt_t_status"]  = d[1]
        state["ext"]["mppt_t"]  = t   # maps to MPPT heatsink display slot
        return

    # ── 0x18D601AA — DC/DC heatsink temperature  ────────────────────────────
    # Byte 0: DC/DC heatsink temp  (uint8, offset -40, 0xFF=NC)
    # Byte 1: sensor status  (0x00 normal / 0x01 warning / 0x02 fault)
    if id_hex == 0x18D601AA and len(d) >= 2:
        t_dcdc_hs = decode_temp(d[0])
        status    = d[1]
        state["dcdc_t"]          = t_dcdc_hs
        state["dcdc_t_status"]   = status
        state["ext"]["dcdc_t"]   = t_dcdc_hs
        return

    # ── 0x18D701AA — Handbrake position ──────────────────────────────────────
    # Byte 1: 0x00=released 0x01=engaged 0xFF=fault
    # Byte 2: debounce state (0=stable 1=transitioning)
    if id_hex == 0x18D701AA and len(d) >= 2:
        hb_raw = d[0]
        hb     = None if hb_raw == 0xFF else int(hb_raw)
        state["handbrake_raw"]     = hb
        state["hb_debounce"]       = int(d[1])
        state["ext"]["handbrake"]  = hb
        return


# ─────────────────────────────────────────────────────────────────────────────
#  SENSOR text-line parser (fallback / supplementary)
# ─────────────────────────────────────────────────────────────────────────────
_NUM  = re.compile(r'(\w+)=([-\d.]+)')
_NAME = re.compile(r'scenario_name=([A-Z_]+)')
_CD   = re.compile(r'scenario_countdown=(\d+)')

def parse_sensor(line: str) -> None:
    kv = {m.group(1): float(m.group(2)) for m in _NUM.finditer(line)}
    def f(k, dec=2): return round(kv[k], dec) if k in kv else None

    if 'solar_v'     in kv:
        state["solar_v"]          = f("solar_v")
        state["ext"]["hv_iso_v"]  = f("solar_v")
    if 'solar_i_in'  in kv:
        state["solar_i_raw"]      = f("solar_i_in")
        state["ext"]["mppt_i1"]   = f("solar_i_in")
    if 'solar_i_out' in kv:
        state["mppt_out_i"]       = f("solar_i_out")
        state["ext"]["mppt_i2"]   = f("solar_i_out")
    if 'aux12v'      in kv:
        state["ext"]["aux12v"]    = f("aux12v")
        state["dcdc_v"]           = f("aux12v")
    if 'motor_t'     in kv:
        state["motor_t_winding"]  = f("motor_t", 1)
        state["ext"]["bat_t2"]    = f("motor_t", 1)
    if 'mppt_t'      in kv:
        state["mppt_t_raw"]       = f("mppt_t", 1)
        state["ext"]["mppt_t"]    = f("mppt_t", 1)
    if 'dcdc_t'      in kv:
        state["dcdc_t"]           = f("dcdc_t", 1)
        state["ext"]["dcdc_t"]    = f("dcdc_t", 1)
    if 'handbrake'   in kv:
        state["handbrake_raw"]    = int(kv["handbrake"])
        state["ext"]["handbrake"] = int(kv["handbrake"])
    if 'hv64v'       in kv:
        state["ext"]["hv_iso_v"]  = f("hv64v")

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
    elif line.startswith("[SIM]"):
        # Capture scenario display name from simulator log header
        # [SIM]  SCENARIO: 🏙  CITY DRIVE
        m2 = re.search(r'SCENARIO:\s+(.+)', line)
        if m2:
            state["scenario_name"] = m2.group(1).strip()


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
#  WiFi reader
# ─────────────────────────────────────────────────────────────────────────────
def wifi_reader_loop(listen_host: str, listen_port: int, stop_event: threading.Event):
    """TCP SERVER — listens for the ESP32 to connect and stream CAN frame lines."""
    print(f"[WiFi] TCP server listening on {listen_host}:{listen_port}")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind((listen_host, listen_port))
        srv.listen(1)
        srv.settimeout(1.0)
    except OSError as e:
        with state_lock:
            state["last_error"] = f"WiFi bind: {e}"
        print(f"[WiFi] Bind failed: {e}")
        return

    with state_lock:
        state["mode"] = "wifi"
        state["port"] = f"{listen_host}:{listen_port}"

    while not stop_event.is_set():
        conn = None
        try:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print(f"[WiFi] ESP32 connected from {addr[0]}:{addr[1]}")
            conn.settimeout(2.0)
            with state_lock:
                state["connected"]  = True
                state["last_error"] = ""
                state["port"]       = f"esp32@{addr[0]}:{listen_port}"
            buf = b""
            while not stop_event.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    print("[WiFi] ESP32 disconnected")
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
            with state_lock:
                state["last_error"] = f"WiFi: {e}"
            print(f"[WiFi] {e}")
        finally:
            if conn:
                try: conn.close()
                except Exception: pass
            with state_lock:
                state["connected"] = False

    try: srv.close()
    except Exception: pass
    with state_lock:
        state["connected"] = False
    print("[WiFi] TCP server stopped")


# ─────────────────────────────────────────────────────────────────────────────
#  WebSocket reader (connects to ESP32 WS server on port 81)
# ─────────────────────────────────────────────────────────────────────────────
def ws_reader_loop(listen_host: str, listen_port: int, stop_event: threading.Event):
    """Connect to ESP32 WebSocket server (ws://IP:81) and read frame lines."""
    try:
        import websockets
        import asyncio as _asyncio

        async def _run():
            uri = f"ws://localhost:{listen_port}"
            print(f"[WS-Reader] Connecting to {uri}")
            async with websockets.connect(uri, ping_interval=5) as ws:
                with state_lock:
                    state["connected"]  = True
                    state["mode"]       = "ws"
                    state["port"]       = uri
                    state["last_error"] = ""
                while not stop_event.is_set():
                    try:
                        msg = await _asyncio.wait_for(ws.recv(), timeout=2.0)
                        line = msg.strip()
                        if line:
                            with state_lock:
                                process_line(line)
                    except _asyncio.TimeoutError:
                        continue
                    except Exception as e:
                        with state_lock:
                            state["connected"]  = False
                            state["last_error"] = f"WS-Reader: {e}"
                        raise

        while not stop_event.is_set():
            try:
                loop = _asyncio.new_event_loop()
                loop.run_until_complete(_run())
            except Exception as e:
                with state_lock:
                    state["connected"]  = False
                    state["last_error"] = f"WS-Reader: {e}"
                print(f"[WS-Reader] {e} — retry in 2s")
                if stop_event.wait(2.0):
                    break
    except ImportError:
        with state_lock:
            state["last_error"] = "websockets not installed: pip install websockets"
        print("[WS-Reader] pip install websockets required for ws mode")
    with state_lock:
        state["connected"] = False
    print("[WS-Reader] stopped")



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
    return FileResponse("../frontend/index.html")

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
        if req.mode not in ("serial", "wifi", "ws", "off"):
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
    elif cfg["mode"] == "wifi":
        host = cfg["wifi"].get("listen_host", "0.0.0.0")
        port = cfg["wifi"].get("listen_port", 9000)
        print(f"  TCP server  : {host}:{port}  (waiting for ESP32 to connect)")
        print(f"  Set SERVER_IP={host if host != '0.0.0.0' else '<your LAN IP>'} in the ESP32 firmware")
    print(f"  Dashboard   : http://localhost:{args.web_port}")
    print("  Ctrl+C to stop")
    print("=" * 64)

    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
