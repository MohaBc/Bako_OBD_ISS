#!/usr/bin/env python3
"""
BAKO Motors — CAN Dashboard Backend Server
===========================================
Reads ESP32 + MCP2515 serial output, decodes BAKO 60V/100Ah LFP
19S1P CAN frames, streams live JSON to the browser via WebSocket.

Battery : BAKO 60V / 100Ah  LiFePO4  19S1P
CAN baud: 250 kbps  (ISO 11898 / SAE J1939 29-bit extended IDs)

Requirements:  pip install fastapi uvicorn pyserial

Usage:
    python server.py                      # auto-detect USB port
    python server.py --port COM3
    python server.py --port /dev/ttyUSB0
    python server.py --baud 115200        # default
    python server.py --web-port 8765      # default

════════════════════════════════════════════════════════════════════
FRAME MAP  (verified vs BAKO CAN Protocol v1.0 + real car logs)
════════════════════════════════════════════════════════════════════

J1939 29-bit ID layout:
  Bits 28-26  Priority (3)
  Bit  25     Reserved = 0
  Bit  24     DP = 0
  Bits 23-16  PF — PDU Format (8)  ← used to identify frame type
  Bits 15-8   PS — destination/group (8)  ← SA of receiver node
  Bits 7-0    SA — source address (8)     ← 0xF4 = BMS, 0xAA = ESP32

──────────────────────────────────────────────────────────────────
BMS FRAMES  (SA = 0xF4)
──────────────────────────────────────────────────────────────────

0x18FF28F4  BMS basic message 1  (100 ms)
  byte 0    : status bit-field
                bit 0 = charging cable connected  (0=no, 1=yes)
                bit 1 = battery pack charging status  (0=no, 1=yes)
                bit 2 = battery pack discharge status (0=no, 1=loss)
                bit 3 = battery pack ready  (0=no, 1=ready)
                bit 4 = discharge contactor (0=open, 1=closed)
                bit 5 = charge contactor    (0=open, 1=closed)
                bits 6-7 = reserved
  byte 1    : SOC %  (integer 0-100, scale 1)
  bytes 2-3 : pack current  LE uint16, offset -5000, scale 0.1 A/bit
                raw=5000 → 0.0 A  raw<5000 → charging  raw>5000 → discharging
  bytes 4-5 : pack voltage  LE uint16, offset 0, scale 0.1 V/bit
  byte 6    : battery failure level  (0x00=normal, 0x01=Level-1 fault)
  byte 7    : error code  (0x00=normal, 0x01-0x14=fault codes, see docs)

0x18FE28F4  BMS basic message 2  (100 ms)
  bytes 0-1 : max cell voltage  LE uint16, mV (1 mV/bit)
  bytes 2-3 : min cell voltage  LE uint16, mV (1 mV/bit)
  byte 4    : max cell temperature  uint8, raw-40 = °C
  byte 5    : min cell temperature  uint8, raw-40 = °C
  bytes 6-7 : max allowable discharge current  LE uint16, 0.1 A/bit

0x18C8-CC28F4  Cell voltages  (500 ms)  PF = 0xC8 … 0xCC
  4 × big-endian uint16 pairs per frame, unit mV (1 mV/bit)
  0xC8 → cells  1- 4   0xC9 → cells  5- 8   0xCA → cells  9-12
  0xCB → cells 13-16   0xCC → cells 17-19   (last pair = 0x0000 pad)

0x18B428F4  Temperature probes  (500 ms)
  bytes 0-7 : up to 8 probes, uint8, raw-40 = °C, 0xFF = not connected

0x18FFE5F4  BMS charging request to charger  (1000 ms, SA=0xF4 → PS=0xE5)
  bytes 0-1 : max allowable charge voltage  LE uint16, 0.1 V/bit
  bytes 2-3 : max allowable charge current  LE uint16, 0.1 A/bit
  byte 4    : control byte  (bit 0 = charger start, 0=start charging)
  bytes 5-7 : warranty/protection window flags  (0x00 = no active protection)

──────────────────────────────────────────────────────────────────
CHARGER FRAMES  (SA = 0x29, sent by on-board charger)
──────────────────────────────────────────────────────────────────

0x18FF50E5  Charger feedback to BMS  (1000 ms)
  bytes 0-1 : charger output voltage  LE uint16, 0.1 V/bit
  bytes 2-3 : charger output current  LE uint16, 0.1 A/bit
  byte 4    : status flags
                bit 0 = hardware malfunction  (0=ok, 1=fault)
                bit 1 = charger over-temperature  (0=ok, 1=fault)
                bit 2 = low-voltage power-limit mode  (0=normal, 1=low-V)
                bit 3 = input voltage status  (0=normal, 1=over/under-V)
                bit 4 = output over-current  (0=ok, 1=overcurrent)
                bit 5 = automatic state  (0=off, 1=charging)
                bit 6 = communication state  (0=ok, 1=timeout)
                bit 7 = battery connection  (0=ok, 1=reversed/disconnected)
  bytes 5-7 : reserved (0x00)

──────────────────────────────────────────────────────────────────
NEW ESP32 SENSOR FRAMES  (SA = 0xAA)
──────────────────────────────────────────────────────────────────

0x18D001AA  Solar current before MPPT  (200 ms)
  bytes 0-1 : raw current    LE uint16, 0.1 A/bit
  bytes 2-3 : filtered avg   LE uint16, 0.1 A/bit
  byte 4    : status  (0=normal, 1=out-of-range, 2=fault)

0x18D101AA  Solar voltage before MPPT  (200 ms)
  bytes 0-1 : panel voltage  LE uint16, 0.1 V/bit
  bytes 2-3 : open-circuit V LE uint16, 0.1 V/bit
  byte 4    : status  (0=normal, 1=over-voltage, 2=fault)

0x18D201AA  MPPT output current after MPPT  (200 ms)
  bytes 0-1 : output current  LE uint16, 0.1 A/bit
  byte 2    : MPPT mode  (0=off, 1=tracking, 2=CV, 3=float)
  byte 3    : MPPT efficiency %  (0-100 integer)
  byte 4    : status  (0=normal, 1=fault)

0x18D301AA  DC/DC 12V converter  (500 ms)
  bytes 0-1 : output voltage  LE uint16, 0.01 V/bit
  bytes 2-3 : output current  LE uint16, 0.1 A/bit  (0xFFFF = not fitted)
  byte 4    : status  (0=normal, 1=over-V, 2=under-V, 3=fault)

0x18D401AA  Motor temperature  (1000 ms)
  byte 0    : winding temp   uint8, raw-40 = °C, 0xFF=not fitted
  byte 1    : housing temp   uint8, raw-40 = °C, 0xFF=not fitted
  byte 2    : status  (0=normal, 1=warning >80°C, 2=critical >100°C, 0xFF=fault)

0x18D501AA  MPPT temperature  (1000 ms)
  byte 0    : heatsink temp  uint8, raw-40 = °C
  byte 1    : status  (0=normal, 1=warning, 2=critical, 0xFF=fault)

0x18D601AA  Cabin temperature  (1000 ms)
  byte 0    : air temp    uint8, raw-40 = °C
  byte 1    : humidity %  uint8, 0-100  (0xFF = not fitted)
  byte 2    : status  (0=normal, 0xFF=fault)

0x18D701AA  Handbrake position  (100 ms)
  byte 0    : state   (0x00=released, 0x01=engaged, 0xFF=fault)
  byte 1    : debounce  (0x00=stable, 0x01=transitioning)

════════════════════════════════════════════════════════════════════
SOC CALIBRATION — matched to car display
════════════════════════════════════════════════════════════════════
Calibrated from 4 real car logs:
  2026-03-31 10:06  avg=3301.6 mV  car=89.0%  solved_top=3400.7 mV
  2026-03-31 10:57  avg=3306.2 mV  car=89.0%  solved_top=3405.8 mV
  2026-04-01 13:03  avg=3284.8 mV  car=89.4%  solved_top=3377.9 mV
  2026-04-01 16:04  avg=3285.0 mV  car=88.4%  solved_top=3387.4 mV
  Average → SOC_CAR_TOP_MV = 3387 mV = 64.35 V pack

Formula:  SOC = (avg_cell_mV - 2500) / (3387 - 2500) * 100
  2500 mV/cell = 47.50 V pack = 0%  (discharge cut-off)
  3387 mV/cell = 64.35 V pack = 100% (car display top)
  Values above 3387 mV (during active charging) are clamped to 100%.

Pack voltage: sum of 19 cell voltages / 1000 — matches car display to 0.01 V.
"""

import re, sys, time, asyncio, argparse, threading
from collections import deque
from datetime import datetime

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


# ---------------------------------------------------------------------------
# Frame line parser
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
    """Big-endian uint16 at offset o."""
    return (d[o] << 8) | d[o + 1]


def u16le(d, o):
    """Little-endian uint16 at offset o."""
    return d[o] | (d[o + 1] << 8)


def temp_raw(raw):
    """Convert BMS temperature byte → °C.  0xFF = sensor absent → None."""
    if raw == 0xFF:
        return None
    return float(raw) - 40.0


# ---------------------------------------------------------------------------
# Unified vehicle state
# ---------------------------------------------------------------------------

class VehicleState:

    # ── Battery thresholds (BAKO 60V/100Ah spec) ────────────────────────────
    CELL_OV        = 3750   # mV  over-voltage trigger (TYP from OCELL BMS spec)
    CELL_OV_REL    = 3500   # mV  over-voltage release
    CELL_FULL      = 3650   # mV  absolute charge cut-off  (69.35 V / 19)
    CELL_BAL_THR   = 3300   # mV  cell balancing start threshold
    CELL_BAL_DELTA = 20     # mV  minimum delta to trigger balancing
    CELL_NOMINAL   = 3200   # mV  nominal mid-state voltage
    CELL_UV        = 2553   # mV  discharge cut-off  (48.5 V / 19)
    CELL_UV_REL    = 2800   # mV  under-voltage release

    PACK_FULL_V    = 69.35  # V   = 3.65 V × 19  (BMS charge cut-off)
    PACK_CHG_V     = 68.00  # V   recommended charge voltage
    PACK_FLOAT_V   = 64.00  # V   recommended float voltage
    PACK_EMPTY_V   = 48.50  # V   = 2.553 V × 19  (BMS discharge cut-off)
    PACK_NOM_V     = 60.80  # V   nominal

    # SOC calibration — verified against car dashboard (see module docstring)
    SOC_CAR_TOP_MV = 3387   # mV  = 64.35 V pack = 100% on car display
    SOC_CAR_BOT_MV = 2500   # mV  = 47.50 V pack = 0%   (old OCELL reference)
    # NOTE: BAKO discharge cut-off is 48.5 V (2553 mV/cell) but the calibrated
    # SOC bottom reference from real logs is 2500 mV — kept for display match.

    # Battery capacity
    CAP_RATED_AH   = 100.0  # Ah  BAKO spec
    CAP_ACTUAL_AH  = 100.0  # Ah  update after a capacity test
    NUM_CELLS      = 19

    # Current limits
    MAX_CHARGE_A   = 40.0   # A   BAKO spec max
    MAX_DISCH_A    = 100.0  # A   BAKO spec continuous
    MAX_DISCH_PEAK = 140.0  # A   BAKO spec 10 s peak

    # Fault code lookup (byte 7 of 0x18FF28F4)
    FAULT_NAMES = {
        0x00: "Normal",
        0x01: "Severe over-temperature",
        0x02: "Total voltage too high",
        0x03: "Total voltage too low",
        0x04: "Discharge serious overcurrent",
        0x05: "Cell monomer voltage too high",
        0x06: "Cell monomer voltage too low",
    }

    # MPPT mode names (byte 2 of 0x18D201AA)
    MPPT_MODES = {0: "Off", 1: "MPPT tracking", 2: "CV charge", 3: "Float"}

    def __init__(self):
        self.lock = threading.Lock()

        # ── BMS 0x18FF28F4 ─────────────────────────────────────────────────
        self.status_flags      = None   # raw byte 0 bit-field
        self.cable_connected   = None   # bool — charging cable plugged in
        self.is_charging       = None   # bool — battery actively charging
        self.is_discharging    = None   # bool — battery discharging (loss)
        self.pack_ready        = None   # bool — pack ready for use
        self.disch_contactor   = None   # bool — discharge contactor closed
        self.chg_contactor     = None   # bool — charge contactor closed
        self.soc_bms           = None   # % integer 0-100  (byte 1)
        self.pack_current_a    = None   # A  positive=discharge, negative=charge
        self.pack_voltage_v    = None   # V  from 0x18FF28F4 bytes 4-5
        self.fault_level       = None   # 0=normal, 1=Level-1 fault
        self.error_code        = None   # 0x00=normal, see FAULT_NAMES

        # ── BMS 0x18FE28F4 ─────────────────────────────────────────────────
        self.cell_max_mv       = None   # mV
        self.cell_min_mv       = None   # mV
        self.temp_max_c        = None   # °C  (from FE28 byte 4)
        self.temp_min_c        = None   # °C  (from FE28 byte 5)
        self.max_disch_allow_a = None   # A  BMS-reported limit

        # ── Cell voltages 0x18C8-CC28F4 ────────────────────────────────────
        self.cell_mv           = {}     # cell_number (1-19) → int mV

        # ── Temperature probes 0x18B428F4 ──────────────────────────────────
        self.temp_c            = {}     # probe_number (1-8) → float °C

        # ── BMS charging request 0x18FFE5F4 ────────────────────────────────
        self.chg_max_v         = None   # V  max charge terminal voltage
        self.chg_max_a         = None   # A  max charge current request
        self.chg_control       = None   # raw control byte
        self.chg_start_signal  = None   # bool  (bit 0 of control byte)

        # ── Charger feedback 0x18FF50E5 ────────────────────────────────────
        self.chgr_out_v        = None   # V  actual charger output voltage
        self.chgr_out_a        = None   # A  actual charging current
        self.chgr_hw_fault     = None   # bool
        self.chgr_temp_fault   = None   # bool
        self.chgr_low_v_mode   = None   # bool
        self.chgr_input_fault  = None   # bool
        self.chgr_overcurrent  = None   # bool
        self.chgr_active       = None   # bool  (in charging state)
        self.chgr_comm_timeout = None   # bool
        self.chgr_batt_fault   = None   # bool  (reversed/disconnected)

        # ── Solar before MPPT 0x18D001AA ───────────────────────────────────
        self.solar_i_raw_a     = None   # A  instantaneous
        self.solar_i_avg_a     = None   # A  5-cycle average
        self.solar_i_status    = None   # 0=ok, 1=range, 2=fault

        # ── Solar voltage before MPPT 0x18D101AA ───────────────────────────
        self.solar_v           = None   # V  panel voltage
        self.solar_voc         = None   # V  open-circuit voltage
        self.solar_v_status    = None   # 0=ok, 1=over-V, 2=fault

        # ── MPPT output 0x18D201AA ─────────────────────────────────────────
        self.mppt_out_a        = None   # A  current going into battery
        self.mppt_mode         = None   # int 0-3
        self.mppt_efficiency   = None   # %  0-100
        self.mppt_status       = None   # 0=ok, 1=fault

        # ── DC/DC 12V 0x18D301AA ───────────────────────────────────────────
        self.dcdc_v            = None   # V  12V bus voltage
        self.dcdc_a            = None   # A  (None if sensor not fitted)
        self.dcdc_status       = None   # 0=ok, 1=over-V, 2=under-V, 3=fault

        # ── Motor temperature 0x18D401AA ───────────────────────────────────
        self.motor_temp_winding = None  # °C  (None if not fitted)
        self.motor_temp_housing = None  # °C  (None if not fitted)
        self.motor_temp_status  = None  # 0=ok, 1=warn, 2=critical, None=fault

        # ── MPPT temperature 0x18D501AA ────────────────────────────────────
        self.mppt_temp_c       = None   # °C
        self.mppt_temp_status  = None   # 0=ok, 1=warn, 2=critical, None=fault

        # ── Cabin temperature 0x18D601AA ───────────────────────────────────
        self.cabin_temp_c      = None   # °C
        self.cabin_humidity    = None   # %  (None if not fitted)
        self.cabin_status      = None   # 0=ok, None=fault

        # ── Handbrake 0x18D701AA ───────────────────────────────────────────
        self.handbrake_engaged = None   # bool  (None if fault/unknown)
        self.handbrake_stable  = None   # bool  True=stable, False=bouncing

        # ── Bookkeeping ────────────────────────────────────────────────────
        self.frame_count  = 0
        self.connected    = False
        self.port_name    = ""
        self.last_update  = None
        self.raw_log      = deque(maxlen=300)

    # ── Frame decoder ───────────────────────────────────────────────────────

    def decode(self, ts, can_id, dlc, data):
        # Extract PF (bits 23-16) and PS/sub (bits 15-8)
        pf = (can_id >> 16) & 0xFF
        ps = (can_id >>  8) & 0xFF
        sa = (can_id      ) & 0xFF

        with self.lock:
            self.frame_count += 1
            self.last_update  = datetime.now()

            # ── BMS frames  (SA = 0xF4) ─────────────────────────────────

            # 0x18FF28F4 — BMS basic message 1
            if pf == 0xFF and ps == 0x28 and sa == 0xF4 and dlc == 8:
                self._decode_ff28(data)

            # 0x18FE28F4 — BMS basic message 2
            elif pf == 0xFE and ps == 0x28 and sa == 0xF4 and dlc == 8:
                self._decode_fe28(data)

            # 0x18C8-CC28F4 — Cell voltages (big-endian uint16 pairs)
            elif 0xC8 <= pf <= 0xCC and ps == 0x28 and sa == 0xF4 and dlc == 8:
                self._decode_cell_voltages(pf, data)

            # 0x18B428F4 — Temperature probes (up to 8)
            elif pf == 0xB4 and ps == 0x28 and sa == 0xF4 and dlc == 8:
                self._decode_temperatures(data)

            # 0x18FFE5F4 — BMS charging request (BMS → charger)
            elif pf == 0xFF and ps == 0xE5 and sa == 0xF4 and dlc == 8:
                self._decode_ffe5(data)

            # ── Charger frame  (SA = 0x29 or any, PS = 0x50) ────────────

            # 0x18FF50E5 — Charger feedback (charger → BMS)
            elif pf == 0xFF and ps == 0x50 and dlc == 8:
                self._decode_ff50(data)

            # ── New ESP32 sensor frames  (SA = 0xAA) ────────────────────

            elif sa == 0xAA and dlc == 8:
                if   pf == 0xD0: self._decode_solar_current(data)
                elif pf == 0xD1: self._decode_solar_voltage(data)
                elif pf == 0xD2: self._decode_mppt_output(data)
                elif pf == 0xD3: self._decode_dcdc(data)
                elif pf == 0xD4: self._decode_motor_temp(data)
                elif pf == 0xD5: self._decode_mppt_temp(data)
                elif pf == 0xD6: self._decode_cabin(data)
                elif pf == 0xD7: self._decode_handbrake(data)

    # ── Individual frame decoders ────────────────────────────────────────────

    def _decode_ff28(self, d):
        """0x18FF28F4 — BMS basic message 1 (100 ms)"""
        flags                = d[0]
        self.status_flags    = flags
        self.cable_connected = bool(flags & 0x01)
        self.is_charging     = bool(flags & 0x02)
        self.is_discharging  = bool(flags & 0x04)
        self.pack_ready      = bool(flags & 0x08)
        self.disch_contactor = bool(flags & 0x10)
        self.chg_contactor   = bool(flags & 0x20)

        self.soc_bms = d[1]   # integer 0-100 %

        # Current: LE uint16, offset -5000, scale 0.1 A/bit
        raw_i = u16le(d, 2)
        self.pack_current_a = round((raw_i - 5000) * 0.1, 1)

        # Voltage: LE uint16, scale 0.1 V/bit
        self.pack_voltage_v = round(u16le(d, 4) * 0.1, 1)

        self.fault_level = d[6]
        self.error_code  = d[7]

    def _decode_fe28(self, d):
        """0x18FE28F4 — BMS basic message 2 (100 ms)"""
        self.cell_max_mv       = u16le(d, 0)
        self.cell_min_mv       = u16le(d, 2)
        t_max = temp_raw(d[4])
        t_min = temp_raw(d[5])
        if t_max is not None: self.temp_max_c = t_max
        if t_min is not None: self.temp_min_c = t_min
        self.max_disch_allow_a = round(u16le(d, 6) * 0.1, 1)

    def _decode_cell_voltages(self, pf, d):
        """0x18C8-CC28F4 — Cell voltages, big-endian uint16 mV (500 ms)"""
        group = pf - 0xC8          # 0-4
        base  = group * 4 + 1      # first cell number in this frame (1-based)
        for i in range(4):
            mv = u16be(d, i * 2)
            if mv != 0:            # 0x0000 = padding (no cell 20)
                self.cell_mv[base + i] = mv

    def _decode_temperatures(self, d):
        """0x18B428F4 — Temperature probes 1-8 (500 ms)"""
        for i in range(8):
            t = temp_raw(d[i])
            if t is not None:
                self.temp_c[i + 1] = t
            elif i + 1 in self.temp_c:
                # Remove previously active probe if now reading 0xFF
                del self.temp_c[i + 1]

    def _decode_ffe5(self, d):
        """0x18FFE5F4 — BMS charging request to charger (1000 ms)"""
        self.chg_max_v       = round(u16le(d, 0) * 0.1, 1)
        self.chg_max_a       = round(u16le(d, 2) * 0.1, 1)
        self.chg_control     = d[4]
        self.chg_start_signal = bool(d[4] & 0x01)

    def _decode_ff50(self, d):
        """0x18FF50E5 — Charger feedback to BMS (1000 ms)"""
        self.chgr_out_v      = round(u16le(d, 0) * 0.1, 1)
        self.chgr_out_a      = round(u16le(d, 2) * 0.1, 1)
        flags = d[4]
        self.chgr_hw_fault    = bool(flags & 0x01)
        self.chgr_temp_fault  = bool(flags & 0x02)
        self.chgr_low_v_mode  = bool(flags & 0x04)
        self.chgr_input_fault = bool(flags & 0x08)
        self.chgr_overcurrent = bool(flags & 0x10)
        self.chgr_active      = bool(flags & 0x20)
        self.chgr_comm_timeout= bool(flags & 0x40)
        self.chgr_batt_fault  = bool(flags & 0x80)

    def _decode_solar_current(self, d):
        """0x18D001AA — Solar panel current before MPPT (200 ms)"""
        self.solar_i_raw_a = round(u16le(d, 0) * 0.1, 1)
        self.solar_i_avg_a = round(u16le(d, 2) * 0.1, 1)
        self.solar_i_status = d[4]

    def _decode_solar_voltage(self, d):
        """0x18D101AA — Solar panel voltage before MPPT (200 ms)"""
        self.solar_v        = round(u16le(d, 0) * 0.1, 1)
        self.solar_voc      = round(u16le(d, 2) * 0.1, 1)
        self.solar_v_status = d[4]

    def _decode_mppt_output(self, d):
        """0x18D201AA — MPPT output current after MPPT (200 ms)"""
        self.mppt_out_a      = round(u16le(d, 0) * 0.1, 1)
        self.mppt_mode       = d[2]
        self.mppt_efficiency = d[3]
        self.mppt_status     = d[4]

    def _decode_dcdc(self, d):
        """0x18D301AA — DC/DC 12V converter (500 ms)"""
        self.dcdc_v = round(u16le(d, 0) * 0.01, 2)
        raw_a = u16le(d, 2)
        self.dcdc_a = round(raw_a * 0.1, 1) if raw_a != 0xFFFF else None
        self.dcdc_status = d[4]

    def _decode_motor_temp(self, d):
        """0x18D401AA — Motor temperature (1000 ms)"""
        self.motor_temp_winding = temp_raw(d[0])
        self.motor_temp_housing = temp_raw(d[1])
        s = d[2]
        self.motor_temp_status = None if s == 0xFF else s

    def _decode_mppt_temp(self, d):
        """0x18D501AA — MPPT temperature (1000 ms)"""
        self.mppt_temp_c = temp_raw(d[0])
        s = d[1]
        self.mppt_temp_status = None if s == 0xFF else s

    def _decode_cabin(self, d):
        """0x18D601AA — Cabin temperature/humidity (1000 ms)"""
        self.cabin_temp_c  = temp_raw(d[0])
        self.cabin_humidity = None if d[1] == 0xFF else d[1]
        self.cabin_status  = None if d[2] == 0xFF else d[2]

    def _decode_handbrake(self, d):
        """0x18D701AA — Handbrake position (100 ms)"""
        s = d[0]
        self.handbrake_engaged = None if s == 0xFF else bool(s)
        self.handbrake_stable  = (d[1] == 0x00)

    # ── Derived / helper calculations ───────────────────────────────────────

    def cell_status(self, mv):
        if mv >= self.CELL_OV:       return "overvoltage"
        if mv >= self.CELL_FULL:     return "full"
        if mv >= self.CELL_BAL_THR:  return "good"
        if mv >= self.CELL_NOMINAL:  return "normal"
        if mv >= self.CELL_UV:       return "low"
        return "undervoltage"

    def _soc_from_cells(self, cv):
        """Calibrated SOC from average cell voltage — matches car display."""
        if not cv:
            return None
        avg_mv = sum(cv.values()) / len(cv)
        soc = (avg_mv - self.SOC_CAR_BOT_MV) / (self.SOC_CAR_TOP_MV - self.SOC_CAR_BOT_MV) * 100.0
        return round(max(0.0, min(100.0, soc)), 1)

    def _pack_v_from_cells(self, cv):
        """Pack voltage = sum of all 19 cell voltages (V). Needs ≥5 cells."""
        if len(cv) < 5:
            return None
        return round(sum(cv.values()) / 1000.0, 2)

    def _remaining_ah(self, soc):
        if soc is None:
            return None
        return round(soc / 100.0 * self.CAP_ACTUAL_AH, 2)

    def _fault_name(self):
        if self.error_code is None:
            return None
        return self.FAULT_NAMES.get(self.error_code, f"Unknown fault 0x{self.error_code:02X}")

    def _solar_power_w(self):
        """Instantaneous solar panel power in Watts."""
        if self.solar_v is not None and self.solar_i_raw_a is not None:
            return round(self.solar_v * self.solar_i_raw_a, 1)
        return None

    def _mppt_power_out_w(self):
        """MPPT output power into battery in Watts."""
        if self.pack_voltage_v is not None and self.mppt_out_a is not None:
            return round(self.pack_voltage_v * self.mppt_out_a, 1)
        return None

    # ── JSON serialisation ───────────────────────────────────────────────────

    def to_dict(self):
        with self.lock:
            cv   = dict(self.cell_mv)
            temp = dict(self.temp_c)

            # Primary SOC: voltage-calibrated (matches dashboard)
            soc_volt = self._soc_from_cells(cv)
            # Fallback: use BMS SOC byte if no cell voltages yet
            soc_disp = soc_volt if soc_volt is not None else (
                float(self.soc_bms) if self.soc_bms is not None else None
            )

            # Pack voltage: prefer cell-sum (accurate); fallback to FF28 value
            pack_v = self._pack_v_from_cells(cv) or self.pack_voltage_v

            cells_out = {
                str(k): {"mv": cv[k], "status": self.cell_status(cv[k])}
                for k in sorted(cv)
            }

            return {
                # ── Connection ───────────────────────────────────────────────
                "connected":    self.connected,
                "port":         self.port_name,
                "frame_count":  self.frame_count,
                "timestamp":    self.last_update.isoformat() if self.last_update else None,

                # ── SOC ──────────────────────────────────────────────────────
                # soc          = primary display SOC (voltage-calibrated, matches dashboard)
                # soc_bms      = SOC % from BMS byte 1 of 0x18FF28F4 (integer 0-100)
                # soc_chg_req  = BMS SOC used for charger logic (from 0x18FFE5F4 voltage)
                "soc":          soc_disp,
                "soc_bms":      self.soc_bms,

                # ── Capacity ─────────────────────────────────────────────────
                "remaining_ah": self._remaining_ah(soc_disp),
                "capacity_ah":  self.CAP_ACTUAL_AH,
                "capacity_rated": self.CAP_RATED_AH,
                "soh":          round(self.CAP_ACTUAL_AH / self.CAP_RATED_AH * 100, 1),

                # ── Pack ─────────────────────────────────────────────────────
                "pack_v":           pack_v,
                "pack_current_a":   self.pack_current_a,   # + = discharge, - = charge
                "pack_voltage_v":   self.pack_voltage_v,   # directly from FF28 bytes 4-5

                # ── BMS status flags ─────────────────────────────────────────
                "cable_connected":  self.cable_connected,
                "is_charging":      self.is_charging,
                "is_discharging":   self.is_discharging,
                "pack_ready":       self.pack_ready,
                "disch_contactor":  self.disch_contactor,
                "chg_contactor":    self.chg_contactor,

                # ── Fault ────────────────────────────────────────────────────
                "fault_level":      self.fault_level,
                "error_code":       self.error_code,
                "fault_name":       self._fault_name(),

                # ── BMS limits ───────────────────────────────────────────────
                "max_disch_allow_a": self.max_disch_allow_a,

                # ── Charging request (BMS → charger) ────────────────────────
                "chg_max_v":        self.chg_max_v,
                "chg_max_a":        self.chg_max_a,
                "chg_start_signal": self.chg_start_signal,

                # ── Charger feedback ─────────────────────────────────────────
                "chgr_out_v":       self.chgr_out_v,
                "chgr_out_a":       self.chgr_out_a,
                "chgr_active":      self.chgr_active,
                "chgr_hw_fault":    self.chgr_hw_fault,
                "chgr_temp_fault":  self.chgr_temp_fault,
                "chgr_overcurrent": self.chgr_overcurrent,
                "chgr_batt_fault":  self.chgr_batt_fault,

                # ── Cells ────────────────────────────────────────────────────
                "cells":          cells_out,
                "cell_count":     len(cv),
                "cell_max_mv":    self.cell_max_mv,
                "cell_min_mv":    self.cell_min_mv,
                "cell_spread_mv": (self.cell_max_mv - self.cell_min_mv)
                                  if (self.cell_max_mv is not None and self.cell_min_mv is not None)
                                  else None,
                "cell_avg_mv":    round(sum(cv.values()) / len(cv)) if cv else None,

                # ── Temperatures ─────────────────────────────────────────────
                # temp_c        : dict of BMS probe index → °C (probes 1-8 from 0x18B428F4)
                # temp_max_c    : max cell temp from 0x18FE28F4 byte 4
                # temp_min_c    : min cell temp from 0x18FE28F4 byte 5
                "temps":      {str(k): round(v, 1) for k, v in sorted(temp.items())},
                "avg_temp":   round(sum(temp.values()) / len(temp), 1) if temp else None,
                "temp_max_c": self.temp_max_c,
                "temp_min_c": self.temp_min_c,

                # ── Solar + MPPT ─────────────────────────────────────────────
                "solar_i_raw_a":   self.solar_i_raw_a,
                "solar_i_avg_a":   self.solar_i_avg_a,
                "solar_v":         self.solar_v,
                "solar_voc":       self.solar_voc,
                "solar_power_w":   self._solar_power_w(),
                "solar_i_status":  self.solar_i_status,
                "solar_v_status":  self.solar_v_status,

                "mppt_out_a":      self.mppt_out_a,
                "mppt_mode":       self.mppt_mode,
                "mppt_mode_name":  self.MPPT_MODES.get(self.mppt_mode, "Unknown") if self.mppt_mode is not None else None,
                "mppt_efficiency": self.mppt_efficiency,
                "mppt_power_w":    self._mppt_power_out_w(),
                "mppt_status":     self.mppt_status,
                "mppt_temp_c":     self.mppt_temp_c,
                "mppt_temp_status":self.mppt_temp_status,

                # ── DC/DC 12V ────────────────────────────────────────────────
                "dcdc_v":      self.dcdc_v,
                "dcdc_a":      self.dcdc_a,
                "dcdc_status": self.dcdc_status,

                # ── Motor ─────────────────────────────────────────────────────
                "motor_temp_winding": self.motor_temp_winding,
                "motor_temp_housing": self.motor_temp_housing,
                "motor_temp_status":  self.motor_temp_status,

                # ── Cabin ────────────────────────────────────────────────────
                "cabin_temp_c":   self.cabin_temp_c,
                "cabin_humidity": self.cabin_humidity,
                "cabin_status":   self.cabin_status,

                # ── Handbrake ────────────────────────────────────────────────
                "handbrake_engaged": self.handbrake_engaged,
                "handbrake_stable":  self.handbrake_stable,

                # ── UI thresholds (for dashboard colour coding) ───────────────
                "thresh": {
                    "cell_ov":       self.CELL_OV,
                    "cell_ov_rel":   self.CELL_OV_REL,
                    "cell_full":     self.CELL_FULL,
                    "cell_soc_top":  self.SOC_CAR_TOP_MV,
                    "cell_bal":      self.CELL_BAL_THR,
                    "cell_bal_d":    self.CELL_BAL_DELTA,
                    "cell_nom":      self.CELL_NOMINAL,
                    "cell_uv":       self.CELL_UV,
                    "cell_uv_rel":   self.CELL_UV_REL,
                    "pack_full":     self.PACK_FULL_V,
                    "pack_chg":      self.PACK_CHG_V,
                    "pack_float":    self.PACK_FLOAT_V,
                    "pack_soc_top":  round(self.SOC_CAR_TOP_MV * self.NUM_CELLS / 1000, 2),
                    "pack_empty":    self.PACK_EMPTY_V,
                    "max_chg_a":     self.MAX_CHARGE_A,
                    "max_dch_a":     self.MAX_DISCH_A,
                    "max_dch_peak":  self.MAX_DISCH_PEAK,
                },

                "log": list(self.raw_log)[-80:],
            }


# ---------------------------------------------------------------------------
# Serial reader
# ---------------------------------------------------------------------------

def auto_detect_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc + hwid for k in ["cp210", "ch340", "ftdi", "esp32",
                                           "silicon labs", "uart"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


_start_time = time.time()


def millis():
    return int((time.time() - _start_time) * 1000)


def normalize_line(line):
    """
    Normalise any CAN log line to [Nms] ID: 0xXXXXXXXX DLC: N Data: AA BB ...
    Handles: MCP2515 receiver output, ESP32 simulator output,
             and other tools with 0x-prefixed data bytes.
    """
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


# ---------------------------------------------------------------------------
# FastAPI
# ---------------------------------------------------------------------------

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"],
                   allow_methods=["*"], allow_headers=["*"])

vehicle = VehicleState()
_stop   = threading.Event()


@app.get("/", response_class=HTMLResponse)
async def index():
    with open("../frontend/index.html", encoding="utf-8") as f:
        return f.read()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(vehicle.to_dict())
            await asyncio.sleep(0.1)    # 10 Hz
    except (WebSocketDisconnect, Exception):
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="BAKO Motors CAN dashboard server")
    parser.add_argument("--port",     "-p", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud",     "-b", type=int, default=115200)
    parser.add_argument("--host",          default="0.0.0.0")
    parser.add_argument("--web-port",      type=int, default=8765, dest="web_port")
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("No serial port found. Use --port to specify one.")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  {p.description}")
        sys.exit(1)

    top_v = round(VehicleState.SOC_CAR_TOP_MV * VehicleState.NUM_CELLS / 1000, 2)
    cap   = VehicleState.CAP_ACTUAL_AH
    soh   = round(cap / VehicleState.CAP_RATED_AH * 100, 1)

    print("-" * 56)
    print("  BAKO Motors CAN Dashboard")
    print("-" * 56)
    print(f"  Serial    : {port} @ {args.baud} baud")
    print(f"  Browser   : http://localhost:{args.web_port}")
    print(f"  Battery   : 60V / {cap} Ah LFP  19S1P  (SOH {soh}%)")
    print(f"  SOC range : {VehicleState.PACK_EMPTY_V} V = 0%  →  {top_v} V = 100%")
    print(f"  CAN nodes : BMS=0xF4  Display=0x28  Charger=0xE5  ESP32=0xAA")
    print("-" * 56)

    threading.Thread(target=serial_reader,
                     args=(port, args.baud, vehicle, _stop),
                     daemon=True).start()

    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
