#!/usr/bin/env python3
"""
BMS Log → VPS Cloud Pipeline
=============================
Reads a raw CAN log file, decodes every BMS frame (same logic as the ESP32
firmware), builds a complete JSON snapshot, and POSTs it to the VPS backend
at POST /api/ingest — exactly what the real ESP32 does via SIM800L.

Pipeline:
    log file → decode frames → JSON → POST /api/ingest → server.py → browser

Usage:
    python replay_to_cloud.py                     # default log, localhost
    python replay_to_cloud.py --speed 5            # 5× real-time
    python replay_to_cloud.py --interval 5000      # POST every 5 s of log time
    python replay_to_cloud.py --loop               # loop file forever
    python replay_to_cloud.py --host 1.2.3.4 --port 8765
"""

import re, sys, time, json, argparse
import urllib.request, urllib.error
from datetime import datetime
from pathlib import Path

# ─── VPS credentials ─────────────────────────────────────────────────────────
VPS_HOST   = "localhost"
VPS_PORT   = 8765
VPS_PATH   = "/api/ingest"
VPS_API_KEY = "bako-bms-2024"
DEVICE_ID  = "esp32-bms-001"

# ─── CAN decode constants (mirrors firmware) ─────────────────────────────────
CELL_UV     = 2500
SOC_TOP_MV  = 3387

FAULT_NAMES = {
    0x00: "ok",
    0x01: "over_temp_severe",
    0x02: "total_voltage_high",
    0x03: "total_voltage_low",
    0x04: "discharge_overcurrent",
    0x05: "cell_voltage_high",
    0x06: "cell_voltage_low",
}

FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:\s+([0-9A-Fa-f\s]+)'
)

# ─── Helpers ─────────────────────────────────────────────────────────────────

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

def u16be(d, o): return (d[o] << 8) | d[o + 1]
def u16le(d, o): return d[o] | (d[o + 1] << 8)

def cell_status(mv):
    if mv >= 3750: return "overvoltage"
    if mv >= 3650: return "full"
    if mv >= 3300: return "good"
    if mv >= 3200: return "normal"
    if mv >= 2500: return "low"
    return "undervoltage"


# ─── BMS decoder (exact mirror of firmware decodeFrame + BMSData) ─────────────

class BMSDecoder:

    def __init__(self):
        # 0x18C8-CC28F4 — cell voltages
        self.cell_mv      = {}          # {1..19: mV}
        self.cell_count   = 0
        self.valid_cells  = 0
        self.cell_max     = 0
        self.cell_min     = 0xFFFF
        self.cell_avg     = 0
        self.pack_v       = 0.0
        self.soc          = -1.0

        # 0x18B428F4 — temperatures
        self.temp_c       = {}          # {1..8: °C}
        self.temp_count   = 0

        # 0x18FF28F4 — BMS Basic Message 1
        self.status_flags = 0
        self.soc_bms      = 0xFF        # 0xFF = unknown
        self.pack_current = 0.0
        self.pack_v_bms   = 0.0
        self.fault_level  = 0
        self.error_code   = 0

        # 0x18FE28F4 — BMS Basic Message 2
        self.temp_max_c   = None
        self.temp_min_c   = None
        self.max_disch_a  = 0.0

        # 0x18FFE5F4 — BMS Charging Request
        self.charge_max_v = 0.0
        self.charge_max_a = 0.0
        self.charger_start = False

        self.frame_count  = 0

    def decode(self, can_id, dlc, data):
        func = (can_id >> 16) & 0xFF
        sub  = (can_id >>  8) & 0xFF
        self.frame_count += 1

        # ── 0x18C8-CC28F4  Cell voltages ────────────────────────────────────
        if 0xC8 <= func <= 0xCC and dlc == 8:
            base = (func - 0xC8) * 4
            for i in range(4):
                mv = u16be(data, i * 2)
                if mv != 0 and (base + i) < 19:
                    self.cell_mv[base + i + 1] = mv
                    if base + i + 1 > self.cell_count:
                        self.cell_count = base + i + 1

            valid = 0; total = 0
            mx = 0; mn = 0xFFFF
            for mv in self.cell_mv.values():
                total += mv; valid += 1
                if mv > mx: mx = mv
                if mv < mn: mn = mv
            self.valid_cells = valid
            self.cell_max    = mx
            self.cell_min    = mn
            self.cell_avg    = total // valid if valid else 0
            self.pack_v      = total / 1000.0
            s = (self.cell_avg - CELL_UV) / (SOC_TOP_MV - CELL_UV) * 100.0
            self.soc         = round(max(0.0, min(100.0, s)), 1)

        # ── 0x18B428F4  Temperatures ─────────────────────────────────────────
        elif func == 0xB4 and dlc >= 1:
            self.temp_c.clear()
            self.temp_count = 0
            for i in range(min(dlc, 8)):
                if data[i] != 0xFF and data[i] != 0x00:
                    self.temp_c[i + 1] = float(data[i]) - 40.0
                    self.temp_count += 1

        # ── 0x18FF28F4  BMS Basic Message 1 ─────────────────────────────────
        elif func == 0xFF and sub == 0x28 and dlc >= 8:
            self.status_flags = data[0]
            self.soc_bms      = data[1]
            self.pack_current = round((u16le(data, 2) - 5000) * 0.1, 1)
            self.pack_v_bms   = round(u16le(data, 4) * 0.1, 1)
            self.fault_level  = data[6]
            self.error_code   = data[7]

        # ── 0x18FFE5F4  BMS Charging Request ────────────────────────────────
        elif func == 0xFF and sub == 0xE5 and dlc >= 5:
            self.charge_max_v  = round(u16le(data, 0) * 0.1, 1)
            self.charge_max_a  = round(u16le(data, 2) * 0.1, 1)
            self.charger_start = not bool(data[4] & 0x01)

        # ── 0x18FE28F4  BMS Basic Message 2 ─────────────────────────────────
        elif func == 0xFE and sub == 0x28 and dlc >= 8:
            if self.cell_count == 0:
                self.cell_max = u16le(data, 0)
                self.cell_min = u16le(data, 2)
                avg = (self.cell_max + self.cell_min) // 2
                self.pack_v = avg * 19 / 1000.0
                s = (avg - CELL_UV) / (SOC_TOP_MV - CELL_UV) * 100.0
                self.soc = round(max(0.0, min(100.0, s)), 1)
            if data[4] != 0xFF:
                self.temp_max_c = float(data[4]) - 40.0
            if data[5] != 0xFF:
                self.temp_min_c = float(data[5]) - 40.0
            self.max_disch_a = round(u16le(data, 6) * 0.1, 1)

    def build_json(self, timestamp: str = "") -> dict:
        """Build nested JSON snapshot — mirrors firmware buildJSON() schema."""

        pack_v   = self.pack_v_bms if self.pack_v_bms > 0 else round(self.pack_v, 2)
        cell_max = self.cell_max if self.cell_max > 0 else None
        cell_min = self.cell_min if self.cell_min < 0xFFFF else None

        # cells array: index 0 = null, indices 1-19 = {mv, status} or null
        cells_arr = [None]
        for i in range(1, 20):
            mv = self.cell_mv.get(i)
            if mv:
                cells_arr.append({"mv": mv, "status": cell_status(mv)})
            else:
                cells_arr.append(None)

        # battery_cells array: index 0 = null, indices 1-4 = probe readings or null
        temp_arr = [None]
        for i in range(1, 5):
            t = self.temp_c.get(i)
            temp_arr.append(round(t, 1) if t is not None else None)

        avg_temp = None
        if self.temp_c:
            avg_temp = round(sum(self.temp_c.values()) / len(self.temp_c), 1)

        return {
            "device_id":   DEVICE_ID,
            "timestamp":   timestamp or datetime.now().isoformat(),
            "source":      "replay",
            "connected":   True,
            "frame_count": self.frame_count,
            "fault_level": int(self.fault_level),
            "error_code":  int(self.error_code),

            "solar": {
                "pre_mppt":  {"voltage_v": None, "current_a": None},
                "post_mppt": {"current_a": None},
            },
            "dc_dc": {
                "output_64v": {"voltage_v": None},
                "output_12v": {"voltage_v": None},
            },

            "battery": {
                "pack_v":         pack_v,
                "pack_current_a": self.pack_current,
                "soc":            self.soc if self.soc >= 0 else None,
                "soc_bms":        int(self.soc_bms) if self.soc_bms <= 100 else None,
                "max_disch_a":    self.max_disch_a if self.max_disch_a > 0 else 0,
                "cell_count":     self.valid_cells,
                "cell_avg_mv":    self.cell_avg if self.cell_avg > 0 else None,
                "cell_min_mv":    cell_min,
                "cell_max_mv":    cell_max,
                "cell_spread_mv": (cell_max - cell_min) if (cell_max and cell_min) else None,
                "cells":          cells_arr,
                "charger": {
                    "max_charge_v": self.charge_max_v,
                    "max_charge_a": self.charge_max_a,
                    "start_signal": self.charger_start,
                },
                "status": {
                    "charge_cable":     bool(self.status_flags & 0x01),
                    "charging":         bool(self.status_flags & 0x02),
                    "discharging":      bool(self.status_flags & 0x04),
                    "ready":            bool(self.status_flags & 0x08),
                    "disch_contactor":  bool(self.status_flags & 0x10),
                    "charge_contactor": bool(self.status_flags & 0x20),
                },
            },

            "temperatures": {
                "motor_c":       None,
                "mppt_c":        None,
                "cabin_c":       None,
                "battery_avg_c": avg_temp,
                "battery_min_c": self.temp_min_c,
                "battery_max_c": self.temp_max_c,
                "battery_cells": temp_arr,
            },

            "vehicle": {"handbrake": None},
        }


# ─── VPS POST ─────────────────────────────────────────────────────────────────

def vps_post(payload: dict, host: str, port: int) -> bool:
    """POST a JSON snapshot to the VPS /api/ingest endpoint."""
    url  = f"http://{host}:{port}{VPS_PATH}"
    body = json.dumps(payload).encode()
    req  = urllib.request.Request(
        url, data=body,
        headers={
            "Content-Type": "application/json",
            "X-Api-Key":    VPS_API_KEY,
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            resp.read()
            return True
    except urllib.error.HTTPError as e:
        print(f"  [ERR] HTTP {e.code}: {e.reason}")
    except Exception as e:
        print(f"  [ERR] POST failed: {e}")
    return False


# ─── Replay loop ──────────────────────────────────────────────────────────────

def replay(log_path: str, speed: float, interval_ms: int, loop_forever: bool,
           host: str, port: int):

    print(f"  Log      : {log_path}")
    print(f"  Endpoint : http://{host}:{port}{VPS_PATH}")
    print(f"  Speed    : {speed}×   interval: {interval_ms} ms log-time")
    print("-" * 55)

    frames = []
    with open(log_path, encoding="utf-8") as f:
        for line in f:
            r = parse_line(line.strip())
            if r:
                frames.append(r)

    if not frames:
        print("No parseable frames found."); sys.exit(1)

    print(f"  {len(frames)} frames  ({frames[0][0]}–{frames[-1][0]} ms)")
    print()

    loop_n = 0
    while True:
        loop_n += 1
        dec  = BMSDecoder()
        t0_log  = frames[0][0]
        t0_wall = time.time()
        next_post_ms = t0_log + interval_ms
        post_n = 0

        for ts, can_id, dlc, data in frames:
            elapsed_log  = (ts - t0_log) / 1000.0 / speed
            elapsed_wall = time.time() - t0_wall
            sleep_s = elapsed_log - elapsed_wall
            if sleep_s > 0.001:
                time.sleep(sleep_s)

            dec.decode(can_id, dlc, data)

            if ts >= next_post_ms and dec.cell_mv:
                post_n += 1
                payload = dec.build_json(datetime.now().isoformat())

                batt    = payload.get("battery", {})
                cells   = batt.get("cell_count", 0)
                soc     = batt.get("soc")
                pack    = batt.get("pack_v")
                current = batt.get("pack_current_a")
                temps   = payload.get("temperatures", {})
                avg_t   = temps.get("battery_avg_c")
                fault   = payload.get("fault_level")
                ready   = batt.get("status", {}).get("ready")

                ok  = vps_post(payload, host, port)
                tag = "OK  " if ok else "FAIL"
                print(f"  [{tag}] #{post_n:02d}  "
                      f"cells={cells}  SOC={soc}%  pack={pack}V  I={current}A  "
                      f"temp={avg_t}°C  fault={fault}  ready={ready}")
                next_post_ms = ts + interval_ms

        print(f"\n  Loop {loop_n} done — {post_n} snapshots posted to VPS.")
        if not loop_forever:
            break
        print("  Restarting …\n")
        time.sleep(1.0 / speed)

    print("\nReplay complete.")


# ─── Entry point ─────────────────────────────────────────────────────────────

DEFAULT_LOG = str(
    Path(__file__).parent.parent / "data" / "raw" /
    "bms_log_2026-03-10T10-20-02.txt"
)

def main():
    parser = argparse.ArgumentParser(
        description="Replay BMS CAN log → POST /api/ingest → dashboard"
    )
    parser.add_argument("--log",      "-l", default=DEFAULT_LOG,
                        help="Path to raw CAN log file")
    parser.add_argument("--speed",    "-s", type=float, default=1.0,
                        help="Replay speed multiplier (default 1.0 = real-time)")
    parser.add_argument("--interval", "-i", type=int,   default=5000,
                        help="Snapshot interval in log-time ms (default 5000)")
    parser.add_argument("--loop",          action="store_true",
                        help="Loop file forever")
    parser.add_argument("--host",          default=VPS_HOST,
                        help=f"VPS host (default {VPS_HOST})")
    parser.add_argument("--port",     "-p", type=int, default=VPS_PORT,
                        help=f"VPS port (default {VPS_PORT})")
    args = parser.parse_args()

    print("-" * 55)
    print("  BMS Log → VPS Cloud Pipeline")
    print("-" * 55)
    replay(
        log_path    = args.log,
        speed       = args.speed,
        interval_ms = args.interval,
        loop_forever= args.loop,
        host        = args.host,
        port        = args.port,
    )

if __name__ == "__main__":
    main()
