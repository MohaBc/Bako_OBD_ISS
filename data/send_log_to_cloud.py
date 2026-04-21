#!/usr/bin/env python3
"""
send_log_to_cloud.py — Replay a BMS CAN log to the VPS backend
===============================================================
Parses a raw CAN log file and sends BMS state as JSON to the VPS backend
at POST /api/ingest — the same endpoint the ESP32 uses via SIM800L.

Modes
─────
  snapshot   Parse the whole file → send one final snapshot  (default)
  replay     Stream snapshots in time order → animates the dashboard live

Usage
─────
  # Quick snapshot → local backend
  python send_log_to_cloud.py

  # Live replay at 20× speed
  python send_log_to_cloud.py --mode replay --speed 20

  # Remote VPS
  python send_log_to_cloud.py --api-url http://1.2.3.4:8765/api/ingest

  # Specify a different log file
  python send_log_to_cloud.py raw/bms_log_2026-03-10T10-20-02.txt
"""

import re, sys, time, json, argparse, urllib.request, urllib.error
from pathlib import Path
from datetime import datetime

# ── Defaults ──────────────────────────────────────────────────────────────────
DEFAULT_LOG      = Path(__file__).parent / "raw" / "bms_log_2026-03-10T10-20-02.txt"
DEFAULT_API_URL  = "http://localhost:8765/api/ingest"
DEFAULT_API_KEY  = "bako-bms-2024"
DEVICE_ID        = "esp32-bms-001"

# ── Frame regex ───────────────────────────────────────────────────────────────
FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:\s+([0-9A-Fa-f\s]+)',
    re.IGNORECASE,
)

CELL_UV        = 2500
SOC_CAR_TOP_MV = 3387

# ── Byte helpers ──────────────────────────────────────────────────────────────
def u16be(d, o): return (d[o] << 8) | d[o + 1]
def u16le(d, o): return d[o] | (d[o + 1] << 8)

def cell_status(mv):
    if mv >= 3750: return "overvoltage"
    if mv >= 3650: return "full"
    if mv >= 3300: return "good"
    if mv >= 3200: return "normal"
    if mv >= 2500: return "low"
    return "undervoltage"

# ── BMS accumulator ───────────────────────────────────────────────────────────
class BMS:
    def __init__(self):
        self.cell_mv      = {}
        self.temp_c       = {}
        self.status_flags = 0
        self.soc_bms      = 0xFF
        self.pack_current = 0.0
        self.pack_v_bms   = 0.0
        self.fault_level  = 0
        self.error_code   = 0
        self.max_disch_a  = 0.0
        self.temp_max_c   = None
        self.temp_min_c   = None
        self.charge_max_v = 0.0
        self.charge_max_a = 0.0
        self.charger_start = False
        self.frames       = 0

    def decode(self, can_id, data):
        func = (can_id >> 16) & 0xFF
        sub  = (can_id >>  8) & 0xFF
        self.frames += 1

        if 0xC8 <= func <= 0xCC and len(data) == 8:
            group = func - 0xC8
            base  = group * 4 + 1
            for i in range(4):
                o = i * 2
                if o + 1 < len(data):
                    mv = u16be(data, o)
                    if mv and base + i <= 19:
                        self.cell_mv[base + i] = mv

        elif func == 0xB4 and len(data) >= 4:
            self.temp_c = {}
            for i in range(min(len(data), 8)):
                v = data[i]
                if v not in (0x00, 0xFF):
                    self.temp_c[i + 1] = round(float(v) - 40.0, 1)

        elif func == 0xFF and sub == 0x28 and len(data) >= 8:
            self.status_flags = data[0]
            self.soc_bms      = data[1]
            self.pack_current = round((u16le(data, 2) - 5000) * 0.1, 1)
            self.pack_v_bms   = round(u16le(data, 4) * 0.1, 1)
            self.fault_level  = data[6]
            self.error_code   = data[7]

        elif func == 0xFF and sub == 0xE5 and len(data) >= 5:
            self.charge_max_v  = round(u16le(data, 0) * 0.1, 1)
            self.charge_max_a  = round(u16le(data, 2) * 0.1, 1)
            self.charger_start = not bool(data[4] & 0x01)

        elif func == 0xFE and sub == 0x28 and len(data) >= 8:
            if data[4] != 0xFF:
                self.temp_max_c = float(data[4]) - 40.0
            if data[5] != 0xFF:
                self.temp_min_c = float(data[5]) - 40.0
            self.max_disch_a = round(u16le(data, 6) * 0.1, 1)

    def complete(self):
        return len(self.cell_mv) >= 4

    def to_json(self, ts=None):
        cv = dict(self.cell_mv)
        avg_mv = round(sum(cv.values()) / len(cv)) if cv else None
        pack_v_cells = round(sum(cv.values()) / 1000.0, 2) if len(cv) >= 5 else None
        pack_v = self.pack_v_bms if self.pack_v_bms > 0 else pack_v_cells

        soc = None
        if avg_mv:
            s = (avg_mv - CELL_UV) / (SOC_CAR_TOP_MV - CELL_UV) * 100
            soc = round(max(0.0, min(100.0, s)), 1)

        cell_min = min(cv.values()) if cv else None
        cell_max = max(cv.values()) if cv else None

        # cells array: index 0 = null, indices 1-19
        cells_arr = [None]
        for i in range(1, 20):
            mv = cv.get(i)
            if mv:
                cells_arr.append({"mv": mv, "status": cell_status(mv)})
            else:
                cells_arr.append(None)

        # battery_cells array: index 0 = null, indices 1-4
        temp_arr = [None]
        for i in range(1, 5):
            t = self.temp_c.get(i)
            temp_arr.append(t if t is not None else None)

        avg_temp = round(sum(self.temp_c.values()) / len(self.temp_c), 1) if self.temp_c else None

        return {
            "device_id":   DEVICE_ID,
            "timestamp":   ts or datetime.now().isoformat(),
            "source":      "log-replay",
            "connected":   True,
            "frame_count": self.frames,
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
                "soc":            soc,
                "soc_bms":        int(self.soc_bms) if self.soc_bms <= 100 else None,
                "max_disch_a":    self.max_disch_a if self.max_disch_a > 0 else 0,
                "cell_count":     len(cv),
                "cell_avg_mv":    avg_mv,
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


# ── Frame iterator ────────────────────────────────────────────────────────────
def iter_frames(path):
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = FRAME_RE.search(line)
            if not m:
                continue
            ts_ms  = int(m.group(1))
            can_id = int(m.group(2), 16)
            data   = bytes(int(b, 16) for b in m.group(4).split())
            yield ts_ms, can_id, data


# ── HTTP sender ───────────────────────────────────────────────────────────────
def post_vps(payload: dict, api_url: str, api_key: str) -> bool:
    body = json.dumps(payload).encode()
    req  = urllib.request.Request(
        api_url, data=body,
        headers={"Content-Type": "application/json", "X-Api-Key": api_key},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            ok = r.status == 200
            print(f"  [POST] {r.status} → {api_url}")
            return ok
    except urllib.error.HTTPError as e:
        print(f"  [POST] HTTP {e.code}: {e.read().decode()[:120]}")
    except Exception as e:
        print(f"  [POST] Error: {e}")
    return False


# ── Modes ─────────────────────────────────────────────────────────────────────
def mode_snapshot(path, args):
    print(f"\nParsing {path.name} ...")
    bms = BMS()
    for _, can_id, data in iter_frames(path):
        bms.decode(can_id, data)

    if not bms.complete():
        print("No cell voltage frames found — check the log file.")
        sys.exit(1)

    payload = bms.to_json()
    _print_summary(payload)
    print(f"\nSending snapshot to {args.api_url} ...")
    post_vps(payload, args.api_url, args.api_key)
    print("\nDone.")


def mode_replay(path, args):
    print(f"\nReplaying {path.name} at {args.speed}× speed → {args.api_url}")
    print("Press Ctrl-C to stop.\n")

    bms          = BMS()
    cells_seen   = set()
    prev_real    = None
    prev_log_ms  = None
    snap_count   = 0

    for ts_ms, can_id, data in iter_frames(path):
        func = (can_id >> 16) & 0xFF

        now = time.monotonic()
        if prev_real is not None and prev_log_ms is not None:
            log_delta  = (ts_ms - prev_log_ms) / 1000.0
            real_delay = log_delta / args.speed
            elapsed    = now - prev_real
            if real_delay > elapsed:
                time.sleep(real_delay - elapsed)

        prev_real   = time.monotonic()
        prev_log_ms = ts_ms

        bms.decode(can_id, data)

        if 0xC8 <= func <= 0xCC:
            cells_seen.add(func)
            if cells_seen >= {0xC8, 0xC9, 0xCA, 0xCB, 0xCC}:
                cells_seen = set()
                snap_count += 1
                payload = bms.to_json(ts=datetime.now().isoformat())
                batt = payload.get("battery", {})
                print(f"[{ts_ms:8d}ms]  snap #{snap_count:3d}  "
                      f"SOC {batt.get('soc', '—')}%  "
                      f"Pack {batt.get('pack_v', 0):.2f} V  "
                      f"Cells {batt.get('cell_count', 0)}")
                post_vps(payload, args.api_url, args.api_key)

    print(f"\nReplay complete — {snap_count} snapshots sent.")


# ── Summary printer ───────────────────────────────────────────────────────────
def _print_summary(p):
    batt = p.get("battery", {})
    temps = p.get("temperatures", {})
    print(f"\n  {'='*50}")
    print(f"  SOC           : {batt.get('soc')} %   (BMS: {batt.get('soc_bms')} %)")
    print(f"  Pack voltage  : {batt.get('pack_v')} V")
    print(f"  Cells decoded : {batt.get('cell_count')}   avg {batt.get('cell_avg_mv')} mV   "
          f"spread {batt.get('cell_spread_mv')} mV")
    print(f"  Avg temp      : {temps.get('battery_avg_c')} °C")
    print(f"  Disch limit   : {batt.get('max_disch_a')} A")
    print(f"  Frames parsed : {p.get('frame_count')}")
    print(f"  {'='*50}")


# ── CLI ───────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="Parse a BMS CAN log and POST to /api/ingest",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("file", nargs="?", default=str(DEFAULT_LOG),
                    help=f"CAN log file (default: {DEFAULT_LOG.name})")
    ap.add_argument("--mode", choices=["snapshot", "replay"], default="snapshot",
                    help="snapshot: send final state once  |  replay: stream live")
    ap.add_argument("--speed", type=float, default=10.0,
                    help="Replay speed multiplier (default: 10)")
    ap.add_argument("--api-url", default=DEFAULT_API_URL,
                    help=f"Backend ingest URL (default: {DEFAULT_API_URL})")
    ap.add_argument("--api-key", default=DEFAULT_API_KEY,
                    help="Backend API key (default: bako-bms-2024)")

    args = ap.parse_args()
    path = Path(args.file)
    if not path.exists():
        ap.error(f"File not found: {path}")

    if args.mode == "replay":
        mode_replay(path, args)
    else:
        mode_snapshot(path, args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nStopped by user.")
