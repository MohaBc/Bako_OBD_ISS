#!/usr/bin/env python3
"""
battery_can_parser.py — Offline O'CELL BMS CAN log parser
==========================================================
Parses raw CAN log files captured by the ESP32 logger and prints a
human-readable battery state summary.  Works with both MCP2515 serial
output and the normalised [Nms] format produced by server.py.

Usage:
    python battery_can_parser.py                         # reads raw/batterie.txt
    python battery_can_parser.py path/to/log.txt
    python battery_can_parser.py path/to/log.txt --csv   # CSV to stdout
"""

import re, sys, argparse
from pathlib import Path

DEFAULT_LOG = Path(__file__).parent / "raw" / "batterie.txt"

FRAME_RE = re.compile(
    r'(?:\[\d+ms\]\s+)?'
    r'(?:Extended\s+)?ID:\s+(0x[0-9A-Fa-f]+)\s+'
    r'DLC:\s+(\d+)\s+'
    r'Data:\s+([\s0-9A-Fa-fx]+)',
    re.IGNORECASE,
)

# ── Byte helpers ─────────────────────────────────────────────────────────────

def u16be(d, o): return (d[o] << 8) | d[o + 1]
def u16le(d, o): return d[o] | (d[o + 1] << 8)

# ── SOC calibration ──────────────────────────────────────────────────────────

CELL_UV        = 2500
SOC_CAR_TOP_MV = 3387


def cell_status(mv):
    if mv >= 3750: return "OVERVOLTAGE"
    if mv >= 3650: return "FULL"
    if mv >= 3300: return "GOOD"
    if mv >= 3200: return "NORMAL"
    if mv >= 2500: return "LOW"
    return "UNDERVOLTAGE"


# ── Parser ────────────────────────────────────────────────────────────────────

def parse_file(path):
    cell_mv       = {}
    temp_c        = {}
    status_flags  = 0
    soc_bms_raw   = 0xFF
    pack_current  = None
    pack_v_bms    = None
    fault_level   = 0
    error_code    = 0
    max_disch_a   = None
    temp_max_c    = None
    temp_min_c    = None
    charge_max_v  = None
    charge_max_a  = None
    charger_start = False
    frames        = 0

    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            m = FRAME_RE.search(line)
            if not m:
                continue

            can_id = int(m.group(1), 16)
            dlc    = int(m.group(2))
            raw_bytes = [
                b.replace("0x", "").replace("0X", "")
                for b in m.group(3).split()
                if b.strip()
            ]
            try:
                data = bytes(int(b, 16) for b in raw_bytes if b)
            except ValueError:
                continue
            if len(data) < dlc:
                continue

            func = (can_id >> 16) & 0xFF
            sub  = (can_id >>  8) & 0xFF
            frames += 1

            if 0xC8 <= func <= 0xCC and dlc == 8:
                group = func - 0xC8
                base  = group * 4 + 1
                for i in range(4):
                    o = i * 2
                    if o + 1 < len(data):
                        mv = u16be(data, o)
                        if mv != 0 and base + i <= 19:
                            cell_mv[base + i] = mv

            elif func == 0xB4 and dlc == 8:
                for i in range(4):
                    v = data[i]
                    if v not in (0x00, 0xFF):
                        temp_c[i + 1] = float(v) - 40.0

            elif func == 0xFF and sub == 0x28 and len(data) >= 8:
                status_flags = data[0]
                soc_bms_raw  = data[1]
                pack_current = round((u16le(data, 2) - 5000) * 0.1, 1)
                pack_v_bms   = round(u16le(data, 4) * 0.1, 1)
                fault_level  = data[6]
                error_code   = data[7]

            elif func == 0xFF and sub == 0xE5 and len(data) >= 5:
                charge_max_v  = round(u16le(data, 0) * 0.1, 1)
                charge_max_a  = round(u16le(data, 2) * 0.1, 1)
                charger_start = not bool(data[4] & 0x01)

            elif func == 0xFE and sub == 0x28 and len(data) >= 8:
                if data[4] != 0xFF:
                    temp_max_c = float(data[4]) - 40.0
                if data[5] != 0xFF:
                    temp_min_c = float(data[5]) - 40.0
                max_disch_a = round(u16le(data, 6) * 0.1, 1)

    cv = cell_mv
    avg_mv = round(sum(cv.values()) / len(cv)) if cv else None
    pack_v_cells = round(sum(cv.values()) / 1000.0, 2) if len(cv) >= 5 else None
    pack_v = pack_v_bms if pack_v_bms and pack_v_bms > 0 else pack_v_cells

    soc_v = None
    if avg_mv:
        s = (avg_mv - CELL_UV) / (SOC_CAR_TOP_MV - CELL_UV) * 100
        soc_v = round(max(0.0, min(100.0, s)), 1)

    cell_min = min(cv.values()) if cv else None
    cell_max = max(cv.values()) if cv else None
    spread   = (cell_max - cell_min) if (cell_max and cell_min) else None

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
        t = temp_c.get(i)
        temp_arr.append(t if t is not None else None)

    avg_temp = round(sum(temp_c.values()) / len(temp_c), 1) if temp_c else None

    return {
        "frames_parsed": frames,
        "fault_level":   int(fault_level),
        "error_code":    int(error_code),

        "battery": {
            "pack_v":         pack_v,
            "pack_current_a": pack_current,
            "soc":            soc_v,
            "soc_bms":        int(soc_bms_raw) if soc_bms_raw <= 100 else None,
            "max_disch_a":    max_disch_a,
            "cell_count":     len(cv),
            "cell_avg_mv":    avg_mv,
            "cell_min_mv":    cell_min,
            "cell_max_mv":    cell_max,
            "cell_spread_mv": spread,
            "cells":          cells_arr,
            "charger": {
                "max_charge_v": charge_max_v,
                "max_charge_a": charge_max_a,
                "start_signal": charger_start,
            },
            "status": {
                "charge_cable":     bool(status_flags & 0x01),
                "charging":         bool(status_flags & 0x02),
                "discharging":      bool(status_flags & 0x04),
                "ready":            bool(status_flags & 0x08),
                "disch_contactor":  bool(status_flags & 0x10),
                "charge_contactor": bool(status_flags & 0x20),
            },
        },

        "temperatures": {
            "motor_c":       None,
            "mppt_c":        None,
            "cabin_c":       None,
            "battery_avg_c": avg_temp,
            "battery_min_c": temp_min_c,
            "battery_max_c": temp_max_c,
            "battery_cells": temp_arr,
        },

        "solar": {
            "pre_mppt":  {"voltage_v": None, "current_a": None},
            "post_mppt": {"current_a": None},
        },

        "dc_dc": {
            "output_64v": {"voltage_v": None},
            "output_12v": {"voltage_v": None},
        },

        "vehicle": {"handbrake": None},
    }


# ── Output formatters ─────────────────────────────────────────────────────────

def print_report(data, path):
    batt  = data.get("battery", {})
    temps = data.get("temperatures", {})
    solar = data.get("solar", {})
    dcdc  = data.get("dc_dc", {})
    veh   = data.get("vehicle", {})
    status = batt.get("status", {})
    charger = batt.get("charger", {})

    cv = {}
    for i, entry in enumerate(batt.get("cells", [])):
        if entry and i > 0:
            cv[i] = entry["mv"]

    print(f"\n{'='*60}")
    print("  O'CELL BMS — Offline CAN Log Analysis")
    print(f"  File   : {path}")
    print(f"  Frames : {data['frames_parsed']}")
    print(f"{'='*60}")

    print("\n  PACK SUMMARY")
    print(f"  ├─ Pack voltage   : {batt.get('pack_v')} V")
    print(f"  ├─ Pack current   : {batt.get('pack_current_a')} A")
    print(f"  ├─ SOC (voltage)  : {batt.get('soc')} %")
    print(f"  ├─ SOC (BMS)      : {batt.get('soc_bms')} %")
    print(f"  ├─ Avg cell       : {batt.get('cell_avg_mv')} mV")
    print(f"  ├─ Max cell       : {batt.get('cell_max_mv')} mV")
    print(f"  ├─ Min cell       : {batt.get('cell_min_mv')} mV")
    print(f"  ├─ Cell spread    : {batt.get('cell_spread_mv')} mV")
    print(f"  ├─ Disch limit    : {batt.get('max_disch_a')} A")
    print(f"  ├─ Fault level    : {data.get('fault_level')}   code: {data.get('error_code')}")
    print(f"  └─ Status         : "
          f"charging={status.get('charging')}  "
          f"discharging={status.get('discharging')}  "
          f"ready={status.get('ready')}")

    print("\n  CHARGER")
    print(f"  ├─ Max charge V   : {charger.get('max_charge_v')} V")
    print(f"  ├─ Max charge A   : {charger.get('max_charge_a')} A")
    print(f"  └─ Start signal   : {charger.get('start_signal')}")

    print("\n  TEMPERATURES")
    t_cells = temps.get("battery_cells", [])
    active = [(i, t_cells[i]) for i in range(1, len(t_cells)) if t_cells[i] is not None]
    if active:
        for i, (idx, val) in enumerate(active):
            prefix = "└─" if i == len(active) - 1 else "├─"
            print(f"  {prefix} Probe {idx}: {val:.1f} °C")
        print(f"  avg={temps.get('battery_avg_c')} °C  "
              f"min={temps.get('battery_min_c')} °C  "
              f"max={temps.get('battery_max_c')} °C")
    else:
        print("  └─ No temperature data")

    print("\n  SOLAR  (stub — not yet instrumented)")
    pre = solar.get("pre_mppt", {})
    post = solar.get("post_mppt", {})
    print(f"  ├─ Pre-MPPT  V={pre.get('voltage_v')}  I={pre.get('current_a')}")
    print(f"  └─ Post-MPPT I={post.get('current_a')}")

    print("\n  DC/DC CONVERTERS  (stub — not yet instrumented)")
    print(f"  ├─ 64V output: {dcdc.get('output_64v', {}).get('voltage_v')} V")
    print(f"  └─ 12V output: {dcdc.get('output_12v', {}).get('voltage_v')} V")

    print("\n  VEHICLE  (stub — not yet instrumented)")
    print(f"  └─ Handbrake: {veh.get('handbrake')}")

    print(f"\n  CELL VOLTAGES ({len(cv)} cells decoded)")
    BAR_MAX = 24
    for k in sorted(cv):
        mv  = cv[k]
        pct = max(0.0, min(1.0, (mv - 2500) / (3650 - 2500)))
        bar = ("█" * int(pct * BAR_MAX)).ljust(BAR_MAX)
        st  = cell_status(mv)
        print(f"  C{k:02d}  {mv:4d} mV  [{bar}]  {st}")

    print(f"\n{'='*60}\n")


def print_csv(data, path):
    batt  = data.get("battery", {})
    temps = data.get("temperatures", {})
    print("# O'CELL BMS offline parse —", path)
    print("cell,mv,status")
    for entry in batt.get("cells", []):
        if entry:
            print(f"{entry['mv']},{cell_status(entry['mv'])}")
    print()
    print("metric,value")
    print(f"pack_v,{batt.get('pack_v')}")
    print(f"pack_current_a,{batt.get('pack_current_a')}")
    print(f"avg_cell_mv,{batt.get('cell_avg_mv')}")
    print(f"soc_voltage_pct,{batt.get('soc')}")
    print(f"soc_bms_pct,{batt.get('soc_bms')}")
    print(f"cell_spread_mv,{batt.get('cell_spread_mv')}")
    print(f"max_disch_a,{batt.get('max_disch_a')}")
    print(f"fault_level,{data.get('fault_level')}")
    print(f"error_code,{data.get('error_code')}")
    t_cells = temps.get("battery_cells", [])
    for i in range(1, len(t_cells)):
        if t_cells[i] is not None:
            print(f"temp_{i}_c,{t_cells[i]:.1f}")
    print(f"battery_avg_c,{temps.get('battery_avg_c')}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Offline O'CELL BMS CAN log parser")
    ap.add_argument("file", nargs="?", default=str(DEFAULT_LOG),
                    help=f"CAN log file (default: {DEFAULT_LOG})")
    ap.add_argument("--csv", action="store_true",
                    help="Output CSV to stdout instead of a report")
    args = ap.parse_args()

    path = Path(args.file)
    if not path.exists():
        print(f"ERROR: file not found: {path}", file=sys.stderr)
        sys.exit(1)

    data = parse_file(path)

    if args.csv:
        print_csv(data, path)
    else:
        print_report(data, path)


if __name__ == "__main__":
    main()
