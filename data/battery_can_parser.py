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

# Matches both [Nms] ID: 0xXXXXXXXX  and  Extended ID: 0xXXXXXXXX
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

# ── SOC calibration (mirrors server.py) ──────────────────────────────────────

CELL_UV        = 2500   # mV  0% reference
SOC_CAR_TOP_MV = 3387   # mV  100% reference (car-display calibrated)


def cell_status(mv):
    if mv >= 3750: return "OVERVOLTAGE"
    if mv >= 3650: return "FULL"
    if mv >= 3300: return "GOOD"
    if mv >= 3200: return "NORMAL"
    if mv >= 2500: return "LOW"
    return "UNDERVOLTAGE"


# ── Parser ────────────────────────────────────────────────────────────────────

def parse_file(path):
    cell_mv   = {}
    temp_c    = {}
    soc_coul  = None
    soc_bms   = None
    chg_req   = None
    disch_lim = None
    cell_max  = None
    cell_min  = None
    frames    = 0

    with open(path, encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            m = FRAME_RE.search(line)
            if not m:
                continue

            can_id = int(m.group(1), 16)
            dlc    = int(m.group(2))

            # Normalise data bytes (strip optional 0x prefix)
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

            # Cell voltages: func 0xC8–0xCC, big-endian uint16 mV
            if 0xC8 <= func <= 0xCC and dlc == 8:
                group = func - 0xC8
                base  = group * 4 + 1
                for i in range(4):
                    o = i * 2
                    if o + 1 < len(data):
                        mv = u16be(data, o)
                        if mv != 0:
                            cell_mv[base + i] = mv

            # Temperatures: func 0xB4, raw − 40 = °C, 0xFF = absent
            elif func == 0xB4 and dlc == 8:
                for i in range(4):
                    v = data[i]
                    if v not in (0x00, 0xFF):
                        temp_c[i + 1] = float(v) - 40.0

            # SOC + charge request: func 0xFF sub 0xE5
            elif func == 0xFF and sub == 0xE5 and len(data) >= 4:
                soc_coul = u16le(data, 0) / 10.0
                chg_req  = u16le(data, 2) / 10.0

            # Pack summary: func 0xFF sub 0x28
            elif func == 0xFF and sub == 0x28 and len(data) >= 6:
                disch_lim = u16le(data, 2) / 100.0
                soc_bms   = u16le(data, 4) / 10.0

            # Min/Max + temps: func 0xFE sub 0x28
            elif func == 0xFE and sub == 0x28 and len(data) >= 8:
                cell_max = u16le(data, 0)
                cell_min = u16le(data, 2)
                for i in range(2):
                    v = data[4 + i]
                    if v not in (0x00, 0xFF):
                        temp_c[i + 1] = float(v) - 40.0
                if disch_lim is None:
                    disch_lim = u16le(data, 6) / 10.0

    return dict(
        cells=cell_mv, temps=temp_c,
        soc_coulomb=soc_coul, soc_bms=soc_bms,
        chg_req_a=chg_req, disch_lim_a=disch_lim,
        cell_max_mv=cell_max, cell_min_mv=cell_min,
        frames_parsed=frames,
    )


# ── Derived metrics ───────────────────────────────────────────────────────────

def derived(data):
    cv = data["cells"]
    avg_mv = round(sum(cv.values()) / len(cv)) if cv else None
    pack_v = round(sum(cv.values()) / 1000.0, 2) if len(cv) >= 5 else None
    soc_v  = None
    if avg_mv is not None:
        s = (avg_mv - CELL_UV) / (SOC_CAR_TOP_MV - CELL_UV) * 100
        soc_v = round(max(0.0, min(100.0, s)), 1)
    spread = (max(cv.values()) - min(cv.values())) if len(cv) > 1 else None
    return avg_mv, pack_v, soc_v, spread


# ── Output formatters ─────────────────────────────────────────────────────────

def print_report(data, path):
    avg_mv, pack_v, soc_v, spread = derived(data)
    cv = data["cells"]

    print(f"\n{'='*60}")
    print("  O'CELL BMS — Offline CAN Log Analysis")
    print(f"  File   : {path}")
    print(f"  Frames : {data['frames_parsed']}")
    print(f"{'='*60}")

    print("\n  PACK SUMMARY")
    print(f"  ├─ Pack voltage   : {pack_v} V")
    print(f"  ├─ SOC (voltage)  : {soc_v} %")
    print(f"  ├─ SOC (coulomb)  : {data['soc_coulomb']} %")
    print(f"  ├─ SOC (BMS)      : {data['soc_bms']} %")
    print(f"  ├─ Avg cell       : {avg_mv} mV")
    print(f"  ├─ Max cell       : {data['cell_max_mv']} mV")
    print(f"  ├─ Min cell       : {data['cell_min_mv']} mV")
    print(f"  ├─ Cell spread    : {spread} mV")
    print(f"  ├─ Chg request    : {data['chg_req_a']} A")
    print(f"  └─ Disch limit    : {data['disch_lim_a']} A")

    print("\n  TEMPERATURES")
    if data["temps"]:
        items = sorted(data["temps"].items())
        for i, (k, v) in enumerate(items):
            prefix = "└─" if i == len(items) - 1 else "├─"
            print(f"  {prefix} T{k}: {v:.1f} °C")
    else:
        print("  └─ No temperature data")

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
    avg_mv, pack_v, soc_v, _ = derived(data)
    print("# O'CELL BMS offline parse —", path)
    print("cell,mv,status")
    for k in sorted(data["cells"]):
        print(f"{k},{data['cells'][k]},{cell_status(data['cells'][k])}")
    print()
    print("metric,value")
    print(f"pack_v,{pack_v}")
    print(f"avg_cell_mv,{avg_mv}")
    print(f"soc_voltage_pct,{soc_v}")
    print(f"soc_coulomb_pct,{data['soc_coulomb']}")
    print(f"soc_bms_pct,{data['soc_bms']}")
    print(f"chg_req_a,{data['chg_req_a']}")
    print(f"disch_lim_a,{data['disch_lim_a']}")
    for k, v in sorted(data["temps"].items()):
        print(f"temp_{k}_c,{v:.1f}")


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
        print("Usage: python battery_can_parser.py [path/to/log.txt] [--csv]",
              file=sys.stderr)
        sys.exit(1)

    data = parse_file(path)

    if args.csv:
        print_csv(data, path)
    else:
        print_report(data, path)


if __name__ == "__main__":
    main()
