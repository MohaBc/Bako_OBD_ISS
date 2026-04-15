#!/usr/bin/env python3
"""
send_log_to_cloud.py — Replay a BMS CAN log to the cloud
=========================================================
Parses a raw CAN log file and sends the BMS state as JSON to:
  • The local FastAPI backend  POST /api/ingest
  • Firebase Realtime Database  PUT /bms/live.json  (+ POST /bms/history.json)

Modes
─────
  snapshot   Parse the whole file → send one final snapshot  (default)
  replay     Stream snapshots in time order → animates the dashboard live

Usage
─────
  # Quick snapshot → local backend
  python send_log_to_cloud.py

  # Snapshot → Firebase
  python send_log_to_cloud.py --target firebase \
      --firebase-url https://YOUR_PROJECT-default-rtdb.firebaseio.com \
      --firebase-token YOUR_TOKEN

  # Live replay at 20× speed → local backend
  python send_log_to_cloud.py --mode replay --speed 20

  # Replay → Firebase
  python send_log_to_cloud.py --mode replay --speed 10 --target firebase \
      --firebase-url https://YOUR_PROJECT-default-rtdb.firebaseio.com \
      --firebase-token YOUR_TOKEN

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
        self.cell_mv   = {}
        self.temp_c    = {}
        self.soc_coul  = None
        self.soc_bms   = None
        self.chg_req   = None
        self.disch_lim = None
        self.cell_max  = None
        self.cell_min  = None
        self.frames    = 0

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
                    if mv:
                        self.cell_mv[base + i] = mv

        elif func == 0xB4 and len(data) >= 4:
            self.temp_c = {}
            for i in range(4):
                v = data[i]
                if v not in (0x00, 0xFF):
                    self.temp_c[i + 1] = round(float(v) - 40.0, 1)

        elif func == 0xFF and sub == 0xE5 and len(data) >= 4:
            self.soc_coul = u16le(data, 0) / 10.0
            self.chg_req  = u16le(data, 2) / 10.0

        elif func == 0xFF and sub == 0x28 and len(data) >= 6:
            self.disch_lim = u16le(data, 2) / 100.0
            self.soc_bms   = u16le(data, 4) / 10.0

        elif func == 0xFE and sub == 0x28 and len(data) >= 8:
            self.cell_max = u16le(data, 0)
            self.cell_min = u16le(data, 2)
            for i in range(2):
                v = data[4 + i]
                if v not in (0x00, 0xFF):
                    self.temp_c[i + 1] = round(float(v) - 40.0, 1)
            if self.disch_lim is None:
                self.disch_lim = u16le(data, 6) / 10.0

    def complete(self):
        """True once we have at least one complete cell voltage frame."""
        return len(self.cell_mv) >= 4

    def to_json(self, ts=None, log_file=""):
        cv = dict(self.cell_mv)
        avg_mv = round(sum(cv.values()) / len(cv)) if cv else None
        pack_v = round(sum(cv.values()) / 1000.0, 2) if len(cv) >= 5 else None

        soc_v = None
        if avg_mv:
            s = (avg_mv - CELL_UV) / (SOC_CAR_TOP_MV - CELL_UV) * 100
            soc_v = round(max(0.0, min(100.0, s)), 1)

        soc_disp = soc_v if soc_v is not None else self.soc_coul

        cells_out = {
            str(k): {"mv": cv[k], "status": cell_status(cv[k])}
            for k in sorted(cv)
        }

        return {
            "connected":      True,
            "source":         "log-replay",
            "device_id":      Path(log_file).stem if log_file else "log",
            "timestamp":      ts or datetime.now().isoformat(),
            "frame_count":    self.frames,

            "soc":            soc_disp,
            "soc_coulomb":    round(self.soc_coul, 1) if self.soc_coul is not None else None,
            "soc_bms":        round(self.soc_bms,  1) if self.soc_bms  is not None else None,

            "pack_v":         pack_v,
            "chg_i_req":      round(self.chg_req,   1) if self.chg_req   is not None else None,
            "disch_i_lim":    round(self.disch_lim, 1) if self.disch_lim is not None else None,

            "cells":          cells_out,
            "cell_count":     len(cv),
            "cell_max_mv":    self.cell_max,
            "cell_min_mv":    self.cell_min,
            "cell_spread_mv": (max(cv.values()) - min(cv.values())) if len(cv) > 1 else None,
            "cell_avg_mv":    avg_mv,

            "temps":          {str(k): v for k, v in sorted(self.temp_c.items())},
            "avg_temp":       round(sum(self.temp_c.values()) / len(self.temp_c), 1)
                              if self.temp_c else None,
        }


# ── Frame iterator ────────────────────────────────────────────────────────────
def iter_frames(path):
    """Yield (timestamp_ms, can_id, data_bytes) for each valid CAN frame."""
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = FRAME_RE.search(line)
            if not m:
                continue
            ts_ms  = int(m.group(1))
            can_id = int(m.group(2), 16)
            data   = bytes(int(b, 16) for b in m.group(4).split())
            yield ts_ms, can_id, data


# ── HTTP senders ──────────────────────────────────────────────────────────────
def post_local(payload: dict, api_url: str, api_key: str) -> bool:
    body = json.dumps(payload).encode()
    req  = urllib.request.Request(
        api_url, data=body,
        headers={"Content-Type": "application/json", "X-Api-Key": api_key},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            ok = r.status == 200
            print(f"  [LOCAL] POST {r.status} → {api_url}")
            return ok
    except urllib.error.HTTPError as e:
        print(f"  [LOCAL] HTTP {e.code}: {e.read().decode()[:120]}")
    except Exception as e:
        print(f"  [LOCAL] Error: {e}")
    return False


def put_firebase(payload: dict, fb_url: str, fb_token: str) -> bool:
    """PUT to /bms/live.json  (always overwrites the live snapshot)."""
    url  = f"{fb_url.rstrip('/')}/bms/live.json?auth={fb_token}"
    body = json.dumps(payload).encode()
    req  = urllib.request.Request(
        url, data=body,
        headers={"Content-Type": "application/json"},
        method="PUT",
    )
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            print(f"  [FIREBASE] PUT {r.status} → /bms/live")
            return r.status == 200
    except urllib.error.HTTPError as e:
        print(f"  [FIREBASE] HTTP {e.code}: {e.read().decode()[:120]}")
    except Exception as e:
        print(f"  [FIREBASE] Error: {e}")
    return False


def post_firebase_history(payload: dict, fb_url: str, fb_token: str):
    """POST to /bms/history.json  (appends with auto-key — best effort)."""
    url  = f"{fb_url.rstrip('/')}/bms/history.json?auth={fb_token}"
    body = json.dumps(payload).encode()
    req  = urllib.request.Request(
        url, data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=8):
            pass
    except Exception:
        pass


def send(payload: dict, args):
    if args.target in ("local", "both"):
        post_local(payload, args.api_url, args.api_key)
    if args.target in ("firebase", "both"):
        ok = put_firebase(payload, args.firebase_url, args.firebase_token)
        if ok:
            post_firebase_history(payload, args.firebase_url, args.firebase_token)


# ── Modes ─────────────────────────────────────────────────────────────────────
def mode_snapshot(path, args):
    """Parse entire file → send one final snapshot."""
    print(f"\nParsing {path.name} ...")
    bms = BMS()
    for ts_ms, can_id, data in iter_frames(path):
        bms.decode(can_id, data)

    if not bms.complete():
        print("No cell voltage frames found — check the log file.")
        sys.exit(1)

    payload = bms.to_json(log_file=str(path))
    _print_summary(payload)
    print(f"\nSending snapshot to [{args.target}] ...")
    send(payload, args)
    print("\nDone.")


def mode_replay(path, args):
    """
    Stream snapshots as the log is read, preserving relative timing.
    A new snapshot is emitted every time a complete cell-voltage sweep
    (all groups 0xC8–0xCC) has been received.
    """
    print(f"\nReplaying {path.name} at {args.speed}× speed → [{args.target}]")
    print("Press Ctrl-C to stop.\n")

    bms          = BMS()
    cells_seen   = set()      # groups received in current sweep (0–4)
    prev_real    = None
    prev_log_ms  = None
    snap_count   = 0

    for ts_ms, can_id, data in iter_frames(path):
        func = (can_id >> 16) & 0xFF

        # Track inter-frame wall-clock delay
        now = time.monotonic()
        if prev_real is not None and prev_log_ms is not None:
            log_delta  = (ts_ms - prev_log_ms) / 1000.0   # seconds in log time
            real_delay = log_delta / args.speed
            elapsed    = now - prev_real
            if real_delay > elapsed:
                time.sleep(real_delay - elapsed)

        prev_real   = time.monotonic()
        prev_log_ms = ts_ms

        bms.decode(can_id, data)

        # Track completed cell sweeps
        if 0xC8 <= func <= 0xCC:
            cells_seen.add(func)
            if cells_seen >= {0xC8, 0xC9, 0xCA, 0xCB, 0xCC}:
                cells_seen = set()
                snap_count += 1
                ts_iso = datetime.now().isoformat()
                payload = bms.to_json(ts=ts_iso, log_file=str(path))
                print(f"[{ts_ms:8d}ms]  snap #{snap_count:3d}  "
                      f"SOC {payload.get('soc', '—'):.1f}%  "
                      f"Pack {payload.get('pack_v', 0):.2f} V  "
                      f"Cells {payload.get('cell_count', 0)}")
                send(payload, args)

    print(f"\nReplay complete — {snap_count} snapshots sent.")


# ── Summary printer ───────────────────────────────────────────────────────────
def _print_summary(p):
    print(f"\n  {'='*50}")
    print(f"  SOC           : {p.get('soc')} %   (coulomb: {p.get('soc_coulomb')} %  BMS: {p.get('soc_bms')} %)")
    print(f"  Pack voltage  : {p.get('pack_v')} V")
    print(f"  Cells decoded : {p.get('cell_count')}   avg {p.get('cell_avg_mv')} mV   "
          f"spread {p.get('cell_spread_mv')} mV")
    print(f"  Temperatures  : {p.get('temps')}")
    print(f"  Disch limit   : {p.get('disch_i_lim')} A    Chg req: {p.get('chg_i_req')} A")
    print(f"  Frames parsed : {p.get('frame_count')}")
    print(f"  {'='*50}")


# ── CLI ───────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(
        description="Parse a BMS CAN log and send to the cloud",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    ap.add_argument("file", nargs="?", default=str(DEFAULT_LOG),
                    help=f"CAN log file (default: {DEFAULT_LOG.name})")
    ap.add_argument("--mode", choices=["snapshot", "replay"], default="snapshot",
                    help="snapshot: send final state once  |  replay: stream live (default: snapshot)")
    ap.add_argument("--target", choices=["local", "firebase", "both"], default="local",
                    help="Where to send data (default: local)")
    ap.add_argument("--speed", type=float, default=10.0,
                    help="Replay speed multiplier (default: 10)")

    # Local backend
    ap.add_argument("--api-url", default=DEFAULT_API_URL,
                    help=f"Backend ingest URL (default: {DEFAULT_API_URL})")
    ap.add_argument("--api-key", default=DEFAULT_API_KEY,
                    help="Backend API key (default: bako-bms-2024)")

    # Firebase
    ap.add_argument("--firebase-url", default="",
                    help="Firebase Realtime DB URL  e.g. https://proj-rtdb.firebaseio.com")
    ap.add_argument("--firebase-token", default="",
                    help="Firebase database secret / legacy token")

    args = ap.parse_args()

    # Validate Firebase args
    if args.target in ("firebase", "both"):
        if not args.firebase_url or not args.firebase_token:
            ap.error("--firebase-url and --firebase-token are required when --target is firebase/both")

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
