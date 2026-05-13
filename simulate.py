#!/usr/bin/env python3
"""
BAKO BMS — Local simulation launcher
Pipes the captured CAN log into the server over a virtual serial port,
then opens the dashboard in the browser.

Usage:
    python simulate.py            # 1× real-time speed
    python simulate.py --speed 5  # 5× faster
    python simulate.py --no-browser
"""

import argparse, os, re, subprocess, sys, time, webbrowser
from pathlib import Path

ROOT    = Path(__file__).parent
LOG     = ROOT / "data" / "raw" / "bms_log_2026-03-10T10-20-02.txt"
SERVER  = ROOT / "backend" / "server_frame_parser_Json.py"
VENV_PY = ROOT / "backend" / "venv" / "bin" / "python"
PORT    = 8765

PYTHON  = str(VENV_PY) if VENV_PY.exists() else sys.executable
TS_RE   = re.compile(r'^\[(\d+)ms\]')


def create_pty_pair():
    """Return (server_pty, feeder_pty) paths using socat."""
    import tempfile, atexit
    s_link = "/tmp/bako_sim_server"
    f_link = "/tmp/bako_sim_feeder"
    for p in (s_link, f_link):
        try: os.unlink(p)
        except FileNotFoundError: pass

    proc = subprocess.Popen(
        ["socat", "-d", "-d",
         f"pty,raw,echo=0,link={s_link}",
         f"pty,raw,echo=0,link={f_link}"],
        stderr=subprocess.DEVNULL,
    )
    atexit.register(proc.terminate)
    # wait for symlinks to appear
    for _ in range(20):
        if os.path.exists(s_link) and os.path.exists(f_link):
            return s_link, f_link
        time.sleep(0.1)
    raise RuntimeError("socat did not create PTY links")


def feed_log(feeder_pty: str, log_path: Path, speed: float, loop: bool):
    """Read log file and write lines to the feeder end of the PTY."""
    lines = log_path.read_text(encoding="utf-8").splitlines()
    print(f"[SIM] {len(lines)} frames  speed={speed}×  loop={loop}")

    run = True
    while run:
        prev_ts = None
        with open(feeder_pty, "wb", buffering=0) as f:
            for line in lines:
                m = TS_RE.match(line)
                if m:
                    ts = int(m.group(1))
                    if prev_ts is not None and ts > prev_ts:
                        delay = (ts - prev_ts) / 1000.0 / speed
                        if delay > 0:
                            time.sleep(delay)
                    prev_ts = ts
                f.write((line + "\n").encode())
        if not loop:
            run = False
        else:
            print("[SIM] loop restarting")
            time.sleep(0.5)
    print("[SIM] done")


def main():
    p = argparse.ArgumentParser(description="BAKO local BMS simulation")
    p.add_argument("--speed",       "-s", type=float, default=2.0,
                   help="Replay speed multiplier (default 2)")
    p.add_argument("--loop",        "-l", action="store_true", default=True,
                   help="Loop the log file (default: on)")
    p.add_argument("--no-loop",     dest="loop", action="store_false")
    p.add_argument("--no-browser",  action="store_true",
                   help="Don't open browser automatically")
    args = p.parse_args()

    if not LOG.exists():
        sys.exit(f"[ERR] Log file not found: {LOG}")

    # ── 1. virtual serial port pair ──────────────────────────────────────────
    print("[SIM] Creating virtual serial port pair…")
    server_pty, feeder_pty = create_pty_pair()
    real_server_pty = os.path.realpath(server_pty)
    print(f"[SIM] PTY pair: {real_server_pty} ↔ {os.path.realpath(feeder_pty)}")

    # ── 2. start server ──────────────────────────────────────────────────────
    print(f"[SIM] Starting server on port {PORT}…")
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    server_proc = subprocess.Popen(
        [PYTHON, str(SERVER), "--web-port", str(PORT)],
        cwd=str(ROOT),
        env=env,
    )

    # wait for server to be ready
    import urllib.request, urllib.error
    for _ in range(30):
        try:
            urllib.request.urlopen(f"http://localhost:{PORT}/api/mode", timeout=1)
            break
        except Exception:
            time.sleep(0.3)
    else:
        server_proc.terminate()
        sys.exit("[ERR] Server did not start in time")
    print("[SIM] Server ready")

    # ── 3. point server at the virtual serial port ───────────────────────────
    import json, urllib.request
    body = json.dumps({
        "mode": "serial",
        "serial": {"port": real_server_pty, "baud": 115200}
    }).encode()
    req = urllib.request.Request(
        f"http://localhost:{PORT}/api/mode",
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    urllib.request.urlopen(req, timeout=5)
    print(f"[SIM] Server → serial mode on {real_server_pty}")
    time.sleep(0.5)

    # ── 4. open browser ──────────────────────────────────────────────────────
    if not args.no_browser:
        time.sleep(0.5)
        webbrowser.open(f"http://localhost:{PORT}")
        print(f"[SIM] Dashboard → http://localhost:{PORT}")

    # ── 5. stream frames ─────────────────────────────────────────────────────
    print(f"[SIM] Streaming frames from {LOG.name}…  (Ctrl+C to stop)")
    try:
        feed_log(feeder_pty, LOG, speed=args.speed, loop=args.loop)
    except KeyboardInterrupt:
        print("\n[SIM] Stopped")
    finally:
        server_proc.terminate()
        print("[SIM] Server stopped")


if __name__ == "__main__":
    main()
