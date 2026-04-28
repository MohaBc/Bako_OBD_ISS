#!/usr/bin/env python3
"""
One-shot migration: SQLite bms_cloud.db → MariaDB telemetry table.

Usage:
    python migrate_sqlite_to_mariadb.py [--dry-run] [--sqlite PATH] [--device-id ID]

Requires .env in the project root (or env vars):
    DB_HOST, DB_PORT, DB_NAME, DB_USER, DB_PASS
"""

import argparse
import json
import os
import sqlite3
import sys
from datetime import datetime, timezone
from pathlib import Path

# ── optional dotenv ──────────────────────────────────────────────────────────
try:
    from dotenv import load_dotenv
    _env = Path(__file__).resolve().parents[2] / ".env"
    if _env.exists():
        load_dotenv(_env)
except ImportError:
    pass

DEFAULT_SQLITE = Path(__file__).resolve().parents[1] / "bms_cloud.db"
DEFAULT_DEVICE = "ocell-001"


def _mariadb_conn():
    import pymysql  # pip install pymysql
    return pymysql.connect(
        host=os.environ["DB_HOST"],
        port=int(os.environ.get("DB_PORT", 3306)),
        database=os.environ["DB_NAME"],
        user=os.environ["DB_USER"],
        password=os.environ["DB_PASS"],
        charset="utf8mb4",
        autocommit=False,
    )


def _parse_ts(raw: str) -> str:
    """Normalise SQLite timestamp to MariaDB DATETIME(3) string."""
    raw = raw.strip()
    for fmt in ("%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S",
                "%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
        try:
            dt = datetime.strptime(raw, fmt).replace(tzinfo=timezone.utc)
            return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # trim to ms
        except ValueError:
            continue
    # fallback — store as-is and let MariaDB reject bad values
    return raw


def _upgrade_payload(payload_str: str, device_id: str) -> str:
    """
    Best-effort upgrade of a v0.6.0 payload to v2.0.0 shape.

    Old shape (flat or nested-but-old):
      cells: [null, {mv,status}, ...]   len 20
      temperatures.battery_cells: [null, float, ...]   len 5

    New shape:
      cells: [{index,mv,status}, ...]   len 19
      temperatures.battery_cells: [{index,c}, ...]   len 4
    """
    try:
        d = json.loads(payload_str)
    except json.JSONDecodeError:
        return payload_str

    # Inject envelope fields if missing
    d.setdefault("schema_version", "2.0.0")
    d.setdefault("device_id", device_id)

    batt = d.get("battery", {})

    # ── cells array ─────────────────────────────────────────────────────────
    old_cells = batt.get("cells", [])
    if old_cells and (old_cells[0] is None or not isinstance(old_cells[0], dict)
                      or "index" not in old_cells[0]):
        # strip leading null, re-index
        new_cells = []
        src = old_cells[1:] if (old_cells and old_cells[0] is None) else old_cells
        for i, entry in enumerate(src[:19], start=1):
            if entry and isinstance(entry, dict):
                new_cells.append({"index": i, "mv": entry.get("mv"), "status": entry.get("status", "unknown")})
            else:
                new_cells.append({"index": i, "mv": None, "status": "unknown"})
        # pad to 19 if short
        while len(new_cells) < 19:
            new_cells.append({"index": len(new_cells) + 1, "mv": None, "status": "unknown"})
        batt["cells"] = new_cells
        d["battery"] = batt

    # ── battery_cells array ─────────────────────────────────────────────────
    temps = d.get("temperatures", {})
    old_tc = temps.get("battery_cells", [])
    if old_tc and (old_tc[0] is None or not isinstance(old_tc[0], dict)
                   or "index" not in old_tc[0]):
        src = old_tc[1:] if (old_tc and old_tc[0] is None) else old_tc
        new_tc = []
        for i, val in enumerate(src[:4], start=1):
            c = val if isinstance(val, (int, float)) else None
            new_tc.append({"index": i, "c": c})
        while len(new_tc) < 4:
            new_tc.append({"index": len(new_tc) + 1, "c": None})
        temps["battery_cells"] = new_tc
        d["temperatures"] = temps

    return json.dumps(d, separators=(",", ":"))


def migrate(sqlite_path: Path, device_id: str, dry_run: bool) -> None:
    src = sqlite3.connect(sqlite_path)
    src.row_factory = sqlite3.Row
    rows = src.execute(
        "SELECT id, ts, device_id, payload FROM snapshots ORDER BY id"
    ).fetchall()
    src.close()

    print(f"Found {len(rows)} rows in {sqlite_path}")
    if not rows:
        print("Nothing to migrate.")
        return

    if dry_run:
        print("-- DRY RUN: showing first 3 converted rows --")
        for row in rows[:3]:
            dev = row["device_id"] or device_id
            ts  = _parse_ts(row["ts"])
            pay = _upgrade_payload(row["payload"], dev)
            print(f"  id={row['id']}  ts={ts}  device={dev}")
            print(f"  payload={pay[:120]}...")
        print(f"-- DRY RUN complete. {len(rows)} rows would be inserted. --")
        return

    dst = _mariadb_conn()
    cur = dst.cursor()

    inserted = 0
    skipped  = 0
    batch    = []

    for row in rows:
        dev = row["device_id"] or device_id
        ts  = _parse_ts(row["ts"])
        pay = _upgrade_payload(row["payload"], dev)
        batch.append((dev, ts, "2.0.0", "http", pay))

        if len(batch) >= 200:
            cur.executemany(
                "INSERT INTO telemetry (vehicle_id, ts, schema_version, transport, payload) "
                "VALUES (%s, %s, %s, %s, %s)",
                batch,
            )
            inserted += len(batch)
            batch = []
            print(f"  inserted {inserted}/{len(rows)} ...", end="\r", flush=True)

    if batch:
        cur.executemany(
            "INSERT INTO telemetry (vehicle_id, ts, schema_version, transport, payload) "
            "VALUES (%s, %s, %s, %s, %s)",
            batch,
        )
        inserted += len(batch)

    dst.commit()
    cur.close()
    dst.close()

    print(f"\nDone. {inserted} rows inserted, {skipped} skipped.")


def main() -> None:
    ap = argparse.ArgumentParser(description="Migrate SQLite snapshots → MariaDB telemetry")
    ap.add_argument("--sqlite",    default=str(DEFAULT_SQLITE), help="Path to bms_cloud.db")
    ap.add_argument("--device-id", default=DEFAULT_DEVICE,      help="Fallback device_id for rows with empty device_id")
    ap.add_argument("--dry-run",   action="store_true",         help="Parse and convert rows without writing to MariaDB")
    args = ap.parse_args()

    sqlite_path = Path(args.sqlite)
    if not sqlite_path.exists():
        print(f"Error: SQLite file not found: {sqlite_path}", file=sys.stderr)
        sys.exit(1)

    for var in ("DB_HOST", "DB_NAME", "DB_USER", "DB_PASS"):
        if not args.dry_run and not os.environ.get(var):
            print(f"Error: environment variable {var} is not set.", file=sys.stderr)
            sys.exit(1)

    migrate(sqlite_path, args.device_id, args.dry_run)


if __name__ == "__main__":
    main()
