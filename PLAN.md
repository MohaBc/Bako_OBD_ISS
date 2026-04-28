# Migration Plan — v3.0 (TCP + MariaDB + Unified Schema)

> Status: **AWAITING APPROVAL** — do not touch any source file until this is signed off.

---

## 0. What I found in the codebase

| File | Lines | Role |
|------|-------|------|
| `backend/server.py` | 599 | FastAPI serial + HTTP-cloud server (SQLite) |
| `backend/server_wifi.py` | 565 | Alternative TCP-listener server (port 9000) — this is the "collaborator's server" |
| `backend/replay_to_cloud.py` | 401 | CAN log → HTTP POST to VPS |
| `data/send_log_to_cloud.py` | 354 | Snapshot/replay → HTTP POST |
| `data/battery_can_parser.py` | 346 | Offline CLI parser |
| `firmware/.../main.cpp` | 766 | ESP32 — AT+HTTP* sequence, nested buildJSON() |
| `frontend/index.html` | 2244 | Dashboard — dual WS endpoints |
| `backend/bms_cloud.db` | — | SQLite (snapshots table) — 1 existing table |
| `docs/` | — | Only CONTRIBUTING.md; **no schema files yet** |

### Key findings relevant to this plan

1. **No `SENSOR:` line parser exists yet** — `server_wifi.py` has TCP listening (port 9000) and replay, but no solar/GNSS/scenario parsing. Those are all null stubs. The "merge collaborator's server" task is primarily absorbing `server_wifi.py`'s TCP reader and replay logic.
2. **Current `cells` array** is `[null, {mv,status}, …]` (length 20, index 0 = null). The spec wants `[{index:1,mv,status}, …]` (length 19, no null padding). **Schema break — all consumers must update together.**
3. **`battery_cells`** in temperatures has the same null-padding pattern that changes.
4. **`solar.post_mppt`** currently only has `current_a`; spec adds `voltage_v` there too.
5. **`temperatures`** gains `dcdc_c` field (not currently present).
6. **`vehicle`** gains `oil_level` and `ignition` (not currently present).
7. **SQLite `bms_cloud.db`** has a minimal `snapshots(id, ts, device_id, payload)` table — migration to MariaDB is straightforward.
8. **`requirements.txt`** has only 3 packages — needs `sqlalchemy`, `asyncmy`, `jsonschema`, `mariadb`/`asyncmy` added.

---

## 1. Questions I need answered before starting (§9 of spec)

**Please answer these four before I write a single line of code:**

| # | Question | Why it matters |
|---|----------|----------------|
| Q1 | What is the `schema_version` string for this release? (e.g. `"2.0.0"`) | Goes into every payload and the JSON Schema `$id` |
| Q2 | Keep `POST /api/ingest` for one release (deprecated) or remove immediately? | Affects firmware build flag and backend routing |
| Q3 | MariaDB host / port / database name / credentials? (or confirm `.env` file with those keys) | Needed before writing `db/schema.sql` and `asyncmy` connection string |
| Q4 | GNSS in v1: map tile (Leaflet) or just lat/lon text? | Affects frontend scope and whether leaflet.js is a new dependency |

---

## 2. What will change (by file)

### NEW files

| Path | Purpose |
|------|---------|
| `docs/schema/telemetry.schema.json` | JSON Schema Draft 2020-12, single source of truth |
| `docs/schema/telemetry_types.md` | Human-readable field reference (type, unit, range, source) |
| `backend/db/schema.sql` | MariaDB DDL: vehicles, telemetry, vehicle_config + seed data |
| `backend/db/migrate_sqlite_to_mariadb.py` | One-shot SQLite → MariaDB migration, `--dry-run` flag |
| `backend/db/repositories.py` | `VehicleRepo`, `TelemetryRepo`, `VehicleConfigRepo` (SQLAlchemy 2 async) |
| `backend/tcp_server.py` | asyncio TCP server on port 8766, handshake + framing |
| `backend/parsers/__init__.py` | Package init |
| `backend/parsers/can_parser.py` | Pure functions: `parse_can_frame()` per CAN ID |
| `backend/parsers/sensor_parser.py` | Pure functions: `parse_sensor_line()` per `SENSOR:` key |
| `backend/snapshot_builder.py` | `SnapshotBuilder` class — accumulates partial updates → validated snapshot |
| `backend/validator.py` | Thin wrapper: `validate(payload)` raises on schema error, counts errors |
| `tests/test_can_parser.py` | Unit tests for each CAN ID (fixtures from real log) |
| `tests/test_sensor_parser.py` | Unit tests for each `SENSOR:` key |
| `tests/test_snapshot_builder.py` | Unit tests for accumulation logic |
| `tests/test_tcp_server.py` | Integration test with fake TCP client |
| `MIGRATION.md` | Ops runbook: VPS deploy steps, MariaDB setup, systemd units |
| `CHANGELOG.md` | Version history |

### MODIFIED files

| Path | Changes |
|------|---------|
| `backend/server.py` | Replace `BMSState` + flat routes with `SnapshotBuilder`; keep `/ws` + `/ws/cloud` for one release; add `/api/schema`, `/api/vehicles/{id}/config`, `/metrics`; deprecate `/api/ingest` (Q2) |
| `backend/replay_to_cloud.py` | Add `--transport http\|tcp` flag (default `tcp`); TCP framing client |
| `data/send_log_to_cloud.py` | Same `--transport` flag |
| `firmware/.../main.cpp` | Replace AT+HTTP* with AT+CIP* TCP; length-prefixed framing; keep HTTP behind `#ifdef USE_HTTP` build flag (Q2); RTC-RAM ring buffer |
| `frontend/index.html` | New panels: solar, DC/DC, GNSS, scenario, oil/ignition; load config from `/api/vehicles/{id}/config`; `schema_version` footer; raw-payload drawer; update cell/temp array reads for new index-object format |
| `report/Section Files/04-systems-architecture.tex` | TCP diagram, MariaDB ERD |
| `report/Section Files/08-software-implementation.tex` | New subsections: unified schema, TCP framing/handshake, AT+CIP firmware, ring buffer |
| `report/Section Files/09-cloud-connectivity.tex` | Replace SQLite with MariaDB, new endpoints, updated schema section, security subsection |
| `backend/requirements.txt` | Add: `sqlalchemy>=2.0`, `asyncmy`, `jsonschema`, `python-dotenv` |

### NEW report section

| Path | Content |
|------|---------|
| `report/Section Files/10-deployment-operations.tex` | VPS requirements, MariaDB backup, upgrade path from SQLite, systemd units |

### Files I will NOT touch

- `data/battery_can_parser.py` — offline tool, no network changes needed (only schema-output update if cells array changes)
- `backend/server_wifi.py` — absorbed into `tcp_server.py`; original kept as reference then deleted after tests pass
- `firmware/.../platformio.ini` — no changes
- `firmware/.../include/secrets.h` — only new constants added (`VPS_TCP_PORT`)
- `report/` LaTeX infrastructure (preamble, main.tex, titlepage) — no changes
- All hardware files, CAN simulation sketches

---

## 3. Execution order

Step | Task | Deliverable | Blocks
-----|------|-------------|-------
1 | ✅ This plan | `PLAN.md` | Nothing starts without approval
2 | JSON Schema + types doc | `docs/schema/telemetry.schema.json`, `telemetry_types.md` | Steps 3–8 all reference this
3 | MariaDB schema + migration | `backend/db/schema.sql`, `migrate_sqlite_to_mariadb.py` | Step 5 (DB writes)
4 | Parsers + SnapshotBuilder + unit tests | `backend/parsers/`, `snapshot_builder.py`, `tests/test_*.py` | Steps 5, 6
5 | TCP server (VPS side) + integration test | `backend/tcp_server.py`, `tests/test_tcp_server.py` | Step 7
6 | `server.py` refactor (use SnapshotBuilder, new endpoints) | Updated `server.py` | Step 8 (frontend)
7 | Firmware: AT+CIP TCP + ring buffer | `main.cpp` | Can be done in parallel with 6
8 | Replay tools: `--transport tcp` | `replay_to_cloud.py`, `send_log_to_cloud.py` | After step 5
9 | Frontend: new panels + config API | `index.html` | After step 6
10 | Report §04, §08, §09, new §10 | 4 `.tex` files | After steps 5–7 are stable
11 | `MIGRATION.md`, `CHANGELOG.md` | 2 docs | Last

---

## 4. Schema breaking changes (cells array format)

The current `cells` field is a null-padded 1-based array:
```json
"cells": [null, {"mv": 3280, "status": "good"}, ...]
```
The spec requires a fixed-length array of typed objects with an explicit `index` field:
```json
"cells": [{"index":1,"mv":3280,"status":"good"}, ..., {"index":19,"mv":null,"status":"unknown"}]
```
This is a **breaking change** for:
- `buildJSON()` in firmware
- `build_json()` in replay tools
- Cell grid rendering in `index.html`
- The MariaDB `payload` column (stored JSON)

All four must be updated atomically in the same commit (Step 2 defines the schema; Steps 4–9 implement it).

Same applies to `temperatures.battery_cells`:
```json
"battery_cells": [{"index":1,"c":21.0}, {"index":2,"c":22.0}, {"index":3,"c":null}, {"index":4,"c":null}]
```

---

## 5. What will NOT change

- The CAN bus decoding logic (frame IDs, byte offsets, scaling factors) — this is correct and tested
- The SOC calibration formula and constants (2500 mV / 3387 mV)
- Cell voltage thresholds and status labels
- The `X-Api-Key` authentication mechanism (promoted to TCP handshake)
- The WebSocket push cadence (10 Hz serial, 2 Hz cloud)
- The serial mode pipeline (USB serial → BMSState → /ws)

---

## 6. Definition of done (per spec §8)

- [ ] `pytest` green with ≥80% parser coverage
- [ ] 10-minute replay produces identical dashboard output on HTTP and TCP paths
- [ ] MariaDB contains rows for replay run; `payload` round-trips through schema validator
- [ ] Report §04, §08, §09, §10 reflect new architecture
- [ ] `PLAN.md`, `MIGRATION.md`, `CHANGELOG.md` present and current
- [ ] `/metrics` endpoint returns `validation_errors_total` counter
- [ ] `GET /api/schema` returns the JSON Schema
- [ ] `GET /api/vehicles/{id}/config` returns the seed config for the O'CELL pack
