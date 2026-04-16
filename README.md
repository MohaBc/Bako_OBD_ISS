# Bako OBD ISS

**SAE J1939 Battery Management System — Diagnostic, Monitoring & Cloud Platform**

ISS Senior Project 2026 — MEDTECH
Real-time CAN bus decoding, live dashboard, cloud telemetry, and diagnostic tooling for the O'CELL IFS60.8-500-F-E3 LiFePO₄ battery pack.

---

## Table of Contents

1. [What This Project Is](#1-what-this-project-is)
2. [System Architecture](#2-system-architecture)
3. [Repository Structure](#3-repository-structure)
4. [Hardware & Target System](#4-hardware--target-system)
5. [CAN Protocol Reference](#5-can-protocol-reference)
6. [Tools Overview](#6-tools-overview)
7. [Quick Start](#7-quick-start)
8. [Tool 1 — Python CLI Parser](#8-tool-1--python-cli-parser)
9. [Tool 2 — Log-to-Cloud Replay](#9-tool-2--log-to-cloud-replay)
10. [Tool 3 — Live Dashboard](#10-tool-3--live-dashboard)
11. [Cloud Pipeline](#11-cloud-pipeline)
12. [ESP32 Cloud Publisher Firmware](#12-esp32-cloud-publisher-firmware)
13. [Backend API Reference](#13-backend-api-reference)
14. [Sample Data](#14-sample-data)
15. [Contributing & Team Workflow](#15-contributing--team-workflow)
16. [Versioning](#16-versioning)
17. [Known Issues](#17-known-issues)
18. [License & Credits](#18-license--credits)

---

## 1. What This Project Is

Bako OBD ISS is a complete battery diagnostic and cloud monitoring system built around the SAE J1939 CAN bus protocol. It decodes raw CAN frames from a LiFePO₄ battery management system and presents the data in multiple ways: a terminal report, a live serial dashboard, and a cloud-connected real-time interface fed from an ESP32 over cellular (SIM800L GPRS).

The system supports four real-world use cases:

- **Field diagnostics** — connect a laptop to the battery via ESP32 USB, open the live dashboard, and immediately see pack voltage, cell balance, temperatures, and fault status in real time.
- **Cloud monitoring** — the ESP32 reads CAN data and publishes to Firebase Realtime Database over cellular (SIM800L). The dashboard pulls from Firebase so it can be viewed from anywhere.
- **Log analysis** — take a captured `.txt` CAN log from any session and analyze it offline, or replay it to Firebase to test the full cloud pipeline without hardware.
- **Development & testing** — replay a real car log through the cloud pipeline to verify dashboard behavior before connecting live hardware.

---

## 2. System Architecture

### Serial Mode (local, USB)

```
┌───────────────────────────────────────────────────────────────────┐
│  O'CELL BMS  ──CAN 250kbps──  ESP32 + MCP2515  ──USB 115200──  PC │
└───────────────────────────────────────────────────────────────────┘
                                                         │
                                                  server.py (FastAPI)
                                                  pyserial reader
                                                  J1939 frame decoder
                                                         │ WebSocket /ws
                                                         │ 10 Hz JSON push
                                                  index.html (browser)
                                                  SERIAL mode
```

### Cloud Mode (cellular, Firebase)

```
┌──────────────────────────────────────────────────────────────────────┐
│  O'CELL BMS  ──CAN 250kbps──  ESP32 + MCP2515                        │
│                                     │                                │
│                               SIM800L (GPRS)                         │
│                               AT+HTTP* service                       │
└──────────────────────────────────────────────────────────────────────┘
                                      │ HTTPS PUT every 5 s
                                      ▼
                          Firebase Realtime Database
                          /bms/live.json   (live snapshot)
                          /bms/history.json (time-series log)
                                      │
                               server.py polls
                               /ws/cloud every 2 s
                                      │
                              index.html (browser)
                              CLOUD mode
```

### Log Replay (no hardware needed)

```
data/raw/bms_log_*.txt
        │
  send_log_to_cloud.py
  (parses J1939 frames,
   streams JSON snapshots)
        │ HTTPS PUT
        ▼
  Firebase Realtime DB  ──→  server.py /ws/cloud  ──→  browser CLOUD mode
```

---

## 3. Repository Structure

```
Bako_OBD_ISS/
│
├── firmware/
│   ├── esp32_cloud_publisher/         # Cloud publisher — MCP2515 CAN + SIM800L GPRS
│   │   └── esp32_cloud_publisher/     # PlatformIO project
│   │       ├── src/main.cpp           # Main firmware (CAN read → JSON → Firebase PUT)
│   │       ├── include/secrets.h      # Firebase credentials + APN (NOT committed)
│   │       └── platformio.ini         # Board config + library deps
│   ├── CAN_receive/                   # Basic MCP2515 CAN frame receiver sketch
│   │   └── CAN_receive.ino
│   ├── CAN_simulation_7_phases/       # 7-phase BMS cycle simulator
│   │   └── CAN_simulation_7_phases.ino
│   └── CAN_simulation_7_phases_wifi/  # WiFi variant of simulator
│       └── CAN_simulation_7_phases_wifi.ino
│
├── backend/
│   ├── server.py                      # FastAPI server — serial + cloud endpoints
│   ├── replay_to_cloud.py             # CAN log → decode → Firebase replay tool
│   ├── requirements.txt               # Pinned pip dependencies
│   └── bms_cloud.db                   # SQLite cloud snapshot store (auto-created)
│
├── frontend/
│   └── index.html                     # Live dashboard (SERIAL / CLOUD toggle)
│
├── data/
│   ├── raw/
│   │   └── bms_log_2026-03-10T10-20-02.txt   # Real car CAN capture (3196 frames)
│   ├── battery_can_parser.py          # Offline CLI log parser
│   ├── send_log_to_cloud.py           # Log-to-Firebase replay tool
│   └── README.md
│
├── hardware/                          # Schematics, PCB, mechanical design
├── report/                            # LaTeX ISS academic report
├── docs/
│   └── CONTRIBUTING.md                # Branch strategy, commit rules
│
├── .gitignore
└── README.md
```

---

## 4. Hardware & Target System

### Battery Pack

| Property | Value |
|----------|-------|
| Model | O'CELL IFS60.8-500-F-E3 |
| Chemistry | LiFePO₄ (LFP) |
| Configuration | 19S1P — 19 cells in series, 1 parallel |
| Nominal voltage | 60.8 V |
| Capacity | 50 Ah (actual measured: 44.7 Ah, SOH 89.4%) |
| Cell nominal voltage | 3.2 V |
| Cell full voltage | 3.65 V |
| Cell min voltage | 2.5 V |
| Max charge current | 25 A |
| Max discharge current | 50 A |

### CAN Interface

| Property | Value |
|----------|-------|
| Standard | SAE J1939 |
| Frame type | 29-bit extended CAN ID |
| Baud rate | 250 kbps |
| Physical layer | CAN high / CAN low differential pair |

### ESP32 + MCP2515 Wiring

| MCP2515 Pin | ESP32 GPIO |
|-------------|------------|
| CS | 15 |
| INT | 4 |
| SCK | 18 (VSPI) |
| MISO | 19 (VSPI) |
| MOSI | 23 (VSPI) |
| VCC | 3.3 V |

### SIM800L Wiring

| SIM800L Pin | ESP32 GPIO |
|-------------|------------|
| TXD | 16 (UART1 RX) |
| RXD | 17 (UART1 TX) |
| RST | 5 |
| VCC | 4.0 V external (≥ 2 A) |
| GND | GND (shared) |

> The SIM800L requires a separate 4 V supply capable of 2 A peak. Powering it from the ESP32 3.3 V rail will cause random resets.

---

## 5. CAN Protocol Reference

All frame IDs use `0x18_FUNC_SUB_F4` structure (29-bit extended J1939). The log files may show `0x98...` — this is the same PGN with J1939 priority bits set.

### Supported Frame Types

| CAN ID | Name | Tx rate | Description |
|--------|------|---------|-------------|
| `0x18C828F4` | Cell voltages group 1 | 500 ms | Cells 1–4, **big-endian** uint16, 1 mV/bit |
| `0x18C928F4` | Cell voltages group 2 | 500 ms | Cells 5–8 |
| `0x18CA28F4` | Cell voltages group 3 | 500 ms | Cells 9–12 |
| `0x18CB28F4` | Cell voltages group 4 | 500 ms | Cells 13–16 |
| `0x18CC28F4` | Cell voltages group 5 | 500 ms | Cells 17–19 (bytes 6–7 padded 0x0000) |
| `0x18B428F4` | Temperatures | 500 ms | Bytes 0–3: probes 1–4, `raw − 40 = °C`, 0xFF = absent |
| `0x18FFE5F4` | BMS Charging Request | 500 ms | LE uint16: max charge voltage ÷10 V, max charge current ÷10 A; byte 4 bit0: start signal |
| `0x18FF28F4` | BMS Basic Message 1 | 100 ms | byte 0: status flags; byte 1: SOC %; bytes 2–3: pack current; bytes 4–5: pack voltage; byte 6: fault level; byte 7: error code |
| `0x18FE28F4` | BMS Basic Message 2 | 100 ms | LE uint16: max/min cell mV; temp bytes; discharge limit ÷10 A |

### Byte Layouts

**`0x18FF28F4` — BMS Basic Message 1 (100 ms)**
```
Byte  0    uint8      Status flags (bit-field, see below)
Byte  1    uint8      SOC 0–100 %
Bytes 2–3  LE uint16  Pack current  (value − 5000) × 0.1  → A  (negative = charging)
Bytes 4–5  LE uint16  Pack voltage  × 0.1                 → V
Byte  6    uint8      Fault level
Byte  7    uint8      Error code (see fault table below)
```

Status flags (byte 0 of 0x18FF28F4):

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | 0x01 | Charge cable connected |
| 1 | 0x02 | Charging in progress |
| 2 | 0x04 | Discharging in progress |
| 3 | 0x08 | BMS ready |
| 4 | 0x10 | Discharge contactor closed |
| 5 | 0x20 | Charge contactor closed |

Fault codes (byte 7 of 0x18FF28F4):

| Code | Name |
|------|------|
| 0x00 | ok |
| 0x01 | over_temp_severe |
| 0x02 | total_voltage_high |
| 0x03 | total_voltage_low |
| 0x04 | discharge_overcurrent |
| 0x05 | cell_voltage_high |
| 0x06 | cell_voltage_low |

**`0x18FE28F4` — BMS Basic Message 2 (100 ms)**
```
Bytes 0–1  LE uint16  Max cell voltage             → mV
Bytes 2–3  LE uint16  Min cell voltage             → mV
Byte  4    uint8      Max temperature  (raw − 40)  → °C  (0xFF = absent)
Byte  5    uint8      Min temperature  (raw − 40)  → °C  (0xFF = absent)
Bytes 6–7  LE uint16  Max discharge current ÷ 10   → A
```

**`0x18FFE5F4` — BMS Charging Request (500 ms)**
```
Bytes 0–1  LE uint16  Max charge voltage  × 0.1  → V
Bytes 2–3  LE uint16  Max charge current  × 0.1  → A
Byte  4    uint8      Bit 0 = 0 → charger start signal
Byte  5    uint8      Protection flags (optional, DLC ≥ 6)
```

**`0x18C8–CC28F4` — Cell voltage frames (500 ms)**
```
4 cells × 2 bytes each, big-endian uint16, 1 mV/bit
Last frame (0xCC): bytes 6–7 = 0x0000 (cell 20 does not exist)
```

### SOC Calibration

Calibrated from 4 real car sessions against the vehicle's onboard display:

```
Formula:  SOC% = (avg_cell_mV − 2500) / (3387 − 2500) × 100
  2500 mV/cell = 47.50 V pack = 0%
  3387 mV/cell = 64.35 V pack = 100%  (car-display calibrated)
```

Values above 3387 mV (during active charging) are clamped to 100%.

### Voltage Thresholds

| Threshold | Value | Status |
|-----------|-------|--------|
| Overvoltage | ≥ 3750 mV | Critical |
| Full | ≥ 3650 mV | Full |
| Good | ≥ 3300 mV | Good |
| Normal | ≥ 3200 mV | Normal |
| Low | ≥ 2500 mV | Low |
| Undervoltage | < 2500 mV | Critical |

---

## 6. Tools Overview

| Tool | File | Hardware needed | Description |
|------|------|----------------|-------------|
| CLI parser | `data/battery_can_parser.py` | No | Parse a log file, print report or CSV |
| Log-to-cloud replay | `backend/replay_to_cloud.py` | No | Replay log → Firebase Realtime DB (direct PUT) |
| Live dashboard | `frontend/index.html` + `backend/server.py` | ESP32 (or log replay) | Real-time SERIAL / CLOUD dashboard |
| Cloud firmware | `firmware/esp32_cloud_publisher/` | ESP32 + MCP2515 + SIM800L | Reads CAN, publishes to Firebase via GPRS |

---

## 7. Quick Start

### Live monitoring over USB (SERIAL mode)

```bash
cd backend
pip install -r requirements.txt
python3 server.py                        # auto-detects USB port
python3 server.py --port /dev/ttyUSB0   # or specify port
```

Open `http://localhost:8765` — click **SERIAL** in the header.

### Cloud dashboard from a real log file (no ESP32 needed)

**Terminal 1 — start backend pointed at your Firebase:**
```bash
cd /path/to/Bako_OBD_ISS

FIREBASE_URL=https://<project>-default-rtdb.firebaseio.com \
FIREBASE_TOKEN=<your-token> \
python3 backend/server.py --cloud-only
```

**Terminal 2 — replay the log to Firebase:**
```bash
python3 backend/replay_to_cloud.py --speed 5 --interval 5000
```

Open `http://localhost:8765` — click **CLOUD** in the header. The dashboard animates live as the log replays.

### Offline log analysis (no server, no internet)

```bash
python3 data/battery_can_parser.py raw/bms_log_2026-03-10T10-20-02.txt
python3 data/battery_can_parser.py raw/bms_log_2026-03-10T10-20-02.txt --csv
```

---

## 8. Tool 1 — Python CLI Parser

**File:** `data/battery_can_parser.py`
**Requirements:** Python 3.8+ — no external dependencies

Reads any `.txt` CAN log file and prints a formatted battery analysis report to the terminal, or outputs CSV.

### Usage

```bash
# Default report (uses bms_log_2026-03-10T10-20-02.txt)
python3 data/battery_can_parser.py

# Specify a log file
python3 data/battery_can_parser.py data/raw/bms_log_2026-03-10T10-20-02.txt

# CSV output
python3 data/battery_can_parser.py data/raw/bms_log_2026-03-10T10-20-02.txt --csv
```

### Example Output

```
============================================================
  O'CELL BMS — Offline CAN Log Analysis
  File   : bms_log_2026-03-10T10-20-02.txt
  Frames : 3197
============================================================

  PACK SUMMARY
  ├─ Pack voltage   : 63.17 V
  ├─ SOC (voltage)  : 93.0 %
  ├─ SOC (coulomb)  : 69.3 %
  ├─ SOC (BMS)      : 63.1 %
  ├─ Avg cell       : 3325 mV
  ├─ Max cell       : 3327 mV
  ├─ Min cell       : 3322 mV
  ├─ Cell spread    : 5 mV
  └─ Disch limit    : 50.0 A

  CELL VOLTAGES (19 cells decoded)
  C01  3325 mV  [████████████████████████]  GOOD
  C02  3326 mV  [████████████████████████]  GOOD
  ...
```

---

## 9. Tool 2 — Log-to-Cloud Replay

**File:** `backend/replay_to_cloud.py`
**Requirements:** Python 3.8+ — no external dependencies

Reads a raw CAN log file, decodes every BMS frame using the same logic as the ESP32 firmware, builds a complete JSON snapshot, and PUTs it directly to Firebase Realtime Database. The local dashboard server (`server.py`) polls Firebase and pushes the data to the browser via `/ws/cloud`.

Pipeline: `log file → decode frames → JSON → Firebase /bms/live → server.py → browser`

### CLI Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--log` / `-l` | `data/raw/bms_log_2026-03-10T10-20-02.txt` | Path to raw CAN log file |
| `--speed` / `-s` | `1.0` | Replay speed multiplier (e.g. `5` = 5× real-time) |
| `--interval` / `-i` | `5000` | Snapshot interval in log-time ms |
| `--loop` | off | Loop file forever |

### Usage

```bash
# Default — real-time replay, snapshot every 5 s of log time
python3 backend/replay_to_cloud.py

# 5× speed
python3 backend/replay_to_cloud.py --speed 5

# Custom log file, 10× speed, loop forever
python3 backend/replay_to_cloud.py --log data/raw/my_log.txt --speed 10 --loop
```

Firebase credentials are configured directly in the script (`FIREBASE_HOST`, `FIREBASE_AUTH`).

### Terminal Output

```
-------------------------------------------------------
  BMS Log → Firebase → Interface
-------------------------------------------------------
  Log      : data/raw/bms_log_2026-03-10T10-20-02.txt
  Firebase : https://esp32-connection-test-default-rtdb.firebaseio.com/bms/live
  Speed    : 1.0×   interval: 5000 ms log-time
-------------------------------------------------------
  3197 frames  (11455–139267 ms)

  [OK  ] #01  cells=19  SOC=100.0%  pack=63.1V  I=-15.1A  temps=4  fault=0  ready=True  charger=yes
  [OK  ] #02  cells=19  SOC=100.0%  pack=63.1V  I=-15.1A  temps=4  fault=0  ready=True  charger=yes
  ...
  Loop 1 done — 25 snapshots sent to Firebase.
Replay complete.
```

---

## 10. Tool 3 — Live Dashboard

**Files:** `backend/server.py` + `frontend/index.html`
**Requirements:** Python 3.8+, `pip install -r backend/requirements.txt`

### Source Toggle

The dashboard header has a **SERIAL / CLOUD** toggle:

| Mode | Data source | WebSocket |
|------|-------------|-----------|
| SERIAL | ESP32 USB serial port | `/ws` — 10 Hz push |
| CLOUD | Firebase Realtime DB (via backend) | `/ws/cloud` — 2 Hz push |

### Running

```bash
# Serial mode (ESP32 connected via USB)
python3 backend/server.py

# Cloud-only mode (no serial port required)
python3 backend/server.py --cloud-only

# Cloud mode with Firebase
FIREBASE_URL=https://<project>-default-rtdb.firebaseio.com \
FIREBASE_TOKEN=<token> \
python3 backend/server.py --cloud-only
```

Open **http://localhost:8765**

### Dashboard Panels

| Panel | Contents |
|-------|----------|
| SOC ring | Animated state-of-charge gauge; BMS + coulomb sub-values shown below |
| Pack KPIs | Pack voltage · Discharge limit (A) · Charge request (A) · Cell count |
| Cell grid | All 19 cells — voltage bars, colour-coded status, min/max/avg/spread |
| Temperature | Up to 4 probes with real-time bars and chart history |
| Serial monitor | Raw CAN frame log with export to TXT / XLSX |
| **Cloud log** | Real-time cloud communication events (RECV · POLL · WS · ERR) — independent of source toggle |

---

## 11. Cloud Pipeline

### Overview

```
ESP32 firmware (main.cpp)
    ├─ MCP2515 reads CAN frames at 250 kbps
    ├─ Decodes J1939: cell voltages, temps, SOC
    ├─ Builds JSON snapshot every 5 s
    └─ SIM800L HTTPS PUT → Firebase /bms/live.json
                               POST → Firebase /bms/history.json

Firebase Realtime Database
    └─ Stores live snapshot + history entries

backend/server.py  (FIREBASE_URL env set)
    └─ /ws/cloud WebSocket polls Firebase REST every 2 s
           └─ Pushes JSON to browser

index.html (CLOUD mode)
    └─ Receives and renders live BMS data
```

### Firebase Data Structure

```json
{
  "bms": {
    "live": {
      "connected": true,
      "source": "esp32-sim800l",
      "device_id": "esp32-bms-001",
      "timestamp": "26/04/15,09:32:11+00",
      "frame_count": 3840,

      "soc": 87.4,
      "soc_bms": 85,

      "pack_v": 62.31,
      "pack_current_a": -15.1,

      "cell_count": 19,
      "cell_avg_mv": 3280,
      "cell_max_mv": 3295,
      "cell_min_mv": 3265,
      "cell_spread_mv": 30,
      "cells": { "1": {"mv": 3280, "status": "good"}, "...": "..." },

      "temps": { "1": 21.0, "2": 22.0, "3": 21.0, "4": 20.0 },
      "avg_temp": 21.0,
      "temp_max_c": 22.0,
      "temp_min_c": 20.0,

      "max_disch_a": 100.0,

      "fault_level": 0,
      "error_code": 0,
      "fault_name": "ok",

      "status": {
        "charge_cable": true,
        "charging": true,
        "discharging": false,
        "ready": true,
        "disch_contactor": false,
        "charge_contactor": true
      },

      "charger": {
        "max_charge_v": 69.3,
        "max_charge_a": 35.0,
        "start_signal": true,
        "prot_flags": null
      }
    },
    "history": {
      "-NxAbCd...": { "...": "..." }
    }
  }
}
```

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `FIREBASE_URL` | — | Firebase Realtime DB URL (enables cloud mode) |
| `FIREBASE_TOKEN` | — | Firebase database secret / legacy token |
| `BMS_API_KEY` | `bako-bms-2024` | API key for ESP32 → `/api/ingest` POST |

### Backend REST Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Serve dashboard HTML |
| `WS` | `/ws` | Serial stream — 10 Hz |
| `WS` | `/ws/cloud` | Cloud stream — Firebase or in-memory, 2 Hz |
| `POST` | `/api/ingest` | Receive BMS JSON from ESP32 (requires `X-Api-Key` header) |
| `GET` | `/api/latest` | Most recent cloud snapshot |
| `GET` | `/api/history?limit=N` | Last N snapshots from SQLite (default 100) |
| `GET` | `/api/cloud-log` | Last 100 cloud communication events (polled by Cloud Log panel every 2 s) |

---

## 12. ESP32 Cloud Publisher Firmware

**Path:** `firmware/esp32_cloud_publisher/esp32_cloud_publisher/`
**Framework:** PlatformIO (ESP32 Arduino)

### Features

- Reads SAE J1939 CAN frames via MCP2515 at 250 kbps
- Decodes all 5 BMS frame types: cell voltages (19 cells), temperatures, BMS Basic Message 1 & 2, Charging Request
- Computes SOC from average cell voltage; also reads BMS coulomb counter and direct pack voltage
- Decodes status bit-field (charging/discharging/ready/contactors) and fault level + error code with human-readable fault name
- Decodes charger limits: max charge voltage, max charge current, charger start signal
- Uses `HardwareSerial` UART1 for SIM800L — more reliable than SoftwareSerial
- Uses SIM800L `AT+HTTP*` service with `AT+HTTPSSL=1` for HTTPS to Firebase
- Reads real network time from `AT+CCLK?` and includes it in each snapshot
- Auto-reconnects bearer on GPRS failure with graceful brownout recovery:
  - Detects `"Call Ready"` URC in all AT response buffers → sets `g_moduleRebooted` flag
  - Aborts current HTTP transaction immediately on mid-transaction reboot detection
  - Waits 20 s for SIM to initialize, then retries `AT+CPIN?` up to 6 times (4 s apart) before reconnecting
- Publishes every 5 seconds — PUT to `/bms/live` and POST to `/bms/history`

### Dependencies (auto-installed by PlatformIO)

```ini
lib_deps =
    coryjfowler/MCP_CAN@^1.5.1
    bblanchon/ArduinoJson@^7.0.0
```

### Configuration — `include/secrets.h`

```cpp
const char FIREBASE_HOST[] = "https://YOUR_PROJECT-default-rtdb.firebaseio.com";
const char FIREBASE_AUTH[] = "YOUR_DATABASE_SECRET";
const char DEVICE_ID[]     = "esp32-bms-001";

// Optional APN override (default: "internet")
// #define APN  "iam"
```

> `secrets.h` is in `.gitignore` — credentials are never committed.

### Flashing

```bash
cd firmware/esp32_cloud_publisher/esp32_cloud_publisher
pio run --target upload
pio device monitor   # 115200 baud
```

### Expected Serial Output

```
=== BMS Firebase Publisher ===
[CAN] Init MCP2515... OK (250 kbps)
[GSM] Attempt 1/3
[GSM] SIM ready
[GSM] Good signal
[GSM] Bearer OK
=== Ready — reading CAN bus ===

[CAN] Frames: 120  SOC: 87.4%  Pack: 62.31 V  Cells: 19  Temps: 2
[HTTP] >> AT+HTTPACTION=3
[HTTP] PUT 200  (12 bytes)
[HTTP] >> AT+HTTPACTION=1
[HTTP] POST 200  (36 bytes)
```

---

## 13. Backend API Reference

### POST `/api/ingest`

Receives a BMS snapshot from the ESP32. Stores in SQLite and updates the in-memory cloud state for `/ws/cloud`.

```bash
curl -X POST http://localhost:8765/api/ingest \
  -H "Content-Type: application/json" \
  -H "X-Api-Key: bako-bms-2024" \
  -d '{
    "connected": true,
    "soc": 87.4,
    "pack_v": 62.31,
    "cell_count": 19,
    "cells": {"1": {"mv": 3280, "status": "good"}},
    "temps": {"1": 21.0}
  }'
```

Response: `{"status": "ok", "ts": "2026-04-15T09:32:11.123456"}`

### GET `/api/latest`

Returns the most recent cloud snapshot.

### GET `/api/history?limit=100`

Returns the last N snapshots from SQLite, newest first.

### WebSocket `/ws/cloud`

Connect from JS: `new WebSocket("ws://localhost:8765/ws/cloud")`

Sends the same JSON structure as `/api/latest` every 2 seconds. If `FIREBASE_URL` is set, the data comes from Firebase; otherwise from the last `/api/ingest` POST.

### GET `/api/cloud-log`

Returns a JSON array of the last 100 timestamped cloud communication events. The Cloud Log panel in the dashboard polls this endpoint every 2 seconds independently of the SERIAL/CLOUD source toggle.

```json
[
  "[08:20:38] [POLL] Firebase OK — 19 cells, SOC 93.0%",
  "[08:20:39] [RECV] POST /api/ingest from 192.168.1.x — device=esp32-bms-001 cells=19 SOC=87.4% pack=62.31V",
  "[08:20:40] [WS]   Browser connected to /ws/cloud from 127.0.0.1",
  "[08:20:41] [ERR]  Firebase HTTP 401: Unauthorized"
]
```

| Level | Colour | Meaning |
|-------|--------|---------|
| `RECV` | Green | BMS snapshot received via `POST /api/ingest` |
| `POLL` | Blue | Firebase REST poll completed (deduplicated, max 1 per 5 s) |
| `WS` | Yellow | Browser tab connected or disconnected from `/ws/cloud` |
| `ERR` | Red | HTTP error or network failure |

---

## 14. Sample Data

### `data/raw/bms_log_2026-03-10T10-20-02.txt`

Real CAN capture from the physical vehicle — 3196 frames, ~4 minutes of operation.

| Parameter | Value |
|-----------|-------|
| Pack voltage | 63.17 V |
| SOC (voltage-based) | 93.0 % |
| SOC (coulomb counter) | 69.3 % |
| SOC (BMS internal) | 63.1 % |
| Cell avg | 3325 mV |
| Cell spread | 5 mV (excellent balance) |
| Temperatures | 20 / 20 / 21 / 21 °C |
| Discharge limit | 50 A |
| Frames | 3197 |

### Decoding a Frame by Hand

```
[11771ms] ID: 0x18CB28F4  DLC: 8  Data: 0D 3F 0D 38 0D 3A 0D 39

Frame 0x18CB28F4 — Cell voltages group 4  (cells 13–16)
func = (0x18CB28F4 >> 16) & 0xFF = 0xCB → group = 0xCB - 0xC8 = 3

  Bytes 0–1  0x0D 0x3F  → big-endian = 0x0D3F = 3391 mV  → Cell 13
  Bytes 2–3  0x0D 0x38  → big-endian = 0x0D38 = 3384 mV  → Cell 14
  Bytes 4–5  0x0D 0x3A  → big-endian = 0x0D3A = 3386 mV  → Cell 15
  Bytes 6–7  0x0D 0x39  → big-endian = 0x0D39 = 3385 mV  → Cell 16
```

---

## 15. Contributing & Team Workflow

Full rules in `docs/CONTRIBUTING.md`.

### Branch Naming

| Type | Pattern | Example |
|------|---------|---------|
| Feature | `feature/<short-desc>` | `feature/cloud-ingest` |
| Bug fix | `fix/<short-desc>` | `fix/can-endianness` |
| Firmware | `firmware/<short-desc>` | `firmware/sim800l-gprs` |
| Docs | `docs/<short-desc>` | `docs/report-section-3` |

All branches cut from `main`.

### Commit Format

```
<type>: <short summary>

feat: add Firebase cloud ingest endpoint
fix: correct cell voltage big-endian decoding
firmware: rewrite SIM800L publisher with AT+HTTP service
docs: update README with cloud pipeline
```

### Domain Ownership

| Domain | Folder | Owner |
|--------|--------|-------|
| Firmware | `firmware/` | Embedded team |
| Backend | `backend/` | Backend team |
| Frontend | `frontend/` | Frontend team |
| Data | `data/` | Data / analysis |
| Report | `report/` | Academic report |

### Hard Rules

```
Never:  git push --force         on any shared branch
Never:  push directly to main    always use a PR
Never:  commit secrets.h or .env  always in .gitignore
```

---

## 16. Versioning

| Increment | When |
|-----------|------|
| MAJOR | Breaking change to CAN protocol, API, or hardware interface |
| MINOR | New backward-compatible feature |
| PATCH | Bug fix |

Current version: **v0.4.0** — 2026-04-16 — Full BAKO CAN protocol decoder + SIM800L brownout recovery + Firebase replay pipeline

---

## 17. Known Issues

**SIM800L power brownout mid-transaction** — The SIM800L requires a stable 3.7–4.2 V supply capable of 2 A peak. If the supply droops during GPRS TX (peak ~1.5 A), the module resets and emits a `"Call Ready"` URC mid-transaction. The firmware detects this in all AT response loops, sets `g_moduleRebooted`, aborts the HTTP transaction, waits 20 s, then retries `AT+CPIN?` up to 6 times before reconnecting. Long-term fix: add a 1000 µF 10 V bulk capacitor close to the SIM800L VCC pin.

**SIM800L HTTPS on older firmware** — `AT+HTTPSSL=1` requires SIM800L firmware ≥ R14.18. Check with `AT+GSV`. If your module does not support it, use an HTTP-only endpoint (change `FIREBASE_HOST` to `http://`) or upgrade to a SIM7000/A7670 module with native TLS.

**APN varies by carrier** — The default APN is `"internet"`. Maroc Telecom uses `"iam"`, Inwi uses `"inwi"`. Set yours in `include/secrets.h`.

**Firebase legacy token deprecation** — Firebase database secrets (legacy tokens) are deprecated but still functional. For new projects prefer Firebase service account + short-lived tokens. The current implementation uses the legacy token for simplicity.

**Firebase cells array format** — When the ESP32 firmware pushes cells, Firebase may store them as a JSON array (`[null, {mv,status}, ...]`) instead of a dict. The backend's `_firebase_fetch()` automatically normalises this to the standard `{"1": {…}, "2": {…}}` dict format so the dashboard always receives consistent data.

**Port already in use on server restart** — If `server.py` fails with `address already in use`, run: `lsof -ti :8765 | xargs kill -9`

**Report content** — all 22 section files in `report/Section Files/` contain only `% Content placeholder`. The LaTeX structure compiles but written content has not been added.

---

## 18. License & Credits

- **Project:** Bako OBD ISS — ISS Senior Project 2026
- **Institution:** MEDTECH
- **Protocol:** SAE J1939 (Society of Automotive Engineers)
- **Battery:** O'CELL IFS60.8-500-F-E3 LiFePO₄
- **Repository:** [github.com/MohaBc/Bako_OBD_ISS](https://github.com/MohaBc/Bako_OBD_ISS)

---

*Last updated: April 2026 — v0.4.0 — Status: Active Development — Platforms: Linux, macOS, Windows*
