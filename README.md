# Bako OBD ISS

**SAE J1939 Battery Management System Diagnostic & Monitoring Platform**

ISS Senior Project 2026 — MEDTECH
Real-time CAN bus decoding, live dashboard, and diagnostic tooling for the O'CELL IFS60.8-500-F-E3 LiFePO₄ battery pack.

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
9. [Tool 2 — Standalone Web Analyzer](#9-tool-2--standalone-web-analyzer)
10. [Tool 3 — Live Dashboard](#10-tool-3--live-dashboard)
11. [ESP32 BMS Simulator](#11-esp32-bms-simulator)
12. [Sample Data](#12-sample-data)
13. [Contributing & Team Workflow](#13-contributing--team-workflow)
14. [Versioning](#14-versioning)
15. [Known Issues](#15-known-issues)
16. [License & Credits](#16-license--credits)

---

## 1. What This Project Is

Bako OBD ISS is a complete battery diagnostic system built around the SAE J1939 CAN bus protocol. It decodes raw CAN frames from a LiFePO₄ battery management system and presents the data in three ways: a terminal output, a standalone browser dashboard, and a real-time live monitoring interface fed directly from an ESP32 connected over USB serial.

The system was designed for three real-world use cases:

- **Field diagnostics** — connect a laptop to the battery via ESP32, open the live dashboard, and immediately see pack voltage, cell balance, temperatures, and fault status in real time.
- **Log analysis** — take a captured `.txt` CAN log from any session and analyze it offline using either the Python CLI or the web UI without needing any hardware present.
- **Development & testing** — use the ESP32 simulator to develop and test dashboard features without requiring the physical battery pack.

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Physical Layer                          │
│                                                             │
│   O'CELL BMS  ──CAN bus──  ESP32 CAN Logger  ──USB──  PC   │
│  (19S1P LFP)   250 kbps    (Arduino sketch)  115200 baud   │
└─────────────────────────────────────────────────────────────┘
                                  │
                    ┌─────────────┴──────────────┐
                    │         Software Layer      │
                    │                             │
              ┌─────▼──────┐             ┌────────▼────────┐
              │  server.py  │             │  battery_       │
              │  (FastAPI)  │             │  can_parser.py  │
              │  pyserial   │             │  (CLI, offline) │
              └─────┬───────┘             └─────────────────┘
                    │ WebSocket
                    │ 10 Hz JSON
              ┌─────▼───────┐             ┌─────────────────┐
              │  index.html  │             │  battery_       │
              │  (live HUD)  │             │  analyzer.html  │
              └──────────────┘             │  (offline UI)   │
                                           └─────────────────┘
```

The live path (left side) is the primary workflow for real hardware. The offline path (right side) works from saved `.txt` CAN log files with no hardware required.

---

## 3. Repository Structure

```
Bako_OBD_ISS/
│
├── firmware/                          # ESP32 Arduino sketches
│   ├── esp32_can_logger/              # Real CAN bus logger — flash to hardware
│   │   └── esp32_can_logger.ino
│   ├── esp32_bms_simulator/           # Simulator — no hardware needed
│   │   └── esp32_bms_simulator.ino
│   └── README.md
│
├── hardware/                          # Circuit, PCB, and mechanical design
│   ├── schematics/                    # KiCad / EasyEDA schematic source files
│   ├── pcb/                           # PCB layout + exported Gerber files
│   ├── casing/                        # 3D design files (.step / .stl / .f3d)
│   └── README.md
│
├── backend/                           # Python FastAPI server
│   ├── server.py                      # Main server — serial reader + WebSocket
│   ├── requirements.txt               # Pinned pip dependencies
│   ├── .env.example                   # Environment variable template
│   └── README.md
│
├── frontend/                          # Web interfaces
│   ├── index.html                     # Live dashboard (served by backend)
│   ├── battery_analyzer.html          # Standalone offline analyzer
│   └── README.md
│
├── data/                              # CAN logs and analysis tools
│   ├── raw/                           # Unmodified captured logs (.txt)
│   │   └── batterie.txt               # Real hardware capture session
│   ├── processed/                     # Parsed exports (.xlsx, .csv)
│   │   └── bms_log_2026-03-10.xlsx
│   ├── battery_can_parser.py          # CLI parser tool
│   └── README.md
│
├── report/                            # LaTeX ISS academic report
│   ├── main.tex                       # Entry point
│   ├── main.pdf                       # Compiled output (tracked)
│   ├── preamble.tex
│   ├── references.bib
│   ├── Preliminary Files/
│   ├── Section Files/                 # 22 sections (00–21)
│   ├── images/
│   └── README.md
│
├── docs/                              # Project documentation
│   ├── CONTRIBUTING.md                # Branch strategy, commit rules, governance
│   ├── CHANGELOG.md                   # Version history
│   └── ARCHITECTURE.md               # Deep-dive technical architecture
│
├── .github/
│   └── PULL_REQUEST_TEMPLATE.md       # PR checklist auto-filled on every PR
│
├── .gitignore
└── README.md                          # This file
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
| Capacity | 50 Ah |
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

### ESP32 Logger

The ESP32 sits between the CAN bus and the PC. It reads raw CAN frames using a MCP2515 or SN65HVD230 CAN transceiver and outputs them to USB serial in the following format:

```
[1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 0C DF 0C DE
```

Serial baud rate: **115200**

---

## 5. CAN Protocol Reference

All frame IDs use the structure `0x98_FUNC_SUB_F4` (29-bit extended J1939).

> **ID prefix note:** Protocol documentation sometimes references `0x18...` IDs. The real hardware outputs `0x98...` IDs — the top 3 bits encode J1939 priority. Both refer to the same PGN. Always use `0x98...` as the authoritative format.

### Supported Frame Types

| CAN ID | Name | Tx rate | Description |
|--------|------|---------|-------------|
| `0x98C828F4` | Cell voltages group 1 | 500 ms | Cells 1–4, big-endian uint16, 1 mV/bit |
| `0x98C928F4` | Cell voltages group 2 | 500 ms | Cells 5–8 |
| `0x98CA28F4` | Cell voltages group 3 | 500 ms | Cells 9–12 |
| `0x98CB28F4` | Cell voltages group 4 | 500 ms | Cells 13–16 |
| `0x98CC28F4` | Cell voltages group 5 | 500 ms | Cells 17–19 (bytes 6–7 = 0x0000 pad) |
| `0x98B428F4` | Temperature detail | 500 ms | Bytes 0–2: probes 1–3, `raw − 40 = °C` |
| `0x98FFE5F4` | SOC + charge request | 500 ms | LE uint16: SOC ×10, charge current req ×10 |
| `0x98FF28F4` | Pack summary | 100 ms | LE uint16: pack V ×100, disch limit ×100, SOC ×10 |
| `0x98FE28F4` | Min/max cell + temps | 100 ms | LE uint16: max/min cell mV; temp bytes; disch limit ×10 |

### Byte Layouts

**`0x98FF28F4` — Pack summary (100 ms)**
```
Bytes 0–1  LE uint16  Pack voltage       ÷ 100  → V
Bytes 2–3  LE uint16  Discharge limit    ÷ 100  → A
Bytes 4–5  LE uint16  SOC                ÷ 10   → %
Bytes 6–7  0x0000     Reserved
```

**`0x98FFE5F4` — SOC + charge request (500 ms)**
```
Bytes 0–1  LE uint16  SOC                ÷ 10   → %
Bytes 2–3  LE uint16  Charge current req ÷ 10   → A
Bytes 4–7  0x00       Reserved
```

**`0x98FE28F4` — Min/max cell + temps (100 ms)**
```
Bytes 0–1  LE uint16  Max cell voltage            → mV
Bytes 2–3  LE uint16  Min cell voltage            → mV
Byte  4    uint8      Temp probe 1  (raw − 40)    → °C
Byte  5    uint8      Temp probe 2  (raw − 40)    → °C
Bytes 6–7  LE uint16  Discharge current limit ÷ 10 → A
```

**`0x98C8–CC28F4` — Cell voltage frames (500 ms)**
```
Each frame: 4 cells × 2 bytes, big-endian uint16, unit = 1 mV
Cells beyond 19 are padded with 0x0000
```

### Voltage Thresholds

| Threshold | Value | Status |
|-----------|-------|--------|
| Cell overvoltage | ≥ 3750 mV | Critical |
| Cell full | ≥ 3650 mV | Full |
| Cell good | ≥ 3300 mV | Good |
| Cell nominal | ≥ 3200 mV | Normal |
| Cell low | ≥ 2500 mV | Low |
| Cell undervoltage | < 2500 mV | Critical |
| Pack full | 69.35 V | — |
| Pack empty | 47.50 V | — |

### Temperature Thresholds

| Range | Status | Indicator |
|-------|--------|-----------|
| < 35 °C | Normal | Cyan |
| 35–50 °C | Caution | Yellow |
| > 50 °C | Critical | Red |

---

## 6. Tools Overview

| Tool | File | Needs hardware? | Needs server? |
|------|------|-----------------|---------------|
| CLI parser | `data/battery_can_parser.py` | No — reads `.txt` log | No |
| Standalone web UI | `frontend/battery_analyzer.html` | No — upload `.txt` log | No |
| Live dashboard | `frontend/index.html` + `backend/server.py` | Yes (or simulator) | Yes |
| ESP32 simulator | `firmware/esp32_bms_simulator/` | ESP32 board only | No |

---

## 7. Quick Start

### I have real hardware and want live monitoring

```bash
# 1. Install backend dependencies
cd backend
pip install -r requirements.txt

# 2. Connect your ESP32 (CAN logger flashed) via USB

# 3. Start the server
python server.py                        # auto-detects USB port
python server.py --port COM3            # Windows — specify port
python server.py --port /dev/ttyUSB0   # Linux/macOS — specify port

# 4. Open http://localhost:8765 in your browser
```

### I have a CAN log file and want terminal output

```bash
# Edit FILE_PATH at line 6 of the script first (see section 8)
cd data
python battery_can_parser.py
```

### I have a CAN log file and want a visual dashboard

```
Open frontend/battery_analyzer.html in any modern browser
→ Click "Upload File" and select your .txt log
```

### I have no hardware and want to test the live dashboard

```bash
# 1. Flash firmware/esp32_bms_simulator/esp32_bms_simulator.ino
#    to any ESP32 board using Arduino IDE (board: ESP32 Dev Module)

# 2. Start the backend server
cd backend
python server.py

# 3. Open http://localhost:8765
```

---

## 8. Tool 1 — Python CLI Parser

**File:** `data/battery_can_parser.py`
**Requirements:** Python 3.6+ — no external dependencies

Reads a `.txt` CAN log file and prints a fully formatted battery analysis to the terminal.

### Configuration

Before running, update the file path at **line 6**:

```python
FILE_PATH = "raw/batterie.txt"       # relative path — recommended
# or absolute:
FILE_PATH = "/absolute/path/to/your/log.txt"
```

The default value is a hardcoded developer path and will fail on any other machine.

### Running

```bash
cd data
python battery_can_parser.py
```

### Example Output

```
═════════════════════════════════════════════════════════════════
  SAE J1939 BATTERY ANALYSIS  —  Based on Manufacturer Protocol
═════════════════════════════════════════════════════════════════

📊  PACK STATUS (last known values)

  Pack Voltage          : 62.6 V
  Pack Current          : 0.0 A  (neg = charging)
  State of Charge (SOC) : 26 %
  Charge Status         : IDLE / STANDBY
  Charge Cable          : Not connected
  Pack Ready            : Yes
  Fault                 : No Fault

🌡  TEMPERATURE

  Max cell temperature  : 28 °C
  Min cell temperature  : 28 °C
  Probe 1               : 28 °C
  Probe 2               : 28 °C
  Probe 3               : 28 °C

⚡  CELL VOLTAGES

  Max cell voltage      : 3.297 V
  Min cell voltage      : 3.294 V
  Spread                : 3 mV  (excellent balance)

  Cell  1  3294 mV  |████████████████████|
  Cell  2  3297 mV  |████████████████████|
  ...
  Cell 19  3295 mV  |████████████████████|

🔋  CHARGE REQUEST (BMS → Charger)

  Max charge voltage    : 71.7 V
  Max charge current    : 25.0 A
  Charge permission     : Charging ALLOWED
```

---

## 9. Tool 2 — Standalone Web Analyzer

**File:** `frontend/battery_analyzer.html`
**Requirements:** Any modern browser — no install, no server

### Opening

Double-click `battery_analyzer.html` in your file explorer, or drag it into a browser window.

### Usage

1. Click **Upload File** and select a `.txt` CAN log
2. Click **Analyze**

### Dashboard Panels

| Panel | Contents |
|-------|----------|
| Pack overview | Voltage, current, charge status — 3 KPI cards |
| SOC bar | Visual 0–100% state of charge |
| Temperature | Min/max cell temps + up to 8 individual probe readings |
| Charge request | Max charge voltage, current, and permission status |
| Cell voltage grid | All 19 cells with color-coded health bars and spread indicator |
| Raw frame log | Every decoded frame: timestamp, CAN ID, hex payload |

Click **Save as .txt** to export the full analysis as a text report.

---

## 10. Tool 3 — Live Dashboard

**Files:** `backend/server.py` + `frontend/index.html`
**Requirements:** Python 3.8+, pip packages, ESP32 connected via USB

### How It Works

```
ESP32 (CAN logger)
    │  USB serial @ 115200 baud
    ▼
server.py  ←  FastAPI + pyserial
    │  decodes CAN frames in real time (BMSState thread-safe object)
    │  WebSocket /ws  →  pushes full state JSON at 10 Hz
    ▼
index.html  ←  browser dashboard
    live cell grid, pack KPIs, temperature panel, raw log
```

### Installation

```bash
cd backend
pip install -r requirements.txt
```

### Running

```bash
python server.py                          # auto-detect USB port
python server.py --port COM3             # Windows
python server.py --port /dev/ttyUSB0    # Linux / macOS
python server.py --baud 115200           # default
python server.py --web-port 8765         # default
```

Open **http://localhost:8765**

### Auto Port Detection

The server scans connected USB devices for these chip identifiers: `CP210x`, `CH340`, `FTDI`, `Silicon Labs`, `ESP32`. To list all available ports manually:

```bash
python -c "import serial.tools.list_ports; [print(p.device, p.description) for p in serial.tools.list_ports.comports()]"
```

---

## 11. ESP32 BMS Simulator

**File:** `firmware/esp32_bms_simulator/esp32_bms_simulator.ino`
**Board:** Any ESP32 variant — **baud rate: 115200**

Simulates the exact CAN frame output of the real O'CELL BMS over USB serial. Use this to develop and test the dashboard without needing physical hardware.

### Uploading

1. Open the `.ino` file in Arduino IDE
2. Select **Tools → Board → ESP32 Dev Module**
3. Select your COM port and click Upload

### Simulated Behaviour

| Parameter | Simulation |
|-----------|------------|
| SOC | Ramps 70% → 20% (discharge) then 20% → 90% (charge), repeating |
| Pack voltage | Linear between 47.5 V (0%) and 69.35 V (100%) with ±50 mV noise |
| Cell voltages | 19 cells tracking SOC with ±8 mV individual drift |
| Temperatures | 3 probes — rise during discharge, fall during charge |
| Current | +8 A discharge, −15 A charge |
| Frame timing | Full set every 500 ms; FF28 + FE28 fast-update every 100 ms |

---

## 12. Sample Data

### `data/raw/batterie.txt`

A real CAN capture from the physical battery hardware (638 lines, ~10 seconds of idle operation).

| Parameter | Value |
|-----------|-------|
| Pack voltage | 62.6 V |
| Pack current | 0.0 A (idle) |
| SOC | 26% |
| Cell voltages | 3294–3297 mV across all 19 cells |
| Cell spread | 3 mV — excellent balance |
| Temperatures (all 3 probes) | 28 °C |
| Fault | None |
| Charge permission | Allowed |
| Max charge voltage | 71.7 V |
| Max charge current | 25.0 A |

### Decoding a Frame by Hand

```
Raw:  [1889ms] ID: 0x98FF28F4 DLC: 8 Data: 38 1A 88 13 72 02 00 00

Frame 0x98FF28F4 — Pack summary

Bytes 0–1  0x38 0x1A  →  LE = 0x1A38 = 6712  →  6712 ÷ 100 = 62.6 V
Bytes 2–3  0x88 0x13  →  LE = 0x1388 = 5000  →  5000 ÷ 100 = 50.0 A (disch limit)
Bytes 4–5  0x72 0x02  →  LE = 0x0272 = 626   →  626  ÷ 10  = 62.6 %  (secondary SOC)
Bytes 6–7  0x00 0x00  →  reserved
```

---

## 13. Contributing & Team Workflow

Full rules are in `docs/CONTRIBUTING.md`. Every collaborator must read it before their first commit.

### Branch Naming

```
feature/<domain>/<description>
bugfix/<domain>/<description>
hotfix/<domain>/<description>
release/v<MAJOR>.<MINOR>.<PATCH>
```

Domain tokens: `firmware` `hardware` `backend` `frontend` `data` `report` `docs`

### Commit Format

```
<type>(<domain>): <short description>

feat(backend): add WebSocket reconnect with exponential backoff
fix(data): correct cell voltage byte order to big-endian
docs(report): complete section 06 circuit design content
hw(hardware): update PCB rev2 with CAN transceiver decoupling caps
```

### Domain Ownership

| Domain | Folder | Owner |
|--------|--------|-------|
| Firmware | `firmware/` | TBD |
| Hardware | `hardware/` | TBD |
| Backend | `backend/` | TBD |
| Frontend | `frontend/` | TBD |
| Data | `data/` | TBD |
| Report | `report/` | TBD |

### Hard Rules

```
Never:  git push --force         on any shared branch
Never:  push directly to main    always use a PR
Never:  commit myenv/ or .env    always in .gitignore
```

Branch protection (force push disabled) must be configured in **GitHub → Settings → Branches** for both `main` and `develop` immediately after the repo is created.

---

## 14. Versioning

Semantic versioning: `vMAJOR.MINOR.PATCH`

| Increment | When |
|-----------|------|
| MAJOR | Breaking change to CAN protocol format, API, or hardware interface |
| MINOR | New backward-compatible feature |
| PATCH | Bug fix |

Current version: **v0.1.0** — 2026-03-24 — Initial structured rebuild

### Tagging a Release

```bash
git checkout main && git pull origin main
git tag -a v1.0.0 -m "Release v1.0.0 — description"
git push origin v1.0.0
```

---

## 15. Known Issues

**Cell voltage endianness in CLI parser** — `data/battery_can_parser.py` uses little-endian byte order for cell voltage frames. The actual hardware outputs cell voltages in big-endian. This produces wrong readings when the high and low bytes differ significantly. The backend server is correct. Fix: replace `le16(data[i*2], data[i*2+1])` with `(data[i*2] << 8) | data[i*2+1]` in `decode_cell_voltages()`.

**Hardcoded file path** — `FILE_PATH` at line 6 of `data/battery_can_parser.py` points to a developer's local machine. Always update before running.

**Virtual environment in original repo** — the original repository committed the full `myenv/` Python virtual environment. The `.gitignore` in this repo excludes all `venv/` folders. Never commit them.

**ISS report content** — all 22 section files in `report/Section Files/` contain only `% Content placeholder`. The LaTeX structure compiles but written content has not yet been added.

---

## 16. License & Credits

- **Project:** Bako OBD ISS — ISS Senior Project 2026
- **Institution:** MEDTECH
- **Protocol:** SAE J1939 (Society of Automotive Engineers)
- **Battery system:** O'CELL IFS60.8-500-F-E3 / Bat72 230Ah BMS
- **Repository:** [github.com/MohaBc/Bako_OBD_ISS](https://github.com/MohaBc/Bako_OBD_ISS)

---

*Last updated: March 2026 — Status: Active Development — Platforms: Windows, macOS, Linux*
