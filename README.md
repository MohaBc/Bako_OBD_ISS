# 🔋 Bako OBD ISS — Battery CAN Analyzer

**SAE J1939 Battery Management System (BMS) Communication Protocol Decoder**

ISS Senior Project 2026 | Battery Health & Diagnostic Analysis Tool | O'CELL IFS60.8-500-F-E3

---

## 📋 Overview

Bako OBD ISS is a comprehensive battery analysis system designed to decode and visualize CAN bus communications from an **O'CELL IFS60.8-500-F-E3 19S1P LiFePO₄ battery pack** (60.8V / 50Ah). The project provides three complementary tools to interpret SAE J1939 standard battery CAN frames:

| Tool | File | Mode | Use Case |
|------|------|------|----------|
| Python CLI Parser | `battery_can_parser.py` | Offline | Quick terminal analysis from a `.txt` log file |
| Standalone Web UI | `battery_analyzer.html` | Offline | Interactive browser dashboard, no server needed |
| Live Dashboard | `Bako_diagnostic_interface_v2/` | Live | Real-time monitoring via ESP32 USB serial + WebSocket |

---

## 📁 Project Structure

```
Bako_OBD_ISS/
│
├── battery_can_parser.py              # Python CLI parser — decodes CAN log to terminal
├── battery_analyzer.html              # Standalone web UI — no server, upload & analyze
├── batterie.txt                       # Real captured CAN log from hardware (638 lines)
├── bms_log_2026-03-10T10-20-02.txt   # Additional real BMS session log
├── bms_log_2026-03-10T10-20-06.xlsx  # Session log exported as Excel
├── README.md                          # This file
│
├── Bako_diagnostic_interface/         # v1 — Live dashboard (server + frontend)
│   ├── server.py                      # FastAPI + WebSocket backend
│   ├── index.html                     # Dark HUD dashboard frontend
│   └── esp32_bms_simulator/
│       └── esp32_bms_simulator.ino    # ESP32 Arduino simulator sketch
│
├── Bako_diagnostic_interface_v2/      # v2 — Recommended version (multi-format support)
│   ├── server.py                      # Same as v1 + normalize_line() for multiple CAN loggers
│   ├── index.html                     # Same dashboard UI
│   └── temp                           # Temporary scratch file (ignore)
│
└── ISS_Report/                        # LaTeX academic report (XeLaTeX)
    ├── main.tex                        # Entry point
    ├── main.pdf                        # Compiled output
    ├── preamble.tex
    ├── references.bib
    ├── Preliminary Files/             # Abstract, acknowledgements, declaration
    └── Section Files/                 # 22 sections (00–21) + appendices
```

---

## 🔋 Target Hardware

| Property | Value |
|----------|-------|
| Battery model | O'CELL IFS60.8-500-F-E3 |
| Chemistry | LiFePO₄ (LFP) |
| Configuration | 19S1P (19 cells in series) |
| Nominal voltage | 60.8 V |
| Capacity | 50 Ah |
| CAN standard | SAE J1939, 29-bit extended IDs |
| CAN baud rate | 250 kbps |
| ESP32 serial baud | 115200 |

---

## 📡 CAN Protocol Reference

All frame IDs use the format `0x98_FUNC_SUB_F4` (29-bit extended J1939).

> **Note:** The README and protocol docs reference both `0x18...` and `0x98...` prefix variants. The real hardware outputs `0x98...` IDs (priority bits set). The server and simulator use `0x98...`. Use `0x98...` as the authoritative format.

### Frame Map

| CAN ID | Name | Rate | Contents |
|--------|------|------|----------|
| `0x98C828F4`–`0x98CC28F4` | Cell voltages 1–19 | 500 ms | 4 cells × 2 bytes each, big-endian uint16, unit = 1 mV |
| `0x98B428F4` | Temperature detail | 500 ms | Bytes 0–2: probes 1–3, `raw − 40 = °C` |
| `0x98FFE5F4` | SOC + charge request | 500 ms | LE uint16: SOC × 10, charge current request × 10 |
| `0x98FF28F4` | Pack summary | 100 ms | LE uint16: pack V × 100, discharge limit × 100, SOC × 10 |
| `0x98FE28F4` | Min/max cell + temps | 100 ms | LE uint16: max cell mV, min cell mV; temps; discharge limit × 10 |

### Cell Voltage Frames — Byte Layout

Each cell voltage frame carries 4 cells, **big-endian** uint16, 1 mV per bit:

| Frame ID | Cells |
|----------|-------|
| `0x98C828F4` | 1–4 |
| `0x98C928F4` | 5–8 |
| `0x98CA28F4` | 9–12 |
| `0x98CB28F4` | 13–16 |
| `0x98CC28F4` | 17–19 (bytes 6–7 = 0x0000 pad) |

### Voltage & Current Thresholds

| Parameter | Value |
|-----------|-------|
| Cell overvoltage | ≥ 3750 mV |
| Cell full | ≥ 3650 mV |
| Cell good | ≥ 3300 mV |
| Cell nominal | ≥ 3200 mV |
| Cell low | ≥ 2500 mV |
| Cell undervoltage | < 2500 mV |
| Pack full voltage | 69.35 V |
| Pack empty voltage | 47.50 V |
| Max charge current | 25 A |
| Max discharge current | 50 A |

### Temperature Thresholds

| Range | Status |
|-------|--------|
| < 35 °C | Normal (cyan) |
| 35–50 °C | Caution (yellow) |
| > 50 °C | Critical (red) |

---

## 🛠️ Tool 1 — Python CLI Parser (`battery_can_parser.py`)

A zero-dependency Python 3 script that reads a `.txt` CAN log and prints a formatted analysis to the terminal.

### Setup

```bash
# No pip installs needed — uses only Python standard library
python battery_can_parser.py
```

### ⚠️ Configuration Required

Before running, edit **line 6** of `battery_can_parser.py` to point to your log file:

```python
# Change this to your actual file path:
FILE_PATH = "batterie.txt"          # relative path (recommended)
# or
FILE_PATH = "/absolute/path/to/batterie.txt"
```

The default is hardcoded to a developer's local machine path and will fail on any other system.

### Input File Format

```
========================================
       RAW CAN LOGGER - ARDUINO
========================================
[1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 0C DF 0C DE
[1854ms] ID: 0x98C928F4 DLC: 8 Data: 0C DF 0C DF 0C E0 0C DF
```

- `[XXXms]` — timestamp in milliseconds
- `ID: 0xXXXXXXXX` — 29-bit extended CAN ID
- `DLC: X` — data length (8 for all supported frames)
- `Data: XX XX ...` — payload bytes, space-separated hex

### Example Output

```
═════════════════════════════════════════════════════════════════
  SAE J1939 BATTERY ANALYSIS  —  Based on Manufacturer Protocol
═════════════════════════════════════════════════════════════════

📊  PACK STATUS (last known values)

  🔌 Pack Voltage          : 62.6 V
  ⚡ Pack Current          : 0.0 A  (neg = charging)
  📈 State of Charge (SOC) : 26 %
  ⚙️  Charge Status         : ⏸  IDLE / STANDBY
  🔌 Charge Cable          : Not connected
  🟢 Pack Ready            : Yes
  🚨 Fault                 : ✅ No Fault

🌡  TEMPERATURE

  Max cell temperature     : 28 °C
  Min cell temperature     : 28 °C
  Probe 1                  : 28 °C
  Probe 2                  : 28 °C
  Probe 3                  : 28 °C

⚡  CELL VOLTAGES

  Max cell voltage         : 3.297 V
  Min cell voltage         : 3.294 V
  Cell  1  3294 mV  |████████████████████|
  Cell  2  3297 mV  |████████████████████|
  ...
```

---

## 🌐 Tool 2 — Standalone Web UI (`battery_analyzer.html`)

A fully client-side single-file HTML/JavaScript dashboard. No server, no install — open directly in any modern browser.

### Opening

```bash
# Option A: double-click battery_analyzer.html in your file explorer
# Option B:
open battery_analyzer.html          # macOS
xdg-open battery_analyzer.html      # Linux
start battery_analyzer.html         # Windows
```

### Features

- Upload a `.txt` CAN log file via the file picker
- **Pack Overview** — Voltage, current, charge status KPI cards
- **SOC Bar** — Visual 0–100% state of charge indicator
- **Temperature Panel** — Min/max + up to 8 individual probes
- **Charge Request Card** — Max charge voltage, current, and permission status
- **Cell Voltage Grid** — All 19 cells with color-coded health bars
- **Raw CAN Frame Log** — Timestamp, ID, hex payload for every decoded frame
- **Export** — Save full analysis as `.txt`

---

## ⚡ Tool 3 — Live Dashboard (`Bako_diagnostic_interface_v2/`) ← Recommended

Real-time BMS monitoring over a live ESP32 USB serial connection. The server decodes incoming CAN frames and pushes updates to the browser via WebSocket at **10 Hz**.

### Requirements

```bash
pip install fastapi uvicorn pyserial
```

### Running

```bash
cd Bako_diagnostic_interface_v2

python server.py                        # auto-detect USB port
python server.py --port COM3            # Windows
python server.py --port /dev/ttyUSB0    # Linux/macOS
python server.py --baud 115200          # default baud rate
python server.py --web-port 8765        # default web port
```

Then open **http://localhost:8765** in your browser.

### Architecture

```
ESP32 (CAN logger)
    │  USB Serial @ 115200 baud
    ▼
server.py  (FastAPI + pyserial)
    │  WebSocket /ws  @ 10 Hz JSON push
    ▼
index.html (browser dashboard)
```

The `BMSState` class holds all decoded values thread-safely (with a `threading.Lock`). The serial reader runs in a background daemon thread; the WebSocket handler runs in the async FastAPI event loop.

### v2 vs v1 — What Changed

v2 adds `normalize_line()`, which makes the server compatible with multiple CAN logger formats:

| Format | Example | Support |
|--------|---------|---------|
| ESP32 native (correct) | `[1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE...` | v1 + v2 |
| Other CAN tools | `Extended ID: 0x18FE28F4  DLC: 8  Data: 0x43 0x0D...` | v2 only |

Use **v2** unless you have a specific reason to use v1.

---

## 🧪 ESP32 BMS Simulator (`esp32_bms_simulator.ino`)

An Arduino sketch that simulates the exact CAN frame output of the real BMS over USB serial — useful for testing the live dashboard without physical hardware.

### Upload & Run

1. Open `Bako_diagnostic_interface/esp32_bms_simulator/esp32_bms_simulator.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module** (or any ESP32 variant)
3. Upload and open Serial Monitor at **115200 baud**
4. Run `server.py` — it will connect to the simulator just like real hardware

### Simulation Behaviour

| Parameter | Behaviour |
|-----------|-----------|
| SOC | Ramps 70% → 20% (discharge), then 20% → 90% (charge), cycling |
| Pack voltage | Linear between 47.5V (0%) and 69.35V (100%) + ±50mV noise |
| Cell voltages | Track SOC with ±8mV individual drift per cell per cycle |
| Temperature | 3 probes rise at +0.05°C/cycle during discharge, −0.03°C/cycle during charge |
| Current | +8A (discharge) or −15A (charge) |
| Frame timing | Full set (all frames) every 500ms; FF28 + FE28 repeated every 100ms |

---

## 📊 Sample Data — `batterie.txt`

This is a real capture from the physical battery hardware. Key values from this session:

| Parameter | Value |
|-----------|-------|
| Pack voltage | 62.6 V |
| Pack current | 0.0 A (idle) |
| SOC | ~26% |
| All cell voltages | 3294–3297 mV (3 mV spread — excellent balance) |
| All temperatures | 28 °C across 3 probes |
| Fault | None |
| Charge permission | Allowed |
| Max charge voltage | 71.7 V |
| Max charge current | 25.0 A |

---

## ⚙️ Technical Notes & Known Issues

### Cell Voltage Endianness
Cell voltage frames (`0x98C8–CC28F4`) must be parsed as **big-endian** uint16. The live server and simulator both use big-endian correctly. The standalone CLI parser (`battery_can_parser.py`) uses a little-endian helper (`le16`) for cell frames — this is inconsistent with the actual hardware data and may produce incorrect cell mV values when the high and low bytes differ significantly.

### Hardcoded File Path
`battery_can_parser.py` has `FILE_PATH` hardcoded to a developer's local Windows path on line 6. Always update this before running.

### Frame ID Prefix (`0x18` vs `0x98`)
The protocol documentation uses `0x18XXXXXX` IDs. The actual hardware outputs `0x98XXXXXX` IDs (the top 3 priority bits differ). Both represent the same PGN — the server masks correctly using `func = (can_id >> 16) & 0xFF`. The CLI parser uses `endswith()` string matching which handles both variants.

### ISS Report Status
The LaTeX report structure (`ISS_Report/`) is complete and compiles successfully to PDF. All 22 section files currently contain only `% Content placeholder` — the written content has not yet been committed to the repository.

---

## 🚀 Quick Start Guide

**I have real hardware and want live monitoring:**
```bash
cd Bako_diagnostic_interface_v2
pip install fastapi uvicorn pyserial
python server.py
# Open http://localhost:8765
```

**I have a CAN log file and want a quick analysis:**
```bash
# Edit FILE_PATH in battery_can_parser.py first, then:
python battery_can_parser.py
```

**I have a CAN log file and want a visual dashboard:**
```bash
# Open battery_analyzer.html in your browser, then upload the .txt file
```

**I have no hardware and want to test the live dashboard:**
```bash
# 1. Flash esp32_bms_simulator.ino to an ESP32
# 2. cd Bako_diagnostic_interface_v2
# 3. python server.py
# 4. Open http://localhost:8765
```

---

## 📚 Protocol Reference

- **SAE J1939** — Automotive CAN standard for commercial vehicles and industrial equipment
- **Battery model** — O'CELL / Bat72 230Ah BMS communication protocol
- **CAN frame type** — 29-bit extended IDs at 250 kbps

---

## 📄 License & Credits

- **Project**: Bako OBD ISS (2026) — ISS Senior Project
- **Institution**: MEDTECH
- **Protocol**: SAE J1939 Standard
- **Battery System**: O'CELL IFS60.8-500-F-E3 / Bat72 230Ah BMS
- **Repository**: [github.com/MohaBc/Bako_OBD_ISS](https://github.com/MohaBc/Bako_OBD_ISS)

---

**Last Updated**: March 2026 | **Status**: Active Development | **Platforms**: Windows, macOS, Linux