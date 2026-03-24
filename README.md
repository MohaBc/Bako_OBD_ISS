
# 🔋 Bako OBD ISS — Battery CAN Analyzer

**SAE J1939 Battery Management System (BMS) Communication Protocol Decoder**

ISS Senior Project 2026 | Battery Health & Diagnostic Analysis Tool

---

## 📋 Overview

Bako OBD ISS is a comprehensive battery analysis system designed to decode and visualize CAN bus communications from LiFePO₄ (LFP) battery management systems. The project provides both a **Python command-line parser** and an **interactive web-based analyzer** to interpret SAE J1939 standard battery CAN frames.

This tool is essential for:
- **Battery Health Monitoring**: Real-time SOC, voltage, current, and temperature tracking
- **Diagnostics**: Fault detection and detailed error reporting
- **Development & Testing**: Analyze CAN frame data from battery hardware
- **Educational**: Learn SAE J1939 protocol implementation for automotive battery systems

---

## 📁 Project Structure

```
Bako_OBD_ISS/
├── battery_can_parser.py        # Python CLI parser - decodes CAN frames to console output
├── battery_analyzer.html        # Web UI - interactive visual analyzer with real-time dashboard
├── batterie.txt                 # Sample CAN frame log - Arduino raw logger output
└── README.md                    # This file
```

---

## 🚀 Features

### Core Capabilities

#### **Pack-Level Monitoring**
- **Pack Voltage** (0–100 V range, 0.1 V resolution)
- **Pack Current** (-500 to +500 A, where negative = charging)
- **State of Charge (SOC)** (0–100%)
- **Charge State Detection** (Charging / Discharging / Idle)
- **Contactor Status** (Charge & discharge contactor states)
- **Ready Signal** (Pack operational readiness)

#### **Cell-Level Diagnostics**
- **Individual Cell Voltages** (all 19 cells × 1 mV resolution)
- **Max/Min Cell Voltage** tracking
- **Cell Voltage Spread** (balance monitoring)
- **Low Cell Detection** (visual alerts)

#### **Temperature Monitoring**
- **Min/Max Cell Temperatures** (±1°C resolution)
- **Multiple Temperature Probes** (up to 8 probes)
- **Thermal Warnings** (color-coded thresholds)

#### **Charge Control**
- **Max Charge Voltage** (BMS → Charger request)
### Option 2: Web UI Analyzer

**Requirements:**
 - Any modern web browser (Chrome, Firefox, Safari, Edge)
 - Aiohttp server for live updates
- **Max Charge Current** (BMS → Charger request)
- **Charge Permission** (Allow/Block charging)

#### **Safety & Faults**
- **Multi-Level Fault Detection**
  - Level-0: No Fault ✅
  - Level-1: Critical Stop (🚨 STOP VEHICLE IMMEDIATELY)
  - Level-2+: Warning Faults ⚠️
- **Fault Codes** (detailed error identification)

---

## 📡 CAN Protocol Details

### Supported J1939 CAN IDs (250 kbps, 29-bit extended)

#### **0x18FF28F4** — BMS Basic Info Frame 1
- **Byte 0**: Status flags (charging connected, charge state, etc.)
- **Byte 1**: SOC [0–100%, 1%/bit]
- **Bytes 2–3**: Pack current [offset -500A, 0.1 A/bit]
- **Bytes 4–5**: Pack voltage [0.1 V/bit]
- **Byte 6**: Fault level
- **Byte 7**: Fault code

#### **0x18FE28F4** — BMS Basic Info Frame 2
- **Bytes 0–1**: Max cell voltage [mV]
- **Bytes 2–3**: Min cell voltage [mV]
- **Byte 4**: Max cell temperature [offset -40°C, 1°C/bit]
- **Byte 5**: Min cell temperature [offset -40°C, 1°C/bit]
- **Bytes 6–7**: Max discharge current [0.1 A/bit]

#### **0x18B428F4** — Temperature Detail Frame
- **Bytes 0–7**: Temperature probes [offset -40°C, 1°C/bit each]

#### **0x18FFE5F4** — Charge Request Frame
- **Bytes 0–1**: Max charge voltage [0.1 V/bit]
- **Bytes 2–3**: Max charge current [0.1 A/bit]
- **Byte 4**: Charge control flag (bit 1 = block charging)

#### **0x18C828F4–0x18CD28F4** — Cell Voltage Detail Frames
- **4 frames, 8 bytes each**
- **2 bytes per cell** (4 cells/frame, 19 cells total)
- **Resolution**: 1 mV/bit

---

## 🛠️ Usage

### Option 1: Python CLI Parser

**Requirements:**
- Python 3.6+
- No external dependencies

**Running the Parser:**

```bash
python battery_can_parser.py
```

If you want to read CAN frames live from a serial monitor (e.g. Arduino serial output or an RS-232/USB-CAN bridge), install `pyserial` and run the parser with `--serial`.

```bash
# install dependency
pip install -r requirements.txt

# read from a serial port (example on Linux)
python battery_can_parser.py --serial /dev/ttyUSB0 --baud 115200

# on Windows the port might be COM3:
python battery_can_parser.py --serial COM3 --baud 115200
```

**What it does:**
1. Reads the CAN log file specified in `FILE_PATH` (default: `batterie.txt`)
2. Decodes all J1939 frames
3. Prints formatted analysis to console with:
   - Pack voltage, current, SOC
   - Temperature readings
   - Cell voltage visualization with bar charts
   - Charge request details
   - Fault status

**Example Output:**
```
═════════════════════════════════════════════════════════════
  SAE J1939 BATTERY ANALYSIS  —  Based on Manufacturer Protocol
═════════════════════════════════════════════════════════════

📊  PACK STATUS (last known values)

  🔌 Pack Voltage          : 76.8 V
  ⚡ Pack Current          : -50.0 A  (neg = charging)
  📈 State of Charge (SOC) : 85 %
  ⚙️  Charge Status         : 🔋 CHARGING
  🔌 Charge Cable          : Connected
  🟢 Pack Ready            : Yes
  🚨 Fault                 : ✅ No Fault

🌡  TEMPERATURE

  Max cell temperature     : 35 °C
  Min cell temperature     : 28 °C
  Probe 1                  : 32 °C
  Probe 2                  : 31 °C

⚡  CELL VOLTAGES

  Max cell voltage         : 3.313 V
  Min cell voltage         : 3.278 V
  Cell  1  3313 mV  |████████████████████|
  Cell  2  3301 mV  |███████████████████░|
  Cell  3  3290 mV  |██████████████████░░|
  ...
```

### Option 2: Web UI Analyzer

**Requirements:**
- Any modern web browser (Chrome, Firefox, Safari, Edge)
- No server needed (pure client-side HTML/JavaScript)

**Opening the Tool:**

1. **Option A**: Double-click `battery_analyzer.html` in your file explorer
2. **Option B**: Open in browser: `File > Open > Select battery_analyzer.html`
3. **Option C**: Drag and drop the file onto your browser window

**Using the Web UI:**

1. **Load a CAN Log File**:
   - Enter file path in the input box, then click **▶ Analyze**
   - OR click **⬆ Upload File** to select from your computer

2. **View the Dashboard**:
   - 📊 **Pack Overview**: Voltage, Current, Charge Status (3 KPI cards)
   - 📈 **State of Charge**: Visual percentage bar
   - 🌡️ **Temperature Panel**: Min/Max/Probe readings
   - 🔋 **Charge Request**: Max voltage, current, permission status
   - ⚡ **Cell Voltages**: Individual cell readings with color-coded health bars
   - 📋 **Raw CAN Frames**: All parsed frames with syntax highlighting

3. **Interactive Features**:
   - **Live Status Indicator**: Green dot = data loaded
   - **Color Coding**: Green (good), Yellow (warning), Red (critical)
   - **Health Visualization**: Cell voltage bars indicate charge levels
   - **Fault Banner**: Prominent alert system for errors

4. **Export Analysis**:
   - Click **💾 Save as .txt** to export the analysis report
   - Click **🗑 Clear Data** to reset the interface

---

## 📊 Dashboard Components

### Pack Overview (KPI Cards)
- **Pack Voltage**: Current pack voltage with nominal 76.8V reference
- **Pack Current**: Positive (discharge) or negative (charging) current
- **Charge Status**: Current operational state with cable/ready indicators

### SOC (State of Charge) Bar
- Visual percentage indicator (0–100%)
- Color gradient from empty to full

### Temperature Section
- **Min/Max Cell Temps**: Overall pack temperature range
- **Temp Probes**: Individual sensor readings (up to 8)
- **Color Warnings**: Red (>50°C), Yellow (35–50°C), Cyan (<35°C)

### Cell Voltage Grid
- **All 19 Cells** displayed in real-time
- **Spread Calculation**: Max − Min voltage difference
- **Health Bars**: Visual representation (green = healthy)
- **Imbalance Detection**: Low cells highlighted in red

### Charge Request Card
- **Max Charge Voltage**: Requested voltage limit
- **Max Charge Current**: Requested current limit
- **Permission Status**: Charging allowed or blocked

### Raw CAN Frames Panel
- **Total Frames**: Count of all CAN messages in log
- **Matched Frames**: Count of decoded protocol frames
- **Frame Details**: Timestamp, CAN ID, payload hex dump

---

## 📝 Input File Format

The tool expects CAN log files in the following format:

```
========================================
       RAW CAN LOGGER - ARDUINO
========================================
[1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 0C DF 0C DE
[1854ms] ID: 0x98C928F4 DLC: 8 Data: 0C DF 0C DF 0C E0 0C DF
[1859ms] ID: 0x98CA28F4 DLC: 8 Data: 0C DE 0C DF 0C E0 0C E0
```

**Format Specification:**
- `[XXXms]` — Timestamp in milliseconds
- `ID: 0xXXXXXXXX` — 29-bit CAN extended ID in hexadecimal
- `DLC: X` — Data length (typically 8)
- `Data: XX XX XX XX XX XX XX XX` — Payload bytes in hex (space-separated)

---

## 🔧 Configuration

### Python Parser
Edit `battery_can_parser.py` line 6:
```python
FILE_PATH = "C:/Users/legio/Desktop/Bako/batterie.txt"
```
Change to your CAN log file path.

### Web UI
- **File Input Field**: Enter path or click upload button
- **No configuration needed** — works entirely client-side

---

## ⚙️ Technical Details

### Protocol Specifications
- **Standard**: SAE J1939 automotive CAN
- **Baud Rate**: 250 kbps
- **Frame Type**: 29-bit extended CAN IDs
- **Battery Chemistry**: LiFePO₄ (LFP)
- **Pack Configuration**: 19S1P (19 cells in series)
- **Cell Type**: 230 Ah LiFePO₄

### Voltage Ranges
- **Pack Nominal**: 76.8 V (19S × 4.04 V)
- **Pack Min**: ~60 V (19S × 3.15 V)
- **Pack Max**: ~87.6 V (19S × 4.61 V)
- **Cell Range**: 3.0–3.65 V (LFP typical)

### Current Limits
- **Max Discharge**: 200 A (continuous)
- **Max Charge**: 100 A (typical)
- **Current Offset**: -500 A (raw value 0 = -500 A)

### Temperature Thresholds
- **Normal**: <35°C (blue)
- **Caution**: 35–50°C (yellow)
- **Critical**: >50°C (red)
- **Min Safe**: -40°C to +85°C

---

## 🐛 Troubleshooting

### Python Parser Issues

**"FileNotFoundError: No such file or directory"**
- Check that `FILE_PATH` in the script points to a valid CAN log file
- Use absolute paths (C:/... or /path/to/...)

**"No output or empty analysis"**
- Verify the log file contains valid CAN frames in the expected format
- Check that frame IDs match supported J1939 protocol (0x18FF28F4, etc.)

### Web UI Issues

**"Could not fetch file from server"**
- The HTML tries to load files as web assets (requires a server)
- **Solution**: Use the **Upload File** button instead to load directly from your computer

**Charts not appearing**
- Try clearing browser cache (Ctrl+Shift+Delete)
- Ensure JavaScript is enabled

**Copy/Paste test data**
- Manually paste CAN frame data using the upload feature

---

## 📚 Example: Interpreting a Frame

**Raw Frame:**
```
[1889ms] ID: 0x98FF28F4 DLC: 8 Data: 38 1A 88 13 72 02 00 00
```

**Decoded:**
- **CID**: 0x98FF28F4 → **FF28F4** (BMS Basic Info Frame 1)
- **Byte 0** (0x38 = 56): Status flags
  - Bit 0 (Charge Wire): 0 = Not connected
  - Bit 1 (Is Charging): 0 = Not charging
  - Bit 3 (Ready): 1 = Pack ready
  - Bit 4 (Discharge Contactor): 1 = Enabled
- **Byte 1** (0x1A = 26): SOC = **26%**
- **Bytes 2–3** (0x88 0x13 = 0x1388 = 5000): Current = 5000 × 0.1 − 500 = **0 A**
- **Bytes 4–5** (0x72 0x02 = 0x0272 = 626): Voltage = 626 × 0.1 = **62.6 V**
- **Bytes 6–7** (0x00 0x00): No fault

---

## 🎯 Use Cases

1. **Battery Debugging**: Identify cell imbalances, temperature issues, or faults
2. **Development Testing**: Verify BMS communication during hardware testing
3. **Data Logging**: Record battery behavior during charge/discharge cycles
4. **Diagnostics**: Quickly assess battery health from CAN frame data
5. **Integration Testing**: Validate charger/battery communication handshakes
6. **Field Service**: Analyze customer battery data for warranty claims

---

## 📄 License & Credits

- **Project**: Bako OBD ISS (2026)
- **Protocol**: SAE J1939 Standard
- **Battery System**: Bat72 230Ah BMS
- **Repository**: [GitHub/MohaBc/Bako_OBD_ISS](https://github.com/MohaBc/Bako_OBD_ISS)

---

## 🤝 Contributing

For issues, improvements, or additional CAN frame types:
1. Document the new frame structure
2. Add decoder function (Python + JavaScript)
3. Add UI visualization component
4. Test with real CAN logs

---

## 📞 Support

For questions about:
- **CAN Protocol**: Refer to SAE J1939 standard documentation
- **BMS Communication**: Check Bat72 protocol specifications
- **Tool Usage**: See examples in this README and sample `batterie.txt`

---

**Last Updated**: March 2026  
**Status**: Active Development  
**Supported Platforms**: Windows, macOS, Linux (Python & HTML5)