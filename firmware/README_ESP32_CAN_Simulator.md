# 🚗 ESP32 CAN Bus Simulator for EV Diagnostics

## 📌 Overview
This project uses an **ESP32** to simulate CAN bus messages from an electric vehicle Battery Management System (BMS).

It is designed to:
- Generate realistic CAN frames
- Emulate a 19S LiFePO4 battery pack


## ⚙️ Hardware Requirements
- ESP32
- MCP2515 CAN module
- CAN transceiver (TJA1050 or similar)
- Jumper wires

---

## 🔌 Wiring (ESP32 ↔ MCP2515)
| MCP2515 | ESP32 |
|--------|------|
| VCC    | 5V / 3.3V |
| GND    | GND |
| CS     | GPIO 5 |
| SCK    | GPIO 18 |
| MOSI   | GPIO 23 |
| MISO   | GPIO 19 |
| INT    | GPIO 4 |

---

## 📡 CAN Configuration
- **Baud rate:** 250 kbps
- **Frame type:** Extended (29-bit)
- **Protocol:** Custom BMS protocol

---

## 📦 Features
- Simulates:
  - Cell voltages (19 cells)
  - Pack voltage
  - State of Charge (SOC)
  - Charge request current
  - Discharge limit
  - Temperatures
- Multi-phase simulation (dynamic values)
- Serial output logging (for debugging & backend parsing)

## ❗7 Phases for simulation

  ## 0 REST 20s 
  All temps converge to ~24°C, cells tightly balanced
  ## 1 BULK DISCHARGE 120s 
  38A load — temps climb, cells 7 & 14 visibly sag first
  ## 2 LOW SOC WARNING 25s 
  BMS throttles from 38A → 12A, discharge limit drops 50→15A, charge request jumps to 25A
  ## 3 RECOVERY 15s 
  Load off — cells bounce back 10–20mV (polarisation), temps plateau
  ## 4 CC CHARGE 90s
  22A constant, SOC climbs, mild thermal rise then plateau
  ## 5 CV TAPER40s
  Current tapers exponentially 22A → 2A, cell spread narrows
  ## 6 FULL REST20s
  SOC locked at 100%, zero charge request, pack cools
  

## 🧠 CAN Frames Structure

### 🔋 Cell Voltages (0x98C8–0x98CC)
- 4 cells per frame
- Format: **Big Endian (uint16)**
- Unit: mV

---

### 🌡️ Temperature (0x98B4)
- 3 sensors
- Formula:
Temperature = Byte - 40

---

### 🔌 SOC + Charge Request (0x98FFE5)
- SOC = uint16 / 10
- Charge current = uint16 / 10

---

### ⚡ Pack Data (0x98FF28)
- Voltage = uint16 / 100
- Discharge limit = uint16 / 100
- SOC = uint16 / 10

---

### 📊 Min / Max Cells (0x98FE28)
- Max cell voltage
- Min cell voltage
- Temperature
- Discharge limit

---

## 🚀 How to Use

1. Open the project in Arduino IDE
2. Install required libraries:
   - mcp_can
   - SPI
3. Select ESP32 board
4. Upload the code
5. Open Serial Monitor (115200 baud)

---

## 🧪 Example Output
[1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 ...

---

## 🎯 Purpose
This simulator is used for:
- Testing EV diagnostic software
- Debugging CAN decoding logic
- Demonstrating BMS behavior

---


