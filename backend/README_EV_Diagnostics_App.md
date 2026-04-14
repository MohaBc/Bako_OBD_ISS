# 🧪 EV Battery Diagnostics Dashboard

## 📌 Overview
This application provides a **real-time diagnostics dashboard** for an electric vehicle battery system.

It connects to an ESP32 CAN simulator via serial, decodes CAN frames, and visualizes battery data in a web interface.

---

## 🧱 Architecture

ESP32 (CAN Simulator)
        ↓ Serial (UART)
Python Backend (FastAPI)
        ↓ WebSocket
Web Dashboard (HTML + JS)

---

## ⚙️ Backend (Python)

### 🔧 Requirements
pip install fastapi uvicorn pyserial

---

### ▶️ Run the Server
python server.py

Optional:
python server.py --port COM3
python server.py --baud 115200

---

### 🌐 Access Dashboard
http://localhost:8765

---

## 🧠 Backend Features
- Serial communication with ESP32
- CAN frame parsing
- Real-time state tracking
- WebSocket streaming (10 Hz)
- Auto port detection

---

## 📊 Decoded Data
- State of Charge (SOC)
- Pack voltage
- Charge / discharge currents
- Cell voltages (19 cells)
- Temperature sensors
- Min / max cell analysis

---

## 🖥️ Frontend Features

### 🔋 Battery Overview
- SOC circular indicator
- Pack voltage
- Current limits

### ⚡ Cell Monitoring
- Individual cell voltages
- Color-coded health status
- Min/Max highlighting
- Voltage spread calculation

### 🌡️ Temperature Monitoring
- Multiple sensors
- Color-coded ranges
- Average temperature

### 📜 Serial Log Panel
- Live CAN frame stream
- Export options:
  - TXT
  - XLSX (structured data)

---

## 📡 Data Flow

1. ESP32 sends CAN logs via Serial
2. Backend parses frames
3. Data stored in BMS state
4. WebSocket sends JSON updates
5. Frontend updates UI in real time

---

## 📁 Project Structure
/project
│── server.py
│── index.html
│── README.md

---

## 🚀 Key Highlights
- Real-time visualization
- Clean and modern UI
- Efficient CAN decoding
- Exportable diagnostics data
- Fully testable without real vehicle

---

## 🧪 Use Cases
- EV diagnostics development
- BMS testing & validation
- Embedded systems demonstration
- CAN protocol learning




