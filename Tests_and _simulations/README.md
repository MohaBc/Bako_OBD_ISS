
# BAKO SMU — Combined Backend Server (Serial + WiFi AP, runtime-switchable)
==========================================================================
CAN frame parsing updated to match BAKO CAN Protocol Rev 1.0:

BMS frames (SA=0xF4):
  0x18FF28F4  BMS Basic Msg 1 — status flags + SOC + pack current + pack voltage + fault
  0x18FE28F4  BMS Basic Msg 2 — max/min cell V + temps + max disch current
  0x18C8..CC28F4  Cell voltages (big-endian uint16 pairs)
  0x18B428F4  Temperature probes (up to 8, uint8 offset-40, 0xFF=NC)
  0x18FFE5F4  BMS charging request

New ESP32 sensor frames (SA=0xAA):
  0x18D001AA  Solar panel current before MPPT
  0x18D101AA  Solar panel voltage before MPPT
  0x18D201AA  MPPT output current + mode + efficiency
  0x18D301AA  DC/DC 12V output voltage
  0x18D401AA  Motor temperature (winding + housing)
  0x18D501AA  MPPT heatsink temperature
  0x18D601AA  Cabin interior temperature + humidity
  0x18D701AA  Handbrake position

REST API:
  GET  /          → dashboard (index.html)
  GET  /api/mode  → current mode + config
  POST /api/mode  → switch mode
  WS   /ws        → 10 Hz JSON snapshot

Requirements: pip install fastapi uvicorn pyserial
"""