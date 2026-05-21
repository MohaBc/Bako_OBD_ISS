# BAKO Backend — Server Reference

> **VPS quick-start:** jump to [Starting the Server](#starting-the-server).

---

## Server file

```
backend/server_frame_parser_Json.py     ← the only production server
```

The old files `server_frame_parser.py` and `server_all_no_frames.py` are archived and not used.

---

## Installation

```bash
cd /home/bako/backend                  # or wherever you cloned the repo
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`requirements.txt` pins:
```
fastapi==0.115.0
uvicorn==0.29.0
pyserial==3.5
```

---

## Starting the Server

```bash
# Default — port 8787, all interfaces
python3 server_frame_parser_Json.py

# Custom port (e.g. 8787 on the VPS)
python3 server_frame_parser_Json.py --web-port 8787

# Bind to localhost only
python3 server_frame_parser_Json.py --host 127.0.0.1 --web-port 8787
```

**Supported arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--host` | `0.0.0.0` | Bind address |
| `--web-port` | `8787` | TCP port |

> There is no `--cloud-only`, `--port`, or `--mode` argument.
> Mode (serial / wifi / off) is set via the dashboard UI or the `/api/mode` REST call.

---

## Operating Modes

The server supports three reader modes, persisted in `mode_config.json`:

| Mode | Description |
|------|-------------|
| `serial` | Reads CAN frames from a USB-connected ESP32 (auto-detects port) |
| `wifi` | Opens a TCP client socket to the ESP32 AP at `192.168.4.1:9000` |
| `off` | No reader; server still accepts `/api/ingest` POSTs from the ESP32 |

**On the VPS** the correct mode is `off` (no USB or Wi-Fi — the ESP32 pushes data over cellular):

```bash
curl -s -X POST http://localhost:8787/api/mode \
  -H "Content-Type: application/json" \
  -d '{"mode": "off"}'
```

This is saved to `mode_config.json` and survives restarts.

---

## API Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/` | Serve the dashboard (`../frontend/index.html`) |
| `GET` | `/api/mode` | Return current mode + config + available serial ports |
| `POST` | `/api/mode` | Switch mode at runtime (body: `{"mode": "serial"\|"wifi"\|"off"}`) |
| `POST` | `/api/ingest` | Receive a BMS JSON snapshot from the ESP32 (requires `X-Api-Key` header) |
| `GET` | `/api/latest_push` | Return the most recent snapshot received via `/api/ingest` |
| `WS` | `/ws` | WebSocket — pushes the full server state at 10 Hz |

### Authentication for `/api/ingest`

The ESP32 must include the header `X-Api-Key: bako-bms-2024` (default).
Override the key by setting the environment variable `BMS_API_KEY` before starting the server:

```bash
BMS_API_KEY=my-secret python3 server_frame_parser_Json.py --web-port 8787
```

---

## Data Persistence

The current server keeps all received data **in memory only**.
Data is lost when the server restarts. There is no automatic SQLite write in `server_frame_parser_Json.py`.

The file `bms_cloud.db` in this directory was created by an earlier version of the server and can be ignored or deleted.

To inspect or query what was received during a session, use `/api/latest_push`:

```bash
curl -s http://localhost:8787/api/latest_push | python3 -m json.tool
```

---

## Testing the Ingest Endpoint (without hardware)

```bash
curl -X POST http://localhost:8787/api/ingest \
  -H "Content-Type: application/json" \
  -H "X-Api-Key: bako-bms-2024" \
  -d '{
    "device_id": "test",
    "connected": true,
    "fault_level": 0,
    "error_code": 0,
    "battery": {
      "soc": 87.4,
      "pack_v": 62.3,
      "pack_current_a": -15.1,
      "cell_count": 19,
      "cells": [null, {"mv": 3280}, {"mv": 3295}]
    },
    "temperatures": {
      "battery_avg_c": 21.0,
      "battery_cells": [null, 21.0, 22.0, 21.0, 20.0]
    }
  }'
```

Expected response: `{"ok": true}`

---

## Replaying a Log File to the Server (no ESP32 needed)

```bash
# Replay the captured vehicle log at 5× speed
python3 replay_to_cloud.py --speed 5

# Point at the VPS instead of localhost
python3 replay_to_cloud.py --host YOUR_VPS_IP --port 8787 --speed 5

# Loop forever
python3 replay_to_cloud.py --speed 5 --loop
```

---

## Keeping the Server Running (VPS)

```bash
# Run in background with nohup
nohup python3 server_frame_parser_Json.py --web-port 8787 > server.log 2>&1 &
echo $! > server.pid

# Stop it
kill $(cat server.pid)

# Check if it's running
ps aux | grep server_frame_parser_Json
```

---

## Port Already in Use

```bash
lsof -ti :8787 | xargs kill -9
```
