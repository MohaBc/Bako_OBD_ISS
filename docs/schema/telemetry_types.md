# Telemetry Payload — Field Reference v2.0.0

> Share this document with frontend/interface developers.  
> Source of truth: [`telemetry.schema.json`](telemetry.schema.json)

---

## Full example payload

```json
{
  "schema_version": "2.0.0",
  "vehicle_id": 1,
  "ts": "2026-04-16T08:51:04.522Z",
  "seq": 2266,

  "battery": {
    "pack_v": 63.2,
    "pack_current_a": 0.0,
    "soc": 93.2,
    "soc_bms": 74,
    "max_disch_a": 100,
    "cell_count": 19,
    "cell_avg_mv": 3327,
    "cell_min_mv": 3325,
    "cell_max_mv": 3330,
    "cell_spread_mv": 5,
    "cells": [3328, 3329, 3329, 3329, 3329, 3329, 3329, 3328, 3329, 3329, 3329, 3330, 3329, 3329, 3325, 3326, 3326, 3326, 3325],
    "charger": { "max_charge_v": 69.3, "max_charge_a": 35, "enable": true },
    "status": {
      "fault_level": 0, "error_code": 0,
      "ready": true, "charging": false, "discharging": false,
      "charge_cable": false, "charge_contactor": true, "disch_contactor": true
    }
  },

  "temperatures": {
    "battery_avg_c": 20.5, "battery_min_c": 20.0, "battery_max_c": 21.0,
    "dcdc_c": null, "motor_c": null, "mppt_c": null, "cabin_c": null,
    "battery_cells": [20.0, 20.0, 21.0, 21.0]
  },

  "solar": {
    "pre_mppt":  { "voltage_v": null, "current_a": null },
    "post_mppt": { "voltage_v": null, "current_a": null }
  },

  "dc_dc": {
    "output_64v": { "voltage_v": null },
    "output_12v": { "voltage_v": null }
  },

  "gnss": { "lat": null, "lon": null, "alt_m": null, "speed_kmh": null, "fix": false },

  "vehicle": { "handbrake": null, "oil_level": null, "ignition": null }
}
```

---

## Top-level fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `schema_version` | `string` | yes | Always `"2.0.0"` |
| `vehicle_id` | `integer` | yes | Car ID from the database (1, 2, 3…) |
| `ts` | `string\|null` | yes | ISO-8601 UTC timestamp; `null` if device has no RTC fix |
| `seq` | `integer\|null` | no | Frame counter — resets on device reboot |

---

## `battery` object

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `pack_v` | `number\|null` | V | Total pack voltage |
| `pack_current_a` | `number\|null` | A | Positive = discharging, negative = charging |
| `soc` | `number\|null` | % | State of charge computed from cell voltage (0–100) |
| `soc_bms` | `number\|null` | % | SOC as reported by the BMS directly |
| `max_disch_a` | `number\|null` | A | Maximum discharge current allowed |
| `cell_count` | `integer\|null` | — | Number of cells (19 for this pack) |
| `cell_avg_mv` | `number\|null` | mV | Average cell voltage |
| `cell_min_mv` | `number\|null` | mV | Lowest cell voltage |
| `cell_max_mv` | `number\|null` | mV | Highest cell voltage |
| `cell_spread_mv` | `number\|null` | mV | Difference between highest and lowest cell |

### `battery.cells` — individual cell voltages

Plain array of **19 integers** (mV). Index 0 = cell 1, index 18 = cell 19.  
A `null` entry means that cell's data has not been received yet.

```js
// Reading cell voltages on the frontend:
const cells = data.battery.cells; // length 19
cells.forEach((mv, i) => {
  const cellNumber = i + 1;  // 1-based label
  const status = mv === null ? 'unknown'
               : mv < 2800   ? 'low'
               : mv > 3500   ? 'high'
               : 'good';
});
```

> **Note:** `status` is not in the payload — compute it from `mv` using the thresholds above.

### `battery.charger` object

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `max_charge_v` | `number\|null` | V | Requested charge voltage |
| `max_charge_a` | `number\|null` | A | Requested charge current |
| `enable` | `boolean\|null` | — | Charger enable flag |

### `battery.status` object

| Field | Type | Description |
|-------|------|-------------|
| `fault_level` | `0\|1\|2\|null` | 0 = normal, 1 = warning, 2 = fault |
| `error_code` | `integer\|null` | BMS error code (0 = no error) |
| `ready` | `boolean\|null` | Pack ready to deliver power |
| `charging` | `boolean\|null` | Currently charging |
| `discharging` | `boolean\|null` | Currently discharging |
| `charge_cable` | `boolean\|null` | Charge cable connected |
| `charge_contactor` | `boolean\|null` | Charge relay closed |
| `disch_contactor` | `boolean\|null` | Discharge relay closed |

---

## `temperatures` object

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `battery_avg_c` | `number\|null` | °C | Average of all battery probes |
| `battery_min_c` | `number\|null` | °C | Coldest battery probe |
| `battery_max_c` | `number\|null` | °C | Hottest battery probe |
| `dcdc_c` | `number\|null` | °C | DC/DC converter temperature |
| `motor_c` | `number\|null` | °C | Motor temperature |
| `mppt_c` | `number\|null` | °C | MPPT controller temperature |
| `cabin_c` | `number\|null` | °C | Cabin ambient temperature |

### `temperatures.battery_cells` — per-probe temperatures

Plain array of **4 numbers** (°C). Index 0 = probe 1. `null` = not received.

```js
const probes = data.temperatures.battery_cells; // length 4
probes.forEach((c, i) => console.log(`Probe ${i + 1}: ${c}°C`));
```

---

## `solar` object

| Field | Unit | Description |
|-------|------|-------------|
| `pre_mppt.voltage_v` | V | Panel voltage before MPPT controller |
| `pre_mppt.current_a` | A | Panel current before MPPT controller |
| `post_mppt.voltage_v` | V | MPPT output voltage |
| `post_mppt.current_a` | A | MPPT output current |

---

## `dc_dc` object

| Field | Unit | Description |
|-------|------|-------------|
| `output_64v.voltage_v` | V | High-side 64 V bus |
| `output_12v.voltage_v` | V | Low-side 12 V accessory bus |

---

## `gnss` object

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `lat` | `number\|null` | ° | Latitude, WGS-84 (positive = North) |
| `lon` | `number\|null` | ° | Longitude, WGS-84 (positive = East) |
| `alt_m` | `number\|null` | m | Altitude above ellipsoid |
| `speed_kmh` | `number\|null` | km/h | Ground speed |
| `fix` | `boolean\|null` | — | `true` = valid GPS fix |

---

## `vehicle` object

| Field | Type | Description |
|-------|------|-------------|
| `handbrake` | `boolean\|null` | Handbrake engaged |
| `oil_level` | `"ok"\|"low"\|"critical"\|null` | Engine oil level |
| `ignition` | `boolean\|null` | Ignition key ON |

---

## Important notes for frontend developers

- **`null` means no data yet** — the sensor exists but hasn't sent a value. Always handle `null` gracefully.
- **`cells` is 0-based** — `cells[0]` is cell 1, `cells[18]` is cell 19.
- **`battery_cells` is 0-based** — `battery_cells[0]` is probe 1.
- **Cell status is not in the payload** — compute it from `mv` thresholds: good 2800–3500 mV, low <2800, high >3500.
- **Sensors not yet wired** (solar, DC/DC, GNSS, vehicle) will arrive with all `null` values — render them as `—` or hide the panel.
- **`vehicle_id`** is an integer, not a string like `"esp32-bms-001"`.
