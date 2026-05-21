/*
 * BAKO SMU — ESP32 Full CAN Bus Simulator  (WiFi STA → VPS client)
 * =================================================================
 * The ESP32 connects to your existing WiFi network (internet access)
 * and pushes a JSON telemetry snapshot to the VPS every 2 minutes
 * over a persistent TCP connection.
 *
 * Transport topology (opposite of the AP version):
 *   ESP32 = TCP CLIENT  →  VPS 62.100.50.100:8787 = TCP SERVER
 *
 * Outputs:
 *   • USB Serial  (115 200 baud) — all CAN frame logs + status
 *   • TCP socket to VPS          — JSON push every JSON_PUSH_INTERVAL_MS
 *
 * BMS frames (SA = 0xF4):
 *   0x18FF28F4  BMS Basic Msg 1 — status flags + SOC + pack current + voltage + fault
 *   0x18FE28F4  BMS Basic Msg 2 — max/min cell V + max/min temp + max disch current
 *   0x18C828F4  Cell voltages cells  1-4   (big-endian uint16 mV)
 *   0x18C928F4  Cell voltages cells  5-8
 *   0x18CA28F4  Cell voltages cells  9-12
 *   0x18CB28F4  Cell voltages cells 13-16
 *   0x18CC28F4  Cell voltages cells 17-19 (last pair = 0x0000 padding)
 *   0x18B428F4  Temperature probes 1-4  (uint8, offset -40, 0xFF = NC)
 *   0x18FFE5F4  BMS charging request
 *
 * ESP32 sensor frames (SA = 0xAA):
 *   0x18D001AA  Solar panel current before MPPT
 *   0x18D101AA  Solar panel voltage before MPPT
 *   0x18D201AA  MPPT output current + mode + efficiency
 *   0x18D301AA  DC/DC 12V output voltage
 *   0x18D401AA  Motor temperature
 *   0x18D501AA  MPPT heatsink temperature
 *   0x18D601AA  DC/DC temperature
 *   0x18D701AA  Handbrake position
 *   0x18D801AA  GNSS position + speed + fix
 *
 * CAN: 250 kbps  |  TX: GPIO5  |  RX: GPIO4
 * 7 scenarios × 20 s each, auto-cycling.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi STA (your existing network with internet access)
// ─────────────────────────────────────────────────────────────────────────────
#define WIFI_SSID   "YOUR_WIFI_SSID"      // ← change to your network name
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"  // ← change to your network password

// ─────────────────────────────────────────────────────────────────────────────
//  VPS target — the ESP32 connects TO this server
// ─────────────────────────────────────────────────────────────────────────────
#define VPS_HOST    "62.100.50.100"
#define VPS_PORT    8787

// JSON push every 2 minutes
#define JSON_PUSH_INTERVAL_MS  120000UL

// VPS reconnect back-off (ms)
#define VPS_RECONNECT_MS  5000UL

// ── CAN pins ──────────────────────────────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

// ── BMS frame IDs (SA = 0xF4) ─────────────────────────────────────────────────
#define ID_BMS_MSG1      0x18FF28F4UL
#define ID_BMS_MSG2      0x18FE28F4UL
#define ID_CELLS_1_4     0x18C828F4UL
#define ID_CELLS_5_8     0x18C928F4UL
#define ID_CELLS_9_12    0x18CA28F4UL
#define ID_CELLS_13_16   0x18CB28F4UL
#define ID_CELLS_17_19   0x18CC28F4UL
#define ID_BMS_TEMPS     0x18B428F4UL
#define ID_BMS_CHG_REQ   0x18FFE5F4UL

// ── ESP32 sensor frame IDs (SA = 0xAA) ────────────────────────────────────────
#define ID_SOLAR_CURRENT 0x18D001AAUL
#define ID_SOLAR_VOLTAGE 0x18D101AAUL
#define ID_MPPT_OUTPUT   0x18D201AAUL
#define ID_DCDC_OUTPUT   0x18D301AAUL
#define ID_MOTOR_TEMP    0x18D401AAUL
#define ID_MPPT_TEMP     0x18D501AAUL
#define ID_DCDC_TEMP     0x18D601AAUL
#define ID_HANDBRAKE     0x18D701AAUL
#define ID_GNSS          0x18D801AAUL

// ── TX cycle periods (ms) ─────────────────────────────────────────────────────
#define CY_BMS_MSG1     100
#define CY_BMS_MSG2     100
#define CY_CELLS        500
#define CY_BMS_TEMPS    500
#define CY_CHG_REQ      1000
#define CY_SOLAR_I      200
#define CY_SOLAR_V      200
#define CY_MPPT_OUT     200
#define CY_DCDC_OUT     500
#define CY_MOTOR_TEMP   1000
#define CY_MPPT_TEMP    1000
#define CY_DCDC_TEMP    1000
#define CY_HANDBRAKE    100
#define CY_GNSS         1000

// ── MPPT modes ────────────────────────────────────────────────────────────────
#define MPPT_OFF       0x00
#define MPPT_TRACKING  0x01
#define MPPT_CV        0x02
#define MPPT_FLOAT     0x03

// ── Sensor status codes ───────────────────────────────────────────────────────
#define STS_OK    0x00
#define STS_WARN  0x01
#define STS_FAULT 0x02

// ── BAKO 19S1P LFP pack constants ─────────────────────────────────────────────
#define N_CELLS          19
#define CELL_FULL_MV     3387
#define CELL_EMPTY_MV    2500
#define CELL_OV_MV       3650
#define CELL_UV_MV       2500

#define SIM_DURATION_MS  20000UL

// ─────────────────────────────────────────────────────────────────────────────
//  VPS TCP client  (persistent, reconnects automatically)
// ─────────────────────────────────────────────────────────────────────────────
static WiFiClient vpsClient;
static uint32_t   vps_last_connect_attempt = 0;
static bool       vps_connected            = false;
static uint32_t   json_seq                 = 0;

// Try to open / keep open the TCP connection to the VPS.
// Call every loop iteration — it is non-blocking.
void vps_maintain_connection() {
  if (WiFi.status() != WL_CONNECTED) {
    vps_connected = false;
    return;
  }

  if (vps_connected && vpsClient.connected()) {
    return;   // already up
  }

  // Mark as disconnected if the socket dropped
  if (vps_connected && !vpsClient.connected()) {
    vpsClient.stop();
    vps_connected = false;
    Serial.println("[VPS] Connection lost — will retry");
  }

  uint32_t now = millis();
  if (now - vps_last_connect_attempt < VPS_RECONNECT_MS) {
    return;   // back-off
  }
  vps_last_connect_attempt = now;

  Serial.printf("[VPS] Connecting to %s:%d ...\n", VPS_HOST, VPS_PORT);
  if (vpsClient.connect(VPS_HOST, VPS_PORT)) {
    vps_connected = true;
    vpsClient.setNoDelay(true);
    Serial.println("[VPS] Connected OK");
  } else {
    Serial.println("[VPS] Connection failed — will retry in 5 s");
  }
}

// Write a C-string to Serial only (CAN frame logging stays local)
void serialWrite(const char *s) {
  Serial.print(s);
}

// Write a C-string to the VPS socket only
void vpsWrite(const char *s) {
  if (vps_connected && vpsClient.connected()) {
    vpsClient.print(s);
  }
}

// Printf to Serial only
void serialPrintf(const char *fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  serialWrite(buf);
}

// Printf to VPS socket only (used for JSON push)
void vpsPrintf(const char *fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  vpsWrite(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Full vehicle state
// ─────────────────────────────────────────────────────────────────────────────
struct VehicleState {
  // BMS
  uint8_t  soc;
  float    pack_current_a;
  float    pack_voltage_v;
  uint8_t  status_byte;
  uint8_t  fault_level;
  uint8_t  error_code;

  uint16_t cell_mv[N_CELLS];
  uint16_t cell_max_mv;
  uint16_t cell_min_mv;

  float    bms_t[4];
  float    max_dch_i;
  float    chg_v_max;
  float    chg_i_max;

  // Solar / MPPT
  float    solar_i_raw;
  float    solar_i_avg;
  uint8_t  solar_i_status;

  float    solar_v;
  float    solar_voc;
  uint8_t  solar_v_status;

  float    mppt_out_i;
  uint8_t  mppt_mode;
  uint8_t  mppt_efficiency;
  uint8_t  mppt_status;

  // DC/DC 12V
  float    dcdc_v;
  uint8_t  dcdc_status;

  // Motor temperature (0x18D401AA)
  float    motor_t_winding;
  float    motor_t_housing;
  uint8_t  motor_t_status;

  // MPPT heatsink temperature (0x18D501AA)
  float    mppt_t;
  uint8_t  mppt_t_status;

  // DC/DC temperature (0x18D601AA)
  float    dcdc_t;
  uint8_t  dcdc_t_status;

  // Handbrake (0x18D701AA)
  uint8_t  handbrake;
  uint8_t  hb_debounce;

  // GNSS (0x18D801AA)
  float    gnss_lat;
  float    gnss_lon;
  float    gnss_speed_kmh;
  uint8_t  gnss_fix;
};

VehicleState vs;

// Moving-average ring buffer for solar current
static float   solar_i_buf[5] = {0};
static uint8_t solar_i_idx    = 0;

float solar_avg(float v) {
  solar_i_buf[solar_i_idx++ % 5] = v;
  float s = 0; for (int i = 0; i < 5; i++) s += solar_i_buf[i];
  return s / 5.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encode helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline void pu16le(uint8_t *b, int o, uint16_t v) {
  b[o] = v & 0xFF; b[o+1] = (v >> 8) & 0xFF;
}
static inline void pu16be(uint8_t *b, int o, uint16_t v) {
  b[o] = (v >> 8) & 0xFF; b[o+1] = v & 0xFF;
}
static inline uint16_t enc_01(float val) {
  return (uint16_t)constrain((int32_t)roundf(val * 10.0f), 0, 65535);
}
static inline uint16_t enc_pack_i(float amps) {
  int32_t v = (int32_t)roundf(amps * 10.0f) + 5000;
  return (uint16_t)constrain(v, 0, 65535);
}
static inline uint8_t enc_t(float c) {
  if (c < -40.0f || c > 215.0f) return 0xFF;
  return (uint8_t)roundf(c + 40.0f);
}
static inline void pu32le(uint8_t *b, int o, int32_t v) {
  b[o]   =  v        & 0xFF;
  b[o+1] = (v >>  8) & 0xFF;
  b[o+2] = (v >> 16) & 0xFF;
  b[o+3] = (v >> 24) & 0xFF;
}

// ─────────────────────────────────────────────────────────────────────────────
//  CAN send
// ─────────────────────────────────────────────────────────────────────────────
bool can_send(uint32_t id, const uint8_t *data, uint8_t dlc) {
  twai_message_t m = {};
  m.identifier       = id;
  m.extd             = 1;
  m.data_length_code = dlc;
  memcpy(m.data, data, dlc);
  return twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK;
}

// Log CAN frame to Serial only (not sent to VPS — too high bandwidth)
void log_frame(uint32_t id, const uint8_t *d, uint8_t dlc) {
  char buf[128];
  int  pos = snprintf(buf, sizeof(buf),
                      "[%lums] ID: 0x%08lX DLC: %d Data:",
                      (unsigned long)millis(), (unsigned long)id, dlc);
  for (int i = 0; i < dlc && pos < (int)sizeof(buf) - 4; i++)
    pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", d[i]);
  if (pos < (int)sizeof(buf) - 2) { buf[pos++] = '\n'; buf[pos] = '\0'; }
  serialWrite(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Cell voltage helpers
// ─────────────────────────────────────────────────────────────────────────────
void set_cells(float avg_mv, float spread_mv) {
  for (int i = 0; i < N_CELLS; i++) {
    float offset = ((float)i / (N_CELLS - 1) - 0.5f) * spread_mv;
    float mv = avg_mv + offset;
    mv = constrain(mv, (float)CELL_UV_MV, (float)CELL_OV_MV);
    vs.cell_mv[i] = (uint16_t)roundf(mv);
  }
  vs.cell_max_mv = vs.cell_mv[0];
  vs.cell_min_mv = vs.cell_mv[0];
  float sum = 0;
  for (int i = 0; i < N_CELLS; i++) {
    if (vs.cell_mv[i] > vs.cell_max_mv) vs.cell_max_mv = vs.cell_mv[i];
    if (vs.cell_mv[i] < vs.cell_min_mv) vs.cell_min_mv = vs.cell_mv[i];
    sum += vs.cell_mv[i];
  }
  vs.pack_voltage_v = sum / 1000.0f;
}

float soc_to_cell_mv(uint8_t soc) {
  return CELL_EMPTY_MV + (float)soc / 100.0f * (CELL_FULL_MV - CELL_EMPTY_MV);
}

float osc(float centre, float amp, uint32_t period_ms) {
  return centre + amp * sinf(2.0f * M_PI * (float)millis() / (float)period_ms);
}

// ─────────────────────────────────────────────────────────────────────────────
//  BMS frame transmitters — CAN bus + Serial log
// ─────────────────────────────────────────────────────────────────────────────

// 0x18FF28F4 — BMS Basic Msg 1
void tx_bms_msg1() {
  uint8_t d[8] = {0};
  d[0] = vs.status_byte;
  d[1] = vs.soc;
  pu16le(d, 2, enc_pack_i(vs.pack_current_a));
  pu16le(d, 4, enc_01(vs.pack_voltage_v));
  d[6] = vs.fault_level;
  d[7] = vs.error_code;
  can_send(ID_BMS_MSG1, d, 8);
  log_frame(ID_BMS_MSG1, d, 8);
}

// 0x18FE28F4 — BMS Basic Msg 2
void tx_bms_msg2() {
  uint8_t d[8] = {0};
  pu16le(d, 0, vs.cell_max_mv);
  pu16le(d, 2, vs.cell_min_mv);
  float tmax = vs.bms_t[0], tmin = vs.bms_t[0];
  for (int i = 1; i < 4; i++) {
    if (vs.bms_t[i] > tmax) tmax = vs.bms_t[i];
    if (vs.bms_t[i] < tmin) tmin = vs.bms_t[i];
  }
  d[4] = enc_t(tmax);
  d[5] = enc_t(tmin);
  pu16le(d, 6, enc_01(vs.max_dch_i));
  can_send(ID_BMS_MSG2, d, 8);
  log_frame(ID_BMS_MSG2, d, 8);
}

// 0x18C8-CC28F4 — Cell voltages (big-endian uint16, 4 cells per frame)
void tx_cell_voltages() {
  uint32_t ids[5] = {
    ID_CELLS_1_4, ID_CELLS_5_8, ID_CELLS_9_12, ID_CELLS_13_16, ID_CELLS_17_19
  };
  for (int g = 0; g < 5; g++) {
    uint8_t d[8] = {0};
    for (int i = 0; i < 4; i++) {
      int cell_idx = g * 4 + i;
      uint16_t mv = (cell_idx < N_CELLS) ? vs.cell_mv[cell_idx] : 0;
      pu16be(d, i * 2, mv);
    }
    can_send(ids[g], d, 8);
    log_frame(ids[g], d, 8);
  }
}

// 0x18B428F4 — BMS temperature probes
void tx_bms_temps() {
  uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  for (int i = 0; i < 4; i++) d[i] = enc_t(vs.bms_t[i]);
  can_send(ID_BMS_TEMPS, d, 8);
  log_frame(ID_BMS_TEMPS, d, 8);
}

// 0x18FFE5F4 — BMS charging request
void tx_chg_request() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_01(vs.chg_v_max));
  pu16le(d, 2, enc_01(vs.chg_i_max));
  d[4] = 0x00;
  can_send(ID_BMS_CHG_REQ, d, 8);
  log_frame(ID_BMS_CHG_REQ, d, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ESP32 sensor frame transmitters
// ─────────────────────────────────────────────────────────────────────────────

// 0x18D001AA — Solar panel current (before MPPT)
void tx_solar_current() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_01(vs.solar_i_raw));
  pu16le(d, 2, enc_01(vs.solar_i_avg));
  d[4] = vs.solar_i_status;
  can_send(ID_SOLAR_CURRENT, d, 8);
  log_frame(ID_SOLAR_CURRENT, d, 8);
}

// 0x18D101AA — Solar panel voltage (before MPPT)
void tx_solar_voltage() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_01(vs.solar_v));
  pu16le(d, 2, enc_01(vs.solar_voc));
  d[4] = vs.solar_v_status;
  can_send(ID_SOLAR_VOLTAGE, d, 8);
  log_frame(ID_SOLAR_VOLTAGE, d, 8);
}

// 0x18D201AA — MPPT output current + mode + efficiency
void tx_mppt_output() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_01(vs.mppt_out_i));
  d[2] = vs.mppt_mode;
  d[3] = vs.mppt_efficiency;
  d[4] = vs.mppt_status;
  can_send(ID_MPPT_OUTPUT, d, 8);
  log_frame(ID_MPPT_OUTPUT, d, 8);
}

// 0x18D301AA — DC/DC 12V output voltage
void tx_dcdc_output() {
  uint8_t d[8] = {0};
  uint16_t v_raw = (uint16_t)constrain((int32_t)roundf(vs.dcdc_v * 100.0f), 0, 65535);
  pu16le(d, 0, v_raw);
  d[2] = 0xFF; d[3] = 0xFF;
  d[4] = vs.dcdc_status;
  can_send(ID_DCDC_OUTPUT, d, 8);
  log_frame(ID_DCDC_OUTPUT, d, 8);
}

// 0x18D401AA — Motor temperature
void tx_motor_temp() {
  uint8_t d[8] = {0};
  d[0] = enc_t(vs.motor_t_winding);
  d[1] = enc_t(vs.motor_t_housing);
  d[2] = vs.motor_t_status;
  can_send(ID_MOTOR_TEMP, d, 8);
  log_frame(ID_MOTOR_TEMP, d, 8);
}

// 0x18D501AA — MPPT heatsink temperature
void tx_mppt_temp() {
  uint8_t d[8] = {0};
  d[0] = enc_t(vs.mppt_t);
  d[1] = vs.mppt_t_status;
  can_send(ID_MPPT_TEMP, d, 8);
  log_frame(ID_MPPT_TEMP, d, 8);
}

// 0x18D601AA — DC/DC temperature
void tx_dcdc_temp() {
  uint8_t d[8] = {0};
  d[0] = enc_t(vs.dcdc_t);
  d[1] = vs.dcdc_t_status;
  can_send(ID_DCDC_TEMP, d, 8);
  log_frame(ID_DCDC_TEMP, d, 8);
}

// 0x18D701AA — Handbrake position
void tx_handbrake() {
  uint8_t d[8] = {0};
  d[0] = vs.handbrake;
  d[1] = vs.hb_debounce;
  can_send(ID_HANDBRAKE, d, 8);
  log_frame(ID_HANDBRAKE, d, 8);
}

// 0x18D801AA — GNSS position + speed + fix
void tx_gnss() {
  uint8_t d[8] = {0};
  int32_t lat_raw = (int32_t)(vs.gnss_lat * 1e7f);
  pu32le(d, 0, lat_raw);
  uint16_t spd_raw = (uint16_t)constrain((int32_t)roundf(vs.gnss_speed_kmh * 10.0f), 0, 65535);
  pu16le(d, 4, spd_raw);
  d[6] = vs.gnss_fix;
  d[7] = 0x00;
  can_send(ID_GNSS, d, 8);
  log_frame(ID_GNSS, d, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
//  JSON push to VPS  — schema v2.0.0 matching Data_format.Json / SQL schema
//  Sent as a single line: JSON\n  (newline-delimited, easy to parse server-side)
// ─────────────────────────────────────────────────────────────────────────────
void push_json_to_vps() {
  if (!vps_connected || !vpsClient.connected()) {
    serialPrintf("[VPS] Push skipped — not connected (seq=%lu)\n",
                 (unsigned long)(json_seq + 1));
    return;
  }

  json_seq++;

  // Derived values
  float t_sum = 0, t_min = vs.bms_t[0], t_max = vs.bms_t[0];
  for (int i = 0; i < 4; i++) {
    t_sum += vs.bms_t[i];
    if (vs.bms_t[i] < t_min) t_min = vs.bms_t[i];
    if (vs.bms_t[i] > t_max) t_max = vs.bms_t[i];
  }
  float t_avg = t_sum / 4.0f;

  float cell_sum = 0;
  for (int i = 0; i < N_CELLS; i++) cell_sum += vs.cell_mv[i];
  float cell_avg = cell_sum / N_CELLS;

  bool flag_charging    = (vs.status_byte & 0x02) != 0;
  bool flag_discharging = (vs.status_byte & 0x04) != 0;
  bool flag_ready       = (vs.status_byte & 0x08) != 0;

  // ── Send JSON in chunks (avoids a huge single-call stack buffer) ──────────

  // Root open + meta
  vpsPrintf("{");
  vpsPrintf("\"schema_version\":\"2.0.0\",");
  vpsPrintf("\"vehicle_id\":1,");
  vpsPrintf("\"seq\":%lu,", (unsigned long)json_seq);
  // Uptime as ISO-like timestamp (no RTC — use millis since boot)
  vpsPrintf("\"uptime_ms\":%lu,", (unsigned long)millis());

  // battery
  vpsPrintf("\"battery\":{");
  vpsPrintf("\"pack_v\":%.3f,", vs.pack_voltage_v);
  vpsPrintf("\"pack_current_a\":%.3f,", vs.pack_current_a);
  vpsPrintf("\"soc\":%.2f,", (float)vs.soc);
  vpsPrintf("\"soc_bms\":%.2f,", (float)vs.soc);
  vpsPrintf("\"max_disch_a\":%.1f,", vs.max_dch_i);
  vpsPrintf("\"cell_count\":%d,", N_CELLS);
  vpsPrintf("\"cell_avg_mv\":%.2f,", cell_avg);
  vpsPrintf("\"cell_min_mv\":%u,", vs.cell_min_mv);
  vpsPrintf("\"cell_max_mv\":%u,", vs.cell_max_mv);
  vpsPrintf("\"cell_spread_mv\":%u,", vs.cell_max_mv - vs.cell_min_mv);

  vpsPrintf("\"cells_voltages\":[");
  for (int i = 0; i < N_CELLS; i++) {
    vpsPrintf("%u", vs.cell_mv[i]);
    if (i < N_CELLS - 1) vpsPrintf(",");
  }
  vpsPrintf("],");

  vpsPrintf("\"charge_limit\":{");
  vpsPrintf("\"max_charge_v\":%.1f,", vs.chg_v_max);
  vpsPrintf("\"max_charge_a\":%.1f,", vs.chg_i_max);
  vpsPrintf("\"enable\":%s", (vs.chg_i_max > 0.0f) ? "true" : "false");
  vpsPrintf("},");

  vpsPrintf("\"status\":{");
  vpsPrintf("\"fault_level\":%u,", vs.fault_level);
  vpsPrintf("\"error_code\":%u,", vs.error_code);
  vpsPrintf("\"ready\":%s,", flag_ready ? "true" : "false");
  vpsPrintf("\"charging\":%s,", flag_charging ? "true" : "false");
  vpsPrintf("\"discharging\":%s", flag_discharging ? "true" : "false");
  vpsPrintf("}");
  vpsPrintf("},");   // end battery

  // temperatures
  vpsPrintf("\"temperatures\":{");
  vpsPrintf("\"battery_avg_c\":%.2f,", t_avg);
  vpsPrintf("\"battery_min_c\":%.2f,", t_min);
  vpsPrintf("\"battery_max_c\":%.2f,", t_max);
  vpsPrintf("\"battery_temps_c\":[%.1f,%.1f,%.1f,%.1f],",
            vs.bms_t[0], vs.bms_t[1], vs.bms_t[2], vs.bms_t[3]);
  vpsPrintf("\"dcdc_temp_c\":%.2f,", vs.dcdc_t);
  vpsPrintf("\"motor_temp_c\":%.2f,", vs.motor_t_winding);
  vpsPrintf("\"mppt_temp_c\":%.2f", vs.mppt_t);
  vpsPrintf("},");   // end temperatures

  // solar
  vpsPrintf("\"solar\":{");
  vpsPrintf("\"pre_mppt\":{\"voltage_v\":%.3f,\"current_a\":%.3f},",
            vs.solar_v, vs.solar_i_raw);
  vpsPrintf("\"post_mppt\":{\"current_a\":%.3f}", vs.mppt_out_i);
  vpsPrintf("},");   // end solar

  // dc_dc
  vpsPrintf("\"dc_dc\":{");
  vpsPrintf("\"input_64v\":{\"voltage_v\":%.3f},", vs.pack_voltage_v);
  vpsPrintf("\"output_12v\":{\"voltage_v\":%.3f}", vs.dcdc_v);
  vpsPrintf("},");   // end dc_dc

  // gnss
  vpsPrintf("\"gnss\":{");
  if (vs.gnss_fix) {
    vpsPrintf("\"lat\":%.7f,\"lon\":%.7f,\"alt_m\":null,\"speed_kmh\":%.2f,\"fix\":true",
              vs.gnss_lat, vs.gnss_lon, vs.gnss_speed_kmh);
  } else {
    vpsPrintf("\"lat\":null,\"lon\":null,\"alt_m\":null,\"speed_kmh\":null,\"fix\":false");
  }
  vpsPrintf("},");   // end gnss

  // vehicle
  vpsPrintf("\"vehicle\":{\"handbrake\":%s}", vs.handbrake ? "true" : "false");

  vpsPrintf("}\n");   // close root — newline is the message delimiter

  serialPrintf("[VPS] JSON push #%lu sent to %s:%d\n",
               (unsigned long)json_seq, VPS_HOST, VPS_PORT);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SCENARIO DEFINITIONS  (identical physics to the AP version)
// ─────────────────────────────────────────────────────────────────────────────

void scenario_idle_parked() {
  vs.soc            = 75;
  vs.pack_current_a = 0.0f;
  set_cells(soc_to_cell_mv(vs.soc), 3.0f);
  vs.status_byte    = 0b00011000;
  vs.fault_level    = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(22.0f, 0.3f, 8000);
  vs.bms_t[1] = osc(21.5f, 0.3f, 9000);
  vs.bms_t[2] = osc(22.2f, 0.2f, 7000);
  vs.bms_t[3] = osc(21.8f, 0.2f, 10000);
  vs.max_dch_i = 100.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw = 0.0f; vs.solar_i_avg = solar_avg(0.0f); vs.solar_i_status = STS_OK;
  vs.solar_v = 0.0f; vs.solar_voc = 75.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i = 0.0f; vs.mppt_mode = MPPT_OFF; vs.mppt_efficiency = 0; vs.mppt_status = STS_OK;
  vs.dcdc_v = osc(12.45f, 0.05f, 5000); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(26.0f, 0.5f, 12000);
  vs.motor_t_housing = vs.motor_t_winding - 1.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t = 24.0f; vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(30.0f, 0.5f, 9000); vs.dcdc_t_status = STS_OK;
  vs.handbrake = 1; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8197f; vs.gnss_lon = 10.1658f;
  vs.gnss_speed_kmh = 0.0f; vs.gnss_fix = 1;
}

void scenario_city_drive() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc            = (uint8_t)constrain(68 - (int)(elapsed / 3000), 60, 70);
  vs.pack_current_a = osc(18.0f, 5.0f, 4000);
  set_cells(soc_to_cell_mv(vs.soc), osc(6.0f, 2.0f, 7000));
  vs.status_byte    = 0b00010100;
  vs.fault_level    = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(28.0f, 1.5f, 5000);
  vs.bms_t[1] = osc(27.5f, 1.0f, 6000);
  vs.bms_t[2] = osc(29.0f, 1.5f, 4500);
  vs.bms_t[3] = osc(28.5f, 1.0f, 7000);
  vs.max_dch_i = 100.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw = osc(4.5f, 0.8f, 5000); vs.solar_i_avg = solar_avg(vs.solar_i_raw); vs.solar_i_status = STS_OK;
  vs.solar_v = osc(68.0f, 3.0f, 7000); vs.solar_voc = 75.5f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i = osc(3.8f, 0.4f, 4500); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 93; vs.mppt_status = STS_OK;
  vs.dcdc_v = osc(12.3f, 0.15f, 3000); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(55.0f, 5.0f, 6000);
  vs.motor_t_housing = vs.motor_t_winding - 7.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t = osc(42.0f, 2.0f, 9000); vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(45.0f, 2.0f, 8000); vs.dcdc_t_status = STS_OK;
  vs.handbrake = 0; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8197f + (float)elapsed * 8.3e-8f;
  vs.gnss_lon = 10.1658f + (float)elapsed * 3.0e-8f;
  vs.gnss_speed_kmh = osc(30.0f, 10.0f, 8000); vs.gnss_fix = 1;
}

void scenario_solar_charging() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc            = (uint8_t)constrain(50 + (int)(elapsed / 1000), 50, 70);
  vs.pack_current_a = osc(-18.0f, 2.0f, 5000);
  set_cells(soc_to_cell_mv(vs.soc), 4.0f);
  vs.status_byte    = 0b00100010;
  vs.fault_level    = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(25.0f, 1.0f, 8000);
  vs.bms_t[1] = osc(24.5f, 0.8f, 9000);
  vs.bms_t[2] = osc(25.5f, 1.0f, 7000);
  vs.bms_t[3] = osc(25.0f, 0.8f, 10000);
  vs.max_dch_i = 0.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 35.0f;

  vs.solar_i_raw = osc(22.0f, 1.5f, 6000); vs.solar_i_avg = solar_avg(vs.solar_i_raw); vs.solar_i_status = STS_OK;
  vs.solar_v = osc(82.0f, 2.0f, 8000); vs.solar_voc = 90.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i = osc(19.0f, 1.0f, 5000); vs.mppt_mode = MPPT_CV; vs.mppt_efficiency = 96; vs.mppt_status = STS_OK;
  vs.dcdc_v = 13.8f; vs.dcdc_status = STS_OK;
  vs.motor_t_winding = 28.0f; vs.motor_t_housing = 27.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t = osc(55.0f, 3.0f, 7000); vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(38.0f, 1.5f, 9000); vs.dcdc_t_status = STS_OK;
  vs.handbrake = 1; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8200f; vs.gnss_lon = 10.1660f;
  vs.gnss_speed_kmh = 0.0f; vs.gnss_fix = 1;
}

void scenario_highway() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc            = (uint8_t)constrain(55 - (int)(elapsed / 2500), 40, 56);
  vs.pack_current_a = osc(42.0f, 6.0f, 3000);
  set_cells(soc_to_cell_mv(vs.soc), osc(10.0f, 3.0f, 8000));
  vs.status_byte    = 0b00010100;
  vs.fault_level    = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(34.0f, 2.0f, 4000);
  vs.bms_t[1] = osc(33.0f, 1.5f, 5000);
  vs.bms_t[2] = osc(35.0f, 2.0f, 3500);
  vs.bms_t[3] = osc(34.5f, 1.5f, 6000);
  vs.max_dch_i = 100.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw = osc(7.0f, 1.0f, 4000); vs.solar_i_avg = solar_avg(vs.solar_i_raw); vs.solar_i_status = STS_OK;
  vs.solar_v = osc(71.0f, 2.5f, 6000); vs.solar_voc = 77.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i = osc(5.5f, 0.5f, 3500); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 92; vs.mppt_status = STS_OK;
  vs.dcdc_v = osc(12.1f, 0.12f, 2500); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(72.0f, 4.0f, 5000);
  vs.motor_t_housing = vs.motor_t_winding - 8.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t = osc(48.0f, 2.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(55.0f, 2.5f, 7000); vs.dcdc_t_status = STS_OK;
  vs.handbrake = 0; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8197f + (float)elapsed * 2.5e-7f;
  vs.gnss_lon = 10.1658f + (float)elapsed * 1.0e-7f;
  vs.gnss_speed_kmh = osc(90.0f, 8.0f, 6000); vs.gnss_fix = 1;
}

void scenario_low_battery() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc            = (uint8_t)constrain(12 - (int)(elapsed / 5000), 5, 13);
  vs.pack_current_a = osc(8.0f, 2.0f, 4000);
  set_cells(soc_to_cell_mv(vs.soc), osc(18.0f, 5.0f, 6000));
  vs.status_byte    = 0b00010100;
  vs.fault_level    = 1; vs.error_code = 0x03;
  vs.bms_t[0] = osc(30.0f, 1.0f, 6000);
  vs.bms_t[1] = osc(29.5f, 1.0f, 7000);
  vs.bms_t[2] = osc(30.5f, 1.0f, 5500);
  vs.bms_t[3] = osc(30.0f, 0.8f, 8000);
  vs.max_dch_i = 35.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw = osc(1.2f, 0.3f, 4000); vs.solar_i_avg = solar_avg(vs.solar_i_raw); vs.solar_i_status = STS_WARN;
  vs.solar_v = osc(52.0f, 4.0f, 6000); vs.solar_voc = 72.0f; vs.solar_v_status = STS_WARN;
  vs.mppt_out_i = osc(1.0f, 0.2f, 3000); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 88; vs.mppt_status = STS_WARN;
  vs.dcdc_v = osc(11.6f, 0.25f, 2000); vs.dcdc_status = STS_WARN;
  vs.motor_t_winding = osc(40.0f, 2.0f, 7000);
  vs.motor_t_housing = vs.motor_t_winding - 6.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t = 36.0f; vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(40.0f, 1.5f, 8000); vs.dcdc_t_status = STS_OK;
  vs.handbrake = 0; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8197f + (float)elapsed * 4.2e-8f;
  vs.gnss_lon = 10.1658f + (float)elapsed * 1.5e-8f;
  vs.gnss_speed_kmh = osc(15.0f, 4.0f, 7000); vs.gnss_fix = 1;
}

void scenario_overtemp_motor() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc            = 45;
  vs.pack_current_a = osc(25.0f, 5.0f, 4000);
  set_cells(soc_to_cell_mv(vs.soc), 7.0f);
  vs.status_byte    = 0b00010100;
  vs.fault_level    = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(31.0f, 1.5f, 5000);
  vs.bms_t[1] = osc(30.5f, 1.0f, 6000);
  vs.bms_t[2] = osc(32.0f, 1.5f, 4500);
  vs.bms_t[3] = osc(31.5f, 1.0f, 7000);
  vs.max_dch_i = 100.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw = osc(5.0f, 0.5f, 4500); vs.solar_i_avg = solar_avg(vs.solar_i_raw); vs.solar_i_status = STS_OK;
  vs.solar_v = osc(70.0f, 2.5f, 7000); vs.solar_voc = 78.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i = osc(4.2f, 0.4f, 4000); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 91; vs.mppt_status = STS_OK;
  vs.dcdc_v = osc(12.2f, 0.15f, 3000); vs.dcdc_status = STS_OK;

  float t_rise = 65.0f + (float)elapsed / 1000.0f * 3.5f;
  t_rise = constrain(t_rise, 65.0f, 135.0f);
  vs.motor_t_winding = t_rise;
  vs.motor_t_housing = t_rise - 9.0f;
  if (t_rise > 100.0f)     vs.motor_t_status = STS_FAULT;
  else if (t_rise > 80.0f) vs.motor_t_status = STS_WARN;
  else                      vs.motor_t_status = STS_OK;

  vs.mppt_t = osc(50.0f, 4.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(48.0f + t_rise * 0.1f, 2.0f, 7000); vs.dcdc_t_status = (t_rise > 100.0f) ? STS_WARN : STS_OK;
  vs.handbrake = 0; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8197f + (float)elapsed * 1.1e-7f;
  vs.gnss_lon = 10.1658f + (float)elapsed * 4.0e-8f;
  vs.gnss_speed_kmh = osc(40.0f, 6.0f, 5000); vs.gnss_fix = 1;
}

void scenario_float_charge() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc            = (uint8_t)constrain(95 + (int)(elapsed / 5000), 95, 100);
  vs.pack_current_a = osc(-2.5f, 0.5f, 5000);
  set_cells(soc_to_cell_mv(vs.soc), 2.0f);
  vs.status_byte    = 0b00100010;
  vs.fault_level    = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(23.5f, 0.5f, 9000);
  vs.bms_t[1] = osc(23.0f, 0.5f, 10000);
  vs.bms_t[2] = osc(24.0f, 0.5f, 8000);
  vs.bms_t[3] = osc(23.5f, 0.5f, 11000);
  vs.max_dch_i = 0.0f;
  vs.chg_v_max = 69.3f; vs.chg_i_max = 5.0f;

  vs.solar_i_raw = osc(3.0f, 0.4f, 7000); vs.solar_i_avg = solar_avg(vs.solar_i_raw); vs.solar_i_status = STS_OK;
  vs.solar_v = osc(76.0f, 1.5f, 9000); vs.solar_voc = 82.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i = osc(2.5f, 0.3f, 6000); vs.mppt_mode = MPPT_FLOAT; vs.mppt_efficiency = 95; vs.mppt_status = STS_OK;
  vs.dcdc_v = 13.6f; vs.dcdc_status = STS_OK;
  vs.motor_t_winding = 26.0f; vs.motor_t_housing = 25.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t = osc(38.0f, 2.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.dcdc_t = osc(32.0f, 1.0f, 10000); vs.dcdc_t_status = STS_OK;
  vs.handbrake = 1; vs.hb_debounce = 0;
  vs.gnss_lat = 36.8202f; vs.gnss_lon = 10.1662f;
  vs.gnss_speed_kmh = 0.0f; vs.gnss_fix = 1;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Scenario table
// ─────────────────────────────────────────────────────────────────────────────
struct Scenario {
  const char *id_name;
  const char *display_name;
  void (*fn)();
};

static const Scenario SCENARIOS[] = {
  { "IDLE_PARKED",    "P  IDLE - PARKED",      scenario_idle_parked    },
  { "CITY_DRIVE",     "CITY DRIVE",             scenario_city_drive     },
  { "SOLAR_CHARGING", "SOLAR CHARGING",         scenario_solar_charging },
  { "HIGHWAY",        "HIGHWAY CRUISE",         scenario_highway        },
  { "LOW_BATTERY",    "LOW BATTERY WARNING",    scenario_low_battery    },
  { "OVERTEMP_MOTOR", "MOTOR OVERTEMP",         scenario_overtemp_motor },
  { "FLOAT_CHARGE",   "FLOAT CHARGE (FULL)",    scenario_float_charge   },
};
static const uint8_t N_SIM = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);

uint8_t  cur_scn      = 0;
uint32_t scn_start_ms = 0;

void advance_scenario() {
  cur_scn = (cur_scn + 1) % N_SIM;
  scn_start_ms = millis();
  for (int i = 0; i < 5; i++) solar_i_buf[i] = 0;
  solar_i_idx = 0;
  Serial.printf("\n[SIM] ============================================\n");
  Serial.printf("[SIM]  SCENARIO: %s\n", SCENARIOS[cur_scn].display_name);
  Serial.printf("[SIM]  Duration: %lu s\n", (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ============================================\n\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi connect helper (blocking until connected, with timeout)
// ─────────────────────────────────────────────────────────────────────────────
void wifi_connect() {
  Serial.printf("[WiFi] Connecting to SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 20000) {
      Serial.println("\n[WiFi] Timeout — will retry in loop");
      return;
    }
  }
  Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n[BAKO] VPS Client Simulator starting...");
  Serial.printf("[BAKO] VPS target: %s:%d\n", VPS_HOST, VPS_PORT);
  Serial.printf("[BAKO] JSON push interval: %lu s\n",
                (unsigned long)(JSON_PUSH_INTERVAL_MS / 1000));

  // Connect to WiFi (internet access needed)
  wifi_connect();

  // CAN / TWAI
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_fiGPRSr_config_t  f = TWAI_FIGPRSR_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
  ESP_ERROR_CHECK(twai_start());

  scn_start_ms = millis();

  Serial.printf("[BAKO] Full CAN Bus Simulator - 250 kbps\n");
  Serial.printf("[BAKO] BMS frames (SA=F4): 9  |  Sensor frames (SA=AA): 9\n");
  Serial.printf("[BAKO] %d scenarios x %lu s each\n\n",
                N_SIM, (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ============================================\n");
  Serial.printf("[SIM]  SCENARIO: %s\n", SCENARIOS[cur_scn].display_name);
  Serial.printf("[SIM]  Duration: %lu s\n", (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ============================================\n\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // Reconnect WiFi if dropped
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Disconnected — reconnecting...");
    wifi_connect();
  }

  // Maintain VPS TCP connection (non-blocking)
  vps_maintain_connection();

  // Per-frame timestamps
  static uint32_t t_msg1    = 0, t_msg2    = 0;
  static uint32_t t_cells   = 0, t_btemps  = 0, t_chgreq  = 0;
  static uint32_t t_solar_i = 0, t_solar_v = 0, t_mppt    = 0;
  static uint32_t t_dcdc    = 0, t_mtemp   = 0;
  static uint32_t t_mppt_t  = 0, t_dcdc_t  = 0;
  static uint32_t t_hb      = 0, t_gnss    = 0;
  static uint32_t t_json    = 0;

  uint32_t now = millis();

  // Scenario advance
  if (now - scn_start_ms >= SIM_DURATION_MS) advance_scenario();

  // Update vehicle state
  SCENARIOS[cur_scn].fn();

  // ── BMS frames (SA = 0xF4) ─────────────────────────────────────────────────
  if (now - t_msg1   >= CY_BMS_MSG1)  { t_msg1   = now; tx_bms_msg1();     }
  if (now - t_msg2   >= CY_BMS_MSG2)  { t_msg2   = now; tx_bms_msg2();     }
  if (now - t_cells  >= CY_CELLS)     { t_cells  = now; tx_cell_voltages(); }
  if (now - t_btemps >= CY_BMS_TEMPS) { t_btemps = now; tx_bms_temps();    }
  if (now - t_chgreq >= CY_CHG_REQ)   { t_chgreq = now; tx_chg_request();  }

  // ── ESP32 sensor frames (SA = 0xAA) ───────────────────────────────────────
  if (now - t_solar_i >= CY_SOLAR_I)   { t_solar_i = now; tx_solar_current(); }
  if (now - t_solar_v >= CY_SOLAR_V)   { t_solar_v = now; tx_solar_voltage(); }
  if (now - t_mppt    >= CY_MPPT_OUT)  { t_mppt    = now; tx_mppt_output();   }
  if (now - t_dcdc    >= CY_DCDC_OUT)  { t_dcdc    = now; tx_dcdc_output();   }
  if (now - t_mtemp   >= CY_MOTOR_TEMP){ t_mtemp   = now; tx_motor_temp();    }
  if (now - t_mppt_t  >= CY_MPPT_TEMP) { t_mppt_t  = now; tx_mppt_temp();     }
  if (now - t_dcdc_t  >= CY_DCDC_TEMP) { t_dcdc_t  = now; tx_dcdc_temp();     }
  if (now - t_hb      >= CY_HANDBRAKE) { t_hb      = now; tx_handbrake();     }
  if (now - t_gnss    >= CY_GNSS)      { t_gnss    = now; tx_gnss();           }

  // ── JSON push to VPS every 2 minutes ──────────────────────────────────────
  if (now - t_json >= JSON_PUSH_INTERVAL_MS) {
    t_json = now;
    push_json_to_vps();
  }

  delay(2);
}
