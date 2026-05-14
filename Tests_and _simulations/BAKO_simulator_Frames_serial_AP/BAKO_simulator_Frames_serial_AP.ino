/*
 * BAKO SMU — ESP32 Full CAN Bus Simulator  (Serial + WiFi AP)
 * ============================================================
 * Identical simulation engine to the Serial-only version, but
 * every output line is now sent on BOTH:
 *
 *   • USB Serial  (115 200 baud, same as before)
 *   • WiFi AP     SSID "BAKO_SMU" / password "12345678"
 *                 TCP server on 192.168.4.1:9000
 *                 (server_combined.py connects here as a TCP client)
 *
 * No reflashing needed to switch transport — the PC picks the side.
 *
 * BMS frames  (SA = 0xF4):
 *   0x18FF28F4  BMS Basic Msg 1  — status flags + SOC + pack current + voltage + fault
 *   0x18FE28F4  BMS Basic Msg 2  — max/min cell V + max/min temp + max disch current
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
 *   0x18D401AA  Motor  temperature
 *   0x18D501AA  MPPT heatsink temperature
 *   0x18D601AA  DC/DC temperature 
 *   0x18D701AA  Handbrake position
 *
 * CAN:   250 kbps  |  TX: GPIO5  |  RX: GPIO4
 * 7 scenarios × 20 s each, auto-cycling.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi AP config
// ─────────────────────────────────────────────────────────────────────────────
#define WIFI_SSID     "ESP_32"
#define WIFI_PASS     "12345678"   // WPA2, must be ≥ 8 chars
#define WIFI_CHANNEL  6
#define TCP_PORT      9000

static WiFiServer tcpServer(TCP_PORT);
static WiFiClient tcpClient;          // holds the one connected PC client

// ── CAN pins ──────────────────────────────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

// ── BMS frame IDs (SA = 0xF4) ─────────────────────────────────────────────────
#define ID_BMS_MSG1      0x18FF28F4
#define ID_BMS_MSG2      0x18FE28F4
#define ID_CELLS_1_4     0x18C828F4
#define ID_CELLS_5_8     0x18C928F4
#define ID_CELLS_9_12    0x18CA28F4
#define ID_CELLS_13_16   0x18CB28F4
#define ID_CELLS_17_19   0x18CC28F4
#define ID_BMS_TEMPS     0x18B428F4
#define ID_BMS_CHG_REQ   0x18FFE5F4

// ── ESP32 sensor frame IDs (SA = 0xAA) ────────────────────────────────────────
#define ID_SOLAR_CURRENT 0x18D001AA
#define ID_SOLAR_VOLTAGE 0x18D101AA
#define ID_MPPT_OUTPUT   0x18D201AA
#define ID_DCDC_OUTPUT   0x18D301AA
#define ID_MOTOR_TEMP    0x18D401AA
#define ID_MPPT_TEMP     0x18D501AA
#define ID_CABIN_TEMP    0x18D601AA
#define ID_HANDBRAKE     0x18D701AA

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
#define CY_CABIN_TEMP   1000
#define CY_HANDBRAKE    100

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
#define PACK_CHG_CUTOFF  693
#define MAX_CHG_I_RAW    350
#define MAX_DCH_I_RAW    1000

#define SIM_DURATION_MS  20000UL   // 20 s per scenario

// ─────────────────────────────────────────────────────────────────────────────
//  Full vehicle state
// ─────────────────────────────────────────────────────────────────────────────
struct VehicleState {
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

  float    dcdc_v;
  uint8_t  dcdc_status;

  float    motor_t_winding;
  float    motor_t_housing;
  uint8_t  motor_t_status;

  float    mppt_t;
  uint8_t  mppt_t_status;

  float    cabin_t;
  uint8_t  cabin_humidity;
  uint8_t  cabin_status;

  uint8_t  handbrake;
  uint8_t  hb_debounce;
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
//  Dual-output helpers
//  dualPrint / dualPrintLn / dualPrintf write to Serial AND the WiFi client.
// ─────────────────────────────────────────────────────────────────────────────

// Accept a new TCP client if one is waiting (call every loop iteration)
void wifi_accept_pending() {
  WiFiClient c = tcpServer.accept();   // non-blocking
  if (c) {
    if (tcpClient) tcpClient.stop();   // drop old client if any
    tcpClient = c;
    Serial.printf("[WiFi] Client connected: %s\n",
                  tcpClient.remoteIP().toString().c_str());
  }
}

// Write a raw C-string to both outputs
void dualWrite(const char *s) {
  Serial.print(s);
  if (tcpClient && tcpClient.connected()) {
    tcpClient.print(s);
  }
}

// Printf-style helper (max 256 chars per call — enough for any log line)
void dualPrintf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  dualWrite(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encode helpers (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
static inline void pu16le(uint8_t *b, int o, uint16_t v) {
  b[o] = v & 0xFF; b[o+1] = (v >> 8) & 0xFF;
}
static inline void pu16be(uint8_t *b, int o, uint16_t v) {
  b[o] = (v >> 8) & 0xFF; b[o+1] = v & 0xFF;
}
static inline uint16_t enc_i01(float a) {
  int32_t v = (int32_t)roundf(a * 10.0f);
  return (uint16_t)constrain(v, 0, 65535);
}
static inline uint16_t enc_v01(float v) {
  return (uint16_t)constrain((int32_t)roundf(v * 10.0f), 0, 65535);
}
static inline uint16_t enc_v001(float v) {
  return (uint16_t)constrain((int32_t)roundf(v * 100.0f), 0, 65535);
}
static inline uint8_t enc_t(float c) {
  if (c < -40.0f || c > 215.0f) return 0xFF;
  return (uint8_t)roundf(c + 40.0f);
}
static inline uint16_t enc_pack_i(float amps) {
  int32_t v = (int32_t)roundf(amps * 10.0f) + 5000;
  return (uint16_t)constrain(v, 0, 65535);
}

// ─────────────────────────────────────────────────────────────────────────────
//  CAN send (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
bool can_send(uint32_t id, const uint8_t *data, uint8_t dlc) {
  twai_message_t m = {};
  m.identifier       = id;
  m.extd             = 1;
  m.data_length_code = dlc;
  memcpy(m.data, data, dlc);
  return twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Log a CAN frame — now goes to BOTH Serial and WiFi client
// ─────────────────────────────────────────────────────────────────────────────
void log_frame(uint32_t id, const uint8_t *d, uint8_t dlc) {
  // Build the line into a local buffer, then dualWrite it in one call
  // to avoid interleaving Serial and WiFi bytes.
  char buf[96];
  int  pos = snprintf(buf, sizeof(buf),
                      "[%lums] ID: 0x%08X DLC: %d Data:",
                      (unsigned long)millis(), id, dlc);
  for (int i = 0; i < dlc && pos < (int)sizeof(buf) - 4; i++)
    pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", d[i]);
  // Append newline
  if (pos < (int)sizeof(buf) - 2) { buf[pos++] = '\n'; buf[pos] = '\0'; }
  dualWrite(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Cell voltage helpers (unchanged)
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
//  BMS frame transmitters (unchanged logic, use log_frame which now dualWrites)
// ─────────────────────────────────────────────────────────────────────────────
void tx_bms_msg1() {
  uint8_t d[8] = {0};
  d[0] = vs.status_byte;
  d[1] = vs.soc;
  pu16le(d, 2, enc_pack_i(vs.pack_current_a));
  pu16le(d, 4, enc_v01(vs.pack_voltage_v));
  d[6] = vs.fault_level;
  d[7] = vs.error_code;
  can_send(ID_BMS_MSG1, d, 8);
  log_frame(ID_BMS_MSG1, d, 8);
}

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
  pu16le(d, 6, enc_i01(vs.max_dch_i));
  can_send(ID_BMS_MSG2, d, 8);
  log_frame(ID_BMS_MSG2, d, 8);
}

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

void tx_bms_temps() {
  uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  for (int i = 0; i < 4; i++) d[i] = enc_t(vs.bms_t[i]);
  can_send(ID_BMS_TEMPS, d, 8);
  log_frame(ID_BMS_TEMPS, d, 8);
}

void tx_chg_request() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_v01(vs.chg_v_max));
  pu16le(d, 2, enc_i01(vs.chg_i_max));
  can_send(ID_BMS_CHG_REQ, d, 8);
  log_frame(ID_BMS_CHG_REQ, d, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Sensor frame transmitters (unchanged logic)
// ─────────────────────────────────────────────────────────────────────────────
void tx_solar_current() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_i01(vs.solar_i_raw));
  pu16le(d, 2, enc_i01(vs.solar_i_avg));
  d[4] = vs.solar_i_status;
  can_send(ID_SOLAR_CURRENT, d, 8);
  log_frame(ID_SOLAR_CURRENT, d, 8);
}

void tx_solar_voltage() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_v01(vs.solar_v));
  pu16le(d, 2, enc_v01(vs.solar_voc));
  d[4] = vs.solar_v_status;
  can_send(ID_SOLAR_VOLTAGE, d, 8);
  log_frame(ID_SOLAR_VOLTAGE, d, 8);
}

void tx_mppt_output() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_i01(vs.mppt_out_i));
  d[2] = vs.mppt_mode;
  d[3] = vs.mppt_efficiency;
  d[4] = vs.mppt_status;
  can_send(ID_MPPT_OUTPUT, d, 8);
  log_frame(ID_MPPT_OUTPUT, d, 8);
}

void tx_dcdc_output() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_v001(vs.dcdc_v));
  d[2] = 0xFF; d[3] = 0xFF;
  d[4] = vs.dcdc_status;
  can_send(ID_DCDC_OUTPUT, d, 8);
  log_frame(ID_DCDC_OUTPUT, d, 8);
}

void tx_motor_temp() {
  uint8_t d[8] = {0};
  d[0] = enc_t(vs.motor_t_winding);
  d[1] = enc_t(vs.motor_t_housing);
  d[2] = vs.motor_t_status;
  can_send(ID_MOTOR_TEMP, d, 8);
  log_frame(ID_MOTOR_TEMP, d, 8);
}

void tx_mppt_temp() {
  uint8_t d[8] = {0};
  d[0] = enc_t(vs.mppt_t);
  d[1] = vs.mppt_t_status;
  can_send(ID_MPPT_TEMP, d, 8);
  log_frame(ID_MPPT_TEMP, d, 8);
}

void tx_cabin_temp() {
  uint8_t d[8] = {0};
  d[0] = enc_t(vs.cabin_t);
  d[1] = vs.cabin_humidity;
  d[2] = vs.cabin_status;
  can_send(ID_CABIN_TEMP, d, 8);
  log_frame(ID_CABIN_TEMP, d, 8);
}

void tx_handbrake() {
  uint8_t d[8] = {0};
  d[0] = vs.handbrake;
  d[1] = vs.hb_debounce;
  can_send(ID_HANDBRAKE, d, 8);
  log_frame(ID_HANDBRAKE, d, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SENSOR text line — now sent on both Serial and WiFi
// ─────────────────────────────────────────────────────────────────────────────
void print_sensor_line(const char *name, uint32_t countdown) {
  dualPrintf(
    "SENSOR:solar_v=%.2f solar_i_in=%.2f solar_i_out=%.2f "
    "aux12v=%.2f hv64v=%.2f "
    "motor_t=%.1f mppt_t=%.1f cabin_t=%.1f "
    "handbrake=%d "
    "scenario_name=%s scenario_countdown=%lu\n",
    vs.solar_v, vs.solar_i_raw, vs.mppt_out_i,
    vs.dcdc_v, vs.pack_voltage_v,
    vs.motor_t_winding, vs.mppt_t, vs.cabin_t,
    vs.handbrake,
    name, (unsigned long)countdown
  );
}

// ─────────────────────────────────────────────────────────────────────────────
//  SCENARIO DEFINITIONS  (100% identical to Serial-only version)
// ─────────────────────────────────────────────────────────────────────────────

// ── 0: IDLE PARKED ────────────────────────────────────────────────────────────
void scenario_idle_parked() {
  vs.soc           = 75;
  vs.pack_current_a = 0.0f;
  set_cells(soc_to_cell_mv(vs.soc), 3.0f);
  vs.status_byte   = 0b00011000;
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(22.0f, 0.3f, 8000);
  vs.bms_t[1] = osc(21.5f, 0.3f, 9000);
  vs.bms_t[2] = osc(22.2f, 0.2f, 7000);
  vs.bms_t[3] = osc(21.8f, 0.2f, 10000);
  vs.max_dch_i  = 100.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw  = 0.0f;
  vs.solar_i_avg  = solar_avg(0.0f);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = 0.0f; vs.solar_voc = 75.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = 0.0f; vs.mppt_mode = MPPT_OFF; vs.mppt_efficiency = 0; vs.mppt_status = STS_OK;
  vs.dcdc_v       = osc(12.45f, 0.05f, 5000); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(26.0f, 0.5f, 12000);
  vs.motor_t_housing = vs.motor_t_winding - 1.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = 24.0f; vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 22.0f; vs.cabin_humidity = 55; vs.cabin_status = STS_OK;
  vs.handbrake    = 1; vs.hb_debounce = 0;
}

// ── 1: CITY DRIVE ─────────────────────────────────────────────────────────────
void scenario_city_drive() {
  vs.soc           = (uint8_t)constrain(68 - (int)((millis() % SIM_DURATION_MS) / 3000), 60, 70);
  vs.pack_current_a = osc(18.0f, 5.0f, 4000);
  set_cells(soc_to_cell_mv(vs.soc), osc(6.0f, 2.0f, 7000));
  vs.status_byte   = 0b00010100;
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(28.0f, 1.5f, 5000);
  vs.bms_t[1] = osc(27.5f, 1.0f, 6000);
  vs.bms_t[2] = osc(29.0f, 1.5f, 4500);
  vs.bms_t[3] = osc(28.5f, 1.0f, 7000);
  vs.max_dch_i  = 100.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw  = osc(4.5f, 0.8f, 5000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = osc(68.0f, 3.0f, 7000); vs.solar_voc = 75.5f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = osc(3.8f, 0.4f, 4500); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 93; vs.mppt_status = STS_OK;
  vs.dcdc_v       = osc(12.3f, 0.15f, 3000); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(55.0f, 5.0f, 6000);
  vs.motor_t_housing = vs.motor_t_winding - 7.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = osc(42.0f, 2.0f, 9000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 24.0f; vs.cabin_humidity = 52; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 2: SOLAR CHARGING ─────────────────────────────────────────────────────────
void scenario_solar_charging() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc           = (uint8_t)constrain(50 + (int)(elapsed / 1000), 50, 70);
  vs.pack_current_a = osc(-18.0f, 2.0f, 5000);
  set_cells(soc_to_cell_mv(vs.soc), 4.0f);
  vs.status_byte   = 0b00100010;
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(25.0f, 1.0f, 8000);
  vs.bms_t[1] = osc(24.5f, 0.8f, 9000);
  vs.bms_t[2] = osc(25.5f, 1.0f, 7000);
  vs.bms_t[3] = osc(25.0f, 0.8f, 10000);
  vs.max_dch_i  = 0.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 35.0f;

  vs.solar_i_raw  = osc(22.0f, 1.5f, 6000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = osc(82.0f, 2.0f, 8000); vs.solar_voc = 90.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = osc(19.0f, 1.0f, 5000); vs.mppt_mode = MPPT_CV; vs.mppt_efficiency = 96; vs.mppt_status = STS_OK;
  vs.dcdc_v       = 13.8f; vs.dcdc_status = STS_OK;
  vs.motor_t_winding = 28.0f; vs.motor_t_housing = 27.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = osc(55.0f, 3.0f, 7000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 28.0f; vs.cabin_humidity = 48; vs.cabin_status = STS_OK;
  vs.handbrake    = 1; vs.hb_debounce = 0;
}

// ── 3: HIGHWAY CRUISE ─────────────────────────────────────────────────────────
void scenario_highway() {
  vs.soc           = (uint8_t)constrain(55 - (int)((millis() % SIM_DURATION_MS) / 2500), 40, 56);
  vs.pack_current_a = osc(42.0f, 6.0f, 3000);
  set_cells(soc_to_cell_mv(vs.soc), osc(10.0f, 3.0f, 8000));
  vs.status_byte   = 0b00010100;
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(34.0f, 2.0f, 4000);
  vs.bms_t[1] = osc(33.0f, 1.5f, 5000);
  vs.bms_t[2] = osc(35.0f, 2.0f, 3500);
  vs.bms_t[3] = osc(34.5f, 1.5f, 6000);
  vs.max_dch_i  = 100.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw  = osc(7.0f, 1.0f, 4000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = osc(71.0f, 2.5f, 6000); vs.solar_voc = 77.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = osc(5.5f, 0.5f, 3500); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 92; vs.mppt_status = STS_OK;
  vs.dcdc_v       = osc(12.1f, 0.12f, 2500); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(72.0f, 4.0f, 5000);
  vs.motor_t_housing = vs.motor_t_winding - 8.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = osc(48.0f, 2.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 23.0f; vs.cabin_humidity = 50; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 4: LOW BATTERY ────────────────────────────────────────────────────────────
void scenario_low_battery() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc           = (uint8_t)constrain(12 - (int)(elapsed / 5000), 5, 13);
  vs.pack_current_a = osc(8.0f, 2.0f, 4000);
  float avg_mv = soc_to_cell_mv(vs.soc);
  set_cells(avg_mv, osc(18.0f, 5.0f, 6000));
  vs.status_byte   = 0b00010100;
  vs.fault_level   = 1;
  vs.error_code    = 0x03;
  vs.bms_t[0] = osc(30.0f, 1.0f, 6000);
  vs.bms_t[1] = osc(29.5f, 1.0f, 7000);
  vs.bms_t[2] = osc(30.5f, 1.0f, 5500);
  vs.bms_t[3] = osc(30.0f, 0.8f, 8000);
  vs.max_dch_i  = 35.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw  = osc(1.2f, 0.3f, 4000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_WARN;
  vs.solar_v      = osc(52.0f, 4.0f, 6000); vs.solar_voc = 72.0f; vs.solar_v_status = STS_WARN;
  vs.mppt_out_i   = osc(1.0f, 0.2f, 3000); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 88; vs.mppt_status = STS_WARN;
  vs.dcdc_v       = osc(11.6f, 0.25f, 2000); vs.dcdc_status = STS_WARN;
  vs.motor_t_winding = osc(40.0f, 2.0f, 7000);
  vs.motor_t_housing = vs.motor_t_winding - 6.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = 36.0f; vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 27.0f; vs.cabin_humidity = 60; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 5: MOTOR OVERTEMP ─────────────────────────────────────────────────────────
void scenario_overtemp_motor() {
  vs.soc           = 45;
  vs.pack_current_a = osc(25.0f, 5.0f, 4000);
  set_cells(soc_to_cell_mv(vs.soc), 7.0f);
  vs.status_byte   = 0b00010100;
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(31.0f, 1.5f, 5000);
  vs.bms_t[1] = osc(30.5f, 1.0f, 6000);
  vs.bms_t[2] = osc(32.0f, 1.5f, 4500);
  vs.bms_t[3] = osc(31.5f, 1.0f, 7000);
  vs.max_dch_i  = 100.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw  = osc(5.0f, 0.5f, 4500);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = osc(70.0f, 2.5f, 7000); vs.solar_voc = 78.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = osc(4.2f, 0.4f, 4000); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 91; vs.mppt_status = STS_OK;
  vs.dcdc_v       = osc(12.2f, 0.15f, 3000); vs.dcdc_status = STS_OK;

  uint32_t elapsed = millis() % SIM_DURATION_MS;
  float t_rise = 65.0f + (float)elapsed / 1000.0f * 3.5f;
  t_rise = constrain(t_rise, 65.0f, 135.0f);
  vs.motor_t_winding = t_rise;
  vs.motor_t_housing = t_rise - 9.0f;
  if (t_rise > 100.0f)     vs.motor_t_status = STS_FAULT;
  else if (t_rise > 80.0f) vs.motor_t_status = STS_WARN;
  else                      vs.motor_t_status = STS_OK;

  vs.mppt_t       = osc(50.0f, 4.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 29.0f; vs.cabin_humidity = 58; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 6: FLOAT CHARGING ─────────────────────────────────────────────────────────
void scenario_float_charge() {
  vs.soc           = (uint8_t)constrain(95 + (int)((millis() % SIM_DURATION_MS) / 5000), 95, 100);
  vs.pack_current_a = osc(-2.5f, 0.5f, 5000);
  set_cells(soc_to_cell_mv(vs.soc), 2.0f);
  vs.status_byte   = 0b00100010;
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(23.5f, 0.5f, 9000);
  vs.bms_t[1] = osc(23.0f, 0.5f, 10000);
  vs.bms_t[2] = osc(24.0f, 0.5f, 8000);
  vs.bms_t[3] = osc(23.5f, 0.5f, 11000);
  vs.max_dch_i  = 0.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 5.0f;

  vs.solar_i_raw  = osc(3.0f, 0.4f, 7000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = osc(76.0f, 1.5f, 9000); vs.solar_voc = 82.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = osc(2.5f, 0.3f, 6000); vs.mppt_mode = MPPT_FLOAT; vs.mppt_efficiency = 95; vs.mppt_status = STS_OK;
  vs.dcdc_v       = 13.6f; vs.dcdc_status = STS_OK;
  vs.motor_t_winding = 26.0f; vs.motor_t_housing = 25.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = osc(38.0f, 2.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 21.0f; vs.cabin_humidity = 52; vs.cabin_status = STS_OK;
  vs.handbrake    = 1; vs.hb_debounce = 0;
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
  { "IDLE_PARKED",    "P  IDLE - PARKED",          scenario_idle_parked    },
  { "CITY_DRIVE",     "CITY DRIVE",                scenario_city_drive     },
  { "SOLAR_CHARGING", "SOLAR CHARGING",            scenario_solar_charging },
  { "HIGHWAY",        "HIGHWAY CRUISE",            scenario_highway        },
  { "LOW_BATTERY",    "LOW BATTERY WARNING",       scenario_low_battery    },
  { "OVERTEMP_MOTOR", "MOTOR OVERTEMP",            scenario_overtemp_motor },
  { "FLOAT_CHARGE",   "FLOAT CHARGE (FULL)",       scenario_float_charge   },
};
static const uint8_t N_SIM = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);

uint8_t  cur_scn      = 0;
uint32_t scn_start_ms = 0;

void advance_scenario() {
  cur_scn = (cur_scn + 1) % N_SIM;
  scn_start_ms = millis();
  for (int i = 0; i < 5; i++) solar_i_buf[i] = 0;
  solar_i_idx = 0;

  // Announce on both transports
  dualPrintf("\n[SIM] ============================================\n");
  dualPrintf("[SIM]  SCENARIO: %s\n", SCENARIOS[cur_scn].display_name);
  dualPrintf("[SIM]  Duration: %lu s\n", (unsigned long)(SIM_DURATION_MS / 1000));
  dualPrintf("[SIM] ============================================\n\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // ── WiFi Access Point ──────────────────────────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  delay(100);   // let AP stabilise

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  Serial.printf("\n[WiFi] AP  SSID: %s  Ch: %d\n", WIFI_SSID, WIFI_CHANNEL);
  Serial.printf("[WiFi] IP : %s  Port: %d\n",
                WiFi.softAPIP().toString().c_str(), TCP_PORT);
  Serial.printf("[WiFi] Waiting for TCP client (server_combined.py)...\n\n");

  // ── CAN / TWAI ─────────────────────────────────────────────────────────────
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
  ESP_ERROR_CHECK(twai_start());

  scn_start_ms = millis();

  Serial.printf("[BAKO] Full CAN Bus Simulator - 250 kbps\n");
  Serial.printf("[BAKO] Frames: 9 BMS (SA=F4) + 8 sensor (SA=AA)\n");
  Serial.printf("[BAKO] %d scenarios x %lu s each\n\n",
                N_SIM, (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ============================================\n");
  Serial.printf("[SIM]  SCENARIO: %s\n", SCENARIOS[cur_scn].display_name);
  Serial.printf("[SIM]  Duration: %lu s\n", (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ============================================\n\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop — independent timers for every frame type
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // Accept any new TCP connection from the PC
  wifi_accept_pending();

  // Per-frame last-TX timestamps
  static uint32_t t_msg1    = 0, t_msg2    = 0;
  static uint32_t t_cells   = 0, t_btemps  = 0, t_chgreq = 0;
  static uint32_t t_solar_i = 0, t_solar_v = 0, t_mppt   = 0;
  static uint32_t t_dcdc    = 0, t_mtemp   = 0;
  static uint32_t t_mppt_t  = 0, t_cabin   = 0, t_hb     = 0;
  static uint32_t t_sensor  = 0;

  uint32_t now = millis();

  // Scenario advance
  if (now - scn_start_ms >= SIM_DURATION_MS) advance_scenario();

  // Update vehicle state for current scenario
  SCENARIOS[cur_scn].fn();

  uint32_t countdown = (SIM_DURATION_MS - (now - scn_start_ms)) / 1000;

  // ── BMS frames (SA = 0xF4) ─────────────────────────────────────────────────
  if (now - t_msg1    >= CY_BMS_MSG1)   { t_msg1   = now; tx_bms_msg1();     }
  if (now - t_msg2    >= CY_BMS_MSG2)   { t_msg2   = now; tx_bms_msg2();     }
  if (now - t_cells   >= CY_CELLS)      { t_cells  = now; tx_cell_voltages(); }
  if (now - t_btemps  >= CY_BMS_TEMPS)  { t_btemps = now; tx_bms_temps();    }
  if (now - t_chgreq  >= CY_CHG_REQ)    { t_chgreq = now; tx_chg_request();  }

  // ── ESP32 sensor frames (SA = 0xAA) ───────────────────────────────────────
  if (now - t_solar_i >= CY_SOLAR_I)    { t_solar_i = now; tx_solar_current(); }
  if (now - t_solar_v >= CY_SOLAR_V)    { t_solar_v = now; tx_solar_voltage(); }
  if (now - t_mppt    >= CY_MPPT_OUT)   { t_mppt    = now; tx_mppt_output();   }
  if (now - t_dcdc    >= CY_DCDC_OUT)   { t_dcdc    = now; tx_dcdc_output();   }
  if (now - t_mtemp   >= CY_MOTOR_TEMP) { t_mtemp   = now; tx_motor_temp();    }
  if (now - t_mppt_t  >= CY_MPPT_TEMP)  { t_mppt_t  = now; tx_mppt_temp();     }
  if (now - t_cabin   >= CY_CABIN_TEMP) { t_cabin   = now; tx_cabin_temp();    }
  if (now - t_hb      >= CY_HANDBRAKE)  { t_hb      = now; tx_handbrake();     }

  // ── Human-readable SENSOR line at 1 Hz ────────────────────────────────────
  if (now - t_sensor >= 1000) {
    t_sensor = now;
    print_sensor_line(SCENARIOS[cur_scn].id_name, countdown);
  }

  delay(2);
}
