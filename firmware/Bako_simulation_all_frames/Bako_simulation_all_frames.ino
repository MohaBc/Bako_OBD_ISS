/*
 * BAKO SMU — ESP32 Full CAN Bus Simulator
 * ========================================
 * Simulates ALL CAN frames on the BAKO 60V/100Ah LFP network:
 *
 * BMS frames  (SA = 0xF4, as real BMS would send):
 *   0x18FF28F4  BMS Basic Msg 1  — status flags + SOC + pack current + pack voltage + fault
 *   0x18FE28F4  BMS Basic Msg 2  — max/min cell V + max/min temp + max disch current
 *   0x18C828F4  Cell voltages cells  1-4   (big-endian uint16 mV pairs)
 *   0x18C928F4  Cell voltages cells  5-8
 *   0x18CA28F4  Cell voltages cells  9-12
 *   0x18CB28F4  Cell voltages cells 13-16
 *   0x18CC28F4  Cell voltages cells 17-19 (byte pair 8 = 0x0000 padding)
 *   0x18B428F4  Temperature probes 1-4  (uint8, offset -40, 0xFF = NC)
 *   0x18FFE5F4  BMS charging request
 *
 * ESP32 sensor frames  (SA = 0xAA):
 *   0x18D001AA  Solar panel current before MPPT
 *   0x18D101AA  Solar panel voltage before MPPT
 *   0x18D201AA  MPPT output current + mode + efficiency
 *   0x18D301AA  DC/DC 12V output voltage
 *   0x18D401AA  Motor winding + housing temperature
 *   0x18D501AA  MPPT heatsink temperature
 *   0x18D601AA  Cabin interior temperature + humidity
 *   0x18D701AA  Handbrake position
 *
 * Baud: 250 kbps  |  TX: GPIO5  |  RX: GPIO4
 *
 * 7 scenarios  ×  20 s each — scenario name is printed to Serial
 * at every transition so the dashboard log always shows what is running.
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <math.h>

// ── CAN pins ─────────────────────────────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

// ── BMS frame IDs (SA = 0xF4) ────────────────────────────────────────────────
#define ID_BMS_MSG1      0x18FF28F4   // 100 ms  — status + SOC + current + voltage + fault
#define ID_BMS_MSG2      0x18FE28F4   // 100 ms  — min/max cell + temps + disch limit
#define ID_CELLS_1_4     0x18C828F4   // 500 ms  — cells  1-4   big-endian uint16 mV
#define ID_CELLS_5_8     0x18C928F4   // 500 ms  — cells  5-8
#define ID_CELLS_9_12    0x18CA28F4   // 500 ms  — cells  9-12
#define ID_CELLS_13_16   0x18CB28F4   // 500 ms  — cells 13-16
#define ID_CELLS_17_19   0x18CC28F4   // 500 ms  — cells 17-19 + 0x0000 padding
#define ID_BMS_TEMPS     0x18B428F4   // 500 ms  — up to 4 temp probes
#define ID_BMS_CHG_REQ   0x18FFE5F4   // 1000 ms — max charge voltage + current

// ── ESP32 sensor frame IDs (SA = 0xAA) ───────────────────────────────────────
#define ID_SOLAR_CURRENT 0x18D001AA   // 200 ms
#define ID_SOLAR_VOLTAGE 0x18D101AA   // 200 ms
#define ID_MPPT_OUTPUT   0x18D201AA   // 200 ms
#define ID_DCDC_OUTPUT   0x18D301AA   // 500 ms
#define ID_MOTOR_TEMP    0x18D401AA   // 1000 ms
#define ID_MPPT_TEMP     0x18D501AA   // 1000 ms
#define ID_CABIN_TEMP    0x18D601AA   // 1000 ms
#define ID_HANDBRAKE     0x18D701AA   // 100 ms

// ── TX cycle periods (ms) ────────────────────────────────────────────────────
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

// ── MPPT modes ───────────────────────────────────────────────────────────────
#define MPPT_OFF       0x00
#define MPPT_TRACKING  0x01
#define MPPT_CV        0x02
#define MPPT_FLOAT     0x03

// ── Sensor status codes ──────────────────────────────────────────────────────
#define STS_OK    0x00
#define STS_WARN  0x01
#define STS_FAULT 0x02

// ── BAKO 19S1P LFP pack constants ────────────────────────────────────────────
#define N_CELLS          19
#define CELL_FULL_MV     3387    // 100% SOC (car display top)
#define CELL_EMPTY_MV    2500    // 0% SOC
#define CELL_OV_MV       3647   // BMS charge cut-off / cell
#define CELL_UV_MV       2553   // BMS discharge cut-off / cell
#define PACK_CHG_CUTOFF  693    // 69.3 V  × 10 (0.1 V/bit)
#define MAX_CHG_I_RAW    350    // 35.0 A  × 10 (0.1 A/bit)
#define MAX_DCH_I_RAW    1000   // 100.0 A × 10 (0.1 A/bit)

#define SIM_DURATION_MS  20000UL   // 20 s per scenario

// ─────────────────────────────────────────────────────────────────────────────
//  Full vehicle state — one struct for BMS + sensors
// ─────────────────────────────────────────────────────────────────────────────
struct VehicleState {
  // ── BMS / Battery ─────────────────────────────────────────────────────────
  uint8_t  soc;             // 0-100 %  (byte 2 of MSG1)
  float    pack_current_a;  // A  (+discharge / -charge)
  float    pack_voltage_v;  // V  (derived from cells)
  uint8_t  status_byte;     // byte 1 of MSG1  (bit-field)
  uint8_t  fault_level;     // byte 7 of MSG1
  uint8_t  error_code;      // byte 8 of MSG1

  uint16_t cell_mv[N_CELLS]; // individual cell voltages in mV
  // Derived from cells:
  uint16_t cell_max_mv;
  uint16_t cell_min_mv;

  // BMS temperature probes (offset-40 raw; 0xFF = NC)
  float    bms_t[4];         // °C  — probes 1-4

  // Max discharge current (MSG2 bytes 7-8)
  float    max_dch_i;        // A

  // Charging request
  float    chg_v_max;        // V  max allowed charge voltage
  float    chg_i_max;        // A  max allowed charge current

  // ── ESP32 sensors ─────────────────────────────────────────────────────────
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

  float    dcdc_v;           // V  12V bus
  uint8_t  dcdc_status;

  float    motor_t_winding;  // °C
  float    motor_t_housing;  // °C  (0xFF = NC)
  uint8_t  motor_t_status;

  float    mppt_t;           // °C
  uint8_t  mppt_t_status;

  float    cabin_t;          // °C
  uint8_t  cabin_humidity;   // % RH
  uint8_t  cabin_status;

  uint8_t  handbrake;        // 0=released 1=engaged
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
//  Encode helpers
// ─────────────────────────────────────────────────────────────────────────────
static inline void pu16le(uint8_t *b, int o, uint16_t v) {
  b[o] = v & 0xFF; b[o+1] = (v >> 8) & 0xFF;
}
static inline void pu16be(uint8_t *b, int o, uint16_t v) {
  b[o] = (v >> 8) & 0xFF; b[o+1] = v & 0xFF;
}
static inline uint16_t enc_i01(float a) {           // 0.1 A/bit  LE
  int32_t v = (int32_t)roundf(a * 10.0f);
  return (uint16_t)constrain(v, 0, 65535);
}
static inline uint16_t enc_v01(float v) {           // 0.1 V/bit  LE
  return (uint16_t)constrain((int32_t)roundf(v * 10.0f), 0, 65535);
}
static inline uint16_t enc_v001(float v) {          // 0.01 V/bit LE
  return (uint16_t)constrain((int32_t)roundf(v * 100.0f), 0, 65535);
}
static inline uint8_t enc_t(float c) {              // uint8 offset-40
  if (c < -40.0f || c > 215.0f) return 0xFF;
  return (uint8_t)roundf(c + 40.0f);
}
// Pack current: offset 5000, scale 0.1 A/bit  (charging = negative = < 5000)
static inline uint16_t enc_pack_i(float amps) {
  int32_t v = (int32_t)roundf(amps * 10.0f) + 5000;
  return (uint16_t)constrain(v, 0, 65535);
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

// Log helper — prints one CAN frame line to Serial in the format the server expects
void log_frame(uint32_t id, const uint8_t *d, uint8_t dlc) {
  Serial.printf("[%lums] ID: 0x%08X DLC: %d Data:", millis(), id, dlc);
  for (int i = 0; i < dlc; i++) Serial.printf(" %02X", d[i]);
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Cell voltage helpers
// ─────────────────────────────────────────────────────────────────────────────

// Set all 19 cells to a base voltage with a small spread (simulates balancing state)
void set_cells(float avg_mv, float spread_mv) {
  for (int i = 0; i < N_CELLS; i++) {
    float offset = ((float)i / (N_CELLS - 1) - 0.5f) * spread_mv;
    float mv = avg_mv + offset;
    mv = constrain(mv, (float)CELL_UV_MV, (float)CELL_OV_MV);
    vs.cell_mv[i] = (uint16_t)roundf(mv);
  }
  // Derive pack stats
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

// SOC → average cell voltage (linear LFP approximation)
float soc_to_cell_mv(uint8_t soc) {
  return CELL_EMPTY_MV + (float)soc / 100.0f * (CELL_FULL_MV - CELL_EMPTY_MV);
}

// Small oscillation helper
float osc(float centre, float amp, uint32_t period_ms) {
  return centre + amp * sinf(2.0f * M_PI * (float)millis() / (float)period_ms);
}

// ─────────────────────────────────────────────────────────────────────────────
//  BMS frame transmitters
// ─────────────────────────────────────────────────────────────────────────────

/*
 * 0x18FF28F4 — BMS Basic Message 1  (100 ms)
 *
 * Byte 1 : status bitfield
 *   bit1 = charging cable connected
 *   bit2 = pack charging status
 *   bit3 = pack discharge status
 *   bit4 = pack ready
 *   bit5 = discharge contactor
 *   bit6 = charge contactor
 *
 * Byte 2 : SOC %  (0-100, scale 1)
 * Bytes 3-4 : pack current LE uint16, offset 5000, scale 0.1 A/bit
 * Bytes 5-6 : pack voltage LE uint16, scale 0.1 V/bit
 * Byte 7 : fault level  (0=none, 1=Level1)
 * Byte 8 : error code   (0=normal)
 */
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

/*
 * 0x18FE28F4 — BMS Basic Message 2  (100 ms)
 *
 * Bytes 1-2 : max cell voltage LE uint16, 1 mV/bit
 * Bytes 3-4 : min cell voltage LE uint16, 1 mV/bit
 * Byte  5   : max cell temp uint8, offset -40
 * Byte  6   : min cell temp uint8, offset -40
 * Bytes 7-8 : max allowable discharge current LE uint16, 0.1 A/bit
 */
void tx_bms_msg2() {
  uint8_t d[8] = {0};
  pu16le(d, 0, vs.cell_max_mv);
  pu16le(d, 2, vs.cell_min_mv);
  // Find max and min of BMS probes
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

/*
 * 0x18C8..CC28F4 — Cell voltage frames  (500 ms)
 * Big-endian uint16 pairs per BAKO doc section 3.3
 * Last pair of 0x18CC28F4 is 0x0000 padding (no cell 20)
 */
void tx_cell_voltages() {
  uint32_t ids[5] = {
    ID_CELLS_1_4, ID_CELLS_5_8, ID_CELLS_9_12, ID_CELLS_13_16, ID_CELLS_17_19
  };
  for (int g = 0; g < 5; g++) {
    uint8_t d[8] = {0};
    for (int i = 0; i < 4; i++) {
      int cell_idx = g * 4 + i;  // 0-based
      uint16_t mv = (cell_idx < N_CELLS) ? vs.cell_mv[cell_idx] : 0;
      pu16be(d, i * 2, mv);
    }
    can_send(ids[g], d, 8);
    log_frame(ids[g], d, 8);
  }
}

/*
 * 0x18B428F4 — BMS Temperature probes  (500 ms)
 * Bytes 1-4 : probes 1-4 (uint8, offset -40, 0xFF = NC)
 * Bytes 5-8 : probes 5-8 = 0xFF (not connected on this unit)
 */
void tx_bms_temps() {
  uint8_t d[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  for (int i = 0; i < 4; i++) d[i] = enc_t(vs.bms_t[i]);
  can_send(ID_BMS_TEMPS, d, 8);
  log_frame(ID_BMS_TEMPS, d, 8);
}

/*
 * 0x18FFE5F4 — BMS Charging Request  (1000 ms)
 * Bytes 1-2 : max charge voltage LE uint16, 0.1 V/bit  → 69.3 V = 0x02B5
 * Bytes 3-4 : max charge current LE uint16, 0.1 A/bit
 * Bytes 5-8 : control + protection window (0x00 when not charging)
 */
void tx_chg_request() {
  uint8_t d[8] = {0};
  pu16le(d, 0, enc_v01(vs.chg_v_max));
  pu16le(d, 2, enc_i01(vs.chg_i_max));
  // d[4]-d[7] = 0x00 (no active fault, not currently charging in most scenarios)
  can_send(ID_BMS_CHG_REQ, d, 8);
  log_frame(ID_BMS_CHG_REQ, d, 8);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Sensor frame transmitters
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
  d[2] = 0xFF; d[3] = 0xFF;     // current sensor not fitted
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
//  SENSOR text line (server's fallback / dashboard log enrichment)
// ─────────────────────────────────────────────────────────────────────────────
void print_sensor_line(const char *name, uint32_t countdown) {
  Serial.printf(
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
//  SCENARIO DEFINITIONS
//  Each function fills the entire VehicleState for that scenario.
//  BMS data (cells, SOC, current, temps) + sensor data in one place.
// ─────────────────────────────────────────────────────────────────────────────

// ── 0: IDLE PARKED ──────────────────────────────────────────────────────────
// SOC ~75%, zero current, stable temps, handbrake ON, MPPT off (no sun)
void scenario_idle_parked() {
  // BMS
  vs.soc           = 75;
  vs.pack_current_a = 0.0f;
  set_cells(soc_to_cell_mv(vs.soc), 3.0f);  // tight 3 mV spread
  vs.status_byte   = 0b00011000;  // ready=1, discharge_contactor=1
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(22.0f, 0.3f, 8000);
  vs.bms_t[1] = osc(21.5f, 0.3f, 9000);
  vs.bms_t[2] = osc(22.2f, 0.2f, 7000);
  vs.bms_t[3] = osc(21.8f, 0.2f, 10000);
  vs.max_dch_i  = 100.0f;
  vs.chg_v_max  = 69.3f;
  vs.chg_i_max  = 0.0f;   // not requesting charge

  // Sensors
  vs.solar_i_raw  = 0.0f;
  vs.solar_i_avg  = solar_avg(0.0f);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = 0.0f; vs.solar_voc = 75.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = 0.0f; vs.mppt_mode = MPPT_OFF; vs.mppt_efficiency = 0; vs.mppt_status = STS_OK;
  vs.dcdc_v       = osc(12.45f, 0.05f, 5000); vs.dcdc_status = STS_OK;
  vs.motor_t_winding = osc(26.0f, 0.5f, 12000);
  vs.motor_t_housing = vs.motor_t_winding - 1.0f;
  vs.motor_t_status  = STS_OK;
  vs.mppt_t       = 24.0f; vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 22.0f; vs.cabin_humidity = 55; vs.cabin_status = STS_OK;
  vs.handbrake    = 1; vs.hb_debounce = 0;
}

// ── 1: CITY DRIVE ───────────────────────────────────────────────────────────
// SOC slowly draining, moderate discharge current, solar assisting, brake OFF
void scenario_city_drive() {
  vs.soc           = (uint8_t)constrain(68 - (int)((millis() % SIM_DURATION_MS) / 3000), 60, 70);
  vs.pack_current_a = osc(18.0f, 5.0f, 4000);  // 13-23 A discharge
  set_cells(soc_to_cell_mv(vs.soc), osc(6.0f, 2.0f, 7000));
  vs.status_byte   = 0b00010100;  // discharge_status=1, ready=1
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
  vs.motor_t_housing = vs.motor_t_winding - 7.0f;
  vs.motor_t_status  = STS_OK;
  vs.mppt_t       = osc(42.0f, 2.0f, 9000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 24.0f; vs.cabin_humidity = 52; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 2: SOLAR CHARGING ───────────────────────────────────────────────────────
// SOC rising, negative pack current (charging), MPPT in CV mode
void scenario_solar_charging() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc           = (uint8_t)constrain(50 + (int)(elapsed / 1000), 50, 70);
  vs.pack_current_a = osc(-18.0f, 2.0f, 5000);  // negative = charging
  set_cells(soc_to_cell_mv(vs.soc), 4.0f);
  vs.status_byte   = 0b00100010;  // cable_connected=1, charging=1, charge_contactor=1
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(25.0f, 1.0f, 8000);
  vs.bms_t[1] = osc(24.5f, 0.8f, 9000);
  vs.bms_t[2] = osc(25.5f, 1.0f, 7000);
  vs.bms_t[3] = osc(25.0f, 0.8f, 10000);
  vs.max_dch_i  = 0.0f;   // discharge not allowed while charging
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 35.0f;  // requesting charge

  vs.solar_i_raw  = osc(22.0f, 1.5f, 6000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_OK;
  vs.solar_v      = osc(82.0f, 2.0f, 8000); vs.solar_voc = 90.0f; vs.solar_v_status = STS_OK;
  vs.mppt_out_i   = osc(19.0f, 1.0f, 5000); vs.mppt_mode = MPPT_CV; vs.mppt_efficiency = 96; vs.mppt_status = STS_OK;
  vs.dcdc_v       = 13.8f; vs.dcdc_status = STS_OK;
  vs.motor_t_winding = 28.0f; vs.motor_t_housing = 27.0f; vs.motor_t_status = STS_OK;
  vs.mppt_t       = osc(55.0f, 3.0f, 7000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 28.0f; vs.cabin_humidity = 48; vs.cabin_status = STS_OK;
  vs.handbrake    = 1; vs.hb_debounce = 0;   // parked, charging
}

// ── 3: HIGHWAY CRUISE ───────────────────────────────────────────────────────
// SOC 55%, high sustained discharge, motor warm, cells slightly unbalanced
void scenario_highway() {
  vs.soc           = (uint8_t)constrain(55 - (int)((millis() % SIM_DURATION_MS) / 2500), 40, 56);
  vs.pack_current_a = osc(42.0f, 6.0f, 3000);  // sustained high discharge
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
  vs.motor_t_housing = vs.motor_t_winding - 8.0f;
  vs.motor_t_status  = STS_OK;
  vs.mppt_t       = osc(48.0f, 2.0f, 8000); vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 23.0f; vs.cabin_humidity = 50; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 4: LOW BATTERY ──────────────────────────────────────────────────────────
// SOC 12%, cells near UV threshold, BMS limiting discharge, fault flag raised
void scenario_low_battery() {
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  vs.soc           = (uint8_t)constrain(12 - (int)(elapsed / 5000), 5, 13);
  vs.pack_current_a = osc(8.0f, 2.0f, 4000);   // BMS has limited power to 35%
  float avg_mv = soc_to_cell_mv(vs.soc);
  set_cells(avg_mv, osc(18.0f, 5.0f, 6000));    // cells drifting apart
  vs.status_byte   = 0b00010100;
  vs.fault_level   = 1;   // Level 1 — "total voltage too low"
  vs.error_code    = 0x03; // code 03 = total voltage too low
  vs.bms_t[0] = osc(30.0f, 1.0f, 6000);
  vs.bms_t[1] = osc(29.5f, 1.0f, 7000);
  vs.bms_t[2] = osc(30.5f, 1.0f, 5500);
  vs.bms_t[3] = osc(30.0f, 0.8f, 8000);
  vs.max_dch_i  = 35.0f;   // BMS cut to 35%
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 0.0f;

  vs.solar_i_raw  = osc(1.2f, 0.3f, 4000);
  vs.solar_i_avg  = solar_avg(vs.solar_i_raw);
  vs.solar_i_status = STS_WARN;
  vs.solar_v      = osc(52.0f, 4.0f, 6000); vs.solar_voc = 72.0f; vs.solar_v_status = STS_WARN;
  vs.mppt_out_i   = osc(1.0f, 0.2f, 3000); vs.mppt_mode = MPPT_TRACKING; vs.mppt_efficiency = 88; vs.mppt_status = STS_WARN;
  vs.dcdc_v       = osc(11.6f, 0.25f, 2000); vs.dcdc_status = STS_WARN;
  vs.motor_t_winding = osc(40.0f, 2.0f, 7000);
  vs.motor_t_housing = vs.motor_t_winding - 6.0f;
  vs.motor_t_status  = STS_OK;
  vs.mppt_t       = 36.0f; vs.mppt_t_status = STS_OK;
  vs.cabin_t      = 27.0f; vs.cabin_humidity = 60; vs.cabin_status = STS_OK;
  vs.handbrake    = 0; vs.hb_debounce = 0;
}

// ── 5: MOTOR OVERTEMP ───────────────────────────────────────────────────────
// SOC 45%, normal current, motor climbing past 100°C, fault status rising
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

  // Motor temperature rising over the 20-second window
  uint32_t elapsed = millis() % SIM_DURATION_MS;
  float t_rise = 65.0f + (float)elapsed / 1000.0f * 3.5f;  // +3.5°C/s → peaks ~135°C
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

// ── 6: FLOAT CHARGING (full battery, trickle from solar) ────────────────────
// SOC 98%, very small positive balance current, cells near FULL, MPPT in float
void scenario_float_charge() {
  vs.soc           = (uint8_t)constrain(95 + (int)((millis() % SIM_DURATION_MS) / 5000), 95, 100);
  vs.pack_current_a = osc(-2.5f, 0.5f, 5000);  // tiny trickle charge
  set_cells(soc_to_cell_mv(vs.soc), 2.0f);       // very tight spread at top
  vs.status_byte   = 0b00100010;  // charging + cable + contactor
  vs.fault_level   = 0; vs.error_code = 0;
  vs.bms_t[0] = osc(23.5f, 0.5f, 9000);
  vs.bms_t[1] = osc(23.0f, 0.5f, 10000);
  vs.bms_t[2] = osc(24.0f, 0.5f, 8000);
  vs.bms_t[3] = osc(23.5f, 0.5f, 11000);
  vs.max_dch_i  = 0.0f;
  vs.chg_v_max  = 69.3f; vs.chg_i_max = 5.0f;  // low float current

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
  const char *id_name;       // used in SENSOR: line
  const char *display_name;  // printed to Serial log
  void (*fn)();
};

static const Scenario SCENARIOS[] = {
  { "IDLE_PARKED",    "🅿  IDLE — PARKED",           scenario_idle_parked    },
  { "CITY_DRIVE",     "🏙  CITY DRIVE",              scenario_city_drive     },
  { "SOLAR_CHARGING", "☀  SOLAR CHARGING",          scenario_solar_charging },
  { "HIGHWAY",        "🛣  HIGHWAY CRUISE",          scenario_highway        },
  { "LOW_BATTERY",    "🪫  LOW BATTERY WARNING",     scenario_low_battery    },
  { "OVERTEMP_MOTOR", "🔥  MOTOR OVERTEMP",         scenario_overtemp_motor },
  { "FLOAT_CHARGE",   "🔋  FLOAT CHARGE (FULL)",    scenario_float_charge   },
};
static const uint8_t N_SIM = sizeof(SCENARIOS) / sizeof(SCENARIOS[0]);

uint8_t  cur_scn        = 0;
uint32_t scn_start_ms   = 0;

void advance_scenario() {
  cur_scn = (cur_scn + 1) % N_SIM;
  scn_start_ms = millis();
  // Clear solar moving average so it doesn't bleed between scenarios
  for (int i = 0; i < 5; i++) solar_i_buf[i] = 0;
  solar_i_idx = 0;

  Serial.printf("\n[SIM] ════════════════════════════════════════════\n");
  Serial.printf("[SIM]  SCENARIO: %s\n", SCENARIOS[cur_scn].display_name);
  Serial.printf("[SIM]  Duration: %lu s\n", (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ════════════════════════════════════════════\n\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
  ESP_ERROR_CHECK(twai_start());

  scn_start_ms = millis();

  Serial.printf("\n[BAKO] Full CAN Bus Simulator — 250 kbps\n");
  Serial.printf("[BAKO] Frames: 9 BMS (SA=F4) + 8 sensor (SA=AA)\n");
  Serial.printf("[BAKO] %d scenarios × %lu s each\n\n",
                N_SIM, (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ════════════════════════════════════════════\n");
  Serial.printf("[SIM]  SCENARIO: %s\n", SCENARIOS[cur_scn].display_name);
  Serial.printf("[SIM]  Duration: %lu s\n", (unsigned long)(SIM_DURATION_MS / 1000));
  Serial.printf("[SIM] ════════════════════════════════════════════\n\n");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop — independent timers for every frame type
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
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

  // Run current scenario to update VehicleState
  SCENARIOS[cur_scn].fn();

  uint32_t countdown = (SIM_DURATION_MS - (now - scn_start_ms)) / 1000;

  // ── BMS frames (SA = 0xF4) ────────────────────────────────────────────────

  if (now - t_msg1 >= CY_BMS_MSG1)   { t_msg1  = now; tx_bms_msg1();     }
  if (now - t_msg2 >= CY_BMS_MSG2)   { t_msg2  = now; tx_bms_msg2();     }
  if (now - t_cells >= CY_CELLS)     { t_cells = now; tx_cell_voltages(); }
  if (now - t_btemps >= CY_BMS_TEMPS){ t_btemps= now; tx_bms_temps();    }
  if (now - t_chgreq >= CY_CHG_REQ)  { t_chgreq= now; tx_chg_request();  }

  // ── ESP32 sensor frames (SA = 0xAA) ──────────────────────────────────────

  if (now - t_solar_i >= CY_SOLAR_I) { t_solar_i = now; tx_solar_current(); }
  if (now - t_solar_v >= CY_SOLAR_V) { t_solar_v = now; tx_solar_voltage(); }
  if (now - t_mppt    >= CY_MPPT_OUT){ t_mppt    = now; tx_mppt_output();   }
  if (now - t_dcdc    >= CY_DCDC_OUT){ t_dcdc    = now; tx_dcdc_output();   }
  if (now - t_mtemp  >= CY_MOTOR_TEMP){ t_mtemp  = now; tx_motor_temp();    }
  if (now - t_mppt_t >= CY_MPPT_TEMP){ t_mppt_t  = now; tx_mppt_temp();     }
  if (now - t_cabin  >= CY_CABIN_TEMP){ t_cabin   = now; tx_cabin_temp();    }
  if (now - t_hb     >= CY_HANDBRAKE){ t_hb      = now; tx_handbrake();     }

  // ── Human-readable SENSOR line at 1 Hz ───────────────────────────────────
  if (now - t_sensor >= 1000) {
    t_sensor = now;
    print_sensor_line(SCENARIOS[cur_scn].id_name, countdown);
  }

  delay(2);
}