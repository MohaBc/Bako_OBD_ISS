/*
 * O'CELL BMS CAN Simulator — 7-Phase + Sensors + WiFi TCP
 * =========================================================
 * Identical 7-phase BMS scenario as CAN_simulation_7_phases, extended with:
 *   • SENSOR_JSON lines matching parse_sensor_json() in server_frame_parser.py
 *   • WiFi TCP connection — lines sent to server the same way as the real
 *     gateway firmware (test sensor temp project)
 *   • LED1 (G25) — connectivity state
 *   • LED2 (G26) — system health / temperature state
 *
 * emitLine() sends every line to BOTH Serial and the TCP socket, so the
 * server_frame_parser.py receives it identically to the real hardware.
 *
 * ── LED1  Connectivity ──────────────────────────────────────────────────────
 *   BLINK_FAST  → WiFi not connected
 *   BLINK_SLOW  → WiFi OK, TCP disconnected / retrying
 *   LED_ON      → TCP connected and live
 *
 * ── LED2  Health ────────────────────────────────────────────────────────────
 *   BLINK_SLOW   → all temps normal
 *   BLINK_DOUBLE → temperature warning  (any sensor > 70 °C)
 *   BLINK_TRIPLE → temperature critical (any sensor > 85 °C)
 *
 * ── SENSOR_JSON fields (server mapping) ────────────────────────────────────
 *   v12_dc_out    → ext.aux12v      12V DC-DC output
 *   v72_dc_in     → ext.hv_iso_v   72V battery input
 *   v72_mppt_in   → ext.dcdc_t     72V MPPT solar input
 *   current_in    → ext.mppt_i1    MPPT input current
 *   current_out   → ext.mppt_i2    MPPT output current
 *   temp_mppt     → ext.bat_t1     MPPT heatsink
 *   temp_dcdc     → ext.bat_t2     DC/DC heatsink
 *   temp_motor    → ext.mppt_t     Motor winding
 *   handbrake     → ext.handbrake  1=engaged 0=released
 *   v12_handbrake → raw only
 *
 * Board : ESP32
 * Baud  : 115200
 */

#include <WiFi.h>

// ─────────────────────────────────────────────────────────────────────────────
// Network config  (match your server_frame_parser.py local mode port)
// ─────────────────────────────────────────────────────────────────────────────
#define WIFI_SSID    "EVENT_SMU"
#define WIFI_PASS    "$mUeV&nt2@25"
#define SERVER_IP    "192.168.56.60"
#define SERVER_PORT  9000

// ─────────────────────────────────────────────────────────────────────────────
// LED pins
// ─────────────────────────────────────────────────────────────────────────────
#define PIN_LED1  25   // connectivity
#define PIN_LED2  26   // health

// ─────────────────────────────────────────────────────────────────────────────
// Temperature alert thresholds (°C)
// ─────────────────────────────────────────────────────────────────────────────
#define TEMP_WARN     70.0f
#define TEMP_CRITICAL 85.0f

// ─────────────────────────────────────────────────────────────────────────────
// LED pattern engine  (identical to test sensor temp firmware)
// ─────────────────────────────────────────────────────────────────────────────
enum LedPattern { LED_OFF, LED_ON, BLINK_SLOW, BLINK_FAST, BLINK_DOUBLE, BLINK_TRIPLE };

struct LedState {
  uint8_t       pin;
  LedPattern    pattern;
  unsigned long lastToggle;
  uint8_t       phase;
};

static const uint16_t SEQ_DOUBLE[5] = { 80, 80, 80, 80, 800 };
static const uint8_t  VAL_DOUBLE[5] = {  1,  0,  1,  0,   0 };
static const uint16_t SEQ_TRIPLE[7] = { 80, 80, 80, 80, 80, 80, 800 };
static const uint8_t  VAL_TRIPLE[7] = {  1,  0,  1,  0,  1,  0,   0 };

LedState led1 = { PIN_LED1, LED_OFF, 0, 0 };
LedState led2 = { PIN_LED2, LED_OFF, 0, 0 };

void setPattern(LedState& led, LedPattern p) {
  if (led.pattern == p) return;
  led.pattern    = p;
  led.phase      = 0;
  led.lastToggle = millis();
}

void updateLed(LedState& led) {
  unsigned long now = millis();
  switch (led.pattern) {
    case LED_OFF:  digitalWrite(led.pin, LOW);  return;
    case LED_ON:   digitalWrite(led.pin, HIGH); return;
    case BLINK_SLOW:
      if (now - led.lastToggle >= 500) {
        led.lastToggle = now;
        digitalWrite(led.pin, !digitalRead(led.pin));
      }
      break;
    case BLINK_FAST:
      if (now - led.lastToggle >= 100) {
        led.lastToggle = now;
        digitalWrite(led.pin, !digitalRead(led.pin));
      }
      break;
    case BLINK_DOUBLE:
      if (now - led.lastToggle >= SEQ_DOUBLE[led.phase]) {
        led.lastToggle = now;
        led.phase      = (led.phase + 1) % 5;
        digitalWrite(led.pin, VAL_DOUBLE[led.phase]);
      }
      break;
    case BLINK_TRIPLE:
      if (now - led.lastToggle >= SEQ_TRIPLE[led.phase]) {
        led.lastToggle = now;
        led.phase      = (led.phase + 1) % 7;
        digitalWrite(led.pin, VAL_TRIPLE[led.phase]);
      }
      break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// TCP client
// ─────────────────────────────────────────────────────────────────────────────
WiFiClient    tcp;
unsigned long tcpRetry = 0;

// Maintain WiFi + TCP — same logic as test sensor temp firmware
void maintainTCP() {
  if (WiFi.status() != WL_CONNECTED) {
    tcp.stop();
    setPattern(led1, BLINK_FAST);
    return;
  }
  if (tcp.connected()) {
    setPattern(led1, LED_ON);
    return;
  }
  setPattern(led1, BLINK_SLOW);
  if (millis() - tcpRetry < 3000) return;
  tcpRetry = millis();
  if (tcp.connect(SERVER_IP, SERVER_PORT))
    Serial.println("[TCP] Connected to server");
  else
    Serial.println("[TCP] Connection failed — retrying in 3 s");
}

// Send to Serial AND TCP socket — identical to real firmware
void emitLine(const String& line) {
  Serial.println(line);
  if (tcp.connected()) tcp.println(line);
}

// ─────────────────────────────────────────────────────────────────────────────
// Cell personalities — fixed per-cell offset in mV
// ─────────────────────────────────────────────────────────────────────────────
const int CELL_OFFSET[19] = {
   2,  0,  5, -3,
   1,  4, -18,  3,
  -1,  2,  6,  0,
   3, -18,  1, -2,
   4,  0,  2
};

// ─────────────────────────────────────────────────────────────────────────────
// BMS live state
// ─────────────────────────────────────────────────────────────────────────────
float soc      = 72.0f;
float packV    = 0.0f;
float currentA = 0.0f;
float tempC[3] = {24.0f, 23.5f, 23.0f};
int   cellMv[19];

// ─────────────────────────────────────────────────────────────────────────────
// Sensor live state
// ─────────────────────────────────────────────────────────────────────────────
float s_v12_dc_out    = 12.8f;
float s_v72_dc_in     = 65.0f;
float s_v72_mppt_in   = 52.0f;
float s_current_in    = 0.5f;
float s_current_out   = 0.4f;
float s_temp_mppt     = 27.0f;
float s_temp_dcdc     = 26.5f;
float s_temp_motor    = 25.0f;
int   s_handbrake     = 1;
float s_v12_handbrake = 12.6f;

// ─────────────────────────────────────────────────────────────────────────────
// Phase engine
// ─────────────────────────────────────────────────────────────────────────────
enum Phase {
  PHASE_REST = 0,
  PHASE_BULK_DISCHARGE,
  PHASE_LOW_SOC,
  PHASE_RECOVERY,
  PHASE_CC_CHARGE,
  PHASE_CV_TAPER,
  PHASE_FULL_REST,
  PHASE_COUNT
};

const int PHASE_TICKS[PHASE_COUNT] = {
  40, 240, 50, 30, 180, 80, 40
};

Phase currentPhase = PHASE_REST;
int   phaseTick    = 0;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
void packBE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = (v >> 8) & 0xFF; buf[o+1] = v & 0xFF;
}
void packLE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = v & 0xFF; buf[o+1] = (v >> 8) & 0xFF;
}

float randNoise(float amp) {
  return ((float)random(-1000, 1001)) / 1000.0f * amp;
}

float socToCellOCV(float s) {
  if (s >= 90.0f) return 3.40f + (s - 90.0f) * 0.0310f;
  if (s >= 20.0f) return 3.20f + (s - 20.0f) * 0.00286f;
  return 2.50f + (s / 20.0f) * 0.70f;
}

// Emit a formatted CAN frame line to Serial + TCP
void printFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
  char buf[96];
  int n = snprintf(buf, sizeof(buf), "[%lums] ID: 0x%08X DLC: %d Data:", millis(), id, dlc);
  for (int i = 0; i < dlc; i++)
    n += snprintf(buf + n, sizeof(buf) - n, " %02X", data[i]);
  emitLine(String(buf));
}

// ─────────────────────────────────────────────────────────────────────────────
// CAN frame senders
// ─────────────────────────────────────────────────────────────────────────────
void sendCellFrames() {
  uint8_t buf[8];
  for (int group = 0; group < 5; group++) {
    memset(buf, 0, 8);
    uint32_t id = (0x98UL << 24) | ((0xC8 + group) << 16) | (0x28 << 8) | 0xF4;
    int base = group * 4;
    for (int i = 0; i < 4; i++) {
      int idx = base + i;
      if (idx < 19) packBE16(buf, i * 2, (uint16_t)cellMv[idx]);
    }
    printFrame(id, buf, 8);
  }
}

void sendCDFrame() {
  uint8_t buf[8] = {0};
  printFrame((0x98UL<<24)|(0xCD<<16)|(0x28<<8)|0xF4, buf, 8);
}

void sendTempFrame() {
  uint8_t buf[8] = {0};
  for (int i = 0; i < 3; i++)
    buf[i] = (uint8_t)constrain(tempC[i] + 40.0f + 0.5f, 0, 255);
  printFrame((0x98UL<<24)|(0xB4<<16)|(0x28<<8)|0xF4, buf, 8);
}

void sendFFE5Frame(float chgReq) {
  uint8_t buf[8] = {0};
  packLE16(buf, 0, (uint16_t)(soc * 10.0f));
  packLE16(buf, 2, (uint16_t)(max(0.0f, chgReq) * 10.0f));
  printFrame((0x98UL<<24)|(0xFF<<16)|(0xE5<<8)|0xF4, buf, 8);
}

void sendFF28Frame(float dischLimA) {
  uint8_t buf[8] = {0};
  packLE16(buf, 0, (uint16_t)(packV * 100.0f));
  packLE16(buf, 2, (uint16_t)(dischLimA * 100.0f));
  packLE16(buf, 4, (uint16_t)(soc * 10.0f));
  printFrame((0x98UL<<24)|(0xFF<<16)|(0x28<<8)|0xF4, buf, 8);
}

void sendFE28Frame(float dischLimA) {
  uint8_t buf[8] = {0};
  int maxMv = cellMv[0], minMv = cellMv[0];
  for (int i = 1; i < 19; i++) {
    if (cellMv[i] > maxMv) maxMv = cellMv[i];
    if (cellMv[i] < minMv) minMv = cellMv[i];
  }
  packLE16(buf, 0, (uint16_t)maxMv);
  packLE16(buf, 2, (uint16_t)minMv);
  buf[4] = (uint8_t)constrain(tempC[0] + 40.0f + 0.5f, 0, 255);
  buf[5] = (uint8_t)constrain(tempC[1] + 40.0f + 0.5f, 0, 255);
  packLE16(buf, 6, (uint16_t)(dischLimA * 10.0f));
  printFrame((0x98UL<<24)|(0xFE<<16)|(0x28<<8)|0xF4, buf, 8);
}

// Sensor JSON — parsed by server_frame_parser.py:parse_sensor_json()
void sendSensorJSON() {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "SENSOR_JSON:{"
    "\"v12_dc_out\":%.2f,"
    "\"v12_handbrake\":%.2f,"
    "\"v72_dc_in\":%.2f,"
    "\"v72_mppt_in\":%.2f,"
    "\"current_in\":%.2f,"
    "\"current_out\":%.2f,"
    "\"temp_mppt\":%.1f,"
    "\"temp_dcdc\":%.1f,"
    "\"temp_motor\":%.1f,"
    "\"handbrake\":%d"
    "}",
    s_v12_dc_out, s_v12_handbrake,
    s_v72_dc_in,  s_v72_mppt_in,
    s_current_in, s_current_out,
    s_temp_mppt,  s_temp_dcdc, s_temp_motor,
    s_handbrake
  );
  emitLine(String(buf));
}

// ─────────────────────────────────────────────────────────────────────────────
// Health LED — driven by max of all simulated temperatures
// ─────────────────────────────────────────────────────────────────────────────
void updateHealthLed() {
  float maxT = max(s_temp_motor, max(s_temp_mppt, max(s_temp_dcdc,
               max(tempC[0], max(tempC[1], tempC[2])))));
  if      (maxT >= TEMP_CRITICAL) setPattern(led2, BLINK_TRIPLE);
  else if (maxT >= TEMP_WARN)     setPattern(led2, BLINK_DOUBLE);
  else                            setPattern(led2, BLINK_SLOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase transition
// ─────────────────────────────────────────────────────────────────────────────
void nextPhase() {
  currentPhase = (Phase)((currentPhase + 1) % PHASE_COUNT);
  if (currentPhase == PHASE_REST && phaseTick > 0)
    currentPhase = PHASE_BULK_DISCHARGE;
  phaseTick = 0;
  emitLine(String("[INFO] ── Phase → ") + currentPhase + " ──");
}

// ─────────────────────────────────────────────────────────────────────────────
// Main simulation tick — every 500 ms
// ─────────────────────────────────────────────────────────────────────────────
void updateSimulation() {

  float dischLimA = 50.0f;
  float chgReqA   = 0.0f;
  float loadSagMv = 0.0f;
  float progress  = (PHASE_TICKS[currentPhase] > 0)
                    ? (float)phaseTick / PHASE_TICKS[currentPhase] : 1.0f;

  switch (currentPhase) {

    case PHASE_REST:
      currentA  = 0.0f; chgReqA = 5.0f; dischLimA = 50.0f;
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.04f + randNoise(0.05f);
      s_handbrake      = 1;
      s_v12_dc_out    += (12.8f - s_v12_dc_out)   * 0.05f + randNoise(0.02f);
      s_v12_handbrake += (12.6f - s_v12_handbrake) * 0.05f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.08f + randNoise(0.05f);
      s_v72_mppt_in   += (48.0f - s_v72_mppt_in)  * 0.05f + randNoise(0.3f);
      s_current_in    += (0.4f  - s_current_in)   * 0.08f + randNoise(0.05f);
      s_current_out    = s_current_in * 0.88f + randNoise(0.02f);
      s_temp_mppt     += (27.0f - s_temp_mppt)    * 0.04f + randNoise(0.1f);
      s_temp_dcdc     += (26.5f - s_temp_dcdc)    * 0.04f + randNoise(0.1f);
      s_temp_motor    += (25.0f - s_temp_motor)   * 0.04f + randNoise(0.1f);
      break;

    case PHASE_BULK_DISCHARGE:
      currentA = 38.0f + randNoise(1.5f); loadSagMv = 18.0f;
      soc     -= 0.30f + randNoise(0.02f);
      tempC[0] += 0.08f + randNoise(0.03f);
      tempC[1] += 0.06f + randNoise(0.02f);
      tempC[2] += 0.03f + randNoise(0.02f);
      dischLimA = 50.0f;
      s_handbrake      = 0;
      s_v12_dc_out    += (12.4f - s_v12_dc_out)   * 0.06f + randNoise(0.04f);
      s_v12_handbrake  = 0.0f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.10f + randNoise(0.1f);
      s_v72_mppt_in   += (58.0f - s_v72_mppt_in)  * 0.04f + randNoise(0.4f);
      s_current_in    += (4.0f  - s_current_in)   * 0.07f + randNoise(0.1f);
      s_current_out    = s_current_in * 0.90f + randNoise(0.05f);
      s_temp_motor    += 0.20f + randNoise(0.05f);
      s_temp_mppt     += 0.10f + randNoise(0.04f);
      s_temp_dcdc     += 0.07f + randNoise(0.03f);
      break;

    case PHASE_LOW_SOC:
      currentA  = 38.0f - progress * 26.0f + randNoise(0.8f);
      loadSagMv = 10.0f * (1.0f - progress);
      chgReqA   = 25.0f;
      soc      -= 0.22f + randNoise(0.02f);
      dischLimA = 50.0f - progress * 35.0f;
      tempC[0] += 0.02f + randNoise(0.03f);
      tempC[1] += 0.01f + randNoise(0.02f);
      tempC[2] += randNoise(0.02f);
      s_handbrake      = 0;
      s_v12_dc_out    += (12.1f - s_v12_dc_out)   * 0.06f + randNoise(0.05f);
      s_v12_handbrake  = 0.0f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.10f + randNoise(0.1f);
      s_v72_mppt_in   += (55.0f - s_v72_mppt_in)  * 0.04f + randNoise(0.3f);
      s_current_in    += (3.0f  - s_current_in)   * 0.05f + randNoise(0.08f);
      s_current_out    = s_current_in * 0.88f + randNoise(0.04f);
      s_temp_motor    += 0.06f * (1.0f - progress) + randNoise(0.05f);
      s_temp_mppt     += 0.03f + randNoise(0.04f);
      s_temp_dcdc     += 0.02f + randNoise(0.03f);
      break;

    case PHASE_RECOVERY:
      currentA  = 0.0f; chgReqA = 25.0f; dischLimA = 10.0f;
      soc      += 0.08f * (1.0f - progress);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.06f + randNoise(0.04f);
      s_handbrake      = 1;
      s_v12_dc_out    += (12.7f - s_v12_dc_out)   * 0.06f + randNoise(0.03f);
      s_v12_handbrake += (12.5f - s_v12_handbrake) * 0.06f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.08f + randNoise(0.08f);
      s_v72_mppt_in   += (50.0f - s_v72_mppt_in)  * 0.05f + randNoise(0.3f);
      s_current_in    += (1.0f  - s_current_in)   * 0.08f + randNoise(0.05f);
      s_current_out    = s_current_in * 0.87f + randNoise(0.03f);
      s_temp_motor    += (28.0f - s_temp_motor)   * 0.06f + randNoise(0.1f);
      s_temp_mppt     += (27.0f - s_temp_mppt)    * 0.05f + randNoise(0.1f);
      s_temp_dcdc     += (26.5f - s_temp_dcdc)    * 0.05f + randNoise(0.1f);
      break;

    case PHASE_CC_CHARGE:
      currentA  = -22.0f + randNoise(0.5f); chgReqA = 22.0f; dischLimA = 50.0f;
      soc      += 0.28f + randNoise(0.02f);
      { float h = (progress < 0.3f) ? (1.0f - progress/0.3f * 0.5f) : 0.5f;
        tempC[0] += 0.04f * h + randNoise(0.03f);
        tempC[1] += 0.03f * h + randNoise(0.02f);
        tempC[2] += 0.01f * h + randNoise(0.02f); }
      s_handbrake      = 1;
      s_v12_dc_out    += (13.6f - s_v12_dc_out)   * 0.05f + randNoise(0.04f);
      s_v12_handbrake += (13.4f - s_v12_handbrake) * 0.05f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.08f + randNoise(0.08f);
      s_v72_mppt_in   += (60.0f - s_v72_mppt_in)  * 0.04f + randNoise(0.4f);
      s_current_in    += (8.5f  - s_current_in)   * 0.05f + randNoise(0.1f);
      s_current_out    = s_current_in * 0.91f + randNoise(0.05f);
      s_temp_mppt     += (38.0f - s_temp_mppt)    * 0.04f + randNoise(0.1f);
      s_temp_dcdc     += (36.0f - s_temp_dcdc)    * 0.04f + randNoise(0.1f);
      s_temp_motor    += (26.0f - s_temp_motor)   * 0.05f + randNoise(0.08f);
      break;

    case PHASE_CV_TAPER:
      currentA  = -(22.0f*(1.0f-progress)*(1.0f-progress)+2.0f) + randNoise(0.3f);
      chgReqA   = max(0.0f, 22.0f * (1.0f - progress));
      dischLimA = 50.0f;
      soc      += 0.10f * (1.0f - progress * 0.7f) + randNoise(0.01f);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.05f + randNoise(0.03f);
      s_handbrake      = 1;
      s_v12_dc_out    += (13.8f - s_v12_dc_out)   * 0.04f + randNoise(0.03f);
      s_v12_handbrake += (13.6f - s_v12_handbrake) * 0.04f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.08f + randNoise(0.06f);
      s_v72_mppt_in   += (58.0f - s_v72_mppt_in)  * 0.04f + randNoise(0.3f);
      { float taperI = 8.5f * (1.0f - progress * 0.85f);
        s_current_in  += (taperI - s_current_in)  * 0.06f + randNoise(0.06f); }
      s_current_out    = s_current_in * 0.90f + randNoise(0.03f);
      s_temp_mppt     += (27.0f - s_temp_mppt)    * 0.05f + randNoise(0.1f);
      s_temp_dcdc     += (26.5f - s_temp_dcdc)    * 0.05f + randNoise(0.1f);
      s_temp_motor    += (25.5f - s_temp_motor)   * 0.05f + randNoise(0.08f);
      break;

    case PHASE_FULL_REST:
      currentA  = 0.0f; chgReqA = 0.0f; dischLimA = 50.0f;
      soc       = constrain(soc, 99.5f, 100.0f);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.07f + randNoise(0.03f);
      s_handbrake      = 1;
      s_v12_dc_out    += (13.8f - s_v12_dc_out)   * 0.04f + randNoise(0.02f);
      s_v12_handbrake += (13.6f - s_v12_handbrake) * 0.04f;
      s_v72_dc_in     += (packV - s_v72_dc_in)    * 0.08f + randNoise(0.05f);
      s_v72_mppt_in   += (42.0f - s_v72_mppt_in)  * 0.05f + randNoise(0.3f);
      s_current_in    += (0.2f  - s_current_in)   * 0.08f + randNoise(0.03f);
      s_current_out    = s_current_in * 0.88f + randNoise(0.02f);
      s_temp_mppt     += (25.5f - s_temp_mppt)    * 0.06f + randNoise(0.08f);
      s_temp_dcdc     += (25.0f - s_temp_dcdc)    * 0.06f + randNoise(0.08f);
      s_temp_motor    += (24.5f - s_temp_motor)   * 0.06f + randNoise(0.08f);
      break;

    default: break;
  }

  // ── Clamp ─────────────────────────────────────────────────────────────────
  soc = constrain(soc, 5.0f, 100.0f);
  for (int i = 0; i < 3; i++) tempC[i] = constrain(tempC[i], 15.0f, 58.0f);
  s_v12_dc_out    = constrain(s_v12_dc_out,    10.0f, 15.0f);
  s_v12_handbrake = constrain(s_v12_handbrake,  0.0f, 15.0f);
  s_v72_dc_in     = constrain(s_v72_dc_in,     40.0f, 80.0f);
  s_v72_mppt_in   = constrain(s_v72_mppt_in,   30.0f, 75.0f);
  s_current_in    = constrain(s_current_in,     0.0f, 22.0f);
  s_current_out   = constrain(s_current_out,    0.0f, 22.0f);
  s_temp_mppt     = constrain(s_temp_mppt,     20.0f, 80.0f);
  s_temp_dcdc     = constrain(s_temp_dcdc,     20.0f, 80.0f);
  s_temp_motor    = constrain(s_temp_motor,    20.0f, 85.0f);

  // ── Cell voltages ─────────────────────────────────────────────────────────
  float baseOCV = socToCellOCV(soc) * 1000.0f;
  for (int i = 0; i < 19; i++) {
    float offset  = (float)CELL_OFFSET[i];
    float weakSag = ((i == 6 || i == 13) && currentA > 0) ? loadSagMv * 0.7f : 0.0f;
    float loadSag = (currentA > 0) ? (currentA * 0.25f + loadSagMv) : 0.0f;
    float mv      = baseOCV + offset - loadSag - weakSag + randNoise(2.5f);
    cellMv[i]     = (int)constrain(mv, 2480.0f, 3720.0f);
  }
  float sumMv = 0;
  for (int i = 0; i < 19; i++) sumMv += cellMv[i];
  packV = (sumMv / 1000.0f) + randNoise(0.03f);

  // ── Phase advance ─────────────────────────────────────────────────────────
  phaseTick++;
  if (phaseTick >= PHASE_TICKS[currentPhase]) nextPhase();

  // ── Emit all frames + sensor JSON ────────────────────────────────────────
  sendCellFrames();
  sendCDFrame();
  sendTempFrame();
  sendFFE5Frame(chgReqA);
  sendFF28Frame(dischLimA);
  sendFE28Frame(dischLimA);
  sendSensorJSON();

  updateHealthLed();
}

// ─────────────────────────────────────────────────────────────────────────────
// Timing
// ─────────────────────────────────────────────────────────────────────────────
unsigned long lastSlowMs = 0;
unsigned long lastFastMs = 0;
float _fastDischLim = 50.0f;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  setPattern(led1, BLINK_FAST);   // WiFi connecting
  setPattern(led2, BLINK_SLOW);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    updateLed(led1);
    updateLed(led2);
    delay(10);
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[WiFi] Connected — IP %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("\n[WiFi] Not connected — Serial-only mode");

  randomSeed(analogRead(0));
  delay(500);
  Serial.println("============================================================");
  Serial.println("  O'CELL BMS + SENSOR SIMULATOR — 7-PHASE + WiFi TCP");
  Serial.printf ("  Server: %s:%d\n", SERVER_IP, SERVER_PORT);
  Serial.println("============================================================");
  emitLine("[INFO] Phase 0: REST — pack idle at 72% SOC, handbrake ON");
}

void loop() {
  unsigned long now = millis();

  updateLed(led1);
  updateLed(led2);
  maintainTCP();

  // Slow cycle (500 ms) — full simulation update + all frames
  if (now - lastSlowMs >= 500) {
    lastSlowMs = now;
    updateSimulation();
  }

  // Fast cycle (100 ms) — pack summary only, no sensor re-send
  if (now - lastFastMs >= 100) {
    lastFastMs = now;
    if (now - lastSlowMs > 10) {
      sendFF28Frame(_fastDischLim);
      sendFE28Frame(_fastDischLim);
    }
  }
}
