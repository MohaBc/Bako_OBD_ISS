#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ── WiFi / TCP config ────────────────────────────────────────
#define WIFI_SSID   "EVENT_SMU"
#define WIFI_PASS   "$mUeV&nt2@25"
#define SERVER_IP   "192.168.56.60"
#define SERVER_PORT 9000

// ── GPRS / SIM800L config ────────────────────────────────────
#define SIM800_TX        16
#define SIM800_RX        17
#define SIM800_BAUD      9600
#define GPRS_APN         "internet.ooredoo.tn"
#define VPS_URL          "http://webhook.site/9485e215-17c5-4974-9efa-553971099a18"
#define VPS_API_KEY      "bako-bms-2024"
#define DEVICE_ID        "esp32-bms-001"
#define VEHICLE_ID       1
#define GPRS_INTERVAL_MS 300000UL    // 5-minute GPRS heartbeat

// ── Pins ────────────────────────────────────────────────────
#define PIN_12V_DC_out      36
#define PIN_12V_Handbrake   34
#define PIN_72V_DC_in       39
#define PIN_72V_MPPT_in     35
#define ONE_WIRE_BUS        15
#define PIN_CURRENT_in      33
#define PIN_CURRENT_2_out   32
#define PIN_LED1            25
#define PIN_LED2            26
#define MCP_CS               5
#define MCP_INT              4

// ── Dividers ────────────────────────────────────────────────
#define DIV_12V_RATIO   (2650.0 / (10000.0 + 2650.0))
#define DIV_72V_RATIO   (1200.0 / (47000.0 + 1200.0))
#define ADC_REF         3.3
#define ADC_RES         4095.0
#define ACS712_VREF     1.65
#define ACS712_MV_PER_A 100.0

// ── Temperature thresholds (°C) ─────────────────────────────
#define TEMP_WARN     70.0
#define TEMP_CRITICAL 85.0

// ── LED blink patterns ──────────────────────────────────────
// LED1 = connectivity  |  LED2 = system health
//
//  LED_OFF        — solid off
//  LED_ON         — solid on
//  BLINK_FAST     — 100 ms toggle  (WiFi searching / CAN error)
//  BLINK_SLOW     — 500 ms toggle  (WiFi OK, no TCP / system normal)
//  BLINK_DOUBLE   — ·· pause       (temperature warning  >70 °C)
//  BLINK_TRIPLE   — ··· pause      (temperature critical >85 °C)

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
    case LED_OFF:   digitalWrite(led.pin, LOW);  return;
    case LED_ON:    digitalWrite(led.pin, HIGH); return;
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

// ── Globals — WiFi / sensors ─────────────────────────────────
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MCP_CAN           can(MCP_CS);
WiFiClient        tcp;
unsigned long     tcpRetry      = 0;
bool              canReady      = false;
unsigned long     sensorReqAt   = 0;
bool              sensorPending = false;

#define SENSOR_INTERVAL_MS 2000
#define DS18B20_CONV_MS     750

// ── Globals — GPRS ───────────────────────────────────────────
HardwareSerial sim800(1);
unsigned long  gprsLastSent = 0;

// ── Globals — sensor snapshot (updated every 2 s, read by GPRS) ─
float   g_v12_dc_out  = 0;
float   g_v12_hbrake  = 0;
float   g_v72_dc_in   = 0;
float   g_v72_mppt_in = 0;
float   g_cur_in      = 0;
float   g_cur_out     = 0;
float   g_temp_mppt   = -127;
float   g_temp_dcdc   = -127;
float   g_temp_motor  = -127;
uint8_t g_handbrake   = 0;

// ── Globals — BMS decoded state (updated from CAN) ───────────
// Negative sentinel = not yet received
float   bms_soc        = -1;
float   bms_pack_v     = -1;
float   bms_cell_min   = -1;   // mV
float   bms_cell_max   = -1;   // mV
float   bms_cell_avg   = -1;   // mV
int     bms_cell_count = 0;
float   bms_avg_temp   = -1;
float   bms_soh        = -1;
float   bms_remaining  = -1;
uint8_t bms_fault      = 0;
uint8_t bms_error      = 0;
uint16_t bmsCell[20]      = {};   // mV per cell, index 1-19 (0 = unused)
float    bmsTemp[9]       = {};   // °C per probe, index 1-8
int      bmsTempCount     = 0;
float    bms_pack_current = 0;    // A (negative = charging)
float    bms_soc_bms      = -1;   // % raw from BMS byte[1]
float    bms_max_disch_a  = -1;   // A, max discharge limit from FE28
float    bms_chg_req_a    = -1;   // A, charge request from FFE5
float    bms_temp_min     = -999; // °C min BMS probe
float    bms_temp_max     = -999; // °C max BMS probe
uint32_t gprs_seq         = 0;    // POST sequence counter

// SOC calibration  (mirrors server_frame_parser.py constants)
#define SOC_TOP_MV  3387.0f
#define SOC_BOT_MV  2500.0f
#define CAP_ACT_AH  44.7f
#define CAP_RAT_AH  50.0f


// ── ADC helpers ──────────────────────────────────────────────
float adcToVoltage(int pin) { return (analogRead(pin) / ADC_RES) * ADC_REF; }
float read12V(int pin)      { return adcToVoltage(pin) / DIV_12V_RATIO; }
float read72V(int pin)      { return adcToVoltage(pin) / DIV_72V_RATIO; }
float readCurrent(int pin) {
  float vOut = adcToVoltage(pin);
  return (vOut - ACS712_VREF) / (ACS712_MV_PER_A / 1000.0);
}

// ── Connectivity (LED1) ──────────────────────────────────────
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
    Serial.println("[TCP] Connected");
}

void emitLine(const String& line) {
  Serial.println(line);
  if (tcp.connected()) tcp.println(line);
}

// ── BMS CAN decoder (mirrors server_frame_parser.py parse_can) ──
// Called for every received CAN frame before forwarding it raw.
void decodeCAN(unsigned long id, uint8_t* d, uint8_t len) {
  uint8_t pf = (id >> 16) & 0xFF;

  // Cell voltages — 0x18C8..CC28F4  (big-endian uint16 pairs)
  if (pf >= 0xC8 && pf <= 0xCC && (id & 0xFF) == 0xF4) {
    int g = pf - 0xC8;
    for (int i = 0; i < 4; i++) {
      int idx = g * 4 + i + 1;
      int off = i * 2;
      if (idx <= 19 && off + 1 < len) {
        uint16_t mv = ((uint16_t)d[off] << 8) | d[off + 1];
        if (mv > 0) bmsCell[idx] = mv;
      }
    }
    uint32_t sum = 0;
    int      cnt = 0;
    uint16_t mn  = 65535, mx = 0;
    for (int i = 1; i <= 19; i++) {
      if (bmsCell[i] > 0) {
        sum += bmsCell[i]; cnt++;
        if (bmsCell[i] < mn) mn = bmsCell[i];
        if (bmsCell[i] > mx) mx = bmsCell[i];
      }
    }
    if (cnt > 0) {
      bms_cell_count = cnt;
      bms_cell_min   = mn;
      bms_cell_max   = mx;
      bms_cell_avg   = (float)sum / cnt;
      float soc = (bms_cell_avg - SOC_BOT_MV) / (SOC_TOP_MV - SOC_BOT_MV) * 100.0f;
      bms_soc = max(0.0f, min(100.0f, soc));
    }
    return;
  }

  // Temperature probes — 0x18B428F4  (uint8, offset -40 °C, 0xFF = NC)
  if (id == 0x18B428F4) {
    float tsum = 0; float tmin = 999; float tmax = -999; bmsTempCount = 0;
    for (int i = 0; i < min((int)len, 8); i++) {
      if (d[i] != 0xFF) {
        bmsTemp[i + 1] = d[i] - 40.0f;
        tsum += bmsTemp[i + 1];
        if (bmsTemp[i + 1] < tmin) tmin = bmsTemp[i + 1];
        if (bmsTemp[i + 1] > tmax) tmax = bmsTemp[i + 1];
        bmsTempCount++;
      }
    }
    if (bmsTempCount > 0) {
      bms_avg_temp = tsum / bmsTempCount;
      bms_temp_min = tmin;
      bms_temp_max = tmax;
    }
    return;
  }

  // BMS Basic Msg 1 — 0x18FF28F4
  if (id == 0x18FF28F4 && len >= 8) {
    if (d[1] <= 100) {
      bms_soc_bms = d[1];
      if (bms_soc < 0) bms_soc = d[1];   // fallback if no cell frames yet
    }
    uint16_t cur_raw = (uint16_t)d[2] | ((uint16_t)d[3] << 8);  // LE, offset 5000
    bms_pack_current = (cur_raw - 5000) * 0.1f;
    uint16_t v_raw = (uint16_t)d[4] | ((uint16_t)d[5] << 8);    // LE
    if (v_raw > 0) bms_pack_v = v_raw * 0.1f;
    bms_fault = d[6];
    bms_error = d[7];
    if (bms_soc >= 0) {
      bms_remaining = CAP_ACT_AH * bms_soc / 100.0f;
      bms_soh       = CAP_ACT_AH / CAP_RAT_AH * 100.0f;
    }
    return;
  }

  // BMS Basic Msg 2 — 0x18FE28F4  (fallback cell min/max + discharge limit)
  if (id == 0x18FE28F4 && len >= 8) {
    uint16_t mx = (uint16_t)d[0] | ((uint16_t)d[1] << 8);   // LE
    uint16_t mn = (uint16_t)d[2] | ((uint16_t)d[3] << 8);
    if (mx > 0 && bms_cell_max < 0) bms_cell_max = mx;
    if (mn > 0 && bms_cell_min < 0) bms_cell_min = mn;
    uint16_t dl = (uint16_t)d[6] | ((uint16_t)d[7] << 8);   // LE, A * 10
    if (dl > 0) bms_max_disch_a = dl * 0.1f;
    return;
  }

  // Charge request — 0x18FFE5F4
  if (id == 0x18FFE5F4 && len >= 4) {
    uint16_t chg_raw = (uint16_t)d[2] | ((uint16_t)d[3] << 8);  // LE, A * 10
    bms_chg_req_a = chg_raw * 0.1f;
    return;
  }
}

// ── CAN receive — forwards raw + decodes BMS state ───────────
void receiveCAN() {
  if (!canReady || digitalRead(MCP_INT) == HIGH) return;
  unsigned long rxId; uint8_t len; uint8_t rxBuf[8];
  while (can.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK) {
    // Decode BMS data before forwarding
    if ((rxId & 0x80000000) == 0x80000000)
      decodeCAN(rxId & 0x1FFFFFFF, rxBuf, len);
    else
      decodeCAN(rxId, rxBuf, len);

    // Forward raw frame via Serial + TCP (server parser handles it too)
    char line[128];
    int n;
    if ((rxId & 0x80000000) == 0x80000000)
      n = snprintf(line, sizeof(line),
                   "Extended ID: 0x%.8lX  DLC: %1d  Data:",
                   (rxId & 0x1FFFFFFF), len);
    else
      n = snprintf(line, sizeof(line),
                   "Standard ID: 0x%.3lX       DLC: %1d  Data:",
                   rxId, len);
    if ((rxId & 0x40000000) == 0x40000000) {
      strncat(line, " REMOTE REQUEST FRAME", sizeof(line) - n - 1);
    } else {
      for (byte i = 0; i < len; i++)
        n += snprintf(line + n, sizeof(line) - n, " 0x%.2X", rxBuf[i]);
    }
    emitLine(String(line));
  }
}

// ── Health LED (LED2) ────────────────────────────────────────
void updateHealthLed(float t0, float t1, float t2) {
  if (!canReady) { setPattern(led2, BLINK_FAST); return; }
  float maxTemp = max(t0, max(t1, t2));
  if      (maxTemp >= TEMP_CRITICAL) setPattern(led2, BLINK_TRIPLE);
  else if (maxTemp >= TEMP_WARN)     setPattern(led2, BLINK_DOUBLE);
  else                               setPattern(led2, BLINK_SLOW);
}

// ── GPRS helpers ─────────────────────────────────────────────
String gprsSendAT(const char* cmd, uint32_t timeout = 3000) {
  while (sim800.available()) sim800.read();   // flush stale URCs before sending
  sim800.println(cmd);
  String r = "";
  uint32_t t = millis();
  while (millis() - t < timeout) {
    while (sim800.available()) r += (char)sim800.read();
    yield();   // feed ESP32 WDT and allow WiFi stack to run
  }
  r.trim();
  Serial.printf("[GPRS] %s → %s\n", cmd, r.c_str());
  return r;
}

bool gprsEnsureBearer() {
  if (gprsSendAT("AT+SAPBR=2,1").indexOf("+SAPBR: 1,1") >= 0) return true;
  gprsSendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  gprsSendAT("AT+SAPBR=3,1,\"APN\",\"" GPRS_APN "\"");
  gprsSendAT("AT+SAPBR=1,1", 10000);
  return gprsSendAT("AT+SAPBR=2,1").indexOf("+SAPBR: 1,1") >= 0;
}

void gprsPost(const String& body) {
  if (!gprsEnsureBearer()) {
    Serial.println("[GPRS] Bearer failed — skipping POST");
    return;
  }
  gprsSendAT("AT+HTTPTERM", 1000);
  gprsSendAT("AT+HTTPINIT");
  gprsSendAT("AT+HTTPPARA=\"CID\",1");
  gprsSendAT("AT+HTTPPARA=\"URL\",\"" VPS_URL "\"");
  gprsSendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  gprsSendAT("AT+HTTPPARA=\"USERDATA\",\"X-Api-Key: " VPS_API_KEY "\"");
  gprsSendAT("AT+HTTPSSL=0");

  String dataCmd = "AT+HTTPDATA=" + String(body.length()) + ",10000";
  if (gprsSendAT(dataCmd.c_str(), 5000).indexOf("DOWNLOAD") < 0) {
    gprsSendAT("AT+HTTPTERM");
    return;
  }
  sim800.print(body);
  delay(2000);
  gprsSendAT("AT+HTTPACTION=1", 10000);
  delay(6000);
  String r = gprsSendAT("AT+HTTPREAD");
  gprsSendAT("AT+HTTPTERM");
  Serial.println("[GPRS] POST done: " + r.substring(0, 100));
}

// ── GPRS heartbeat — structured JSON ────────────────────────
void gprsLoop() {
  if (millis() - gprsLastSent < GPRS_INTERVAL_MS) return;
  gprsLastSent = millis();
  receiveCAN();   // drain FIFO before blocking
  gprs_seq++;

  JsonDocument doc;
  doc["schema_version"] = "2.0.0";
  doc["vehicle_id"]     = VEHICLE_ID;
  doc["seq"]            = gprs_seq;

  // ── battery ───────────────────────────────────────────────
  JsonObject bat = doc["battery"].to<JsonObject>();
  bat["pack_v"]        = (bms_pack_v    >= 0) ? round(bms_pack_v    * 1000) / 1000.0 : 0.0;
  bat["pack_current_a"]= round(bms_pack_current * 1000) / 1000.0;
  bat["soc"]           = (bms_soc       >= 0) ? round(bms_soc       * 100)  / 100.0  : 0.0;
  bat["soc_bms"]       = (bms_soc_bms   >= 0) ? round(bms_soc_bms   * 100)  / 100.0  : 0.0;
  bat["max_disch_a"]   = (bms_max_disch_a >= 0) ? round(bms_max_disch_a * 10) / 10.0 : 0.0;
  bat["cell_count"]    = bms_cell_count;
  bat["cell_avg_mv"]   = (bms_cell_avg  >= 0) ? round(bms_cell_avg  * 100)  / 100.0  : 0.0;
  bat["cell_min_mv"]   = (bms_cell_min  >= 0) ? (int)bms_cell_min : 0;
  bat["cell_max_mv"]   = (bms_cell_max  >= 0) ? (int)bms_cell_max : 0;
  bat["cell_spread_mv"]= (bms_cell_min >= 0 && bms_cell_max >= 0)
                           ? (int)(bms_cell_max - bms_cell_min) : 0;

  JsonArray cells = bat["cells_voltages"].to<JsonArray>();
  for (int i = 1; i <= 19; i++) cells.add((int)bmsCell[i]);

  JsonObject chgLim = bat["charge_limit"].to<JsonObject>();
  chgLim["max_charge_v"] = (bms_cell_count > 0)
                             ? round(bms_cell_count * 3.65f * 10) / 10.0 : 0.0;
  chgLim["max_charge_a"] = (bms_chg_req_a >= 0) ? round(bms_chg_req_a * 10) / 10.0 : 0.0;
  chgLim["enable"]       = (bms_chg_req_a > 0.0f);

  JsonObject status = bat["status"].to<JsonObject>();
  status["fault_level"] = bms_fault;
  status["error_code"]  = bms_error;
  status["ready"]       = (bms_fault == 0 && bms_error == 0 && bms_soc >= 0);
  status["charging"]    = (bms_pack_current < -0.5f);
  status["discharging"] = (bms_pack_current >  0.5f);

  // ── temperatures ─────────────────────────────────────────
  JsonObject temps = doc["temperatures"].to<JsonObject>();
  temps["battery_avg_c"] = (bms_avg_temp  > -999) ? round(bms_avg_temp  * 100) / 100.0 : 0.0;
  temps["battery_min_c"] = (bms_temp_min  > -999) ? round(bms_temp_min  * 100) / 100.0 : 0.0;
  temps["battery_max_c"] = (bms_temp_max  > -999) ? round(bms_temp_max  * 100) / 100.0 : 0.0;
  JsonArray btemps = temps["battery_temps_c"].to<JsonArray>();
  for (int i = 1; i <= bmsTempCount; i++) btemps.add(round(bmsTemp[i] * 100) / 100.0);
  temps["dcdc_temp_c"]  = round(g_temp_dcdc  * 100) / 100.0;
  temps["motor_temp_c"] = round(g_temp_motor * 100) / 100.0;
  temps["mppt_temp_c"]  = round(g_temp_mppt  * 100) / 100.0;

  // ── solar ────────────────────────────────────────────────
  JsonObject solar    = doc["solar"].to<JsonObject>();
  JsonObject preMppt  = solar["pre_mppt"].to<JsonObject>();
  preMppt["voltage_v"] = round(g_v72_mppt_in * 1000) / 1000.0;
  preMppt["current_a"] = round(g_cur_in      * 1000) / 1000.0;
  JsonObject postMppt = solar["post_mppt"].to<JsonObject>();
  postMppt["current_a"] = round(g_cur_out * 1000) / 1000.0;

  // ── dc_dc ────────────────────────────────────────────────
  JsonObject dcdc   = doc["dc_dc"].to<JsonObject>();
  JsonObject dcIn   = dcdc["input_64v"].to<JsonObject>();
  dcIn["voltage_v"]  = round(g_v72_dc_in  * 1000) / 1000.0;
  JsonObject dcOut  = dcdc["output_12v"].to<JsonObject>();
  dcOut["voltage_v"] = round(g_v12_dc_out * 1000) / 1000.0;

  // ── gnss — no GPS hardware, fields are null ───────────────
  JsonObject gnss = doc["gnss"].to<JsonObject>();
  gnss["lat"]       = nullptr;
  gnss["lon"]       = nullptr;
  gnss["alt_m"]     = nullptr;
  gnss["speed_kmh"] = nullptr;
  gnss["fix"]       = false;

  // ── vehicle ──────────────────────────────────────────────
  JsonObject vehicle = doc["vehicle"].to<JsonObject>();
  vehicle["handbrake"] = (bool)g_handbrake;

  String json;
  serializeJson(doc, json);
  Serial.println("[GPRS] POST seq=" + String(gprs_seq) + " len=" + String(json.length()));
  gprsPost(json);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(MCP_INT,  INPUT);

  setPattern(led1, BLINK_FAST);   // WiFi connecting

  if (can.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    can.setMode(MCP_NORMAL);
    canReady = true;
  } else {
    Serial.println("[CAN] Init failed");
    setPattern(led2, BLINK_FAST);
  }

  // SIM800L — start UART, give module 3 s to wake
  sim800.begin(SIM800_BAUD, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(3000);
  gprsSendAT("AT");             // probe
  gprsSendAT("ATE0");           // echo off
  Serial.println("[GPRS] SIM800L initialised");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    updateLed(led1);
    updateLed(led2);
    delay(10);
  }

  sensors.begin();
  delay(1000);
  Serial.printf("[DS18B20] %d sensor(s)\n", sensors.getDeviceCount());
  Serial.println("=== BAKO Gateway Ready ===");
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  updateLed(led1);
  updateLed(led2);
  maintainTCP();
  receiveCAN();   // non-blocking; also decodes BMS state

  // DS18B20 async conversion every SENSOR_INTERVAL_MS
  if (!sensorPending && now - sensorReqAt >= SENSOR_INTERVAL_MS) {
    sensors.requestTemperatures();
    sensorReqAt   = now;
    sensorPending = true;
  }

  // Read results once 750 ms conversion window has elapsed
  if (sensorPending && now - sensorReqAt >= DS18B20_CONV_MS) {
    sensorPending = false;

    float temp_mppt  = sensors.getTempCByIndex(0);
    float temp_dcdc  = sensors.getTempCByIndex(1);
    float temp_motor = sensors.getTempCByIndex(2);

    Serial.printf("Temp MPPT: %.2f  Temp DC/DC: %.2f  Temp Motor: %.2f\n",
                  temp_mppt, temp_dcdc, temp_motor);

    float v12_dc_out    = read12V(PIN_12V_DC_out);
    float v12_handbrake = read12V(PIN_12V_Handbrake);
    float v72_dc_in     = read72V(PIN_72V_DC_in);
    float v72_mppt_in   = read72V(PIN_72V_MPPT_in);
    float current_in    = readCurrent(PIN_CURRENT_in);
    float current_out   = readCurrent(PIN_CURRENT_2_out);
    uint8_t handbrake   = (v12_handbrake <= 5.0) ? 1 : 0;

    // Cache for GPRS snapshot
    g_v12_dc_out  = v12_dc_out;
    g_v12_hbrake  = v12_handbrake;
    g_v72_dc_in   = v72_dc_in;
    g_v72_mppt_in = v72_mppt_in;
    g_cur_in      = current_in;
    g_cur_out     = current_out;
    g_temp_mppt   = temp_mppt;
    g_temp_dcdc   = temp_dcdc;
    g_temp_motor  = temp_motor;
    g_handbrake   = handbrake;

    updateHealthLed(temp_mppt, temp_dcdc, temp_motor);

    // Emit sensor JSON over Serial + TCP (local server path)
    JsonDocument doc;
    doc["v12_dc_out"]    = round(v12_dc_out    * 100) / 100.0;
    doc["v12_handbrake"] = round(v12_handbrake * 100) / 100.0;
    doc["v72_dc_in"]     = round(v72_dc_in     * 100) / 100.0;
    doc["v72_mppt_in"]   = round(v72_mppt_in   * 100) / 100.0;
    doc["current_in"]    = round(current_in    * 100) / 100.0;
    doc["current_out"]   = round(current_out   * 100) / 100.0;
    doc["temp_mppt"]     = round(temp_mppt     * 10)  / 10.0;
    doc["temp_dcdc"]     = round(temp_dcdc     * 10)  / 10.0;
    doc["temp_motor"]    = round(temp_motor    * 10)  / 10.0;
    doc["handbrake"]     = handbrake;
    String json;
    serializeJson(doc, json);
    emitLine("SENSOR_JSON:" + json);
  }

  // GPRS 2-minute heartbeat (blocks ~20 s while posting)
  gprsLoop();
}
