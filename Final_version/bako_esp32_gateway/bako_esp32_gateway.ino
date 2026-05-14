#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <mcp_can.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// ── WiFi config ─────────────────────────────────────────────
#define WIFI_SSID   "Ooredoo003CF8"
#define WIFI_PASS   "DU7T979#Z@@G2"
#define SERVER_IP   "192.168.1.14"
#define SERVER_PORT 9000

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

OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MCP_CAN           can(MCP_CS);
WiFiClient        tcp;
unsigned long     tcpRetry = 0;

// ── Helpers ─────────────────────────────────────────────────
float adcToVoltage(int pin) { return (analogRead(pin) / ADC_RES) * ADC_REF; }
float read12V(int pin)      { return adcToVoltage(pin) / DIV_12V_RATIO; }
float read72V(int pin)      { return adcToVoltage(pin) / DIV_72V_RATIO; }
float readCurrent(int pin) {
  float vOut = adcToVoltage(pin);
  return (vOut - ACS712_VREF) / (ACS712_MV_PER_A / 1000.0);
}

void maintainTCP() {
  if (WiFi.status() != WL_CONNECTED) { tcp.stop(); return; }
  if (tcp.connected()) return;
  if (millis() - tcpRetry < 3000) return;
  tcpRetry = millis();
  if (tcp.connect(SERVER_IP, SERVER_PORT)) {
    Serial.println("[TCP] Connected");
    digitalWrite(PIN_LED2, HIGH);
  } else {
    digitalWrite(PIN_LED2, LOW);
  }
}

void emitLine(const String& line) {
  Serial.println(line);
  if (tcp.connected()) tcp.println(line);
}

void receiveCAN() {
  if (digitalRead(MCP_INT) == HIGH) return;
  uint32_t id; uint8_t ext, dlc, buf[8];
  while (can.readMsgBuf(&id, &ext, &dlc, buf) == CAN_OK) {
    if (!ext) continue;
    char line[120];
    int n = snprintf(line, sizeof(line),
                     "[%lums] ID: 0x%08lX DLC: %u Data:",
                     millis(), (unsigned long)id, dlc);
    for (int i = 0; i < dlc; i++)
      n += snprintf(line + n, sizeof(line) - n, " %02X", buf[i]);
    emitLine(String(line));
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_LED1,      OUTPUT);
  pinMode(PIN_LED2,      OUTPUT);
  pinMode(MCP_INT,       INPUT);

  while (can.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("[CAN] Init failed, retrying...");
    delay(1000);
  }
  can.setMode(MCP_LISTENONLY);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500);
  }

  // DS18B20 LAST — after all other init
  sensors.begin();
  delay(1000);
  Serial.printf("[DS18B20] %d sensor(s)\n", sensors.getDeviceCount());

  Serial.println("=== BAKO Gateway Ready ===");
}
void loop() {
  maintainTCP();
  receiveCAN();

  sensors.requestTemperatures();
  delay(750);
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

  StaticJsonDocument<256> doc;
  doc["v12_dc_out"]    = round(v12_dc_out    * 100) / 100.0;
  doc["v12_handbrake"] = round(v12_handbrake * 100) / 100.0;
  doc["v72_dc_in"]     = round(v72_dc_in     * 100) / 100.0;
  doc["v72_mppt_in"]   = round(v72_mppt_in   * 100) / 100.0;
  doc["current_in"]    = round(current_in    * 100) / 100.0;
  doc["current_out"]   = round(current_out   * 100) / 100.0;
  doc["temp_mppt"]     = round(temp_mppt     * 10)  / 10.0;
  doc["temp_dcdc"]     = round(temp_dcdc     * 10)  / 10.0;
  doc["temp_motor"]    = round(temp_motor    * 10)  / 10.0;


  String json;
  serializeJson(doc, json);
  emitLine("SENSOR_JSON:" + json);
}