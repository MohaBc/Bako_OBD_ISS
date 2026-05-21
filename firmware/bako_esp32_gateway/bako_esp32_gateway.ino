/**
 * ============================================================
 *  BAKO Motors — ESP32 Sensor Gateway Firmware
 *  Rev 2.0  — SA = 0xAA
 * ============================================================
 *
 *  Flow:
 *   1. Read physical sensors (ADC, ACS712, DS18B20, handbrake)
 *   2. Encode readings into BAKO CAN frame format (SA=0xAA)
 *   3. Print those frames to Serial + TCP  (NOT sent to MCP2515)
 *   4. Receive real BMS frames from MCP2515
 *   5. Print received frames to Serial + TCP
 *
 *  server_frame_parser.py sees a single stream of canonical lines:
 *    [Nms] ID: 0xHHHHHHHH DLC: N Data: HH HH ...
 *  and parses both SA=0xF4 (BMS) and SA=0xAA (ESP32 sensors).
 *
 *  Pin assignments (GPIOsESP32.txt / sensors_test_bako.ino):
 *   GPIO36  12V-A  ADC  divider x2.65k/(10k+2.65k)  [input-only]
 *   GPIO34  12V-B  ADC  same divider
 *   GPIO39  72V-A  ADC  divider x1.2k/(47k+1.2k)    [input-only]
 *   GPIO35  72V-B  ADC  same divider
 *   GPIO15  DS18B20 1-Wire (3 sensors)
 *             #0 -> motor winding temp   -> 0x18D401AA
 *             #1 -> MPPT heatsink temp   -> 0x18D501AA
 *             #2 -> DC/DC heatsink temp  -> 0x18D601AA
 *   GPIO33  ACS712 CH1  solar current (before MPPT)
 *   GPIO32  ACS712 CH2  MPPT output current
 *   GPIO27  Handbrake switch (INPUT_PULLUP, LOW=engaged)
 *   GPIO25  LED1  blinks on CAN RX
 *   GPIO26  LED2  solid when TCP connected
 *   MCP2515: INT=4  CS=5  SCK=18  MOSI=23  MISO=19
 *
 *  Libraries: mcp_can (coryjfowler), OneWire, DallasTemperature
 * ============================================================
 */

#include <SPI.h>
#include <mcp_can.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>

// ─────────────────────────────────────────────────────────────
//  WiFi / TCP config  ← EDIT THESE
// ─────────────────────────────────────────────────────────────
static const char*    WIFI_SSID   = "EVENT_SMU";
static const char*    WIFI_PASS   = "$mUeV&nt2@25";
static const char*    SERVER_IP   = "192.168.52.56";  // PC running server_frame_parser.py
static const uint16_t SERVER_PORT = 9000;

// ─────────────────────────────────────────────────────────────
//  Pins
// ─────────────────────────────────────────────────────────────
#define PIN_12V_A       36   // input-only — no pinMode()
#define PIN_12V_B       34
#define PIN_72V_A       39   // input-only — no pinMode()
#define PIN_72V_B       35
#define PIN_DS18B20     15
#define PIN_CURRENT_1   33
#define PIN_CURRENT_2   32
#define PIN_HANDBRAKE   27
#define PIN_LED1        25
#define PIN_LED2        26
#define MCP_CS           5
#define MCP_INT          4

// ─────────────────────────────────────────────────────────────
//  Dividers & ADC  (from sensors_test_bako.ino)
// ─────────────────────────────────────────────────────────────
#define DIV_12V   (2650.0f / (10000.0f + 2650.0f))
#define DIV_72V   (1200.0f / (47000.0f + 1200.0f))
#define ADC_REF   3.3f
#define ADC_RES   4095.0f

// ACS712: change to 185.0 (5A) or 100.0 (20A) if needed
#define ACS_VREF      1.65f
#define ACS_MV_PER_A  66.0f

// ─────────────────────────────────────────────────────────────
//  CAN frame IDs  (SA = 0xAA)
// ─────────────────────────────────────────────────────────────
#define ID_SOLAR_I   0x18D001AAUL
#define ID_SOLAR_V   0x18D101AAUL
#define ID_MPPT_OUT  0x18D201AAUL
#define ID_DCDC      0x18D301AAUL
#define ID_MOTOR_T   0x18D401AAUL
#define ID_MPPT_T    0x18D501AAUL
#define ID_DCDC_T    0x18D601AAUL
#define ID_HANDBRAKE 0x18D701AAUL

// ─────────────────────────────────────────────────────────────
//  TX intervals (ms)
// ─────────────────────────────────────────────────────────────
#define T_SOLAR_I    200
#define T_SOLAR_V    200
#define T_MPPT_OUT   200
#define T_DCDC       200
#define T_MOTOR_T   1000
#define T_MPPT_T    1000
#define T_DCDC_T    1000
#define T_HANDBRAKE  500
#define T_DS18B20    800

// ─────────────────────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────────────────────
MCP_CAN           can(MCP_CS);
OneWire           oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

WiFiClient        tcp;
static unsigned long tcpRetry = 0;

// 5-sample moving average for solar current
static float   solarBuf[5] = {0};
static uint8_t solarIdx    = 0;

// DS18B20: [0]=motor  [1]=MPPT heatsink  [2]=DC/DC heatsink
static float dsTemp[3] = {-127.0f, -127.0f, -127.0f};

// Handbrake debounce
static uint8_t       hbPrev  = 0xFF;
static uint8_t       hbDeb   = 0;
static unsigned long hbLast  = 0;

// Timestamps
static unsigned long tSolarI   = 0, tSolarV  = 0, tMpptOut = 0;
static unsigned long tDcdc     = 0, tMotorT  = 0, tMpptT   = 0;
static unsigned long tDcdcT    = 0, tHandbrake = 0, tDs18b20 = 0;

// ─────────────────────────────────────────────────────────────
//  Helpers
// ─────────────────────────────────────────────────────────────
static float adcV(int pin) {
    return (analogRead(pin) / ADC_RES) * ADC_REF;
}
static float read12V(int pin)  { return adcV(pin) / DIV_12V; }
static float read72V(int pin)  { return adcV(pin) / DIV_72V; }

static float readCurrent(int pin) {
    long sum = 0;
    for (int i = 0; i < 32; i++) { sum += analogRead(pin); delayMicroseconds(200); }
    float v = ((float)sum / 32.0f / ADC_RES) * ADC_REF;
    return (v - ACS_VREF) / (ACS_MV_PER_A / 1000.0f);
}

static void u16le(uint8_t* b, uint16_t v) { b[0] = v & 0xFF; b[1] = v >> 8; }

// Temperature: uint8 offset-40, 0xFF = not connected / error
static uint8_t tempRaw(float t) {
    if (isnan(t) || t <= -126.0f) return 0xFF;
    return (uint8_t)constrain((int)roundf(t + 40.0f), 0, 254);
}

// ─────────────────────────────────────────────────────────────
//  Output: print a line to Serial and TCP
// ─────────────────────────────────────────────────────────────
static void emit(const char* line) {
    Serial.println(line);
    if (tcp.connected()) tcp.println(line);
}

// ─────────────────────────────────────────────────────────────
//  Build canonical CAN log line and emit it (no bus TX)
// ─────────────────────────────────────────────────────────────
static void logFrame(uint32_t id, uint8_t* d, uint8_t dlc) {
    char buf[120];
    int  n = snprintf(buf, sizeof(buf),
                      "[%lums] ID: 0x%08lX DLC: %u Data:",
                      millis(), (unsigned long)id, dlc);
    for (int i = 0; i < dlc; i++)
        n += snprintf(buf + n, sizeof(buf) - n, " %02X", d[i]);
    emit(buf);
}

// ─────────────────────────────────────────────────────────────
//  Receive real frames from MCP2515 (BMS etc.) and forward
// ─────────────────────────────────────────────────────────────
static void receiveCAN() {
    if (digitalRead(MCP_INT) == HIGH) return;
    uint32_t id; uint8_t ext, dlc, buf[8];
    while (can.readMsgBuf(&id, &ext, &dlc, buf) == CAN_OK) {
        logFrame(id, buf, dlc);
        digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    }
}

// ─────────────────────────────────────────────────────────────
//  TCP: reconnect if dropped
// ─────────────────────────────────────────────────────────────
static void maintainTCP() {
    if (WiFi.status() != WL_CONNECTED) { tcp.stop(); digitalWrite(PIN_LED2, LOW); return; }
    if (tcp.connected()) return;
    if (millis() - tcpRetry < 3000) return;
    tcpRetry = millis();
    if (tcp.connect(SERVER_IP, SERVER_PORT)) {
        Serial.printf("[TCP] Connected to %s:%u\n", SERVER_IP, SERVER_PORT);
        digitalWrite(PIN_LED2, HIGH);
    } else {
        digitalWrite(PIN_LED2, LOW);
    }
}

// ─────────────────────────────────────────────────────────────
//  DS18B20 update
// ─────────────────────────────────────────────────────────────
static void updateTemps() {
    ds18b20.requestTemperatures();
    int n = ds18b20.getDeviceCount();
    for (int i = 0; i < 3; i++) {
        if (i < n) {
            float t = ds18b20.getTempCByIndex(i);
            if (t > -127.0f) dsTemp[i] = t;
        }
    }
}

// ─────────────────────────────────────────────────────────────
//  Sensor frame builders — read sensor, pack bytes, logFrame()
// ─────────────────────────────────────────────────────────────

// 0x18D001AA  Solar current before MPPT  200ms
// B0-1: raw current  x0.1A  | B2-3: 5-avg x0.1A | B4: status
static void frameSolarI() {
    float raw = readCurrent(PIN_CURRENT_1);
    if (raw < 0) raw = 0;
    solarBuf[solarIdx] = raw;
    solarIdx = (solarIdx + 1) % 5;
    float avg = 0; for (int i = 0; i < 5; i++) avg += solarBuf[i]; avg /= 5.0f;

    uint8_t b[8] = {0};
    u16le(b+0, (uint16_t)constrain((int)(raw*10), 0, 65534));
    u16le(b+2, (uint16_t)constrain((int)(avg*10), 0, 65534));
    b[4] = raw > 100.0f ? 0x01 : 0x00;
    logFrame(ID_SOLAR_I, b, 8);
}

// 0x18D101AA  Solar voltage before MPPT  200ms
// B0-1: panel V x0.1V | B2-3: Voc x0.1V | B4: status
static void frameSolarV() {
    float vp = read72V(PIN_72V_A);
    float vo = read72V(PIN_72V_B);
    uint8_t b[8] = {0};
    u16le(b+0, (uint16_t)constrain((int)(vp*10), 0, 65534));
    u16le(b+2, (uint16_t)constrain((int)(vo*10), 0, 65534));
    b[4] = vp > 150.0f ? 0x01 : 0x00;
    logFrame(ID_SOLAR_V, b, 8);
}

// 0x18D201AA  MPPT output current + mode + efficiency  200ms
// B0-1: out current x0.1A | B2: mode | B3: eff% | B4: status
static void frameMpptOut() {
    float out = readCurrent(PIN_CURRENT_2);
    if (out < 0) out = 0;
    uint8_t mode = out > 0.5f ? (out > 18.0f ? 2 : 1) : 0;
    uint8_t eff  = 0;
    float inI = solarBuf[(solarIdx + 4) % 5];
    float vIn = read72V(PIN_72V_A), vOut = read12V(PIN_12V_A);
    if (inI > 0.1f && vIn > 1.0f)
        eff = (uint8_t)constrain((int)(out*vOut / (inI*vIn) * 100), 0, 100);
    uint8_t b[8] = {0};
    u16le(b+0, (uint16_t)constrain((int)(out*10), 0, 65534));
    b[2] = mode; b[3] = eff;
    logFrame(ID_MPPT_OUT, b, 8);
}

// 0x18D301AA  DC/DC 12V voltage + current  200ms
// B0-1: V x0.01V | B2-3: I x0.1A (0xFFFF=not fitted) | B4: status
static void frameDcdc() {
    float v = read12V(PIN_12V_A);
    uint8_t b[8] = {0};
    u16le(b+0, (uint16_t)constrain((int)(v*100), 0, 65534));
    b[2] = 0xFF; b[3] = 0xFF;  // current not fitted
    b[4] = (v < 10.5f || v > 14.5f) ? 0x01 : 0x00;
    logFrame(ID_DCDC, b, 8);
}

// 0x18D401AA  Motor winding temp  1000ms
// B0: DS18B20#0 offset-40 | B1: status
static void frameMotorT() {
    uint8_t b[8] = {0};
    b[0] = tempRaw(dsTemp[0]);
    b[1] = dsTemp[0] > 80 ? 0x02 : dsTemp[0] > 60 ? 0x01 : 0x00;
    logFrame(ID_MOTOR_T, b, 8);
}

// 0x18D501AA  MPPT heatsink temp  1000ms
// B0: DS18B20#1 offset-40 | B1: status
static void frameMpptT() {
    uint8_t b[8] = {0};
    b[0] = tempRaw(dsTemp[1]);
    b[1] = dsTemp[1] > 70 ? 0x02 : dsTemp[1] > 50 ? 0x01 : 0x00;
    logFrame(ID_MPPT_T, b, 8);
}

// 0x18D601AA  DC/DC heatsink temp  1000ms
// B0: DS18B20#2 offset-40 | B1: status
static void frameDcdcT() {
    uint8_t b[8] = {0};
    b[0] = tempRaw(dsTemp[2]);
    b[1] = dsTemp[2] > 80 ? 0x02 : dsTemp[2] > 60 ? 0x01 : 0x00;
    logFrame(ID_DCDC_T, b, 8);
}

// 0x18D701AA  Handbrake  500ms
// B0: 0=released 1=engaged | B1: debounce state
static void frameHandbrake() {
    uint8_t cur = digitalRead(PIN_HANDBRAKE) == LOW ? 0x01 : 0x00;
    unsigned long now = millis();
    if (cur != hbPrev) { hbDeb = 1; hbLast = now; }
    if (hbDeb && now - hbLast > 50) hbDeb = 0;
    hbPrev = cur;
    uint8_t b[8] = {0};
    b[0] = cur; b[1] = hbDeb;
    logFrame(ID_HANDBRAKE, b, 8);
}

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== BAKO ESP32 Gateway v2.0 ===");

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    // GPIO36 & GPIO39 are input-only — no pinMode()
    pinMode(PIN_LED1,      OUTPUT);
    pinMode(PIN_LED2,      OUTPUT);
    pinMode(PIN_HANDBRAKE, INPUT_PULLUP);
    pinMode(MCP_INT,       INPUT);
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);

    ds18b20.begin();
    Serial.printf("[DS18B20] %d sensor(s)\n", ds18b20.getDeviceCount());

    // MCP2515 in LISTEN-ONLY mode — receives everything, transmits nothing
    Serial.print("[CAN] Init...");
    while (can.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) != CAN_OK) {
        Serial.print(".");
        delay(1000);
    }
    can.setMode(MCP_LISTENONLY);   // <-- listen only, no TX, no ACK errors
    Serial.println(" OK (listen-only, 250 kbps)");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
        delay(500); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
        digitalWrite(PIN_LED2, HIGH);
    }
    maintainTCP();

    Serial.println("[BOOT] Running.\n");
}

// ─────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();

    maintainTCP();

    // Receive real BMS frames → forward to Serial + TCP
    receiveCAN();

    // DS18B20 conversion
    if (now - tDs18b20 >= T_DS18B20) { tDs18b20 = now; updateTemps(); }

    // Build + log sensor frames (no bus TX)
    if (now - tSolarI   >= T_SOLAR_I)   { tSolarI   = now; frameSolarI();   }
    if (now - tSolarV   >= T_SOLAR_V)   { tSolarV   = now; frameSolarV();   }
    if (now - tMpptOut  >= T_MPPT_OUT)  { tMpptOut  = now; frameMpptOut();  }
    if (now - tDcdc     >= T_DCDC)      { tDcdc     = now; frameDcdc();     }
    if (now - tMotorT   >= T_MOTOR_T)   { tMotorT   = now; frameMotorT();   }
    if (now - tMpptT    >= T_MPPT_T)    { tMpptT    = now; frameMpptT();    }
    if (now - tDcdcT    >= T_DCDC_T)    { tDcdcT    = now; frameDcdcT();    }
    if (now - tHandbrake >= T_HANDBRAKE){ tHandbrake = now; frameHandbrake();}
}
