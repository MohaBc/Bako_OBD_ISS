/*
 * ESP32 BMS Firebase Publisher
 * ============================
 * Reads O'CELL IFS60.8-500 BMS CAN frames via MCP2515 (250 kbps),
 * decodes SAE J1939 data, and PUTs a live JSON snapshot to Firebase
 * Realtime Database every PUBLISH_INTERVAL ms via SIM800L GPRS.
 *
 * Uses the SIM800L AT+HTTP* service (no TinyGSM needed on ESP32) and
 * HardwareSerial UART1 for reliable high-speed AT communication.
 *
 * Wiring
 * ──────
 *   MCP2515  │  ESP32
 *   ─────────┼──────────────────────────────────
 *   VCC      │  3.3 V
 *   GND      │  GND
 *   CS       │  GPIO 15   (VSPI CS0)
 *   INT      │  GPIO  4
 *   SCK      │  GPIO 18   (VSPI SCK)
 *   MISO     │  GPIO 19   (VSPI MISO)
 *   MOSI     │  GPIO 23   (VSPI MOSI)
 *
 *   SIM800L  │  ESP32
 *   ─────────┼──────────────────────────────────
 *   VCC      │  4.0 V external supply (≥ 2 A)
 *   GND      │  GND (shared)
 *   TXD      │  GPIO 16   (ESP32 UART1 RX)
 *   RXD      │  GPIO 17   (ESP32 UART1 TX)
 *   RST      │  GPIO  5
 *
 * Config — edit secrets.h
 * ────────────────────────
 *   FIREBASE_HOST  → https://<project>-default-rtdb.firebaseio.com
 *   FIREBASE_AUTH  → database secret (Firebase console → Project settings →
 *                    Service accounts → Database secrets)
 *   APN            → your SIM carrier APN  (e.g. "internet", "iam", "maroctel")
 *
 * Firebase data paths
 * ───────────────────
 *   PUT  /bms/live.json          ← overwrites latest snapshot (dashboard)
 *   POST /bms/history.json       ← appends timestamped entry   (log)
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <mcp_can.h>
#include <SPI.h>
#include "secrets.h"

// ─── SIM800L — UART1 (HardwareSerial avoids SoftwareSerial timing issues) ────
HardwareSerial sim800l(1);
constexpr uint8_t SIM_RX  = 16;
constexpr uint8_t SIM_TX  = 17;
constexpr uint8_t SIM_RST =  5;

// ─── MCP2515 CAN ──────────────────────────────────────────────────────────────
constexpr uint8_t CAN_CS  = 15;
constexpr uint8_t CAN_INT =  4;

// ─── GPRS APN — override in secrets.h if needed ───────────────────────────────
#ifndef APN
  #define APN       "internet.ooredoo.tn"
#endif
#ifndef APN_USER
  #define APN_USER  ""
#endif
#ifndef APN_PASS
  #define APN_PASS  ""
#endif

// ─── Timing ───────────────────────────────────────────────────────────────────
constexpr unsigned long PUBLISH_INTERVAL = 5000;   // 5 s between publishes
constexpr int           MAX_RETRIES      = 3;

// ─── SOC calibration (mirrors server.py) ──────────────────────────────────────
static constexpr uint16_t CELL_UV        = 2500;   // mV → 0%
static constexpr uint16_t SOC_CAR_TOP_MV = 3387;   // mV → 100% (car-calibrated)

// ─── BMS state ────────────────────────────────────────────────────────────────
struct BMSData {
    uint16_t cell_mv[19] = {};
    uint8_t  cell_count  = 0;
    float    temp_c[4]   = {};
    uint8_t  temp_count  = 0;
    float    soc         = -1.0f;
    float    pack_v      = 0.0f;
    uint16_t cell_max    = 0;
    uint16_t cell_min    = 0xFFFF;
    uint16_t cell_avg    = 0;
    uint32_t frame_count = 0;
};

static BMSData       bms;
static unsigned long lastPublish = 0;

MCP_CAN CAN0(CAN_CS);
char    responseBuf[512];

// ─── Prototypes ───────────────────────────────────────────────────────────────
bool   connectGPRS();
bool   openBearer();
void   closeBearer();
bool   publishToFirebase();
bool   httpPut(const char* path, const char* body, size_t bodyLen);
bool   httpPost(const char* path, const char* body, size_t bodyLen);
void   sendAT(const char* cmd, int waitMs = 1000);
bool   sendATExpect(const char* cmd, const char* expected, int waitMs = 5000);
size_t readResponse(char* buf, size_t bufLen, unsigned long timeoutMs);
void   checkSignal();
void   getNetworkTime(char* buf, size_t bufLen);

// ─── Byte helpers ─────────────────────────────────────────────────────────────
static inline uint16_t u16be(const uint8_t* d, int o) {
    return ((uint16_t)d[o] << 8) | d[o + 1];
}
static inline uint16_t u16le(const uint8_t* d, int o) {
    return d[o] | ((uint16_t)d[o + 1] << 8);
}

static const char* cellStatus(uint16_t mv) {
    if (mv >= 3750) return "overvoltage";
    if (mv >= 3650) return "full";
    if (mv >= 3300) return "good";
    if (mv >= 3200) return "normal";
    if (mv >= 2500) return "low";
    return "undervoltage";
}

// ─── J1939 frame decoder ──────────────────────────────────────────────────────
void decodeFrame(uint32_t can_id, uint8_t len, uint8_t* data) {
    uint8_t func = (can_id >> 16) & 0xFF;
    uint8_t sub  = (can_id >>  8) & 0xFF;
    bms.frame_count++;

    // Cell voltages: func 0xC8–0xCC, big-endian uint16 mV, 4 cells/frame
    if (func >= 0xC8 && func <= 0xCC && len == 8) {
        uint8_t group = func - 0xC8;
        uint8_t base  = group * 4;    // 0-indexed
        for (int i = 0; i < 4; i++) {
            int o = i * 2;
            if (o + 1 < len) {
                uint16_t mv = u16be(data, o);
                if (mv != 0 && (base + i) < 19) {
                    bms.cell_mv[base + i] = mv;
                    if (base + i + 1 > bms.cell_count)
                        bms.cell_count = base + i + 1;
                }
            }
        }
        // Recompute derived stats
        uint32_t sum = 0;
        bms.cell_max = 0;
        bms.cell_min = 0xFFFF;
        for (int i = 0; i < bms.cell_count; i++) {
            if (!bms.cell_mv[i]) continue;
            sum += bms.cell_mv[i];
            if (bms.cell_mv[i] > bms.cell_max) bms.cell_max = bms.cell_mv[i];
            if (bms.cell_mv[i] < bms.cell_min) bms.cell_min = bms.cell_mv[i];
        }
        bms.cell_avg = bms.cell_count ? (uint16_t)(sum / bms.cell_count) : 0;
        bms.pack_v   = sum / 1000.0f;
        float s = (bms.cell_avg - CELL_UV) / (float)(SOC_CAR_TOP_MV - CELL_UV) * 100.0f;
        bms.soc = max(0.0f, min(100.0f, s));
    }
    // Temperatures: func 0xB4, raw − 40 = °C, 0xFF = absent
    else if (func == 0xB4 && len >= 4) {
        bms.temp_count = 0;
        for (int i = 0; i < 4 && i < len; i++) {
            if (data[i] != 0xFF && data[i] != 0x00)
                bms.temp_c[bms.temp_count++] = (float)data[i] - 40.0f;
        }
    }
    // Min/Max summary: func 0xFE sub 0x28, little-endian
    else if (func == 0xFE && sub == 0x28 && len >= 4) {
        bms.cell_max = u16le(data, 0);
        bms.cell_min = u16le(data, 2);
    }
}

// ─── JSON builder ─────────────────────────────────────────────────────────────
String buildJSON(const char* timestamp) {
    JsonDocument doc;

    doc["connected"]    = true;
    doc["source"]       = "esp32-sim800l";
    doc["device_id"]    = DEVICE_ID;
    doc["frame_count"]  = bms.frame_count;

    if (strlen(timestamp) > 0)
        doc["timestamp"] = timestamp;
    else
        doc["uptime_ms"] = millis();

    if (bms.soc >= 0)
        doc["soc"]      = roundf(bms.soc * 10.0f)     / 10.0f;
    if (bms.pack_v > 0)
        doc["pack_v"]   = roundf(bms.pack_v * 100.0f)  / 100.0f;

    doc["cell_max_mv"]  = (int)bms.cell_max;
    doc["cell_min_mv"]  = (bms.cell_min < 0xFFFF) ? (int)bms.cell_min : 0;
    doc["cell_avg_mv"]  = (int)bms.cell_avg;
    doc["cell_count"]   = (int)bms.cell_count;
    if (bms.cell_count > 1)
        doc["cell_spread_mv"] = (int)(bms.cell_max - bms.cell_min);

    // Individual cells
    JsonObject cells = doc["cells"].to<JsonObject>();
    for (int i = 0; i < bms.cell_count; i++) {
        if (!bms.cell_mv[i]) continue;
        JsonObject c = cells[String(i + 1)].to<JsonObject>();
        c["mv"]     = bms.cell_mv[i];
        c["status"] = cellStatus(bms.cell_mv[i]);
    }

    // Temperatures
    JsonObject temps = doc["temps"].to<JsonObject>();
    float tsum = 0.0f;
    for (int i = 0; i < bms.temp_count; i++) {
        temps[String(i + 1)] = roundf(bms.temp_c[i] * 10.0f) / 10.0f;
        tsum += bms.temp_c[i];
    }
    if (bms.temp_count > 0)
        doc["avg_temp"] = roundf(tsum / bms.temp_count * 10.0f) / 10.0f;

    String out;
    serializeJson(doc, out);
    return out;
}

// ─── Firebase publish (PUT /bms/live + POST /bms/history) ─────────────────────
bool publishToFirebase() {
    char timestamp[32] = "";
    getNetworkTime(timestamp, sizeof(timestamp));

    String body = buildJSON(timestamp);
    Serial.printf("[JSON] %s\n", body.c_str());

    // PUT → always overwrite live snapshot (dashboard view)
    char livePath[128];
    snprintf(livePath, sizeof(livePath), "%s/bms/live.json?auth=%s",
             FIREBASE_HOST, FIREBASE_AUTH);

    bool ok = httpPut(livePath, body.c_str(), body.length());

    // POST → append to history log (creates auto-key entry)
    if (ok) {
        char histPath[128];
        snprintf(histPath, sizeof(histPath), "%s/bms/history.json?auth=%s",
                 FIREBASE_HOST, FIREBASE_AUTH);
        httpPost(histPath, body.c_str(), body.length());   // best-effort, ignore failure
    }

    return ok;
}

// ─── SIM800L HTTP service — PUT ────────────────────────────────────────────────
bool httpRequest(const char* method, int methodId,
                 const char* url, const char* body, size_t bodyLen) {

    if (!sendATExpect("AT+HTTPINIT", "OK", 3000)) {
        Serial.println(F("[HTTP] HTTPINIT failed"));
        return false;
    }
    sendAT("AT+HTTPPARA=\"CID\",1", 1000);

    char urlCmd[600];
    snprintf(urlCmd, sizeof(urlCmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    sendAT(urlCmd, 1000);

    sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
    sendAT("AT+HTTPSSL=1", 1000);

    // Upload body
    char dataCmd[64];
    snprintf(dataCmd, sizeof(dataCmd), "AT+HTTPDATA=%d,10000", (int)bodyLen);
    if (!sendATExpect(dataCmd, "DOWNLOAD", 5000)) {
        Serial.println(F("[HTTP] HTTPDATA prompt failed"));
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }
    sim800l.write((const uint8_t*)body, bodyLen);
    delay(2000);
    readResponse(responseBuf, sizeof(responseBuf), 3000);

    // Execute request (0=GET, 1=POST, 2=HEAD, 3=PUT)
    Serial.printf("[HTTP] >> AT+HTTPACTION=%d\n", methodId);
    sim800l.printf("AT+HTTPACTION=%d\r\n", methodId);

    // Wait for +HTTPACTION (SSL handshake can take 15–30 s)
    char actionResp[128] = {};
    size_t actionLen = 0;
    unsigned long start = millis();
    while (millis() - start < 30000) {
        while (sim800l.available() && actionLen < sizeof(actionResp) - 1)
            actionResp[actionLen++] = sim800l.read();
        if (strstr(actionResp, "+HTTPACTION:")) break;
        delay(100);
    }
    Serial.println(actionResp);

    bool success = false;
    char* ptr = strstr(actionResp, "+HTTPACTION:");
    if (ptr) {
        int m, statusCode, dataLen;
        if (sscanf(ptr, "+HTTPACTION: %d,%d,%d", &m, &statusCode, &dataLen) == 3) {
            Serial.printf("[HTTP] %s %d  (%d bytes)\n", method, statusCode, dataLen);
            success = (statusCode == 200);
            if (dataLen > 0) sendAT("AT+HTTPREAD", 3000);
        }
    }

    sendAT("AT+HTTPTERM", 1000);
    return success;
}

bool httpPut(const char* url, const char* body, size_t bodyLen) {
    return httpRequest("PUT", 3, url, body, bodyLen);
}

bool httpPost(const char* url, const char* body, size_t bodyLen) {
    return httpRequest("POST", 1, url, body, bodyLen);
}

// ─── GPRS connection ──────────────────────────────────────────────────────────
bool connectGPRS() {
    Serial.println(F("[GSM] Connecting GPRS..."));
    sendAT("AT", 1000);

    if (!sendATExpect("AT+CPIN?", "READY", 5000)) {
        Serial.println(F("[GSM] SIM not ready — check SIM card"));
        return false;
    }
    Serial.println(F("[GSM] SIM ready"));

    checkSignal();

    if (!sendATExpect("AT+CREG?", "0,1", 10000) &&
        !sendATExpect("AT+CREG?", "0,5",  5000)) {
        Serial.println(F("[GSM] Not registered on network — continuing anyway"));
    }
    return true;
}

bool openBearer() {
    Serial.println(F("[GSM] Opening bearer..."));

    sendAT("AT+SAPBR=0,1", 2000);   // close any stale bearer
    sendAT("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", 1000);

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", APN);
    sendAT(cmd, 1000);

    if (strlen(APN_USER) > 0) {
        snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"USER\",\"%s\"", APN_USER);
        sendAT(cmd, 1000);
    }
    if (strlen(APN_PASS) > 0) {
        snprintf(cmd, sizeof(cmd), "AT+SAPBR=3,1,\"PWD\",\"%s\"", APN_PASS);
        sendAT(cmd, 1000);
    }

    if (!sendATExpect("AT+SAPBR=1,1", "OK", 15000)) {
        Serial.println(F("[GSM] Bearer open failed"));
        return false;
    }
    if (!sendATExpect("AT+SAPBR=2,1", "+SAPBR: 1,1", 5000)) {
        Serial.println(F("[GSM] Bearer not fully connected"));
        return false;
    }
    Serial.println(F("[GSM] Bearer OK"));
    return true;
}

void closeBearer() {
    sendAT("AT+SAPBR=0,1", 2000);
}

// ─── AT helpers ───────────────────────────────────────────────────────────────
void sendAT(const char* cmd, int waitMs) {
    Serial.printf(">> %s\n", cmd);
    sim800l.println(cmd);
    delay(waitMs);
    while (sim800l.available()) Serial.write(sim800l.read());
    Serial.println();
}

bool sendATExpect(const char* cmd, const char* expected, int waitMs) {
    Serial.printf(">> %s\n", cmd);
    sim800l.println(cmd);

    char buf[256] = {};
    size_t len = 0;
    unsigned long start = millis();

    while (millis() - start < (unsigned long)waitMs) {
        while (sim800l.available() && len < sizeof(buf) - 1)
            buf[len++] = sim800l.read();
        buf[len] = '\0';
        if (strstr(buf, expected)) { Serial.println(buf); return true; }
        delay(10);
    }
    Serial.println(buf);
    return false;
}

size_t readResponse(char* buf, size_t bufLen, unsigned long timeoutMs) {
    unsigned long start = millis();
    size_t len = 0;
    while (millis() - start < timeoutMs && len < bufLen - 1) {
        if (sim800l.available()) buf[len++] = sim800l.read();
    }
    buf[len] = '\0';
    return len;
}

void checkSignal() {
    sim800l.println("AT+CSQ");
    delay(1000);
    char resp[64] = {};
    size_t len = 0;
    while (sim800l.available() && len < sizeof(resp) - 1)
        resp[len++] = sim800l.read();
    Serial.print(resp);
    char* idx = strstr(resp, "+CSQ: ");
    if (idx) {
        int rssi = atoi(idx + 6);
        if      (rssi == 99) Serial.println(F("[GSM] No signal"));
        else if (rssi <  10) Serial.println(F("[GSM] Poor signal"));
        else if (rssi <  20) Serial.println(F("[GSM] Good signal"));
        else                 Serial.println(F("[GSM] Excellent signal"));
    }
}

void getNetworkTime(char* buf, size_t bufLen) {
    sim800l.println("AT+CCLK?");
    delay(1000);
    char resp[128] = {};
    size_t len = 0;
    while (sim800l.available() && len < sizeof(resp) - 1)
        resp[len++] = sim800l.read();

    // Format: +CCLK: "yy/MM/dd,HH:mm:ss±zz"
    char* qs = strchr(resp, '"');
    char* qe = qs ? strchr(qs + 1, '"') : nullptr;
    if (qs && qe && (size_t)(qe - qs - 1) < bufLen) {
        size_t n = qe - qs - 1;
        strncpy(buf, qs + 1, n);
        buf[n] = '\0';
    } else {
        buf[0] = '\0';
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    sim800l.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
    delay(3000);

    Serial.println(F("\n========================================"));
    Serial.println(F("   ESP32 BMS Firebase Publisher"));
    Serial.println(F("========================================\n"));

    // ── MCP2515 CAN ──────────────────────────────────────────────────────────
    Serial.print(F("[CAN] Init MCP2515... "));
    if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
        pinMode(CAN_INT, INPUT);
        Serial.println(F("OK (250 kbps)"));
    } else {
        Serial.println(F("FAILED — check CS/INT/SPI wiring"));
    }

    // ── GPRS with retries ─────────────────────────────────────────────────────
    bool connected = false;
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        Serial.printf("[GSM] Attempt %d/%d\n", attempt, MAX_RETRIES);
        if (connectGPRS() && openBearer()) { connected = true; break; }
        delay(3000);
    }
    if (!connected) {
        Serial.println(F("[GSM] All retries failed — restarting in 10 s..."));
        delay(10000);
        ESP.restart();
    }

    Serial.println(F("\n=== Ready — reading CAN bus ===\n"));
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // Drain all pending CAN frames before the next publish
    while (!digitalRead(CAN_INT)) {
        unsigned long rxId = 0;
        uint8_t  len  = 0;
        uint8_t  buf[8] = {};

        if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
            if (rxId & 0x80000000) rxId &= 0x1FFFFFFF;   // strip extended flag
            decodeFrame(rxId, len, buf);
        }
    }

    if (millis() - lastPublish >= PUBLISH_INTERVAL) {
        lastPublish = millis();

        if (bms.frame_count == 0) {
            Serial.println(F("[CAN] No frames yet — waiting..."));
            return;
        }

        Serial.printf("[CAN] Frames: %lu  SOC: %.1f%%  Pack: %.2f V  "
                      "Cells: %d  Temps: %d\n",
                      bms.frame_count, bms.soc, bms.pack_v,
                      bms.cell_count, bms.temp_count);

        if (!publishToFirebase()) {
            Serial.println(F("[FB] Failed — reconnecting bearer..."));
            closeBearer();
            delay(2000);
            if (connectGPRS() && openBearer()) {
                if (!publishToFirebase())
                    Serial.println(F("[FB] Retry also failed — skipping"));
            }
        }
    }
}
