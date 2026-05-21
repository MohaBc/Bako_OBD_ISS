/*
 * ESP32 BMS Cloud Publisher
 * =========================
 * Reads O'CELL IFS60.8-500 BMS CAN frames via MCP2515 (250 kbps),
 * decodes SAE J1939 data, and POSTs a live JSON snapshot to a VPS
 * backend every PUBLISH_INTERVAL ms via SIM800L GPRS.
 *
 * Uses the SIM800L AT+HTTP* service over plain HTTP (no SSL).
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
 *   VPS_HOST     → IP or hostname of the VPS  (e.g. "203.0.113.42")
 *   VPS_PORT     → port the server listens on (default "8787")
 *   VPS_PATH     → ingest endpoint            (default "/api/ingest")
 *   VPS_API_KEY  → must match BMS_API_KEY env var on the VPS
 *   APN          → your SIM carrier APN  (e.g. "internet", "iam", "maroctel")
 *
 * Cloud endpoint
 * ──────────────
 *   POST  http://VPS_HOST:VPS_PORT/VPS_PATH   ← bearer: X-Api-Key header
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
    // Cell voltages (0x18C8-CC28F4)
    uint16_t cell_mv[19]  = {};
    uint8_t  cell_count   = 0;     // highest cell index seen + 1
    uint8_t  valid_cells  = 0;     // non-zero cells actually decoded
    uint16_t cell_max     = 0;
    uint16_t cell_min     = 0xFFFF;
    uint16_t cell_avg     = 0;

    // Temperatures (0x18B428F4 — up to 8 probes)
    float    temp_c[8]    = {};
    uint8_t  temp_count   = 0;

    // Derived
    float    soc          = -1.0f;
    float    pack_v       = 0.0f;  // summed from cells (may be partial)
    uint32_t frame_count  = 0;

    // 0x18FF28F4 — BMS Basic Message 1 (100 ms)
    uint8_t  status_flags = 0;     // byte 0 bit-field
    uint8_t  soc_bms      = 0xFF;  // byte 1: coulomb-counter SOC 0–100, 0xFF=unknown
    float    pack_current = 0.0f;  // bytes 2–3: + = discharging, − = charging (A)
    float    pack_v_bms   = 0.0f;  // bytes 4–5: direct BMS pack voltage (V)
    uint8_t  fault_level  = 0;     // byte 6: 0=OK, 1=serious fault
    uint8_t  error_code   = 0;     // byte 7: fault code (see doc section 4)

    // 0x18FE28F4 — BMS Basic Message 2 (100 ms, extended)
    float    temp_max_c   = -99.0f;
    float    temp_min_c   = -99.0f;
    float    max_disch_a  = 0.0f;

    // 0x18FFE5F4 — BMS Charging Request (1000 ms, to on-board charger)
    float    charge_max_v  = 0.0f;   // max allowable charge terminal voltage (V)
    float    charge_max_a  = 0.0f;   // max allowable charge current (A)
    bool     charger_start = false;  // byte 5 bit 1: 0 = charger start charging
    uint8_t  charge_prot   = 0;      // byte 6 protection window flags
};

static BMSData       bms;
static unsigned long lastPublish = 0;
static bool          g_moduleRebooted = false;   // set when "Call Ready" URC detected
static bool          g_canOK = false;            // set only if MCP2515 init succeeded

MCP_CAN CAN0(CAN_CS);
char    responseBuf[512];

// ─── Prototypes ───────────────────────────────────────────────────────────────
bool   connectGPRS();
bool   openBearer();
void   closeBearer();
bool   publishToCloud();
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

// ─── Fault code lookup (section 4 of BAKO CAN Protocol doc) ──────────────────
// Prefix 0xx = BMS fault; 1xx prefix = motor controller fault (not on this bus)
static const char* faultCodeName(uint8_t code) {
    switch (code) {
        case 0x00: return "ok";
        case 0x01: return "over_temp_severe";          // Batt max temp > protection limit
        case 0x02: return "total_voltage_high";        // > total volt protection limit
        case 0x03: return "total_voltage_low";         // < lower limit
        case 0x04: return "discharge_overcurrent";     // > discharge protection (may be short)
        case 0x05: return "cell_voltage_high";         // max cell > single cell high protection
        case 0x06: return "cell_voltage_low";          // min cell < low volt protection
        default:   return "unknown";
    }
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

    // ── 0x18C8–CC28F4  Cell voltages (500 ms) ─────────────────────────────────
    // Big-endian uint16 pairs, 4 cells per frame, group = PF − 0xC8
    if (func >= 0xC8 && func <= 0xCC && len == 8) {
        uint8_t base = (func - 0xC8) * 4;
        for (int i = 0; i < 4; i++) {
            uint16_t mv = u16be(data, i * 2);
            if (mv != 0 && (base + i) < 19) {
                bms.cell_mv[base + i] = mv;
                if (base + i + 1 > bms.cell_count)
                    bms.cell_count = base + i + 1;
            }
        }
        // Recompute stats using only valid (non-zero) cells
        uint32_t sum = 0;
        uint8_t  valid = 0;
        bms.cell_max = 0;
        bms.cell_min = 0xFFFF;
        for (int i = 0; i < bms.cell_count; i++) {
            if (!bms.cell_mv[i]) continue;
            sum += bms.cell_mv[i];
            valid++;
            if (bms.cell_mv[i] > bms.cell_max) bms.cell_max = bms.cell_mv[i];
            if (bms.cell_mv[i] < bms.cell_min) bms.cell_min = bms.cell_mv[i];
        }
        bms.valid_cells = valid;
        bms.cell_avg    = valid ? (uint16_t)(sum / valid) : 0;
        bms.pack_v      = sum / 1000.0f;   // sum of decoded cells in V
        float s = (bms.cell_avg - CELL_UV) / (float)(SOC_CAR_TOP_MV - CELL_UV) * 100.0f;
        bms.soc = max(0.0f, min(100.0f, s));
    }

    // ── 0x18B428F4  Temperature probes (500 ms) ────────────────────────────────
    // Up to 8 probes, uint8 raw − 40 = °C, 0xFF = not connected
    else if (func == 0xB4 && len >= 1) {
        bms.temp_count = 0;
        for (int i = 0; i < (int)len && i < 8; i++) {
            if (data[i] != 0xFF && data[i] != 0x00)
                bms.temp_c[bms.temp_count++] = (float)data[i] - 40.0f;
        }
    }

    // ── 0x18FF28F4  BMS Basic Message 1 (100 ms) ───────────────────────────────
    // Byte 0: status flags | Byte 1: SOC% | Bytes 2–3: current | Bytes 4–5: voltage
    // Byte 6: fault level | Byte 7: error code
    else if (func == 0xFF && sub == 0x28 && len >= 8) {
        bms.status_flags = data[0];
        bms.soc_bms      = data[1];   // 0–100, direct from BMS coulomb counter

        // Pack current: LE uint16, offset −5000, scale 0.1 A/bit
        // 5000 = 0 A; >5000 = discharging (+); <5000 = charging (−)
        bms.pack_current = (int16_t)(u16le(data, 2) - 5000) * 0.1f;

        // Pack voltage: LE uint16, scale 0.1 V/bit
        bms.pack_v_bms = u16le(data, 4) * 0.1f;

        bms.fault_level = data[6];
        bms.error_code  = data[7];
    }

    // ── 0x18FFE5F4  BMS Charging Request (1000 ms) ────────────────────────────
    // SA=0xF4(BMS) → PS=0xE5(on-board charger). Tells charger the allowed limits.
    // Bytes 0–1: max charge terminal voltage  LE uint16, 0.1 V/bit
    // Bytes 2–3: max charge current           LE uint16, 0.1 A/bit
    // Byte 4 bit 0: charger start signal      0 = start charging, 1 = stop
    // Bytes 5–7: protection window flags
    else if (func == 0xFF && sub == 0xE5 && len >= 5) {
        bms.charge_max_v  = u16le(data, 0) * 0.1f;
        bms.charge_max_a  = u16le(data, 2) * 0.1f;
        bms.charger_start = !(data[4] & 0x01);   // 0 = charger should start
        if (len >= 6) bms.charge_prot = data[5];
    }

    // ── 0x18FE28F4  BMS Basic Message 2 (100 ms) ───────────────────────────────
    // Bytes 0–1: max cell mV | Bytes 2–3: min cell mV
    // Byte 4: temp_max | Byte 5: temp_min | Bytes 6–7: max discharge current
    else if (func == 0xFE && sub == 0x28 && len >= 8) {
        bms.cell_max = u16le(data, 0);
        bms.cell_min = u16le(data, 2);

        if (data[4] != 0xFF) bms.temp_max_c  = (float)data[4] - 40.0f;
        if (data[5] != 0xFF) bms.temp_min_c  = (float)data[5] - 40.0f;
        bms.max_disch_a = u16le(data, 6) * 0.1f;

        // Fallback SOC/pack_v when cell voltage frames haven't arrived yet
        if (bms.cell_count == 0 && bms.cell_max > 0) {
            uint16_t avg = (bms.cell_max + bms.cell_min) / 2;
            bms.pack_v = avg * 19 / 1000.0f;
            float s = (avg - CELL_UV) / (float)(SOC_CAR_TOP_MV - CELL_UV) * 100.0f;
            bms.soc = max(0.0f, min(100.0f, s));
        }
    }
}

// ─── JSON builder — nested schema ─────────────────────────────────────────────
String buildJSON(const char* timestamp) {
    JsonDocument doc;

    // ── Top-level metadata ─────────────────────────────────────────────────────
    doc["device_id"]   = DEVICE_ID;
    if (strlen(timestamp) > 0)
        doc["timestamp"] = timestamp;
    else
        doc["uptime_ms"] = millis();
    doc["source"]      = "esp32-sim800l";
    doc["connected"]   = true;
    doc["frame_count"] = bms.frame_count;
    doc["fault_level"] = (int)bms.fault_level;
    doc["error_code"]  = (int)bms.error_code;

    // ── solar (stub — sensor group reserved for future hardware) ──────────────
    JsonObject solar       = doc["solar"].to<JsonObject>();
    JsonObject solarPre    = solar["pre_mppt"].to<JsonObject>();
    solarPre["voltage_v"]  = nullptr;
    solarPre["current_a"]  = nullptr;
    JsonObject solarPost   = solar["post_mppt"].to<JsonObject>();
    solarPost["current_a"] = nullptr;

    // ── dc_dc (stub) ───────────────────────────────────────────────────────────
    JsonObject dcdc     = doc["dc_dc"].to<JsonObject>();
    JsonObject dc64     = dcdc["output_64v"].to<JsonObject>();
    dc64["voltage_v"]   = nullptr;
    JsonObject dc12     = dcdc["output_12v"].to<JsonObject>();
    dc12["voltage_v"]   = nullptr;

    // ── battery ───────────────────────────────────────────────────────────────
    JsonObject battery = doc["battery"].to<JsonObject>();

    float pack_v = bms.pack_v_bms > 0 ? bms.pack_v_bms : bms.pack_v;
    battery["pack_v"]         = roundf(pack_v * 10.0f) / 10.0f;
    battery["pack_current_a"] = roundf(bms.pack_current * 10.0f) / 10.0f;

    if (bms.soc >= 0)
        battery["soc"]     = roundf(bms.soc * 10.0f) / 10.0f;
    if (bms.soc_bms <= 100)
        battery["soc_bms"] = (int)bms.soc_bms;

    battery["max_disch_a"]    = bms.max_disch_a > 0 ? bms.max_disch_a : (float)0;
    battery["cell_count"]     = (int)bms.valid_cells;
    battery["cell_avg_mv"]    = (int)bms.cell_avg;
    battery["cell_min_mv"]    = (bms.cell_min < 0xFFFF) ? (int)bms.cell_min : 0;
    battery["cell_max_mv"]    = (int)bms.cell_max;
    battery["cell_spread_mv"] = (bms.valid_cells > 1)
                                ? (int)(bms.cell_max - bms.cell_min) : 0;

    // cells: 1-based array — index 0 is null, indices 1-19 are cell objects
    JsonArray cells = battery["cells"].to<JsonArray>();
    cells.add(nullptr);
    for (int i = 0; i < 19; i++) {
        if (bms.cell_mv[i] == 0) {
            cells.add(nullptr);
        } else {
            JsonObject c = cells.add<JsonObject>();
            c["mv"]     = bms.cell_mv[i];
            c["status"] = cellStatus(bms.cell_mv[i]);
        }
    }

    // charger request
    JsonObject charger      = battery["charger"].to<JsonObject>();
    charger["max_charge_v"] = roundf(bms.charge_max_v * 10.0f) / 10.0f;
    charger["max_charge_a"] = roundf(bms.charge_max_a * 10.0f) / 10.0f;
    charger["start_signal"] = bms.charger_start;

    // status bit-field
    JsonObject st            = battery["status"].to<JsonObject>();
    st["charge_cable"]       = (bool)(bms.status_flags & 0x01);
    st["charging"]           = (bool)(bms.status_flags & 0x02);
    st["discharging"]        = (bool)(bms.status_flags & 0x04);
    st["ready"]              = (bool)(bms.status_flags & 0x08);
    st["disch_contactor"]    = (bool)(bms.status_flags & 0x10);
    st["charge_contactor"]   = (bool)(bms.status_flags & 0x20);

    // ── temperatures ──────────────────────────────────────────────────────────
    JsonObject temperatures  = doc["temperatures"].to<JsonObject>();
    temperatures["motor_c"]  = nullptr;
    temperatures["mppt_c"]   = nullptr;
    temperatures["cabin_c"]  = nullptr;

    if (bms.temp_count > 0) {
        float tsum = 0.0f;
        for (int i = 0; i < bms.temp_count; i++) tsum += bms.temp_c[i];
        temperatures["battery_avg_c"] = roundf(tsum / bms.temp_count * 10.0f) / 10.0f;
    } else {
        temperatures["battery_avg_c"] = nullptr;
    }
    if (bms.temp_min_c > -99.0f) temperatures["battery_min_c"] = bms.temp_min_c;
    else temperatures["battery_min_c"] = nullptr;
    if (bms.temp_max_c > -99.0f) temperatures["battery_max_c"] = bms.temp_max_c;
    else temperatures["battery_max_c"] = nullptr;

    // battery_cells: 1-based array — index 0 null, indices 1-4 probe readings
    JsonArray tempCells = temperatures["battery_cells"].to<JsonArray>();
    tempCells.add(nullptr);
    for (int i = 0; i < 4; i++) {
        if (i < bms.temp_count)
            tempCells.add(roundf(bms.temp_c[i] * 10.0f) / 10.0f);
        else
            tempCells.add(nullptr);
    }

    // ── vehicle (stub) ────────────────────────────────────────────────────────
    JsonObject vehicle   = doc["vehicle"].to<JsonObject>();
    vehicle["handbrake"] = nullptr;

    String out;
    serializeJson(doc, out);
    return out;
}

// ─── VPS cloud publish (POST /api/ingest) ────────────────────────────────────
bool publishToCloud() {
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%s%s", VPS_HOST, VPS_PORT, VPS_PATH);

#ifdef TEST_MINIMAL_JSON
    // Plain-text hello — proves the GPRS → VPS path without any JSON
    snprintf(url, sizeof(url), "http://%s:%s/hello", VPS_HOST, VPS_PORT);
    const char* body = "Hello to Bako";
    Serial.printf("[HELLO-TEST] sending: %s\n", body);
    return httpPost(url, body, strlen(body));
#else
    char timestamp[32] = "";
    getNetworkTime(timestamp, sizeof(timestamp));
    String body = buildJSON(timestamp);
    Serial.printf("[JSON] %s\n", body.c_str());
    return httpPost(url, body.c_str(), body.length());
#endif
}

// ─── SIM800L HTTP service — PUT ────────────────────────────────────────────────
bool httpRequest(const char* method, int methodId,
                 const char* url, const char* body, size_t bodyLen) {

    // Close any lingering HTTP session — retry once if first attempt fails
    // (session may still be alive from a previous timeout)
    sendAT("AT+HTTPTERM", 1000);
    delay(500);
    sendAT("AT+HTTPTERM", 1000);   // second call always succeeds once session is gone

    if (!sendATExpect("AT+HTTPINIT", "OK", 3000)) {
        Serial.println(F("[HTTP] HTTPINIT failed"));
        return false;
    }
    sendAT("AT+HTTPPARA=\"CID\",1", 1000);

    char urlCmd[600];
    snprintf(urlCmd, sizeof(urlCmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    sendAT(urlCmd, 1000);

    #ifdef TEST_MINIMAL_JSON
    sendAT("AT+HTTPPARA=\"CONTENT\",\"text/plain\"", 1000);
    #else
    sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 1000);
    #endif
    sendAT("AT+HTTPSSL=0", 500);

    // USERDATA — no \r\n at the end; SIM800L adds CRLF automatically.
    // Including \r\n in the value adds an extra blank line that shifts body offset.
    char hdrCmd[128];
    snprintf(hdrCmd, sizeof(hdrCmd), "AT+HTTPPARA=\"USERDATA\",\"X-Api-Key: %s\"", VPS_API_KEY);
    sendAT(hdrCmd, 500);

    // Upload body
    char dataCmd[64];
    snprintf(dataCmd, sizeof(dataCmd), "AT+HTTPDATA=%d,10000", (int)bodyLen);
    if (!sendATExpect(dataCmd, "DOWNLOAD", 5000)) {
        Serial.println(F("[HTTP] HTTPDATA prompt failed"));
        sendAT("AT+HTTPTERM", 1000);
        return false;
    }
    sim800l.write((const uint8_t*)body, bodyLen);
    delay(3000);   // extra time to ensure modem fully buffers body before HTTPACTION
    readResponse(responseBuf, sizeof(responseBuf), 2000);
    Serial.printf("[HTTP] after-data resp: %s\n", responseBuf);

    // Execute request — SIM800L supports 0=GET 1=POST 2=HEAD only (no native PUT)
    Serial.printf("[HTTP] >> AT+HTTPACTION=%d\n", methodId);
    sim800l.printf("AT+HTTPACTION=%d\r\n", methodId);

    // Wait for +HTTPACTION — plain HTTP to Railway typically resolves in < 5 s.
    // Drain CAN bus in the same loop so no cell-voltage frames are lost while
    // the modem is busy — the BMS struct will be fully populated before the
    // next publish cycle starts.
    char actionResp[256] = {};
    size_t actionLen = 0;
    unsigned long start = millis();
    while (millis() - start < 30000) {
        // Keep reading CAN frames during the HTTP wait
        if (g_canOK) {
            while (!digitalRead(CAN_INT)) {
                unsigned long rxId = 0;
                uint8_t rxLen = 0;
                uint8_t rxBuf[8] = {};
                if (CAN0.readMsgBuf(&rxId, &rxLen, rxBuf) == CAN_OK) {
                    if (rxId & 0x80000000) rxId &= 0x1FFFFFFF;
                    decodeFrame(rxId, rxLen, rxBuf);
                }
            }
        }

        // Drain modem UART
        while (sim800l.available()) {
            char c = sim800l.read();
            // Keep only the last 255 bytes so the URC is never pushed out
            if (actionLen >= sizeof(actionResp) - 1) {
                memmove(actionResp, actionResp + 1, sizeof(actionResp) - 2);
                actionLen = sizeof(actionResp) - 2;
            }
            actionResp[actionLen++] = c;
            actionResp[actionLen]   = '\0';
        }
        // Abort early if the module rebooted mid-transaction
        if (strstr(actionResp, "Call Ready")) {
            Serial.println(F("[HTTP] Module reboot detected during HTTPACTION — aborting"));
            g_moduleRebooted = true;
            break;
        }
        // Once we see +HTTPACTION: keep draining for 500 ms to collect the
        // full "method,status,dataLen" line — SIM800L sometimes splits it
        if (strstr(actionResp, "+HTTPACTION:")) {
            unsigned long tail = millis();
            while (millis() - tail < 500) {
                while (sim800l.available()) {
                    char c = sim800l.read();
                    if (actionLen >= sizeof(actionResp) - 1) {
                        memmove(actionResp, actionResp + 1, sizeof(actionResp) - 2);
                        actionLen = sizeof(actionResp) - 2;
                    }
                    actionResp[actionLen++] = c;
                    actionResp[actionLen]   = '\0';
                }
                delay(10);
            }
            break;
        }
        delay(10);   // tighter poll so CAN frames aren't delayed
    }
    Serial.println(actionResp);

    bool success = false;
    char* ptr = strstr(actionResp, "+HTTPACTION:");
    if (ptr) {
        int m, statusCode, dataLen = 0;
        int parsed = sscanf(ptr, "+HTTPACTION: %d,%d,%d", &m, &statusCode, &dataLen);
        if (parsed >= 2) {   // dataLen may arrive late — status code is enough
            Serial.printf("[HTTP] %s %d  (%d bytes)\n", method, statusCode, dataLen);
            success = (statusCode == 200);
            if (dataLen > 0) sendAT("AT+HTTPREAD", 3000);
        }
    }

    sendAT("AT+HTTPTERM", 1000);
    return success;
}

bool httpPost(const char* url, const char* body, size_t bodyLen) {
    return httpRequest("POST", 1, url, body, bodyLen);
}

// ─── GPRS connection ──────────────────────────────────────────────────────────
bool connectGPRS() {
    Serial.println(F("[GSM] Connecting GPRS..."));
    sendAT("AT", 1000);

    // After a module reboot the SIM can take 15–20 s to initialise.
    // Retry AT+CPIN? up to 6× (≈ 30 s total) before giving up.
    bool simReady = false;
    for (int attempt = 0; attempt < 6 && !simReady; attempt++) {
        if (attempt > 0) {
            Serial.printf("[GSM] SIM not ready yet — retry %d/5...\n", attempt);
            delay(4000);
        }
        simReady = sendATExpect("AT+CPIN?", "READY", 5000);
    }
    if (!simReady) {
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

    // SAPBR=1,1 can take up to 30 s on Ooredoo — retry once before giving up
    bool bearerUp = sendATExpect("AT+SAPBR=1,1", "OK", 20000);
    if (!bearerUp) {
        Serial.println(F("[GSM] Bearer open slow — retrying..."));
        sendAT("AT+SAPBR=0,1", 3000);
        bearerUp = sendATExpect("AT+SAPBR=1,1", "OK", 25000);
    }
    if (!bearerUp) {
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
static void flushRX() {
    // Discard any stale bytes left in the UART RX buffer before a new command
    delay(20);
    while (sim800l.available()) sim800l.read();
}

void sendAT(const char* cmd, int waitMs) {
    flushRX();
    Serial.printf(">> %s\n", cmd);
    sim800l.println(cmd);
    delay(waitMs);
    // Buffer the response so we can scan for the reboot URC
    char tmp[256] = {}; size_t tl = 0;
    while (sim800l.available() && tl < sizeof(tmp) - 1)
        tmp[tl++] = sim800l.read();
    Serial.print(tmp);
    Serial.println();
    if (strstr(tmp, "Call Ready")) {
        g_moduleRebooted = true;
        Serial.println(F("[GSM] !! Module reboot detected (Call Ready) !!"));
    }
}

bool sendATExpect(const char* cmd, const char* expected, int waitMs) {
    flushRX();
    Serial.printf(">> %s\n", cmd);
    sim800l.println(cmd);

    char buf[256] = {};
    size_t len = 0;
    unsigned long start = millis();

    while (millis() - start < (unsigned long)waitMs) {
        while (sim800l.available() && len < sizeof(buf) - 1)
            buf[len++] = sim800l.read();
        buf[len] = '\0';
        if (strstr(buf, "Call Ready")) {
            g_moduleRebooted = true;
            Serial.println(F("[GSM] !! Module reboot detected (Call Ready) !!"));
            Serial.println(buf);
            return false;
        }
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
    // Flush any stale AT responses left in the RX buffer
    while (sim800l.available()) sim800l.read();

    sim800l.println("AT+CCLK?");
    delay(1500);
    char resp[128] = {};
    size_t len = 0;
    while (sim800l.available() && len < sizeof(resp) - 1)
        resp[len++] = sim800l.read();

    // Format: +CCLK: "yy/MM/dd,HH:mm:ss±zz"
    // Anchor on "+CCLK: \"" to avoid picking up stray quote bytes in UART noise
    char* anchor = strstr(resp, "+CCLK: \"");
    char* qs = anchor ? anchor + 8 : nullptr;   // skip past: +CCLK: "
    char* qe = qs ? strchr(qs, '"') : nullptr;
    if (qs && qe && (size_t)(qe - qs) < bufLen) {
        size_t n = qe - qs;
        strncpy(buf, qs, n);
        buf[n] = '\0';
    } else {
        buf[0] = '\0';
    }
}

// ─── SIM800L hardware reset via RST pin ───────────────────────────────────────
void hardResetSIM800L() {
    Serial.println(F("[GSM] Hardware reset via RST pin..."));
    pinMode(SIM_RST, OUTPUT);
    digitalWrite(SIM_RST, LOW);
    delay(200);
    digitalWrite(SIM_RST, HIGH);
    delay(3000);   // module boot time after RST
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Hardware-reset the SIM800L before UART init to clear any stuck state
    hardResetSIM800L();

    sim800l.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
    delay(2000);

    Serial.println(F("\n========================================"));
    Serial.println(F("   ESP32 BMS Cloud Publisher (VPS)"));
    Serial.printf ("   VPS  : %s:%s\n", VPS_HOST, VPS_PORT);
    Serial.printf ("   APN  : %s\n", APN);
    Serial.printf ("   DEV  : %s\n", DEVICE_ID);
    Serial.println(F("========================================\n"));

    // ── MCP2515 CAN ──────────────────────────────────────────────────────────
    // NOTE: CAN0.begin() calls SPI.begin() which by default takes GPIO 18 (SCK)
    // and GPIO 19 (MISO). If SIM_TX/SIM_RX share those pins, UART must be
    // re-initialised after CAN init to reclaim them regardless of CAN result.
    Serial.print(F("[CAN] Init MCP2515... "));
    if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
        pinMode(CAN_INT, INPUT_PULLUP);
        g_canOK = true;
        Serial.println(F("OK (250 kbps)"));
    } else {
        pinMode(CAN_INT, INPUT_PULLUP);
        Serial.println(F("FAILED — check CS/INT/SPI wiring — running without CAN"));
    }
    // Re-claim SIM800L UART pins after SPI.begin() may have reconfigured them
    sim800l.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
    delay(500);

    // ── GPRS with retries ─────────────────────────────────────────────────────
    bool connected = false;
    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        Serial.printf("[GSM] Attempt %d/%d\n", attempt, MAX_RETRIES);
        if (connectGPRS() && openBearer()) { connected = true; break; }
        if (attempt < MAX_RETRIES) {
            Serial.println(F("[GSM] Retrying after hardware reset..."));
            hardResetSIM800L();
            sim800l.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
            delay(2000);
        }
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
    if (g_canOK) {
        while (!digitalRead(CAN_INT)) {
            unsigned long rxId = 0;
            uint8_t  len  = 0;
            uint8_t  buf[8] = {};

            if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
                if (rxId & 0x80000000) rxId &= 0x1FFFFFFF;   // strip extended flag
                decodeFrame(rxId, len, buf);
            }
        }
    }

    if (millis() - lastPublish >= PUBLISH_INTERVAL) {
        lastPublish = millis();

        Serial.printf("[CAN] Frames: %lu  SOC: %.1f%%  Pack: %.2f V  "
                      "Cells: %d  Temps: %d\n",
                      bms.frame_count, bms.soc, bms.pack_v,
                      bms.cell_count, bms.temp_count);

        if (!publishToCloud()) {
            Serial.println(F("[CLOUD] Publish failed — reconnecting bearer..."));
            closeBearer();

            if (g_moduleRebooted) {
                // Power brownout: SIM needs full re-initialisation time
                Serial.println(F("[GSM] Brownout recovery — hardware reset + 20 s SIM wait..."));
                hardResetSIM800L();
                sim800l.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
                delay(17000);   // 3 s already elapsed in hardReset, total ~20 s
                g_moduleRebooted = false;
            } else {
                delay(2000);
            }

            if (connectGPRS() && openBearer()) {
                if (!publishToCloud())
                    Serial.println(F("[CLOUD] Retry also failed — will retry next cycle"));
            } else {
                Serial.println(F("[GSM] Re-connect failed — will retry next cycle"));
            }
        }
    }
}
