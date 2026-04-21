/*
 * O'CELL BMS CAN Frame Simulator — WiFi TCP Edition
 * ==================================================
 * Identical 7-phase scenario as before, but frames are sent over
 * a WiFi TCP connection instead of USB Serial.
 *
 * ── HOW IT WORKS ───────────────────────────────────────────────────────────
 *  1. ESP32 connects to your WiFi network.
 *  2. It opens a TCP connection to your PC (server.py) on TCP_PORT.
 *  3. It sends the exact same text lines server.py already knows how to parse:
 *       [1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 0C DF 0C DE
 *  4. If the connection drops it retries automatically every 2 s.
 *
 * ── SETUP ──────────────────────────────────────────────────────────────────
 *  1. Fill in WIFI_SSID, WIFI_PASS, SERVER_IP below.
 *  2. SERVER_IP  = the local IP of the PC running server.py
 *                  (run `ipconfig` on Windows or `ip a` on Linux to find it)
 *  3. TCP_PORT must match --esp-port on server.py (default 9000).
 *  4. Flash, open the Serial Monitor at 115200 to watch connection status.
 *
 * Board  : ESP32 (any variant)
 * Baud   : 115200  (Serial Monitor / debug only — data goes over WiFi)
 */

#include <WiFi.h>

// ─────────────────────────────────────────────────────────────────────────────
// ★  CONFIGURE THESE  ★
// ─────────────────────────────────────────────────────────────────────────────
const char* WIFI_SSID  = "EVENT_SMU";
const char* WIFI_PASS  = "$mUeV&nt2@25";
const char* SERVER_IP  = "192.168.49.149";   // PC running server.py
const uint16_t TCP_PORT = 9000;            // must match --esp-port
// ─────────────────────────────────────────────────────────────────────────────

WiFiClient client;

// ─────────────────────────────────────────────────────────────────────────────
// Cell personalities — fixed per-cell offset in mV
// ─────────────────────────────────────────────────────────────────────────────
const int CELL_OFFSET[19] = {
   2,  0,  5, -3,   //  1-4
   1,  4, -18,  3,  //  5-8  (cell 7 = weak)
  -1,  2,  6,  0,   //  9-12
   3, -18,  1, -2,  // 13-16 (cell 14 = weak)
   4,  0,  2        // 17-19
};

// ─────────────────────────────────────────────────────────────────────────────
// Live simulation state
// ─────────────────────────────────────────────────────────────────────────────
float  soc       = 72.0f;
float  packV     = 0.0f;
float  currentA  = 0.0f;
float  tempC[3]  = {24.0f, 23.5f, 23.0f};
int    cellMv[19];

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
  40,   // 0 REST           — 20 s
  240,  // 1 BULK DISCHARGE — 120 s
  50,   // 2 LOW SOC        — 25 s
  30,   // 3 RECOVERY       — 15 s
  180,  // 4 CC CHARGE      — 90 s
  80,   // 5 CV TAPER       — 40 s
  40    // 6 FULL REST      — 20 s
};

Phase currentPhase = PHASE_REST;
int   phaseTick    = 0;

// ─────────────────────────────────────────────────────────────────────────────
// WiFi / TCP helpers
// ─────────────────────────────────────────────────────────────────────────────
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print("[WiFi] Connecting to ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("  IP: ");
  Serial.println(WiFi.localIP());
}

bool ensureTCP() {
  if (client.connected()) return true;
  client.stop();
  Serial.printf("[TCP] Connecting to %s:%d ...\n", SERVER_IP, TCP_PORT);
  if (!client.connect(SERVER_IP, TCP_PORT)) {
    Serial.println("[TCP] Connection failed — retrying in 2 s");
    delay(2000);
    return false;
  }
  Serial.println("[TCP] Connected to server.py");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame helpers
// ─────────────────────────────────────────────────────────────────────────────
void packBE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = (v >> 8) & 0xFF; buf[o+1] = v & 0xFF;
}
void packLE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = v & 0xFF; buf[o+1] = (v >> 8) & 0xFF;
}

float randNoise(float amplitude) {
  float r = ((float)random(-1000, 1001)) / 1000.0f;
  return r * amplitude;
}

float socToCellOCV(float s) {
  if (s >= 90.0f) return 3.40f + (s - 90.0f) * 0.0310f;
  if (s >= 20.0f) return 3.20f + (s - 20.0f) * 0.00286f;
  return 2.50f + (s / 20.0f) * 0.70f;
}

// printFrame — sends over TCP instead of Serial
void printFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
  if (!client.connected()) return;  // drop frame if disconnected; ensureTCP() catches it next tick

  // Build the line in a small buffer to send in one write (more efficient)
  char line[80];
  int  pos = snprintf(line, sizeof(line),
                      "[%lums] ID: 0x%08X DLC: %d Data:",
                      millis(), id, dlc);
  for (int i = 0; i < dlc && pos < (int)sizeof(line) - 4; i++)
    pos += snprintf(line + pos, sizeof(line) - pos, " %02X", data[i]);

  client.println(line);   // println adds \r\n — server.py strips it correctly
  client.flush();
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame senders (unchanged logic)
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

// ─────────────────────────────────────────────────────────────────────────────
// Phase transition
// ─────────────────────────────────────────────────────────────────────────────
void nextPhase() {
  currentPhase = (Phase)((currentPhase + 1) % PHASE_COUNT);
  if (currentPhase == PHASE_REST && phaseTick > 0)
    currentPhase = PHASE_BULK_DISCHARGE;
  phaseTick = 0;
  Serial.printf("[INFO] ── Phase → %d ──\n", currentPhase);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main simulation update (unchanged logic)
// ─────────────────────────────────────────────────────────────────────────────
void updateSimulation() {

  float dischLimA = 50.0f;
  float chgReqA   = 0.0f;
  float loadSagMv = 0.0f;

  float progress = (PHASE_TICKS[currentPhase] > 0)
                   ? (float)phaseTick / PHASE_TICKS[currentPhase]
                   : 1.0f;

  switch (currentPhase) {

    case PHASE_REST:
      currentA  = 0.0f;
      chgReqA   = 5.0f;
      dischLimA = 50.0f;
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.04f + randNoise(0.05f);
      break;

    case PHASE_BULK_DISCHARGE:
      currentA  = 38.0f + randNoise(1.5f);
      loadSagMv = 18.0f;
      chgReqA   = 0.0f;
      soc      -= 0.30f + randNoise(0.02f);
      tempC[0] += 0.08f + randNoise(0.03f);
      tempC[1] += 0.06f + randNoise(0.02f);
      tempC[2] += 0.03f + randNoise(0.02f);
      dischLimA = 50.0f;
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
      break;

    case PHASE_RECOVERY:
      currentA  = 0.0f;
      chgReqA   = 25.0f;
      dischLimA = 10.0f;
      soc      += 0.08f * (1.0f - progress);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.06f + randNoise(0.04f);
      break;

    case PHASE_CC_CHARGE:
      currentA  = -22.0f + randNoise(0.5f);
      chgReqA   = 22.0f;
      dischLimA = 50.0f;
      soc      += 0.28f + randNoise(0.02f);
      {
        float heatFactor = (progress < 0.3f) ? (1.0f - progress/0.3f * 0.5f) : 0.5f;
        tempC[0] += 0.04f * heatFactor + randNoise(0.03f);
        tempC[1] += 0.03f * heatFactor + randNoise(0.02f);
        tempC[2] += 0.01f * heatFactor + randNoise(0.02f);
      }
      break;

    case PHASE_CV_TAPER:
      currentA  = -(22.0f * (1.0f - progress) * (1.0f - progress) + 2.0f)
                  + randNoise(0.3f);
      chgReqA   = max(0.0f, 22.0f * (1.0f - progress));
      dischLimA = 50.0f;
      soc      += 0.10f * (1.0f - progress * 0.7f) + randNoise(0.01f);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.05f + randNoise(0.03f);
      break;

    case PHASE_FULL_REST:
      currentA  = 0.0f;
      chgReqA   = 0.0f;
      dischLimA = 50.0f;
      soc       = constrain(soc, 99.5f, 100.0f);
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.07f + randNoise(0.03f);
      break;

    default: break;
  }

  soc = constrain(soc, 5.0f, 100.0f);
  for (int i = 0; i < 3; i++)
    tempC[i] = constrain(tempC[i], 15.0f, 58.0f);

  float baseOCV = socToCellOCV(soc) * 1000.0f;

  for (int i = 0; i < 19; i++) {
    float offset  = (float)CELL_OFFSET[i];
    float weakSag = 0.0f;
    if ((i == 6 || i == 13) && currentA > 0)
      weakSag = loadSagMv * 0.7f;
    float loadSag = (currentA > 0) ? (currentA * 0.25f + loadSagMv) : 0.0f;
    float noise   = randNoise(2.5f);
    float mv      = baseOCV + offset - loadSag - weakSag + noise;
    cellMv[i]     = (int)constrain(mv, 2480.0f, 3720.0f);
  }

  float sumMv = 0;
  for (int i = 0; i < 19; i++) sumMv += cellMv[i];
  packV = (sumMv / 1000.0f) + randNoise(0.03f);

  phaseTick++;
  if (phaseTick >= PHASE_TICKS[currentPhase]) nextPhase();

  sendCellFrames();
  sendCDFrame();
  sendTempFrame();
  sendFFE5Frame(chgReqA);
  sendFF28Frame(dischLimA);
  sendFE28Frame(dischLimA);
}

// ─────────────────────────────────────────────────────────────────────────────
// Timing
// ─────────────────────────────────────────────────────────────────────────────
unsigned long lastSlowMs = 0;
unsigned long lastFastMs = 0;
float _fastDischLim = 50.0f;
float _fastChgReq   = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("============================================================");
  Serial.println("  O'CELL BMS CAN SIMULATOR — WiFi TCP Edition");
  Serial.println("  19S1P LiFePO4  60.8V / 50Ah  |  7-Phase Cycle");
  Serial.println("============================================================");

  ensureWiFi();
  ensureTCP();
  randomSeed(analogRead(0));

  Serial.println("[INFO] Phase 0: REST — pack idle at 72% SOC");
}

void loop() {
  // Keep WiFi and TCP alive
  ensureWiFi();
  if (!ensureTCP()) return;   // skip this loop iteration if not connected

  unsigned long now = millis();

  // Slow cycle (500 ms): full simulation update + all frames
  if (now - lastSlowMs >= 500) {
    lastSlowMs = now;
    updateSimulation();
  }

  // Fast cycle (100 ms): pack summary only
  if (now - lastFastMs >= 100) {
    lastFastMs = now;
    if (now - lastSlowMs > 10) {
      sendFF28Frame(_fastDischLim);
      sendFE28Frame(_fastDischLim);
    }
  }
}
