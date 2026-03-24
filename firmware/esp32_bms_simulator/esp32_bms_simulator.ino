/*
 * O'CELL BMS CAN Frame Simulator
 * ================================
 * Simulates the exact CAN frames produced by an O'CELL IFS60.8-500
 * 19S1P LiFePO4 battery pack (60.8V / 50Ah).
 *
 * Outputs frames to USB Serial in the format:
 *   [1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 0C DF 0C DE
 *
 * Frame schedule (mirrors real BMS timing):
 *   Every 500ms : 0x98C8–CC (cell voltages), 0x98B4 (temps), 0x98FFE5, 0x98FF28, 0x98FE28
 *   Every 100ms : 0x98FF28, 0x98FE28  (pack summary repeated)
 *
 * Simulation behaviour:
 *   - Cell voltages drift slowly ±10mV around nominal 3295mV
 *   - SOC ramps from 70% down to 20% then back up (discharge/charge cycle)
 *   - Pack voltage tracks SOC linearly between 47.5V and 69.35V
 *   - Temperature rises slowly with "load", cools at rest
 *   - Current swings between charging (+) and discharging (-)
 *
 * Board  : ESP32 (any variant)
 * Baud   : 115200
 */

// ── Simulation state ──────────────────────────────────────────────────────

// 19 cells (mV) — start at healthy mid-charge values
int cellMv[19] = {
  3295, 3294, 3296, 3295,   // 1-4
  3295, 3297, 3294, 3295,   // 5-8
  3294, 3295, 3296, 3294,   // 9-12
  3295, 3296, 3295, 3294,   // 13-16
  3295, 3294, 3295           // 17-19
};

float soc       = 69.3f;   // %    — matches real log snapshot
float soh       = 98.0f;   // %
float tempC[3]  = {28.0f, 28.0f, 28.0f};  // °C
float packV     = 67.12f;  // V    — matches real log snapshot
float currentA  = 5.0f;    // A    positive = discharge, negative = charge

// Simulation direction: 1 = discharging, -1 = charging
int   dir       = 1;

// ── Helpers ───────────────────────────────────────────────────────────────

// Print one CAN frame in the logger format
void printFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
  Serial.printf("[%lums] ID: 0x%08X DLC: %d Data:", millis(), id, dlc);
  for (int i = 0; i < dlc; i++) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

// Pack big-endian uint16 into buf[offset]
void packBE16(uint8_t* buf, int offset, uint16_t val) {
  buf[offset]   = (val >> 8) & 0xFF;
  buf[offset+1] = val & 0xFF;
}

// Pack little-endian uint16 into buf[offset]
void packLE16(uint8_t* buf, int offset, uint16_t val) {
  buf[offset]   = val & 0xFF;
  buf[offset+1] = (val >> 8) & 0xFF;
}

// ── Frame builders ────────────────────────────────────────────────────────

/*
 * 0x98C8–CC28F4 — Cell voltages
 * 4 cells per frame, big-endian uint16, unit = 1 mV
 * Group C8 = cells 1-4, C9 = 5-8, CA = 9-12, CB = 13-16, CC = 17-19(+pad)
 */
void sendCellFrames() {
  uint8_t buf[8];
  for (int group = 0; group < 5; group++) {
    memset(buf, 0, 8);
    uint32_t id = 0x98C80000 | ((0x28 + group) << 16) | 0x28F4;
    // Actually: ID = 0x98 | funcCode | subAddr | srcAddr
    // funcCode for group 0 = 0xC8, group 1 = 0xC9, etc.
    id = (0x98UL << 24) | ((0xC8 + group) << 16) | (0x28 << 8) | 0xF4;

    int base = group * 4;  // 0-indexed cell base
    for (int i = 0; i < 4; i++) {
      int cellIdx = base + i;
      if (cellIdx < 19) {
        packBE16(buf, i * 2, (uint16_t)cellMv[cellIdx]);
      }
      // cells beyond 19 stay 0x0000 (pad)
    }
    printFrame(id, buf, 8);
  }
}

/*
 * 0x98CD28F4 — Unused / all zeros in this firmware
 */
void sendCDFrame() {
  uint8_t buf[8] = {0};
  uint32_t id = (0x98UL << 24) | (0xCD << 16) | (0x28 << 8) | 0xF4;
  printFrame(id, buf, 8);
}

/*
 * 0x98B428F4 — Temperatures
 * Bytes 0,1,2 = sensor 1,2,3 — encoding: raw = tempC + 40
 * Bytes 3-7 = 0x00
 */
void sendTempFrame() {
  uint8_t buf[8] = {0};
  for (int i = 0; i < 3; i++) {
    buf[i] = (uint8_t)(tempC[i] + 40.0f + 0.5f);
  }
  uint32_t id = (0x98UL << 24) | (0xB4 << 16) | (0x28 << 8) | 0xF4;
  printFrame(id, buf, 8);
}

/*
 * 0x98FFE5F4 — SOC / Charge current request
 * Bytes 0-1 LE: SOC × 10  (e.g. 693 = 69.3%)
 * Bytes 2-3 LE: Charge current request × 10 in A
 *               (BMS datasheet: 25A max charge → 250)
 * Bytes 4-7: 0x00
 */
void sendFFE5Frame() {
  uint8_t buf[8] = {0};
  uint16_t socRaw = (uint16_t)(soc * 10.0f);
  // Charge current request: 25A when SOC < 90%, reduce near full
  float chgReq = 25.0f;
  if (soc >= 95.0f) chgReq = 10.0f;
  if (soc >= 100.0f) chgReq = 0.0f;
  uint16_t chgRaw = (uint16_t)(chgReq * 10.0f);

  packLE16(buf, 0, socRaw);
  packLE16(buf, 2, chgRaw);
  uint32_t id = (0x98UL << 24) | (0xFF << 16) | (0xE5 << 8) | 0xF4;
  printFrame(id, buf, 8);
}

/*
 * 0x98FF28F4 — Pack summary (sent every 100ms)
 * Bytes 0-1 LE: Pack voltage × 100 (e.g. 6712 = 67.12V)
 * Bytes 2-3 LE: Discharge current limit × 100 in A (e.g. 5000 = 50.00A)
 * Bytes 4-5 LE: SOC × 10 (e.g. 626 = 62.6%)
 * Bytes 6-7: 0x00
 *
 * Note: bytes 4-5 SOC here is a secondary SOC field (may differ from FFE5).
 *       Using the same value for simplicity.
 */
void sendFF28Frame() {
  uint8_t buf[8] = {0};
  uint16_t vRaw    = (uint16_t)(packV * 100.0f);
  uint16_t iLimRaw = (uint16_t)(50.0f * 100.0f);   // 50A discharge limit
  uint16_t socRaw  = (uint16_t)(soc * 10.0f);

  packLE16(buf, 0, vRaw);
  packLE16(buf, 2, iLimRaw);
  packLE16(buf, 4, socRaw);
  uint32_t id = (0x98UL << 24) | (0xFF << 16) | (0x28 << 8) | 0xF4;
  printFrame(id, buf, 8);
}

/*
 * 0x98FE28F4 — Min/Max cell + temps + discharge limit (sent every 100ms)
 * Bytes 0-1 LE: Max cell voltage (mV)
 * Bytes 2-3 LE: Min cell voltage (mV)
 * Byte  4   : Temp sensor 1  (raw−40 = °C)
 * Byte  5   : Temp sensor 2  (raw−40 = °C)
 * Bytes 6-7 LE: Discharge current limit × 10 in A (e.g. 500 = 50.0A)
 */
void sendFE28Frame() {
  uint8_t buf[8] = {0};

  int maxMv = cellMv[0], minMv = cellMv[0];
  for (int i = 1; i < 19; i++) {
    if (cellMv[i] > maxMv) maxMv = cellMv[i];
    if (cellMv[i] < minMv) minMv = cellMv[i];
  }

  packLE16(buf, 0, (uint16_t)maxMv);
  packLE16(buf, 2, (uint16_t)minMv);
  buf[4] = (uint8_t)(tempC[0] + 40.0f + 0.5f);
  buf[5] = (uint8_t)(tempC[1] + 40.0f + 0.5f);
  packLE16(buf, 6, (uint16_t)(50.0f * 10.0f));  // 500 = 50.0A limit

  uint32_t id = (0x98UL << 24) | (0xFE << 16) | (0x28 << 8) | 0xF4;
  printFrame(id, buf, 8);
}

// ── Simulation update ─────────────────────────────────────────────────────

/*
 * Called every 500ms. Updates all simulation values realistically.
 */
void updateSimulation() {
  // SOC: discharge from 70% → 20%, then charge back to 90%, repeat
  float socDelta = dir * 0.15f;   // ~0.3%/s during active cycle
  soc += socDelta;
  if (soc <= 20.0f) { dir = -1; currentA = -15.0f; }  // start charging
  if (soc >= 90.0f) { dir =  1; currentA =  8.0f;  }  // start discharging

  soc = constrain(soc, 15.0f, 100.0f);

  // Pack voltage: linear between cutoff voltages
  // 2.5V*19=47.5V at 0%,  3.65V*19=69.35V at 100%
  packV = 47.5f + (soc / 100.0f) * (69.35f - 47.5f);
  packV += ((float)random(-5, 6)) * 0.01f;  // ±50mV noise

  // Cell voltages: track SOC with individual noise
  float nominalMv = 2500.0f + (soc / 100.0f) * (3650.0f - 2500.0f);
  for (int i = 0; i < 19; i++) {
    int drift = random(-8, 9);  // ±8mV per cell per cycle
    cellMv[i] = (int)(nominalMv + drift);
    cellMv[i] = constrain(cellMv[i], 2500, 3700);
  }

  // Temperature: rises with discharge load, cools when charging
  float heatRate = (dir == 1) ? 0.05f : -0.03f;
  for (int i = 0; i < 3; i++) {
    tempC[i] += heatRate + ((float)random(-10, 11)) * 0.01f;
    tempC[i] = constrain(tempC[i], 20.0f, 55.0f);
  }
}

// ── Timing ────────────────────────────────────────────────────────────────

unsigned long lastSlowMs = 0;   // 500ms cycle: all frames
unsigned long lastFastMs = 0;   // 100ms cycle: FF28 + FE28 repeats

// ── Setup & Loop ─────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("========================================");
  Serial.println("   O'CELL BMS CAN SIMULATOR - ESP32");
  Serial.println("========================================");
  randomSeed(analogRead(0));
}

void loop() {
  unsigned long now = millis();

  // ── Slow cycle (500ms): full frame set ───────────────────────────────
  if (now - lastSlowMs >= 500) {
    lastSlowMs = now;
    updateSimulation();

    sendCellFrames();   // 0x98C8-CC28F4 (5 frames)
    sendCDFrame();      // 0x98CD28F4
    sendTempFrame();    // 0x98B428F4
    sendFFE5Frame();    // 0x98FFE5F4
    sendFF28Frame();    // 0x98FF28F4
    sendFE28Frame();    // 0x98FE28F4
  }

  // ── Fast cycle (100ms): pack summary only (matches real BMS) ─────────
  if (now - lastFastMs >= 100) {
    lastFastMs = now;
    // Only send the fast frames if we're NOT in a slow cycle tick
    // (avoid duplicate at the 500ms boundary)
    if (now - lastSlowMs > 10) {
      sendFF28Frame();  // 0x98FF28F4
      sendFE28Frame();  // 0x98FE28F4
    }
  }
}
