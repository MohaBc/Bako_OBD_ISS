/*
 * O'CELL BMS CAN Frame Simulator — Detailed Scenario Edition
 * ===========================================================
 * Simulates a realistic multi-phase charge/discharge/thermal scenario
 * for an O'CELL IFS60.8-500-F-E3  19S1P LiFePO4  60.8V / 50Ah pack.
 *
 * Outputs frames to USB Serial in the format expected by server.py:
 *   [1849ms] ID: 0x98C828F4 DLC: 8 Data: 0C DE 0C E1 0C DF 0C DE
 *
 * ── SCENARIO PHASES ────────────────────────────────────────────────────────
 *
 *  Phase 0 — REST (20 s)
 *    Pack idle at ~72% SOC. Cells balanced, temps at ambient (~24°C).
 *    Tiny voltage micro-drift. Discharge limit full, charge request low.
 *
 *  Phase 1 — BULK DISCHARGE (120 s)
 *    Heavy load: 38A discharge. SOC falls fast (~72% → ~30%).
 *    Temperatures climb steadily. Cells develop spread as weak cells sag.
 *    Cell 7 and 14 run ~15mV lower (simulated weak cells).
 *    Discharge limit starts reducing below 30% SOC.
 *
 *  Phase 2 — LOW SOC WARNING (25 s)
 *    SOC 30% → 18%. Current forced down to 12A (BMS protection).
 *    Temps peak. Cell 7 approaches low-voltage threshold (≈2520mV).
 *    Discharge limit drops sharply. Charge request rises to 25A.
 *
 *  Phase 3 — REST / RECOVERY (15 s)
 *    Load removed. Cells recover voltage (surface-charge bounce-back).
 *    SOC stabilises. Temps plateau then start falling slowly.
 *
 *  Phase 4 — CC CHARGE (90 s)
 *    Constant-current charge at 22A. SOC climbs ~18% → ~80%.
 *    Temps rise slightly then stabilise. Cells re-balance gradually.
 *    Charge request stays at 22A then tapers above 85%.
 *
 *  Phase 5 — CV / TAPER (40 s)
 *    Pack voltage near full (≈69V). Current tapers 22A → 4A.
 *    Cells balance: spread narrows. Temps cool slowly.
 *    Charge request ramps down to 0A at 100%.
 *
 *  Phase 6 — FULL / REST (20 s)
 *    SOC 100%. Zero charge current. Cells tightly balanced (≤8mV spread).
 *    Temps back near ambient. Discharge limit restored to 50A.
 *    → loops back to Phase 1.
 *
 * ── CELL PERSONALITY ───────────────────────────────────────────────────────
 *   Each of the 19 cells has a fixed offset (−20 … +12 mV) so the spread
 *   looks realistic and stable rather than random every frame.
 *   Cells 7 & 14 are "weak" (−18 mV offset) and sag faster under load.
 *
 * ── THERMAL MODEL ──────────────────────────────────────────────────────────
 *   Three sensors at different pack locations:
 *     Sensor 1 (centre): warmest, tracks load most closely.
 *     Sensor 2 (end):    lags centre by ~2°C.
 *     Sensor 3 (BMS):    coolest, ambient-biased.
 *
 * Board  : ESP32 (any variant)
 * Baud   : 115200
 */

// ─────────────────────────────────────────────────────────────────────────────
// Cell personalities — fixed per-cell offset in mV
// ─────────────────────────────────────────────────────────────────────────────
const int CELL_OFFSET[19] = {
   2,  0,  5, -3,   //  1-4
   1,  4, -18,  3,  //  5-8  (cell 7 = weak)
  -1,  2,  6,  0,   //  9-12
   3, -18,  1, -2,  // 13-16 (cell 14 = weak)
   4,  0,  2          // 17-19
};

// ─────────────────────────────────────────────────────────────────────────────
// Live simulation state
// ─────────────────────────────────────────────────────────────────────────────
float  soc       = 72.0f;    // %
float  packV     = 0.0f;     // V   (derived)
float  currentA  = 0.0f;     // A   +discharge / −charge
float  tempC[3]  = {24.0f, 23.5f, 23.0f};  // °C — sensors 1,2,3
int    cellMv[19];           // computed each tick

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

// Duration of each phase in simulation ticks (1 tick = 500 ms)
const int PHASE_TICKS[PHASE_COUNT] = {
  40,   // 0 REST          — 20 s
  240,  // 1 BULK DISCHARGE — 120 s
  50,   // 2 LOW SOC        — 25 s
  30,   // 3 RECOVERY       — 15 s
  180,  // 4 CC CHARGE      — 90 s
  80,   // 5 CV TAPER       — 40 s
  40    // 6 FULL REST       — 20 s
};

Phase currentPhase = PHASE_REST;
int   phaseTick    = 0;   // tick counter within current phase

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
void packBE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = (v >> 8) & 0xFF; buf[o+1] = v & 0xFF;
}
void packLE16(uint8_t* buf, int o, uint16_t v) {
  buf[o] = v & 0xFF; buf[o+1] = (v >> 8) & 0xFF;
}

// Gaussian-ish noise using two uniform samples (Box-Muller lite)
float randNoise(float amplitude) {
  float r = ((float)random(-1000, 1001)) / 1000.0f;
  return r * amplitude;
}

// Map SOC → nominal open-circuit cell voltage for LiFePO4
// Flat plateau 20-90%, steep ends
float socToCellOCV(float s) {
  if (s >= 90.0f) return 3.40f + (s - 90.0f) * 0.0310f;   // 3.40→3.65 (90-100%)
  if (s >= 20.0f) return 3.20f + (s - 20.0f) * 0.00286f;  // 3.20→3.40 (20-90%) flat plateau
  return 2.50f  + (s / 20.0f) * 0.70f;                    // 2.50→3.20 (0-20%)
}

void printFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
  Serial.printf("[%lums] ID: 0x%08X DLC: %d Data:", millis(), id, dlc);
  for (int i = 0; i < dlc; i++) Serial.printf(" %02X", data[i]);
  Serial.println();
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame senders (unchanged from original protocol)
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
  // Skip phase 0 (REST) when looping — go straight to discharge
  if (currentPhase == PHASE_REST && phaseTick > 0) {
    currentPhase = PHASE_BULK_DISCHARGE;
  }
  phaseTick = 0;
  Serial.printf("[INFO] ── Phase → %d ──\n", currentPhase);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main simulation update — called every 500 ms
// ─────────────────────────────────────────────────────────────────────────────
void updateSimulation() {

  // ── Phase-specific state machine ─────────────────────────────────────────

  float dischLimA = 50.0f;  // BMS discharge current limit (default)
  float chgReqA   = 0.0f;   // BMS charge current request
  float loadSagMv = 0.0f;   // extra cell sag under current load (mV)

  float progress = (PHASE_TICKS[currentPhase] > 0)
                   ? (float)phaseTick / PHASE_TICKS[currentPhase]
                   : 1.0f;

  switch (currentPhase) {

    // ── 0 REST ──────────────────────────────────────────────────────────────
    case PHASE_REST:
      currentA  = 0.0f;
      chgReqA   = 5.0f;
      dischLimA = 50.0f;
      // Gentle thermal settling toward ambient
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.04f + randNoise(0.05f);
      break;

    // ── 1 BULK DISCHARGE ────────────────────────────────────────────────────
    case PHASE_BULK_DISCHARGE:
      currentA  = 38.0f + randNoise(1.5f);        // ~38A load
      loadSagMv = 18.0f;                           // under-load sag
      chgReqA   = 0.0f;
      // SOC drops at ~0.30%/tick (≈0.6%/s)
      soc -= 0.30f + randNoise(0.02f);
      // Temps rise: centre hottest
      tempC[0] += 0.08f + randNoise(0.03f);
      tempC[1] += 0.06f + randNoise(0.02f);
      tempC[2] += 0.03f + randNoise(0.02f);
      // Discharge limit unchanged until low SOC
      dischLimA = 50.0f;
      break;

    // ── 2 LOW SOC WARNING ───────────────────────────────────────────────────
    case PHASE_LOW_SOC:
      // BMS throttles current; linearly ramp down 38A → 12A
      currentA  = 38.0f - progress * 26.0f + randNoise(0.8f);
      loadSagMv = 10.0f * (1.0f - progress);
      chgReqA   = 25.0f;   // BMS asking to charge ASAP
      soc      -= 0.22f + randNoise(0.02f);
      // Discharge limit ramps down 50A → 15A
      dischLimA = 50.0f - progress * 35.0f;
      // Temps plateau then tick up slightly
      tempC[0] += 0.02f + randNoise(0.03f);
      tempC[1] += 0.01f + randNoise(0.02f);
      tempC[2] += randNoise(0.02f);
      break;

    // ── 3 RECOVERY / REST ───────────────────────────────────────────────────
    case PHASE_RECOVERY:
      currentA  = 0.0f;
      chgReqA   = 25.0f;
      dischLimA = 10.0f;   // BMS keeps limit low while recovering
      // Voltage bounce-back: SOC appears to tick up slightly (polarisation)
      soc      += 0.08f * (1.0f - progress);
      // Temps begin falling
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.06f + randNoise(0.04f);
      break;

    // ── 4 CC CHARGE ─────────────────────────────────────────────────────────
    case PHASE_CC_CHARGE:
      currentA  = -22.0f + randNoise(0.5f);   // negative = charging
      chgReqA   = 22.0f;
      dischLimA = 50.0f;
      // SOC climbs ~0.28%/tick
      soc      += 0.28f + randNoise(0.02f);
      // Mild thermal rise at start, plateaus mid-charge
      {
        float heatFactor = (progress < 0.3f) ? (1.0f - progress/0.3f * 0.5f) : 0.5f;
        tempC[0] += 0.04f * heatFactor + randNoise(0.03f);
        tempC[1] += 0.03f * heatFactor + randNoise(0.02f);
        tempC[2] += 0.01f * heatFactor + randNoise(0.02f);
      }
      break;

    // ── 5 CV TAPER ──────────────────────────────────────────────────────────
    case PHASE_CV_TAPER:
      // Current tapers from 22A → 2A exponentially
      currentA  = -(22.0f * (1.0f - progress) * (1.0f - progress) + 2.0f)
                  + randNoise(0.3f);
      chgReqA   = max(0.0f, 22.0f * (1.0f - progress));
      dischLimA = 50.0f;
      // SOC creeps up 80% → 100%
      soc      += 0.10f * (1.0f - progress * 0.7f) + randNoise(0.01f);
      // Temps drift back toward ambient
      for (int i = 0; i < 3; i++)
        tempC[i] += (23.0f - tempC[i]) * 0.05f + randNoise(0.03f);
      break;

    // ── 6 FULL REST ─────────────────────────────────────────────────────────
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

  // ── Clamp SOC & temps ────────────────────────────────────────────────────
  soc = constrain(soc, 5.0f, 100.0f);
  for (int i = 0; i < 3; i++)
    tempC[i] = constrain(tempC[i], 15.0f, 58.0f);

  // ── Cell voltages ─────────────────────────────────────────────────────────
  // Base OCV from SOC
  float baseOCV = socToCellOCV(soc) * 1000.0f;  // → mV

  for (int i = 0; i < 19; i++) {
    // Per-cell fixed personality offset
    float offset = (float)CELL_OFFSET[i];

    // Weak-cell extra sag under discharge load (cells 6 & 13, 0-indexed)
    float weakSag = 0.0f;
    if ((i == 6 || i == 13) && currentA > 0)
      weakSag = loadSagMv * 0.7f;

    // General load sag proportional to current
    float loadSag = (currentA > 0) ? (currentA * 0.25f + loadSagMv) : 0.0f;

    // Small per-frame noise
    float noise = randNoise(2.5f);

    float mv = baseOCV + offset - loadSag - weakSag + noise;
    cellMv[i] = (int)constrain(mv, 2480.0f, 3720.0f);
  }

  // ── Pack voltage: sum of average cell OCV + noise ─────────────────────────
  float sumMv = 0;
  for (int i = 0; i < 19; i++) sumMv += cellMv[i];
  packV = (sumMv / 1000.0f) + randNoise(0.03f);

  // ── Phase advance ─────────────────────────────────────────────────────────
  phaseTick++;
  if (phaseTick >= PHASE_TICKS[currentPhase]) nextPhase();

  // ── Send all frames ───────────────────────────────────────────────────────
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

// Cached limits for the fast cycle (updated on slow tick)
float _fastDischLim = 50.0f;
float _fastChgReq   = 0.0f;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("============================================================");
  Serial.println("  O'CELL BMS CAN SIMULATOR — DETAILED SCENARIO EDITION");
  Serial.println("  19S1P LiFePO4  60.8V / 50Ah  |  7-Phase Cycle");
  Serial.println("============================================================");
  Serial.println("[INFO] Phase 0: REST — pack idle at 72% SOC");
  randomSeed(analogRead(0));
}

void loop() {
  unsigned long now = millis();

  // ── Slow cycle (500ms): full simulation update + all frames ─────────────
  if (now - lastSlowMs >= 500) {
    lastSlowMs = now;
    updateSimulation();
  }

  // ── Fast cycle (100ms): pack summary only ───────────────────────────────
  if (now - lastFastMs >= 100) {
    lastFastMs = now;
    if (now - lastSlowMs > 10) {
      sendFF28Frame(_fastDischLim);
      sendFE28Frame(_fastDischLim);
    }
  }
}
