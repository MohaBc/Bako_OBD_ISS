/*
 * ============================================================================
 *  BAKO SMU — ESP32 Combined Simulator (Serial + WiFi AP simultaneously)
 *
 *  Merges:
 *    • Selima's 7-scenario test-vehicle engine (9 sensors + GNSS)
 *    • Hana's    WiFi Access-Point TCP output
 *
 *  The ESP32 boots and outputs every CAN frame and SENSOR line on BOTH
 *  channels at the same time, so the PC can choose either transport
 *  (the choice is made from the web dashboard, not the firmware).
 *
 *      Channel 1 — Serial (USB, 115200 baud)
 *      Channel 2 — WiFi AP "BAKO_SMU" (password 12345678), TCP port 9000
 *                  PC joins the AP, server connects as TCP client to
 *                  192.168.4.1:9000.
 *
 *  Board : ESP32 (any variant)   Baud : 115200
 * ============================================================================
 */

#include <WiFi.h>

// ── WiFi AP CONFIGURATION ────────────────────────────────────────────────────
const char*    AP_SSID    = "BAKO_SMU";
const char*    AP_PASS    = "12345678";   // min 8 chars (WPA2)
const uint16_t TCP_PORT   = 9000;
const uint8_t  AP_CHANNEL = 6;

WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// ── TIMING ───────────────────────────────────────────────────────────────────
#define SCENARIO_DURATION_MS  30000UL
#define TICK_MS               500UL
#define SCENARIO_COUNT        7

// ── CELL OFFSETS (mV) — cells 7 & 14 are intentionally weaker ───────────────
const int cellPersonalityOffset_mV[19] = {
   2,  0,  5, -3,
   1,  4, -18,  3,
  -1,  2,  6,  0,
   3, -18,  1, -2,
   4,  0,  2
};

// ── LIVE SENSOR STATE ────────────────────────────────────────────────────────
float batterySOC_pct       = 72.0f;
float batteryPackVoltage_V = 0.0f;
float bmsInternalTempC[3]  = {24.0f, 23.5f, 23.0f};
int   cellVoltage_mV[19];

float solarPanelVoltageV   = 0.0f;
float solarPanelCurrentA   = 0.0f;
float mpptOutputCurrentA   = 0.0f;
float dcdc_HV_InputV       = 62.5f;
float dcdc_12V_OutputV     = 12.8f;

float motorTempC           = 28.0f;
float mpptHeatSinkTempC    = 27.0f;
float cabinTempC           = 26.0f;
uint8_t handbrakeEngaged   = 1;

float gnssLatitude         = 36.8065f;   // Tunis
float gnssLongitude        = 10.1815f;
float gnssSpeedKmh         = 0.0f;
uint8_t gnssFixValid       = 1;
float gnssAltitude_m       = 15.0f;
float gnssHeading_deg      = 0.0f;

// ── SCENARIO STATE ──────────────────────────────────────────────────────────
uint8_t  currentScenario   = 0;
uint32_t scenarioStartTime = 0;
uint32_t lastTickTime      = 0;
uint32_t lastFastTickTime  = 0;
float    _lastDischLim     = 50.0f;

const char* SCENARIO_NAME[SCENARIO_COUNT] = {
  "PARKED_IDLE",
  "SOLAR_CHARGING",
  "NORMAL_DRIVING",
  "HEAVY_LOAD",
  "LOW_BATTERY_WARNING",
  "OVERHEATING",
  "HANDBRAKE_FAULT"
};

// ── UTILITIES ────────────────────────────────────────────────────────────────
float addNoise(float amp) {
  return ((float)random(0, 2001) - 1000.0f) / 1000.0f * amp;
}
float clampFloat(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
float socToCellOCV_mV(float soc) {
  if (soc >= 90.0f) return 3400.0f + (soc - 90.0f) * 31.0f;
  if (soc >= 20.0f) return 3200.0f + (soc - 20.0f) * 2.86f;
  return 2500.0f + (soc / 20.0f) * 700.0f;
}
void encodeLE16(uint8_t* b, int o, uint16_t v) { b[o]=v&0xFF; b[o+1]=(v>>8)&0xFF; }
void encodeBE16(uint8_t* b, int o, uint16_t v) { b[o]=(v>>8)&0xFF; b[o+1]=v&0xFF; }

// ── DUAL OUTPUT — print to Serial AND any connected TCP client ──────────────
void dualPrintLine(const char* line) {
  Serial.println(line);
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(line);
  }
}

void sendCANFrame(uint32_t id, const uint8_t* data, uint8_t dlc) {
  char line[96];
  int pos = snprintf(line, sizeof(line),
                     "[%lums] ID: 0x%08X DLC: %d Data:",
                     millis(), id, dlc);
  for (int i = 0; i < dlc && pos < (int)sizeof(line) - 4; i++)
    pos += snprintf(line + pos, sizeof(line) - pos, " %02X", data[i]);
  dualPrintLine(line);
}

// ── CELL VOLTAGE COMPUTATION ────────────────────────────────────────────────
void computeCellVoltages(float loadSag_mV, float weakExtra_mV) {
  float baseV = socToCellOCV_mV(batterySOC_pct);
  float sum = 0;
  for (int i = 0; i < 19; i++) {
    float mv = baseV + cellPersonalityOffset_mV[i] - loadSag_mV + addNoise(2.0f);
    if ((i == 6 || i == 13) && loadSag_mV > 0) mv -= weakExtra_mV;
    cellVoltage_mV[i] = (int)clampFloat(mv, 2480.0f, 3650.0f);
    sum += cellVoltage_mV[i];
  }
  batteryPackVoltage_V = sum / 1000.0f;
  dcdc_HV_InputV = batteryPackVoltage_V + addNoise(0.05f);
}

// ── GNSS POSITION ────────────────────────────────────────────────────────────
void updateGNSSPosition(bool moving, float kmh, float altPerMin) {
  gnssSpeedKmh = moving ? clampFloat(kmh + addNoise(3.0f), 0, 120) : 0.0f;
  gnssFixValid = 1;
  if (moving && gnssSpeedKmh > 2.0f) {
    float dDeg = (gnssSpeedKmh / 3600.0f) * (1.0f / 111.0f) * (TICK_MS / 1000.0f);
    gnssHeading_deg += 1.5f + addNoise(0.5f);
    if (gnssHeading_deg > 360.0f) gnssHeading_deg -= 360.0f;
    float r = gnssHeading_deg * 3.14159f / 180.0f;
    gnssLatitude  += dDeg * cos(r);
    gnssLongitude += dDeg * sin(r);
    gnssAltitude_m += (altPerMin / 60.0f) * (TICK_MS / 1000.0f) + addNoise(0.3f);
  } else {
    gnssAltitude_m += addNoise(0.2f);
  }
  gnssAltitude_m = clampFloat(gnssAltitude_m, 0.0f, 500.0f);
}

// ── CAN FRAME SENDERS ────────────────────────────────────────────────────────
void sendCellVoltageFrames() {
  uint8_t buf[8];
  for (int g = 0; g < 5; g++) {
    memset(buf, 0, 8);
    uint32_t id = (0x98UL<<24) | ((0xC8+g)<<16) | (0x28<<8) | 0xF4;
    for (int i = 0; i < 4; i++) {
      int idx = g*4 + i;
      if (idx < 19) encodeBE16(buf, i*2, (uint16_t)cellVoltage_mV[idx]);
    }
    sendCANFrame(id, buf, 8);
  }
}
void sendBMSTemperatureFrame() {
  uint8_t buf[8] = {0};
  for (int i = 0; i < 3; i++)
    buf[i] = (uint8_t)clampFloat(bmsInternalTempC[i] + 40.5f, 0, 255);
  sendCANFrame((0x98UL<<24)|(0xB4<<16)|(0x28<<8)|0xF4, buf, 8);
}
void sendSOCAndChargeFrame(float chgReqA) {
  uint8_t buf[8] = {0};
  encodeLE16(buf, 0, (uint16_t)(batterySOC_pct * 10.0f));
  encodeLE16(buf, 2, (uint16_t)(max(0.0f, chgReqA) * 10.0f));
  sendCANFrame((0x98UL<<24)|(0xFF<<16)|(0xE5<<8)|0xF4, buf, 8);
}
void sendPackSummaryFrame(float dischA) {
  uint8_t buf[8] = {0};
  encodeLE16(buf, 0, (uint16_t)(batteryPackVoltage_V * 100.0f));
  encodeLE16(buf, 2, (uint16_t)(dischA * 100.0f));
  encodeLE16(buf, 4, (uint16_t)(batterySOC_pct * 10.0f));
  sendCANFrame((0x98UL<<24)|(0xFF<<16)|(0x28<<8)|0xF4, buf, 8);
}
void sendCellStatsFrame(float dischA) {
  int maxMv = cellVoltage_mV[0], minMv = cellVoltage_mV[0];
  for (int i = 1; i < 19; i++) {
    if (cellVoltage_mV[i] > maxMv) maxMv = cellVoltage_mV[i];
    if (cellVoltage_mV[i] < minMv) minMv = cellVoltage_mV[i];
  }
  uint8_t buf[8] = {0};
  encodeLE16(buf, 0, (uint16_t)maxMv);
  encodeLE16(buf, 2, (uint16_t)minMv);
  buf[4] = (uint8_t)clampFloat(bmsInternalTempC[0]+40.5f, 0, 255);
  buf[5] = (uint8_t)clampFloat(bmsInternalTempC[1]+40.5f, 0, 255);
  encodeLE16(buf, 6, (uint16_t)(dischA * 10.0f));
  sendCANFrame((0x98UL<<24)|(0xFE<<16)|(0x28<<8)|0xF4, buf, 8);
}
void sendDCDCVoltageFrame() {
  uint8_t buf[8] = {0};
  encodeLE16(buf, 0, (uint16_t)(dcdc_12V_OutputV * 100.0f));
  encodeLE16(buf, 2, (uint16_t)(dcdc_HV_InputV   * 100.0f));
  sendCANFrame((0x98UL<<24)|(0xD0<<16)|(0x01<<8)|0xF4, buf, 8);
}
void sendSolarCurrentFrame() {
  uint8_t buf[8] = {0};
  int16_t pA = (int16_t)(solarPanelCurrentA * 100.0f);
  int16_t mA = (int16_t)(mpptOutputCurrentA * 100.0f);
  buf[0] = pA & 0xFF; buf[1] = (pA >> 8) & 0xFF;
  buf[2] = mA & 0xFF; buf[3] = (mA >> 8) & 0xFF;
  uint16_t pwr = (uint16_t)clampFloat(dcdc_HV_InputV * mpptOutputCurrentA * 10.0f, 0, 65535);
  encodeLE16(buf, 4, pwr);
  sendCANFrame((0x98UL<<24)|(0xD0<<16)|(0x02<<8)|0xF4, buf, 8);
}
void sendExternalTemperatureFrame() {
  uint8_t buf[8] = {0};
  buf[0] = (uint8_t)clampFloat(cabinTempC        + 40.5f, 0, 255);
  buf[1] = (uint8_t)clampFloat(motorTempC        + 40.5f, 0, 255);
  buf[2] = (uint8_t)clampFloat(mpptHeatSinkTempC + 40.5f, 0, 255);
  sendCANFrame((0x98UL<<24)|(0xD0<<16)|(0x03<<8)|0xF4, buf, 8);
}
void sendDiscreteInputFrame() {
  uint8_t buf[8] = {0};
  buf[0] = handbrakeEngaged;
  sendCANFrame((0x98UL<<24)|(0xD0<<16)|(0x04<<8)|0xF4, buf, 8);
}
void sendSensorTextLine() {
  uint32_t secLeft = (SCENARIO_DURATION_MS - (millis() - scenarioStartTime)) / 1000;
  char line[256];
  snprintf(line, sizeof(line),
    "SENSOR: solar_v=%.2f solar_i_in=%.2f solar_i_out=%.2f "
    "hv64v=%.2f aux12v=%.2f motor_t=%.1f mppt_t=%.1f cabin_t=%.1f "
    "handbrake=%d gnss_lat=%.5f gnss_lon=%.5f gnss_alt=%.1f "
    "gnss_speed=%.1f gnss_fix=%d scenario_name=%s scenario_countdown=%lu",
    solarPanelVoltageV, solarPanelCurrentA, mpptOutputCurrentA,
    dcdc_HV_InputV, dcdc_12V_OutputV,
    motorTempC, mpptHeatSinkTempC, cabinTempC,
    handbrakeEngaged,
    gnssLatitude, gnssLongitude, gnssAltitude_m,
    gnssSpeedKmh, gnssFixValid,
    SCENARIO_NAME[currentScenario], secLeft);
  dualPrintLine(line);
}

// ── SCENARIO ENGINE (Selima's 7 scenarios, complete) ────────────────────────
void runCurrentScenario() {
  float dischLimA  = 50.0f;
  float chargeReqA = 0.0f;
  float cellSag    = 0.0f;

  switch (currentScenario) {
    case 0: // PARKED_IDLE
      batterySOC_pct = clampFloat(batterySOC_pct + addNoise(0.05f), 65.0f, 75.0f);
      solarPanelVoltageV = 0; solarPanelCurrentA = 0; mpptOutputCurrentA = 0;
      dcdc_12V_OutputV  += (12.8f - dcdc_12V_OutputV) * 0.10f + addNoise(0.02f);
      motorTempC        += (25.0f - motorTempC)        * 0.05f + addNoise(0.1f);
      mpptHeatSinkTempC += (26.0f - mpptHeatSinkTempC) * 0.05f + addNoise(0.1f);
      cabinTempC        += (28.0f - cabinTempC)        * 0.04f + addNoise(0.2f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] += (24.0f-bmsInternalTempC[i])*0.05f + addNoise(0.05f);
      handbrakeEngaged = 1; chargeReqA = 2.0f;
      updateGNSSPosition(false, 0, 0);
      computeCellVoltages(0, 0);
      break;

    case 1: // SOLAR_CHARGING
      batterySOC_pct = clampFloat(batterySOC_pct + 0.20f + addNoise(0.01f), 0, 100);
      solarPanelVoltageV = clampFloat(38.0f + addNoise(1.5f), 30, 45);
      solarPanelCurrentA = clampFloat(6.5f  + addNoise(0.3f),  0,  8);
      mpptOutputCurrentA = clampFloat(solarPanelCurrentA * 0.93f + addNoise(0.1f), 0, 22);
      dcdc_12V_OutputV  += (13.2f - dcdc_12V_OutputV) * 0.05f + addNoise(0.02f);
      mpptHeatSinkTempC = clampFloat(mpptHeatSinkTempC + 0.08f + addNoise(0.04f), 20, 65);
      motorTempC        += (26.0f - motorTempC) * 0.04f + addNoise(0.1f);
      cabinTempC        += (30.0f - cabinTempC) * 0.03f + addNoise(0.2f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] = clampFloat(bmsInternalTempC[i]+0.03f+addNoise(0.02f), 15, 55);
      handbrakeEngaged = 1; chargeReqA = 22.0f;
      updateGNSSPosition(false, 0, 0);
      computeCellVoltages(0, 0);
      break;

    case 2: // NORMAL_DRIVING
      batterySOC_pct = clampFloat(batterySOC_pct - 0.18f + addNoise(0.02f), 5, 100);
      solarPanelVoltageV = clampFloat(12.0f + addNoise(2.0f), 0, 45);
      solarPanelCurrentA = clampFloat(1.2f  + addNoise(0.3f), 0,  8);
      mpptOutputCurrentA = clampFloat(solarPanelCurrentA * 0.90f, 0, 22);
      dcdc_12V_OutputV  += (12.5f - dcdc_12V_OutputV) * 0.04f + addNoise(0.03f);
      motorTempC         = clampFloat(motorTempC + 0.07f + addNoise(0.05f), 20, 90);
      mpptHeatSinkTempC  = clampFloat(mpptHeatSinkTempC + 0.03f + addNoise(0.04f), 20, 65);
      cabinTempC        += (32.0f - cabinTempC) * 0.03f + addNoise(0.2f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] = clampFloat(bmsInternalTempC[i]+0.05f+addNoise(0.03f), 15, 55);
      handbrakeEngaged = 0; cellSag = 12.0f;
      updateGNSSPosition(true, 55.0f, 1.5f);
      computeCellVoltages(cellSag, 8.0f);
      break;

    case 3: // HEAVY_LOAD
      batterySOC_pct = clampFloat(batterySOC_pct - 0.35f + addNoise(0.03f), 5, 100);
      solarPanelVoltageV = clampFloat(35.0f + addNoise(2.0f), 0, 45);
      solarPanelCurrentA = clampFloat(5.0f  + addNoise(0.4f), 0,  8);
      mpptOutputCurrentA = clampFloat(solarPanelCurrentA * 0.91f, 0, 22);
      dcdc_12V_OutputV  += (12.1f - dcdc_12V_OutputV) * 0.04f + addNoise(0.04f);
      motorTempC         = clampFloat(motorTempC + 0.18f + addNoise(0.06f), 20, 95);
      mpptHeatSinkTempC  = clampFloat(mpptHeatSinkTempC + 0.10f + addNoise(0.05f), 20, 70);
      cabinTempC        += (34.0f - cabinTempC) * 0.03f + addNoise(0.2f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] = clampFloat(bmsInternalTempC[i]+0.10f+addNoise(0.04f), 15, 58);
      handbrakeEngaged = 0; cellSag = 22.0f;
      updateGNSSPosition(true, 35.0f, 8.0f);
      computeCellVoltages(cellSag, 15.0f);
      break;

    case 4: // LOW_BATTERY_WARNING
      batterySOC_pct = clampFloat(batterySOC_pct - 0.22f + addNoise(0.02f), 3, 20);
      solarPanelVoltageV = clampFloat(30.0f + addNoise(2.0f), 0, 45);
      solarPanelCurrentA = clampFloat(3.5f  + addNoise(0.3f), 0,  8);
      mpptOutputCurrentA = clampFloat(solarPanelCurrentA * 0.90f, 0, 22);
      dcdc_12V_OutputV  += (11.8f - dcdc_12V_OutputV) * 0.05f + addNoise(0.05f);
      motorTempC        += 0.04f + addNoise(0.05f);
      mpptHeatSinkTempC += 0.02f + addNoise(0.04f);
      cabinTempC        += (30.0f - cabinTempC) * 0.04f + addNoise(0.2f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] += addNoise(0.05f);
      handbrakeEngaged = 0; cellSag = 14.0f;
      dischLimA  = clampFloat(50.0f * (batterySOC_pct / 15.0f), 8, 50);
      chargeReqA = 25.0f;
      updateGNSSPosition(true, 40.0f, -3.0f);
      computeCellVoltages(cellSag, 10.0f);
      break;

    case 5: // OVERHEATING
      batterySOC_pct = clampFloat(batterySOC_pct - 0.15f + addNoise(0.02f), 30, 70);
      solarPanelVoltageV = clampFloat(40.0f + addNoise(1.0f), 0, 45);
      solarPanelCurrentA = clampFloat(7.0f  + addNoise(0.3f), 0,  8);
      mpptOutputCurrentA = clampFloat(solarPanelCurrentA * 0.88f, 0, 22);
      dcdc_12V_OutputV  += (12.3f - dcdc_12V_OutputV) * 0.04f + addNoise(0.03f);
      motorTempC         = clampFloat(motorTempC + 0.25f + addNoise(0.07f), 20, 95);
      mpptHeatSinkTempC  = clampFloat(mpptHeatSinkTempC + 0.20f + addNoise(0.06f), 20, 72);
      cabinTempC        += (40.0f - cabinTempC) * 0.04f + addNoise(0.3f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] = clampFloat(bmsInternalTempC[i]+0.12f+addNoise(0.04f), 15, 58);
      handbrakeEngaged = 0; cellSag = 16.0f;
      dischLimA = clampFloat(50.0f - max(0.0f, motorTempC - 80.0f) * 5.0f, 5, 50);
      updateGNSSPosition(true, 28.0f, 5.0f);
      computeCellVoltages(cellSag, 12.0f);
      break;

    case 6: // HANDBRAKE_FAULT
      batterySOC_pct = clampFloat(batterySOC_pct - 0.20f + addNoise(0.02f), 20, 80);
      solarPanelVoltageV = clampFloat(20.0f + addNoise(3.0f), 0, 45);
      solarPanelCurrentA = clampFloat(2.0f  + addNoise(0.3f), 0,  8);
      mpptOutputCurrentA = clampFloat(solarPanelCurrentA * 0.90f, 0, 22);
      dcdc_12V_OutputV  += (12.4f - dcdc_12V_OutputV) * 0.04f + addNoise(0.03f);
      motorTempC        += 0.06f + addNoise(0.05f);
      mpptHeatSinkTempC += 0.04f + addNoise(0.04f);
      cabinTempC        += (31.0f - cabinTempC) * 0.03f + addNoise(0.2f);
      for (int i=0;i<3;i++) bmsInternalTempC[i] += (28.0f-bmsInternalTempC[i])*0.03f + addNoise(0.03f);
      handbrakeEngaged = 1;   // FAULT: engaged while moving
      cellSag = 13.0f;
      updateGNSSPosition(true, 45.0f, 0.5f);
      computeCellVoltages(cellSag, 8.0f);
      break;
  }

  // Safety clamps
  batterySOC_pct     = clampFloat(batterySOC_pct,      0,   100);
  dcdc_12V_OutputV   = clampFloat(dcdc_12V_OutputV,   10,   14.5f);
  dcdc_HV_InputV     = clampFloat(dcdc_HV_InputV,     50,   68);
  solarPanelVoltageV = clampFloat(solarPanelVoltageV,  0,   45);
  solarPanelCurrentA = clampFloat(solarPanelCurrentA,  0,    8);
  mpptOutputCurrentA = clampFloat(mpptOutputCurrentA,  0,   22);
  motorTempC         = clampFloat(motorTempC,         15,   95);
  mpptHeatSinkTempC  = clampFloat(mpptHeatSinkTempC,  15,   72);
  cabinTempC         = clampFloat(cabinTempC,         15,   50);
  for (int i=0;i<3;i++) bmsInternalTempC[i] = clampFloat(bmsInternalTempC[i], 15, 58);

  _lastDischLim = dischLimA;

  sendCellVoltageFrames();
  sendBMSTemperatureFrame();
  sendSOCAndChargeFrame(chargeReqA);
  sendPackSummaryFrame(dischLimA);
  sendCellStatsFrame(dischLimA);
  sendDCDCVoltageFrame();
  sendSolarCurrentFrame();
  sendExternalTemperatureFrame();
  sendDiscreteInputFrame();
  sendSensorTextLine();
}

// ── WIFI AP HELPERS ──────────────────────────────────────────────────────────
void startAP() {
  WiFi.mode(WIFI_AP);
  bool ok = (strlen(AP_PASS) >= 8)
            ? WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL)
            : WiFi.softAP(AP_SSID, nullptr, AP_CHANNEL);
  if (ok) {
    Serial.printf("[AP] SSID: %s  IP: %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[AP] softAP() failed — check password (min 8 chars)");
  }
}

void maintainTCPClient() {
  if (tcpClient && tcpClient.connected()) return;
  tcpClient.stop();
  WiFiClient incoming = tcpServer.accept();
  if (!incoming) return;
  tcpClient = incoming;
  Serial.printf("[TCP] Client connected from %s\n",
                tcpClient.remoteIP().toString().c_str());
}

// ── SETUP / LOOP ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("============================================================");
  Serial.println("  BAKO SMU — Combined Simulator (Serial + WiFi AP)");
  Serial.println("  7 scenarios | 9 sensors | 19 cells | GNSS");
  Serial.println("============================================================");

  startAP();
  tcpServer.begin();
  Serial.printf("[TCP] Listening on port %d — server.py can connect anytime\n", TCP_PORT);

  randomSeed(analogRead(0));
  scenarioStartTime = millis();
  lastTickTime      = millis();
  lastFastTickTime  = millis();

  Serial.printf("[INFO] Starting scenario 0: %s\n", SCENARIO_NAME[0]);
}

void loop() {
  unsigned long now = millis();

  maintainTCPClient();

  // 500 ms tick: full scenario update
  if (now - lastTickTime >= TICK_MS) {
    lastTickTime = now;

    if (now - scenarioStartTime >= SCENARIO_DURATION_MS) {
      currentScenario = (currentScenario + 1) % SCENARIO_COUNT;
      scenarioStartTime = now;
      if (currentScenario == 0 || currentScenario == 1) {
        motorTempC = 30.0f; mpptHeatSinkTempC = 28.0f; cabinTempC = 28.0f;
        for (int i=0;i<3;i++) bmsInternalTempC[i] = 24.0f - i * 0.5f;
      }
      if (currentScenario == 4) batterySOC_pct = 18.0f;
      Serial.printf("\n== SCENARIO %d: %s ==\n",
                    currentScenario, SCENARIO_NAME[currentScenario]);
    }

    runCurrentScenario();
  }

  // 100 ms fast tick: pack summary + cell stats (between full updates)
  if (now - lastFastTickTime >= 100UL && now - lastTickTime > 10) {
    lastFastTickTime = now;
    sendPackSummaryFrame(_lastDischLim);
    sendCellStatsFrame(_lastDischLim);
  }
}
