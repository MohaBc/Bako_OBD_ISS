/**
 * BAKO SMU — CAN Frame Sender (MCP2515 via SPI)
 * Outputs frames in the exact format of the real BMS log:
 *   [Nms] ID: 0x18FF28F4  DLC: 8 Data: 38 4A 88 13 77 02 00 00
 *
 * SPI wiring (ESP32 VSPI):
 *   CS   → GPIO 5
 *   SCK  → GPIO 18
 *   MISO → GPIO 19
 *   MOSI → GPIO 23
 *
 * Library: coryjfowler/MCP_CAN (Arduino Library Manager)
 * Change MCP_8MHZ → MCP_16MHZ if your module has a 16 MHz crystal.
 */

#include <SPI.h>
#include <mcp_can.h>

#define CS_PIN 5
MCP_CAN CAN(CS_PIN);

// ── Timing ────────────────────────────────────────────────────────────────────
static uint32_t t_100ms  = 0;
static uint32_t t_500ms  = 0;
static uint32_t t_1000ms = 0;

// ── Real payloads from the captured log (steady-state, car idling) ────────────
// 0x18FF28F4  BMS Basic Msg 1
uint8_t d_FF28[8] = {0x38, 0x4A, 0x88, 0x13, 0x77, 0x02, 0x00, 0x00};
// Decoded: status=0x38, SOC=74%, current=0.0A, voltage=63.1V, fault=0, err=0

// 0x18FE28F4  BMS Basic Msg 2
uint8_t d_FE28[8] = {0xFF, 0x0C, 0xFA, 0x0C, 0x3D, 0x3C, 0xE8, 0x03};
// Decoded: maxCell=3327mV, minCell=3322mV, maxT=21°C, minT=20°C, maxI=100A

// 0x18C828F4  Cell voltages 1–4  (big-endian)
uint8_t d_C828[8] = {0x0C, 0xFD, 0x0C, 0xFE, 0x0C, 0xFE, 0x0C, 0xFE};
// Cells: 3325, 3326, 3326, 3326 mV

// 0x18C928F4  Cell voltages 5–8
uint8_t d_C928[8] = {0x0C, 0xFE, 0x0C, 0xFE, 0x0C, 0xFD, 0x0C, 0xFD};
// Cells: 3326, 3326, 3325, 3325 mV

// 0x18CA28F4  Cell voltages 9–12
uint8_t d_CA28[8] = {0x0C, 0xFE, 0x0C, 0xFD, 0x0C, 0xFF, 0x0C, 0xFD};
// Cells: 3326, 3325, 3327, 3325 mV

// 0x18CB28F4  Cell voltages 13–16
uint8_t d_CB28[8] = {0x0C, 0xFE, 0x0C, 0xFA, 0x0C, 0xFB, 0x0C, 0xFB};
// Cells: 3326, 3322, 3323, 3323 mV

// 0x18CC28F4  Cell voltages 17–19 + padding
uint8_t d_CC28[8] = {0x0C, 0xFB, 0x0C, 0xFB, 0x0C, 0xFA, 0x00, 0x00};
// Cells: 3323, 3323, 3322 mV, last pair = 0x0000 (cell 20 doesn't exist)

// 0x18B428F4  Temperature probes 1–4 (probes 5–8 = 0xFF, not connected)
uint8_t d_B428[8] = {0x3C, 0x3C, 0x3D, 0x3D, 0xFF, 0xFF, 0xFF, 0xFF};
// Decoded: 20°C, 20°C, 21°C, 21°C

// 0x18FFE5F4  BMS charging request
uint8_t d_FFE5[8] = {0xB5, 0x02, 0x5E, 0x01, 0x00, 0x00, 0x00, 0x00};
// Decoded: maxChargeV=69.3V, maxChargeI=35.0A

// ── Send helper — sends frame and prints the exact log-format line ────────────
static void sendFrame(uint32_t id, uint8_t *data, uint8_t len = 8) {
  CAN.sendMsgBuf(id, 1, len, data);
  Serial.printf("[%lums] ID: 0x%08X  DLC: %d Data:", millis(), id, len);
  for (int i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
  Serial.println();
}

// ── Arduino setup ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(300);

  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("MCP2515 init failed, retrying...");
    delay(1000);
  }

  CAN.setMode(MCP_LOOPBACK);   // no other node needed; remove when on real bus
  Serial.println("MCP2515 OK");
}

// ── Arduino loop ──────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // 100 ms — BMS basic msg 1 & 2
  if (now - t_100ms >= 100) {
    t_100ms = now;
    sendFrame(0x18FF28F4, d_FF28);
    sendFrame(0x18FE28F4, d_FE28);
  }

  // 500 ms — cell voltages (5 frames) + temperatures
  if (now - t_500ms >= 500) {
    t_500ms = now;
    sendFrame(0x18C828F4, d_C828);
    sendFrame(0x18C928F4, d_C928);
    sendFrame(0x18CA28F4, d_CA28);
    sendFrame(0x18CB28F4, d_CB28);
    sendFrame(0x18CC28F4, d_CC28);
    sendFrame(0x18B428F4, d_B428);
  }

  // 1000 ms — charging request
  if (now - t_1000ms >= 1000) {
    t_1000ms = now;
    sendFrame(0x18FFE5F4, d_FFE5);
  }
}
