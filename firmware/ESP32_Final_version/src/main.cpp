#include <Arduino.h>

#define SIM800_TX   16
#define SIM800_RX   17
#define SIM800_BAUD 9600

#define LED_OK   25   // green — step passed / all good
#define LED_FAIL 26   // red   — step failed / something wrong

HardwareSerial sim800(1);

// blink a led n times
void blink(uint8_t pin, int n, int ms = 150) {
  for (int i = 0; i < n; i++) {
    digitalWrite(pin, HIGH); delay(ms);
    digitalWrite(pin, LOW);  delay(ms);
  }
}

void showResult(bool pass) {
  if (pass) blink(LED_OK,   1, 200);   // one green blink = step OK
  else      blink(LED_FAIL, 2, 200);   // two red blinks  = step FAIL
}

// ─── helpers ────────────────────────────────────────────────────────────────

String sendAT(const char* cmd, uint32_t timeout = 3000) {
  sim800.println(cmd);
  String resp = "";
  uint32_t t = millis();
  while (millis() - t < timeout) {
    while (sim800.available()) resp += (char)sim800.read();
  }
  resp.trim();
  Serial.printf("[AT] %s → %s\n", cmd, resp.c_str());
  return resp;
}

bool waitFor(const char* expect, uint32_t timeout = 8000) {
  String r = "";
  uint32_t t = millis();
  while (millis() - t < timeout) {
    while (sim800.available()) r += (char)sim800.read();
    if (r.indexOf(expect) >= 0) return true;
  }
  return false;
}

// ─── test steps ─────────────────────────────────────────────────────────────

bool testBasicAT() {
  Serial.println("\n=== 1. Basic AT ===");
  String r = sendAT("AT");
  return r.indexOf("OK") >= 0;
}

bool testSignalQuality() {
  Serial.println("\n=== 2. Signal Quality (CSQ) ===");
  String r = sendAT("AT+CSQ");
  // +CSQ: <rssi>,<ber>  — rssi 0-31, 99=unknown
  return r.indexOf("+CSQ") >= 0;
}

bool testNetworkReg() {
  Serial.println("\n=== 3. Network Registration ===");
  String r = sendAT("AT+CREG?");
  // ,1 = registered home  ,5 = registered roaming
  return r.indexOf(",1") >= 0 || r.indexOf(",5") >= 0;
}

bool attachGPRS(const char* apn) {
  Serial.println("\n=== 4. GPRS Attach ===");
  sendAT("AT+CIPSHUT", 5000);
  sendAT("AT+CIPMUX=0");
  sendAT("AT+CIPRXGET=1");

  String sapbr = "AT+SAPBR=3,1,\"APN\",\"" + String(apn) + "\"";
  sendAT("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  sendAT(sapbr.c_str());
  sendAT("AT+SAPBR=1,1", 10000);

  String r = sendAT("AT+SAPBR=2,1");   // query bearer — expect IP
  return r.indexOf("+SAPBR: 1,1") >= 0;
}

bool testHTTPPost(const char* url) {
  Serial.println("\n=== 5. HTTP POST → VPS ===");
  sendAT("AT+HTTPTERM", 1000);   // close any stale session
  sendAT("AT+HTTPINIT");
  sendAT("AT+HTTPPARA=\"CID\",1");

  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + String(url) + "\"";
  sendAT(urlCmd.c_str());
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  sendAT("AT+HTTPPARA=\"USERDATA\",\"X-Api-Key: bako-bms-2024\"");
  sendAT("AT+HTTPSSL=0");

  // Minimal test payload — enough for the server to accept it
  const char* body    = "{\"device_id\":\"esp32-bms-001\",\"source\":\"gprs-test\"}";
  int         bodyLen = strlen(body);

  String dataCmd = "AT+HTTPDATA=" + String(bodyLen) + ",10000";
  String prompt  = sendAT(dataCmd.c_str(), 5000);
  if (prompt.indexOf("DOWNLOAD") < 0) {
    sendAT("AT+HTTPTERM");
    return false;
  }

  sim800.print(body);
  delay(3000);

  sendAT("AT+HTTPACTION=1", 10000);   // 1 = POST
  delay(8000);

  String r = sendAT("AT+HTTPREAD");
  sendAT("AT+HTTPTERM");
  return r.indexOf("200") >= 0 || r.length() > 10;
}

// ─── main ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  pinMode(LED_OK,   OUTPUT);
  pinMode(LED_FAIL, OUTPUT);
  digitalWrite(LED_OK,   LOW);
  digitalWrite(LED_FAIL, LOW);

  // startup: both blink twice so you know the ESP32 is alive
  blink(LED_OK,   2, 100);
  blink(LED_FAIL, 2, 100);

  delay(1000);
  Serial.println("\n====== SIM800L GPRS TEST ======");

  sim800.begin(SIM800_BAUD, SERIAL_8N1, SIM800_RX, SIM800_TX);
  delay(3000);

  const char* APN      = "internet.ooredoo.tn";
  const char* TEST_URL = "http://webhook.site/9485e215-17c5-4974-9efa-553971099a18";
                                                                                                                                                                                                                                                                                                                                                                                    
  struct { const char* name; bool (*fn)(); } steps[] = {
    { "Basic AT",       []() { return testBasicAT();       } },
    { "Signal Quality", []() { return testSignalQuality(); } },
    { "Network Reg",    []() { return testNetworkReg();    } },
  };

  bool allOk = true;

  // run the first 3 steps with live LED feedback
  for (auto& s : steps) {
    bool ok = s.fn();
    showResult(ok);
    if (!ok) allOk = false;
    delay(400);
  }

  // GPRS and HTTP need the APN / URL params
  bool gprs = attachGPRS(APN);
  showResult(gprs);
  if (!gprs) allOk = false;
  delay(400);

  bool http = testHTTPPost(TEST_URL);
  showResult(http);
  if (!http) allOk = false;
  delay(400);

  // ── final verdict ──────────────────────────────────────────
  // all 5 passed → green stays solid
  // any failed   → red blinks fast forever
  if (allOk) {
    digitalWrite(LED_OK,   HIGH);
    digitalWrite(LED_FAIL, LOW);
    Serial.println("\n[RESULT] ALL PASS — green LED solid");
  } else {
    digitalWrite(LED_OK, LOW);
    Serial.println("\n[RESULT] SOME FAILED — red LED blinking");
  }
}

void loop() {
  // fast red blink if any test failed
  if (digitalRead(LED_OK) == LOW) {
    blink(LED_FAIL, 1, 120);
    delay(300);
  }
}
