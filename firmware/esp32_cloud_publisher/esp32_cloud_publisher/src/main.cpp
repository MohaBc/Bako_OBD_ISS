#include <Arduino.h>
#include <SoftwareSerial.h>

// SIM800L pins
#define SIM800_RX  16
#define SIM800_TX  17
#define SIM800_RST 5

SoftwareSerial sim800(SIM800_RX, SIM800_TX);

// APN settings — change to your carrier
const char APN[]  = "internet";
const char USER[] = "";
const char PASS[] = "";

// ─────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────
bool sendAT(const char* cmd, const char* expected = "OK", uint16_t timeout = 3000) {
  sim800.println(cmd);
  Serial.print(">> "); Serial.println(cmd);

  long deadline = millis() + timeout;
  String resp = "";

  while (millis() < deadline) {
    while (sim800.available()) {
      char c = sim800.read();
      resp += c;
    }
    if (resp.indexOf(expected) != -1) {
      Serial.print("<< "); Serial.println(resp);
      return true;
    }
  }
  Serial.print("TIMEOUT/FAIL: "); Serial.println(resp);
  return false;
}

String sendATResponse(const char* cmd, uint16_t timeout = 3000) {
  sim800.println(cmd);
  Serial.print(">> "); Serial.println(cmd);

  long deadline = millis() + timeout;
  String resp = "";

  while (millis() < deadline) {
    while (sim800.available()) {
      char c = sim800.read();
      resp += c;
    }
  }
  Serial.print("<< "); Serial.println(resp);
  return resp;
}

// ─────────────────────────────────────────
// Wait for SIM to be ready
// ─────────────────────────────────────────
bool waitForSIM(uint16_t timeout = 15000) {
  Serial.println("=== Waiting for SIM ===");
  long deadline = millis() + timeout;

  while (millis() < deadline) {
    sim800.println("AT+CPIN?");
    delay(500);

    String resp = "";
    long t = millis() + 1000;
    while (millis() < t) {
      while (sim800.available()) resp += (char)sim800.read();
    }

    Serial.print("CPIN: "); Serial.println(resp);

    if (resp.indexOf("READY") != -1) {
      Serial.println(">>> SIM is READY!");
      return true;
    } else if (resp.indexOf("SIM PIN") != -1) {
      Serial.println(">>> SIM requires PIN — unlock it first.");
      return false;
    } else if (resp.indexOf("SIM not inserted") != -1) {
      Serial.println(">>> SIM not detected, retrying...");
    }

    delay(2000);
  }

  Serial.println(">>> SIM timeout — check insertion.");
  return false;
}

// ─────────────────────────────────────────
// Wait for network registration
// ─────────────────────────────────────────
bool waitForNetwork(uint16_t timeout = 30000) {
  Serial.println("=== Waiting for Network ===");
  long deadline = millis() + timeout;

  while (millis() < deadline) {
    sim800.println("AT+CREG?");
    delay(500);

    String resp = "";
    long t = millis() + 1000;
    while (millis() < t) {
      while (sim800.available()) resp += (char)sim800.read();
    }

    Serial.print("CREG: "); Serial.println(resp);

    // 0,1 = registered home | 0,5 = roaming
    if (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1) {
      Serial.println(">>> Network registered!");
      return true;
    }

    delay(3000);
  }

  Serial.println(">>> Network timeout — check SIM/signal.");
  return false;
}

// ─────────────────────────────────────────
// Init SIM800L
// ─────────────────────────────────────────
void initSIM800() {
  Serial.println("=== Initialising SIM800L ===");

  // Hardware reset
  pinMode(SIM800_RST, OUTPUT);
  digitalWrite(SIM800_RST, LOW);
  delay(100);
  digitalWrite(SIM800_RST, HIGH);
  delay(3000);

  sendAT("AT");           // basic check
  sendAT("ATE0");         // echo off
  sendAT("AT+CMEE=2");    // verbose errors
  sendAT("AT+CSQ");       // signal quality
}

// ─────────────────────────────────────────
// Open GPRS
// ─────────────────────────────────────────
bool openGPRS() {
  Serial.println("=== Opening GPRS ===");

  // Attach GPRS
  if (!sendAT("AT+CGATT=1", "OK", 10000)) {
    Serial.println("GPRS attach failed!");
    return false;
  }

  sendAT("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");

  char apnCmd[64];
  snprintf(apnCmd, sizeof(apnCmd), "AT+SAPBR=3,1,\"APN\",\"%s\"", APN);
  sendAT(apnCmd);

  if (*USER) {
    char userCmd[64];
    snprintf(userCmd, sizeof(userCmd), "AT+SAPBR=3,1,\"USER\",\"%s\"", USER);
    sendAT(userCmd);
  }
  if (*PASS) {
    char passCmd[64];
    snprintf(passCmd, sizeof(passCmd), "AT+SAPBR=3,1,\"PWD\",\"%s\"", PASS);
    sendAT(passCmd);
  }

  // Open bearer
  if (!sendAT("AT+SAPBR=1,1", "OK", 10000)) {
    Serial.println("Failed to open bearer!");
    return false;
  }

  // Print assigned IP
  String ip = sendATResponse("AT+SAPBR=2,1", 3000);
  Serial.print("IP: "); Serial.println(ip);
  return true;
}

// ─────────────────────────────────────────
// HTTP GET
// ─────────────────────────────────────────
void httpGet(const char* url) {
  Serial.println("=== HTTP GET ===");

  sendAT("AT+HTTPINIT");
  sendAT("AT+HTTPPARA=\"CID\",1");

  char urlCmd[128];
  snprintf(urlCmd, sizeof(urlCmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
  sendAT(urlCmd);

  if (!sendAT("AT+HTTPACTION=0", "+HTTPACTION", 10000)) {
    Serial.println("HTTP GET failed!");
  } else {
    String data = sendATResponse("AT+HTTPREAD", 5000);
    Serial.println("=== Response ===");
    Serial.println(data);
  }

  sendAT("AT+HTTPTERM");
}

// ─────────────────────────────────────────
// Close GPRS
// ─────────────────────────────────────────
void closeGPRS() {
  sendAT("AT+HTTPTERM");   // close HTTP if still open
  sendAT("AT+SAPBR=0,1"); // close bearer
  sendAT("AT+CGATT=0");   // detach GPRS
  Serial.println("GPRS closed.");
}

// ─────────────────────────────────────────
// SIM Compatibility Diagnostic
// ─────────────────────────────────────────
void runDiagnostics() {
  Serial.println("\n\n╔═════════════════════════════════════════╗");
  Serial.println("║  SIM800L SIM CARD COMPATIBILITY TEST     ║");
  Serial.println("╚═════════════════════════════════════════╝\n");

  // Test 1: Check if modem is responding
  Serial.println("[TEST 1] Modem Response...");
  if (sendAT("AT", "OK", 2000)) {
    Serial.println("✓ Modem is responding\n");
  } else {
    Serial.println("✗ Modem not responding. Check wiring and power.\n");
    return;
  }

  // Test 2: Get modem info
  Serial.println("[TEST 2] Modem Information...");
  sendAT("ATI", "OK", 2000);
  sendAT("AT+GMM", "OK", 2000);
  sendAT("AT+CGMM", "OK", 2000);

  // Test 3: Check SIM status
  Serial.println("[TEST 3] SIM Card Status (AT+CPIN)...");
  String cpinResp = sendATResponse("AT+CPIN?", 2000);
  if (cpinResp.indexOf("READY") != -1) {
    Serial.println("✓ SIM is READY\n");
  } else if (cpinResp.indexOf("SIM PIN") != -1) {
    Serial.println("✗ SIM is PIN-locked. Need to unlock first.\n");
  } else if (cpinResp.indexOf("SIM PUK") != -1) {
    Serial.println("✗ SIM is PUK-locked. Contact carrier.\n");
  } else if (cpinResp.indexOf("SIM wrong") != -1) {
    Serial.println("✗ SIM WRONG - SIM may not be compatible with this modem.\n");
    Serial.println("  Possible causes:");
    Serial.println("  1. SIM card is not compatible with SIM800L");
    Serial.println("  2. SIM card needs to be activated by carrier");
    Serial.println("  3. Defective SIM card or modem\n");
  } else {
    Serial.println("? Unknown SIM status: " + cpinResp + "\n");
  }

  // Test 4: Check IMSI
  Serial.println("[TEST 4] SIM IMSI (International Mobile Subscriber Identity)...");
  String imsiResp = sendATResponse("AT+CIMI", 3000);
  if (imsiResp.length() > 5 && imsiResp.indexOf("ERROR") == -1) {
    Serial.println("✓ SIM IMSI detected: " + imsiResp + "\n");
  } else {
    Serial.println("✗ Cannot read IMSI - SIM may not be compatible\n");
  }

  // Test 5: Check ICCID
  Serial.println("[TEST 5] SIM Card ICCID...");
  String iccidResp = sendATResponse("AT+CCID", 3000);
  if (iccidResp.length() > 10 && iccidResp.indexOf("ERROR") == -1) {
    Serial.println("✓ SIM ICCID detected: " + iccidResp + "\n");
  } else {
    Serial.println("✗ Cannot read ICCID - SIM may not be compatible\n");
  }

  // Test 6: Check network registration
  Serial.println("[TEST 6] Network Registration...");
  String nregResp = sendATResponse("AT+CREG?", 3000);
  Serial.println("Network registration response: " + nregResp);
  if (nregResp.indexOf("1") != -1 || nregResp.indexOf("5") != -1) {
    Serial.println("✓ Registered on network\n");
  } else if (nregResp.indexOf("2") != -1) {
    Serial.println("⊘ Searching for network...\n");
  } else {
    Serial.println("✗ Not registered on network\n");
  }

  // Test 7: Check signal quality
  Serial.println("[TEST 7] Signal Quality...");
  String csqResp = sendATResponse("AT+CSQ", 3000);
  Serial.println("Signal quality: " + csqResp + "\n");

  // Test 8: Try basic operations
  Serial.println("[TEST 8] Try storing SMS to SIM...");
  sendAT("AT+CMGF=1", "OK", 2000); // Set SMS text mode
  String smemResp = sendATResponse("AT+CPMS?", 3000);
  Serial.println("Message storage: " + smemResp + "\n");

  Serial.println("╔═════════════════════════════════════════╗");
  Serial.println("║  TROUBLESHOOTING STEPS                   ║");
  Serial.println("╠═════════════════════════════════════════╣");
  Serial.println("║ 1. Verify SIM with phone first           ║");
  Serial.println("║ 2. Try different SIM cards               ║");
  Serial.println("║ 3. Check power supply (2A recommended)   ║");
  Serial.println("║ 4. Add 1000µF capacitor on power lines   ║");
  Serial.println("║ 5. Verify RX/TX pins are correct         ║");
  Serial.println("║ 6. Test with known-working SIM           ║");
  Serial.println("║ 7. Check if carrier supports 2G/GPRS     ║");
  Serial.println("╚═════════════════════════════════════════╝\n");
}

// ─────────────────────────────────────────
// Main
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000); // Give Serial Monitor time to connect
  
  sim800.begin(9600);
  delay(1000);

  Serial.println("\n\n=== ESP32 SIM800L Diagnostic Tool ===");
  Serial.println("Options:");
  Serial.println("  'd' or 'D' - Run SIM diagnostics");
  Serial.println("  Any other text - Send as AT command");
  Serial.println("====================================\n");

  initSIM800();
  runDiagnostics();
}

void loop() {
  // Read from Serial Monitor
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove any trailing whitespace
    
    if (command.length() > 0) {
      if (command[0] == 'd' || command[0] == 'D') {
        runDiagnostics();
      } else {
        Serial.print(">> ");
        Serial.println(command);
        
        // Send command to SIM800L
        sim800.println(command);
        
        // Read response
        long timeout = millis() + 5000; // 5 seconds timeout
        String response = "";
        
        while (millis() < timeout) {
          while (sim800.available()) {
            char c = sim800.read();
            response += c;
            Serial.write(c);
          }
        }
        
        if (response.length() == 0) {
          Serial.println("\n[No response or timeout]");
        }
        Serial.println("\n");
      }
    }
  }
  
  // Also print any unsolicited responses from the modem
  while (sim800.available()) {
    Serial.write(sim800.read());
  }
}