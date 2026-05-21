#include <OneWire.h>
#include <DallasTemperature.h>

#define PIN_12V_DC_out      36
#define PIN_12V_Handbrake   34
#define PIN_72V_DC_in       39
#define PIN_72V_MPPT_in     35
#define PIN_DS18B20         15
#define PIN_CURRENT_in      33
#define PIN_CURRENT_2_out   32
#define PIN_LED1            25
#define PIN_LED2            26

#define DIV_12V_RATIO   (2650.0 / (10000.0 + 2650.0))
#define DIV_72V_RATIO   (1200.0 / (47000.0 + 1200.0))
#define ADC_REF         3.3
#define ADC_RES         4095.0
#define ACS712_VREF     1.65
#define ACS712_MV_PER_A 100.0

OneWire oneWire(PIN_DS18B20);
DallasTemperature sensors(&oneWire);

float adcToVoltage(int pin) {
  return (analogRead(pin) / ADC_RES) * ADC_REF;
}

float read12V_DC_out()  { return adcToVoltage(PIN_12V_DC_out)  / DIV_12V_RATIO; }
float read12V_Handbrake() { return adcToVoltage(PIN_12V_Handbrake) / DIV_12V_RATIO; }
float read72V_DC_in()   { return adcToVoltage(PIN_72V_DC_in)   / DIV_72V_RATIO; }
float read72V_MPPT_in() { return adcToVoltage(PIN_72V_MPPT_in) / DIV_72V_RATIO; }
float readCurrent(int pin) {
  float vOut = adcToVoltage(pin);
  return (vOut - ACS712_VREF) / (ACS712_MV_PER_A / 1000.0);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  sensors.begin();
  Serial.printf("Found %d sensor(s)\n", sensors.getDeviceCount());

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  Serial.println("=== BAKO Sensor Reader Started ===");
}

void loop() {
  float v12_dc_out    = read12V_DC_out();
  float v12_handbrake = read12V_Handbrake();
  float v72_dc_in     = read72V_DC_in();
  float v72_mppt_in   = read72V_MPPT_in();
  float current_in    = readCurrent(PIN_CURRENT_in);
  float current_out   = readCurrent(PIN_CURRENT_2_out);

  sensors.requestTemperatures();
  float temp0 = sensors.getTempCByIndex(0);
  float temp1 = sensors.getTempCByIndex(1);
  float temp2 = sensors.getTempCByIndex(2);

  Serial.println("--- Sensor Readings ---");
  Serial.printf("12V DC out   (GPIO36): %.2f V\n", v12_dc_out);
  Serial.printf("12V Handbrake(GPIO34): %.2f V\n", v12_handbrake);
  Serial.printf("72V DC in    (GPIO39): %.2f V\n", v72_dc_in);
  Serial.printf("72V MPPT in  (GPIO35): %.2f V\n", v72_mppt_in);
  Serial.printf("Current in   (GPIO33): %.2f A\n", current_in);
  Serial.printf("Current out  (GPIO32): %.2f A\n", current_out);
  Serial.printf("Temp 0 (MPPT)        : %.2f C\n", temp0);
  Serial.printf("Temp 1 (DC/DC)       : %.2f C\n", temp1);
  Serial.printf("Temp 2 (Motor)       : %.2f C\n", temp2);
  Serial.println();

  digitalWrite(PIN_LED1, HIGH); delay(50);
  digitalWrite(PIN_LED1, LOW);  delay(50);
  digitalWrite(PIN_LED2, HIGH); delay(50);
  digitalWrite(PIN_LED2, LOW);

  delay(500);
}