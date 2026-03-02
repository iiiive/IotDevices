#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <DHT.h>

#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============ BLYNK ============
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID   "TMPL6ihUzZBCn"
#define BLYNK_TEMPLATE_NAME "HiveCare"
#define BLYNK_AUTH_TOKEN    "vBp4gRk6qA_5plwEi_fEZBaHwZrwWymI"
#include <BlynkSimpleEsp32.h>
BlynkTimer blynkTimer;

// ================== LCD ==================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================== SERVER (PHP endpoint) ==================
const char* SERVER_URL = "http://192.168.100.224/thesis/Beehive/database/sensor_insert.php";

// ✅ NORMAL MODE: send every 15 minutes
const unsigned long NORMAL_SEND_INTERVAL_MS = 15UL * 60UL * 1000UL;

// ================== ACTUATORS (MOSFET Control) ==================
#define FAN_PIN    25
#define MIST_PIN   13
#define HEATER_PIN 26

// ================== IDEAL RANGE ==================
const float TEMP_MIN = 25.0;
const float TEMP_MAX = 35.0;
const float HUM_MIN  = 70.0;
const float HUM_MAX  = 90.0;

// ✅ Hysteresis margins (OFF points you requested)
// Fan OFF at 34.5 (35 - 0.5)
// Heater OFF at 25.5 (25 + 0.5)
// Humidity clear at 89.5 (90 - 0.5)
const float TEMP_HYS = 0.5;  // 0.5°C
const float HUM_HYS  = 0.5;  // 0.5%

// ================== MIST SETTINGS ==================
int mistSpeedStrong = 255; // 0..255

// ================== HX711 ==================
#define HX711_DOUT 4
#define HX711_SCK  5
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);
float CAL_FACTOR = 20.380953;

// ================== EEPROM ==================
#define EEPROM_SIZE 64
#define CAL_ADDR 0

// ================== DHT22 ==================
#define DHTPIN 27
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ================== Timing ==================
unsigned long lastPrint = 0;
unsigned long lastDHT   = 0;
unsigned long lastLcd   = 0;

// ✅ Smart DB send state
unsigned long nextSendDue = 0;
bool lastOutOfRange = false;

// Latest sensor values
float temperature = NAN;
float humidity    = NAN;
float latestWeightKg = NAN; // ✅ cache for Blynk timer send

// Status strings for DB/logging
String fanStatus    = "OFF";
String mistStatus   = "OFF";
String heaterStatus = "OFF";

// ================== PWM (LEDC) ==================
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;   // 0..255

// Actual PWM outputs (0..255)
int fanDuty    = 0;
int mistDuty   = 0;
int heaterDuty = 0;

// ================== CONTROL MODE ==================
// 0 = NORMAL (auto), 1 = TEST (manual)
volatile int controlMode = 0;

// Manual duties from Blynk (0..255)
volatile int manualFanDuty    = 0;
volatile int manualMistDuty   = 0;
volatile int manualHeaterDuty = 0;

// ================== Mist auto state ==================
static bool mistEnabled = false;

// ========== LCD POPUP SYSTEM ==========
bool systemReady = false;
bool popupActive = false;
unsigned long popupUntil = 0;

String lastFanStatus = "";
String lastMistStatus = "";
String lastHeaterStatus = "";

// ================== LCD helper (NO lcd.clear spam) ==================
void lcdShow2Lines(const String& l1, const String& l2) {
  String line1 = l1.substring(0, 16);
  while (line1.length() < 16) line1 += " ";
  String line2 = l2.substring(0, 16);
  while (line2.length() < 16) line2 += " ";

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void lcdPopup(const String& l1, const String& l2, unsigned long ms = 2000) {
  popupActive = true;
  popupUntil = millis() + ms;
  lcdShow2Lines(l1, l2);
}

void lcdMainScreen(float tempC, float humPct, float weightKg) {
  String l1 = (controlMode == 0) ? "AUTO " : "TEST ";
  l1 += "T:";
  l1 += isnan(tempC) ? "NA" : String(tempC, 1);
  l1 += " H:";
  l1 += isnan(humPct) ? "NA" : String(humPct, 0);

  String l2 = "W:" + String(weightKg, 2) + "kg";
  String marks = " F" + String(fanDuty > 0 ? 1 : 0) + "M" + String(mistDuty > 0 ? 1 : 0) + "H" + String(heaterDuty > 0 ? 1 : 0);

  if (l2.length() + marks.length() <= 16) l2 += marks;
  else l2 = l2.substring(0, 16 - marks.length()) + marks;

  lcdShow2Lines(l1, l2);
}

// ================== Apply PWM + status ==================
void applyActuators(int fDuty, int mDuty, int hDuty) {
  fDuty = constrain(fDuty, 0, 255);
  mDuty = constrain(mDuty, 0, 255);
  hDuty = constrain(hDuty, 0, 255);

  fanDuty    = fDuty;
  mistDuty   = mDuty;
  heaterDuty = hDuty;

  // ESP32 core 3.x style (pin-based)
  ledcWrite(FAN_PIN, fanDuty);
  ledcWrite(MIST_PIN, mistDuty);
  ledcWrite(HEATER_PIN, heaterDuty);

  fanStatus    = (fanDuty    > 0) ? "ON" : "OFF";
  mistStatus   = (mistDuty   > 0) ? "ON" : "OFF";
  heaterStatus = (heaterDuty > 0) ? "ON" : "OFF";
}

// ================== Send to Database ==================
bool sendToDatabase(float tempC, float humPct, float weightKg,
                    const String& fan_status,
                    const String& mist_status,
                    const String& heater_status) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Can't send.");
    return false;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  String postData =
      "temperature=" + String(tempC, 2) +
      "&humidity="   + String(humPct, 2) +
      "&weight="     + String(weightKg, 2) +
      "&fan_status=" + fan_status +
      "&pump_status=" + mist_status +     // ✅ keep your DB field name
      "&heater_status=" + heater_status;

  int httpCode = http.POST(postData);
  String payload = http.getString();
  http.end();

  Serial.print("POST -> HTTP ");
  Serial.println(httpCode);
  Serial.println(payload);

  return (httpCode > 0 && httpCode < 400);
}

// ================== DHT Safe Read + Auto-Recover ==================
void readDHTSafe() {
  static int dhtFailCount = 0;

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  bool okH = !isnan(h);
  bool okT = !isnan(t);

  if (okH) humidity = h;
  if (okT) temperature = t;

  if (!okH || !okT) {
    dhtFailCount++;
    Serial.print("DHT FAIL #"); Serial.print(dhtFailCount);
    Serial.print(" H="); Serial.print(okH ? String(h,1) : "NaN");
    Serial.print(" T="); Serial.println(okT ? String(t,1) : "NaN");

    if (dhtFailCount >= 10) {
      Serial.println("Reinitializing DHT...");
      dht.begin();
      dhtFailCount = 0;
    }
  } else {
    dhtFailCount = 0;
  }
}

// ================== OUT OF RANGE CHECK ==================
bool isOutOfRange(float t, float h) {
  bool tempOut = (!isnan(t) && (t < TEMP_MIN || t > TEMP_MAX));
  bool humOut  = (!isnan(h) && (h < HUM_MIN  || h > HUM_MAX));
  return tempOut || humOut;
}

// ================== AUTO CONTROL (NORMAL MODE) ==================
void autoControlLogic() {
  int f = 0, m = 0, h = 0;

  // --- HEATER (if too cold) ---
  if (!isnan(temperature)) {
    static bool heaterOn = false;
    if (!heaterOn && temperature < TEMP_MIN) heaterOn = true;
    else if (heaterOn && temperature >= TEMP_MIN + TEMP_HYS) heaterOn = false; // ✅ OFF at 25.5
    h = heaterOn ? 255 : 0;
  }

  // --- FAN (if too hot OR too humid) ---
  if (!isnan(temperature) || !isnan(humidity)) {
    static bool fanOn = false;

    bool tooHot   = (!isnan(temperature) && temperature > TEMP_MAX);
    bool hotClear = (!isnan(temperature) && temperature <= TEMP_MAX - TEMP_HYS);  // ✅ OFF at 34.5

    bool tooHumid   = (!isnan(humidity) && humidity > HUM_MAX);
    bool humidClear = (!isnan(humidity) && humidity <= HUM_MAX - HUM_HYS);        // ✅ OFF at 89.5

    if (!fanOn && (tooHot || tooHumid)) fanOn = true;
    else if (fanOn && hotClear && humidClear) fanOn = false;

    f = fanOn ? 255 : 0;
  }

  // --- MIST (continuous, with OFF at 34.5 when cooling-assist) ---
  if (!isnan(humidity) || !isnan(temperature)) {
    bool tooDry   = (!isnan(humidity) && humidity < HUM_MIN);
    bool dryClear = (!isnan(humidity) && humidity >= HUM_MIN + HUM_HYS);  // uses HUM_HYS=0.5

    // Cooling assist ON when >35 and humidity still below max
    bool coolingAssistOn =
      (!isnan(temperature) && temperature > TEMP_MAX &&
       !isnan(humidity) && humidity < HUM_MAX);

    // Cooling assist clear when temp <= 34.5
    bool coolingAssistClear =
      (!isnan(temperature) && temperature <= (TEMP_MAX - TEMP_HYS));

    // Force OFF if humidity too high
    if (!isnan(humidity) && humidity > HUM_MAX) {
      mistEnabled = false;
      m = 0;
    } else {
      // Turn ON if too dry OR cooling assist
      if (!mistEnabled && (tooDry || coolingAssistOn)) {
        mistEnabled = true;
      }
      // Turn OFF when:
      // - humidity recovered and no cooling assist needed
      // OR
      // - cooling has cleared (<=34.5)
      else if (mistEnabled && ((dryClear && !coolingAssistOn) || coolingAssistClear)) {
        mistEnabled = false;
      }

      m = mistEnabled ? mistSpeedStrong : 0; // ✅ continuous ON when enabled
    }
  }

  applyActuators(f, m, h);
}

// ================== BLYNK HANDLERS ==================
BLYNK_WRITE(V0) { // mode switch
  controlMode = param.asInt(); // 0 auto, 1 test

  if (controlMode == 0) {
    lcdPopup("MODE: AUTO", "Normal Control", 1500);
    mistEnabled = false;
    // ✅ DO NOT reset nextSendDue (you requested this)
    // ✅ Update lastOutOfRange so edge-trigger stays clean
    lastOutOfRange = isOutOfRange(temperature, humidity);
  } else {
    lcdPopup("MODE: TEST", "Manual Control", 1500);
    // ✅ DO NOT reset nextSendDue (you requested this)
    lastOutOfRange = isOutOfRange(temperature, humidity);
  }
}

BLYNK_WRITE(V1) { manualFanDuty    = constrain(param.asInt(), 0, 255); }
BLYNK_WRITE(V2) { manualMistDuty   = constrain(param.asInt(), 0, 255); }
BLYNK_WRITE(V3) { manualHeaterDuty = constrain(param.asInt(), 0, 255); }

// ✅ SEND ONLY ON TIMER (best practice)
void blynkSendTelemetry() {
  if (!Blynk.connected()) return;

  if (!isnan(temperature))     Blynk.virtualWrite(V10, temperature);
  if (!isnan(humidity))        Blynk.virtualWrite(V11, humidity);
  if (!isnan(latestWeightKg))  Blynk.virtualWrite(V12, latestWeightKg);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // ===== LCD init =====
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcdShow2Lines("HiveCare Booting", "Please wait...");

  Serial.println("\n=== ESP32 HiveCare (HX711 + DHT22 + WiFi + Blynk + DB + PWM actuators + LCD) ===");

  // ===== PWM init (ESP32 core 3.x LEDC API) =====
  ledcAttach(FAN_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(FAN_PIN, 0);

  ledcAttach(MIST_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(MIST_PIN, 0);

  ledcAttach(HEATER_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(HEATER_PIN, 0);

  applyActuators(0, 0, 0);

  // ===== WiFiManager =====
  lcdShow2Lines("WiFi Setup", "Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFiManager wm;
  wm.setTimeout(30);

  if (!wm.autoConnect("ESP32_Setup", "12345678")) {
    lcdShow2Lines("WiFi Failed", "Restarting...");
    Serial.println("WiFi failed. Restarting...");
    delay(1200);
    ESP.restart();
  }

  lcdShow2Lines("WiFi Connected", WiFi.localIP().toString());
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());
  delay(1200);

  // ===== Blynk connect =====
  lcdShow2Lines("Blynk", "Connecting...");
  Blynk.config(BLYNK_AUTH_TOKEN);
  if (!Blynk.connect(10000)) {
    lcdShow2Lines("Blynk Failed", "Offline Mode");
    Serial.println("Blynk not connected (continuing offline).");
    delay(1200);
  } else {
    lcdShow2Lines("Blynk Connected", "Ready");
    Serial.println("Blynk connected ✅");
    delay(800);
  }

  // ===== EEPROM =====
  EEPROM.begin(EEPROM_SIZE);

  // ===== HX711 INIT =====
  lcdShow2Lines("HX711 Init", "Please wait...");
  LoadCell.begin();
  delay(1200);
  LoadCell.start(2000, false);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    lcdShow2Lines("HX711 ERROR", "Check wiring!");
    Serial.println("HX711 ERROR: Check wiring.");
    while (1) { delay(10); }
  }

  LoadCell.setSamplesInUse(10);

  float savedCal = 0;
  EEPROM.get(CAL_ADDR, savedCal);
  if (!isnan(savedCal) && savedCal > 0.1 && savedCal < 100000.0) {
    CAL_FACTOR = savedCal;
    Serial.print("Loaded calFactor from EEPROM: ");
    Serial.println(CAL_FACTOR, 6);
  } else {
    Serial.print("Using hardcoded calFactor: ");
    Serial.println(CAL_FACTOR, 6);
  }
  LoadCell.setCalFactor(CAL_FACTOR);

  // ===== STARTUP TARE =====
  lcdShow2Lines("Taring Scale", "Keep EMPTY...");
  Serial.println("Stabilizing before tare... keep scale EMPTY.");
  delay(3000);

  LoadCell.update();
  LoadCell.tare();
  Serial.println("Startup tare complete ✅");
  lcdPopup("Tare Done ✅", "System Ready", 2000);

  // ===== DHT INIT =====
  dht.begin();
  readDHTSafe();

  // ✅ Blynk telemetry timer (1 second)
  blynkTimer.setInterval(1000L, blynkSendTelemetry);

  // ✅ Smart DB timer init (starts schedule ONCE at boot)
  nextSendDue = millis() + NORMAL_SEND_INTERVAL_MS;
  lastOutOfRange = isOutOfRange(temperature, humidity);

  systemReady = true;

  Serial.println("\nSystem ready.");
  Serial.println("Commands:");
  Serial.println("  t = tare (zero weight)");
  Serial.println("  s = save calFactor to EEPROM");
  Serial.println();
}

void loop() {
  Blynk.run();
  blynkTimer.run();

  LoadCell.update();

  // ===== Read DHT every 3 seconds =====
  if (millis() - lastDHT >= 3000) {
    lastDHT = millis();
    readDHTSafe();
  }

  // ===== Weight reading (kg) =====
  float weightKg = LoadCell.getData() / 1000.0;
  if (weightKg < 0.5) weightKg = 0.0;
  latestWeightKg = weightKg; // ✅ cache for Blynk timer send

  // ===== CONTROL MODE LOGIC =====
  if (controlMode == 0) {
    autoControlLogic();
  } else {
    applyActuators(manualFanDuty, manualMistDuty, manualHeaterDuty);
  }

  // ===== Serial print every 500 ms =====
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    Serial.print(controlMode == 0 ? "[AUTO] " : "[TEST] ");
    Serial.print("W: "); Serial.print(weightKg, 2); Serial.print("kg | T: ");
    isnan(temperature) ? Serial.print("NA") : Serial.print(temperature, 2);
    Serial.print("C | H: ");
    isnan(humidity) ? Serial.print("NA") : Serial.print(humidity, 2);
    Serial.print("% | Fan: "); Serial.print(fanDuty);
    Serial.print(" | Mist: "); Serial.print(mistDuty);
    Serial.print(" | Heater: "); Serial.println(heaterDuty);
  }

  // ===== LCD logic =====
  if (systemReady) {
    if (fanStatus != lastFanStatus) {
      lastFanStatus = fanStatus;
      lcdPopup("FAN " + fanStatus, "Duty:" + String(fanDuty), 1200);
    }
    if (mistStatus != lastMistStatus) {
      lastMistStatus = mistStatus;
      lcdPopup("MIST " + mistStatus, "Duty:" + String(mistDuty), 1200);
    }
    if (heaterStatus != lastHeaterStatus) {
      lastHeaterStatus = heaterStatus;
      lcdPopup("HEATER " + heaterStatus, "Duty:" + String(heaterDuty), 1200);
    }

    if (popupActive && millis() > popupUntil) popupActive = false;

    // ✅ LCD main screen refresh every 3 seconds (same pace as DHT)
    if (!popupActive && (millis() - lastLcd >= 3000)) {
      lastLcd = millis();
      lcdMainScreen(temperature, humidity, weightKg);
    }
  }

  // ===== SMART Send to DB (AUTO + TEST) =====
  unsigned long now = millis();

  bool outNow   = isOutOfRange(temperature, humidity);
  bool wentOut  = (!lastOutOfRange && outNow);    // normal -> out
  bool wentBack = ( lastOutOfRange && !outNow);   // out -> normal
  bool periodic = (now >= nextSendDue);

  // ✅ Event trigger works in BOTH modes now
  if (wentOut || wentBack || periodic) {
    float tSend = isnan(temperature) ? 0.0 : temperature;
    float hSend = isnan(humidity) ? 0.0 : humidity;

    Serial.print("DB send reason: ");
    if (wentOut) Serial.println("OUT_OF_RANGE");
    else if (wentBack) Serial.println("BACK_TO_NORMAL");
    else Serial.println(controlMode == 0 ? "PERIODIC_15_MIN" : "TEST_PERIODIC_15_MIN");

    sendToDatabase(tSend, hSend, weightKg, fanStatus, mistStatus, heaterStatus);

    // ✅ reset 15-min schedule after ANY send (same as before)
    nextSendDue = now + NORMAL_SEND_INTERVAL_MS;
  }

  // ✅ always update last state so transitions trigger only once
  lastOutOfRange = outNow;

  // ===== Serial Commands =====
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 't') {
      Serial.println("Taring... keep scale empty");
      lcdShow2Lines("Taring...", "Keep EMPTY");
      LoadCell.tareNoDelay();

    } else if (cmd == 's') {
      EEPROM.put(CAL_ADDR, CAL_FACTOR);
      EEPROM.commit();
      Serial.print("Saved calFactor: ");
      Serial.println(CAL_FACTOR, 6);
      lcdPopup("Saved CAL ✅", "", 1500);
    }
  }

  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete ✅");
    lcdPopup("Tare Done ✅", "", 2000);
  }
}