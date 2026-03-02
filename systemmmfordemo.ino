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

// ================== DB SENDING RULES ==================
// Periodic every 30 seconds
// + EVENT send when actuators change
// ✅ 30s schedule RESETS after any EVENT send
const unsigned long PERIODIC_INTERVAL_MS = 30UL * 1000UL;
unsigned long nextPeriodicDue = 0;

// ================== ACTUATORS (MOSFET Control) ==================
#define FAN_PIN    25
#define MIST_PIN   13
#define HEATER_PIN 26

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

// Latest sensor values
float temperatureRaw = NAN; // 1 decimal
float humidity       = NAN; // 1 decimal
float latestWeightKg = NAN;

// ================== Status strings for DB ==================
String fanStatus    = "OFF";
String mistStatus   = "OFF";
String heaterStatus = "OFF";

// last status snapshot (for event triggers)
String lastSentFanStatus    = "OFF";
String lastSentMistStatus   = "OFF";
String lastSentHeaterStatus = "OFF";

// ================== PWM (LEDC) ==================
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;   // 0..255
int fanDuty    = 0;
int mistDuty   = 0;
int heaterDuty = 0;

// ================== MANUAL CONTROL STATES ==================
int fanSetDuty    = 0;
int mistSetDuty   = 0;
int heaterSetDuty = 0;

// ========== LCD POPUP SYSTEM ==========
bool systemReady = false;
bool popupActive = false;
unsigned long popupUntil = 0;

String lastFanStatus = "";
String lastMistStatus = "";
String lastHeaterStatus = "";

// ================== Helpers ==================
static float round1(float x) {
  if (isnan(x)) return x;
  return roundf(x * 10.0f) / 10.0f;
}

// ================== LCD helper ==================
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
  String l1 = "T:";
  l1 += isnan(tempC) ? "NA" : String(tempC, 1);
  l1 += " H:";
  l1 += isnan(humPct) ? "NA" : String(humPct, 1);

  String l2 = "W:" + String(weightKg, 2) + "kg";
  String marks = " F" + String(fanDuty > 0 ? 1 : 0) +
                 "M" + String(mistDuty > 0 ? 1 : 0) +
                 "H" + String(heaterDuty > 0 ? 1 : 0);

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

  // ESP32 core 3.x: ledcWrite(pin, duty) after ledcAttach(pin,...)
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
      "temperature=" + String(round1(tempC), 1) +
      "&humidity="   + String(round1(humPct), 1) +
      "&weight="     + String(weightKg, 2) +
      "&fan_status=" + fan_status +
      "&pump_status=" + mist_status +     // keep your DB field name
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

  if (okH) humidity = round1(h);
  if (okT) temperatureRaw = round1(t);

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

// ================== Blynk telemetry only ==================
void blynkSendTelemetry() {
  if (!Blynk.connected()) return;

  if (!isnan(temperatureRaw))  Blynk.virtualWrite(V10, temperatureRaw);
  if (!isnan(humidity))        Blynk.virtualWrite(V11, humidity);
  if (!isnan(latestWeightKg))  Blynk.virtualWrite(V12, latestWeightKg);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // LCD init
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  lcdShow2Lines("HiveCare Booting", "Please wait...");

  // PWM init
  ledcAttach(FAN_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(FAN_PIN, 0);

  ledcAttach(MIST_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(MIST_PIN, 0);

  ledcAttach(HEATER_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(HEATER_PIN, 0);

  applyActuators(0, 0, 0);

  // WiFiManager
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  WiFiManager wm;
  wm.setTimeout(30);
  if (!wm.autoConnect("ESP32_Setup", "12345678")) {
    Serial.println("WiFi failed. Restarting...");
    delay(1200);
    ESP.restart();
  }

  // Blynk
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(10000); // continue even if offline

  // EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // HX711 init
  LoadCell.begin();
  delay(1200);
  LoadCell.start(2000, false);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    lcdShow2Lines("HX711 ERROR", "Check wiring!");
    while (1) { delay(10); }
  }

  LoadCell.setSamplesInUse(10);

  float savedCal = 0;
  EEPROM.get(CAL_ADDR, savedCal);
  if (!isnan(savedCal) && savedCal > 0.1 && savedCal < 100000.0) {
    CAL_FACTOR = savedCal;
  }
  LoadCell.setCalFactor(CAL_FACTOR);

  // Startup tare
  delay(3000);
  LoadCell.update();
  LoadCell.tare();

  // DHT init
  dht.begin();
  readDHTSafe();

  lcdPopup("System Ready ✅", "", 1500);

  // Blynk telemetry timer (1s)
  blynkTimer.setInterval(1000L, blynkSendTelemetry);

  // DB cadence init (ONCE)
  nextPeriodicDue = millis() + PERIODIC_INTERVAL_MS;

  // Initialize "last sent" actuator statuses
  lastSentFanStatus    = fanStatus;
  lastSentMistStatus   = mistStatus;
  lastSentHeaterStatus = heaterStatus;

  systemReady = true;

  Serial.println("MANUAL ACTUATORS (no thresholds):");
  Serial.println("1 fan ON  | 2 fan OFF");
  Serial.println("3 mist ON | 4 mist OFF");
  Serial.println("5 heat ON | 6 heat OFF");
  Serial.println("t tare | s save CAL");
  Serial.println("DB: every 30s, and on actuator change (event). 30s resets after event.");
}

void loop() {
  Blynk.run();
  blynkTimer.run();

  LoadCell.update();

  // DHT every 3 seconds
  if (millis() - lastDHT >= 3000) {
    lastDHT = millis();
    readDHTSafe();
  }

  // Weight reading (kg)
  float weightKg = LoadCell.getData() / 1000.0;
  if (weightKg < 0.5) weightKg = 0.0;
  latestWeightKg = weightKg;

  // ✅ Manual actuator apply (NO AUTO THRESHOLDS)
  applyActuators(fanSetDuty, mistSetDuty, heaterSetDuty);

  // Serial print every 500 ms (no mode text)
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("W: "); Serial.print(weightKg, 2);
    Serial.print("kg | T: ");
    isnan(temperatureRaw) ? Serial.print("NA") : Serial.print(temperatureRaw, 1);
    Serial.print("C | H: ");
    isnan(humidity) ? Serial.print("NA") : Serial.print(humidity, 1);
    Serial.print("% | Fan: "); Serial.print(fanDuty);
    Serial.print(" | Mist: "); Serial.print(mistDuty);
    Serial.print(" | Heater: "); Serial.println(heaterDuty);
  }

  // LCD popups on actuator changes + main screen
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

    if (!popupActive && (millis() - lastLcd >= 3000)) {
      lastLcd = millis();
      lcdMainScreen(temperatureRaw, humidity, weightKg);
    }
  }

  // ===== DB sending: periodic + event =====
  unsigned long nowMs = millis();

  bool periodicDue = ((long)(nowMs - nextPeriodicDue) >= 0);

  // EVENT = any actuator status changed since last SEND
  bool eventActuatorChange =
      (fanStatus != lastSentFanStatus) ||
      (mistStatus != lastSentMistStatus) ||
      (heaterStatus != lastSentHeaterStatus);

  if (eventActuatorChange || periodicDue) {
    float tSend = isnan(temperatureRaw) ? 0.0 : temperatureRaw;
    float hSend = isnan(humidity) ? 0.0 : humidity;

    sendToDatabase(tSend, hSend, weightKg, fanStatus, mistStatus, heaterStatus);

    // update last sent statuses AFTER successful attempt (still update even if server fails? your choice)
    lastSentFanStatus    = fanStatus;
    lastSentMistStatus   = mistStatus;
    lastSentHeaterStatus = heaterStatus;

    if (eventActuatorChange) {
      // ✅ RESET 30s schedule after EVENT trigger
      nextPeriodicDue = nowMs + PERIODIC_INTERVAL_MS;
    } else {
      // periodic schedule continues
      nextPeriodicDue = nowMs + PERIODIC_INTERVAL_MS;
    }
  }

  // ===== Serial Commands (manual ON/OFF) =====
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 't') {
      lcdShow2Lines("Taring...", "Keep EMPTY");
      LoadCell.tareNoDelay();
    }
    else if (cmd == 's') {
      EEPROM.put(CAL_ADDR, CAL_FACTOR);
      EEPROM.commit();
      lcdPopup("Saved CAL ✅", "", 1500);
    }
    else if (cmd == '1') { fanSetDuty = 255; }  // fan ON
    else if (cmd == '2') { fanSetDuty = 0;   }  // fan OFF
    else if (cmd == '3') { mistSetDuty = 255; } // mist ON
    else if (cmd == '4') { mistSetDuty = 0;   } // mist OFF
    else if (cmd == '5') { heaterSetDuty = 255; } // heater ON
    else if (cmd == '6') { heaterSetDuty = 0;   } // heater OFF
  }

  if (LoadCell.getTareStatus()) {
    lcdPopup("Tare Done ✅", "", 2000);
  }
}