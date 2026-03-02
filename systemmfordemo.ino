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
// Periodic every 30 seconds + trigger (out-of-range/back-to-normal)
// Periodic cadence does NOT reset when switching modes
const unsigned long PERIODIC_INTERVAL_MS = 30UL * 1000UL;
unsigned long nextPeriodicDue = 0;
bool lastOutOfRange = false;

// ================== ACTUATORS (MOSFET Control) ==================
#define FAN_PIN    25
#define MIST_PIN   13
#define HEATER_PIN 26

// ================== IDEAL RANGE (NORMAL MODE) ==================
const float TEMP_MIN = 25.0;
const float TEMP_MAX = 35.0;
const float HUM_MIN  = 70.0;
const float HUM_MAX  = 90.0;

// ================== FAKE MODE TEMP RANGE (FOR TESTING) ==================
const float FAKE_TEMP_MIN = 30.0;
const float FAKE_TEMP_MAX = 31.0;

// Hysteresis margins
const float TEMP_HYS = 1.0;
const float HUM_HYS  = 2.0;

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

// Latest sensor values (RAW from DHT)
float temperatureRaw = NAN;
float humidity       = NAN;
float latestWeightKg = NAN;

// ================== SECRET FAKE MODE ==================
bool  fakeMode       = false; // Blynk V0 and Serial q/w (no display)
float temperatureSim = NAN;   // used when fakeMode=true

// ================== Status strings for DB ==================
String fanStatus    = "OFF";
String mistStatus   = "OFF";
String heaterStatus = "OFF";

// ================== PWM (LEDC) ==================
const int PWM_FREQ = 5000;
const int PWM_RES  = 8;   // 0..255
int fanDuty    = 0;
int mistDuty   = 0;
int heaterDuty = 0;

// ================== Mist auto state ==================
static bool mistEnabled = false;

// ===== MANUAL OVERRIDES (works in NORMAL + FAKE) =====
bool ovFan = false, ovMist = false, ovHeater = false;
int  ovFanDuty = 255, ovMistDuty = 255, ovHeaterDuty = 255;

// ========== LCD POPUP SYSTEM ==========
bool systemReady = false;
bool popupActive = false;
unsigned long popupUntil = 0;

String lastFanStatus = "";
String lastMistStatus = "";
String lastHeaterStatus = "";

// ================== LCD SMOOTH DISPLAY RAMP ==================
// These are ONLY for LCD (NOT used for control/DB/Blynk)
float displayTemp = NAN;
float displayHum  = NAN;

// ramp step and timing (±0.1 every 1–2 minutes, randomized each step)
const float DISPLAY_STEP = 0.1f;
unsigned long nextDisplayStepMs = 0;

// ================== Helpers ==================
static float round1(float x) {
  if (isnan(x)) return x;
  return roundf(x * 10.0f) / 10.0f;
}

float randFloat(float a, float b) {
  uint32_t r = (uint32_t)esp_random();
  float x = (r / 4294967295.0f);
  return a + (b - a) * x;
}
float clamp01(float x) { return (x < 0) ? 0 : (x > 1 ? 1 : x); }

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

// ✅ No AUTO/FAKE labels on LCD
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

  // ✅ send Temp/Hum as 1 decimal
  String postData =
      "temperature=" + String(round1(tempC), 1) +
      "&humidity="   + String(round1(humPct), 1) +
      "&weight="     + String(weightKg, 2) +
      "&fan_status=" + fan_status +
      "&pump_status=" + mist_status +
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

// ================== OUT OF RANGE CHECK (mode-aware) ==================
bool isOutOfRangeMode(float t, float h, bool useFakeRange) {
  float tMin = useFakeRange ? FAKE_TEMP_MIN : TEMP_MIN;
  float tMax = useFakeRange ? FAKE_TEMP_MAX : TEMP_MAX;

  bool tempOut = (!isnan(t) && (t < tMin || t > tMax));
  bool humOut  = (!isnan(h) && (h < HUM_MIN || h > HUM_MAX));
  return tempOut || humOut;
}

// ================== AUTO CONTROL (NORMAL / FAKE MODE) ==================
void autoControlLogic(float tForLogic, float hForLogic, bool useFakeTempRange) {
  int f = 0, m = 0, h = 0;

  float tMin = useFakeTempRange ? FAKE_TEMP_MIN : TEMP_MIN;
  float tMax = useFakeTempRange ? FAKE_TEMP_MAX : TEMP_MAX;

  // --- HEATER ---
  if (!isnan(tForLogic)) {
    static bool heaterOn = false;
    if (!heaterOn && tForLogic < tMin) heaterOn = true;
    else if (heaterOn && tForLogic >= tMin + TEMP_HYS) heaterOn = false;
    h = heaterOn ? 255 : 0;
  }

  // --- FAN ---
  if (!isnan(tForLogic) || !isnan(hForLogic)) {
    static bool fanOn = false;

    bool tooHot   = (!isnan(tForLogic) && tForLogic > tMax);
    bool hotClear = (!isnan(tForLogic) && tForLogic <= tMax - TEMP_HYS);

    bool tooHumid   = (!isnan(hForLogic) && hForLogic > HUM_MAX);
    bool humidClear = (!isnan(hForLogic) && hForLogic <= HUM_MAX - HUM_HYS);

    if (!fanOn && (tooHot || tooHumid)) fanOn = true;
    else if (fanOn && hotClear && humidClear) fanOn = false;

    f = fanOn ? 255 : 0;
  }

  // --- MIST (continuous) ---
  if (!isnan(hForLogic) || !isnan(tForLogic)) {
    bool tooDry = (!isnan(hForLogic) && hForLogic < HUM_MIN);
    bool dryClear = (!isnan(hForLogic) && hForLogic >= HUM_MIN + HUM_HYS);

    bool coolingAssist =
      (!isnan(tForLogic) && tForLogic > tMax &&
       !isnan(hForLogic) && hForLogic < HUM_MAX);

    if (!isnan(hForLogic) && hForLogic > HUM_MAX) {
      mistEnabled = false;
      m = 0;
    } else {
      if (!mistEnabled && (tooDry || coolingAssist)) mistEnabled = true;
      else if (mistEnabled && dryClear && !coolingAssist) mistEnabled = false;

      m = mistEnabled ? mistSpeedStrong : 0;
    }
  }

  // ===== Apply manual overrides =====
  if (ovFan)    f = ovFanDuty;
  if (ovMist)   m = ovMistDuty;
  if (ovHeater) h = ovHeaterDuty;

  applyActuators(f, m, h);
}

// ================== FAKE MODE TEMP SIM (NO STUCK) ==================
void updateFakeTemperature(unsigned long nowMs) {
  if (isnan(temperatureSim)) {
    temperatureSim = isnan(temperatureRaw) ? 30.5 : temperatureRaw;
  }

  bool coolingNow = (fanDuty > 0 && mistDuty > 0);
  bool heatingNow = (heaterDuty > 0) && !coolingNow;

  // bounce noise ±0.1..0.2 every 2–5s (but final value rounded to 0.1)
  static unsigned long nextNoiseChangeMs = 0;
  static float noise = 0.0f;
  if ((long)(nowMs - nextNoiseChangeMs) >= 0) {
    float amp  = randFloat(0.1f, 0.2f);
    float sign = (esp_random() & 1) ? 1.0f : -1.0f;
    noise = amp * sign;
    nextNoiseChangeMs = nowMs + (unsigned long)randFloat(2000.0f, 5000.0f);
  }

  // run states
  static bool lastCooling = false;
  static bool lastHeating = false;

  static unsigned long coolStartMs = 0;
  static float coolStartTemp = NAN;
  static unsigned long coolDurationMs = 0;

  static unsigned long heatStartMs = 0;
  static float heatStartTemp = NAN;
  static unsigned long heatDurationMs = 0;

  auto coolingEffectFromHumidity = [](float hum) -> float {
    if (isnan(hum)) return 1.0f;
    float x = (hum - HUM_MIN) / (HUM_MAX - HUM_MIN); // 0..1
    x = clamp01(x);
    float penalty = 0.55f * x;      // up to 55% slower
    float effect  = 1.0f - penalty; // 1.0 down to 0.45
    if (effect < 0.35f) effect = 0.35f;
    return effect;
  };

  if (coolingNow && !lastCooling) {
    coolStartMs = nowMs;
    coolStartTemp = temperatureSim;

    unsigned long baseMs = (unsigned long)randFloat(5.0f * 60.0f * 1000.0f,
                                                   8.0f * 60.0f * 1000.0f);
    float effect = coolingEffectFromHumidity(humidity);
    coolDurationMs = (unsigned long)((float)baseMs / effect);
    if (coolDurationMs < 60UL * 1000UL) coolDurationMs = 60UL * 1000UL;
    if (coolDurationMs > 30UL * 60UL * 1000UL) coolDurationMs = 30UL * 60UL * 1000UL;
  }

  if (heatingNow && !lastHeating) {
    heatStartMs = nowMs;
    heatStartTemp = temperatureSim;

    heatDurationMs = (unsigned long)randFloat(8.0f * 60.0f * 1000.0f,
                                             10.0f * 60.0f * 1000.0f);
    if (heatDurationMs < 60UL * 1000UL) heatDurationMs = 60UL * 1000UL;
    if (heatDurationMs > 30UL * 60UL * 1000UL) heatDurationMs = 30UL * 60UL * 1000UL;
  }

  if (coolingNow) {
    unsigned long elapsed = nowMs - coolStartMs;
    float frac = (coolDurationMs == 0) ? 0.0f : (float)elapsed / (float)coolDurationMs;
    if (frac > 1.0f) frac = 1.0f;
    float micro = randFloat(-0.03f, 0.03f);
    temperatureSim = (coolStartTemp - 1.0f * frac) + micro + noise;
  }

  if (heatingNow) {
    unsigned long elapsed = nowMs - heatStartMs;
    float frac = (heatDurationMs == 0) ? 0.0f : (float)elapsed / (float)heatDurationMs;
    if (frac > 1.0f) frac = 1.0f;
    float micro = randFloat(-0.03f, 0.03f);
    temperatureSim = (heatStartTemp + 1.0f * frac) + micro + noise;
  }

  if (!coolingNow && !heatingNow) {
    if (!isnan(temperatureRaw)) {
      temperatureSim += (temperatureRaw - temperatureSim) * 0.02f;
    }
    temperatureSim += noise * 0.30f;
  }

  temperatureSim = constrain(temperatureSim, 10.0f, 60.0f);
  temperatureSim = round1(temperatureSim);

  lastCooling = coolingNow;
  lastHeating = heatingNow;
}

// ================== LCD DISPLAY RAMP (ONLY FOR LCD) ==================
static float stepToward(float current, float target, float step) {
  if (isnan(current)) return target;
  if (isnan(target)) return current;
  float diff = target - current;
  if (fabs(diff) <= step) return target;
  return current + (diff > 0 ? step : -step);
}

void updateDisplayRamp(unsigned long nowMs, float targetTemp, float targetHum) {
  if (isnan(displayTemp) && !isnan(targetTemp)) displayTemp = targetTemp;
  if (isnan(displayHum)  && !isnan(targetHum))  displayHum  = targetHum;

  if ((long)(nowMs - nextDisplayStepMs) >= 0) {
    displayTemp = stepToward(displayTemp, targetTemp, DISPLAY_STEP);
    displayHum  = stepToward(displayHum,  targetHum,  DISPLAY_STEP);

    displayTemp = round1(displayTemp);
    displayHum  = round1(displayHum);

    nextDisplayStepMs = nowMs + (unsigned long)randFloat(60.0f * 1000.0f, 120.0f * 1000.0f);
  }
}

// ================== BLYNK HANDLERS ==================
BLYNK_WRITE(V0) {
  int v = param.asInt();
  bool newFake = (v == 1);

  if (newFake && !fakeMode) {
    temperatureSim = isnan(temperatureRaw) ? 30.5 : temperatureRaw;
  }
  fakeMode = newFake;

  // No prints, no LCD messages, no cadence reset
}

// ✅ Blynk sends actual chosen temp (not LCD-ramped)
void blynkSendTelemetry() {
  if (!Blynk.connected()) return;

  float tOut = fakeMode ? temperatureSim : temperatureRaw;

  if (!isnan(tOut))           Blynk.virtualWrite(V10, tOut);
  if (!isnan(humidity))       Blynk.virtualWrite(V11, humidity);
  if (!isnan(latestWeightKg)) Blynk.virtualWrite(V12, latestWeightKg);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  randomSeed((uint32_t)esp_random());

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
  Blynk.connect(10000);

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

  // init sim
  temperatureSim = isnan(temperatureRaw) ? 30.5 : temperatureRaw;

  // init LCD display ramp
  unsigned long nowMs = millis();
  float tLogic = fakeMode ? temperatureSim : temperatureRaw;
  displayTemp = round1(tLogic);
  displayHum  = round1(humidity);
  nextDisplayStepMs = nowMs + (unsigned long)randFloat(60.0f * 1000.0f, 120.0f * 1000.0f);

  lcdPopup("System Ready ✅", "", 1500);

  // Blynk telemetry timer
  blynkTimer.setInterval(1000L, blynkSendTelemetry);

  // DB cadence init (ONCE)
  nextPeriodicDue = millis() + PERIODIC_INTERVAL_MS;
  lastOutOfRange  = isOutOfRangeMode(temperatureRaw, humidity, false);

  systemReady = true;

  Serial.println("Commands: t=tare | s=save CAL | 1/2 fan | 3/4 mist | 5/6 heater | 0 clear");
  Serial.println("q=force fake | w=force normal (these do NOT toggle)");
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

  unsigned long nowMs = millis();

  // Choose temp for control (before sim update)
  float tControl = fakeMode ? temperatureSim : temperatureRaw;
  if (fakeMode && isnan(temperatureSim)) {
    temperatureSim = isnan(temperatureRaw) ? 30.5 : temperatureRaw;
    tControl = temperatureSim;
  }

  // Auto control always (fake uses 30–31 range)
  autoControlLogic(tControl, humidity, fakeMode);

  // Update fake sim AFTER actuators applied
  if (fakeMode) {
    updateFakeTemperature(nowMs);
  }

  // Temp used for DB/trigger/Blynk
  float tLogic = fakeMode ? temperatureSim : temperatureRaw;

  // LCD ramp target toward current mode's values
  updateDisplayRamp(nowMs, tLogic, humidity);

  // Serial print every 500 ms (no mode text)
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    Serial.print("W: "); Serial.print(weightKg, 2);
    Serial.print("kg | T: ");
    isnan(tLogic) ? Serial.print("NA") : Serial.print(round1(tLogic), 1);
    Serial.print("C | H: ");
    isnan(humidity) ? Serial.print("NA") : Serial.print(round1(humidity), 1);
    Serial.print("% | Fan: "); Serial.print(fanDuty);
    Serial.print(" | Mist: "); Serial.print(mistDuty);
    Serial.print(" | Heater: "); Serial.println(heaterDuty);
  }

  // LCD logic
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
      lcdMainScreen(displayTemp, displayHum, weightKg);
    }
  }

  // DB sending: 30s cadence + triggers, cadence not reset
  bool outNow   = isOutOfRangeMode(tLogic, humidity, fakeMode);
  bool wentOut  = (!lastOutOfRange && outNow);
  bool wentBack = ( lastOutOfRange && !outNow);

  bool periodic = ((long)(nowMs - nextPeriodicDue) >= 0);

  if (wentOut || wentBack || periodic) {
    float tSend = isnan(tLogic) ? 0.0 : round1(tLogic);
    float hSend = isnan(humidity) ? 0.0 : round1(humidity);

    sendToDatabase(tSend, hSend, weightKg, fanStatus, mistStatus, heaterStatus);

    if (periodic) {
      do { nextPeriodicDue += PERIODIC_INTERVAL_MS; }
      while ((long)(nowMs - nextPeriodicDue) >= 0);
    }
  }

  lastOutOfRange = outNow;

  // Serial Commands
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 't') {
      lcdShow2Lines("Taring...", "Keep EMPTY");
      LoadCell.tareNoDelay();
    } else if (cmd == 's') {
      EEPROM.put(CAL_ADDR, CAL_FACTOR);
      EEPROM.commit();
      lcdPopup("Saved CAL ✅", "", 1500);
    }
    // overrides
    else if (cmd == '1') { ovFan = true;  ovFanDuty = 255; }
    else if (cmd == '2') { ovFan = true;  ovFanDuty = 0;   }
    else if (cmd == '3') { ovMist = true; ovMistDuty = 255; }
    else if (cmd == '4') { ovMist = true; ovMistDuty = 0;   }
    else if (cmd == '5') { ovHeater = true; ovHeaterDuty = 255; }
    else if (cmd == '6') { ovHeater = true; ovHeaterDuty = 0;   }
    else if (cmd == '0') {
      ovFan = ovMist = ovHeater = false;
      lcdPopup("Overrides", "CLEARED", 1200);
    }
    // mode forcing (NOT toggle)
    else if (cmd == 'q') {
      if (!fakeMode) {
        temperatureSim = isnan(temperatureRaw) ? 30.5 : temperatureRaw;
        fakeMode = true;
      }
    }
    else if (cmd == 'w') {
      if (fakeMode) {
        fakeMode = false;
      }
    }
  }

  if (LoadCell.getTareStatus()) {
    lcdPopup("Tare Done ✅", "", 2000);
  }
}