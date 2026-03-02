#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <DHT.h>

#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>

// ================== SERVER (PHP endpoint) ==================
const char* SERVER_URL = "http://192.168.100.224/thesis/Beehive/database/sensor_insert.php";

// Send interval (ms)
const unsigned long SEND_INTERVAL_MS = 5000; // every 5 seconds

// ================== FAN (MOSFET Control) ==================
#define FAN_PIN 25   // ESP32 GPIO -> MOSFET TRIG/PWM (fan ON/OFF)

// ================== PUMP (MOSFET Control) ==================
#define PUMP_PIN 13  // ESP32 GPIO -> MOSFET TRIG/PWM (pump PWM quiet)

// ================== HEATER (MOSFET Control) ==================
#define HEATER_PIN 26 // ESP32 GPIO -> MOSFET TRIG/PWM (heater ON/OFF)

// ================== PUMP SETTINGS ==================
// Stronger pump: use higher duty
int pumpSpeedStrong = 255;      // 0-255 (255 = strongest)

// Boost/prime mode: continuous ON for a while (for long hose)
const unsigned long PUMP_BOOST_MS = 120000; // 2 minutes continuous ON (change if needed)

// After boost, use burst mode (optional, for less wetting/noise)
const int PUMP_OFF_DUTY = 0;
const unsigned long PUMP_ON_MS  = 5000;    // ON 5s
const unsigned long PUMP_OFF_MS = 25000;   // OFF 25s

// Pump temperature thresholds (with hysteresis)
const float PUMP_ON_TEMP  = 29.0;  // start misting when above this
const float PUMP_OFF_TEMP = 28.0;  // stop misting when <= this

// Heater temperature thresholds (with hysteresis)
const float HEATER_ON_TEMP  = 22.0;
const float HEATER_OFF_TEMP = 23.0;

// ================== HX711 ==================
#define HX711_DOUT 4
#define HX711_SCK  5
HX711_ADC LoadCell(HX711_DOUT, HX711_SCK);

// Calibration factor (grams)
float CAL_FACTOR = 20.380953;

// ================== EEPROM ==================
#define EEPROM_SIZE 64
#define CAL_ADDR 0

// ================== DHT22 ==================
#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// ================== Timing ==================
unsigned long lastPrint = 0;
unsigned long lastDHT   = 0;
unsigned long lastSend  = 0;

// Latest sensor values
float temperature = NAN;
float humidity    = NAN;

// Status strings for DB/logging
String fanStatus    = "OFF";
String pumpStatus   = "OFF";
String heaterStatus = "OFF";

// Fan/heater hysteresis state
static bool fanOn = false;
static bool heaterOn = false;

// Pump control state
static bool pumpEnabledByTemp = false;
static bool pumpRunning = false;
static unsigned long pumpTimer = 0;

// Pump boost state
static bool pumpBoosting = false;
static unsigned long pumpBoostStart = 0;

// ================== Send to Database ==================
bool sendToDatabase(float tempC, float humPct, float weightKg,
                    const String& fan_status,
                    const String& pump_status,
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
      "&pump_status=" + pump_status +
      "&heater_status=" + heater_status;

  int httpCode = http.POST(postData);
  String payload = http.getString();
  http.end();

  Serial.print("POST -> HTTP ");
  Serial.println(httpCode);
  Serial.println(payload);

  return (httpCode > 0 && httpCode < 400);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("\n=== ESP32 HiveCare (HX711 + DHT22 + WiFi POST + FAN + PUMP + HEATER) ===");

  // ===== Fan pin init =====
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  fanStatus = "OFF";
  fanOn = false;

  // ===== Heater pin init =====
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  heaterStatus = "OFF";
  heaterOn = false;

  // ===== Pump PWM init (ESP32 LEDC new API) =====
  ledcAttach(PUMP_PIN, 5000, 8);  // 5kHz, 8-bit (0-255)
  ledcWrite(PUMP_PIN, PUMP_OFF_DUTY);
  pumpStatus = "OFF";
  pumpEnabledByTemp = false;
  pumpRunning = false;
  pumpTimer = millis();
  pumpBoosting = false;
  pumpBoostStart = 0;

  // ===== WiFiManager =====
  WiFiManager wm;
  wm.setTimeout(30);
  if (!wm.autoConnect("ESP32_Setup", "12345678")) {
    Serial.println("WiFi failed. Restarting...");
    delay(1000);
    ESP.restart();
  }

  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  // ===== EEPROM =====
  EEPROM.begin(EEPROM_SIZE);

  // ===== HX711 INIT =====
  LoadCell.begin();
  delay(1200);
  LoadCell.start(2000, false);

  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("HX711 ERROR: Check wiring.");
    while (1) { delay(10); }
  }

  LoadCell.setSamplesInUse(10);

  // Load calibration factor
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
  Serial.println("Stabilizing before tare... keep scale EMPTY.");
  delay(3000);
  LoadCell.update();
  LoadCell.tare();
  Serial.println("Startup tare complete ✅");

  // ===== DHT INIT =====
  dht.begin();

  Serial.println("\nSystem ready.");
  Serial.println("Commands:");
  Serial.println("  t = tare (zero weight)");
  Serial.println("  s = save calFactor to EEPROM");
  Serial.println();
}

void loop() {
  LoadCell.update();

  // ===== Read DHT every 2 seconds =====
  if (millis() - lastDHT >= 2000) {
    lastDHT = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
      humidity = h;
      temperature = t;
    }
  }

  // ===== Automatic Fan Control =====
  if (!isnan(temperature)) {
    if (!fanOn && temperature > 28.0) {
      fanOn = true;
      digitalWrite(FAN_PIN, HIGH);
      fanStatus = "ON";
    } else if (fanOn && temperature <= 27.5) {
      fanOn = false;
      digitalWrite(FAN_PIN, LOW);
      fanStatus = "OFF";
    }
  }

  // ===== Automatic Heater Control =====
  if (!isnan(temperature)) {
    if (!heaterOn && temperature < HEATER_ON_TEMP) {
      heaterOn = true;
      digitalWrite(HEATER_PIN, HIGH);
      heaterStatus = "ON";
    } else if (heaterOn && temperature >= HEATER_OFF_TEMP) {
      heaterOn = false;
      digitalWrite(HEATER_PIN, LOW);
      heaterStatus = "OFF";
    }
  }

  // ===== Pump TEMP logic (enable/disable by temperature) =====
  if (!isnan(temperature)) {
    if (!pumpEnabledByTemp && temperature > PUMP_ON_TEMP) {
      pumpEnabledByTemp = true;

      // Start boost (continuous) immediately when entering high temp
      pumpBoosting = true;
      pumpBoostStart = millis();

      // Turn pump ON strong
      ledcWrite(PUMP_PIN, pumpSpeedStrong);
      pumpStatus = "ON";
      pumpRunning = true;

      // reset burst timer for later
      pumpTimer = millis();
    } else if (pumpEnabledByTemp && temperature <= PUMP_OFF_TEMP) {
      // stop everything
      pumpEnabledByTemp = false;
      pumpBoosting = false;
      pumpRunning = false;

      ledcWrite(PUMP_PIN, PUMP_OFF_DUTY);
      pumpStatus = "OFF";
    }
  }

  // ===== Pump control when enabled =====
  if (pumpEnabledByTemp) {
    unsigned long now = millis();

    // 1) BOOST MODE: continuous ON for PUMP_BOOST_MS
    if (pumpBoosting) {
      if (now - pumpBoostStart >= PUMP_BOOST_MS) {
        // End boost, go to burst mode
        pumpBoosting = false;
        pumpRunning = false;
        pumpTimer = now;
        ledcWrite(PUMP_PIN, PUMP_OFF_DUTY);
        pumpStatus = "OFF";
      } else {
        // Stay ON strong during boost
        ledcWrite(PUMP_PIN, pumpSpeedStrong);
        pumpStatus = "ON";
        pumpRunning = true;
      }
    }
    // 2) BURST MODE after boost
    else {
      if (!pumpRunning) {
        if (now - pumpTimer >= PUMP_OFF_MS) {
          pumpRunning = true;
          pumpTimer = now;
          ledcWrite(PUMP_PIN, pumpSpeedStrong); // strong ON during burst too
          pumpStatus = "ON";
        }
      } else {
        if (now - pumpTimer >= PUMP_ON_MS) {
          pumpRunning = false;
          pumpTimer = now;
          ledcWrite(PUMP_PIN, PUMP_OFF_DUTY);
          pumpStatus = "OFF";
        }
      }
    }
  }

  // ===== Weight reading (kg) =====
  float weightKg = LoadCell.getData() / 1000.0;
  if (weightKg < 0.5) weightKg = 0.0;

  // ===== Serial print every 500 ms =====
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();

    Serial.print("Weight: ");
    Serial.print(weightKg, 2);

    Serial.print(" kg | Temp: ");
    isnan(temperature) ? Serial.print("NA") : Serial.print(temperature, 1);

    Serial.print(" °C | Hum: ");
    isnan(humidity) ? Serial.print("NA") : Serial.print(humidity, 1);

    Serial.print(" % | Fan: ");
    Serial.print(fanStatus);

    Serial.print(" | Pump: ");
    Serial.print(pumpStatus);
    if (pumpBoosting) Serial.print(" (BOOST)");

    Serial.print(" | Heater: ");
    Serial.println(heaterStatus);
  }

  // ===== Send to DB =====
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();

    float tSend = isnan(temperature) ? 0.0 : temperature;
    float hSend = isnan(humidity) ? 0.0 : humidity;

    Serial.println("Sending to database...");
    sendToDatabase(tSend, hSend, weightKg, fanStatus, pumpStatus, heaterStatus);
  }

  // ===== Serial Commands =====
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 't') {
      LoadCell.tareNoDelay();
      Serial.println("Taring... keep scale empty");

    } else if (cmd == 's') {
      EEPROM.put(CAL_ADDR, CAL_FACTOR);
      EEPROM.commit();
      Serial.print("Saved calFactor: ");
      Serial.println(CAL_FACTOR, 6);
    }
  }

  if (LoadCell.getTareStatus()) {
    Serial.println("Tare complete ✅");
  }
}
