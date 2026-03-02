#include <Arduino.h>
#include <DHT.h>

// ================== ACTUATOR PINS ==================
#define FAN_PIN     25
#define MIST_PIN    13
#define HEATER_PIN  26

// ================== PWM SETTINGS ==================
// NOTE: Ultrasonic mist drivers often don't "dim" nicely with fast PWM.
// This code still supports PWM, but if you don't see effect on mist,
// use ON/OFF or time-cycling.
const int PWM_FREQ = 5000;   // 5kHz
const int PWM_RES  = 8;      // 8-bit => 0..255

// ================== ADJUSTABLE POWER (0..255) ==================
int fanDuty    = 255;
int mistDuty   = 255;
int heaterDuty = 255;
const int STEP = 10;

// Track ON/OFF states so duty changes apply immediately
bool fanOn = false;
bool mistOn = false;
bool heaterOn = false;

// ================== DHT22 ==================
#define DHTPIN  15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

unsigned long lastDHT = 0;
float tempC = NAN;
float hum   = NAN;

void printPowers() {
  Serial.print("Powers -> FAN:");
  Serial.print(fanDuty);
  Serial.print(" MIST:");
  Serial.print(mistDuty);
  Serial.print(" HEATER:");
  Serial.println(heaterDuty);

  Serial.print("States -> FAN:");
  Serial.print(fanOn ? "ON" : "OFF");
  Serial.print(" MIST:");
  Serial.print(mistOn ? "ON" : "OFF");
  Serial.print(" HEATER:");
  Serial.println(heaterOn ? "ON" : "OFF");
}

void applyFan() {
  ledcWrite(FAN_PIN, fanOn ? fanDuty : 0);
}

void applyMist() {
  ledcWrite(MIST_PIN, mistOn ? mistDuty : 0);
}

void applyHeater() {
  ledcWrite(HEATER_PIN, heaterOn ? heaterDuty : 0);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // ✅ ESP32 LEDC new API:
  // ledcAttach(pin, freq, resolution)
  ledcAttach(FAN_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MIST_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(HEATER_PIN, PWM_FREQ, PWM_RES);

  // All OFF at startup
  fanOn = false;
  mistOn = false;
  heaterOn = false;
  applyFan();
  applyMist();
  applyHeater();

  // DHT init
  dht.begin();

  Serial.println("=================================");
  Serial.println(" ESP32 ACTUATOR + DHT22 MANUAL TEST ");
  Serial.println("=================================");
  Serial.println("ON/OFF:");
  Serial.println("1 -> FAN ON (uses fanDuty)");
  Serial.println("2 -> FAN OFF");
  Serial.println("3 -> MIST ON (uses mistDuty)");
  Serial.println("4 -> MIST OFF");
  Serial.println("5 -> HEATER ON (uses heaterDuty)");
  Serial.println("6 -> HEATER OFF");
  Serial.println();
  Serial.println("POWER ADJUST (0..255):");
  Serial.println("q -> FAN power DOWN   | w -> FAN power UP");
  Serial.println("a -> MIST power DOWN  | s -> MIST power UP");
  Serial.println("z -> HEATER power DOWN| x -> HEATER power UP");
  Serial.println("p -> print current powers/states");
  Serial.println();
  Serial.println("DHT22 prints every 1s (reads every 3s).");
  Serial.println();

  printPowers();
}

void loop() {
  // ===== Read DHT every 3 seconds (safe timing) =====
  if (millis() - lastDHT >= 3000) {
    lastDHT = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (!isnan(h) && !isnan(t)) {
      hum   = h;
      tempC = t;
    }
  }

  // ===== ALWAYS SHOW MEASUREMENTS (every 1 second) =====
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("🌡 Temp: ");
    if (isnan(tempC)) Serial.print("NA");
    else Serial.print(tempC, 1);

    Serial.print(" °C   💧 Humidity: ");
    if (isnan(hum)) Serial.print("NA");
    else Serial.print(hum, 1);

    Serial.print(" %   | Duty F/M/H: ");
    Serial.print(fanDuty);
    Serial.print("/");
    Serial.print(mistDuty);
    Serial.print("/");
    Serial.print(heaterDuty);

    Serial.print(" | State F/M/H: ");
    Serial.print(fanOn ? "1" : "0");
    Serial.print("/");
    Serial.print(mistOn ? "1" : "0");
    Serial.print("/");
    Serial.println(heaterOn ? "1" : "0");
  }

  // ===== SERIAL COMMANDS =====
  if (Serial.available()) {
    char cmd = Serial.read();

    // ignore line endings
    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      // ===== ON/OFF =====
      case '1':
        fanOn = true;
        applyFan();
        Serial.print("🌀 FAN ON (duty ");
        Serial.print(fanDuty);
        Serial.println(")");
        break;

      case '2':
        fanOn = false;
        applyFan();
        Serial.println("🛑 FAN OFF");
        break;

      case '3':
        mistOn = true;
        applyMist();
        Serial.print("💧 MIST ON (duty ");
        Serial.print(mistDuty);
        Serial.println(")");
        break;

      case '4':
        mistOn = false;
        applyMist();
        Serial.println("🛑 MIST OFF");
        break;

      case '5':
        heaterOn = true;
        applyHeater();
        Serial.print("🔥 HEATER ON (duty ");
        Serial.print(heaterDuty);
        Serial.println(")");
        break;

      case '6':
        heaterOn = false;
        applyHeater();
        Serial.println("❄ HEATER OFF");
        break;

      // ===== FAN power adjust =====
      case 'q':
        fanDuty = max(0, fanDuty - STEP);
        Serial.print("FAN duty DOWN -> ");
        Serial.println(fanDuty);
        if (fanOn) applyFan();
        break;

      case 'w':
        fanDuty = min(255, fanDuty + STEP);
        Serial.print("FAN duty UP -> ");
        Serial.println(fanDuty);
        if (fanOn) applyFan();
        break;

      // ===== MIST power adjust =====
      case 'a':
        mistDuty = max(0, mistDuty - STEP);
        Serial.print("MIST duty DOWN -> ");
        Serial.println(mistDuty);
        if (mistOn) applyMist();
        break;

      case 's':
        mistDuty = min(255, mistDuty + STEP);
        Serial.print("MIST duty UP -> ");
        Serial.println(mistDuty);
        if (mistOn) applyMist();
        break;

      // ===== HEATER power adjust =====
      case 'z':
        heaterDuty = max(0, heaterDuty - STEP);
        Serial.print("HEATER duty DOWN -> ");
        Serial.println(heaterDuty);
        if (heaterOn) applyHeater();
        break;

      case 'x':
        heaterDuty = min(255, heaterDuty + STEP);
        Serial.print("HEATER duty UP -> ");
        Serial.println(heaterDuty);
        if (heaterOn) applyHeater();
        break;

      case 'p':
        printPowers();
        break;

      default:
        Serial.print("Unknown cmd: ");
        Serial.println(cmd);
        break;
    }
  }
}