// === Smart Gasoline System (ESP32) with Blynk ===
#define BLYNK_TEMPLATE_ID "TMPL6pNpL5Rt9"
#define BLYNK_TEMPLATE_NAME "CAO CPE"
#define BLYNK_AUTH_TOKEN "cSNGB0kTOmdoipCfMhB50dAUOsBNZQJj"

#include <WiFi.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// === LCD Setup ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === Pin Definitions ===
#define TRIG_PIN        5
#define ECHO_PIN        2
#define SERVO_PIN       25
#define PAY_BUTTON_PIN  32  
#define FUEL_SENSOR_PIN 34
#define FLAME_SENSOR_PIN 35
#define LED_LOW         14
#define LED_MED         12
#define LED_HIGH        13
#define BUZZER_PIN      27
#define MUTE_BUTTON_PIN 26

// === Globals ===
Servo gateServo;
bool gateOpen = false;
int fuelLevel = 0;
int flameValue = 0;
bool flameDetected = false;
bool buzzerMuted = false;
bool wasLow = false;
unsigned long lastBeepTime = 0;

String gateStatus = "Closed";
String buzzerStatus = "OFF";  // 🔔 NEW for Blynk (V6)
BlynkTimer timer;

// === LCD Priority ===
int lcdPriority = 0; // 0 = Fuel, 1 = Gate, 2 = Flame (highest)

// === Safe LCD Function ===
void showLCDMessage(String line1, String line2, unsigned long holdTime = 1000, int priority = 0) {
  if (priority < lcdPriority) return;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  lcdPriority = priority;
}

// === Ultrasonic Distance ===
long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1; // No reading
  return (long)(duration * 0.0343 / 2.0);
}

// === Send Data to Blynk ===
void sendSensorData() {
  fuelLevel = analogRead(FUEL_SENSOR_PIN);
  flameValue = analogRead(FLAME_SENSOR_PIN);
  long distance = getDistanceCM();

  Blynk.virtualWrite(V1, fuelLevel);
  Blynk.virtualWrite(V2, flameValue);
  Blynk.virtualWrite(V4, gateStatus);
  Blynk.virtualWrite(V6, buzzerStatus); // 🔔 Send buzzer status

  // Car status on V3
  if (distance > 0 && distance < 20)
    Blynk.virtualWrite(V3, String("🚗 Car Detected (") + distance + " cm)");
  else if (distance > 0)
    Blynk.virtualWrite(V3, String("🅿️ No Car (") + distance + " cm)");
  else
    Blynk.virtualWrite(V3, "⚠️ No Reading");
}

// === Flame Detection ===
void detectFlame() {
  const int flameThreshold = 3000;
  flameValue = analogRead(FLAME_SENSOR_PIN);

  if (flameValue <= flameThreshold) {
    if (!flameDetected) {
      flameDetected = true;
      Serial.println("🔥 Flame Detected!");
      showLCDMessage("!! FLAME ALERT !!", "TAKE ACTION NOW!", 1000, 2);
      Blynk.virtualWrite(V5, "🔥 Fire Detected!");
      buzzerStatus = "ON (Fire Alert)";
      Blynk.virtualWrite(V6, buzzerStatus);
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastBeepTime >= 200) {
      lastBeepTime = currentMillis;
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    }
  } 
  else {
    if (flameDetected) {
      flameDetected = false;
      Serial.println("✅ Flame Cleared!");
      digitalWrite(BUZZER_PIN, LOW);
      showLCDMessage("Flame Cleared ✅", "System Normal", 2000, 2);
      Blynk.virtualWrite(V5, "✅ No Fire Detected");
      buzzerStatus = "OFF";
      Blynk.virtualWrite(V6, buzzerStatus);
      lcdPriority = 0;
    }
  }
}

// === Fuel Level & Buzzer ===
void handleFuelLevel() {
  if (flameDetected || gateOpen) return;

  fuelLevel = analogRead(FUEL_SENSOR_PIN);

  if (fuelLevel < 200) {
    // 🔴 LOW
    digitalWrite(LED_LOW, HIGH);
    digitalWrite(LED_MED, LOW);
    digitalWrite(LED_HIGH, LOW);
    showLCDMessage("Fuel Level: LOW", "ALERT: LOW FUEL!", 1000, 0);

    // Blynk LEDs
    Blynk.virtualWrite(V7, 255); // L
    Blynk.virtualWrite(V8, 0);   // M
    Blynk.virtualWrite(V9, 0);   // H

    if (!buzzerMuted) {
      unsigned long currentMillis = millis();
      if (currentMillis - lastBeepTime >= 600) {
        lastBeepTime = currentMillis;
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
        buzzerStatus = "ON (Low Fuel)";
        Blynk.virtualWrite(V6, buzzerStatus);
      }
    } else {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerStatus = "OFF (Muted)";
      Blynk.virtualWrite(V6, buzzerStatus);
    }

    wasLow = true;
  }

  else if (fuelLevel < 700) {
    // 🟡 MEDIUM
    digitalWrite(LED_LOW, HIGH);
    digitalWrite(LED_MED, HIGH);
    digitalWrite(LED_HIGH, LOW);
    showLCDMessage("Fuel Level: MID", "System Normal", 1000, 0);

    // Blynk LEDs
    Blynk.virtualWrite(V7, 255);
    Blynk.virtualWrite(V8, 255);
    Blynk.virtualWrite(V9, 0);

    if (wasLow) { buzzerMuted = false; wasLow = false; }
    digitalWrite(BUZZER_PIN, LOW);
    buzzerStatus = "OFF";
    Blynk.virtualWrite(V6, buzzerStatus);
  }

  else {
    // 🟢 HIGH
    digitalWrite(LED_LOW, HIGH);
    digitalWrite(LED_MED, HIGH);
    digitalWrite(LED_HIGH, HIGH);
    showLCDMessage("Fuel Level: HIGH", "System Normal", 1000, 0);

    // Blynk LEDs
    Blynk.virtualWrite(V7, 255);
    Blynk.virtualWrite(V8, 255);
    Blynk.virtualWrite(V9, 255);

    if (wasLow) { buzzerMuted = false; wasLow = false; }
    digitalWrite(BUZZER_PIN, LOW);
    buzzerStatus = "OFF";
    Blynk.virtualWrite(V6, buzzerStatus);
  }

  // Mute button
  if (digitalRead(MUTE_BUTTON_PIN) == LOW && wasLow) {
    buzzerMuted = true;
    digitalWrite(BUZZER_PIN, LOW);
    showLCDMessage("Buzzer Muted", "Low Fuel Alert!", 1500, 0);
    buzzerStatus = "OFF (Muted)";
    Blynk.virtualWrite(V6, buzzerStatus);
  }
}


// === Payment & Gate Control ===
void handlePaymentGate() {
  static unsigned long gateOpenTime = 0;
  static unsigned long carClearTime = 0;
  static bool carPreviouslyDetected = false;

  if (digitalRead(PAY_BUTTON_PIN) == LOW && !gateOpen) {
    Serial.println("💰 Payment received! Opening gate...");
    gateStatus = "Opening...";
    Blynk.virtualWrite(V4, gateStatus);
    gateServo.write(90);
    delay(500);
    gateOpen = true;
    gateOpenTime = millis();
    gateStatus = "Open";
    Blynk.virtualWrite(V4, gateStatus);
    showLCDMessage("Payment Received", "Gate: OPEN", 1000, 1);
  }

  if (gateOpen) {
    long distance = getDistanceCM();
    unsigned long elapsed = millis() - gateOpenTime;
    bool carDetected = (distance > 0 && distance < 20);

    if (carDetected) {
      carPreviouslyDetected = true;
      carClearTime = 0;
      if (elapsed >= 10000) {
        Serial.println("🚗 Car detected — keeping gate open...");
        showLCDMessage("Car Detected 🚗", "Gate: OPEN HOLD", 1000, 1);
      }
    } else {
      if (carPreviouslyDetected && carClearTime == 0) {
        carClearTime = millis();
        Serial.println("⏳ Car moved away — waiting 3 seconds...");
        showLCDMessage("Car Leaving...", "Closing Soon", 1000, 1);
      }

      if (carClearTime > 0 && millis() - carClearTime >= 3000) {
        Serial.println("⛔ No car detected — closing gate...");
        gateStatus = "Closing...";
        Blynk.virtualWrite(V4, gateStatus);
        gateServo.write(0);
        delay(500);
        gateOpen = false;
        carPreviouslyDetected = false;
        carClearTime = 0;
        gateStatus = "Closed";
        Blynk.virtualWrite(V4, gateStatus);
        showLCDMessage("Gate Closed", "System Normal", 1000, 1);
        lcdPriority = 0;
      }
    }
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  showLCDMessage("Smart Gasoline", "Starting...", 1200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FUEL_SENSOR_PIN, INPUT);
  pinMode(FLAME_SENSOR_PIN, INPUT);
  pinMode(LED_LOW, OUTPUT);
  pinMode(LED_MED, OUTPUT);
  pinMode(LED_HIGH, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MUTE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PAY_BUTTON_PIN, INPUT_PULLUP);

  gateServo.attach(SERVO_PIN);
  gateServo.write(0);

  WiFiManager wifiManager;
  wifiManager.setTimeout(30);
  if (!wifiManager.autoConnect("ESP32_Setup", "12345678")) {
    Serial.println("⚠️ WiFi failed. Restarting...");
    showLCDMessage("WiFi Failed", "Restarting...", 2000, 0);
    ESP.restart();
  }

  Serial.println("✅ WiFi connected!");
  Serial.println(WiFi.localIP());

  Blynk.config(BLYNK_AUTH_TOKEN);
  if (!Blynk.connect()) {
    Serial.println("⚠️ Retrying Blynk...");
    for (int i = 0; i < 5 && !Blynk.connected(); i++) {
      delay(2000);
      Blynk.connect();
    }
  }

  if (Blynk.connected()) Serial.println("✅ Connected to Blynk!");
  else Serial.println("❌ Still not connected.");

  Blynk.virtualWrite(V6, "OFF"); // Initialize buzzer status
  timer.setInterval(2000L, sendSensorData);
}

// === Loop ===
void loop() {
  Blynk.run();
  timer.run();

  detectFlame();
  handlePaymentGate();

  if (!flameDetected && !gateOpen)
    handleFuelLevel();

  delay(200);
}


