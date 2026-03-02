#include <Arduino.h>
#include "HX711.h"

// ===== HX711 Pins (change if needed) =====
#define HX711_DOUT  4
#define HX711_SCK   5

HX711 scale;

// ===== Calibration values =====
// You will change CALIBRATION_FACTOR after calibration
float CALIBRATION_FACTOR = -7050.0; // sample value (will vary per load cell)
float knownWeightGrams = 500.0;     // used in calibration routine

// ===== Simple smoothing =====
const int SAMPLES = 10;

float readAverageGrams() {
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += scale.get_units(1); // 1 reading each loop
    delay(30);
  }
  return (float)sum / SAMPLES;
}

void printHelp() {
  Serial.println("\n=== HX711 ESP32 Commands ===");
  Serial.println("t  -> tare (set current as zero)");
  Serial.println("r  -> read weight (grams)");
  Serial.println("c  -> calibrate using knownWeightGrams");
  Serial.println("f  -> show calibration factor");
  Serial.println("k  -> change knownWeightGrams (example: k1000)");
  Serial.println("===========================\n");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("Starting HX711...");
  scale.begin(HX711_DOUT, HX711_SCK);

  // Optional: set gain (128 default). Some libraries use scale.set_gain(128);
  // scale.set_gain(128);

  // Set initial calibration factor
  scale.set_scale(CALIBRATION_FACTOR);

  Serial.println("Taring... Remove all weight from the scale.");
  delay(1500);
  scale.tare(); // set zero
  Serial.println("Tare done.");

  printHelp();
}

void loop() {
  // If HX711 not ready, skip
  if (!scale.is_ready()) {
    Serial.println("HX711 not ready. Check wiring/power.");
    delay(500);
    return;
  }

  // Live reading every 1 sec (even without commands)
  float grams = readAverageGrams();
  Serial.print("Weight: ");
  Serial.print(grams, 2);
  Serial.println(" g");
  delay(1000);

  // Command handler (non-blocking-ish)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "t") {
      Serial.println("Taring... remove weight.");
      delay(1000);
      scale.tare();
      Serial.println("Tare done.");

    } else if (cmd == "r") {
      float g = readAverageGrams();
      Serial.print("Reading: ");
      Serial.print(g, 2);
      Serial.println(" g");

    } else if (cmd == "f") {
      Serial.print("Current factor: ");
      Serial.println(CALIBRATION_FACTOR, 6);

    } else if (cmd.startsWith("k")) {
      // example: k1000 sets known weight to 1000g
      String val = cmd.substring(1);
      knownWeightGrams = val.toFloat();
      Serial.print("Known weight set to: ");
      Serial.print(knownWeightGrams);
      Serial.println(" g");

    }else if (cmd == "c") {
  Serial.println("\n=== CALIBRATION MODE ===");
  Serial.println("Step 1: Remove ALL weight.");
  Serial.println("Type: go  (then press Enter)");
  while (true) {
    while (!Serial.available()) delay(10);
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "go") break;
  }

  scale.tare();
  Serial.println("Tare done.");

  Serial.print("Step 2: Place the KNOWN weight now (g): ");
  Serial.println(knownWeightGrams);
  Serial.println("Type: go  (then press Enter)");
  while (true) {
    while (!Serial.available()) delay(10);
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "go") break;
  }

  // Read raw average (independent of scale factor)
  long raw = scale.read_average(20) - scale.get_offset();
  Serial.print("Raw units (net): ");
  Serial.println(raw);

  float newFactor = (float)raw / knownWeightGrams;

  Serial.print("NEW CALIBRATION FACTOR: ");
  Serial.println(newFactor, 6);

  Serial.println("Applying new factor now...");
  CALIBRATION_FACTOR = newFactor;
  scale.set_scale(CALIBRATION_FACTOR);

  Serial.println("Done. Type r to check reading.\n");
}


    } else {
      printHelp();
    }
  }

