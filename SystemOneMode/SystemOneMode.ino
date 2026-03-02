#include <Arduino.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <DHT.h>

#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================== BLYNK ==================
#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID   "TMPL6ihUzZBCn"
#define BLYNK_TEMPLATE_NAME "HiveCare"
#define BLYNK_AUTH_TOKEN    "vBp4gRk6qA_5plwEi_fEZBaHwZrwWymI"
#include <BlynkSimpleEsp32.h>
BlynkTimer blynkTimer;

// ================== LCD ==================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================== SERVER ==================
const char* SERVER_URL = "http://192.168.100.224/thesis/Beehive/database/sensor_insert.php";
const unsigned long NORMAL_SEND_INTERVAL_MS = 15UL * 60UL * 1000UL;

// ================== ACTUATORS ==================
#define FAN_PIN    25
#define MIST_PIN   13
#define HEATER_PIN 26

// ================== SAFE RANGE ==================
const float TEMP_MIN = 25.0;
const float TEMP_MAX = 35.0;
const float HUM_MIN  = 70.0;
const float HUM_MAX  = 90.0;

// OFF thresholds
const float TEMP_HYS = 0.5;  // 34.5 / 25.5
const float HUM_HYS  = 0.5;  // 89.5

// ================== MIST ==================
int mistSpeedStrong = 255;

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

// ================== STATE ==================
unsigned long lastPrint = 0;
unsigned long lastDHT   = 0;
unsigned long lastLcd   = 0;

unsigned long nextSendDue = 0;
bool lastOutOfRange = false;

float temperature = NAN;
float humidity    = NAN;
float latestWeightKg = NAN;

String fanStatus    = "OFF";
String mistStatus   = "OFF";
String heaterStatus = "OFF";

int fanDuty = 0;
int mistDuty = 0;
int heaterDuty = 0;

bool mistEnabled = false;

bool popupActive = false;
unsigned long popupUntil = 0;

String lastFanStatus = "";
String lastMistStatus = "";
String lastHeaterStatus = "";

// ================== LCD HELPERS ==================
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

void lcdPopup(const String& l1, const String& l2, unsigned long ms = 1500) {
  popupActive = true;
  popupUntil = millis() + ms;
  lcdShow2Lines(l1, l2);
}

void lcdMain(float t, float h, float w) {
  String l1 = "T:" + String(t,1) + " H:" + String(h,0);
  String l2 = "W:" + String(w,2) + "kg F" + String(fanDuty>0) + "M" + String(mistDuty>0) + "H" + String(heaterDuty>0);
  lcdShow2Lines(l1, l2);
}

// ================== APPLY ACTUATORS ==================
void applyActuators(int f, int m, int h) {
  fanDuty = constrain(f,0,255);
  mistDuty = constrain(m,0,255);
  heaterDuty = constrain(h,0,255);

  ledcWrite(FAN_PIN, fanDuty);
  ledcWrite(MIST_PIN, mistDuty);
  ledcWrite(HEATER_PIN, heaterDuty);

  fanStatus = fanDuty>0?"ON":"OFF";
  mistStatus = mistDuty>0?"ON":"OFF";
  heaterStatus = heaterDuty>0?"ON":"OFF";
}

// ================== DHT SAFE ==================
void readDHTSafe() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if(!isnan(h)) humidity = h;
  if(!isnan(t)) temperature = t;
}

// ================== RANGE CHECK ==================
bool isOutOfRange(float t, float h){
  return (t < TEMP_MIN || t > TEMP_MAX || h < HUM_MIN || h > HUM_MAX);
}

// ================== AUTO CONTROL ==================
void autoControl() {
  int f=0,m=0,h=0;

  static bool heaterOn=false;
  if(!heaterOn && temperature < TEMP_MIN) heaterOn=true;
  else if(heaterOn && temperature >= TEMP_MIN+TEMP_HYS) heaterOn=false;
  h = heaterOn?255:0;

  static bool fanOn=false;
  bool tooHot = temperature > TEMP_MAX;
  bool hotClear = temperature <= TEMP_MAX-TEMP_HYS;
  bool tooHumid = humidity > HUM_MAX;
  bool humidClear = humidity <= HUM_MAX-HUM_HYS;

  if(!fanOn && (tooHot||tooHumid)) fanOn=true;
  else if(fanOn && hotClear && humidClear) fanOn=false;
  f = fanOn?255:0;

  bool tooDry = humidity < HUM_MIN;
  bool dryClear = humidity >= HUM_MIN+HUM_HYS;
  bool coolingAssist = (temperature > TEMP_MAX && humidity < HUM_MAX);
  bool coolingClear = temperature <= TEMP_MAX-TEMP_HYS;

  if(humidity > HUM_MAX) mistEnabled=false;
  else{
    if(!mistEnabled && (tooDry||coolingAssist)) mistEnabled=true;
    else if(mistEnabled && ((dryClear && !coolingAssist)||coolingClear)) mistEnabled=false;
  }
  m = mistEnabled?mistSpeedStrong:0;

  applyActuators(f,m,h);
}

// ================== SEND DB ==================
void sendDB(){
  if(WiFi.status()!=WL_CONNECTED) return;
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type","application/x-www-form-urlencoded");
  String data="temperature="+String(temperature,2)+
              "&humidity="+String(humidity,2)+
              "&weight="+String(latestWeightKg,2)+
              "&fan_status="+fanStatus+
              "&pump_status="+mistStatus+
              "&heater_status="+heaterStatus;
  http.POST(data);
  http.end();
}

// ================== SETUP ==================
void setup(){
  Serial.begin(115200);
  Wire.begin(21,22);
  lcd.init(); lcd.backlight();
  lcdShow2Lines("HiveCare","Booting...");

  ledcAttach(FAN_PIN,5000,8);
  ledcAttach(MIST_PIN,5000,8);
  ledcAttach(HEATER_PIN,5000,8);
  applyActuators(0,0,0);

  WiFiManager wm;
  wm.autoConnect("ESP32_Setup","12345678");

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(10000);

  EEPROM.begin(EEPROM_SIZE);
  LoadCell.begin();
  LoadCell.start(2000,false);
  LoadCell.setCalFactor(CAL_FACTOR);
  LoadCell.tare();

  dht.begin();

  nextSendDue = millis()+NORMAL_SEND_INTERVAL_MS;
  lastOutOfRange = isOutOfRange(temperature,humidity);

  lcdShow2Lines("System Ready","");
}

// ================== LOOP ==================
void loop(){
  Blynk.run();
  LoadCell.update();

  if(millis()-lastDHT>=3000){
    lastDHT=millis();
    readDHTSafe();
  }

  latestWeightKg = LoadCell.getData()/1000.0;
  if(latestWeightKg<0.5) latestWeightKg=0;

  autoControl();

  if(millis()-lastPrint>=1000){
    lastPrint=millis();
    Serial.print("T:");Serial.print(temperature);
    Serial.print(" H:");Serial.print(humidity);
    Serial.print(" W:");Serial.println(latestWeightKg);
  }

  if(!popupActive || millis()>popupUntil){
    popupActive=false;
    if(millis()-lastLcd>=3000){
      lastLcd=millis();
      lcdMain(temperature,humidity,latestWeightKg);
    }
  }

  unsigned long now=millis();
  bool outNow=isOutOfRange(temperature,humidity);
  bool wentOut=!lastOutOfRange && outNow;
  bool wentBack=lastOutOfRange && !outNow;
  bool periodic=now>=nextSendDue;

  if(wentOut||wentBack||periodic){
    sendDB();
    nextSendDue=now+NORMAL_SEND_INTERVAL_MS;
  }
  lastOutOfRange=outNow;
}