/*
Environmental Brain Cube - Environment Determining Mode

This mode runs the cube for a set time, logs data to the SD card,
then compares the full run average BME temperature, humidity,
and pressure to a few saved environment profiles.

What it does:
- reads sensors live
- shows values on the OLED
- blinks LEDs and buzzer for warnings
- logs min/max data to the SD card
- picks the closest environment match at the end

Log file:
- /ebc_modes_log.csv
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// scale factors for the gravity(noraml_g) and gyroscope(noraml_gyro)
float normal_g = 1671.0;  
float normal_gyro = 131.0;

// run time (use 0 if you want it to run forever)
const unsigned long RUN_DURATION_SEC = 300;  

// time trackers
unsigned long runStartTime = 0;
unsigned long lastFastRead = 0;
unsigned long lastBMERead = 0;
unsigned long lastDSRead = 0;
unsigned long lastOLEDRead = 0;
unsigned long lastSDWrite = 0;

bool runFinished = false;
bool finalLogWritten = false;
bool finalSummaryWritten = false;

// pins
const int I2C_SDA = 21;
const int I2C_SCL = 22;

const int PIN_DS18B20 = 14;
const int PIN_LDR = 34;
const int PIN_BATTERY = 35;
const int PIN_SOLAR = 13;

const int LED_RED = 32;
const int LED_BLUE = 25;
const int LED_GREEN = 33;

const int PIN_BUZZER = 26;

const int SD_CS = 5;
const int SD_SCK = 18;
const int SD_MISO = 19;
const int SD_MOSI = 23;

// oled
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// sensors
OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

Adafruit_BME280 bme;
MPU6050 mpu;

// adc references. 3.3 is pin voltage for the esp32. 4095 is max adc reading
const float ADC_REF = 3.3;
const float ADC_MAX = 4095.0;

// battery divider (this setup allows for the system to safely handle about 10V)
// the 9V battery will result in a 2.9V input into the pin
const float R1_BATT = 100000.0;
const float R2_BATT = 46800.0;
const float BATT_CAL = 0.959;     //calibration value for the battery

// solar divider (this setup allows for the system to safely handle about 10V)
const float R1_SOLAR = 100500.0;
const float R2_SOLAR = 46800.0;
const float SOLAR_CAL = 1.167;     //calibration value for the solar

// warning limits
const int LIGHT_DARK_TH = 1000;
const int LIGHT_BRIGHT_TH = 3800;

const float TEMP_WARN_LOW = 4.0;
const float TEMP_WARN_HIGH = 32.0;

const float HUM_WARN_LOW = 15.0;
const float HUM_WARN_HIGH = 85.0;

const float PRES_WARN_LOW = 990.0;
const float PRES_WARN_HIGH = 1025.0;

const float DS_TEMP_HIGH_ALARM_C = 45.0;

// timing
const unsigned long FAST_INTERVAL_MS = 100;
const unsigned long BME_INTERVAL_MS = 500;
const unsigned long DS_INTERVAL_MS = 1000;
const unsigned long OLED_INTERVAL_MS = 250;
const unsigned long SD_INTERVAL_MS = 2000;

// latest values
float bmeTemp = NAN;
float bmeHum = NAN;
float bmePres = NAN;
float dsTemp = NAN;
int lightRaw = 0;

int batteryRaw = 0;
float batteryAdcVoltage = 0.0;
float batteryVoltage = 0.0;

int solarRaw = 0;
float solarAdcVoltage = 0.0;
float solarVoltage = 0.0;

float ax_g = 0;
float ay_g = 0;
float az_g = 0;

float gx_dps = 0;
float gy_dps = 0;
float gz_dps = 0;

// alarms right now
bool bmeTempOutNow = false;
bool dsTempHighNow = false;
bool humOutNow = false;
bool presOutNow = false;

// sd
bool sdOK = false;
const char* logFile = "/ebc_modes_log.csv";
int currentRunNumber = 1;

// blink helper
struct BlinkState {
  bool on = false;
  unsigned long lastToggle = 0;
};

BlinkState blinkR;
BlinkState blinkB;
BlinkState blinkG;

const unsigned long BLINK_MS = 200;

// stat tracker
struct StatTracker {
  float minVal;
  float maxVal;
  float sumVal;
  unsigned long count;
};

// interval stats
StatTracker st_light;
StatTracker st_bmeTemp;
StatTracker st_bmeHum;
StatTracker st_bmePres;
StatTracker st_dsTemp;
StatTracker st_batt;
StatTracker st_solar;
StatTracker st_ax;
StatTracker st_ay;
StatTracker st_az;
StatTracker st_gx;
StatTracker st_gy;
StatTracker st_gz;

// full run summary stats
StatTracker run_bmeTemp;
StatTracker run_bmeHum;
StatTracker run_bmePres;

// final summary
char finalBestEnv[20] = "UNKNOWN";
char finalRating[16] = "UNKNOWN";
float finalAvgTemp = NAN;
float finalAvgHum = NAN;
float finalAvgPres = NAN;
int finalMatchCount = 0;

// environment profiles
struct EnvProfile {
  const char* name;
  float tempMin;
  float tempMax;
  float humMin;
  float humMax;
  float presMin;
  float presMax;
};

//========= IF YOU CHANGE THESE VALUES, CHANGE THEM IN MATLAB SCRIPT AS WELL =======//
EnvProfile envProfiles[] = {
  {"NORMAL", 20.0, 24.0, 30.0, 50.0, 995.0, 1025.0},
  {"WARM_HUMID", 26.0, 32.0, 60.0, 70.0, 995.0, 1025.0},
  {"WARM_DRY", 24.0, 30.0, 15.0, 35.0, 995.0, 1025.0},
  {"COLD", 4.0, 12.0, 35.0, 65.0, 995.0, 1025.0}
};

const int ENV_COUNT = sizeof(envProfiles) / sizeof(envProfiles[0]);

void resetStats(StatTracker &s) {
  s.minVal = 9999999.0;
  s.maxVal = -9999999.0;
  s.sumVal = 0.0;
  s.count = 0;
}

void updateStats(StatTracker &s, float v) {
  if (isnan(v)) {
    return;
  }

  if (v < s.minVal) {
    s.minVal = v;
  }

  if (v > s.maxVal) {
    s.maxVal = v;
  }

  s.sumVal += v;
  s.count++;
}

bool hasStats(const StatTracker &s) {
  return s.count > 0;
}

float getAvg(const StatTracker &s) {
  if (hasStats(s)) {
    return s.sumVal / (float)s.count;
  }
  return NAN;
}

void resetAllStats() {
  resetStats(st_light);
  resetStats(st_bmeTemp);
  resetStats(st_bmeHum);
  resetStats(st_bmePres);
  resetStats(st_dsTemp);
  resetStats(st_batt);
  resetStats(st_solar);
  resetStats(st_ax);
  resetStats(st_ay);
  resetStats(st_az);
  resetStats(st_gx);
  resetStats(st_gy);
  resetStats(st_gz);
}

void resetRunSummaryStats() {
  resetStats(run_bmeTemp);
  resetStats(run_bmeHum);
  resetStats(run_bmePres);
}

void stopBuzzer() {
  ledcWriteTone(PIN_BUZZER, 0);
}

void stopOutputs() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
  stopBuzzer();
}

void blinkLED(int pin, bool alarm, BlinkState &st, unsigned long now) {
  if (!alarm) {
    digitalWrite(pin, LOW);
    st.on = false;
    return;
  }

  if (now - st.lastToggle >= BLINK_MS) {
    st.lastToggle = now;
    st.on = !st.on;
    digitalWrite(pin, st.on ? HIGH : LOW);
  }
}

const char* getLightState(int raw) {
  if (raw <= LIGHT_DARK_TH) {
    return "DARK";
  }

  if (raw < LIGHT_BRIGHT_TH) {
    return "NORMAL";
  }

  return "BRIGHT";
}

float readBatteryVoltage() {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(PIN_BATTERY);
    delay(2);
  }

  batteryRaw = sum / 10.0;
  batteryAdcVoltage = (batteryRaw * ADC_REF) / ADC_MAX;
  batteryVoltage = batteryAdcVoltage * ((R1_BATT + R2_BATT) / R2_BATT) * BATT_CAL;

  return batteryVoltage;
}

float readSolarVoltage() {
  long sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += analogRead(PIN_SOLAR);
    delay(2);
  }

  solarRaw = sum / 10.0;
  solarAdcVoltage = (solarRaw * ADC_REF) / ADC_MAX;
  solarVoltage = solarAdcVoltage * ((R1_SOLAR + R2_SOLAR) / R2_SOLAR) * SOLAR_CAL;

  return solarVoltage;
}

void printStatOrNA(File &file, const StatTracker &s, int decimals) {
  if (hasStats(s)) {
    file.print(s.minVal, decimals);
  } else {
    file.print("NA");
  }

  file.print(",");

  if (hasStats(s)) {
    file.print(s.maxVal, decimals);
  } else {
    file.print("NA");
  }
}

int extractRunNumber(String line) {
  line.trim();

  if (!line.startsWith("RUN")) {
    return -1;
  }

  int commaIndex = line.indexOf(',');
  if (commaIndex == -1) {
    return -1;
  }

  String runText = line.substring(3, commaIndex);
  runText.trim();

  if (runText.length()) {
    return runText.toInt();
  }

  return -1;
}

int findNextRunNumber() {
  if (!SD.exists(logFile)) {
    return 1;
  }

  File file = SD.open(logFile, FILE_READ);
  if (!file) {
    return 1;
  }

  int lastRun = 0;

  while (file.available()) {
    int runNum = extractRunNumber(file.readStringUntil('\n'));
    if (runNum > lastRun) {
      lastRun = runNum;
    }
  }

  file.close();
  return lastRun + 1;
}

void writeRunSeparator() {
  if (!sdOK) {
    return;
  }

  File file = SD.open(logFile, FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open log file for run separator");
    return;
  }

  file.println();
  file.print("=== RUN");
  file.print(currentRunNumber);
  file.println(" START ===");

  file.println(
    "runLabel,millis,elapsedSec,"
    "lightMin,lightMax,lightState,"
    "bmeTempMin,bmeTempMax,"
    "bmeHumMin,bmeHumMax,"
    "bmePresMin,bmePresMax,"
    "dsTempMin,dsTempMax,"
    "battMin,battMax,"
    "solarMin,solarMax,"
    "axMin,axMax,"
    "ayMin,ayMax,"
    "azMin,azMax,"
    "gxMin,gxMax,"
    "gyMin,gyMax,"
    "gzMin,gzMax"
  );

  file.close();

  Serial.print("Run separator written for RUN");
  Serial.println(currentRunNumber);
}

bool valueInRange(float v, float vMin, float vMax) {
  return (!isnan(v) && v >= vMin && v <= vMax);
}

float rangePenalty(float v, float vMin, float vMax) {
  if (isnan(v)) {
    return 999999.0;
  }

  float span = vMax - vMin;
  if (span <= 0.0) {
    span = 1.0;
  }

  if (v < vMin) {
    return (vMin - v) / span;
  }

  if (v > vMax) {
    return (v - vMax) / span;
  }

  float center = (vMin + vMax) / 2.0;
  float halfSpan = span / 2.0;

  if (halfSpan <= 0.0) {
    halfSpan = 1.0;
  }

  return fabs(v - center) / halfSpan;
}

const char* getRatingText(int matchCount) {
  if (matchCount >= 3) {
    return "VERY GOOD";
  }

  if (matchCount == 2) {
    return "GOOD";
  }

  if (matchCount == 1) {
    return "POOR";
  }

  return "VERY POOR";
}

void evaluateEnvironmentFit() {
  finalAvgTemp = getAvg(run_bmeTemp);
  finalAvgHum = getAvg(run_bmeHum);
  finalAvgPres = getAvg(run_bmePres);

  int bestIndex = 0;
  int bestScore = -1;
  float bestPenalty = 999999.0;

  for (int i = 0; i < ENV_COUNT; i++) {
    int score = 0;

    if (valueInRange(finalAvgTemp, envProfiles[i].tempMin, envProfiles[i].tempMax)) {
      score++;
    }

    if (valueInRange(finalAvgHum, envProfiles[i].humMin, envProfiles[i].humMax)) {
      score++;
    }

    if (valueInRange(finalAvgPres, envProfiles[i].presMin, envProfiles[i].presMax)) {
      score++;
    }

    float penalty = 0.0;
    penalty += rangePenalty(finalAvgTemp, envProfiles[i].tempMin, envProfiles[i].tempMax);
    penalty += rangePenalty(finalAvgHum, envProfiles[i].humMin, envProfiles[i].humMax);
    penalty += rangePenalty(finalAvgPres, envProfiles[i].presMin, envProfiles[i].presMax);

    if (score > bestScore || (score == bestScore && penalty < bestPenalty)) {
      bestScore = score;
      bestPenalty = penalty;
      bestIndex = i;
    }
  }

  strncpy(finalBestEnv, envProfiles[bestIndex].name, sizeof(finalBestEnv) - 1);
  finalBestEnv[sizeof(finalBestEnv) - 1] = '\0';

  finalMatchCount = bestScore;

  strncpy(finalRating, getRatingText(bestScore), sizeof(finalRating) - 1);
  finalRating[sizeof(finalRating) - 1] = '\0';
}

int findBestEnvIndex() {
  for (int i = 0; i < ENV_COUNT; i++) {
    if (strcmp(finalBestEnv, envProfiles[i].name) == 0) {
      return i;
    }
  }

  return 0;
}

void printFinalSummaryToSerial() {
  int bestIndex = findBestEnvIndex();

  bool tempMatch = valueInRange(finalAvgTemp, envProfiles[bestIndex].tempMin, envProfiles[bestIndex].tempMax);
  bool humMatch = valueInRange(finalAvgHum, envProfiles[bestIndex].humMin, envProfiles[bestIndex].humMax);
  bool presMatch = valueInRange(finalAvgPres, envProfiles[bestIndex].presMin, envProfiles[bestIndex].presMax);

  Serial.println();
  Serial.println("===== FINAL RUN SUMMARY =====");
  Serial.print("RUN");
  Serial.println(currentRunNumber);

  Serial.print("Best environment fit: ");
  Serial.println(finalBestEnv);

  Serial.print("Rating: ");
  Serial.println(finalRating);

  Serial.print("Average BME Temp: ");
  Serial.print(finalAvgTemp, 2);
  Serial.print(" C  | target ");
  Serial.print(envProfiles[bestIndex].tempMin, 1);
  Serial.print(" to ");
  Serial.print(envProfiles[bestIndex].tempMax, 1);
  Serial.print(" C  -> ");
  Serial.println(tempMatch ? "PASS" : "FAIL");

  Serial.print("Average Humidity: ");
  Serial.print(finalAvgHum, 2);
  Serial.print(" %  | target ");
  Serial.print(envProfiles[bestIndex].humMin, 1);
  Serial.print(" to ");
  Serial.print(envProfiles[bestIndex].humMax, 1);
  Serial.print(" %  -> ");
  Serial.println(humMatch ? "PASS" : "FAIL");

  Serial.print("Average Pressure: ");
  Serial.print(finalAvgPres, 2);
  Serial.print(" hPa  | target ");
  Serial.print(envProfiles[bestIndex].presMin, 1);
  Serial.print(" to ");
  Serial.print(envProfiles[bestIndex].presMax, 1);
  Serial.print(" hPa  -> ");
  Serial.println(presMatch ? "PASS" : "FAIL");

  Serial.print("Match count: ");
  Serial.print(finalMatchCount);
  Serial.println("/3");
  Serial.println("=============================");
  Serial.println();
}

void writeFinalSummaryToSD() {
  if (!sdOK) {
    return;
  }

  int bestIndex = findBestEnvIndex();

  bool tempMatch = valueInRange(finalAvgTemp, envProfiles[bestIndex].tempMin, envProfiles[bestIndex].tempMax);
  bool humMatch = valueInRange(finalAvgHum, envProfiles[bestIndex].humMin, envProfiles[bestIndex].humMax);
  bool presMatch = valueInRange(finalAvgPres, envProfiles[bestIndex].presMin, envProfiles[bestIndex].presMax);

  File file = SD.open(logFile, FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open log file for final summary");
    return;
  }

  file.println();
  file.print("=== RUN");
  file.print(currentRunNumber);
  file.println(" SUMMARY ===");

  file.println("summaryType,bestEnvironment,rating,avgBmeTempC,avgHumPct,avgPreshPa,tempPass,humPass,presPass,matchCount");
  file.print("ENV_FIT,");
  file.print(finalBestEnv);
  file.print(",");
  file.print(finalRating);
  file.print(",");
  file.print(finalAvgTemp, 2);
  file.print(",");
  file.print(finalAvgHum, 2);
  file.print(",");
  file.print(finalAvgPres, 2);
  file.print(",");
  file.print(tempMatch ? "PASS" : "FAIL");
  file.print(",");
  file.print(humMatch ? "PASS" : "FAIL");
  file.print(",");
  file.print(presMatch ? "PASS" : "FAIL");
  file.print(",");
  file.println(finalMatchCount);

  file.close();
}

void updateBuzzer(unsigned long now, bool tempAlarm, bool humAlarm, bool presAlarm) {
  bool anyAlarm = tempAlarm || humAlarm || presAlarm;

  if (!anyAlarm) {
    stopBuzzer();
    return;
  }

  int toneFreq = 1800;

  if (tempAlarm) {
    toneFreq = 2200;
  } else if (humAlarm) {
    toneFreq = 1800;
  } else if (presAlarm) {
    toneFreq = 1500;
  }

  const unsigned long beepOnMs = 150;
  const unsigned long beepPeriodMs = 350;

  if ((now % beepPeriodMs) < beepOnMs) {
    ledcWriteTone(PIN_BUZZER, toneFreq);
  } else {
    stopBuzzer();
  }
}

void bootScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 8);
  display.println("ENV");
  display.setCursor(0, 30);
  display.println("BRAIN CUBE");

  display.setTextSize(1);
  display.setCursor(28, 54);
  display.println("WELCOME");

  display.display();
  delay(2000);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20, 10);
  display.println("Booting System");
  display.drawRect(14, 30, 100, 10, SSD1306_WHITE);
  display.display();

  for (int i = 0; i <= 96; i += 6) {
    display.fillRect(16, 32, i, 6, SSD1306_WHITE);
    display.display();
    delay(120);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(24, 20);
  display.println("READY");
  display.display();
  delay(1500);
}

void showFinishedSummaryScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("RUN");
  display.print(currentRunNumber);
  display.println(" COMPLETE");

  display.setCursor(0, 12);
  display.print("Best:");
  display.print(finalBestEnv);

  display.setCursor(0, 24);
  display.print("Rate:");
  display.print(finalRating);

  display.setCursor(0, 36);
  display.print("T:");
  if (!isnan(finalAvgTemp)) {
    display.print(finalAvgTemp, 1);
  } else {
    display.print("--");
  }
  display.print("C H:");
  if (!isnan(finalAvgHum)) {
    display.print(finalAvgHum, 0);
  } else {
    display.print("--");
  }
  display.print("%");

  display.setCursor(0, 48);
  display.print("P:");
  if (!isnan(finalAvgPres)) {
    display.print(finalAvgPres, 0);
  } else {
    display.print("--");
  }
  display.print(" M:");
  display.print(finalMatchCount);
  display.print("/3");

  display.display();
}

void updateOLED(unsigned long now) {
  float elapsedSec = (now - runStartTime) / 1000.0;
  bool flashOn = ((now / 300) % 2 == 0);

  bool tempAlarm = bmeTempOutNow || dsTempHighNow;
  bool humAlarm = humOutNow;
  bool presAlarm = presOutNow;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("T:");
  if (!isnan(bmeTemp)) {
    if (tempAlarm && !flashOn) {
      display.print("   ");
    } else {
      display.print(bmeTemp, 1);
      display.print("C");
    }
  } else {
    display.print("--");
  }

  display.print(" DS:");
  if (!isnan(dsTemp)) {
    if (tempAlarm && !flashOn) {
      display.print("   ");
    } else {
      display.print(dsTemp, 1);
      display.print("C");
    }
  } else {
    display.print("--");
  }

  display.setCursor(90, 0);
  display.print("   R");
  display.print(currentRunNumber);

  display.setCursor(0, 10);
  display.print("H:");
  if (!isnan(bmeHum)) {
    if (humAlarm && !flashOn) {
      display.print("   ");
    } else {
      display.print(bmeHum, 0);
      display.print("%");
    }
  } else {
    display.print("--");
  }

  display.print(" P:");
  if (!isnan(bmePres)) {
    if (presAlarm && !flashOn) {
      display.print("    ");
    } else {
      display.print(bmePres, 0);
      display.print("h");
    }
  } else {
    display.print("--");
  }

  display.setCursor(0, 20);
  display.print("Bat:");
  display.print(batteryVoltage, 2);
  display.print("V");

  display.setCursor(64, 20);
  display.print("Sol:");
  display.print(solarVoltage, 2);
  display.print("V");

  display.setCursor(0, 30);
  display.print("L:");
  display.print(getLightState(lightRaw));

  display.setCursor(0, 40);
  display.print("X:");
  display.print(ax_g, 1);
  display.print(" Y:");
  display.print(ay_g, 1);
  display.print(" Z:");
  display.print(az_g, 1);

  display.setCursor(0, 50);
  display.print("t:");
  display.print(elapsedSec, 0);
  display.print("s");

  if (dsTempHighNow) {
    display.setCursor(86, 50);
    display.print("DSHOT");
  } else if (bmeTempOutNow) {
    display.setCursor(88, 50);
    display.print("TEMP");
  } else if (humAlarm) {
    display.setCursor(92, 50);
    display.print("HUM");
  } else if (presAlarm) {
    display.setCursor(88, 50);
    display.print("PRES");
  }

  display.display();
}

void logToSD(unsigned long now) {
  if (!sdOK) {
    return;
  }

  File file = SD.open(logFile, FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open log file for writing");
    return;
  }

  float elapsedSec = (now - runStartTime) / 1000.0;

  file.print("RUN");
  file.print(currentRunNumber);
  file.print(",");
  file.print(now);
  file.print(",");
  file.print(elapsedSec, 2);
  file.print(",");

  printStatOrNA(file, st_light, 0);
  file.print(",");
  file.print(getLightState(lightRaw));
  file.print(",");

  printStatOrNA(file, st_bmeTemp, 2);
  file.print(",");
  printStatOrNA(file, st_bmeHum, 2);
  file.print(",");
  printStatOrNA(file, st_bmePres, 2);
  file.print(",");
  printStatOrNA(file, st_dsTemp, 2);
  file.print(",");
  printStatOrNA(file, st_batt, 2);
  file.print(",");
  printStatOrNA(file, st_solar, 2);
  file.print(",");
  printStatOrNA(file, st_ax, 3);
  file.print(",");
  printStatOrNA(file, st_ay, 3);
  file.print(",");
  printStatOrNA(file, st_az, 3);
  file.print(",");
  printStatOrNA(file, st_gx, 2);
  file.print(",");
  printStatOrNA(file, st_gy, 2);
  file.print(",");
  printStatOrNA(file, st_gz, 2);

  file.println();
  file.close();

  Serial.print("Data logged to SD (min/max only), RUN");
  Serial.println(currentRunNumber);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("=== BOOT ===");

  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("I2C started on SDA=21 SCL=22");

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LDR, ADC_11db);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  analogSetPinAttenuation(PIN_SOLAR, ADC_11db);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  stopOutputs();

  ledcAttach(PIN_BUZZER, 2000, 8);
  stopBuzzer();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed");
    while (1) {
    }
  }

  bootScreen();

  Serial.print("BME280 init at 0x76... ");
  bool bmeOK = bme.begin(0x76, &Wire);
  Serial.println(bmeOK ? "OK" : "FAIL");
  if (!bmeOK) {
    Serial.println("Check wiring: VCC=3.3V, GND, SDA=21, SCL=22");
    while (1) {
    }
  }

  Serial.print("MPU6050 init... ");
  mpu.initialize();
  bool mpuOK = mpu.testConnection();
  Serial.println(mpuOK ? "OK" : "FAIL");
  if (!mpuOK) {
    Serial.println("MPU6050 not found! Check I2C wiring.");
    while (1) {
    }
  }

  ds18b20.begin();
  Serial.println("DS18B20 begin() called");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  Serial.print("SD init... ");

  if (SD.begin(SD_CS)) {
    sdOK = true;
    Serial.println("OK");

    if (!SD.exists(logFile)) {
      File file = SD.open(logFile, FILE_WRITE);
      if (file) {
        file.println("Environmental Brain Cube MODES Log");
        file.close();
        Serial.println("Log file created");
      }
    }

    currentRunNumber = findNextRunNumber();

    Serial.print("Current run label: RUN");
    Serial.println(currentRunNumber);

    writeRunSeparator();
  } else {
    sdOK = false;
    Serial.println("FAIL");
    Serial.println("Check SD wiring and card");
  }

  resetAllStats();
  resetRunSummaryStats();

  runStartTime = millis();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println("Sensors");

  display.setTextSize(1);
  display.setCursor(25, 40);
  display.println(sdOK ? "SD Ready" : "SD Not Found");

  display.setCursor(40, 52);
  display.print("RUN");
  display.println(currentRunNumber);

  display.display();

  Serial.println("Environment determining mode active");
  Serial.print("DS independent high alarm: ");
  Serial.print(DS_TEMP_HIGH_ALARM_C, 1);
  Serial.println(" C");

  Serial.println("Environment profiles active:");
  for (int i = 0; i < ENV_COUNT; i++) {
    Serial.print("  ");
    Serial.print(envProfiles[i].name);
    Serial.print(" | T=");
    Serial.print(envProfiles[i].tempMin, 1);
    Serial.print(" to ");
    Serial.print(envProfiles[i].tempMax, 1);
    Serial.print(" C, H=");
    Serial.print(envProfiles[i].humMin, 1);
    Serial.print(" to ");
    Serial.print(envProfiles[i].humMax, 1);
    Serial.print(" %, P=");
    Serial.print(envProfiles[i].presMin, 1);
    Serial.print(" to ");
    Serial.print(envProfiles[i].presMax, 1);
    Serial.println(" hPa");
  }

  delay(1500);
}

void loop() {
  unsigned long now = millis();

  // end run
  if (!runFinished && RUN_DURATION_SEC != 0) {
    if ((now - runStartTime) >= (RUN_DURATION_SEC * 1000UL)) {
      runFinished = true;
      stopOutputs();

      if (!finalLogWritten) {
        logToSD(now);
        finalLogWritten = true;
      }

      evaluateEnvironmentFit();
      printFinalSummaryToSerial();

      if (!finalSummaryWritten) {
        writeFinalSummaryToSD();
        finalSummaryWritten = true;
      }

      Serial.print("RUN");
      Serial.print(currentRunNumber);
      Serial.println(" COMPLETE");

      showFinishedSummaryScreen();
    }
  }

  if (runFinished) {
    showFinishedSummaryScreen();
    return;
  }

  // ds read
  if (now - lastDSRead >= DS_INTERVAL_MS) {
    lastDSRead = now;

    ds18b20.requestTemperatures();
    float t = ds18b20.getTempCByIndex(0);

    if (t != DEVICE_DISCONNECTED_C) {
      dsTemp = t;
      updateStats(st_dsTemp, dsTemp);
    }
  }

  // bme read
  if (now - lastBMERead >= BME_INTERVAL_MS) {
    lastBMERead = now;

    bmeTemp = bme.readTemperature();
    bmeHum = bme.readHumidity();
    bmePres = bme.readPressure() / 100.0f;

    updateStats(st_bmeTemp, bmeTemp);
    updateStats(st_bmeHum, bmeHum);
    updateStats(st_bmePres, bmePres);

    updateStats(run_bmeTemp, bmeTemp);
    updateStats(run_bmeHum, bmeHum);
    updateStats(run_bmePres, bmePres);
  }

  // fast block
  if (now - lastFastRead >= FAST_INTERVAL_MS) {
    lastFastRead = now;

    lightRaw = analogRead(PIN_LDR);
    updateStats(st_light, (float)lightRaw);

    batteryVoltage = readBatteryVoltage();
    solarVoltage = readSolarVoltage();

    updateStats(st_batt, batteryVoltage);
    updateStats(st_solar, solarVoltage);

    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    ax_g = (float)ax / normal_g;
    ay_g = (float)ay / normal_g;
    az_g = (float)az / normal_g;

    int16_t gx_raw, gy_raw, gz_raw;
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

    gx_dps = gx_raw / normal_gyro;
    gy_dps = gy_raw / normal_gyro;
    gz_dps = gz_raw / normal_gyro;

    updateStats(st_ax, ax_g);
    updateStats(st_ay, ay_g);
    updateStats(st_az, az_g);
    updateStats(st_gx, gx_dps);
    updateStats(st_gy, gy_dps);
    updateStats(st_gz, gz_dps);

    bmeTempOutNow = !isnan(bmeTemp) && (bmeTemp < TEMP_WARN_LOW || bmeTemp > TEMP_WARN_HIGH);
    dsTempHighNow = !isnan(dsTemp) && (dsTemp > DS_TEMP_HIGH_ALARM_C);
    humOutNow = !isnan(bmeHum) && (bmeHum < HUM_WARN_LOW || bmeHum > HUM_WARN_HIGH);
    presOutNow = !isnan(bmePres) && (bmePres < PRES_WARN_LOW || bmePres > PRES_WARN_HIGH);

    bool tempAlarm = bmeTempOutNow || dsTempHighNow;
    bool humAlarm = humOutNow;
    bool presAlarm = presOutNow;

    blinkLED(LED_RED, tempAlarm, blinkR, now);
    blinkLED(LED_BLUE, humAlarm, blinkB, now);
    blinkLED(LED_GREEN, presAlarm, blinkG, now);
    updateBuzzer(now, tempAlarm, humAlarm, presAlarm);

    Serial.print("RUN");
    Serial.print(currentRunNumber);
    Serial.print(" | t=");
    Serial.print((now - runStartTime) / 1000.0, 2);
    Serial.print(" s, Light: ");
    Serial.print(getLightState(lightRaw));
    Serial.print(" (raw=");
    Serial.print(lightRaw);
    Serial.print("), Battery: ");
    Serial.print(batteryVoltage, 2);
    Serial.print(" V, Solar: ");
    Serial.print(solarVoltage, 2);
    Serial.print(" V, BME Temp: ");
    Serial.print(bmeTemp, 1);
    Serial.print(" C, Hum: ");
    Serial.print(bmeHum, 1);
    Serial.print(" %, Pres: ");
    Serial.print(bmePres, 1);
    Serial.print(" hPa, DS18B20 Temp: ");
    Serial.print(dsTemp, 2);
    Serial.print(" C, Accel(g): (");
    Serial.print(ax_g, 3);
    Serial.print(", ");
    Serial.print(ay_g, 3);
    Serial.print(", ");
    Serial.print(az_g, 3);
    Serial.print("), Gyro(dps): (");
    Serial.print(gx_dps, 2);
    Serial.print(", ");
    Serial.print(gy_dps, 2);
    Serial.print(", ");
    Serial.print(gz_dps, 2);
    Serial.print(")");

    if (bmeTempOutNow) {
      Serial.print(" [TEMP OUT]");
    }
    if (dsTempHighNow) {
      Serial.print(" [DS HOT]");
    }
    if (humAlarm) {
      Serial.print(" [HUM OUT]");
    }
    if (presAlarm) {
      Serial.print(" [PRES OUT]");
    }

    Serial.println();
  }

  // oled
  if (now - lastOLEDRead >= OLED_INTERVAL_MS) {
    lastOLEDRead = now;
    updateOLED(now);
  }

  // sd logging
  if (now - lastSDWrite >= SD_INTERVAL_MS) {
    lastSDWrite = now;
    logToSD(now);
    resetAllStats();
  }
}