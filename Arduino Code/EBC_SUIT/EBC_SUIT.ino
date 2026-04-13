/*
Environmental Brain Cube - Suitability Mode

This mode runs the system for a test time and checks if the environment
stays safe enough for the cube. It shows live values, logs data to SD,
gives warnings, and can stop early if a warning lasts too long.

Main things it does:
- reads all sensors
- shows live values on OLED
- logs snapshot min/max values to SD
- watches warning times
- stops early if a bad condition lasts too long
- prints a final suitability summary

Log file:
- /ebc_suitability_log.csv
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
const unsigned long RUN_DURATION_SEC = 500;
const unsigned long MAX_CONT_WARN_SEC = 15; //warning counter

// time trackers
unsigned long runStartTime = 0;
bool runFinished = false;
bool finalSummaryWritten = false;
bool stoppedBySuitabilityFail = false;

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

//=========IF YOU CHANGE THESE VALUES, CHANGE THEM IN MATLAB SCRIPT AS WELL=======

// adc references. 3.3 is pin voltage for the esp32. 4095 is max adc reading
const float ADC_REF = 3.3;
const float ADC_MAX = 4095.0;

// battery divider (this setup allows for the system to safely handle about 10V)
// the 9V battery will result in a 2.9V input into the pin
const float R1_BATT = 100000.0;
const float R2_BATT = 46800.0;
const float BATT_CAL = 0.959;       //calibration value for the battery

const float R1_SOLAR = 100500.0;
const float R2_SOLAR = 46800.0;
const float SOLAR_CAL = 1.167;      //calibration value for the solar

// thresholds for system
const int LIGHT_DARK_TH = 3000;
const int LIGHT_BRIGHT_TH = 3500;

const float BME_TEMP_MIN_C = 20.0;
const float BME_TEMP_MAX_C = 25.0;

const float DS_TEMP_MIN_C = 20.0;
const float DS_TEMP_MAX_C = 25.0;

const float HUM_MIN_PCT = 40.0;
const float HUM_MAX_PCT = 55.0;

const float PRES_MIN_HPA = 950.0;
const float PRES_MAX_HPA = 1050.0;

const float BATT_MIN_V = 5.00;
const float BATT_MAX_V = 7.20;

const float SOLAR_MIN_V = 3.50;
const float SOLAR_MAX_V = 5.50;

const float LIGHT_MIN_RAW = 0.0;
const float LIGHT_MAX_RAW = 4095.0;

const float AX_MIN_G = -16.0;
const float AX_MAX_G = 16.0;
const float AY_MIN_G = -16.0;
const float AY_MAX_G = 16.0;
const float AZ_MIN_G = -16.0;
const float AZ_MAX_G = 16.0;

const float GX_MIN_DPS = -250.0;
const float GX_MAX_DPS = 250.0;
const float GY_MIN_DPS = -250.0;
const float GY_MAX_DPS = 250.0;
const float GZ_MIN_DPS = -250.0;
const float GZ_MAX_DPS = 250.0;

// final rating margins
const float VERY_MARGIN_FRAC = 0.20;
const float GOOD_MARGIN_FRAC = 0.10;

// timing
const unsigned long FAST_INTERVAL_MS = 100;
const unsigned long BME_INTERVAL_MS = 500;
const unsigned long DS_INTERVAL_MS = 1000;
const unsigned long OLED_INTERVAL_MS = 250;
const unsigned long SD_INTERVAL_MS = 2000;

unsigned long lastFastRead = 0;
unsigned long lastBMERead = 0;
unsigned long lastDSRead = 0;
unsigned long lastOLEDRead = 0;
unsigned long lastSDWrite = 0;

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

// warnings now
bool bmeTempWarnNow = false;
bool dsTempWarnNow = false;
bool humWarnNow = false;
bool presWarnNow = false;
bool battWarnNow = false;
bool solarWarnNow = false;
bool lightWarnNow = false;
bool axWarnNow = false;
bool ayWarnNow = false;
bool azWarnNow = false;
bool gxWarnNow = false;
bool gyWarnNow = false;
bool gzWarnNow = false;

// how long warnings have lasted
unsigned long bmeTempWarnStart = 0;
unsigned long dsTempWarnStart = 0;
unsigned long humWarnStart = 0;
unsigned long presWarnStart = 0;
unsigned long battWarnStart = 0;
unsigned long solarWarnStart = 0;
unsigned long lightWarnStart = 0;
unsigned long axWarnStart = 0;
unsigned long ayWarnStart = 0;
unsigned long azWarnStart = 0;
unsigned long gxWarnStart = 0;
unsigned long gyWarnStart = 0;
unsigned long gzWarnStart = 0;

String failReason = "NONE";
String finalRating = "UNRATED";

// sd
bool sdOK = false;
const char* logFile = "/ebc_suitability_log.csv";
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

// full run stats
struct StatTracker {
  float minVal;
  float maxVal;
  double sumVal;
  unsigned long count;
};

// one log interval stats
struct IntervalStatTracker {
  float minVal;
  float maxVal;
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

IntervalStatTracker int_light;
IntervalStatTracker int_bmeTemp;
IntervalStatTracker int_bmeHum;
IntervalStatTracker int_bmePres;
IntervalStatTracker int_dsTemp;
IntervalStatTracker int_batt;
IntervalStatTracker int_solar;
IntervalStatTracker int_ax;
IntervalStatTracker int_ay;
IntervalStatTracker int_az;
IntervalStatTracker int_gx;
IntervalStatTracker int_gy;
IntervalStatTracker int_gz;

// basic stat helpers
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

float avgStat(const StatTracker &s) {
  if (hasStats(s)) {
    return s.sumVal / s.count;
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

void resetIntervalStats(IntervalStatTracker &s) {
  s.minVal = 9999999.0;
  s.maxVal = -9999999.0;
  s.count = 0;
}

void updateIntervalStats(IntervalStatTracker &s, float v) {
  if (isnan(v)) {
    return;
  }

  if (v < s.minVal) {
    s.minVal = v;
  }

  if (v > s.maxVal) {
    s.maxVal = v;
  }

  s.count++;
}

bool hasIntervalStats(const IntervalStatTracker &s) {
  return s.count > 0;
}

void resetAllIntervalStats() {
  resetIntervalStats(int_light);
  resetIntervalStats(int_bmeTemp);
  resetIntervalStats(int_bmeHum);
  resetIntervalStats(int_bmePres);
  resetIntervalStats(int_dsTemp);
  resetIntervalStats(int_batt);
  resetIntervalStats(int_solar);
  resetIntervalStats(int_ax);
  resetIntervalStats(int_ay);
  resetIntervalStats(int_az);
  resetIntervalStats(int_gx);
  resetIntervalStats(int_gy);
  resetIntervalStats(int_gz);
}

void printIntervalStatOrNA(File &file, const IntervalStatTracker &s, int decimals) {
  if (hasIntervalStats(s)) {
    file.print(s.minVal, decimals);
  } else {
    file.print("NA");
  }

  file.print(",");

  if (hasIntervalStats(s)) {
    file.print(s.maxVal, decimals);
  } else {
    file.print("NA");
  }
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

// light helpers
const char* getLightState(int raw) {
  if (raw <= LIGHT_DARK_TH) {
    return "DARK";
  }

  if (raw < LIGHT_BRIGHT_TH) {
    return "NORMAL";
  }

  return "BRIGHT";
}

const char* getLightStateFromAvg(float rawAvg) {
  if (isnan(rawAvg)) {
    return "NA";
  }

  if (rawAvg <= LIGHT_DARK_TH) {
    return "DARK";
  }

  if (rawAvg < LIGHT_BRIGHT_TH) {
    return "NORMAL";
  }

  return "BRIGHT";
}

// analog read helpers
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

// run number helpers
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
    int runNum = extractRunNumber(file.readStringUntil('
'));
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
    "gzMin,gzMax,"
    "tempWarn,humWarn,presWarn"
  );

  file.close();

  Serial.print("Run separator written for RUN");
  Serial.println(currentRunNumber);
}

// startup screen
void bootScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 8);
  display.println("ENV");
  display.setCursor(0, 30);
  display.println("BRAIN CUBE");

  display.setTextSize(1);
  display.setCursor(24, 54);
  display.println("SUITABILITY");
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

// warning timers
void updateContinuousWarning(bool warnNow, unsigned long &warnStartRef, unsigned long now) {
  if (warnNow) {
    if (warnStartRef == 0) {
      warnStartRef = now;
    }
  } else {
    warnStartRef = 0;
  }
}

bool exceededContinuousWarning(unsigned long warnStartRef, unsigned long now) {
  return (warnStartRef != 0) && ((now - warnStartRef) >= (MAX_CONT_WARN_SEC * 1000UL));
}

// buzzer
void updateBuzzer(unsigned long now, bool anyCoreWarn, bool anySupportWarn, bool anyMotionWarn) {
  bool anyAlarm = anyCoreWarn || anySupportWarn || anyMotionWarn;

  if (!anyAlarm) {
    stopBuzzer();
    return;
  }

  int toneFreq = 1800;

  if (anyCoreWarn) {
    toneFreq = 2200;
  } else if (anySupportWarn) {
    toneFreq = 1800;
  } else if (anyMotionWarn) {
    toneFreq = 1400;
  }

  const unsigned long beepOnMs = 150;
  const unsigned long beepPeriodMs = 350;

  if ((now % beepPeriodMs) < beepOnMs) {
    ledcWriteTone(PIN_BUZZER, toneFreq);
  } else {
    stopBuzzer();
  }
}

// rating helpers
bool valueInRange(float v, float vmin, float vmax) {
  return !isnan(v) && (v >= vmin && v <= vmax);
}

bool valueInInnerRange(float v, float vmin, float vmax, float marginFrac) {
  if (isnan(v)) {
    return false;
  }

  float range = vmax - vmin;
  float innerMin = vmin + marginFrac * range;
  float innerMax = vmax - marginFrac * range;

  return (v >= innerMin && v <= innerMax);
}

String determineFinalRating() {
  if (stoppedBySuitabilityFail) {
    return "NOT SUITABLE";
  }

  float avgBmeTemp = avgStat(st_bmeTemp);
  float avgDsTemp = avgStat(st_dsTemp);
  float avgHum = avgStat(st_bmeHum);
  float avgPres = avgStat(st_bmePres);

  bool basicBmeTemp = valueInRange(avgBmeTemp, BME_TEMP_MIN_C, BME_TEMP_MAX_C);
  bool basicDsTemp = valueInRange(avgDsTemp, DS_TEMP_MIN_C, DS_TEMP_MAX_C);
  bool basicHum = valueInRange(avgHum, HUM_MIN_PCT, HUM_MAX_PCT);
  bool basicPres = valueInRange(avgPres, PRES_MIN_HPA, PRES_MAX_HPA);

  if (!(basicBmeTemp && basicDsTemp && basicHum && basicPres)) {
    return "MARGINAL";
  }

  bool veryBmeTemp = valueInInnerRange(avgBmeTemp, BME_TEMP_MIN_C, BME_TEMP_MAX_C, VERY_MARGIN_FRAC);
  bool veryDsTemp = valueInInnerRange(avgDsTemp, DS_TEMP_MIN_C, DS_TEMP_MAX_C, VERY_MARGIN_FRAC);
  bool veryHum = valueInInnerRange(avgHum, HUM_MIN_PCT, HUM_MAX_PCT, VERY_MARGIN_FRAC);
  bool veryPres = valueInInnerRange(avgPres, PRES_MIN_HPA, PRES_MAX_HPA, VERY_MARGIN_FRAC);

  if (veryBmeTemp && veryDsTemp && veryHum && veryPres) {
    return "VERY SUITABLE";
  }

  bool goodBmeTemp = valueInInnerRange(avgBmeTemp, BME_TEMP_MIN_C, BME_TEMP_MAX_C, GOOD_MARGIN_FRAC);
  bool goodDsTemp = valueInInnerRange(avgDsTemp, DS_TEMP_MIN_C, DS_TEMP_MAX_C, GOOD_MARGIN_FRAC);
  bool goodHum = valueInInnerRange(avgHum, HUM_MIN_PCT, HUM_MAX_PCT, GOOD_MARGIN_FRAC);
  bool goodPres = valueInInnerRange(avgPres, PRES_MIN_HPA, PRES_MAX_HPA, GOOD_MARGIN_FRAC);

  if (goodBmeTemp && goodDsTemp && goodHum && goodPres) {
    return "SUITABLE";
  }

  return "MARGINAL";
}

// final summary helpers
void printParamStatus(const char* name, float avgVal, float minAllowed, float maxAllowed, const char* units) {
  Serial.print(name);
  Serial.print(": ");

  if (isnan(avgVal)) {
    Serial.println("NO DATA");
    return;
  }

  Serial.print(avgVal, 2);
  Serial.print(" ");
  Serial.print(units);
  Serial.print(" | allowed ");
  Serial.print(minAllowed, 2);
  Serial.print(" to ");
  Serial.print(maxAllowed, 2);
  Serial.print(" ");
  Serial.print(units);
  Serial.print(" | ");

  if (avgVal < minAllowed) {
    Serial.println("BELOW RANGE");
  } else if (avgVal > maxAllowed) {
    Serial.println("ABOVE RANGE");
  } else {
    Serial.println("IN RANGE");
  }
}

void writeFinalSummaryToSD(unsigned long now) {
  if (!sdOK) {
    return;
  }

  File file = SD.open(logFile, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open log file for final summary");
    return;
  }

  float elapsedSec = (now - runStartTime) / 1000.0;

  float avgBmeTemp = avgStat(st_bmeTemp);
  float avgDsTemp = avgStat(st_dsTemp);
  float avgHum = avgStat(st_bmeHum);
  float avgPres = avgStat(st_bmePres);
  float avgBatt = avgStat(st_batt);
  float avgSolar = avgStat(st_solar);
  float avgLight = avgStat(st_light);
  float avgAx = avgStat(st_ax);
  float avgAy = avgStat(st_ay);
  float avgAz = avgStat(st_az);
  float avgGx = avgStat(st_gx);
  float avgGy = avgStat(st_gy);
  float avgGz = avgStat(st_gz);

  file.println("SUMMARY");
  file.print("FinalRating,"); file.println(finalRating);
  file.print("Result,"); file.println(stoppedBySuitabilityFail ? "STOPPED EARLY" : "FULL RUN COMPLETED");
  file.print("Reason,"); file.println(failReason);
  file.print("RunTimeSec,"); file.println(elapsedSec, 2);

  file.print("Avg_BME_Temp_C,"); if (hasStats(st_bmeTemp)) file.println(avgBmeTemp, 2); else file.println("NA");
  file.print("Avg_DS_Temp_C,"); if (hasStats(st_dsTemp)) file.println(avgDsTemp, 2); else file.println("NA");
  file.print("Avg_Humidity_pct,"); if (hasStats(st_bmeHum)) file.println(avgHum, 2); else file.println("NA");
  file.print("Avg_Pressure_hPa,"); if (hasStats(st_bmePres)) file.println(avgPres, 2); else file.println("NA");

  file.print("Avg_Battery_V,"); if (hasStats(st_batt)) file.println(avgBatt, 2); else file.println("NA");
  file.print("Avg_Solar_V,"); if (hasStats(st_solar)) file.println(avgSolar, 2); else file.println("NA");
  file.print("Avg_Light_Raw,"); if (hasStats(st_light)) file.println(avgLight, 2); else file.println("NA");
  file.print("Avg_Light_State,"); file.println(getLightStateFromAvg(avgLight));

  file.print("Avg_Ax_g,"); if (hasStats(st_ax)) file.println(avgAx, 3); else file.println("NA");
  file.print("Avg_Ay_g,"); if (hasStats(st_ay)) file.println(avgAy, 3); else file.println("NA");
  file.print("Avg_Az_g,"); if (hasStats(st_az)) file.println(avgAz, 3); else file.println("NA");

  file.print("Avg_Gx_dps,"); if (hasStats(st_gx)) file.println(avgGx, 2); else file.println("NA");
  file.print("Avg_Gy_dps,"); if (hasStats(st_gy)) file.println(avgGy, 2); else file.println("NA");
  file.print("Avg_Gz_dps,"); if (hasStats(st_gz)) file.println(avgGz, 2); else file.println("NA");

  file.println("=== END RUN SUMMARY ===");
  file.println();

  file.close();
}

void printFinalSummary(unsigned long now) {
  float elapsedSec = (now - runStartTime) / 1000.0;

  float avgBmeTemp = avgStat(st_bmeTemp);
  float avgDsTemp = avgStat(st_dsTemp);
  float avgHum = avgStat(st_bmeHum);
  float avgPres = avgStat(st_bmePres);
  float avgBatt = avgStat(st_batt);
  float avgSolar = avgStat(st_solar);
  float avgLight = avgStat(st_light);
  float avgAx = avgStat(st_ax);
  float avgAy = avgStat(st_ay);
  float avgAz = avgStat(st_az);
  float avgGx = avgStat(st_gx);
  float avgGy = avgStat(st_gy);
  float avgGz = avgStat(st_gz);

  Serial.println();
  Serial.println("====================================");
  Serial.print("RUN");
  Serial.print(currentRunNumber);
  Serial.println(" SUITABILITY SUMMARY");
  Serial.println("====================================");

  Serial.print("FINAL RATING: ");
  Serial.println(finalRating);

  if (stoppedBySuitabilityFail) {
    Serial.println("RESULT: TEST STOPPED EARLY");
    Serial.print("WHY: ");
    Serial.println(failReason);
  } else {
    Serial.println("RESULT: FULL RUN COMPLETED");
    Serial.println("WHY: No continuous warning exceeded allowed time");
  }

  Serial.print("Elapsed Time: ");
  Serial.print(elapsedSec, 2);
  Serial.println(" s");

  Serial.println();
  Serial.println("Whole-run averages compared to allowed ranges:");
  printParamStatus("BME Air Temp", avgBmeTemp, BME_TEMP_MIN_C, BME_TEMP_MAX_C, "C");
  printParamStatus("DS Ground Temp", avgDsTemp, DS_TEMP_MIN_C, DS_TEMP_MAX_C, "C");
  printParamStatus("Humidity", avgHum, HUM_MIN_PCT, HUM_MAX_PCT, "%");
  printParamStatus("Pressure", avgPres, PRES_MIN_HPA, PRES_MAX_HPA, "hPa");

  Serial.println();
  Serial.println("Support / behavior values:");
  printParamStatus("Battery", avgBatt, BATT_MIN_V, BATT_MAX_V, "V");
  printParamStatus("Solar", avgSolar, SOLAR_MIN_V, SOLAR_MAX_V, "V");
  printParamStatus("Light Raw", avgLight, LIGHT_MIN_RAW, LIGHT_MAX_RAW, "raw");
  printParamStatus("Accel X", avgAx, AX_MIN_G, AX_MAX_G, "g");
  printParamStatus("Accel Y", avgAy, AY_MIN_G, AY_MAX_G, "g");
  printParamStatus("Accel Z", avgAz, AZ_MIN_G, AZ_MAX_G, "g");
  printParamStatus("Gyro X", avgGx, GX_MIN_DPS, GX_MAX_DPS, "dps");
  printParamStatus("Gyro Y", avgGy, GY_MIN_DPS, GY_MAX_DPS, "dps");
  printParamStatus("Gyro Z", avgGz, GZ_MIN_DPS, GZ_MAX_DPS, "dps");

  Serial.println();
  Serial.print("Light Avg State: ");
  Serial.println(getLightStateFromAvg(avgLight));

  Serial.println("====================================");
  Serial.println();
}

// oled helpers
const char* getTopAlarmLabel() {
  if (bmeTempWarnNow) return "BME T";
  if (dsTempWarnNow) return "DS T";
  if (humWarnNow) return "HUM";
  if (presWarnNow) return "PRES";
  if (battWarnNow) return "BATT";
  if (solarWarnNow) return "SOL";
  if (lightWarnNow) return "LIGHT";
  if (axWarnNow || ayWarnNow || azWarnNow) return "ACC";
  if (gxWarnNow || gyWarnNow || gzWarnNow) return "GYRO";
  return "";
}

void updateOLED(unsigned long now) {
  float elapsedSec = (now - runStartTime) / 1000.0;
  bool flashOn = ((now / 300) % 2 == 0);

  bool anyTempWarn = bmeTempWarnNow || dsTempWarnNow;
  bool anyCoreWarn = anyTempWarn || humWarnNow || presWarnNow;
  bool anySupportWarn = battWarnNow || solarWarnNow || lightWarnNow;
  bool anyMotionWarn = axWarnNow || ayWarnNow || azWarnNow || gxWarnNow || gyWarnNow || gzWarnNow;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("T:");
  if (!isnan(bmeTemp)) {
    if (bmeTempWarnNow && !flashOn) {
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
    if (dsTempWarnNow && !flashOn) {
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
    if (humWarnNow && !flashOn) {
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
    if (presWarnNow && !flashOn) {
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
  if (battWarnNow && !flashOn) {
    display.print("    ");
  } else {
    display.print(batteryVoltage, 2);
    display.print("V");
  }

  display.setCursor(64, 20);
  display.print("Sol:");
  if (solarWarnNow && !flashOn) {
    display.print("    ");
  } else {
    display.print(solarVoltage, 2);
    display.print("V");
  }

  display.setCursor(0, 30);
  display.print("L:");
  if (lightWarnNow && !flashOn) {
    display.print("     ");
  } else {
    display.print(getLightState(lightRaw));
  }

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

  if (anyCoreWarn || anySupportWarn || anyMotionWarn) {
    display.setCursor(78, 50);
    display.print(getTopAlarmLabel());
  }

  display.display();
}

void showFinishedScreen(unsigned long now) {
  float elapsedSec = (now - runStartTime) / 1000.0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(10, 0);
  display.println("SUITABILITY TEST");

  display.setCursor(0, 14);
  display.println(finalRating);

  if (stoppedBySuitabilityFail) {
    display.setCursor(0, 28);
    display.println("Stopped early");
    display.setCursor(0, 38);
    display.println(failReason);
  } else {
    display.setCursor(0, 28);
    display.println("Full run passed");
  }

  display.setCursor(0, 54);
  display.print("t=");
  display.print(elapsedSec, 0);
  display.print("s");

  display.display();
}

// sd snapshot logging
void logSnapshotToSD(unsigned long now) {
  if (!sdOK) {
    return;
  }

  File file = SD.open(logFile, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open log file for snapshot writing");
    return;
  }

  float elapsedSec = (now - runStartTime) / 1000.0;
  bool tempWarn = (bmeTempWarnNow || dsTempWarnNow);

  file.print("RUN"); file.print(currentRunNumber); file.print(",");
  file.print(now); file.print(",");
  file.print(elapsedSec, 2); file.print(",");

  printIntervalStatOrNA(file, int_light, 0); file.print(",");
  file.print(getLightState(lightRaw)); file.print(",");

  printIntervalStatOrNA(file, int_bmeTemp, 2); file.print(",");
  printIntervalStatOrNA(file, int_bmeHum, 2); file.print(",");
  printIntervalStatOrNA(file, int_bmePres, 2); file.print(",");
  printIntervalStatOrNA(file, int_dsTemp, 2); file.print(",");
  printIntervalStatOrNA(file, int_batt, 2); file.print(",");
  printIntervalStatOrNA(file, int_solar, 2); file.print(",");
  printIntervalStatOrNA(file, int_ax, 3); file.print(",");
  printIntervalStatOrNA(file, int_ay, 3); file.print(",");
  printIntervalStatOrNA(file, int_az, 3); file.print(",");
  printIntervalStatOrNA(file, int_gx, 2); file.print(",");
  printIntervalStatOrNA(file, int_gy, 2); file.print(",");
  printIntervalStatOrNA(file, int_gz, 2); file.print(",");

  file.print(tempWarn ? 1 : 0); file.print(",");
  file.print(humWarnNow ? 1 : 0); file.print(",");
  file.print(presWarnNow ? 1 : 0);
  file.println();

  file.close();

  resetAllIntervalStats();
}

// run finish helper
void finishRun(unsigned long now, bool failed, String reason) {
  runFinished = true;
  stoppedBySuitabilityFail = failed;
  failReason = reason;
  stopOutputs();

  finalRating = determineFinalRating();

  if (!finalSummaryWritten) {
    logSnapshotToSD(now);
    writeFinalSummaryToSD(now);
    printFinalSummary(now);
    showFinishedScreen(now);
    finalSummaryWritten = true;
  }
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
        file.println("Environmental Brain Cube Suitability Log");
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
  resetAllIntervalStats();
  runStartTime = millis();

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(5, 8);
  display.println("Suitable?");

  display.setTextSize(1);
  display.setCursor(22, 36);
  display.println(sdOK ? "SD Ready" : "SD Not Found");

  display.setCursor(12, 50);
  display.print("RUN");
  display.print(currentRunNumber);
  display.print(" ");
  display.print(RUN_DURATION_SEC);
  display.print("s");
  display.display();

  Serial.println("Suitability mode active");
  delay(1500);
}

void loop() {
  unsigned long now = millis();

  if (runFinished) {
    return;
  }

  // ground temp
  if (now - lastDSRead >= DS_INTERVAL_MS) {
    lastDSRead = now;

    ds18b20.requestTemperatures();
    float t = ds18b20.getTempCByIndex(0);

    if (t != DEVICE_DISCONNECTED_C) {
      dsTemp = t;
      updateStats(st_dsTemp, dsTemp);
      updateIntervalStats(int_dsTemp, dsTemp);
    }
  }

  // air temp, humidity, pressure
  if (now - lastBMERead >= BME_INTERVAL_MS) {
    lastBMERead = now;

    bmeTemp = bme.readTemperature();
    bmeHum = bme.readHumidity();
    bmePres = bme.readPressure() / 100.0f;

    updateStats(st_bmeTemp, bmeTemp);
    updateStats(st_bmeHum, bmeHum);
    updateStats(st_bmePres, bmePres);

    updateIntervalStats(int_bmeTemp, bmeTemp);
    updateIntervalStats(int_bmeHum, bmeHum);
    updateIntervalStats(int_bmePres, bmePres);
  }

  // fast repeating block
  if (now - lastFastRead >= FAST_INTERVAL_MS) {
    lastFastRead = now;

    lightRaw = analogRead(PIN_LDR);
    updateStats(st_light, (float)lightRaw);
    updateIntervalStats(int_light, (float)lightRaw);

    batteryVoltage = readBatteryVoltage();
    solarVoltage = readSolarVoltage();

    updateStats(st_batt, batteryVoltage);
    updateStats(st_solar, solarVoltage);
    updateIntervalStats(int_batt, batteryVoltage);
    updateIntervalStats(int_solar, solarVoltage);

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

    updateIntervalStats(int_ax, ax_g);
    updateIntervalStats(int_ay, ay_g);
    updateIntervalStats(int_az, az_g);
    updateIntervalStats(int_gx, gx_dps);
    updateIntervalStats(int_gy, gy_dps);
    updateIntervalStats(int_gz, gz_dps);

    // current warning checks
    bmeTempWarnNow = !isnan(bmeTemp) && (bmeTemp < BME_TEMP_MIN_C || bmeTemp > BME_TEMP_MAX_C);
    dsTempWarnNow = !isnan(dsTemp) && (dsTemp < DS_TEMP_MIN_C || dsTemp > DS_TEMP_MAX_C);
    humWarnNow = !isnan(bmeHum) && (bmeHum < HUM_MIN_PCT || bmeHum > HUM_MAX_PCT);
    presWarnNow = !isnan(bmePres) && (bmePres < PRES_MIN_HPA || bmePres > PRES_MAX_HPA);

    battWarnNow = !isnan(batteryVoltage) && (batteryVoltage < BATT_MIN_V || batteryVoltage > BATT_MAX_V);
    solarWarnNow = !isnan(solarVoltage) && (solarVoltage < SOLAR_MIN_V || solarVoltage > SOLAR_MAX_V);
    lightWarnNow = (lightRaw < LIGHT_MIN_RAW || lightRaw > LIGHT_MAX_RAW);

    axWarnNow = (ax_g < AX_MIN_G || ax_g > AX_MAX_G);
    ayWarnNow = (ay_g < AY_MIN_G || ay_g > AY_MAX_G);
    azWarnNow = (az_g < AZ_MIN_G || az_g > AZ_MAX_G);

    gxWarnNow = (gx_dps < GX_MIN_DPS || gx_dps > GX_MAX_DPS);
    gyWarnNow = (gy_dps < GY_MIN_DPS || gy_dps > GY_MAX_DPS);
    gzWarnNow = (gz_dps < GZ_MIN_DPS || gz_dps > GZ_MAX_DPS);

    // warning timers
    updateContinuousWarning(bmeTempWarnNow, bmeTempWarnStart, now);
    updateContinuousWarning(dsTempWarnNow, dsTempWarnStart, now);
    updateContinuousWarning(humWarnNow, humWarnStart, now);
    updateContinuousWarning(presWarnNow, presWarnStart, now);
    updateContinuousWarning(battWarnNow, battWarnStart, now);
    updateContinuousWarning(solarWarnNow, solarWarnStart, now);
    updateContinuousWarning(lightWarnNow, lightWarnStart, now);
    updateContinuousWarning(axWarnNow, axWarnStart, now);
    updateContinuousWarning(ayWarnNow, ayWarnStart, now);
    updateContinuousWarning(azWarnNow, azWarnStart, now);
    updateContinuousWarning(gxWarnNow, gxWarnStart, now);
    updateContinuousWarning(gyWarnNow, gyWarnStart, now);
    updateContinuousWarning(gzWarnNow, gzWarnStart, now);

    bool anyCoreWarn = bmeTempWarnNow || dsTempWarnNow || humWarnNow || presWarnNow;
    bool anySupportWarn = battWarnNow || solarWarnNow || lightWarnNow;
    bool anyMotionWarn = axWarnNow || ayWarnNow || azWarnNow || gxWarnNow || gyWarnNow || gzWarnNow;

    blinkLED(LED_RED, anyCoreWarn, blinkR, now);
    blinkLED(LED_BLUE, anySupportWarn, blinkB, now);
    blinkLED(LED_GREEN, anyMotionWarn, blinkG, now);

    updateBuzzer(now, anyCoreWarn, anySupportWarn, anyMotionWarn);

    // stop if one warning lasts too long
    if (exceededContinuousWarning(bmeTempWarnStart, now)) { finishRun(now, true, "BME TEMP WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(dsTempWarnStart, now)) { finishRun(now, true, "DS TEMP WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(humWarnStart, now)) { finishRun(now, true, "HUMIDITY WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(presWarnStart, now)) { finishRun(now, true, "PRESSURE WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(battWarnStart, now)) { finishRun(now, true, "BATTERY WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(solarWarnStart, now)) { finishRun(now, true, "SOLAR WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(lightWarnStart, now)) { finishRun(now, true, "LIGHT WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(axWarnStart, now)) { finishRun(now, true, "ACCEL X WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(ayWarnStart, now)) { finishRun(now, true, "ACCEL Y WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(azWarnStart, now)) { finishRun(now, true, "ACCEL Z WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(gxWarnStart, now)) { finishRun(now, true, "GYRO X WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(gyWarnStart, now)) { finishRun(now, true, "GYRO Y WARNING TOO LONG"); return; }
    if (exceededContinuousWarning(gzWarnStart, now)) { finishRun(now, true, "GYRO Z WARNING TOO LONG"); return; }

    // serial print
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
    Serial.print(" hPa, DS Temp: ");
    Serial.print(dsTemp, 2);
    Serial.print(" C, Accel(g): (");
    Serial.print(ax_g, 3); Serial.print(", ");
    Serial.print(ay_g, 3); Serial.print(", ");
    Serial.print(az_g, 3);
    Serial.print("), Gyro(dps): (");
    Serial.print(gx_dps, 2); Serial.print(", ");
    Serial.print(gy_dps, 2); Serial.print(", ");
    Serial.print(gz_dps, 2);
    Serial.print(")");

    if (bmeTempWarnNow) Serial.print(" [BME TEMP]");
    if (dsTempWarnNow) Serial.print(" [DS TEMP]");
    if (humWarnNow) Serial.print(" [HUM]");
    if (presWarnNow) Serial.print(" [PRES]");
    if (battWarnNow) Serial.print(" [BATT]");
    if (solarWarnNow) Serial.print(" [SOLAR]");
    if (lightWarnNow) Serial.print(" [LIGHT]");
    if (axWarnNow) Serial.print(" [AX]");
    if (ayWarnNow) Serial.print(" [AY]");
    if (azWarnNow) Serial.print(" [AZ]");
    if (gxWarnNow) Serial.print(" [GX]");
    if (gyWarnNow) Serial.print(" [GY]");
    if (gzWarnNow) Serial.print(" [GZ]");

    Serial.println();
  }

  if (now - lastOLEDRead >= OLED_INTERVAL_MS) {
    lastOLEDRead = now;
    updateOLED(now);
  }

  if (now - lastSDWrite >= SD_INTERVAL_MS) {
    lastSDWrite = now;
    logSnapshotToSD(now);
  }

  if (RUN_DURATION_SEC != 0 && (now - runStartTime) >= (RUN_DURATION_SEC * 1000UL)) {
    finishRun(now, false, "NONE");
    return;
  }
}
