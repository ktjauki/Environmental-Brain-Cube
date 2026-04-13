/*
Environmental Brain Cube - Test Mode

This code tests the main parts of the Environmental Brain Cube one by one to make sure they respond.

It tests:
OLED
Red LED
Blue LED
Green LED
Buzzer
BME280
DS18B20
LDR
Solar input
Battery input
MPU6050
SD card

You answer each test in Serial Monitor:
Y = pass
N = fail
R = rerun

At the end it prints a summary to Serial
and also saves the summary to:

/ebc_test_summary.txt
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
const float BATT_CAL = 0.959; //calibration value for the battery

// solar divider (this setup allows for the system to safely handle about 10V)
const float R1_SOLAR = 100500.0;
const float R2_SOLAR = 46800.0;
const float SOLAR_CAL = 1.167; //calibration value for the solar

// oled print
void oled4(String l1, String l2 = "", String l3 = "", String l4 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(l1);

  display.setCursor(0, 16);
  display.println(l2);

  display.setCursor(0, 32);
  display.println(l3);

  display.setCursor(0, 48);
  display.println(l4);

  display.display();
}

// turn outputs off
void allOutputsOff() {
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
  noTone(PIN_BUZZER);
}

// print pass/fail
void printResult(String name, bool pass) {
  Serial.print("[");
  if (pass) {
    Serial.print("PASS");
  } else {
    Serial.print("FAIL");
  }
  Serial.print("] ");
  Serial.println(name);
}

// final summary line to serial
void summaryLine(String name, bool pass) {
  Serial.print(name);
  Serial.print(": ");
  if (pass) {
    Serial.println("PASS");
  } else {
    Serial.println("FAIL");
  }
}

// final summary line to file
void summaryLineToFile(File &file, String name, bool pass) {
  file.print(name);
  file.print(": ");
  if (pass) {
    file.println("PASS");
  } else {
    file.println("FAIL");
  }
}

// battery voltage
float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY);
  float adcVoltage = (raw / ADC_MAX) * ADC_REF;
  float battVoltage = adcVoltage * ((R1_BATT + R2_BATT) / R2_BATT) * BATT_CAL;
  return battVoltage;
}

// solar voltage
float readSolarVoltage() {
  int raw = analogRead(PIN_SOLAR);
  float adcVoltage = (raw / ADC_MAX) * ADC_REF;
  float solarVoltage = adcVoltage * ((R1_SOLAR + R2_SOLAR) / R2_SOLAR) * SOLAR_CAL;
  return solarVoltage;
}

// wait for y n or r
char askUser(String question) {
  Serial.println();
  Serial.println(question);
  Serial.println("Type Y = PASS, N = FAIL, R = RERUN");

  while (true) {
    if (Serial.available()) {
      char c = Serial.read();

      if (c == 'Y' || c == 'y') {
        Serial.println("User answered: PASS");
        return 'Y';
      }

      if (c == 'N' || c == 'n') {
        Serial.println("User answered: FAIL");
        return 'N';
      }

      if (c == 'R' || c == 'r') {
        Serial.println("User answered: RERUN");
        return 'R';
      }
    }
  }
}

// save summary to sd
bool writeTestSummaryToSD(bool rOLED, bool rRed, bool rBlue, bool rGreen,
                          bool rBuzzer, bool rBME, bool rDS, bool rLDR,
                          bool rSolar, bool rBattery, bool rMPU, bool rSD) {
  bool sdOK = SD.begin(SD_CS);

  if (!sdOK) {
    Serial.println("Could not initialize SD card for summary write.");
    return false;
  }

  File file = SD.open("/ebc_test_summary.txt", FILE_WRITE);

  if (!file) {
    Serial.println("Failed to open /ebc_test_summary.txt for writing.");
    return false;
  }

  file.println("======================================");
  file.println("ENVIRONMENTAL BRAIN CUBE TEST SUMMARY");
  file.println("======================================");
  file.print("Millis at end of run: ");
  file.println(millis());
  file.println();

  summaryLineToFile(file, "OLED", rOLED);
  summaryLineToFile(file, "Red LED", rRed);
  summaryLineToFile(file, "Blue LED", rBlue);
  summaryLineToFile(file, "Green LED", rGreen);
  summaryLineToFile(file, "Buzzer", rBuzzer);
  summaryLineToFile(file, "BME280", rBME);
  summaryLineToFile(file, "DS18B20", rDS);
  summaryLineToFile(file, "LDR", rLDR);
  summaryLineToFile(file, "Solar input", rSolar);
  summaryLineToFile(file, "Battery input", rBattery);
  summaryLineToFile(file, "MPU6050", rMPU);
  summaryLineToFile(file, "SD card", rSD);

  file.println();
  file.println("End of summary.");
  file.close();

  if (SD.exists("/ebc_test_summary.txt")) {
    Serial.println("Test summary saved to /ebc_test_summary.txt");
    return true;
  } else {
    Serial.println("Summary file write failed.");
    return false;
  }
}

// oled test
bool testOLED() {
  while (true) {
    oled4("OLED TEST", "Can you read", "this message?", "Y/N/R in Serial");

    char answer = askUser("Can you read the OLED?");

    if (answer == 'Y') {
      printResult("OLED", true);
      return true;
    }

    if (answer == 'N') {
      printResult("OLED", false);
      return false;
    }

    Serial.println("Rerunning OLED test...");
  }
}

// led test
bool testLED(int pin, String name) {
  while (true) {
    digitalWrite(pin, HIGH);

    oled4(name, "should be ON", "", "Y/N/R in Serial");

    char answer = askUser(name + " ON?");

    digitalWrite(pin, LOW);

    if (answer == 'Y') {
      printResult(name, true);
      return true;
    }

    if (answer == 'N') {
      printResult(name, false);
      return false;
    }

    Serial.println("Rerunning " + name + " test...");
    delay(300);
  }
}

// buzzer test
bool testBuzzer() {
  while (true) {
    tone(PIN_BUZZER, 2000);
    oled4("BUZZER TEST", "Listen for sound", "", "Y/N/R in Serial");
    delay(1200);
    noTone(PIN_BUZZER);

    char answer = askUser("Did you hear the buzzer?");

    if (answer == 'Y') {
      printResult("Buzzer", true);
      return true;
    }

    if (answer == 'N') {
      printResult("Buzzer", false);
      return false;
    }

    Serial.println("Rerunning buzzer test...");
    delay(300);
  }
}

// bme test
bool testBME280Live() {
  while (true) {
    Serial.println();
    Serial.println("BME280 LIVE TEST");
    Serial.println("Watch values live. Breathe on sensor or change surroundings.");

    unsigned long start = millis();
    bool validSeen = false;

    while (millis() - start < 20000) {
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      float p = bme.readPressure() / 100.0F;

      if (!isnan(t) && !isnan(h) && !isnan(p) && p > 100 && p < 1200) {
        validSeen = true;
      }

      Serial.print("BME -> T: ");
      Serial.print(t, 1);
      Serial.print(" C  H: ");
      Serial.print(h, 0);
      Serial.print(" %  P: ");
      Serial.print(p, 1);
      Serial.println(" hPa");

      oled4("BME LIVE",
            "T:" + String(t, 1) + "C H:" + String(h, 0) + "%",
            "P:" + String(p, 1) + "hPa",
            "Change surroundings");

      delay(500);
    }

    char answer = askUser("Did the BME values look valid and respond?");

    if (answer == 'Y') {
      printResult("BME280", validSeen);
      return validSeen;
    }

    if (answer == 'N') {
      printResult("BME280", false);
      return false;
    }

    Serial.println("Rerunning BME280 test...");
    delay(300);
  }
}

// ds18b20 test
bool testDS18B20Live() {
  while (true) {
    Serial.println();
    Serial.println("DS18B20 LIVE TEST");
    Serial.println("Touch the DS sensor and watch temperature change.");

    unsigned long start = millis();
    float firstTemp = NAN;
    bool validSeen = false;
    bool changed = false;

    while (millis() - start < 20000) {
      ds18b20.requestTemperatures();
      float t = ds18b20.getTempCByIndex(0);

      if (t > -100 && t < 150) {
        validSeen = true;

        if (isnan(firstTemp)) {
          firstTemp = t;
        }

        if (fabs(t - firstTemp) >= 0.5) {
          changed = true;
        }
      }

      Serial.print("DS18B20 -> T: ");
      Serial.print(t, 2);
      Serial.println(" C");

      oled4("DS18B20 LIVE",
            "Temp:" + String(t, 2) + "C",
            "Touch sensor",
            "Watch for increase");

      delay(700);
    }

    char answer = askUser("Did the DS18B20 value respond when touched?");

    if (answer == 'Y') {
      if (validSeen && changed) {
        printResult("DS18B20", true);
        return true;
      } else {
        printResult("DS18B20", false);
        return false;
      }
    }

    if (answer == 'N') {
      printResult("DS18B20", false);
      return false;
    }

    Serial.println("Rerunning DS18B20 test...");
    delay(300);
  }
}

// ldr test
bool testLDRLive() {
  while (true) {
    Serial.println();
    Serial.println("LDR LIVE TEST");
    Serial.println("Cover and uncover the LDR by hand.");

    unsigned long start = millis();
    int minVal = 4095;
    int maxVal = 0;

    while (millis() - start < 10000) {
      int val = analogRead(PIN_LDR);

      if (val < minVal) {
        minVal = val;
      }

      if (val > maxVal) {
        maxVal = val;
      }

      Serial.print("LDR -> ");
      Serial.println(val);

      oled4("LDR LIVE",
            "Raw:" + String(val),
            "Cover/uncover",
            "by hand");

      delay(250);
    }

    int diff = maxVal - minVal;

    Serial.print("LDR min: ");
    Serial.println(minVal);
    Serial.print("LDR max: ");
    Serial.println(maxVal);
    Serial.print("LDR diff: ");
    Serial.println(diff);

    char answer = askUser("Did the LDR value clearly change?");

    if (answer == 'Y') {
      if (diff > 300) {
        printResult("LDR", true);
        return true;
      } else {
        printResult("LDR", false);
        return false;
      }
    }

    if (answer == 'N') {
      printResult("LDR", false);
      return false;
    }

    Serial.println("Rerunning LDR test...");
    delay(300);
  }
}

// solar test
bool testSolarLive() {
  while (true) {
    Serial.println();
    Serial.println("SOLAR LIVE TEST");
    Serial.println("Cover/uncover the solar panel or shine light on it.");

    unsigned long start = millis();
    float minV = 9999.0;
    float maxV = -9999.0;

    while (millis() - start < 10000) {
      float v = readSolarVoltage();

      if (v < minV) {
        minV = v;
      }

      if (v > maxV) {
        maxV = v;
      }

      Serial.print("SOLAR -> ");
      Serial.print(v, 3);
      Serial.println(" V");

      oled4("SOLAR LIVE",
            "V:" + String(v, 2) + "V",
            "Cover/uncover",
            "or add light");

      delay(250);
    }

    float diff = maxV - minV;

    Serial.print("Solar min V: ");
    Serial.println(minV, 3);
    Serial.print("Solar max V: ");
    Serial.println(maxV, 3);
    Serial.print("Solar diff V: ");
    Serial.println(diff, 3);

    char answer = askUser("Did the solar reading clearly change?");

    if (answer == 'Y') {
      if (diff > 0.10) {
        printResult("Solar input", true);
        return true;
      } else {
        printResult("Solar input", false);
        return false;
      }
    }

    if (answer == 'N') {
      printResult("Solar input", false);
      return false;
    }

    Serial.println("Rerunning solar test...");
    delay(300);
  }
}

// battery test
bool testBatteryInput() {
  while (true) {
    float batt = readBatteryVoltage();
    bool valid = false;

    if (batt >= 0.0 && batt <= 20.0) {
      valid = true;
    }

    Serial.print("Battery voltage: ");
    Serial.print(batt, 2);
    Serial.println(" V");

    oled4("BATTERY TEST", "Reading:", String(batt, 2) + "V", "Y/N/R in Serial");

    char answer = askUser("Does the battery reading look reasonable?");

    if (answer == 'Y') {
      printResult("Battery input", valid);
      return valid;
    }

    if (answer == 'N') {
      printResult("Battery input", false);
      return false;
    }

    Serial.println("Rerunning battery test...");
    delay(300);
  }
}

// mpu test
bool testMPU6050Live() {
  while (true) {
    Serial.println();
    Serial.println("MPU6050 LIVE TEST");
    Serial.println("Move/tilt/shake the cube and watch values change.");

    int16_t ax0, ay0, az0, gx0, gy0, gz0;
    mpu.getMotion6(&ax0, &ay0, &az0, &gx0, &gy0, &gz0);

    unsigned long start = millis();
    bool changed = false;

    while (millis() - start < 20000) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      long accDiff = abs(ax - ax0) + abs(ay - ay0) + abs(az - az0);
      long gyroDiff = abs(gx - gx0) + abs(gy - gy0) + abs(gz - gz0);

      if (accDiff > 2500 || gyroDiff > 2500) {
        changed = true;
      }

      Serial.print("MPU A:");
      Serial.print(ax);
      Serial.print(",");
      Serial.print(ay);
      Serial.print(",");
      Serial.print(az);
      Serial.print("  G:");
      Serial.print(gx);
      Serial.print(",");
      Serial.print(gy);
      Serial.print(",");
      Serial.println(gz);

      oled4("MPU LIVE",
            "A:" + String(ax) + "," + String(ay),
            "Az:" + String(az),
            "Move the cube");

      delay(350);
    }

    char answer = askUser("Did the MPU values respond to movement?");

    if (answer == 'Y') {
      printResult("MPU6050", changed);
      return changed;
    }

    if (answer == 'N') {
      printResult("MPU6050", false);
      return false;
    }

    Serial.println("Rerunning MPU6050 test...");
    delay(300);
  }
}

// sd card test
bool testSDCard() {
  while (true) {
    Serial.println();
    Serial.println("SD CARD TEST");

    oled4("SD CARD TEST", "Writing file...", "", "");

    bool sdOK = SD.begin(SD_CS);
    bool writeOK = false;

    if (sdOK) {
      File file = SD.open("/ebc_test.txt", FILE_WRITE);

      if (file) {
        file.println("Environmental Brain Cube SD card test.");
        file.println("If you see this file, SD writing works.");
        file.close();

        if (SD.exists("/ebc_test.txt")) {
          writeOK = true;
        }
      }
    }

    oled4("SD CARD TEST",
          writeOK ? "Write OK" : "Write FAIL",
          "/ebc_test.txt",
          "Y/N/R in Serial");

    char answer = askUser("Did the SD card test pass?");

    if (answer == 'Y') {
      printResult("SD card", writeOK);
      return writeOK;
    }

    if (answer == 'N') {
      printResult("SD card", false);
      return false;
    }

    Serial.println("Rerunning SD card test...");
    delay(300);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("ENVIRONMENTAL BRAIN CUBE - COMPONENT TEST PROGRAM");
  Serial.println("Open Serial Monitor at 115200 baud");
  Serial.println("Use Y = PASS, N = FAIL, R = RERUN");

  Wire.begin(I2C_SDA, I2C_SCL);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_LDR, ADC_11db);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  analogSetPinAttenuation(PIN_SOLAR, ADC_11db);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  allOutputsOff();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED init failed");
    while (1) {
    }
  }

  oled4("EBC TEST RUN", "Starting...", "", "");

  bool bmeOK = bme.begin(0x76, &Wire);
  Serial.print("BME280 init: ");
  Serial.println(bmeOK ? "OK" : "FAIL");

  mpu.initialize();
  bool mpuOK = mpu.testConnection();
  Serial.print("MPU6050 init: ");
  Serial.println(mpuOK ? "OK" : "FAIL");

  ds18b20.begin();
  Serial.println("DS18B20 init called");

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  delay(1000);

  bool rOLED = false;
  bool rRed = false;
  bool rBlue = false;
  bool rGreen = false;
  bool rBuzzer = false;
  bool rBME = false;
  bool rDS = false;
  bool rLDR = false;
  bool rSolar = false;
  bool rBattery = false;
  bool rMPU = false;
  bool rSD = false;

  rOLED = testOLED();
  rRed = testLED(LED_RED, "Red LED");
  rBlue = testLED(LED_BLUE, "Blue LED");
  rGreen = testLED(LED_GREEN, "Green LED");
  rBuzzer = testBuzzer();

  if (bmeOK) {
    rBME = testBME280Live();
  } else {
    printResult("BME280", false);
  }

  rDS = testDS18B20Live();
  rLDR = testLDRLive();
  rSolar = testSolarLive();
  rBattery = testBatteryInput();

  if (mpuOK) {
    rMPU = testMPU6050Live();
  } else {
    printResult("MPU6050", false);
  }

  rSD = testSDCard();

  allOutputsOff();

  Serial.println();
  Serial.println("FINAL TEST SUMMARY");
  summaryLine("OLED", rOLED);
  summaryLine("Red LED", rRed);
  summaryLine("Blue LED", rBlue);
  summaryLine("Green LED", rGreen);
  summaryLine("Buzzer", rBuzzer);
  summaryLine("BME280", rBME);
  summaryLine("DS18B20", rDS);
  summaryLine("LDR", rLDR);
  summaryLine("Solar input", rSolar);
  summaryLine("Battery input", rBattery);
  summaryLine("MPU6050", rMPU);
  summaryLine("SD card", rSD);

  bool summarySaved = writeTestSummaryToSD(
    rOLED, rRed, rBlue, rGreen,
    rBuzzer, rBME, rDS, rLDR,
    rSolar, rBattery, rMPU, rSD
  );

  Serial.print("Summary file: ");
  Serial.println(summarySaved ? "SAVED" : "FAILED");

  oled4("TEST COMPLETE",
        summarySaved ? "Summary saved" : "Summary save FAIL",
        "/ebc_test_summary.txt",
        "");
}

void loop() {
}