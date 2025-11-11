// Nuvarande (251111) så är detta en gammal version av mjukvaran
// modifierad för V0.2 av kortet
// stora skillnader är reläerna på kortet, fotomotstånd och vippbrytare istället för en knapp
// Bäst är att uppdatera detta först till att likna hur den nya
// V1 mjukvaran ser ut, innan denna slipas up

#include <Arduino.h>

// Wire library for I2C communication
#include <Wire.h>

// For the I2C LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

// For string conversion functions
#include <stdlib.h>

// For SPI communication with the Digital Potentiometer
#include <SPI.h>

// Stolen from https://learn.sparkfun.com/tutorials/mcp4725-digital-to-analog-converter-hookup-guide
// Default address apparently
#define MCP4725_ADDR 0x60

// R_ON_RELAY is relay nr 8 on the relay board in the schematic
#define R_ON_RELAY 9
#define R_LOAD_RELAY 8
#define B_PLUS_ON_RELAY 7
#define B_MID_OFF_RELAY 6
#define DC_DISCONNECT_RELAY 5
#define DC_GROUND_RELAY 4
#define DY_DISCONNECT_RELAY 3
#define DY_GROUND_RELAY 2
// DY_GROUND_RELAY is relay nr 1 on the relay board in the schematic
#define R_B_FLIP_MEASURE_RELAY 11
#define DC_DY_FLIP_MEASURE_RELAY 10

#define PHOTORESISTOR_PIN A0
#define CURRENT_MEASURE_PIN A1
#define R_B_MEASURE_PIN A3
#define DC_DY_MEASURE_PIN A2

#define SWITCH_UP_PIN 12
#define SWITCH_DOWN_PIN 13

// The delay after triggering the relay, as i noticed the relays are too slow
#define RELAY_DELAY 30

// The delay inbetween the measurments, hopefully only for debugging
#define MEASURE_POINT_DELAY 100

// PSU calculated values because the PSU does not seem very accurate so we have to calculate
// what target voltage actually gives the correct out voltage
#define PSU_10V 9.7
#define PSU_11V5 11
#define PSU_12V 11.5
#define PSU_15V 13.9
#define PSU_24V 21.3
#define PSU_MAX 28

// The ardunio will remeassure the test if it failed up to this amount of remeassures
#define RETEST_COUNT 200
#define RETEST_COUNT_TEMPSENSOR 20

// How much time we measure while the PSU sweeps up, because we dont know when it is at max voltage
// we have to assume after some time that it is finished
#define SWEEP_TIME 1500

enum MeasurePoint
{
  R,
  B,
  DC,
  DY
};

struct Test
{
  String name;
  String measureUnit;
  float min;
  float max;
  float value;
  bool success;
};

bool R_B_FLIP_RELAY_STATE = LOW;
bool DC_DY_FLIP_RELAY_STATE = LOW;

// For the LCD test printing
char rows[3][21];
int current_row = 0;

// For saving the test results for the final result screen.
Test testResults[22] = {};
int testResultsLength = 0;

bool isUpPressed()
{
  return digitalRead(SWITCH_UP_PIN) == LOW;
}

bool isUpLongPressed()
{
  unsigned long start = millis();

  while (isUpPressed())
  {
    if ((millis() - start) > 1000)
    {
      return true;
    }

    delay(5);
  }

  return false;
}

bool isDownPressed()
{
  return digitalRead(SWITCH_DOWN_PIN) == LOW;
}

bool isDownLongPressed()
{
  unsigned long start = millis();

  while (isDownPressed())
  {
    if ((millis() - start) > 1000)
    {
      return true;
    }

    delay(5);
  }

  return false;
}

bool isAnyPressed()
{
  return isUpPressed() || isDownPressed();
}

void waitForInput()
{
  while (!isAnyPressed())
  {
    delay(5);
  }
}

// The DAC thats controlling the PSU is 12-bit resolution so max is 4095.
// Likriktaren bör ställa in 30V när man skickar in högsta spänningen
// Nu skickas det via en Diod så vi behöver lägga till spänningsdroppet till ut spänning
void setInputVoltage(float voltage, bool with_delay)
{
  voltage = constrain(voltage, 0.0, 32.2);

  float send_voltage = voltage * (5.0 / 32.2) + 0.4;

  int DACValue = round(send_voltage * (4095.0 / 5.0));

  Wire.beginTransmission(MCP4725_ADDR);
  // cmd to update the DAC
  Wire.write(64);
  // 8 most significant bits
  Wire.write(DACValue >> 4);
  // 4 least significant bits
  Wire.write((DACValue & 15) << 4);
  Wire.endTransmission();

  // Add a delay to wait for the PSU
  if (with_delay)
  {
    delay(SWEEP_TIME);
  }
}

// Resets the global state to be able to run the test again
void reset()
{
  // All of the pins going out to the relay board
  for (int i = 2; i < 10; i++)
  {
    digitalWrite(i, HIGH);
  }

  // The relays on the board are active high
  digitalWrite(DC_DY_FLIP_MEASURE_RELAY, LOW);
  digitalWrite(R_B_FLIP_MEASURE_RELAY, LOW);

  setInputVoltage(PSU_24V, true);

  for (int i = 0; i < 3; i++)
  {
    rows[i][0] = '\0';
  }
  current_row = 0;

  testResultsLength = 0;

  R_B_FLIP_RELAY_STATE = LOW;
  DC_DY_FLIP_RELAY_STATE = LOW;
}

int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void updateLCD()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Testar...");

  for (int i = 0; i < 3; i++)
  {
    lcd.setCursor(0, i + 1);
    lcd.print(rows[i]);
  }
}

void printTestStartLCD(String testName)
{
  char printString[21];

  snprintf(printString, sizeof(printString), "%s:", testName.c_str());

  if (strcmp(rows[current_row], printString) != 0)
  {
    return;
  }
  else if (strcmp(rows[2], "") != 0)
  {
    strcpy(rows[0], rows[1]);
    strcpy(rows[1], rows[2]);
  }

  strcpy(printString, rows[current_row]);

  updateLCD();
}

void getOneRowTestResultString(Test test, char *returnString)
{
  char sValue[7];
  dtostrf(test.value, 2, 2, sValue);

  if (test.success)
  {
    snprintf(returnString, 20, "%s: %s%s UA",
             test.name.c_str(), sValue, test.measureUnit.c_str());
  }
  else if (test.min >= test.value)
  {
    char sMin[7];
    dtostrf(test.min, 2, 2, sMin);
    snprintf(returnString, 20, "%s: %s%s<=%s%s",
             test.name.c_str(), sValue, test.measureUnit.c_str(), sMin, test.measureUnit.c_str());
  }
  else if (test.max <= test.value)
  {
    char sMax[7];
    dtostrf(test.max, 2, 2, sMax);
    snprintf(returnString, 20, "%s: %s%s>=%s%s",
             test.name.c_str(), sValue, test.measureUnit.c_str(), sMax, test.measureUnit.c_str());
  }
}

void printTestResultLCD(Test test)
{
  getOneRowTestResultString(test, rows[current_row]);

  if (current_row != 2)
  {
    current_row++;
  }

  updateLCD();
}

void printTestSerial(Test test)
{
  Serial.print("Test: ");
  Serial.println(test.name);

  Serial.print(test.min);
  Serial.print(test.measureUnit);
  Serial.print(" <= ");
  Serial.print(test.value);
  Serial.print(test.measureUnit);
  Serial.print(" <= ");
  Serial.print(test.max);
  Serial.println(test.measureUnit);

  if (test.success)
  {
    Serial.println("SUCCESS!");
  }
  else
  {
    Serial.println("FAIL!");
  }
  Serial.println();
}

void showTestResultOnFinishScreen(Test test)
{
  char sValue[7];
  dtostrf(test.value, 2, 2, sValue);
  char sMin[7];
  dtostrf(test.min, 2, 2, sMin);
  char sMax[7];
  dtostrf(test.max, 2, 2, sMax);

  snprintf(rows[1], 21, "Punkt %s %s%s", test.name.c_str(), sValue, test.measureUnit.c_str());
  snprintf(rows[2], 21, "Min: %s%s", sMin, test.measureUnit.c_str());
  snprintf(rows[3], 21, "Max: %s%s", sMax, test.measureUnit.c_str());
}

void enterFinishedState()
{
  int failed_count = 0;

  Test failedTests[5];

  for (int i = 0; i < testResultsLength; i++)
  {
    if (!testResults[i].success)
    {
      failedTests[failed_count] = testResults[i];
      failed_count++;
    }
  }

  lcd.clear();

  if (failed_count == 0)
  {
    lcd.setCursor(0, 0);
    lcd.print("Test resultat:");
    lcd.setCursor(0, 1);
    lcd.print("GT UA");

    waitForInput();
    return;
  }

  int selected_test = 0;

  showTestResultOnFinishScreen(failedTests[selected_test]);

  lcd.setCursor(0, 0);
  lcd.print(F("Misslyckat test"));
  lcd.setCursor(17, 0);
  lcd.print(selected_test + 1);
  lcd.print("/");
  lcd.print(failed_count);

  lcd.setCursor(0, 1);
  lcd.print(rows[1]);
  lcd.setCursor(0, 2);
  lcd.print(rows[2]);
  lcd.setCursor(0, 3);
  lcd.print(rows[3]);

  bool loop = true;

  while (loop)
  {
    if (isDownPressed())
    {
      if (isDownLongPressed())
      {
        loop = false;
      }
      else
      {
        selected_test = (selected_test + 1) % failed_count;
        showTestResultOnFinishScreen(failedTests[selected_test]);
        lcd.setCursor(0, 0);
        lcd.print(F("Misslyckat test"));
        lcd.setCursor(17, 0);
        lcd.print(selected_test + 1);
        lcd.print("/");
        lcd.print(failed_count);

        lcd.setCursor(0, 1);
        lcd.print(rows[1]);
        lcd.setCursor(0, 2);
        lcd.print(rows[2]);
        lcd.setCursor(0, 3);
        lcd.print(rows[3]);
      }
    }
    if (isUpPressed())
    {
      selected_test = (selected_test - 1 + failed_count) % failed_count;
      showTestResultOnFinishScreen(failedTests[selected_test]);
      lcd.setCursor(0, 0);
      lcd.print(F("Misslyckat test"));
      lcd.setCursor(17, 0);
      lcd.print(selected_test + 1);
      lcd.print("/");
      lcd.print(failed_count);

      lcd.setCursor(0, 1);
      lcd.print(rows[1]);
      lcd.setCursor(0, 2);
      lcd.print(rows[2]);
      lcd.setCursor(0, 3);
      lcd.print(rows[3]);
    }

    delay(5);
  }
}

void registerTest(Test test)
{
  printTestResultLCD(test);
  printTestSerial(test);

  testResults[testResultsLength] = test;
  testResultsLength++;
}

void sendToRelay(int relayPin, int value)
{
  digitalWrite(relayPin, value);
  delay(RELAY_DELAY);
}

void turnONRelay(int relayPin)
{
  if (relayPin == R_B_FLIP_MEASURE_RELAY)
  {
    R_B_FLIP_RELAY_STATE = HIGH;
    sendToRelay(relayPin, HIGH);
  }
  else if (relayPin == DC_DY_FLIP_MEASURE_RELAY)
  {
    DC_DY_FLIP_RELAY_STATE = HIGH;
    sendToRelay(relayPin, HIGH);
  }
  else
  {
    sendToRelay(relayPin, LOW);
  }
}

void turnOFFRelay(int relayPin)
{
  if (relayPin == R_B_FLIP_MEASURE_RELAY)
  {
    R_B_FLIP_RELAY_STATE = LOW;
    sendToRelay(relayPin, LOW);
  }
  else if (relayPin == DC_DY_FLIP_MEASURE_RELAY)
  {
    DC_DY_FLIP_RELAY_STATE = LOW;
    sendToRelay(relayPin, LOW);
  }
  else
  {
    sendToRelay(relayPin, HIGH);
  }
}

// Convert the analog reading 0-1023 to the voltage 0-5
float convertReadToVoltage(int read)
{
  return (read / 1023.0) * 5.0;
}

// Flips the relays and determines the correct pin to read
// Reads the voltage on the analog pin and converts analog read values (int 0-1023) to voltage (float 0-18)
float measureVoltage(MeasurePoint point)
{
  int read = 0;

  // Two resistors forming the voltage divider
  // Currently set to 0-18V mapped to 0-5V
  // VCC - R1 - Read - R2 - GND
  int R1 = 9900;
  int R2 = 2700;

  bool use_voltage_divider = true;

  switch (point)
  {
  case MeasurePoint::R:
    if (R_B_FLIP_RELAY_STATE != LOW)
    {
      turnOFFRelay(R_B_FLIP_MEASURE_RELAY);
    }
    read = analogRead(R_B_MEASURE_PIN);
    break;
  case MeasurePoint::B:
    if (R_B_FLIP_RELAY_STATE != HIGH)
    {
      turnONRelay(R_B_FLIP_MEASURE_RELAY);
    }
    read = analogRead(R_B_MEASURE_PIN);
    break;
  case MeasurePoint::DC:
    if (DC_DY_FLIP_RELAY_STATE != LOW)
    {
      turnOFFRelay(DC_DY_FLIP_MEASURE_RELAY);
    }
    read = analogRead(DC_DY_MEASURE_PIN);
    use_voltage_divider = false;
    break;
  case MeasurePoint::DY:
    if (DC_DY_FLIP_RELAY_STATE != HIGH)
    {
      turnONRelay(DC_DY_FLIP_MEASURE_RELAY);
    }
    read = analogRead(DC_DY_MEASURE_PIN);
    use_voltage_divider = false;
    break;
  }

  float voltage = convertReadToVoltage(read);

  if (!use_voltage_divider)
  {
    return voltage;
  }
  return (voltage * (R1 + R2)) / R2;
}

// For some measurements, i need a more stable read, so here i take the average of a few reads
float measureAverageVoltage(MeasurePoint point)
{
  float voltage_sum = 0.0;
  int measureTimes = 50;

  for (int i = 0; i < measureTimes; i++)
  {
    delay(5);
    voltage_sum += measureVoltage(point);
  }

  return voltage_sum / measureTimes;
}

// For the sweeping reads because they cannot be as slow as the other function.
float measureSemiAverageVoltage(MeasurePoint point)
{
  int count = 6;
  float voltage_sum = 0.0;
  for (int i = 0; i < count; i++)
  {
    voltage_sum += measureVoltage(point);
    delay(1);
  }

  return voltage_sum / count;
}

// Strömsensorn ger ut ett spänningsvärde där 0-VCC är mappad till 0-5A
// Om det är ingen ström så ger den VCC/2
// Om den ger ut 0V så är det 5A <-
// Om den ger ut VCC så är det 5A ->
// The current sensor seems to measure only 4.13V when 5A are meassured i.e only 1.63V+- 2.5V
// So i adjust in software
// It was very jumpy and not stable so i take the average of multiple readings
float measureCurrent()
{
  int measureTimes = 50;
  float currentSum = 0.0;

  for (int i = 0; i < measureTimes; i++)
  {
    float reading = convertReadToVoltage(analogRead(CURRENT_MEASURE_PIN));

    currentSum += (abs(reading - 2.5) / 1.63) * 5;
  }

  return currentSum / measureTimes;
}

bool askRetest(Test failedTest)
{
  if (failedTest.success)
  {
    return false;
  }

  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(F("Misslyckad Test:"));
  lcd.setCursor(0, 1);
  char resultString[21];
  getOneRowTestResultString(failedTest, resultString);
  lcd.print(resultString);
  lcd.setCursor(0, 2);
  lcd.print(F("Press Up: Retest"));
  lcd.setCursor(0, 3);
  lcd.print(F("Press Down: Continue"));

  while (!isDownPressed())
  {
    if (isUpPressed())
    {
      return true;
    }
  }

  return false;
}

// When its sweeped, we set the voltage without a delay
// Then while the PSU sweeps up to the set voltage, we measure
// as fast as possible for 2,5s. This gets a measure every 0.4V
Test sweepLoadedVoltageDiff()
{
  printTestStartLCD("5.3");

  setInputVoltage(PSU_10V, true);
  float currentVolt = measureAverageVoltage(MeasurePoint::R);
  float minV = currentVolt;
  float maxV = currentVolt;

  unsigned long start_time = millis();

  setInputVoltage(PSU_MAX, false);

  while (millis() - start_time <= SWEEP_TIME)
  {
    currentVolt = measureVoltage(MeasurePoint::R);
    minV = min(minV, currentVolt);
    maxV = max(maxV, currentVolt);
  }

  Test test = {
      "5.3",
      "V",
      0,
      0.3,
      (maxV - minV),
      (maxV - minV) <= 0.3};

  if (askRetest(test))
  {
    return sweepLoadedVoltageDiff();
  }

  return test;
}

Test sweepCurrentDiff()
{
  printTestStartLCD("5.2");

  setInputVoltage(PSU_10V, true);
  float current = measureCurrent();

  float minA = current;
  float maxA = current;

  unsigned long start_time = millis();

  setInputVoltage(PSU_MAX, false);

  while (millis() - start_time <= SWEEP_TIME)
  {
    current = measureCurrent();

    minA = min(minA, current);
    maxA = max(maxA, current);
  }

  Test test = {
      "5.2",
      "A",
      0,
      0.3,
      (maxA - minA),
      (maxA - minA) <= 0.3};

  if (askRetest(test))
  {
    return sweepCurrentDiff();
  }

  return test;
}

Test sweepUnloadedRadioVoltageDiff()
{
  printTestStartLCD("5.1R");
  setInputVoltage(PSU_10V, true);
  float currentVolt = measureAverageVoltage(MeasurePoint::R);

  unsigned long start_time = millis();

  float minV = currentVolt;
  float maxV = currentVolt;

  setInputVoltage(PSU_MAX, false);

  while (millis() - start_time <= SWEEP_TIME)
  {
    currentVolt = measureVoltage(MeasurePoint::R);

    minV = min(minV, currentVolt);
    maxV = max(maxV, currentVolt);
  }

  Test test = {
      "5.1R",
      "V",
      0,
      0.3,
      (maxV - minV),
      (maxV - minV) <= 0.3};

  if (askRetest(test))
  {
    return sweepUnloadedRadioVoltageDiff();
  }

  return test;
}

Test sweepUnloadedBatteryVoltageDiff()
{
  printTestStartLCD("5.1B");
  setInputVoltage(PSU_10V, true);

  float currentVolt = measureAverageVoltage(MeasurePoint::B);
  float minV = currentVolt;
  float maxV = currentVolt;

  unsigned long start_time = millis();

  setInputVoltage(PSU_MAX, false);

  while (millis() - start_time <= SWEEP_TIME)
  {
    currentVolt = measureVoltage(MeasurePoint::B);

    minV = min(minV, currentVolt);
    maxV = max(maxV, currentVolt);
  }

  Test test = {
      "5.1B",
      "V",
      0,
      0.3,
      (maxV - minV),
      (maxV - minV) <= 0.3};

  if (askRetest(test))
  {
    return sweepUnloadedBatteryVoltageDiff();
  }

  return test;
}

Test testTempSensor()
{
  printTestStartLCD("3.1");

  float dcVolt = measureAverageVoltage(MeasurePoint::DC);
  float dyVolt = measureAverageVoltage(MeasurePoint::DY);

  float diff = dcVolt - dyVolt;

  bool success = diff >= 0.03 && diff <= 0.045;

  int count = 0;

  // Add more delay and less retests for this measurement because it requires a relay flip
  while (!success && count < RETEST_COUNT_TEMPSENSOR)
  {
    delay(100);
    dcVolt = measureVoltage(MeasurePoint::DC);
    dyVolt = measureVoltage(MeasurePoint::DY);

    Serial.println(dcVolt);
    Serial.println(dyVolt);

    diff = dcVolt - dyVolt;
    Serial.println(diff);
    Serial.println();
    success = diff >= 0.03 && diff <= 0.045;

    count++;
  }

  Test test = {
    name : "3.1",
    "mV",
    30,
    45,
    diff * 1000,
    success
  };

  if (askRetest(test))
  {
    return testTempSensor();
  }

  return test;
}

Test testCurrent(String testName, float minA, float maxA)
{
  printTestStartLCD(testName);
  float current = measureCurrent();

  bool success = current >= minA && current <= maxA;

  int count = 0;

  while (!success && count < RETEST_COUNT)
  {
    delay(5);

    current = measureCurrent();

    success = current >= minA && current <= maxA;

    count++;
  }

  Test test = {
      testName,
      "A",
      minA,
      maxA,
      current,
      success};

  if (askRetest(test))
  {
    return testCurrent(testName, minA, maxA);
  }

  return test;
}

Test testVoltage(String testName, MeasurePoint point, float minV, float maxV)
{
  Serial.print("TESTING: ");
  Serial.println(testName);

  printTestStartLCD(testName);
  float volts = measureAverageVoltage(point);

  bool success = volts >= minV && volts <= maxV;

  int count = 0;
  while (!success && count < RETEST_COUNT)
  {
    delay(5);

    volts = measureAverageVoltage(point);

    success = volts >= minV && volts <= maxV;

    count++;
  }

  Test test = {
      testName,
      "V",
      minV,
      maxV,
      volts,
      success};

  if (askRetest(test))
  {
    return testVoltage(testName, point, minV, maxV);
  }

  return test;
}

void resetBPlus()
{
  // Strömen verkar inte villja återställa sig efter att den begränsas
  // Så för att resetta den så stänger jag av och på B+
  turnOFFRelay(B_PLUS_ON_RELAY);
  turnONRelay(B_PLUS_ON_RELAY);
}

void printReadyScreen()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F("Press the button"));
  lcd.setCursor(0, 2);
  lcd.print(F("to start!"));
}

float getPhotoReading()
{
  return convertReadToVoltage(analogRead(PHOTORESISTOR_PIN));
}

void runTest()
{
  Serial.print("Free Ram: ");
  Serial.println(freeRam());

  // Resets everything to be ready
  reset();

  // Detta är för att stänga av LADDAT Lampan
  // Den tänds under uppstart av någon anledning
  // Någonting med den lägre inspänningen och flytande spänningar säkert
  turnONRelay(B_MID_OFF_RELAY);
  turnOFFRelay(B_MID_OFF_RELAY);

  Serial.println("TESTING...");
  updateLCD();

  // MEASUREMENTS
  // 1.1
  turnONRelay(B_PLUS_ON_RELAY);
  registerTest(testVoltage("1.1", MeasurePoint::R, 12.3, 12.8));
  turnONRelay(R_ON_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 1.2
  turnONRelay(R_LOAD_RELAY);
  registerTest(testVoltage("1.2", MeasurePoint::R, 11.7, 12.8));
  turnOFFRelay(R_LOAD_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 2.1
  turnOFFRelay(B_PLUS_ON_RELAY);
  registerTest(testVoltage("2.1", MeasurePoint::B, 15.15, 15.90));
  float volts_2_1 = measureAverageVoltage(MeasurePoint::B);
  turnONRelay(B_PLUS_ON_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 2.2
  // Utspänningen ska även sjunka jämfört med förra mätningen
  registerTest(testCurrent("2.2a", 1.15, 1.35));
  printTestStartLCD("2.2b");
  float volts_2_2 = measureAverageVoltage(MeasurePoint::B);

  Test test_2_2b = {"2.2b",
                    "V",
                    0,
                    volts_2_1,
                    volts_2_2,
                    volts_2_2 < volts_2_1};

  while (askRetest(test_2_2b))
  {
    volts_2_2 = measureAverageVoltage(MeasurePoint::B);
    test_2_2b.value = volts_2_2;
    test_2_2b.success = volts_2_2 < volts_2_1;
  }

  registerTest(test_2_2b);

  delay(MEASURE_POINT_DELAY);

  // 3.1
  registerTest(testTempSensor());

  delay(MEASURE_POINT_DELAY);

  // 3.2
  // Kontrollera att "LADDAT" lampan tänds
  // Photoresistors lose resistance when its lit.
  // That means that the read value should increase when more lights turn on
  float photo_off_reading = getPhotoReading();

  turnONRelay(DY_DISCONNECT_RELAY);
  registerTest(testCurrent("3.2", 0.15, 0.25));
  turnOFFRelay(DY_DISCONNECT_RELAY);

  float photo_on_reading = getPhotoReading();

  Test test_3_2b = {"3.2b",
                    "V",
                    photo_off_reading,
                    photo_on_reading,
                    5,
                    photo_on_reading > photo_off_reading};

  while (askRetest(test_3_2b))
  {
    photo_on_reading = getPhotoReading();
    test_3_2b.value = photo_on_reading;
    test_3_2b.success = photo_on_reading > photo_off_reading;
  }

  registerTest(test_3_2b);
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print(F("Kontrollera att"));
  // lcd.setCursor(0, 1);
  // lcd.print(F("LADDAT lampan lyser"));
  // lcd.setCursor(0, 2);
  // lcd.print(F("To continue press"));
  // lcd.setCursor(0, 3);
  // lcd.print(F("the button"));

  // waitForButtonPress();

  // updateLCD();

  delay(MEASURE_POINT_DELAY);

  // 3.3
  // Kontrollera att "LADDAT" lampan släcks
  turnONRelay(B_MID_OFF_RELAY);
  resetBPlus();

  registerTest(testCurrent("3.3", 1.15, 1.35));
  turnOFFRelay(B_MID_OFF_RELAY);

  photo_off_reading = getPhotoReading();

  Test test_3_3b = {"3.3b",
                    "V",
                    0,
                    photo_off_reading,
                    photo_on_reading,
                    photo_off_reading < photo_on_reading};

  while (askRetest(test_3_3b))
  {
    photo_off_reading = getPhotoReading();
    test_3_3b.value = photo_off_reading;
    test_3_3b.success = photo_off_reading < photo_on_reading;
  }

  registerTest(test_3_3b);

  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print(F("Kolla att LADDAT"));
  // lcd.setCursor(0, 1);
  // lcd.print(F("lampan inte lyser"));
  // lcd.setCursor(0, 2);
  // lcd.print(F("To continue press"));
  // lcd.setCursor(0, 3);
  // lcd.print(F("the button"));

  // waitForButtonPress();

  // updateLCD();

  delay(MEASURE_POINT_DELAY);

  // 3.4
  turnONRelay(DY_GROUND_RELAY);
  registerTest(testCurrent("3.4", 0.4, 0.6));
  turnOFFRelay(DY_GROUND_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 3.5
  turnONRelay(DC_GROUND_RELAY);
  registerTest(testCurrent("3.5", 0.15, 0.25));
  turnOFFRelay(DC_GROUND_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 3.6
  // Behöver resetta B-plus för att verkligen se att den minskar strömmen
  resetBPlus();
  turnONRelay(DC_DISCONNECT_RELAY);
  registerTest(testCurrent("3.6", 0.4, 0.6));
  turnOFFRelay(DC_DISCONNECT_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 4.1
  resetBPlus();
  turnONRelay(R_LOAD_RELAY);
  registerTest(testCurrent("4.1", 1.15, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 4.3
  setInputVoltage(PSU_15V, true);
  registerTest(testCurrent("4.3", 1.15, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 4.2
  setInputVoltage(PSU_12V, true);
  registerTest(testCurrent("4.2", 0.15, 0.25));

  delay(MEASURE_POINT_DELAY);

  // 4.4
  setInputVoltage(PSU_11V5, true);
  registerTest(testCurrent("4.4a", 0.15, 0.25));
  turnOFFRelay(R_LOAD_RELAY);
  resetBPlus();
  registerTest(testCurrent("4.4b", 1.15, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 5.1
  turnOFFRelay(R_ON_RELAY);
  turnOFFRelay(B_PLUS_ON_RELAY);
  registerTest(sweepUnloadedRadioVoltageDiff());
  registerTest(sweepUnloadedBatteryVoltageDiff());
  turnONRelay(B_PLUS_ON_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 5.2
  registerTest(sweepCurrentDiff());

  delay(MEASURE_POINT_DELAY);

  // 5.3
  turnONRelay(R_ON_RELAY);
  turnONRelay(R_LOAD_RELAY);
  registerTest(sweepLoadedVoltageDiff());
  turnOFFRelay(R_LOAD_RELAY);

  delay(MEASURE_POINT_DELAY);

  // Reset the input voltage just because
  setInputVoltage(PSU_24V, false);

  // Turn off all relays as a security measure
  // Now the default of relays connect no load to the strömdon
  for (int i = 2; i < 10; i++)
  {
    digitalWrite(i, HIGH);
  }

  digitalWrite(DC_DY_FLIP_MEASURE_RELAY, LOW);
  digitalWrite(R_B_FLIP_MEASURE_RELAY, LOW);

  bool success = true;

  for (int i = 0; i < testResultsLength; i++)
  {
    success &= testResults[i].success;
  }

  Serial.println(F("TEST FINISHED:"));
  Serial.println(success ? "SUCCESS" : "FAILED");

  Serial.print("Free Ram: ");
  Serial.println(freeRam());

  enterFinishedState();

  // printReadyScreen();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  Wire.begin();

  lcd.init();
  lcd.backlight();

  analogReference(EXTERNAL);

  // Sets all sequential relays pins to output
  // OBS Relays on the breakout board are active low
  for (int i = 2; i < 10; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }

  // The relays on the board are active high
  pinMode(DC_DY_FLIP_MEASURE_RELAY, OUTPUT);
  digitalWrite(DC_DY_FLIP_MEASURE_RELAY, LOW);
  pinMode(R_B_FLIP_MEASURE_RELAY, OUTPUT);
  digitalWrite(R_B_FLIP_MEASURE_RELAY, LOW);

  pinMode(SWITCH_UP_PIN, INPUT_PULLUP);
  pinMode(SWITCH_DOWN_PIN, INPUT_PULLUP);

  pinMode(PHOTORESISTOR_PIN, INPUT);
  pinMode(CURRENT_MEASURE_PIN, INPUT);
  pinMode(R_B_MEASURE_PIN, INPUT);
  pinMode(DC_DY_MEASURE_PIN, INPUT);

  printReadyScreen();
}

void loop()
{
  if (isAnyPressed())
  {
    runTest();
  }

  delay(5);
}