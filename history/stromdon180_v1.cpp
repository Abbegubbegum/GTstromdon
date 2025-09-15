#include <Arduino.h>

// Wire library for I2C communication
#include <Wire.h>

// For the I2C LCD
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

// For string conversion functions
#include <stdlib.h>

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
#define R_B_FLIP_MEASURE_RELAY 10
#define DC_DY_FLIP_MEASURE_RELAY 11
#define BUTTON_PIN 13

#define CURRENT_MEASURE_PIN A0
#define R_B_MEASURE_PIN A1
#define DC_DY_MEASURE_PIN A2

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

bool R_B_FLIP_RELAY_STATE = HIGH;
bool DC_DY_FLIP_RELAY_STATE = HIGH;

// For the LCD test printing
char rows[3][21];
int current_row = 0;

// For saving the test results for the final result screen.
Test testResults[20] = {};
int testResultsLength = 0;

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
  for (int i = 2; i < 12; i++)
  {
    digitalWrite(i, HIGH);
  }

  setInputVoltage(PSU_24V, true);

  for (int i = 0; i < 3; i++)
  {
    rows[i][0] = '\0';
  }
  current_row = 0;

  testResultsLength = 0;

  R_B_FLIP_RELAY_STATE = HIGH;
  DC_DY_FLIP_RELAY_STATE = HIGH;
}

int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

void waitForButtonPress()
{
  while (digitalRead(BUTTON_PIN) != LOW)
  {
    delay(10);
  }
}

void registerTest(Test test)
{
  testResults[testResultsLength] = test;
  testResultsLength++;
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
  if (strcmp(rows[2], "") != 0)
  {
    strcpy(rows[0], rows[1]);
    strcpy(rows[1], rows[2]);
  }

  snprintf(rows[current_row], sizeof(rows[current_row]), "%s:", testName.c_str());

  updateLCD();
}

void printTestResultLCD(Test test)
{
  char sValue[7];
  dtostrf(test.value, 2, 2, sValue);

  if (test.success)
  {
    snprintf(rows[current_row], sizeof(rows[current_row]), "%s: %s%s%s",
             test.name.c_str(), sValue, test.measureUnit.c_str(), test.success ? " UA" : "");
  }
  else if (test.min >= test.value)
  {
    char sMin[7];
    dtostrf(test.min, 2, 2, sMin);
    snprintf(rows[current_row], sizeof(rows[current_row]), "%s: %s%s<=%s%s",
             test.name.c_str(), sValue, test.measureUnit.c_str(), sMin, test.measureUnit.c_str());
  }
  else if (test.max <= test.value)
  {
    char sMax[7];
    dtostrf(test.max, 2, 2, sMax);
    snprintf(rows[current_row], sizeof(rows[current_row]), "%s: %s%s>=%s%s",
             test.name.c_str(), sValue, test.measureUnit.c_str(), sMax, test.measureUnit.c_str());
  }

  if (current_row != 2)
  {
    current_row++;
  }

  updateLCD();
}

void printFinishedTestLCD()
{
  int failed_count = 0;

  for (int i = 0; i < testResultsLength; i++)
  {
    if (!testResults[i].success)
    {
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
  }
  else if (failed_count == 1)
  {
    for (int i = 0; i < testResultsLength; i++)
    {
      if (!testResults[i].success)
      {
        Test test = testResults[i];
        lcd.setCursor(0, 0);
        lcd.print("Misslyckad Test");

        lcd.setCursor(0, 1);
        lcd.print("Punkt " + test.name + " ");
        lcd.print(test.value);
        lcd.print(test.measureUnit);

        lcd.setCursor(0, 2);
        lcd.print("Min: ");
        lcd.print(test.min + test.measureUnit);

        lcd.setCursor(0, 3);
        lcd.print("Max: ");
        lcd.print(test.max + test.measureUnit);
      }
    }
  }
  else if (failed_count <= 3)
  {
    int current_row = 1;
    lcd.setCursor(0, 0);
    lcd.print("Misslyckade Tester");

    for (int i = 0; i < testResultsLength; i++)
    {
      if (!testResults[i].success)
      {
        lcd.setCursor(0, current_row);
        Test test = testResults[i];

        lcd.print(test.name + ": ");
        lcd.print(test.value);
        lcd.print(test.measureUnit);

        if (test.min >= test.value)
        {
          lcd.print("<=");
          lcd.print(test.min);
          lcd.print(test.measureUnit);
        }
        else if (test.max <= test.value)
        {
          lcd.print(">=");
          lcd.print(test.max);
          lcd.print(test.measureUnit);
        }

        current_row++;
      }
    }
  }
  else
  {
    String failed_tests = "";
    for (int i = 0; i < testResultsLength; i++)
    {
      if (!testResults[i].success)
      {
        if (failed_tests != "")
        {
          failed_tests += ", ";
        }
        failed_tests += testResults[i].name;
      }
    }
    lcd.setCursor(0, 0);
    lcd.print("Misslyckade Tester");
    lcd.setCursor(0, 1);
    if (failed_tests.length() <= 20)
    {
      lcd.print(failed_tests);
    }
    else if (failed_tests.length() > 20)
    {
      lcd.print(failed_tests.substring(0, 20));
      lcd.setCursor(0, 2);
      lcd.print(failed_tests.substring(20));
    }
  }
}

void sendToRelay(int relayPin, int value)
{
  digitalWrite(relayPin, value);

  delay(RELAY_DELAY);
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
    if (R_B_FLIP_RELAY_STATE != HIGH)
    {
      R_B_FLIP_RELAY_STATE = HIGH;
      sendToRelay(R_B_FLIP_MEASURE_RELAY, HIGH);
    }
    read = analogRead(R_B_MEASURE_PIN);
    break;
  case MeasurePoint::B:
    if (R_B_FLIP_RELAY_STATE != LOW)
    {
      R_B_FLIP_RELAY_STATE = LOW;
      sendToRelay(R_B_FLIP_MEASURE_RELAY, LOW);
    }
    read = analogRead(R_B_MEASURE_PIN);
    break;
  case MeasurePoint::DC:
    if (DC_DY_FLIP_RELAY_STATE != HIGH)
    {
      DC_DY_FLIP_RELAY_STATE = HIGH;
      sendToRelay(DC_DY_FLIP_MEASURE_RELAY, HIGH);
    }
    read = analogRead(DC_DY_MEASURE_PIN);
    use_voltage_divider = false;
    break;
  case MeasurePoint::DY:
    if (DC_DY_FLIP_RELAY_STATE != LOW)
    {
      DC_DY_FLIP_RELAY_STATE = LOW;
      sendToRelay(DC_DY_FLIP_MEASURE_RELAY, LOW);
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

  printTestResultLCD(test);
  printTestSerial(test);

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

  printTestResultLCD(test);
  printTestSerial(test);

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

  printTestResultLCD(test);
  printTestSerial(test);

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

  printTestResultLCD(test);
  printTestSerial(test);

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

  printTestResultLCD(test);
  printTestSerial(test);

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

  printTestResultLCD(test);
  printTestSerial(test);

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

  printTestResultLCD(test);
  printTestSerial(test);

  return test;
}

void resetBPlus()
{
  // Strömen verkar inte villja återställa sig efter att den begränsas
  // Så för att resetta den så stänger jag av och på B+
  sendToRelay(B_PLUS_ON_RELAY, HIGH);
  sendToRelay(B_PLUS_ON_RELAY, LOW);
}

void runTest()
{
  Serial.print("Free Ram: ");
  Serial.println(freeRam());
  // Resets everything to be ready
  reset();

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(F("Press the button"));
  lcd.setCursor(0, 2);
  lcd.print(F("to start!"));

  // Detta är för att stänga av LADDAT Lampan
  // Den tänds under uppstart av någon anledning
  // Någonting med den lägre inspänningen och flytande spänningar säkert
  sendToRelay(B_MID_OFF_RELAY, LOW);
  sendToRelay(B_MID_OFF_RELAY, HIGH);

  waitForButtonPress();

  Serial.println("TESTING...");
  updateLCD();

  // MEASUREMENTS
  // 1.1
  sendToRelay(B_PLUS_ON_RELAY, LOW);
  registerTest(testVoltage("1.1", MeasurePoint::R, 12.3, 12.8));
  sendToRelay(R_ON_RELAY, LOW);

  delay(MEASURE_POINT_DELAY);

  // 1.2
  sendToRelay(R_LOAD_RELAY, LOW);
  registerTest(testVoltage("1.2", MeasurePoint::R, 11.7, 12.8));
  sendToRelay(R_LOAD_RELAY, HIGH);

  delay(MEASURE_POINT_DELAY);

  // 2.1
  sendToRelay(B_PLUS_ON_RELAY, HIGH);
  registerTest(testVoltage("2.1", MeasurePoint::B, 15.15, 15.90));
  float volts_2_1 = measureAverageVoltage(MeasurePoint::B);
  sendToRelay(B_PLUS_ON_RELAY, LOW);

  delay(MEASURE_POINT_DELAY);

  // 2.2
  // Utspänningen ska även sjunka jämfört med förra mätningen
  registerTest(testCurrent("2.2a", 1.15, 1.35));
  float volts_2_2 = measureAverageVoltage(MeasurePoint::B);
  registerTest({"2.2b",
                "V",
                0,
                volts_2_1,
                volts_2_2,
                volts_2_2 < volts_2_1});

  delay(MEASURE_POINT_DELAY);

  // 3.1
  registerTest(testTempSensor());

  delay(MEASURE_POINT_DELAY);

  // 3.2
  // Kontrollera att "LADDAT" lampan tänds
  sendToRelay(DY_DISCONNECT_RELAY, LOW);
  registerTest(testCurrent("3.2", 0.15, 0.27));
  sendToRelay(DY_DISCONNECT_RELAY, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Kontrollera att"));
  lcd.setCursor(0, 1);
  lcd.print(F("LADDAT lampan lyser"));
  lcd.setCursor(0, 2);
  lcd.print(F("To continue press"));
  lcd.setCursor(0, 3);
  lcd.print(F("the button"));

  waitForButtonPress();

  updateLCD();

  delay(MEASURE_POINT_DELAY);

  // 3.3
  // Kontrollera att "LADDAT" lampan släcks
  sendToRelay(B_MID_OFF_RELAY, LOW);
  resetBPlus();

  registerTest(testCurrent("3.3", 1.15, 1.35));
  sendToRelay(B_MID_OFF_RELAY, HIGH);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Kolla att LADDAT"));
  lcd.setCursor(0, 1);
  lcd.print(F("lampan inte lyser"));
  lcd.setCursor(0, 2);
  lcd.print(F("To continue press"));
  lcd.setCursor(0, 3);
  lcd.print(F("the button"));

  waitForButtonPress();

  updateLCD();

  delay(MEASURE_POINT_DELAY);

  // 3.4
  sendToRelay(DY_GROUND_RELAY, LOW);
  registerTest(testCurrent("3.4", 0.4, 0.6));
  sendToRelay(DY_GROUND_RELAY, HIGH);

  delay(MEASURE_POINT_DELAY);

  // 3.5
  sendToRelay(DC_GROUND_RELAY, LOW);
  registerTest(testCurrent("3.5", 0.15, 0.27));
  sendToRelay(DC_GROUND_RELAY, HIGH);

  delay(MEASURE_POINT_DELAY);

  // 3.6
  // Behöver resetta B-plus för att verkligen se att den minskar strömmen
  resetBPlus();
  sendToRelay(DC_DISCONNECT_RELAY, LOW);
  registerTest(testCurrent("3.6", 0.4, 0.6));
  sendToRelay(DC_DISCONNECT_RELAY, HIGH);

  delay(MEASURE_POINT_DELAY);

  // 4.1
  resetBPlus();
  sendToRelay(R_LOAD_RELAY, LOW);
  registerTest(testCurrent("4.1", 1.15, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 4.3
  setInputVoltage(PSU_15V, true);
  registerTest(testCurrent("4.3", 1.15, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 4.2
  setInputVoltage(PSU_12V, true);
  registerTest(testCurrent("4.2", 0.15, 0.27));

  delay(MEASURE_POINT_DELAY);

  // 4.4
  setInputVoltage(PSU_11V5, true);
  registerTest(testCurrent("4.4a", 0.15, 0.27));
  sendToRelay(R_LOAD_RELAY, HIGH);
  resetBPlus();
  registerTest(testCurrent("4.4b", 1.15, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 5.1
  sendToRelay(R_ON_RELAY, HIGH);
  sendToRelay(B_PLUS_ON_RELAY, HIGH);
  registerTest(sweepUnloadedRadioVoltageDiff());
  registerTest(sweepUnloadedBatteryVoltageDiff());
  sendToRelay(B_PLUS_ON_RELAY, LOW);

  delay(MEASURE_POINT_DELAY);

  // 5.2
  registerTest(sweepCurrentDiff());

  delay(MEASURE_POINT_DELAY);

  // 5.3
  sendToRelay(R_ON_RELAY, LOW);
  sendToRelay(R_LOAD_RELAY, LOW);
  registerTest(sweepLoadedVoltageDiff());
  sendToRelay(R_LOAD_RELAY, HIGH);

  delay(MEASURE_POINT_DELAY);

  // Reset the input voltage just because
  setInputVoltage(PSU_24V, false);

  // Turn off all relays as a security measure
  // Now the default of relays connect no load to the strömdon
  for (int i = 2; i < 12; i++)
  {
    digitalWrite(i, HIGH);
  }

  printFinishedTestLCD();

  bool success = true;

  for (int i = 0; i < testResultsLength; i++)
  {
    success &= testResults[i].success;
  }

  Serial.println(F("TEST FINISHED:"));
  Serial.println(success ? "SUCCESS" : "FAILED");

  Serial.print("Free Ram: ");
  Serial.println(freeRam());

  // Om man trycker på knappen så startar testet om
  waitForButtonPress();

  lcd.clear();

  runTest();
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  Wire.begin();

  lcd.init();
  lcd.backlight();

  // Not required when arduino not powered by USB
  // analogReference(EXTERNAL);

  // Sets all sequential relays pins to output
  // OBS Relays are active low
  for (int i = 2; i < 12; i++)
  {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(CURRENT_MEASURE_PIN, INPUT);
  pinMode(R_B_MEASURE_PIN, INPUT);
  pinMode(DC_DY_MEASURE_PIN, INPUT);

  runTest();
}

void loop()
{
}