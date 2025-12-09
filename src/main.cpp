// V2
// Nuvarande (251111) så är detta en gammal version av mjukvaran
// modifierad för V0.2 av kortet
// stora skillnader är reläerna på kortet, fotomotstånd och vippbrytare istället för en knapp
// Bäst är att uppdatera detta först till att likna hur den nya
// V1 mjukvaran ser ut, innan denna slipas up

#include <common.h>

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
#define SWITCH_UP_PIN 12
#define SWITCH_DOWN_PIN 13

#define PHOTORESISTOR_PIN A0
#define CURRENT_MEASURE_PIN A1
#define R_B_MEASURE_PIN A2
#define DC_DY_MEASURE_PIN A3

void turnONRelay(int relayPin)
{
  if (relayPin == R_B_FLIP_MEASURE_RELAY)
  {
    R_B_FLIP_RELAY_ON = true;
    sendToRelay(relayPin, HIGH);
  }
  else if (relayPin == DC_DY_FLIP_MEASURE_RELAY)
  {
    DC_DY_FLIP_RELAY_ON = true;
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
    R_B_FLIP_RELAY_ON = false;
    sendToRelay(relayPin, LOW);
  }
  else if (relayPin == DC_DY_FLIP_MEASURE_RELAY)
  {
    DC_DY_FLIP_RELAY_ON = false;
    sendToRelay(relayPin, LOW);
  }
  else
  {
    sendToRelay(relayPin, HIGH);
  }
}

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

void enterFinishedState()
{
  int failed_count = 0;

  Test failed_tests[9];

  for (int i = 0; i < testResultsLength; i++)
  {
    if (!isSuccess(testResults[i]) && failed_count < 9)
    {
      failed_tests[failed_count] = testResults[i];
      failed_count++;
    }
  }

  lcd.clear();

  if (failed_count == 0)
  {
    lcd.setCursor(0, 0);
    lcd.print(F("Test resultat:"));
    lcd.setCursor(0, 1);
    lcd.print(F("GT UA"));

    waitForInput();
    waitForRelease();
    return;
  }

  int selected_test = 0;

  showTestResultOnFinishScreen(failed_tests[selected_test]);

  lcd.setCursor(0, 0);
  lcd.print(F("Misslyckat test"));
  lcd.setCursor(17, 3);
  lcd.print(selected_test + 1);
  lcd.print(F("/"));
  lcd.print(failed_count);

  lcd.setCursor(0, 1);
  lcd.print(rows[1]);
  lcd.setCursor(0, 2);
  lcd.print(rows[2]);
  lcd.setCursor(0, 3);
  lcd.print(rows[3]);

  while (true)
  {
    if (!isAnyPressed())
    {
      delay(5);
      continue;
    }

    if (isDownPressed())
    {
      if (isDownLongPressed())
      {
        lcd.clear();
        waitForRelease();
        break;
      }
      selected_test = (selected_test + 1) % failed_count;
    }
    else if (isUpPressed())
    {
      waitForRelease();
      selected_test = (selected_test - 1 + failed_count) % failed_count;
    }

    lcd.clear();
    showTestResultOnFinishScreen(failed_tests[selected_test]);

    lcd.setCursor(0, 0);
    lcd.print(F("Misslyckat test"));
    lcd.setCursor(17, 3);
    lcd.print(selected_test + 1);
    lcd.print(F("/"));
    lcd.print(failed_count);

    lcd.setCursor(0, 1);
    lcd.print(rows[1]);
    lcd.setCursor(0, 2);
    lcd.print(rows[2]);
    lcd.setCursor(0, 3);
    lcd.print(rows[3]);
  }
}

// Flips the relays and determines the correct pin to read
// Reads the voltage on the analog pin and converts analog read values (int 0-1023) to voltage (float 0-18)
float measureVoltage(MeasurePoint point)
{
  int read = 0;

  // Two resistors forming the voltage divider
  // Currently set to 0-18V mapped to 0-5V
  // VCC - R1 - Read - R2 - GND
  int R1 = 10000;
  int R2 = 3300;

  bool use_voltage_divider = true;

  switch (point)
  {
  case MeasurePoint::R:
    if (R_B_FLIP_RELAY_ON != false)
    {
      turnOFFRelay(R_B_FLIP_MEASURE_RELAY);
    }
    read = analogRead(R_B_MEASURE_PIN);
    break;
  case MeasurePoint::B:
    if (R_B_FLIP_RELAY_ON != true)
    {
      turnONRelay(R_B_FLIP_MEASURE_RELAY);
    }
    read = analogRead(R_B_MEASURE_PIN);
    break;
  case MeasurePoint::DC:
    if (DC_DY_FLIP_RELAY_ON != false)
    {
      turnOFFRelay(DC_DY_FLIP_MEASURE_RELAY);
    }
    read = analogRead(DC_DY_MEASURE_PIN);
    use_voltage_divider = false;
    break;
  case MeasurePoint::DY:
    if (DC_DY_FLIP_RELAY_ON != true)
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

// Strömsensorn ger ut ett spänningsvärde med 185mV/A
// Om det är ingen ström så ger den VCC/2
// It was very jumpy and not stable so i take the average of multiple readings
float measureCurrent()
{
  int measureTimes = 200;
  float currentSum = 0.0;

  for (int i = 0; i < measureTimes; i++)
  {
    float reading = convertReadToVoltage(analogRead(CURRENT_MEASURE_PIN));

    currentSum += abs(reading - 2.5) / 0.185;
  }

  return (currentSum / measureTimes);
}

bool askRetest(Test failedTest)
{
  if (isSuccess(failedTest))
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
  lcd.print(F("Up: Retest"));
  lcd.setCursor(0, 3);
  lcd.print(F("Down: Continue"));

  while (true)
  {
    if (isUpPressed())
    {
      updateLCD();
      return true;
    }
    else if (isDownPressed())
    {
      return false;
    }
    delay(5);
  }
}

float getPhotoReading()
{
  return convertReadToVoltage(analogRead(PHOTORESISTOR_PIN));
}

Test testChargingLampOn(float initialReading)
{
  printTestStartLCD("3.2b");

  float newReading = getPhotoReading();

  Test test = {
      "3.2b",
      MeasureUnit::V,
      initialReading + 0.1,
      5.0,
      newReading};

  if (askRetest(test))
  {
    return testChargingLampOn(initialReading);
  }

  return test;
}

Test testChargingLampOff(float initialReading)
{
  printTestStartLCD("3.3b");

  float newReading = getPhotoReading();

  Test test = {
      "3.3b",
      MeasureUnit::V,
      0,
      initialReading - 0.1,
      newReading};

  if (askRetest(test))
  {
    return testChargingLampOff(initialReading);
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

void runTest()
{
  // Resets everything to be ready
  reset();

  // Detta är för att stänga av LADDAT Lampan
  // Den tänds under uppstart av någon anledning
  // Någonting med den lägre inspänningen och flytande spänningar säkert
  turnONRelay(B_MID_OFF_RELAY);
  turnOFFRelay(B_MID_OFF_RELAY);

  Serial.println(F("TESTING..."));
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
  registerTest(testCurrent("2.2a", 1.10, 1.35));
  printTestStartLCD("2.2b");
  float volts_2_2 = measureAverageVoltage(MeasurePoint::B);

  Test test_2_2b = {"2.2b",
                    MeasureUnit::V,
                    0,
                    volts_2_1,
                    volts_2_2};

  int count = 0;
  while (!isSuccess(test_2_2b) && count < RETEST_COUNT)
  {
    volts_2_2 = measureAverageVoltage(MeasurePoint::B);
    test_2_2b.value = volts_2_2;
    count++;
    delay(5);
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
  float photoResistorStart = getPhotoReading();

  turnONRelay(DY_DISCONNECT_RELAY);
  registerTest(testCurrent("3.2", 0.15, 0.43));
  turnOFFRelay(DY_DISCONNECT_RELAY);

  registerTest(testChargingLampOn(photoResistorStart));

  delay(MEASURE_POINT_DELAY);

  // 3.3
  photoResistorStart = getPhotoReading();
  turnONRelay(B_MID_OFF_RELAY);
  resetBPlus();

  registerTest(testCurrent("3.3", 1.10, 1.35));
  turnOFFRelay(B_MID_OFF_RELAY);

  registerTest(testChargingLampOff(photoResistorStart));

  delay(MEASURE_POINT_DELAY);

  // 3.4
  turnONRelay(DY_GROUND_RELAY);
  registerTest(testCurrent("3.4", 0.4, 0.75));
  turnOFFRelay(DY_GROUND_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 3.5
  turnONRelay(DC_GROUND_RELAY);
  registerTest(testCurrent("3.5", 0.15, 0.43));
  turnOFFRelay(DC_GROUND_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 3.6
  // Behöver resetta B-plus för att verkligen se att den minskar strömmen
  resetBPlus();
  turnONRelay(DC_DISCONNECT_RELAY);
  registerTest(testCurrent("3.6", 0.4, 0.75));
  turnOFFRelay(DC_DISCONNECT_RELAY);

  delay(MEASURE_POINT_DELAY);

  // 4.1
  resetBPlus();
  turnONRelay(R_LOAD_RELAY);
  registerTest(testCurrent("4.1", 1.10, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 4.3
  setInputVoltage(PSU_15V, true);
  registerTest(testCurrent("4.3", 1.10, 1.35));

  delay(MEASURE_POINT_DELAY);

  // 4.2
  setInputVoltage(PSU_12V, true);
  registerTest(testCurrent("4.2", 0.15, 0.43));

  delay(MEASURE_POINT_DELAY);

  // 4.4
  setInputVoltage(PSU_11V5, true);
  registerTest(testCurrent("4.4a", 0.15, 0.43));
  turnOFFRelay(R_LOAD_RELAY);
  resetBPlus();
  registerTest(testCurrent("4.4b", 1.10, 1.35));

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
  for (int i = 2; i < 12; i++)
  {
    digitalWrite(i, HIGH);
  }

  bool success = true;

  for (int i = 0; i < testResultsLength; i++)
  {
    success &= isSuccess(testResults[i]);
  }

  Serial.println(F("TEST FINISHED:"));
  Serial.println(success ? F("SUCCESS") : F("FAILED"));

  enterFinishedState();

  printReadyScreen();
}

void setup()
{
  // for debugging
  // debug_init();

  // put your setup code here, to run once:
  Serial.begin(9600);

  Wire.begin();

  lcd.init();
  lcd.backlight();

  analogReference(EXTERNAL);

  // Sets all sequential relays pins to output
  // OBS Relays on the breakout board are active low
  for (int i = 2; i < 12; i++)
  {
    pinMode(i, OUTPUT);
    turnOFFRelay(i);
  }

  pinMode(SWITCH_UP_PIN, INPUT_PULLUP);
  pinMode(SWITCH_DOWN_PIN, INPUT_PULLUP);

  pinMode(CURRENT_MEASURE_PIN, INPUT);
  pinMode(R_B_MEASURE_PIN, INPUT);
  pinMode(DC_DY_MEASURE_PIN, INPUT);

  printReadyScreen();

  setInputVoltage(PSU_24V, false);
}

void loop()
{
  if (isAnyPressed())
  {
    lcd.clear();
    runTest();
  }

  delay(5);
}