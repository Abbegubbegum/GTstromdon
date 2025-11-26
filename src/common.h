#pragma once

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

// The delay after triggering the relay, as i noticed the relays are too slow
#define RELAY_DELAY 30

// The delay inbetween the measurments, hopefully only for debugging
#define MEASURE_POINT_DELAY 100

// PSU calculated values because the PSU does not seem very accurate so we have to calculate
// what target voltage actually gives the correct out voltage
#define PSU_10V 9.4
#define PSU_11V5 11
#define PSU_12V 11
#define PSU_15V 13.9
#define PSU_24V 21.3
#define PSU_MAX 28

// The ardunio will remeassure the test if it failed up to this amount of remeassures
#define RETEST_COUNT 10
#define RETEST_COUNT_CURRENT 100
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

enum MeasureUnit
{
    V,
    A,
    mV
};

const char *units[] = {
    "V",
    "A",
    "mV"};

struct Test
{
    const char *name;
    MeasureUnit measureUnit;
    float min;
    float max;
    float value;
};

bool R_B_FLIP_RELAY_ON = false;
bool DC_DY_FLIP_RELAY_ON = false;

// For the LCD test printing
char rows[3][20];
int current_row = 0;

// For saving the test results for the final result screen.
Test testResults[20] = {};
int testResultsLength = 0;

void turnONRelay(int relayPin);
void turnOFFRelay(int relayPin);
bool isAnyPressed();
float measureVoltage(MeasurePoint point);
float measureCurrent();
bool askRetest(Test failedTest);

void sendToRelay(int relayPin, int value)
{
    digitalWrite(relayPin, value);
    delay(RELAY_DELAY);
}

bool isSuccess(Test test)
{
    return test.min <= test.value && test.value <= test.max;
}

void waitForInput()
{
    while (!isAnyPressed())
    {
        delay(5);
    }
}

void waitForRelease()
{
    while (isAnyPressed())
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
    // All of the pins going out to relays
    for (int i = 2; i < 12; i++)
    {
        turnOFFRelay(i);
    }

    setInputVoltage(PSU_24V, true);

    for (int i = 0; i < 3; i++)
    {
        rows[i][0] = '\0';
    }
    current_row = 0;

    testResultsLength = 0;
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

    if (strcmp(rows[current_row], printString) == 0)
    {
        return;
    }
    else if (strcmp(rows[2], "") != 0)
    {
        strcpy(rows[0], rows[1]);
        strcpy(rows[1], rows[2]);
    }

    strcpy(rows[current_row], printString);

    updateLCD();
}

void getOneRowTestResultString(Test test, char *returnString)
{
    char sValue[7];
    dtostrf(test.value, 2, 2, sValue);

    if (isSuccess(test))
    {
        snprintf(returnString, 20, "%s: %s%s UA",
                 test.name, sValue, units[test.measureUnit]);
    }
    else if (test.min >= test.value)
    {
        char sMin[7];
        dtostrf(test.min, 2, 2, sMin);
        snprintf(returnString, 20, "%s: %s%s<=%s%s",
                 test.name, sValue, units[test.measureUnit], sMin, units[test.measureUnit]);
    }
    else if (test.max <= test.value)
    {
        char sMax[7];
        dtostrf(test.max, 2, 2, sMax);
        snprintf(returnString, 20, "%s: %s%s>=%s%s",
                 test.name, sValue, units[test.measureUnit], sMax, units[test.measureUnit]);
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
    Serial.print(F("Test: "));
    Serial.println(test.name);

    Serial.print(test.min);
    Serial.print(units[test.measureUnit]);
    Serial.print(F(" <= "));
    Serial.print(test.value);
    Serial.print(units[test.measureUnit]);
    Serial.print(F(" <= "));
    Serial.print(test.max);
    Serial.println(units[test.measureUnit]);

    if (isSuccess(test))
    {
        Serial.println(F("SUCCESS!"));
    }
    else
    {
        Serial.println(F("FAIL!"));
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

    snprintf(rows[1], 21, "Punkt %s %s%s", test.name, sValue, units[test.measureUnit]);
    snprintf(rows[2], 21, "Min: %s%s", sMin, units[test.measureUnit]);
    snprintf(rows[3], 21, "Max: %s%s", sMax, units[test.measureUnit]);
}

void registerTest(Test test)
{
    printTestResultLCD(test);
    printTestSerial(test);

    testResults[testResultsLength] = test;
    testResultsLength++;
}

// Convert the analog reading 0-1023 to the voltage 0-5
float convertReadToVoltage(int read)
{
    return (read / 1023.0) * 5.0;
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
        MeasureUnit::V,
        0,
        0.3,
        (maxV - minV)};

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
        MeasureUnit::A,
        0,
        0.3,
        (maxA - minA)};

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
        MeasureUnit::V,
        0,
        0.3,
        (maxV - minV)};

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
        MeasureUnit::V,
        0,
        0.3,
        (maxV - minV)};

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

        diff = dcVolt - dyVolt;
        success = diff >= 0.03 && diff <= 0.045;

        count++;
    }

    Test test = {
        name : "3.1",
        MeasureUnit::mV,
        30,
        45,
        diff * 1000
    };

    if (askRetest(test))
    {
        return testTempSensor();
    }

    return test;
}

Test testCurrent(const char *testName, float minA, float maxA)
{
    printTestStartLCD(testName);
    float current = measureCurrent();

    bool success = current >= minA && current <= maxA;

    int count = 0;

    while (!success && count < RETEST_COUNT_CURRENT)
    {
        delay(5);

        current = measureCurrent();

        success = current >= minA && current <= maxA;

        count++;
    }

    Test test = {
        testName,
        MeasureUnit::A,
        minA,
        maxA,
        current};

    if (askRetest(test))
    {
        return testCurrent(testName, minA, maxA);
    }

    return test;
}

Test testVoltage(const char *testName, MeasurePoint point, float minV, float maxV)
{
    Serial.print(F("TESTING: "));
    Serial.println(testName);

    printTestStartLCD(testName);
    float volts = measureAverageVoltage(point);
    bool success = volts >= minV && volts <= maxV;

    int count = 0;
    while (!success && count < RETEST_COUNT)
    {
        delay(5);

        volts = measureAverageVoltage(point);

        count++;
    }

    Test test = {
        testName,
        MeasureUnit::V,
        minV,
        maxV,
        volts};

    Serial.print(F("Test: "));
    Serial.println(test.name);
    Serial.print(test.min);
    Serial.print(F(" <= "));
    Serial.print(test.value);
    Serial.print(F(" <= "));
    Serial.print(test.max);
    Serial.println();

    if (askRetest(test))
    {
        return testVoltage(testName, point, minV, maxV);
    }

    return test;
}

void printReadyScreen()
{
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(F("Press the switch"));
    lcd.setCursor(0, 2);
    lcd.print(F("to start!"));
}
