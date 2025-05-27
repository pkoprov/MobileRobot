#include <Arduino.h>
#include "battery_level.h"

const int batteryPin = 2;
const float R1 = 43000.0;
const float R2 = 10000.0;
const float ADC_VREF = 2.95;
const float EMPTY_VOLTAGE = 10.5;
const float FULL_VOLTAGE = 13.8;
const float LOW_BATTERY_THRESHOLD = 11.0;
const float calibrationFactor = 0.96;

const int windowSize = 10;
int buffer[windowSize];
int bufIndex = 0;
long total = 0;
unsigned long lastBatteryCheck = 0;
const unsigned long batteryCheckInterval = 1000;

void setupBatteryMonitor() {
  analogReadResolution(12);
  int initial = analogRead(batteryPin);
  for (int i = 0; i < windowSize; i++) {
    buffer[i] = initial;
    total += initial;
  }
}

void updateBatteryStatus() {
  unsigned long now = millis();
  if (now - lastBatteryCheck < batteryCheckInterval) return;
  lastBatteryCheck = now;

  total -= buffer[bufIndex];
  buffer[bufIndex] = analogRead(batteryPin);
  total += buffer[bufIndex];
  bufIndex = (bufIndex + 1) % windowSize;

  float rawAvg = total / (float)windowSize;
  float v_out = rawAvg / 4095.0 * ADC_VREF;
  float batteryVoltage = v_out * (R1 + R2) / R2 * calibrationFactor;

  float percent = (batteryVoltage - EMPTY_VOLTAGE) / (FULL_VOLTAGE - EMPTY_VOLTAGE) * 100.0;
  percent = constrain(percent, 0.0, 100.0);

  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.print(" V | Charge: ");
  Serial.print(percent, 1);
  Serial.print(" % | v_out: ");
  Serial.print(v_out, 2);
  Serial.print(" V | raw_avg: ");
  Serial.println(rawAvg, 0);

  if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
    Serial.println("⚠️ LOW BATTERY! Please recharge or shut down system.");
  }
}
