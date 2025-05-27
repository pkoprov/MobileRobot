#include <Arduino.h>

const int batteryPin = 2; // GPIO4 = D2 on XIAO ESP32C3

// Voltage divider setup
const float R1 = 43000.0;
const float R2 = 10000.0;

// Calibrated ADC reference voltage
const float ADC_VREF = 2.95;

// Battery voltage range
const float EMPTY_VOLTAGE = 10.5;
const float FULL_VOLTAGE = 13.8;
const float LOW_BATTERY_THRESHOLD = 11.0;

// Moving average filter config
const int windowSize = 10;
int buffer[windowSize];
int bufIndex = 0;
long total = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // 12-bit ADC = 0–4095

  // Fill buffer with initial reading
  int initial = analogRead(batteryPin);
  for (int i = 0; i < windowSize; i++) {
    buffer[i] = initial;
    total += initial;
  }
}

void loop() {
  total -= buffer[bufIndex];
  buffer[bufIndex] = analogRead(batteryPin);
  total += buffer[bufIndex];
  bufIndex = (bufIndex + 1) % windowSize;

  float rawAvg = total / (float)windowSize;

  // Calculate ADC voltage and battery voltage
  float v_out = rawAvg / 4095.0 * ADC_VREF;
  float batteryVoltage = v_out * (R1 + R2) / R2;
  batteryVoltage *= 0.96;  // Calibrated

  // Calculate charge %
  float percent = (batteryVoltage - EMPTY_VOLTAGE) / (FULL_VOLTAGE - EMPTY_VOLTAGE) * 100.0;
  percent = constrain(percent, 0.0, 100.0);

  // Print results
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2);
  Serial.print(" V | Charge: ");
  Serial.print(percent, 1);
  Serial.print(" % | v_out: ");
  Serial.print(v_out, 2);
  Serial.print(" V | raw_avg: ");
  Serial.println(rawAvg, 0);

  // Low battery warning
  if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
    Serial.println("⚠️ LOW BATTERY! Please recharge or shut down system.");
  }

  delay(1000);
}
