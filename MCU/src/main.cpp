#include <Arduino.h>
#include "motor_driver.h"
#include "battery_level.h"
#include <ESP32Encoder.h>
ESP32Encoder encoder;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Give time for Serial to initialize

  // Initialize encoder on GPIO9 (A) and GPIO8 (B)
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachFullQuad(9, 8);  // A = GPIO9, B = GPIO8
  encoder.clearCount();

  Serial.println("Rotate the wheel 10 times in one direction.");
  Serial.println("Encoder tick count will be printed every 100 ms.");
}

void loop() {
  static int64_t lastCount = 0;
  int64_t count = encoder.getCount();

  if (count != lastCount) {
    Serial.printf("Ticks: %lld\n", count);
    lastCount = count;
  }

  delay(100);
}