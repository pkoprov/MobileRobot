#include <Arduino.h>
#include "motor_driver.h"
#include "battery_level.h"

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupBatteryMonitor();
}

void loop() {
  processSerialCommand();
  updateBatteryStatus();
}
