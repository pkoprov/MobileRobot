#include <Arduino.h>
#include "motor_driver.h"
#include "encoder_driver.h"

#define FAST_SPEED   140  // Slightly slower fast speed
#define SLOW_SPEED    20  // Reduced further
#define SLOW_ZONE   1000   // Enter slow zone earlier

const int TICKS_PER_REV = 1974;
const int TARGET_TICKS = TICKS_PER_REV;

bool motorStopped = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔁 Spinning left motor for 1 revolution...");

  initMotorController();
  initEncoders();
  resetEncoders();

  setMotorSpeed(RIGHT, FAST_SPEED);
  setMotorSpeed(LEFT, 0);
}

void loop() {
  
  long Ticks = abs(readEncoder(RIGHT));

  if (!motorStopped) {
    if (Ticks < SLOW_ZONE) {
      setMotorSpeed(RIGHT, FAST_SPEED);
    } else if (Ticks < TARGET_TICKS) {
      setMotorSpeed(RIGHT, SLOW_SPEED);
    } else {
      setMotorSpeed(RIGHT, 0);
      motorStopped = true;
      Serial.println("🛑 Target reached. Motor stopped.");
    }
  }

  // Always print ticks so you can manually back it up
  Serial.printf("📈 L: %6ld | Target: %d\n", Ticks, TARGET_TICKS);
  delay(200);
}
