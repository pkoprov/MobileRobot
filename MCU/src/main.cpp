#include <Arduino.h>
#include "motor_driver.h"
#include "encoder_driver.h"

#define FAST_SPEED   255  // Slightly slower fast speed
#define SLOW_SPEED    20  // Reduced further
#define SLOW_ZONE   TARGET_TICKS-1000   // Enter slow zone earlier

const int TICKS_PER_REV = 1974;
const int TARGET_TICKS = TICKS_PER_REV*10;

bool motorStoppedR = false;
bool motorStoppedL = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔁 Spinning motors for 1 revolution...");

  initMotorController();
  initEncoders();
  resetEncoders();

  setMotorSpeed(RIGHT, FAST_SPEED);
}

void loop() {
  
  long TicksL = abs(readEncoder(LEFT));
  long TicksR = abs(readEncoder(RIGHT));

  if (!motorStoppedR) {
    if (TicksR < SLOW_ZONE) {
      setMotorSpeed(RIGHT, FAST_SPEED);
    } else if (TicksR < TARGET_TICKS) {
      setMotorSpeed(RIGHT, SLOW_SPEED);
    } else {
      setMotorSpeed(RIGHT, 0);
      motorStoppedR = true;
      Serial.println("🛑 Right motor target reached. Right motor stopped.");
    }
  }
  if (!motorStoppedL) {
    if (TicksL < SLOW_ZONE) {
      setMotorSpeed(LEFT, FAST_SPEED);
    } else if (TicksL < TARGET_TICKS) {
      setMotorSpeed(LEFT, SLOW_SPEED);
    } else {
      setMotorSpeed(LEFT, 0);
      motorStoppedL = true;
      Serial.println("🛑 Left motor target reached. Left motor stopped.");
    } 
  }

  // Always print ticks so you can manually back it up
  Serial.printf("📈 R: %6ld | L: %6ld | Target: %d\n", TicksR, TicksL, TARGET_TICKS);
  delay(200);
}
