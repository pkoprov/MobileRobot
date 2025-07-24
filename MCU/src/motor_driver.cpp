#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "motor_driver.h"

// Create motor shield and motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x60);
Adafruit_DCMotor* leftMotor = nullptr;
Adafruit_DCMotor* rightMotor = nullptr;

void initMotorController() {
  AFMS.begin();
  leftMotor = AFMS.getMotor(1);   // or 2, depending on wiring
  rightMotor = AFMS.getMotor(2);  // or 1, depending on wiring
}

void setMotorSpeed(int i, int spd) {
  spd = constrain(spd, -255, 255);

  Adafruit_DCMotor* motor = (i == LEFT) ? leftMotor : rightMotor;

  if (spd == 0) {
    motor->run(RELEASE);
    motor->setSpeed(0);
  } else {
    motor->setSpeed(abs(spd));
    motor->run(spd > 0 ? FORWARD : BACKWARD);
  }
}

void setMotorSpeeds(int left, int right) {
  setMotorSpeed(LEFT, left);
  setMotorSpeed(RIGHT, right);
}
