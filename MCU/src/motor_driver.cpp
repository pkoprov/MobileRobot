#include <Arduino.h>
#include "gpio.h"
#include "motor_driver.h"


void initMotorController() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STNBY, OUTPUT);
  digitalWrite(STNBY, HIGH);  // take motor driver out of standby
}

void setMotorSpeed(int i, int spd) {
  int pwm, in1, in2;
  if (i == LEFT) {
    pwm = PWMA; in1 = AIN1; in2 = AIN2;
  } else {
    pwm = PWMB; in1 = BIN1; in2 = BIN2;
  }

  int speed = constrain(abs(spd), 0, 255);

  if (spd > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else if (spd < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, speed);
  } else {
    // Brake
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, 0);
  }
}

void setMotorSpeeds(int left, int right) {
  setMotorSpeed(LEFT, left);
  setMotorSpeed(RIGHT, right);
}
