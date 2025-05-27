#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "motor_driver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

String inputBuffer = "";

void setupMotors() {
  delay(2000);
  bool motorFound = false;
  for (int i = 0; i < 5; i++) {
    if (AFMS.begin()) {
      motorFound = true;
      break;
    }
    Serial.println("Motor shield not found. Retrying...");
    delay(1000);
  }
  if (!motorFound) {
    Serial.println("Restarting ESP...");
    ESP.restart();
  }

  motorLeft->setSpeed(0);
  motorRight->setSpeed(0);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  Serial.println("Motor Shield initialized.");
}

void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0) motorLeft->run(FORWARD);
  else if (leftSpeed < 0) motorLeft->run(BACKWARD);
  else motorLeft->run(RELEASE);
  motorLeft->setSpeed(abs(leftSpeed));

  if (rightSpeed > 0) motorRight->run(FORWARD);
  else if (rightSpeed < 0) motorRight->run(BACKWARD);
  else motorRight->run(RELEASE);
  motorRight->setSpeed(abs(rightSpeed));
  motorRight->setSpeed(abs(rightSpeed));
}

void processSerialCommand() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (inputBuffer.length() > 0) {
        int spaceIndex = inputBuffer.indexOf(' ');
        if (spaceIndex > 0) {
          int x = inputBuffer.substring(0, spaceIndex).toInt();
          int y = inputBuffer.substring(spaceIndex + 1).toInt();
          driveMotors(constrain(y + x, -255, 255), constrain(y - x, -255, 255));
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += ch;
    }
  }
}
