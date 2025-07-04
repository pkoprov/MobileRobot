#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "motor_driver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

String inputBuffer = "";
String lastInput = "";
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000;
bool motorsAreRunning = false;

void setupMotors() {
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
  delay(2000);
  while (Serial1.available()) Serial1.read();  // Flush garbage

  Serial1.println("ESP32 is up.");
  Serial.println("ESP32 is up.");

  bool motorFound = false;
  for (int i = 0; i < 5; i++) {
    if (AFMS.begin()) {
      motorFound = true;
      break;
    }
    Serial.println("Motor shield not found. Retrying...");
    Serial1.println("Motor shield not found. Retrying...");
    delay(1000);
  }
  if (!motorFound) {
    Serial.println("Restarting ESP...");
    Serial1.println("Restarting ESP...");
    ESP.restart();
  }

  Serial.println("Motor Shield is found. Starting...");
  Serial1.println("Motor Shield is found. Starting...");

  motorLeft->setSpeed(0);
  motorRight->setSpeed(0);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void driveMotors(float leftSpeed, float rightSpeed) {
  leftSpeed = constrain(leftSpeed, -1, 1);
  rightSpeed = constrain(rightSpeed, -1, 1);

  if (leftSpeed > 0) motorLeft->run(FORWARD);
  else if (leftSpeed < 0) motorLeft->run(BACKWARD);
  else motorLeft->run(RELEASE);
  motorLeft->setSpeed(static_cast<uint8_t>(abs(leftSpeed) * 255));

  if (rightSpeed > 0) motorRight->run(FORWARD);
  else if (rightSpeed < 0) motorRight->run(BACKWARD);
  else motorRight->run(RELEASE);
  motorRight->setSpeed(static_cast<uint8_t>(abs(rightSpeed) * 255));

  lastCommandTime = millis();
  motorsAreRunning = true;
}

void processSerialCommand() {
  while (Serial1.available()) {
    char ch = Serial1.read();
    if (isDigit(ch) || ch == ' ' || ch == '.' || ch == '-' || ch == '\n' || ch == '\r') {
      inputBuffer += ch;
    }

    if (ch == '\n' || ch == '\r') {
      int spaceIndex = inputBuffer.indexOf(' ');
      if (spaceIndex > 0 && spaceIndex < inputBuffer.length() - 1) {
        float x = inputBuffer.substring(0, spaceIndex).toFloat();
        float y = inputBuffer.substring(spaceIndex + 1).toFloat();

        x = constrain(x, -1.0, 1.0);
        y = constrain(y, -1.0, 1.0);
        float left = constrain(y + x, -1.0, 1.0);
        float right = constrain(y - x, -1.0, 1.0);

        driveMotors(left, right);

        if (lastInput != inputBuffer) {
          Serial.println("Command: " + inputBuffer);
        }
        lastInput = inputBuffer;

      } else {
        Serial1.println("Invalid input: " + inputBuffer);
        Serial.println("Invalid input: " + inputBuffer);
      }
      inputBuffer = "";
    }
  }

  if (motorsAreRunning && (millis() - lastCommandTime > commandTimeout)) {
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    motorLeft->setSpeed(0);
    motorRight->setSpeed(0);
    motorsAreRunning = false;

    Serial1.println("Timeout: Motors stopped.");
    Serial.println("Timeout: Motors stopped.");
  }
}
