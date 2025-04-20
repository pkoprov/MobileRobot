#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Get motors: 1 = left, 2 = right (you can flip them if needed)
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  delay(2000);  // <-- Give FeatherWing time to power up
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

  Serial.println("Motor Shield is found. Starting...");

  Serial.println("Motor Shield initialized.");
  motorLeft->setSpeed(0);
  motorRight->setSpeed(0);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT motor
  if (leftSpeed > 0) {
    motorLeft->run(FORWARD);
  } else if (leftSpeed < 0) {
    motorLeft->run(BACKWARD);
  } else {
    motorLeft->run(RELEASE);
  }
  motorLeft->setSpeed(abs(leftSpeed));

  // RIGHT motor
  if (rightSpeed > 0) {
    motorRight->run(FORWARD);
  } else if (rightSpeed < 0) {
    motorRight->run(BACKWARD);
  } else {
    motorRight->run(RELEASE);
  }
  motorRight->setSpeed(abs(rightSpeed));
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (inputBuffer.length() > 0) {
        int spaceIndex = inputBuffer.indexOf(' ');
        if (spaceIndex > 0) {
          int x = inputBuffer.substring(0, spaceIndex).toInt();     // turn
          int y = inputBuffer.substring(spaceIndex + 1).toInt();    // forward/back

          // Differential drive math
          int left = constrain(y + x, -255, 255);
          int right = constrain(y - x, -255, 255);

          driveMotors(left, right);
        }
      }
      inputBuffer = "";
    } else {
      inputBuffer += ch;
    }
  }
}
