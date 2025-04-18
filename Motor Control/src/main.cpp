#include <Arduino.h>
#define IN1 3  // D1
#define IN2 4  // D2
#define IN3 5  // D3
#define IN4 6  // D4
#define ENA 2  // D0
#define ENB 7  // D5

String inputBuffer = "";

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Direction: Motor A
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Direction: Motor B
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  // PWM
  analogWrite(ENA, abs(leftSpeed));
  analogWrite(ENB, abs(rightSpeed));
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

