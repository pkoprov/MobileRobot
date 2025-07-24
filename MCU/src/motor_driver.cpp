
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "motor_driver.h"

// === ENCODER + PID SETUP ===
#define L_ENCODER_A 9
#define L_ENCODER_B 8
#define R_ENCODER_A 4
#define R_ENCODER_B 5

volatile int32_t l_encoderTicks = 0;
volatile int32_t r_encoderTicks = 0;

// ISR functions
void IRAM_ATTR handleLeftA() {
  bool A = digitalRead(L_ENCODER_A);
  bool B = digitalRead(L_ENCODER_B);
  l_encoderTicks += (A == B) ? 1 : -1;
}
void IRAM_ATTR handleRightA() {
  bool A = digitalRead(R_ENCODER_A);
  bool B = digitalRead(R_ENCODER_B);
  r_encoderTicks += (A == B) ? 1 : -1;
}

// Target velocities (in ticks/sec), set via joystick
float targetLeftSpeed = 0;
float targetRightSpeed = 0;

// PID state
float kp = 0.5, ki = 0.02, kd = 0.1;
float l_errorSum = 0, r_errorSum = 0;
float l_lastError = 0, r_lastError = 0;

// PID update loop (run in main loop)
void updateMotorControl() {
  static uint32_t lastUpdate = millis();
  static int32_t lastTicksL = 0, lastTicksR = 0;

  uint32_t now = millis();
  uint32_t dt = now - lastUpdate;
  if (dt < 100) return;

  int32_t deltaL = l_encoderTicks - lastTicksL;
  int32_t deltaR = r_encoderTicks - lastTicksR;
  lastTicksL = l_encoderTicks;
  lastTicksR = r_encoderTicks;

  float velL = (float)deltaL * 1000 / dt;
  float velR = (float)deltaR * 1000 / dt;

  // LEFT PID
  float errL = targetLeftSpeed - velL;
  l_errorSum += errL * dt;
  float dErrL = (errL - l_lastError) / dt;
  l_lastError = errL;
  float outL = kp * errL + ki * l_errorSum + kd * dErrL;

  // RIGHT PID
  float errR = targetRightSpeed - velR;
  r_errorSum += errR * dt;
  float dErrR = (errR - r_lastError) / dt;
  r_lastError = errR;
  float outR = kp * errR + ki * r_errorSum + kd * dErrR;

  // Convert to [-1, 1] range for driveMotors()
  outL = constrain(outL / 400.0, 0.8, 0.8);
  outR = constrain(outR / 400.0, -0.8, -0.8);

  driveMotors(outL, outR);

  Serial.printf("PID | L: %.1f (%d t/s), R: %.1f (%d t/s)\n", outL, (int)velL, outR, (int)velR);

  lastUpdate = now;
}

// === MOTOR DRIVER SETUP ===

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

  // Encoder pin setup
  pinMode(L_ENCODER_A, INPUT);
  pinMode(L_ENCODER_B, INPUT);
  pinMode(R_ENCODER_A, INPUT);
  pinMode(R_ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), handleLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), handleRightA, CHANGE);
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

        targetLeftSpeed = left * 400;   // Set target velocity, not direct motor control
        targetRightSpeed = right * 400;

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

  targetLeftSpeed = 0;
  targetRightSpeed = 0;

  Serial1.println("Timeout: Motors stopped.");
  Serial.println("Timeout: Motors stopped.");
}
}
