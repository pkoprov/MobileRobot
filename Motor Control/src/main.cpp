#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Get motors: 1 = left, 2 = right (you can flip them if needed)
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);

unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // 1 second in milliseconds
bool motorsAreRunning = false;
bool firstCommandReceived = false;
String inputBuffer = "";

void setup() {
  Serial1.begin(115200, SERIAL_8N1, D7, D6); // RX, TX pins for ESP32
  Serial.begin(115200); // for debugging through USB
  delay(2000);  // <-- Give FeatherWing time to power up

  // Flush UART buffer to avoid startup noise being parsed
  while (Serial1.available()) Serial1.read();

  Serial1.println("ESP32 is up.");
  Serial.println("ESP32 is up.");
  
  bool motorFound = false;
  
  for (int i = 0; i < 5; i++) {
    if (AFMS.begin()) {
      motorFound = true;
      break;
    }
    Serial1.println("Motor shield not found. Retrying...");
    Serial.println("Motor shield not found. Retrying...");
    
    delay(1000);
  }
  if (!motorFound) {
    Serial1.println("Restarting ESP...");
    Serial.println("Restarting ESP...");
    ESP.restart();
  }

  Serial1.println("Motor Shield is found. Starting...");
  Serial.println("Motor Shield is found. Starting...");

  // Set the motor speed to 0
  motorLeft->setSpeed(0);
  motorRight->setSpeed(0);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void driveMotors(float  leftSpeed, float  rightSpeed) {
  leftSpeed = constrain(leftSpeed, -1, 1);
  rightSpeed = constrain(rightSpeed, -1, 1);

  // LEFT motor
  if (leftSpeed > 0) {
    motorLeft->run(FORWARD);
  } else if (leftSpeed < 0) {
    motorLeft->run(BACKWARD);
  } else {
    motorLeft->run(RELEASE);
  }
  motorLeft->setSpeed(static_cast<uint8_t>(abs(leftSpeed) * 255));

  // RIGHT motor
  if (rightSpeed > 0) {
    motorRight->run(FORWARD);
  } else if (rightSpeed < 0) {
    motorRight->run(BACKWARD);
  } else {
    motorRight->run(RELEASE);
  }
  motorRight->setSpeed(static_cast<uint8_t>(abs(rightSpeed) * 255));
  
  // Record activity
  lastCommandTime = millis();
  motorsAreRunning = true;
}

void loop() {
  while (Serial1.available()) {
    char ch = Serial1.read();
    if (ch == '\n' || ch == '\r') {
      int spaceIndex = inputBuffer.indexOf(' ');
      if (spaceIndex > 0 && spaceIndex < inputBuffer.length() - 1) {
        // Input format: "x y", where x = turn (-1.0 to 1.0), y = throttle (-1.0 to 1.0)
        float x = inputBuffer.substring(0, spaceIndex).toFloat();  // turn
        float y = inputBuffer.substring(spaceIndex + 1).toFloat(); // forward/back
        x = constrain(x, -1.0, 1.0);
        y = constrain(y, -1.0, 1.0);

        // Differential drive math
        float left = constrain(y + x, -1.0, 1.0);
        float right = constrain(y - x, -1.0, 1.0);

        driveMotors(left, right);
      } else {
        Serial1.println("Invalid input: " + inputBuffer);
        Serial.println("Invalid input: " + inputBuffer);
      }
      inputBuffer = "";  // Clear buffer after newline
    } else {
      inputBuffer += ch;
    }
  }

  // Stop motors if no command has been received in 1 second
  if (motorsAreRunning && (millis() - lastCommandTime > commandTimeout)) {
    motorLeft->setSpeed(0);
    motorRight->setSpeed(0);
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    motorsAreRunning = false;

    Serial1.println("Timeout: Motors stopped.");
    Serial.println("Timeout: Motors stopped.");
  }
}

