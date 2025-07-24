#include <Arduino.h>
#include "encoder_driver.h"

volatile long leftCount = 0;
volatile long rightCount = 0;

void IRAM_ATTR handleLeftA() {
  bool A = digitalRead(LEFT_A_PIN);
  bool B = digitalRead(LEFT_B_PIN);
  if (A == B)
    leftCount++;
  else
    leftCount--;
}

void IRAM_ATTR handleRightA() {
  bool A = digitalRead(RIGHT_A_PIN);
  bool B = digitalRead(RIGHT_B_PIN);
  if (A == B)
    rightCount++;
  else
    rightCount--;
}

void initEncoders() {
  pinMode(LEFT_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_A_PIN), handleLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A_PIN), handleRightA, CHANGE);
}

long readEncoder(int i) {
  if (i == LEFT) {
    return leftCount;
  } else if (i == RIGHT) {
    return rightCount;
  }
  return 0;
}

void resetEncoder(int i) {
  if (i == LEFT) {
    leftCount = 0;
  } else if (i == RIGHT) {
    rightCount = 0;
  }
}

void resetEncoders() {
  leftCount = 0;
  rightCount = 0;
}
