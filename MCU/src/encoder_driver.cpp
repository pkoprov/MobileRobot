#include <Arduino.h>
#include "encoder_driver.h"

// Encoder tick counters
volatile long leftCount = 0;
volatile long rightCount = 0;

// Previous state for encoders (2 bits: A << 1 | B)
volatile uint8_t lastLeftState = 0;
volatile uint8_t lastRightState = 0;

// Quadrature lookup table: delta = enc_table[(old << 2) | new]
const int8_t enc_table[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// ISR for left encoder (both A and B pins trigger this)
void IRAM_ATTR handleLeftEncoder() {
  uint8_t A = digitalRead(LEFT_A_PIN);
  uint8_t B = digitalRead(LEFT_B_PIN);
  uint8_t newState = (A << 1) | B;
  uint8_t index = (lastLeftState << 2) | newState;
  leftCount += enc_table[index];
  lastLeftState = newState;
}

// ISR for right encoder
void IRAM_ATTR handleRightEncoder() {
  uint8_t A = digitalRead(RIGHT_A_PIN);
  uint8_t B = digitalRead(RIGHT_B_PIN);
  uint8_t newState = (A << 1) | B;
  uint8_t index = (lastRightState << 2) | newState;
  rightCount += enc_table[index];
  lastRightState = newState;
}

void initEncoders() {
  pinMode(LEFT_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_B_PIN, INPUT_PULLUP);

  // Initialize last states
  lastLeftState  = (digitalRead(LEFT_A_PIN) << 1) | digitalRead(LEFT_B_PIN);
  lastRightState = (digitalRead(RIGHT_A_PIN) << 1) | digitalRead(RIGHT_B_PIN);

  // Attach interrupts to both A and B channels
  attachInterrupt(digitalPinToInterrupt(LEFT_A_PIN), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_B_PIN), handleLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A_PIN), handleRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_B_PIN), handleRightEncoder, CHANGE);
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
