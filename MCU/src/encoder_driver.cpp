#include <Arduino.h>
#include "gpio.h"
#include "encoder_driver.h"

// ---- Portable fast read for all ESP32 variants (incl. C3/S2/S3) ----
#if defined(ARDUINO_ARCH_ESP32)
  #include "driver/gpio.h"
  static inline int READ_PIN(int pin) {
    return gpio_get_level((gpio_num_t)pin);   // ISR-safe
  }
#else
  #define READ_PIN(p) digitalRead(p)
#endif

// Encoder tick counters (shared with ISRs)
volatile long leftCount  = 0;
volatile long rightCount = 0;

// -------- ISRs: only A pins generate interrupts --------
// ---- both channels, edge-based 4× decoding ----
void IRAM_ATTR leftA_ISR() {
  int a = READ_PIN(LEFT_A_PIN);
  int b = READ_PIN(LEFT_B_PIN);
  // On A edge: if A == B -> +1 else -1   (flip sign if needed)
  leftCount += (a == b) ? -1 : +1;
}

void IRAM_ATTR leftB_ISR() {
  int a = READ_PIN(LEFT_A_PIN);
  int b = READ_PIN(LEFT_B_PIN);
  // On B edge: if A != B -> +1 else -1   (complementary rule)
  leftCount += (a != b) ? -1 : +1;
}

void IRAM_ATTR rightA_ISR() {
  int a = READ_PIN(RIGHT_A_PIN);
  int b = READ_PIN(RIGHT_B_PIN);
  rightCount += (a == b) ? -1 : +1;
}

void IRAM_ATTR rightB_ISR() {
  int a = READ_PIN(RIGHT_A_PIN);
  int b = READ_PIN(RIGHT_B_PIN);
  rightCount += (a != b) ? -1 : +1;
}

// -------------------- Setup --------------------
void initEncoders() {
  pinMode(LEFT_A_PIN,  INPUT_PULLUP);
  pinMode(LEFT_B_PIN,  INPUT_PULLUP);
  pinMode(RIGHT_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_B_PIN, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(LEFT_A_PIN),  leftA_ISR,  CHANGE);
attachInterrupt(digitalPinToInterrupt(LEFT_B_PIN),  leftB_ISR,  CHANGE);
attachInterrupt(digitalPinToInterrupt(RIGHT_A_PIN), rightA_ISR, CHANGE);
attachInterrupt(digitalPinToInterrupt(RIGHT_B_PIN), rightB_ISR, CHANGE);
}

// -------------------- API --------------------
long readEncoder(int side) {
#if defined(ARDUINO_ARCH_ESP32)
  static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  long v = (side == LEFT) ? leftCount : rightCount;
  portEXIT_CRITICAL(&mux);
  return v;
#else
  noInterrupts();
  long v = (side == LEFT) ? leftCount : rightCount;
  interrupts();
  return v;
#endif
}

void resetEncoder(int side) {
#if defined(ARDUINO_ARCH_ESP32)
  static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  if (side == LEFT)  leftCount = 0;
  else               rightCount = 0;
  portEXIT_CRITICAL(&mux);
#else
  noInterrupts();
  if (side == LEFT)  leftCount = 0;
  else               rightCount = 0;
  interrupts();
#endif
}

void resetEncoders() {
#if defined(ARDUINO_ARCH_ESP32)
  static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL(&mux);
  leftCount = rightCount = 0;
  portEXIT_CRITICAL(&mux);
#else
  noInterrupts();
  leftCount = rightCount = 0;
  interrupts();
#endif
}
