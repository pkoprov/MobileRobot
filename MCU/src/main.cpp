#include <Arduino.h>
#include "encoder_driver.h"

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("🔁 Initializing encoders...");

  initEncoders();
  resetEncoders();

  Serial.println("✅ Encoders ready. Rotate motors to see counts.\n");
}

void loop() {
  unsigned long now = millis();

  // Print every 250 ms
  if (now - lastPrint > 250) {
    long leftTicks = readEncoder(LEFT);
    long rightTicks = readEncoder(RIGHT);

    Serial.printf("📈 L: %6ld | R: %6ld\n", leftTicks, rightTicks);
    lastPrint = now;
  }
}