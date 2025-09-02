// GPIO pins for encoders
#pragma once

enum MotorSide : int { LEFT = 0, RIGHT = 1 };

// Motor driver pins (GPIO numbers)
constexpr int PWMA = 7;
constexpr int AIN1 = 5;
constexpr int AIN2 = 6;
constexpr int PWMB = 8;
constexpr int BIN1 = 10;
constexpr int BIN2 = 9;

// Encoder pins (choose non-conflicting GPIOs)
constexpr int LEFT_A_PIN  = 20;
constexpr int LEFT_B_PIN  = 21;
constexpr int RIGHT_A_PIN = 3;
constexpr int RIGHT_B_PIN = 4;