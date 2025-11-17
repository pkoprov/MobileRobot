// GPIO pins for encoders
#pragma once

enum MotorSide : int { LEFT = 0, RIGHT = 1 };

// Motor driver pins (GPIO numbers)
constexpr int PWMA = 21;
constexpr int AIN1 = 6;
constexpr int AIN2 = 7;
constexpr int PWMB = 20;
constexpr int BIN1 = 9;
constexpr int BIN2 = 8;
constexpr int STNBY = 10;

// Encoder pins (choose non-conflicting GPIOs)
constexpr int LEFT_A_PIN  = 3;
constexpr int LEFT_B_PIN  = 2;
constexpr int RIGHT_A_PIN = 4;
constexpr int RIGHT_B_PIN = 5;