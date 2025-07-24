#pragma once

#include <Adafruit_MotorShield.h>

// Motor indices
#define LEFT  0
#define RIGHT 1

// Forward declarations
void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int left, int right);
