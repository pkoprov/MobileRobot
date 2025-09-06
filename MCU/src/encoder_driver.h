#pragma once
#include "gpio.h"

// Function declarations
void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
