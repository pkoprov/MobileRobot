#pragma once
#include "gpio.h"

// Function declarations
void initEncoders();
long readEncoder(int side);
void resetEncoder(int side);
void resetEncoders();
