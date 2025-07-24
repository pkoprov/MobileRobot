#pragma once

// Motor side definitions
#define LEFT  0
#define RIGHT 1

// GPIO pins for encoders
#define LEFT_A_PIN   4  // D2
#define LEFT_B_PIN   5  // D3
#define RIGHT_A_PIN  9  // D8
#define RIGHT_B_PIN  8  // D9

// Function declarations
void initEncoders();
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
