#pragma once
#include <Arduino.h>


// PID loop rate
#define PID_RATE 100                      // Hz
extern const int PID_INTERVAL;           // ms (computed)

// PID gains (integer style, Ko = scaling)
extern long Kp, Kd, Ki, Ko;

// movement flag (0 when raw PWM is active or both targets zero)
extern volatile int moving;

// Per-side controller state
typedef struct {
  long TargetTicksPerFrame;     // command input (ticks per PID interval)
  long Encoder;                 // last encoder reading
  long PrevEnc;                 // previous encoder reading
  long PrevErr;                 // previous error
  long Iterm;                   // integral accumulator
  int  Output;                  // last PWM output (-255..255)
} pid_control_t;

extern pid_control_t leftPID;
extern pid_control_t rightPID;

// API
void resetPID();
void updatePID();
