#include "gpio.h"
#include "diff_controller.h"
#include "encoder_driver.h"
#include "motor_driver.h"

// PID interval in ms
const int PID_INTERVAL = 1000 / PID_RATE;

// Tunables (start conservative)
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;        // output = (Kp*e + Kd*de + Ki*sum)/Ko

const int  MAX_PWM   = 255;

volatile int moving = 0;

pid_control_t leftPID  = {0, 0, 0, 0, 0, 0};
pid_control_t rightPID = {0, 0, 0, 0, 0, 0};

// telemetry throttle
static int print_div = 3;
static int loop_ctr  = 0;

static inline int clampPWM(long v) {
  if (v >  MAX_PWM) return  MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return (int)v;
}

void resetPID() {
  leftPID.Encoder  = readEncoder(LEFT);
  leftPID.PrevEnc  = leftPID.Encoder;
  leftPID.PrevErr  = 0;
  leftPID.Iterm    = 0;
  leftPID.Output   = 0;
  leftPID.TargetTicksPerFrame = 0;

  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.PrevErr = 0;
  rightPID.Iterm   = 0;
  rightPID.Output  = 0;
  rightPID.TargetTicksPerFrame = 0;
}

static void doSide(pid_control_t& c, int side) {
  // Read encoder since last loop
  c.Encoder = readEncoder(side);
  long delta = c.Encoder - c.PrevEnc;     // ticks in last interval

  // Error: target - actual
  long err   = c.TargetTicksPerFrame - delta;
  long deriv = err - c.PrevErr;
  c.Iterm   += err;

  long out = ( (long)Kp * err + (long)Kd * deriv + (long)Ki * c.Iterm ) / Ko;
  c.Output  = clampPWM(out);

  // Apply to motor
  setMotorSpeed(side, c.Output);

  // Store for next time
  c.PrevEnc = c.Encoder;
  c.PrevErr = err;
}

void updatePID() {
  // If both targets zero, brake motors and hold still
  if (leftPID.TargetTicksPerFrame == 0 && rightPID.TargetTicksPerFrame == 0) {
    setMotorSpeeds(0, 0);
    moving = 0;
    // keep state in case we resume
    leftPID.PrevEnc  = readEncoder(LEFT);
    rightPID.PrevEnc = readEncoder(RIGHT);
    leftPID.PrevErr = rightPID.PrevErr = 0;
    return;
  }

  doSide(leftPID,  LEFT);
  doSide(rightPID, RIGHT);

  // --- Optional telemetry (LEFT only, every 3rd loop) for your Python parser
  // Uncomment if you need it; matches "Target= ... delta= ... Sent=" pattern

  if ((loop_ctr++ % print_div) == 0) {
    Serial.print("L: ");
    Serial.print(" Target="); Serial.print(leftPID.TargetTicksPerFrame);
    Serial.print(" Err="); Serial.print(leftPID.PrevErr);
    Serial.print(" Sent="); Serial.println(leftPID.Output);
  }
}