#include "gpio.h"
#include "diff_controller.h"
#include "encoder_driver.h"
#include "motor_driver.h"

// PID interval in ms
const int PID_INTERVAL = 1000 / PID_RATE;

// Tunables (start conservative)
long Kp = 1000;
long Kd = 120;
long Ki = 80;
long Ko = 50;        // output = (Kp*e + Kd*de + Ki*sum)/Ko

const int  MAX_PWM   = 255;

volatile int moving = 0;

pid_control_t leftPID  = {0, 0, 0, 0, 0, 0};
pid_control_t rightPID = {0, 0, 0, 0, 0, 0};

// telemetry throttle
static int print_div = 6;  // print every Nth loop
static int loop_ctr  = 0;

static inline int clampPWM(long v) {
  if (v >  MAX_PWM) return  MAX_PWM;
  if (v < -MAX_PWM) return -MAX_PWM;
  return (int)v;
}

void resetPID() {
  leftPID.Encoder  = readEncoder(LEFT);
  leftPID.PrevEnc  = leftPID.Encoder;
  leftPID.PrevDelta = 0;
  leftPID.Dterm = 0;
  leftPID.PrevErr  = 0;
  leftPID.Iterm    = 0;
  leftPID.Output   = 0;
  leftPID.TargetTicksPerFrame = 0;

  rightPID.Encoder = readEncoder(RIGHT);
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.PrevDelta = 0;
  rightPID.Dterm = 0;
  rightPID.PrevErr = 0;
  rightPID.Iterm   = 0;
  rightPID.Output  = 0;
  rightPID.TargetTicksPerFrame = 0;
}

static void doSide(pid_control_t& c, int side) {
  // Read encoder since last loop
  c.Encoder = readEncoder(side);
  long delta = c.Encoder - c.PrevEnc;     // ticks in last interval
  const long maxStep = 2 * c.TargetTicksPerFrame + 5; // tune
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;

  // Error: target - actual
  long err   = c.TargetTicksPerFrame - delta;
  
  //Derivative on MEASUREMENT with light LPF (alpha = 1/8)
  // d_raw = -(delta - prevDelta)  -> damps changes in measured speed
  long d_raw = c.PrevDelta - delta;
  c.Dterm = c.Dterm + ((d_raw - c.Dterm) >> 3);  // >>2 = /8

  // Integral anti-windup
  // Integrate only if we were NOT saturated on the previous cycle
  if (abs(c.Output) < MAX_PWM) {
    c.Iterm += err;
    // clamp integral to a safe bound (tune if needed)
    const long ITERM_MAX = 5000;
    if (c.Iterm >  ITERM_MAX) c.Iterm =  ITERM_MAX;
    if (c.Iterm < -ITERM_MAX) c.Iterm = -ITERM_MAX;
  }

  long pid = (Kp * err + Kd * c.Dterm + Ki * c.Iterm ) / Ko;
  long out = pid;

  c.Output  = clampPWM(out);

  // Apply to motor
  setMotorSpeed(side, c.Output);

  // Store for next time
  c.PrevEnc = c.Encoder;
  c.PrevErr = err;
  c.PrevDelta = delta;
}

void updatePID() {
  // Stop condition: zero both targets
  if (leftPID.TargetTicksPerFrame == 0 && rightPID.TargetTicksPerFrame == 0) {
    setMotorSpeeds(0, 0);
    // reset integrators so next run starts clean
    leftPID.Iterm = rightPID.Iterm = 0;
    leftPID.PrevErr = rightPID.PrevErr = 0;
    return;
  }

  doSide(leftPID,  LEFT);
  doSide(rightPID, RIGHT);

  // --- Optional telemetry (LEFT only, every 3rd loop) for your Python parser
  // Uncomment if you need it; matches "Target= ... delta= ... Sent=" pattern
  // if ((loop_ctr++ % print_div) == 0) {
    // Serial.print("E: ");
    // Serial.print(" Target="); Serial.print(rightPID.TargetTicksPerFrame);
    // Serial.print(" Err="); Serial.print(rightPID.PrevErr);
    // Serial.print(" Sent="); Serial.println(rightPID.Output);
  // }
}