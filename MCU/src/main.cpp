#include <Arduino.h>
#include "gpio.h"
#include "commands.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"

// ---------- Serial parsing state ----------
int arg = 0;
int argIdx = 0;        // <— was `index`
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1 = 0;
long arg2 = 0;
bool raw_pwm = 0;   // 1 if in raw PWM mode
const unsigned long MOTION_CMD_TIMEOUT_MS = 100;
unsigned long lastMotionCmdMs = 0;
bool motionStoppedForTimeout = false;

static inline void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = arg2 = 0;
  arg = 0;
  argIdx = 0;         // <— was `index = 0`
}

static inline void markMotionCommand() {
  lastMotionCmdMs = millis();
  motionStoppedForTimeout = false;
}

static inline void stopMotorsForTimeout() {
  raw_pwm = 0;
  leftPID.TargetTicksPerFrame  = 0;
  rightPID.TargetTicksPerFrame = 0;
  resetPID();
  setMotorSpeeds(0, 0);
  motionStoppedForTimeout = true;
}

// ---------- Run a parsed command ----------
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];

  arg1 = atol(argv1);
  arg2 = atol(argv2);

  switch (cmd) {
    case GET_BAUDRATE:  Serial.println(115200); break;
    case PING:          Serial.println(1);      break;

    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(' ');
      Serial.println(readEncoder(RIGHT));
      break;

    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;

    case MOTOR_SPEEDS: {
      raw_pwm = 0;  // exit raw PWM mode if active
      // Expect two floats (or integers) in args
      // Parse as floating to support either normalized [-1..1] or raw ticks
      double s1 = strtod(argv1, nullptr);
      double s2 = strtod(argv2, nullptr);

      // DEBUGGING: Echo what we parsed
      // Serial.print("CMD m args: ");
      // Serial.print(s1, 3);
      // Serial.print(',');
      // Serial.println(s2, 3);

      resetPID();
      // Stop if both ~0
      if (fabs(s1) < 1e-6 && fabs(s2) < 1e-6) {
        setMotorSpeeds(0, 0);
      } else if (fabs(s1) <= 1.0 && fabs(s2) <= 1.0) {
        // Normalized mode: scale to ticks/frame
        long t1 = lround(s1 * MAX_TICKS_PER_FRAME);
        long t2 = lround(s2 * MAX_TICKS_PER_FRAME);
        leftPID.TargetTicksPerFrame  = t1;
        rightPID.TargetTicksPerFrame = t2;
      } else {
        // Raw mode: treat inputs as ticks/frame (round to nearest long)
        long t1 = lround(s1);
        long t2 = lround(s2);
        leftPID.TargetTicksPerFrame  = t1;
        rightPID.TargetTicksPerFrame = t2;
      }
      markMotionCommand();
      Serial.println("OK");
      break;
    }

    case MOTOR_RAW_PWM:
      // DEBUGGING: Echo what we parsed
      // Serial.print("CMD p args: ");
      // Serial.print(arg1);
      // Serial.print(',');
      // Serial.println(arg2);

      resetPID();
      raw_pwm = 1;
      setMotorSpeeds((int)arg1, (int)arg2);
      markMotionCommand();
      Serial.println("OK");
      break;

    case UPDATE_PID: {
    // combine argv1 and argv2 (argv2 holds the rest of the line)
      char buf[64];
      if (argv2[0] != '\0') {
        snprintf(buf, sizeof(buf), "%s %s", argv1, argv2);
      } else {
        snprintf(buf, sizeof(buf), "%s", argv1);
      }

      int pid_args[4] = {0,0,0,0};
      int i = 0;
      char *saveptr = nullptr;
      char *tok = strtok_r(buf, " \t", &saveptr); // split on spaces/tabs
      while (tok != nullptr && i < 4) {
        pid_args[i++] = atoi(tok);
        tok = strtok_r(nullptr, " \t", &saveptr);
      }

      if (i == 4) {
        Kp = pid_args[0];
        Kd = pid_args[1];
        Ki = pid_args[2];
        Ko = pid_args[3];
        Serial.println("OK");
      } else {
        Serial.println("ERR");
      }
      break;
    }

    default:
      Serial.println("Invalid Command");
      break;
  }
  return 0;
}

// ---------- Setup ----------
unsigned long nextPID = PID_INTERVAL;

void setup() {
  Serial.begin(115200);
  delay(500);

  initMotorController();
  initEncoders();
  resetEncoders();
  resetPID();

  lastMotionCmdMs = millis();
  Serial.println("READY");
}

// ---------- Main loop ----------
void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();

    if (chr == 13) {                // CR ends command
      if (arg == 1) argv1[argIdx] = 0;
      else if (arg == 2) argv2[argIdx] = 0;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {          // space splits args
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[argIdx] = 0;
        arg = 2;
        argIdx = 0;
      }
      else if (arg == 2) {
        // Preserve spaces inside argv2 so tokenization later works.
        if (argIdx < (int)sizeof(argv2) - 1) {
          argv2[argIdx++] = ' ';
        }
      }
      continue;
    }
    else {
      if (arg == 0)        cmd = chr;                 // single-letter command
      else if (arg == 1)  { argv1[argIdx++] = chr; }  // collect arg1
      else if (arg == 2)  { argv2[argIdx++] = chr; }  // collect arg2
    }
  }

  unsigned long now = millis();
  if (!motionStoppedForTimeout && (now - lastMotionCmdMs) > MOTION_CMD_TIMEOUT_MS) {
    stopMotorsForTimeout();
  }

  if (now > nextPID && !raw_pwm) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
}
