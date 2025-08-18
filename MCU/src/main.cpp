#include <Arduino.h>
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

static inline void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = arg2 = 0;
  arg = 0;
  argIdx = 0;         // <— was `index = 0`
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

    case MOTOR_SPEEDS:
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      } else {
        moving = 1;
      }
      leftPID.TargetTicksPerFrame  = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;

    case MOTOR_RAW_PWM:
      resetPID();
      moving = 0;
      setMotorSpeeds((int)arg1, (int)arg2);
      Serial.println("OK");
      break;

    case UPDATE_PID: {
      while ((str = strtok_r(p, ":", &p)) != nullptr && i < 4) {
        pid_args[i++] = atoi(str);
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
      continue;
    }
    else {
      if (arg == 0)        cmd = chr;                 // single-letter command
      else if (arg == 1)  { argv1[argIdx++] = chr; }  // collect arg1
      else if (arg == 2)  { argv2[argIdx++] = chr; }  // collect arg2
    }
  }

  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
}
