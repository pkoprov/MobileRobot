#pragma once
//
// Command character definitions
// These are the only ones supported in your current main.cpp
//

#define GET_BAUDRATE   'b'   // report baud rate
#define PING           'z'   // ping test -> returns "1"

#define READ_ENCODERS  'e'   // return encoder counts
#define RESET_ENCODERS 'r'   // reset encoders & PID

#define MOTOR_SPEEDS   'm'   // closed-loop motor control (ticks/frame target)
#define MOTOR_RAW_PWM  'p'   // open-loop raw PWM values

#define UPDATE_PID     'u'   // update PID constants ("u Kp Kd Ki Ko")
