<p align="center">
  <img src="assets/Diagram.png" alt="ESP32 Robot Controller Architecture" width="600"/>
</p>

<h1 align="center">ESP32 Robot Controller with Joystick 🎮</h1>
<p align="center">Host Joystick ➔ Serial ➔ ESP32 ➔ TB6612FNG ➔ Motors</p>

<p align="center">
  <a href="https://platformio.org/">
    <img src="https://img.shields.io/badge/Built%20With-PlatformIO-orange.svg" alt="PlatformIO Badge">
  </a>
  <a href="https://docs.ros.org/en/humble/">
    <img src="https://img.shields.io/badge/Ready%20For-ROS2-blueviolet.svg" alt="ROS2 Badge">
  </a>
  <a href="https://www.python.org/">
    <img src="https://img.shields.io/badge/Python-3.8+-blue.svg" alt="Python Badge">
  </a>
  <a href="https://opensource.org/licenses/MIT">
    <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License">
  </a>
</p>


# 🕹️ ESP32 Robot Controller with Joystick over Serial

This project connects a USB joystick (e.g., EasySMX) to an ESP32-based robot controller using serial communication. The ESP32 receives speed and direction commands and drives motors via a TB6612FNG motor driver. The system is designed to run a joystick interface on a host like Jetson Xavier and control motors with smooth differential drive.

---

## Project Structure

```
MobileRobot/
|- README.md
|- requirements.txt
|- assets/                        # Diagrams and media
|  |- Animation.gif
|  |- Diagram.png
|  └──  Diagram.pptx
|- Battery Control/               # Battery display helper (host-side)
|  └──  battery_display.py          # Shows voltage/health info
|- Joystick/                      # Host joystick -> serial bridge
|  └──  joystick_to_serial.py       # Sends joystick data to the ESP32
|- MCU/                           # PlatformIO project for ESP32 firmware
|  |- platformio.ini              # ESP32 build config (Seeed XIAO ESP32-C3)
|  |- src/                        # Firmware sources
|  |  |- main.cpp                 # Main loop
|  |  |- motor_driver.cpp         # Motor logic and serial input parsing
|  |  |- motor_driver.h
|  |  |- diff_controller.cpp      # Differential drive helper
|  |  |- diff_controller.h
|  |  |- encoder_driver.cpp       # Encoder reading + tick math
|  |  |- encoder_driver.h
|  |  |- commands.h               # Serial command definitions
|  |  └──  ff_params.h              # Feedforward tuning parameters
|  |- include/README              # PlatformIO include placeholder
|  └──  lib/README                  # PlatformIO library placeholder
|- fit_ff_from_csv.py             # Helper: fit feedforward params from CSV
|- sweep_pwm_vs_ticks.py          # Characterize PWM vs encoder ticks
└──  visualize_PID.py               # Plot PID responses from logs
```

---

## 🤖 ROS 2 Integration Plans

ROS 2 control is already implemented. See `ros2/cmd_vel_serial_bridge.py`, and the separate repo at https://github.com/pkoprov/pkis_scout for additional ROS 2 integration.

### ✅ Current Architecture [`Joystick/joystick_to_serial.py`]
- Joystick axis data is sent directly to the ESP32 via serial over USB. 
- The ESP32 applies differential drive logic and controls the motors.
- Communication uses a simple `"X Y\n"` ... from -1.0 to 1.0 (normalized float).

### 🚀 Current ROS 2 Integration

| Component               | Role                                                    |
|-------------------------|-------------------------------------                    |
| `joy_node`              | Reads USB joystick via SDL or evdev (install seprately) |
| `teleop_twist_joy`      | Converts joystick axes to `cmd_vel` (install seprately) |
| `cmd_vel_serial_bridge` | Sends `cmd_vel` as `"X Y"` strings over serial          |
| `esp32_motor_driver`    | Runs on ESP32 to parse and execute commands             |

Next upgrade plans include:
- Publish joint states to `joint_state_publisher`
- Create a URDF for the robot
- Create a launch file to bring up the robot

---

## 🚦 System Overview

- **Host system:** Jetson Xavier (Linux) or PC (Windows)
- **Microcontroller:** [Seeed XIAO ESP32-C3](https://a.co/d/fxnU5IB)
- **Motor driver:** [TB6612FNG](https://a.co/d/c7OfLXC)
- **Control interface:** [USB joystick (e.g., EasySMX)](https://a.co/d/foCvRpO)
- **LiDAR:** [RPLIDAR C1](https://a.co/d/6Qoewwf)
- **Communication:** Serial over USB (`/dev/ttyACM0` or `COMx`)
- **Frameworks:** Arduino (ESP32), Python with `pygame` and `pyserial`

---

## ⚙️ Motor & Encoder Details

This project uses [JGA25-370 12V 130RPM gear motors with encoders](https://www.amazon.com/dp/B07X7M1LLQ?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_3).

- 🔧 Motor type: DC gearmotor with metal gearbox
- 🧭 Encoder: dual-channel hall-effect (quadrature)
- 🌀 No-load speed: ~130 RPM @ 12V
- 📏 Measured encoder counts per output shaft revolution (CPR): **1975**

This CPR value is used in the motor control code to calculate speed, rotations, and distance accurately.


## 🔧 Requirements

### Microcontroller (ESP32) side:
- [PlatformIO](https://platformio.org/)
- ESP32 board support (set to `seeed_xiao_esp32c3`)


---

### Host (Jetson/PC) side:
- Python 3.8 or newer
- `pygame`
- `pyserial`

Install with:
```bash
pip install pygame pyserial
```

---

## 🎮 Usage

### 1. Run on ESP32

Build and upload `main.cpp` using PlatformIO. It will:
- Wait for serial input of joystick axis data (formatted as `"X Y\n"`, with normalized values from `-1.0` to `1.0`)
- Apply differential drive logic to control two DC motors

### 2. Run on Jetson or PC

Use `joystick_to_serial.py` to:
- Auto-detect ESP32 serial port
- Read left joystick axis input (forward = up on the stick, left = left on the stick)
- Mix axes into differential drive (`left = y + x`, `right = y - x`, both clamped to [-1..1])
- Send MCU commands with a selectable command character:
  - `-c m` (default) sends normalized speeds: `m <left> <right>\r` (floats)
  - `-c p` sends raw PWM: `p <leftPWM> <rightPWM>\r` (ints scaled to -255..255)

Examples:
```bash
python Joystick/joystick_to_serial.py -c m        # closed-loop motor speeds
python Joystick/joystick_to_serial.py -c p        # raw PWM
python Joystick/joystick_to_serial.py --port COM3 # override port
```

### 3. ROS 2 `cmd_vel` bridge

Bridge any ROS 2 velocity publisher (e.g., `teleop_twist_keyboard`, `teleop_twist_joy`) to the MCU:

```bash
source /opt/ros/humble/setup.bash  # or your ROS 2 distro
python ros2/cmd_vel_serial_bridge.py --ros-args \
  -p port:=/dev/ttyACM0 \
  -p command_char:=m \
  -p max_linear:=1.0 \
  -p max_angular:=1.0
```

Behavior:
- Subscribes to `cmd_vel` (`geometry_msgs/Twist`)
- Normalizes `linear.x` by `max_linear` and `angular.z` by `max_angular`
- Mixes differential drive: `left = y + x`, `right = y - x`, each clamped to [-1..1]
- Sends MCU commands as `m <left> <right>\r` (floats) or `p <leftPWM> <rightPWM>\r` (ints scaled to -255..255)



> Note: On Jetson or Linux, you may need to install `joystick` and set:
```bash
sudo apt install joystick
export SDL_JOYSTICK_DEVICE=/dev/input/js0
```

---

## 🧠 Differential Drive Logic

```text
left_speed  = y + x
right_speed = y - x
```

- `x` controls turning
- `y` controls forward/backward
- Values are constrained to `[-1.0, 1.0]` and mapped to PWM duty (0–255).

---


## 🛠 Troubleshooting

- If joystick isn't detected: make sure it's initialized (`pygame.joystick.init()`)
- If testing in WSL, first thing you need to bind your PC's port to WSL. Open PowerShell with admin rights and run the following lines:
  ```powershell
  usbipd list # find the BUSID of esp32. In my case it was "USB Serial Device (COM6), USB JTAG/serial debug unit" 
  ```
  Then you need to attach it
  ```powershell
  usbipd attach --wsl --busid <BUSID>
  ```
  Optional: persistent attach when reconnecting the USB device:
  ```powershell
  usbipd bind --busid <BUSID>
  ```
  Verify inside WSL:
  ```bash
  lsusb
  dmesg | tail
  ```
- If serial port permission denied on Linux:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
---

## 📸 Demo

<p align="center">
  <img src="assets/Animation.gif" alt="ESP32 Robot Controller Diagram" width="600"/>
</p>

---

## 📜 License

MIT License. Credit to Articulated Robotics for inspiration on joystick-to-serial architecture.
