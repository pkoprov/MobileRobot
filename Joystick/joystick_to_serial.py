import argparse
import os
import platform
import sys
import time

import pygame
import serial
import serial.tools.list_ports

DEFAULT_BAUD = 115200
POLL_INTERVAL = 0.01  # seconds


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def find_esp32_port(preferred_port=None):
    if preferred_port:
        return preferred_port

    system = platform.system()
    ports = serial.tools.list_ports.comports()"

    for p in ports:
        print(f"{p.device} - {p.description} - VID:{p.vid} PID:{p.pid}")

    for port in ports:
        if system == "Windows":
            if "usb serial" in port.description.lower() or "esp32" in port.description.lower():
                return port.device
        elif system == "Linux":
            if port.vid == 0x303A:
                return port.device

    return None


def format_packet(command_char, left, right):
    if command_char == "p":
        scale = 255
        scaled_left = int(clamp(left, -1.0, 1.0) * scale)
        scaled_right = int(clamp(right, -1.0, 1.0) * scale)
        return f"{command_char} {scaled_left} {scaled_right}\r"

    normalized_left = clamp(left, -1.0, 1.0)
    normalized_right = clamp(right, -1.0, 1.0)
    return f"{command_char} {normalized_left:.3f} {normalized_right:.3f}\r"


def main():
    parser = argparse.ArgumentParser(description="Send joystick commands to the MCU.")
    parser.add_argument(
        "-c",
        "--command",
        choices=["m", "p"],
        default="m",
        help="Command character to send ('m' for closed-loop speeds, 'p' for raw PWM).",
    )
    parser.add_argument("--port", help="Serial port override (e.g., COM3 or /dev/ttyACM0).")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate.")
    args = parser.parse_args()

    port = find_esp32_port(args.port)
    if not port:
        print("ESP32 not found.")
        return 1

    print(f"Found ESP32 on: {port}")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        return 1

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    ser = serial.Serial(port, args.baud, timeout=0.1)
    time.sleep(2)  # give ESP32 time to reboot after opening serial

    print(f"Ready. Sending command '{args.command}' to MCU on {port}.")

    try:
        while True:
            pygame.event.pump()
            # pygame reports left as -1 and right as +1; invert so pushing left turns left
            x = -joystick.get_axis(0)  # Left stick horizontal (left = +1)
            # pygame reports forward as -1, so invert to make forward positive
            y = -joystick.get_axis(1)  # Left stick vertical (forward = +1)

            left = clamp(y + x, -1.0, 1.0)
            right = clamp(y - x, -1.0, 1.0)

            packet = format_packet(args.command, left, right)
            ser.write(packet.encode("ascii"))
            print(f"Sent: {packet.strip()}")

            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()
        pygame.quit()

    return 0


if __name__ == "__main__":
    sys.exit(main())
