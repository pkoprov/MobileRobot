import platform
import pygame
import serial
import serial.tools.list_ports
import time
import os

def find_esp32_port():
    system = platform.system()
    ports = serial.tools.list_ports.comports()

    print("Available ports:")
    if system == "Linux":
        if os.path.exists("/dev/ttyTHS0"):
            return "/dev/ttyTHS0"
        
    for p in ports:
        print(f"{p.device} — {p.description} — VID:{p.vid} PID:{p.pid}")

    for port in ports:
        if system == "Windows":
            if "usb serial" in port.description.lower() or "esp32" in port.description.lower():
                return port.device
        elif system == "Linux":
            if port.vid == 0x303A:
                return port.device


    return None

# Example usage
port = find_esp32_port()
if port:
    print(f"Found ESP32 on: {port}")
else:
    print("ESP32 not found.")


# Initialize joystick and serial
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Serial port (auto-detected above)
ser = serial.Serial(port, 115200, timeout=0.1)
time.sleep(2)  # wait for ESP32 reset

print("Ready. Use joystick to control robot.")

while True:
    pygame.event.pump() 


    x = joystick.get_axis(0)  # Left stick horizontal
    y = joystick.get_axis(1)  # Left stick vertical

    serial_cmd = f"{x} {y}\n"

    # Send command to ESP32
    ser.write(serial_cmd.encode())
    print(f"Sent: {serial_cmd.strip()}")

    time.sleep(0.05)
