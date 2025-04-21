import platform
import pygame
import serial
import serial.tools.list_ports
import time

def find_esp32_port():
    system = platform.system()
    ports = serial.tools.list_ports.comports()

    for port in ports:
        port_name = port.device
        description = port.description.lower()

        if system == "Windows":
            if "usb serial" in description or "esp32" in description:
                return port_name
        elif system == "Linux":
            # Jetson/Xavier typically uses /dev/ttyUSBx or /dev/ttyACMx
            if port.vid == 0x303A:
                return port_name

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

# Serial port — change to match your ESP32 COM port
ser = serial.Serial(port, 115200, timeout=0.1)
time.sleep(2)  # wait for ESP32 reset

print("Ready. Use joystick to control robot.")

last_cmd = ''

while True:
    pygame.event.pump() 


    x = joystick.get_axis(0)  # Left stick horizontal
    y = joystick.get_axis(1)  # Left stick vertical

    Xspeed = int(min(x, 1.0) * 255)
    Yspeed = int(min(y, 1.0) * 255)

    cmd = f"{Xspeed} {Yspeed}"

    serial_cmd = f"{cmd}\n"

    ser.write(serial_cmd.encode())
    print(f"Sent: {serial_cmd.strip()}")

    time.sleep(0.05)
