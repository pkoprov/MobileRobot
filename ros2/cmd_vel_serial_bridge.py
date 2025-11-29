#!/usr/bin/env python3
import os
import platform
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import serial
import serial.tools.list_ports


DEFAULT_BAUD = 115200
DEFAULT_COMMAND = "m"
DEFAULT_MAX_LINEAR = 1.0   # m/s used to normalize linear.x to [-1, 1]
DEFAULT_MAX_ANGULAR = 1.0  # rad/s used to normalize angular.z to [-1, 1]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def find_esp32_port(preferred_port: Optional[str] = None) -> Optional[str]:
    if preferred_port:
        return preferred_port

    system = platform.system()
    ports = serial.tools.list_ports.comports()

    for port in ports:
        if system == "Windows":
            if "usb serial" in port.description.lower() or "esp32" in port.description.lower():
                return port.device
        elif system == "Linux":
            if port.vid == 0x303A:
                return port.device

    return None


def format_packet(command_char: str, left: float, right: float) -> str:
    if command_char == "p":
        scale = 255
        scaled_left = int(clamp(left, -1.0, 1.0) * scale)
        scaled_right = int(clamp(right, -1.0, 1.0) * scale)
        return f"{command_char} {scaled_left} {scaled_right}\r"

    normalized_left = clamp(left, -1.0, 1.0)
    normalized_right = clamp(right, -1.0, 1.0)
    return f"{command_char} {normalized_left:.3f} {normalized_right:.3f}\r"


class CmdVelSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_serial_bridge")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("command_char", DEFAULT_COMMAND)
        self.declare_parameter("max_linear", DEFAULT_MAX_LINEAR)
        self.declare_parameter("max_angular", DEFAULT_MAX_ANGULAR)

        command_char = self.get_parameter("command_char").get_parameter_value().string_value
        command_char = command_char or DEFAULT_COMMAND
        if command_char not in ("m", "p"):
            raise ValueError("command_char must be 'm' or 'p'")
        self.command_char = command_char

        port_param = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.max_linear = float(self.get_parameter("max_linear").get_parameter_value().double_value)
        self.max_angular = float(self.get_parameter("max_angular").get_parameter_value().double_value)

        port = find_esp32_port(port_param)
        if not port:
            self.get_logger().fatal("ESP32 serial port not found. Set the 'port' parameter explicitly.")
            sys.exit(1)

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as exc:
            self.get_logger().fatal(f"Failed to open serial port {port}: {exc}")
            sys.exit(1)

        self.get_logger().info(
            f"Connected to {port} at {baud} baud; sending '{self.command_char}' commands "
            f"(max_linear={self.max_linear}, max_angular={self.max_angular})"
        )

        self.subscription = self.create_subscription(Twist, "cmd_vel", self.twist_callback, 10)

    def twist_callback(self, msg: Twist) -> None:
        linear_norm = 0.0
        angular_norm = 0.0

        if self.max_linear > 0:
            linear_norm = clamp(msg.linear.x / self.max_linear, -1.0, 1.0)
        if self.max_angular > 0:
            angular_norm = clamp(msg.angular.z / self.max_angular, -1.0, 1.0)

        left = clamp(linear_norm + angular_norm, -1.0, 1.0)
        right = clamp(linear_norm - angular_norm, -1.0, 1.0)

        packet = format_packet(self.command_char, left, right)
        try:
            self.ser.write(packet.encode("ascii"))
            self.get_logger().debug(f"Sent: {packet.strip()}")
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")

    def destroy_node(self) -> bool:
        # Stop motors on shutdown
        try:
            stop_packet = format_packet(self.command_char, 0.0, 0.0)
            self.ser.write(stop_packet.encode("ascii"))
        except Exception:
            pass

        try:
            self.ser.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
