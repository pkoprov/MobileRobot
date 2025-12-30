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
from rcl_interfaces.msg import SetParametersResult


DEFAULT_BAUD = 115200
DEFAULT_COMMAND = "m"
DEFAULT_MAX_LINEAR = 0.453   # m/s used to normalize linear.x to [-1, 1]
DEFAULT_MAX_ANGULAR = 4.19  # rad/s used to normalize angular.z to [-1, 1]


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


class CmdVelSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_serial_bridge")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("command_char", DEFAULT_COMMAND)
        self.declare_parameter("max_linear", DEFAULT_MAX_LINEAR)
        self.declare_parameter("max_angular", DEFAULT_MAX_ANGULAR)
        self.declare_parameter("Kp", 60)
        self.declare_parameter("Kd", 0)
        self.declare_parameter("Ki", 4)
        self.declare_parameter("Ko", 10)

        command_char = self.get_parameter("command_char").get_parameter_value().string_value
        if command_char not in ("m", "p"):
            raise ValueError("command_char must be 'm' or 'p'")
        self.command_char = command_char

        port_param = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.max_linear = self.get_parameter("max_linear").get_parameter_value().double_value
        self.max_angular = self.get_parameter("max_angular").get_parameter_value().double_value

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
        self.add_on_set_parameters_callback(self.PID_callback)

    def PID_callback(self, params):
        updates = {p.name: p.value for p in params}

        Kp = updates.get("Kp", self.get_parameter("Kp").value)
        Kd = updates.get("Kd", self.get_parameter("Kd").value)
        Ki = updates.get("Ki", self.get_parameter("Ki").value)
        Ko = updates.get("Ko", self.get_parameter("Ko").value)

        pid_packet = f"u {Kp} {Kd} {Ki} {Ko}\r"
        try:
            self.ser.write(pid_packet.encode("ascii"))
            self.get_logger().info(f"Sent PID parameters: Kp={Kp}, Kd={Kd}, Ki={Ki}, Ko={Ko}")
            return SetParametersResult(successful=True)
        except serial.SerialException as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            return SetParametersResult(
                    successful=False,
                    reason=f"Serial write failed: {exc}",
                )
    
    def twist_callback(self, msg: Twist) -> None:
        linear_norm = 0.0
        angular_norm = 0.0

        if self.max_linear > 0:
            linear_norm = clamp(msg.linear.x / self.max_linear, -1.0, 1.0)
        if self.max_angular > 0:
            angular_norm = clamp(msg.angular.z / self.max_angular, -1.0, 1.0)

        left = clamp(linear_norm + angular_norm, -1.0, 1.0)
        right = clamp(linear_norm - angular_norm, -1.0, 1.0)

        packet = format_packet(self, left, right)
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
        
def format_packet(node: CmdVelSerialBridge, left: float, right: float) -> str:
    if node.command_char == "p":
        scale = 255
        left = int(left * scale)
        right = int(right * scale)

    node.get_logger().info(f"sending {node.command_char} command: left={left} right={right}")
    return f"{node.command_char} {left:.3f} {right:.3f}\r"

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
