#!/usr/bin/env python3
import argparse
import re
import sys
import time

import serial

# Encoder specs
CPR = 1975  # Counts Per Revolution (adjust to your encoder's CPR)
TARGET_REVOLUTIONS = 10
TARGET_TICKS = CPR * TARGET_REVOLUTIONS


def send_cmd(ser, cmd):
    """
    Send a raw command with CR terminator (expected by your firmware).
    """
    data = (cmd + "\r").encode("ascii")
    ser.write(data)
    ser.flush()


def read_line(ser, timeout_s=0.2):
    """
    Read one line from serial within timeout_s seconds.
    Returns decoded string or None.
    """
    end = time.time() + timeout_s
    while time.time() < end:
        line_bytes = ser.readline()
        if line_bytes:
            try:
                return line_bytes.decode(errors="ignore")
            except Exception:
                return None
    return None

def read_ok(ser, timeout_s=0.2):
    end = time.time() + timeout_s
    while time.time() < end:
        line_bytes = ser.readline()
        if not line_bytes:
            continue
        line = line_bytes.decode(errors="ignore").strip()
        if line == "OK":
            return True
        # ignore anything else
    return False

def read_encoder_once(ser, side="left", timeout_s=0.2):
    end = time.time() + timeout_s
    while time.time() < end:
        line_bytes = ser.readline()
        if not line_bytes:
            continue
        line = line_bytes.decode(errors="ignore")
        line_stripped = line.strip()
        # Only accept "digits digits"
        parts = line_stripped.split()
        if side == "left":
            pos = 0
        else:
            pos = 1
        if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
            return int(parts[pos])
        # ignore 'OK' or anything else
    return None

def measure_time_for_10_revs(port, baud, pwm_left, pwm_right, encoder_side):
    print("Opening serial port {} @ {}...".format(port, baud))
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.05)
    except serial.SerialException as e:
        print("Failed to open serial port: {}".format(e), file=sys.stderr)
        sys.exit(1)

    with ser:
        # Flush any old data
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Reset encoders
        print("Resetting encoders with 'r'...")
        send_cmd(ser, "r")
        _ = read_line(ser)
        # set new PID values
        Kp = 60
        Kd = 0
        Ki = 4
        Ko = 10
        print("Setting PID values with 'u Kp Kd Ki Ko'...")
        send_cmd(ser, "u {} {} {} {}".format(Kp, Kd, Ki, Ko))
        _ = read_line(ser)
        print(_)
        send_cmd(ser, "u -1")
        _ = read_line(ser)
        print(_)


        input("Press Enter to start measurement (will send 'm {} {}' repeatedly)...".format(
            pwm_left, pwm_right
        ))

        drive_cmd = "m {} {}".format(pwm_left, pwm_right)
        print("Starting drive with:", drive_cmd)

        start_ticks = None
        start_time = None

        print("Polling encoders with 'e' until {} ticks ({} revs)...".format(
            TARGET_TICKS, TARGET_REVOLUTIONS
        ))
        print("CTRL+C to abort.")

        try:
            while True:
                # Keep motion alive: send drive command
                send_cmd(ser, drive_cmd)
                read_ok(ser, timeout_s=0.1)   # discard the 'OK'

                # Ask for encoder values
                send_cmd(ser, "e")
                ticks = read_encoder_once(ser, encoder_side, timeout_s=0.1)
                if ticks is None:
                    # no valid reading this cycle
                    time.sleep(0.03)
                    continue

                if start_ticks is None:
                    start_ticks = ticks
                    start_time = time.time()
                    print("Start ticks ({}): {}".format(encoder_side, start_ticks))
                    time.sleep(0.03)
                    continue

                delta_ticks = ticks - start_ticks
                if delta_ticks < 0:
                    print("Ticks went backwards ({}), re-baselining.".format(ticks))
                    start_ticks = ticks
                    start_time = time.time()
                    time.sleep(0.03)
                    continue

                sys.stdout.write(
                    "ticks_left={}  delta={}/{}\r".format(
                        ticks, delta_ticks, TARGET_TICKS
                    )
                )
                sys.stdout.flush()

                if delta_ticks >= TARGET_TICKS:
                    end_time = time.time()
                    elapsed = end_time - start_time
                    print(
                        "\nReached target: delta_ticks = {} in {:.3f} seconds."
                        .format(delta_ticks, elapsed)
                    )
                    est_cpr = float(delta_ticks) / float(TARGET_REVOLUTIONS)
                    print(
                        "Estimated CPR from this run: {:.2f} counts/rev"
                        .format(est_cpr)
                    )
                    break

                time.sleep(0.03)

        except KeyboardInterrupt:
            print("\nAborted by user.")

        # Stop wheels
        print("Sending stop command: p 0 0")
        send_cmd(ser, "p 0 0")


def main():
    parser = argparse.ArgumentParser(
        description="Measure time for {} wheel revolutions using encoders via 'p', 'e', 'r' commands.".format(
            TARGET_REVOLUTIONS
        )
    )
    parser.add_argument(
        "--port", default="/dev/ttyACM0",
        help="Serial port of MCU (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud", type=int, default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--pl", type=float, default=255,
        help="Left wheel PWM (0-255, default: 255)",
    )
    parser.add_argument(
        "--pr", type=float, default=255,
        help="Right wheel PWM (0-255, default: 255)",
    )

    parser.add_argument(
        "--side", default="left",
        help="Side for encoder reads (left, right, default: left)",
    )

    args = parser.parse_args()

    measure_time_for_10_revs(
        port=args.port,
        baud=args.baud,
        pwm_left=args.pl,
        pwm_right=args.pr,
        encoder_side=args.side
    )


if __name__ == "__main__":
    main()
