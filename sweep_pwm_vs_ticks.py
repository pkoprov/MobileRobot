# sweep_pwm_vs_ticks.py
# Measures encoder response vs. raw PWM on your MCU (protocol: 'p L R', 'e')
# Produces pwm_sweep.csv and plots ticks_per_frame vs PWM for each side.
#
# Tips:
# - Keep wheels off the ground / on rollers.
# - Ensure firmware echoes "READY" at boot (not required, but helps).
# - Your MCU responds to:
#     p <L> <R>\r   # raw PWM -255..255
#     e\r           # prints "<left_count> <right_count>\n"
#
# Requires: pyserial, matplotlib, numpy, pandas (optional but handy)
#   pip install pyserial matplotlib numpy pandas

import time, csv, argparse, re
from collections import deque

import numpy as np
import matplotlib.pyplot as plt

try:
    import pandas as pd
except Exception:
    pd = None  # optional

import serial

# -------------------- CONFIG DEFAULTS --------------------
DEFAULT_PORT   = "COM6"   # change if needed (e.g., "/dev/ttyUSB0")
DEFAULT_BAUD   = 115200
PID_RATE_HZ    = 100.0    # must match your firmware
SAMPLE_RATE_HZ = 80.0     # how often we read 'e' during logging
SETTLE_S       = 0.8      # time to wait after a PWM step before logging
LOG_S          = 1.2      # time to log for each PWM step
STEP           = 10       # PWM resolution (10 → 10,20,...,250)
MAX_PWM        = 255
PAUSE_BETWEEN_STEPS_S = 0.15 # quiet time between steps (optional)
ENC_LINE = re.compile(r'^\s*-?\d+\s+-?\d+\s*$')  # e.g. "123 -456"

# -------------------- SERIAL HELPERS --------------------
def send_cmd(ser, s):
    ser.write((s + "\r").encode("ascii"))

def read_line(ser, timeout=0.5):
    ser.timeout = timeout
    line = ser.readline().decode(errors="ignore").strip()
    return line

def read_encoders(ser, tries=20, timeout=0.15):
    """
    Ask MCU for 'e' and return (L, R).
    Robust to extra prints like 'CMD p args: ...' or 'OK'.
    """
    # Drop any backlog (e.g., echoes from 'p' command)
    ser.reset_input_buffer()

    # Request once, then scan several lines for the numbers
    ser.write(b"e\r")
    last = ""
    for _ in range(tries):
        line = read_line(ser, timeout=timeout)
        if not line:
            # try re-requesting if we got silence
            ser.write(b"e\r")
            continue
        last = line
        if ENC_LINE.match(line):
            L, R = map(int, line.split())
            return (L, R)
        # else: ignore lines like "CMD p args: ..." or "OK"
    raise ValueError(f"Bad encoder response after tries; last line: {last!r}")

def set_pwm(ser, left, right):
    left  = int(max(-MAX_PWM, min(MAX_PWM, left)))
    right = int(max(-MAX_PWM, min(MAX_PWM, right)))
    ser.write(f"p {left} {right}\r".encode())
    # small delay so the device prints its echo before we ask for 'e'
    time.sleep(0.03)


def wait_ready(ser, boot_wait=0.6, flush=True):
    if flush:
        ser.reset_input_buffer()
    time.sleep(boot_wait)
    # Try to read a READY/any line without assuming it's there
    try:
        for _ in range(10):
            line = read_line(ser, timeout=0.2)
            if not line:
                break
    except Exception:
        pass

# -------------------- MEASUREMENT --------------------
def measure_ticks_per_frame(ser, side="L", pwm=100,
                            settle_s=SETTLE_S, log_s=LOG_S,
                            sample_rate_hz=SAMPLE_RATE_HZ):
    """
    Apply PWM to one side, wait settle, then sample encoders for log_s seconds.
    Returns average ticks/frame for the active side (signed).
    """
    if side not in ("L", "R"):
        raise ValueError("side must be 'L' or 'R'")

    # Apply PWM to the selected side
    if side == "L":
        set_pwm(ser, pwm, 0)
    else:
        set_pwm(ser, 0, pwm)

    # Settling
    time.sleep(settle_s)

    # Sample deltas
    dt = 1.0 / sample_rate_hz
    t0 = time.time()
    # read initial enc
    L0, R0 = read_encoders(ser)
    last_L, last_R = L0, R0
    deltas = []

    while time.time() - t0 < log_s:
        time.sleep(dt)
        L, R = read_encoders(ser)
        dL = L - last_L
        dR = R - last_R
        last_L, last_R = L, R
        deltas.append((dL, dR))

    # Average ticks per second for the active side
    if not deltas:
        return 0.0

    if side == "L":
        avg_ticks_per_sample = np.mean([d[0] for d in deltas])
    else:
        avg_ticks_per_sample = np.mean([d[1] for d in deltas])

    avg_ticks_per_sec = avg_ticks_per_sample * sample_rate_hz
    ticks_per_frame = avg_ticks_per_sec / PID_RATE_HZ
    return ticks_per_frame

def sweep_side(ser, side="L", step=STEP, pwms=None,
               settle_s=SETTLE_S, log_s=LOG_S, sample_rate_hz=SAMPLE_RATE_HZ):
    """
    Returns list of dict rows: {side,pwm,ticks_per_frame}
    Sweeps 0→+MAX then 0→-MAX (skipping 0 in each direction).
    """
    rows = []

    # ensure raw PWM mode is active by sending one p 0 0
    set_pwm(ser, 0, 0)
    time.sleep(0.2)

    if pwms is None:
        pos = list(range(step, MAX_PWM + 1, step))
        neg = [-p for p in pos]
    else:
        pos = sorted([p for p in pwms if p > 0])
        neg = sorted([-abs(p) for p in pwms if p < 0])

    # Forward direction sweep
    for p in pos:
        time.sleep(settle_s)
        ser.reset_input_buffer()
        tpf = measure_ticks_per_frame(ser, side=side, pwm=p,
                                      settle_s=settle_s, log_s=log_s,
                                      sample_rate_hz=sample_rate_hz)
        rows.append({"side": side, "pwm": p, "ticks_per_frame": tpf})
        time.sleep(PAUSE_BETWEEN_STEPS_S)

    # Stop briefly
    set_pwm(ser, 0, 0)
    time.sleep(0.3)

    # Reverse direction sweep
    for p in neg:
        time.sleep(settle_s)
        ser.reset_input_buffer()
        tpf = measure_ticks_per_frame(ser, side=side, pwm=p,
                                      settle_s=settle_s, log_s=log_s,
                                      sample_rate_hz=sample_rate_hz)
        rows.append({"side": side, "pwm": p, "ticks_per_frame": tpf})
        time.sleep(PAUSE_BETWEEN_STEPS_S)

    # stop
    set_pwm(ser, 0, 0)
    time.sleep(0.2)
    return rows

# -------------------- PLOTTING --------------------
def plot_results(rows):
    # Separate by side and direction
    L_pos = [(r["pwm"], r["ticks_per_frame"]) for r in rows if r["side"]=="L" and r["pwm"]>0]
    L_neg = [(r["pwm"], r["ticks_per_frame"]) for r in rows if r["side"]=="L" and r["pwm"]<0]
    R_pos = [(r["pwm"], r["ticks_per_frame"]) for r in rows if r["side"]=="R" and r["pwm"]>0]
    R_neg = [(r["pwm"], r["ticks_per_frame"]) for r in rows if r["side"]=="R" and r["pwm"]<0]

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)
    if L_pos: ax1.plot([p for p,_ in L_pos], [t for _,t in L_pos], marker='o', label="L forward")
    if L_neg: ax1.plot([p for p,_ in L_neg], [t for _,t in L_neg], marker='o', label="L reverse")
    ax1.set_title("Left: ticks/frame vs PWM")
    ax1.set_xlabel("PWM")
    ax1.set_ylabel("ticks per frame")
    ax1.legend()
    ax1.grid(True)

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    if R_pos: ax2.plot([p for p,_ in R_pos], [t for _,t in R_pos], marker='o', label="R forward")
    if R_neg: ax2.plot([p for p,_ in R_neg], [t for _,t in R_neg], marker='o', label="R reverse")
    ax2.set_title("Right: ticks/frame vs PWM")
    ax2.set_xlabel("PWM")
    ax2.set_ylabel("ticks per frame")
    ax2.legend()
    ax2.grid(True)

    plt.show()

# -------------------- MAIN --------------------
def main():
    ap = argparse.ArgumentParser(description="PWM→ticks calibration sweep")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--step", type=int, default=STEP)
    ap.add_argument("--settle", type=float, default=SETTLE_S)
    ap.add_argument("--log", type=float, default=LOG_S)
    ap.add_argument("--sample", type=float, default=SAMPLE_RATE_HZ)
    ap.add_argument("--side", choices=["L","R","both"], default="both")
    ap.add_argument("--csv", default="pwm_sweep.csv")
    args = ap.parse_args()

    rows = []
    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        wait_ready(ser)
        print("Connected. Starting sweep...")

        sides = ["L","R"] if args.side=="both" else [args.side]
        for s in sides:
            rows += sweep_side(
                ser, side=s, step=args.step,
                settle_s=args.settle, log_s=args.log,
                sample_rate_hz=args.sample
            )

        # Stop
        set_pwm(ser, 0, 0)

    # Save CSV
    fieldnames = ["side","pwm","ticks_per_frame"]
    with open(args.csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"Saved {args.csv} with {len(rows)} rows.")

    # Optional: print deadband guesses
    for s in ("L","R"):
        pos = [r for r in rows if r["side"]==s and r["pwm"]>0]
        neg = [r for r in rows if r["side"]==s and r["pwm"]<0]
        def first_over_abs(threshold, seq):
            # seq is a list of rows for a single side & sign
            cand = [r for r in seq if abs(r["ticks_per_frame"]) > threshold]
            if not cand:
                return None
            best = min(cand, key=lambda r: abs(r["pwm"]))  # minimal |PWM| that moves
            return best["pwm"], best["ticks_per_frame"]

        for s in ("L","R"):
            pos = [r for r in rows if r["side"]==s and r["pwm"]>0]
            neg = [r for r in rows if r["side"]==s and r["pwm"]<0]
            pos_db = first_over_abs(0.5, pos)
            neg_db = first_over_abs(0.5, neg)
            print(f"{s} deadband guess: forward {pos_db}, reverse {neg_db}")

    # Plot
    try:
        if pd is not None:
            df = pd.DataFrame(rows)
            print(df.groupby(["side"]).agg({"pwm":["min","max"], "ticks_per_frame":["min","max","mean"]}))
    except Exception:
        pass

    plot_results(rows)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted. Stopping motors...")
        try:
            with serial.Serial(DEFAULT_PORT, DEFAULT_BAUD, timeout=0.2) as ser:
                set_pwm(ser, 0, 0)
        except Exception:
            pass
