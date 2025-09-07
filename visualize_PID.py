# step_verify.py
import serial, time, numpy as np
import matplotlib.pyplot as plt

# ---------------- USER CONFIG ----------------
PORT = "COM6"        # change if needed
BAUD = 115200
RUN_TIME = 5.0       # seconds to log for each step
SETTLING_BAND = 0.10 # +/-10% settling band (change to 0.05 for tighter)
TARGETS = [25, 28, 32, 40]  # step targets to test

# Chosen gains (from your sweep)
KP, KD, KI, KO = 1000, 120, 80, 50
# ---------------------------------------------

def send(ser, s): ser.write(s.encode())
def rl(ser):
    try: return ser.readline().decode(errors="ignore").strip()
    except: return ""

# def set_pid(ser, kp, kd, ki, ko):
    # Firmware expects: u <Kp> <Kd> <Ki> <Ko>
    # send(ser, f"u {kp} {kd} {ki} {ko}\r"); time.sleep(0.12)

def run_step(ser, target, run_time=RUN_TIME):
    # reset and start
    send(ser, "r\r"); time.sleep(0.5); ser.reset_input_buffer()
    send(ser, f"m {target} {target}\r")
    t0 = time.time()
    T, Target, Err, Sent = [], [], [], []
    while time.time() - t0 < run_time:
        line = rl(ser)
        if "Target=" in line and "Err=" in line:
            try:
                parts = line.replace(",", " ").split()
                tgt  = int([p.split("=")[1] for p in parts if p.startswith("Target=")][0])
                err  = int([p.split("=")[1] for p in parts if p.startswith("Err=")][0])
                sent = None
                tok = [p for p in parts if p.startswith("Sent=")]
                if tok:
                    sent = int(tok[0].split("=")[1])
                T.append(time.time() - t0)
                Target.append(tgt); Err.append(err); Sent.append(sent)
            except:
                pass
    send(ser, "m 0 0\r"); time.sleep(0.12)

    T = np.array(T)
    Target = np.array(Target, float)
    Err = np.array(Err, float)
    Sent = np.array(Sent, float) if any(s is not None for s in Sent) else None
    Delta = Target - Err if len(Target) == len(Err) and len(Target) > 0 else np.array([])
    return T, Target, Delta, Sent

def metrics(T, Target, Delta, Sent, settling_band=SETTLING_BAND):
    m = {"target": None, "rise": None, "settling": None, "overshoot": None,
         "smooth_std": None, "iae": None, "sat": None}
    if len(Delta) == 0 or len(Target) == 0:
        return m
    tgt = int(np.median(Target))
    m["target"] = tgt

    # rise time to 90% of target
    try:
        idx = np.where(Delta >= 0.9 * tgt)[0]
        m["rise"] = float(T[idx[0]]) if len(idx) else None
    except:
        m["rise"] = None

    # overshoot
    m["overshoot"] = max(0.0, (float(np.max(Delta)) - tgt) / max(1, tgt) * 100.0)

    # settling time: first time after which all samples stay within band
    band = settling_band * tgt
    m["settling"] = None
    for i in range(len(Delta)):
        if np.all(np.abs(Delta[i:] - tgt) <= band):
            m["settling"] = float(T[i]); break

    # steady-state smoothness: std dev of last 30% of points, or after settling
    if m["settling"] is not None:
        start = np.searchsorted(T, m["settling"])
    else:
        start = int(0.7 * len(Delta))
    ss = Delta[start:] if start < len(Delta) else Delta[-1:]
    m["smooth_std"] = float(np.std(ss)) if len(ss) > 1 else None

    # IAE: integral of |error|
    err = tgt - Delta
    m["iae"] = float(np.trapz(np.abs(err), T)) if len(T) == len(err) and len(T) > 1 else None

    # saturation ratio
    if Sent is not None and len(Sent) == len(Delta):
        m["sat"] = float(np.mean(np.abs(Sent) >= 255.0))
    return m

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(1.0)

    # # set chosen gains
    # set_pid(ser, KP, KD, KI, KO)
    # print(f"Locked gains: Kp={KP}, Ki={KI}, Kd={KD}, Ko={KO}\n")

    results = {}
    for tgt in TARGETS:
        T, TT, DD, S = run_step(ser, tgt)
        results[tgt] = (T, TT, DD, S, metrics(T, TT, DD, S))

    ser.close()

    # print metrics table
    print("Target | Rise(s) | Settle(s) | Overshoot% | SmoothStd | IAE    | SatRatio")
    print("-------+---------+-----------+------------+-----------+--------+---------")
    for tgt in TARGETS:
        T, TT, DD, S, m = results[tgt]
        rise = "-" if m["rise"] is None else f"{m['rise']:.3f}"
        st   = "-" if m["settling"] is None else f"{m['settling']:.3f}"
        os   = "-" if m["overshoot"] is None else f"{m['overshoot']:.1f}"
        sm   = "-" if m["smooth_std"] is None else f"{m['smooth_std']:.2f}"
        iae  = "-" if m["iae"] is None else f"{m['iae']:.2f}"
        sat  = "-" if m["sat"] is None else f"{m['sat']:.2f}"
        print(f"{tgt:>6} | {rise:>7} | {st:>9} | {os:>10} | {sm:>9} | {iae:>6} | {sat:>7}")

    # plot
    plt.figure(figsize=(7,4.8))
    for tgt in TARGETS:
        T, TT, DD, S, m = results[tgt]
        if len(DD) == 0: 
            continue
        plt.plot(T, DD, label=f"Target={tgt}")
        # draw target and settling band
        plt.plot(T, TT, "k--", alpha=0.25)
        band = SETTLING_BAND * tgt
        plt.hlines([tgt + band, tgt - band], xmin=0, xmax=max(T) if len(T) else RUN_TIME,
                   linestyles="dashed", colors="gray", alpha=0.2)
        # annotate settling time if available
        if m["settling"] is not None:
            plt.axvline(m["settling"], color="gray", alpha=0.2)

    plt.title(f"Step responses (Kp={KP}, Ki={KI}, Kd={KD}, Ko={KO})")
    plt.xlabel("Time (s)")
    plt.ylabel("Ticks per PID frame (left)")
    plt.grid(True); plt.legend(); plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()