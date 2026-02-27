#!/usr/bin/env python3
"""
Plot and summarize telemetry from process_telemetry.csv

CSV columns:
  t_sec,jitter_sec,position,velocity,current,power

Printed metrics:
  Jitter:
    - max(|jitter|)
    - mean(|jitter|)
  Motoring power (power > 0 only):
    - max(power)
    - mean(power) averaged over motoring samples
    - motoring duty (% of samples with power > 0)

Plots:
  - jitter (us) vs time
  - position vs time
  - velocity vs time
  - acceleration vs time (computed as d(velocity)/dt)
  - power vs time
"""

from __future__ import annotations

import argparse
import numpy as np
import matplotlib.pyplot as plt


def load_csv(path: str) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float, invalid_raise=False)
    if data.size == 0:
        raise ValueError(f"No rows found in {path}")

    names = list(data.dtype.names or [])
    required = ["t_sec", "jitter_sec", "position", "velocity", "power"]
    missing = [c for c in required if c not in names]
    if missing:
        raise ValueError(f"Missing required columns in CSV: {missing}. Found: {names}")

    return {k: data[k].astype(float) for k in names}


def compute_accel(t: np.ndarray, vel: np.ndarray) -> np.ndarray:
    # Ensure increasing time for gradient
    order = np.argsort(t)
    t_sorted = t[order]
    v_sorted = vel[order]

    a_sorted = np.gradient(v_sorted, t_sorted)

    a = np.empty_like(a_sorted)
    a[order] = a_sorted
    return a


def fmt_jitter_sec(x: float) -> str:
    return f"{x:.9e} s  ({x * 1e6:.3f} us)"


def summarize(jitter: np.ndarray, power: np.ndarray) -> None:
    jitter = jitter[np.isfinite(jitter)]
    power = power[np.isfinite(power)]

    if jitter.size == 0:
        raise ValueError("No finite jitter samples.")
    if power.size == 0:
        raise ValueError("No finite power samples.")

    max_abs_jitter = float(np.max(np.abs(jitter)))
    mean_abs_jitter = float(np.mean(np.abs(jitter)))

    motoring = power[power > 0.0]
    duty = float(motoring.size / power.size)

    print("\n=== Jitter ===")
    print(f"Max |jitter| : {fmt_jitter_sec(max_abs_jitter)}")
    print(f"Mean |jitter|: {fmt_jitter_sec(mean_abs_jitter)}")

    print("\n=== Motoring power (power > 0 only) ===")
    if motoring.size == 0:
        print("No motoring samples found (power > 0).")
    else:
        print(f"Max motoring power : {float(np.max(motoring)):.6g}")
        print(f"Mean motoring power: {float(np.mean(motoring)):.6g}  (avg over motoring samples)")
        print(f"Motoring duty      : {duty * 100:.2f}% of samples")


def plot_series(t: np.ndarray, y: np.ndarray, xlabel: str, ylabel: str, title: str) -> None:
    plt.figure()
    plt.plot(t, y)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="process_telemetry.csv", help="Path to telemetry CSV")
    ap.add_argument("--limit", type=float, default=None, help="Plot only first N seconds (relative time)")
    ap.add_argument("--show_hist", action="store_true", help="Also show jitter histogram")
    ap.add_argument("--power_units", default="Power", help="Y-axis label for power (e.g., W)")
    args = ap.parse_args()

    d = load_csv(args.csv)

    t = d["t_sec"]
    jitter = d["jitter_sec"]
    pos = d["position"]
    vel = d["velocity"]
    power = d["power"]

    # Keep rows with finite time
    m = np.isfinite(t)
    t, jitter, pos, vel, power = t[m], jitter[m], pos[m], vel[m], power[m]

    if t.size < 2:
        raise ValueError("Not enough samples.")

    # Relative time for nicer plots
    t = t - t[0]

    # Optional time limit
    if args.limit is not None:
        mm = t <= args.limit
        t, jitter, pos, vel, power = t[mm], jitter[mm], pos[mm], vel[mm], power[mm]

    # Acceleration from velocity (only where t and vel are finite)
    accel = np.full_like(vel, np.nan)
    mtv = np.isfinite(t) & np.isfinite(vel)
    if np.count_nonzero(mtv) >= 3:
        accel[mtv] = compute_accel(t[mtv], vel[mtv])

    summarize(jitter, power)

    # Plots (jitter in us)
    plot_series(t, jitter * 1e6, "Time (s)", "Jitter (us)", "Loop Jitter vs Time")
    plot_series(t, pos, "Time (s)", "Position", "Position vs Time")
    plot_series(t, vel, "Time (s)", "Velocity", "Velocity vs Time")
    plot_series(t, accel, "Time (s)", "Acceleration", "Acceleration vs Time (computed)")
    plot_series(t, power, "Time (s)", args.power_units, "Power vs Time")

    if args.show_hist:
        plt.figure()
        jj = jitter[np.isfinite(jitter)] * 1e6
        plt.hist(jj, bins=100)
        plt.xlabel("Jitter (us)")
        plt.ylabel("Count")
        plt.title("Jitter Histogram")
        plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()
