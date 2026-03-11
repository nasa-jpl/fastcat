#!/usr/bin/env python3
"""
Plot and summarize telemetry from process_telemetry_<freq>Hz.csv.

CSV columns:
  t_sec,jitter_sec,position,velocity,current,power,cmd_position,ff_velocity,ff_current

Printed metrics:
  Jitter:
    - max(|jitter|)
    - mean(|jitter|)
    - max(process loop duration)
    - mean(process loop duration)
  Motoring power (power > 0 only):
    - max(power)
    - mean(power) averaged over motoring samples
    - motoring duty (% of samples with power > 0)

Plots:
  - jitter (us) vs time
  - process loop duration (us) vs time
  - position vs commanded position vs time
  - velocity vs commanded velocity vs time
  - current vs commanded current vs time
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
    required = ["t_sec", "jitter_sec", "process_loop_sec", "position", "velocity", "current", "power"]
    missing = [c for c in required if c not in names]
    if missing:
        raise ValueError(f"Missing required columns in CSV: {missing}. Found: {names}")

    return {k: data[k].astype(float) for k in names}


def first_present(d: dict[str, np.ndarray], candidates: list[str], label: str) -> np.ndarray:
    for name in candidates:
        if name in d:
            return d[name]
    raise ValueError(f"Missing {label}. Expected one of columns: {candidates}")


def fmt_jitter_sec(x: float) -> str:
    return f"{x:.9e} s  ({x * 1e6:.3f} us)"


def fmt_process_loop_sec(x: float) -> str:
    return f"{x:.9e} s  ({x * 1e6:.3f} us)"


def compute_summary(jitter: np.ndarray, process_loop: np.ndarray, power: np.ndarray) -> dict[str, float]:
    jitter = jitter[np.isfinite(jitter)]
    process_loop = process_loop[np.isfinite(process_loop)]
    power = power[np.isfinite(power)]

    if jitter.size == 0:
        raise ValueError("No finite jitter samples.")
    if process_loop.size == 0:
        raise ValueError("No finite process loop duration samples.")
    if power.size == 0:
        raise ValueError("No finite power samples.")

    max_abs_jitter = float(np.max(np.abs(jitter)))
    mean_abs_jitter = float(np.mean(np.abs(jitter)))
    max_process_loop = float(np.max(process_loop))
    mean_process_loop = float(np.mean(process_loop))
    max_power = float(np.max(power))
    mean_power = float(np.mean(power))

    motoring = power[power > 0.0]
    duty = float(motoring.size / power.size)

    max_motoring_power = float(np.max(motoring)) if motoring.size else float("nan")
    mean_motoring_power = float(np.mean(motoring)) if motoring.size else float("nan")

    return {
        "max_abs_jitter_sec": max_abs_jitter,
        "mean_abs_jitter_sec": mean_abs_jitter,
        "max_process_loop_sec": max_process_loop,
        "mean_process_loop_sec": mean_process_loop,
        "max_power": max_power,
        "mean_power": mean_power,
        "max_motoring_power": max_motoring_power,
        "mean_motoring_power": mean_motoring_power,
        "motoring_duty": duty,
        "n_samples": float(power.size),
    }


def summarize(summary: dict[str, float]) -> None:
    max_abs_jitter = summary["max_abs_jitter_sec"]
    mean_abs_jitter = summary["mean_abs_jitter_sec"]
    max_process_loop = summary["max_process_loop_sec"]
    mean_process_loop = summary["mean_process_loop_sec"]
    max_power = summary["max_power"]
    mean_power = summary["mean_power"]
    max_motoring_power = summary["max_motoring_power"]
    mean_motoring_power = summary["mean_motoring_power"]
    duty = summary["motoring_duty"]

    print("\n=== Jitter ===")
    print(f"Max |jitter| : {fmt_jitter_sec(max_abs_jitter)}")
    print(f"Mean |jitter|: {fmt_jitter_sec(mean_abs_jitter)}")

    print("\n=== Process loop duration ===")
    print(f"Max (process loop) : {fmt_process_loop_sec(max_process_loop)}")
    print(f"Mean (process loop): {fmt_process_loop_sec(mean_process_loop)}")

    print("\n=== Power (all samples) ===")
    print(f"Max power       : {max_power:.6g}")
    print(f"Mean power      : {mean_power:.6g}")

    print("\n=== Motoring power (power > 0 only) ===")
    if not np.isfinite(max_motoring_power):
        print("No motoring samples found (power > 0).")
    else:
        print(f"Max motoring power : {max_motoring_power:.6g}")
        print(f"Mean motoring power: {mean_motoring_power:.6g}  (avg over motoring samples)")
        print(f"Motoring duty      : {duty * 100:.2f}% of samples")


def plot_series(t: np.ndarray, y: np.ndarray, xlabel: str, ylabel: str, title: str) -> None:
    plt.figure()
    plt.plot(t, y)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)


def plot_pair(
    t: np.ndarray,
    actual: np.ndarray,
    commanded: np.ndarray,
    ylabel: str,
    title: str,
    actual_label: str,
    commanded_label: str,
) -> None:
    plt.figure()
    plt.plot(t, actual, label=actual_label)
    plt.plot(t, commanded, label=commanded_label)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.grid(True)
    plt.legend()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="process_telemetry_10Hz.csv", help="Path to telemetry CSV")
    ap.add_argument("--limit", type=float, default=None, help="Plot only first N seconds (relative time)")
    ap.add_argument("--show_hist", action="store_true", help="Also show jitter histogram")
    ap.add_argument("--power_units", default="Power", help="Y-axis label for power (e.g., W)")
    args = ap.parse_args()

    d = load_csv(args.csv)

    t = d["t_sec"]
    jitter = d["jitter_sec"]
    process_loop = d["process_loop_sec"]
    pos = d["position"]
    vel = d["velocity"]
    cur = d["current"]
    power = d["power"]
    cmd_pos = first_present(d, ["cmd_position"], "commanded position")
    cmd_vel = first_present(d, ["cmd_velocity", "ff_velocity"], "commanded velocity")
    cmd_cur = first_present(d, ["cmd_current", "ff_current"], "commanded current")

    # Keep rows with finite time
    m = np.isfinite(t)
    t = t[m]
    jitter = jitter[m]
    process_loop = process_loop[m]
    pos = pos[m]
    vel = vel[m]
    cur = cur[m]
    power = power[m]
    cmd_pos = cmd_pos[m]
    cmd_vel = cmd_vel[m]
    cmd_cur = cmd_cur[m]

    if t.size < 2:
        raise ValueError("Not enough samples.")

    # Relative time for nicer plots
    t = t - t[0]

    # Optional time limit
    if args.limit is not None:
        mm = t <= args.limit
        t = t[mm]
        jitter = jitter[mm]
        process_loop = process_loop[mm]
        pos = pos[mm]
        vel = vel[mm]
        cur = cur[mm]
        power = power[mm]
        cmd_pos = cmd_pos[mm]
        cmd_vel = cmd_vel[mm]
        cmd_cur = cmd_cur[mm]

    summary = compute_summary(jitter, process_loop, power)
    summarize(summary)

    # Plots (jitter in us)
    plot_series(t, jitter * 1e6, "Time (s)", "Jitter (us)", "Loop Jitter vs Time")
    plot_series(t, process_loop * 1e6, "Time (s)", "Process Loop Duration (us)", "Process Loop Duration vs Time")
    plot_pair(
        t,
        pos,
        cmd_pos,
        "Position",
        "Position vs Commanded Position",
        "Actual Position",
        "Commanded Position",
    )
    plot_pair(
        t,
        vel,
        cmd_vel,
        "Velocity",
        "Velocity vs Commanded Velocity",
        "Actual Velocity",
        "Commanded Velocity",
    )
    plot_pair(
        t,
        cur,
        cmd_cur,
        "Current",
        "Current vs Commanded Current",
        "Actual Current",
        "Commanded Current",
    )
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
