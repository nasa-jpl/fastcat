#!/usr/bin/env python3
"""
Aggregate and plot telemetry summaries vs process loop frequency.

Scans a directory for files named:
  process_telemetry_<freq>Hz.csv

Computes per-file:
  - max absolute jitter
  - mean absolute jitter
  - max power
  - mean power
  - whether max absolute jitter exceeds one loop period

Then plots these metrics with frequency on the x-axis.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np


TELEMETRY_FILENAME_RE = re.compile(
    r"^process_telemetry_(?P<freq>[0-9]+(?:\.[0-9]+)?)Hz\.csv$"
)


@dataclass
class SweepPoint:
    frequency_hz: float
    max_abs_jitter_sec: float
    mean_abs_jitter_sec: float
    max_power: float
    mean_power: float
    max_abs_jitter_exceeds_period: int


def load_csv(path: str) -> dict[str, np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float, invalid_raise=False)
    if data.size == 0:
        raise ValueError(f"No rows found in {path}")

    names = list(data.dtype.names or [])
    required = ["jitter_sec", "power"]
    missing = [c for c in required if c not in names]
    if missing:
        raise ValueError(f"Missing required columns in {path}: {missing}")

    return {k: data[k].astype(float) for k in names}


def compute_summary(jitter: np.ndarray, power: np.ndarray) -> tuple[float, float, float, float]:
    jitter = jitter[np.isfinite(jitter)]
    power = power[np.isfinite(power)]

    if jitter.size == 0:
        raise ValueError("No finite jitter samples.")
    if power.size == 0:
        raise ValueError("No finite power samples.")

    max_abs_jitter = float(np.max(np.abs(jitter)))
    mean_abs_jitter = float(np.mean(np.abs(jitter)))
    max_power = float(np.max(power))
    mean_power = float(np.mean(power))
    return max_abs_jitter, mean_abs_jitter, max_power, mean_power


def collect_points(directory: str) -> list[SweepPoint]:
    points: list[SweepPoint] = []
    for name in sorted(os.listdir(directory)):
        m = TELEMETRY_FILENAME_RE.match(name)
        if not m:
            continue

        frequency_hz = float(m.group("freq"))
        path = os.path.join(directory, name)
        d = load_csv(path)
        max_abs_jitter, mean_abs_jitter, max_power, mean_power = compute_summary(
            d["jitter_sec"], d["power"]
        )
        period_sec = 1.0 / frequency_hz
        max_abs_jitter_exceeds_period = int(max_abs_jitter > period_sec)

        points.append(
            SweepPoint(
                frequency_hz=frequency_hz,
                max_abs_jitter_sec=max_abs_jitter,
                mean_abs_jitter_sec=mean_abs_jitter,
                max_power=max_power,
                mean_power=mean_power,
                max_abs_jitter_exceeds_period=max_abs_jitter_exceeds_period,
            )
        )

    points.sort(key=lambda p: p.frequency_hz)
    return points


def write_sweep_csv(points: list[SweepPoint], out_csv: str) -> None:
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "frequency_hz",
                "max_abs_jitter_sec",
                "mean_abs_jitter_sec",
                "max_power",
                "mean_power",
                "max_abs_jitter_exceeds_period",
            ]
        )
        for p in points:
            writer.writerow(
                [
                    p.frequency_hz,
                    p.max_abs_jitter_sec,
                    p.mean_abs_jitter_sec,
                    p.max_power,
                    p.mean_power,
                    p.max_abs_jitter_exceeds_period,
                ]
            )


def plot_sweep(points: list[SweepPoint]) -> None:
    freq = np.array([p.frequency_hz for p in points], dtype=float)
    max_abs_jitter_us = np.array([p.max_abs_jitter_sec for p in points], dtype=float) * 1e6
    mean_abs_jitter_us = np.array([p.mean_abs_jitter_sec for p in points], dtype=float) * 1e6
    max_power = np.array([p.max_power for p in points], dtype=float)
    mean_power = np.array([p.mean_power for p in points], dtype=float)
    exceeds_period = np.array([p.max_abs_jitter_exceeds_period for p in points], dtype=int)

    plt.figure()
    plt.plot(freq, max_abs_jitter_us, marker="o", label="Max |jitter|")
    plt.plot(freq, mean_abs_jitter_us, marker="o", label="Mean |jitter|")
    plt.xlabel("Process loop frequency (Hz)")
    plt.ylabel("Jitter (us)")
    plt.title("Jitter vs Process Loop Frequency")
    plt.grid(True)
    plt.legend()

    plt.figure()
    plt.plot(freq, max_power, marker="o", label="Max power")
    plt.plot(freq, mean_power, marker="o", label="Mean power")
    plt.xlabel("Process loop frequency (Hz)")
    plt.ylabel("Power (W)")
    plt.title("Power vs Process Loop Frequency")
    plt.grid(True)
    plt.legend()

    plt.figure()
    plt.step(freq, exceeds_period, where="mid")
    plt.scatter(freq, exceeds_period)
    plt.xlabel("Process loop frequency (Hz)")
    plt.ylabel("Max |jitter| > period (0/1)")
    plt.yticks([0, 1])
    plt.title("Jitter Exceeds Period vs Process Loop Frequency")
    plt.grid(True)

    plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=".", help="Directory containing process_telemetry_<freq>Hz.csv files")
    ap.add_argument(
        "--out_csv",
        default=None,
        help="Optional output CSV for sweep summary (default: <dir>/process_telemetry_sweep_summary.csv)",
    )
    args = ap.parse_args()

    points = collect_points(args.dir)
    if not points:
        raise ValueError(
            f"No telemetry files found in {args.dir} matching process_telemetry_<freq>Hz.csv"
        )

    out_csv = args.out_csv
    if out_csv is None:
        out_csv = os.path.join(args.dir, "process_telemetry_sweep_summary.csv")
    write_sweep_csv(points, out_csv)
    print(f"Wrote sweep summary CSV: {out_csv}")

    plot_sweep(points)


if __name__ == "__main__":
    main()
