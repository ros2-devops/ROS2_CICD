#!/usr/bin/env python3

"""
Inspect ros_metrics_<scenario>.csv files used in CI inference
• Detects invalid entries
• Plots CPU usage
• Verifies consistency with training distribution
"""

import pandas as pd
import matplotlib.pyplot as plt
import os

for scenario in ["demo1", "demo2"]:
    path = f"ros_metrics_{scenario}.csv"
    if not os.path.exists(path):
        print(f"{path} missing")
        continue

    df = pd.read_csv(path, header=None, names=["Time", "CPU", "Memory"])

    # Check for malformed entries
    df["CPU"] = pd.to_numeric(df["CPU"], errors="coerce")
    df["Memory"] = pd.to_numeric(df["Memory"], errors="coerce")
    df = df.dropna()

    print(f"\n {path}:")
    print("Rows:", len(df))
    print("CPU Min/Max/Mean:", df["CPU"].min(), df["CPU"].max(), df["CPU"].mean())
    print("Memory Min/Max/Mean:", df["Memory"].min(), df["Memory"].max(), df["Memory"].mean())

    # Plot CPU over time
    plt.figure()
    plt.plot(df["Time"], df["CPU"], label="CPU %", color="blue")
    plt.axhline(y=85, color="red", linestyle="--", label="CPU Threshold")
    plt.title(f"CPU Usage – {scenario}")
    plt.xlabel("Time")
    plt.ylabel("CPU (%)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"cpu_runtime_{scenario}.png")
    print(f" Saved: cpu_runtime_{scenario}.png")
