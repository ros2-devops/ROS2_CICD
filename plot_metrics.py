#!/usr/bin/env python3
"""
Generate CPU / Memory line plots from ros_metrics.csv
The script expects the CSV to sit in the same directory
from which it is executed (the GitHub Actions workspace).
"""
import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

CSV_PATH = os.path.join(os.getcwd(), "ros_metrics.csv")

if not os.path.exists(CSV_PATH):
    print(f"{CSV_PATH} not found – did the simulation create it?")
    sys.exit(1)

# ── load data ────────────────────────────────────────────
df = pd.read_csv(CSV_PATH, header=None, names=["Time", "CPU", "Memory"])

# ── CPU plot ─────────────────────────────────────────────
plt.figure()
df.plot(x="Time", y="CPU", title="CPU Usage Over Time")
plt.xlabel("Time (s)")
plt.ylabel("CPU (%)")
plt.savefig("cpu_usage.png")

# ── Memory plot ──────────────────────────────────────────
plt.figure()
df.plot(x="Time", y="Memory", title="Memory Usage Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Memory (%)")
plt.savefig("memory_usage.png")

print("cpu_usage.png and memory_usage.png generated.")
