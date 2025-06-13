#!/usr/bin/env python3
"""
Basic AI-like anomaly flagging
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
Flags CPU > 80% or Memory > 75%
Writes: anomaly_result.txt
"""
import os
import pandas as pd

INPUT = os.path.join(os.getcwd(), "ros_metrics.csv")
OUTPUT = os.path.join(os.getcwd(), "anomaly_result.txt")

if not os.path.exists(INPUT):
    print("ros_metrics.csv not found")
    exit(1)

df = pd.read_csv(INPUT, header=None, names=["Time", "CPU", "Memory"])

cpu_anomalies = df[df["CPU"] > 80]
mem_anomalies = df[df["Memory"] > 75]

with open(OUTPUT, "w") as f:
    if not cpu_anomalies.empty or not mem_anomalies.empty:
        f.write("ANOMALY DETECTED\n")
        f.write(f"High CPU rows: {len(cpu_anomalies)}\n")
        f.write(f"High Mem rows: {len(mem_anomalies)}\n")
    else:
        f.write("NO ANOMALY\n")

print("anomaly_result.txt written.")
