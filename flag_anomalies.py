#!/usr/bin/env python3
"""
AI-Powered Anomaly Detection and Action Logger
• Detects anomalies from ros_metrics_<scenario>.csv
• Applies pretrained model (Isolation Forest)
• Logs structured summary with timestamp, scenario, type, and action
"""

import os
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# ── Setup ──────────────────────────────
scenario = os.getenv("SCENARIO", "unknown")

ros_metrics_path = f"ros_metrics_{scenario}.csv"
model_path = "anomaly_model.pkl"
result_path = f"anomaly_result_{scenario}.txt"
plot_path = f"anomaly_plot_{scenario}.png"
log_path = f"anomaly_result_log_{scenario}.csv"

# ── Validate input ─────────────────────
if not os.path.exists(ros_metrics_path):
    print(f"{ros_metrics_path} not found")
    exit(1)

df = pd.read_csv(ros_metrics_path, header=None, names=["Time", "CPU", "Memory"])

# ── Preprocess ─────────────────────────
df["cpu_norm"] = df["CPU"] / df["CPU"].max()
X = df[["cpu_norm"]]

# ── Load model ─────────────────────────
if not os.path.exists(model_path):
    print("Trained model not found (anomaly_model.pkl)")
    exit(1)

model = joblib.load(model_path)
df["anomaly"] = model.predict(X)

# ── Count anomalies ────────────────────
num_anomalies = (df["anomaly"] == -1).sum()
cpu_anomalies = df[df["CPU"] > 80]
mem_anomalies = df[df["Memory"] > 75]

# ── Decide action only if real resource issue ───────
if num_anomalies > 0 and (not cpu_anomalies.empty or not mem_anomalies.empty):
    if not cpu_anomalies.empty and mem_anomalies.empty:
        anomaly_type = "CPU"
        action = "Scale CPU allocation"
    elif not mem_anomalies.empty and cpu_anomalies.empty:
        anomaly_type = "Memory"
        action = "Optimize memory usage"
    else:
        anomaly_type = "Both"
        action = "Initiate load balancing"
    result_flag = True
else:
    anomaly_type = "None"
    action = "No action"
    result_flag = False

# ── Write anomaly_result_<scenario>.txt ──────────────
with open(result_path, "w") as f:
    if result_flag:
        f.write("ANOMALY DETECTED\n")
        f.write(f"Count: {num_anomalies} rows\n")
        f.write(f"Type: {anomaly_type}\n")
        f.write(f"AI Action: {action}\n")
    else:
        f.write("NO ANOMALY\n")
        f.write("AI Action: No action\n")

print(f"{result_path} written")

# ── Plot anomalies ───────────────────────────────────
df["Time"] = pd.to_numeric(df["Time"], errors="coerce")
df = df.dropna(subset=["Time", "CPU", "anomaly"])
plt.figure()
plt.plot(df["Time"], df["CPU"], label="CPU %", alpha=0.7)
plt.scatter(df["Time"][df["anomaly"] == -1],
            df["CPU"][df["anomaly"] == -1],
            color="red", label="Anomaly", zorder=5)
plt.xlabel("Time (s)")
plt.ylabel("CPU Usage (%)")
plt.title(f"CPU Anomalies Over Time – {scenario}")
plt.legend()
plt.tight_layout()
plt.savefig(plot_path)
print(f"{plot_path} saved")

# ── Append structured log ───────────────
timestamp = datetime.now().isoformat()
header = "Timestamp,Scenario,AnomalyScore,AnomalyType,AI_Action\n"
line = f"{timestamp},{scenario},{num_anomalies},{anomaly_type},{action}\n"

write_header = not os.path.exists(log_path) or os.stat(log_path).st_size == 0
with open(log_path, "a") as log:
    if write_header:
        log.write(header)
    log.write(line)

print(f"{log_path} updated.")
