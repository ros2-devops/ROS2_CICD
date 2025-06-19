#!/usr/bin/env python3
"""
AI-Powered Anomaly Detection and Action Logger
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
• Loads per-scenario ros_metrics_*.csv
• Applies pretrained model to detect anomalies
• Logs structured result with timestamp, scenario, type, and AI action
• Saves annotated CPU anomaly plot per scenario
"""

import os
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

# Scenario (passed from environment)
scenario = os.getenv("SCENARIO", "unknown")

# Paths
ros_metrics_path = f"ros_metrics_{scenario}.csv"
model_path = "anomaly_model.pkl"
result_path = f"anomaly_result_{scenario}.txt"
plot_path = f"anomaly_plot_{scenario}.png"
log_path = "anomaly_result_log.csv"

# Load runtime metrics
if not os.path.exists(ros_metrics_path):
    print(f"{ros_metrics_path} not found")
    exit(1)

df = pd.read_csv(ros_metrics_path, header=None, names=["Time", "CPU", "Memory"])

# Preprocess (normalize CPU only to match training)
df["cpu_norm"] = df["CPU"] / df["CPU"].max()
X = df[["cpu_norm"]]

# Load trained model
if not os.path.exists(model_path):
    print("Trained model not found (anomaly_model.pkl)")
    exit(1)

model = joblib.load(model_path)

# Predict anomalies: -1 = anomaly, 1 = normal
df["anomaly"] = model.predict(X)
num_anomalies = (df["anomaly"] == -1).sum()

# Identify rule-based anomaly types
cpu_anomalies = df[df["CPU"] > 80]
mem_anomalies = df[df["Memory"] > 75]

# Decide AI action
if not cpu_anomalies.empty and mem_anomalies.empty:
    action = "Scale CPU allocation"
elif not mem_anomalies.empty and cpu_anomalies.empty:
    action = "Optimize memory usage"
elif not cpu_anomalies.empty and not mem_anomalies.empty:
    action = "Initiate load balancing"
else:
    action = "No action"

# Determine anomaly type
anomaly_type = "CPU" if not cpu_anomalies.empty and mem_anomalies.empty else \
               "Memory" if not mem_anomalies.empty and cpu_anomalies.empty else \
               "Both" if not cpu_anomalies.empty and not mem_anomalies.empty else \
               "None"

# Write per-scenario result file
with open(result_path, "w") as f:
    if num_anomalies > 0:
        f.write("ANOMALY DETECTED\n")
        f.write(f"Count: {num_anomalies} rows\n")
        f.write(f"Type: {anomaly_type}\n")
        f.write(f"AI Action: {action}\n")
    else:
        f.write("NO ANOMALY\n")
        f.write("AI Action: No action\n")

print(f"{result_path} written")

# Plot anomalies
df["Time"] = pd.to_numeric(df["Time"], errors="coerce")
df = df.dropna(subset=["Time", "CPU", "anomaly"])
times = df["Time"].to_numpy()
cpu = df["CPU"].to_numpy()
anomalies = df["anomaly"].to_numpy()

plt.figure()
plt.plot(times, cpu, label="CPU %", alpha=0.7)
plt.scatter(times[anomalies == -1], cpu[anomalies == -1], color="red", label="Anomaly", zorder=5)
plt.xlabel("Time (s)")
plt.ylabel("CPU Usage (%)")
plt.title(f"CPU Anomalies Over Time – {scenario}")
plt.legend()
plt.tight_layout()
plt.savefig(plot_path)
print(f"{plot_path} saved")

# Structured anomaly log (append mode)
timestamp = datetime.now().isoformat()
header = "Timestamp,Scenario,AnomalyScore,AnomalyType,AI_Action\n"
line = f"{timestamp},{scenario},{num_anomalies},{anomaly_type},{action}\n"

write_header = not os.path.exists(log_path) or os.stat(log_path).st_size == 0
with open(log_path, "a") as log:
    if write_header:
        log.write(header)
    log.write(line)

print("Structured anomaly_result_log.csv updated.")
