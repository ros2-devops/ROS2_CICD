#!/usr/bin/env python3
"""
Summarize simulation results with AI anomaly context
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
• Aggregates sim results, metrics, and AI anomaly detection
• Appends a structured log entry to evaluation_log.csv
"""

import os
import sys
import pandas as pd
from datetime import datetime

scenario = os.getenv("SCENARIO", "unknown")

# Paths
log_path = f"simulation_log_{scenario}.csv"
metrics_path     = f"ros_metrics_{scenario}.csv"
anomaly_log_path = "anomaly_result_log.csv"
summary_path     = f"evaluation_summary_{scenario}.txt"
eval_log_path    = "evaluation_log.csv"

# Verify files
if not os.path.exists(log_path):
    print(f"{log_path} not found")
    sys.exit(1)

if not os.path.exists(metrics_path):
    print(f"{metrics_path} not found")
    sys.exit(1)

# Load core logs
log_df     = pd.read_csv(log_path, header=None, names=["timestamp", "scenario", "result"])
metrics_df = pd.read_csv(metrics_path, header=None, names=["Time", "CPU", "Memory"])

# Load last AI anomaly row for this scenario
if os.path.exists(anomaly_log_path):
    anomaly_df = pd.read_csv(anomaly_log_path)
    anomaly_df = anomaly_df[anomaly_df["Scenario"] == scenario]
    latest_anomaly = anomaly_df.iloc[-1] if not anomaly_df.empty else None
else:
    latest_anomaly = None

# Basic stats
total_runs = len(log_df)
pass_count = (log_df["result"] == "PASS").sum()
fail_count = total_runs - pass_count
avg_cpu    = metrics_df["CPU"].mean()
avg_mem    = metrics_df["Memory"].mean()

# Compose summary
lines = [
    "Simulation Evaluation Summary",
    f"Generated: {datetime.now().isoformat()}",
    "",
    f"Runs recorded in log: {total_runs}",
    f"   PASS: {pass_count}",
    f"   FAIL: {fail_count}",
    "",
    "Current-run averages:",
    f"   CPU   : {avg_cpu:.2f} %",
    f"   Memory: {avg_mem:.2f} %",
    "",
    f"Stability: {'Stable' if fail_count == 0 else 'Unstable (failures detected)'}"
]

if latest_anomaly is not None:
    lines += [
        "",
        "AI-Based Anomaly Summary:",
        f"   Detected Anomalies: {latest_anomaly['AnomalyScore']}",
        f"   Type              : {latest_anomaly['AnomalyType']}",
        f"   AI Response       : {latest_anomaly['AI_Action']}"
    ]

summary_text = "\n".join(lines)

# Write summary
with open(summary_path, "w") as f:
    f.write(summary_text)

# Append to evaluation log
header_needed = not os.path.exists(eval_log_path) or os.stat(eval_log_path).st_size == 0
with open(eval_log_path, "a") as f:
    if header_needed:
        f.write("Timestamp,Scenario,Summary\n")
    f.write(f"{datetime.now().isoformat()},{scenario},\"{summary_text.replace(chr(10), ' | ')}\"\n")

print(f"{summary_path} and {eval_log_path} updated.")
