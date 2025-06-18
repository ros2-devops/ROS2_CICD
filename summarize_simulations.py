#!/usr/bin/env python3
"""
Summarise simulation results
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
• Reads simulation_log.csv to count PASS / FAIL runs
• Reads ros_metrics.csv from the *current* run
• Writes evaluation_summary.txt in the same directory
"""

import os
import sys
import pandas as pd
from datetime import datetime

WORK_DIR = os.getcwd()
log_path      = os.path.join(WORK_DIR, "simulation_log.csv")
metrics_path  = os.path.join(WORK_DIR, "ros_metrics.csv")
summary_path  = os.path.join(WORK_DIR, "evaluation_summary.txt")

# ── verify files exist ─────────────────────────────────
if not os.path.exists(log_path):
    print(f"{log_path} not found – has MetricsCollector appended a row?")
    sys.exit(1)

if not os.path.exists(metrics_path):
    print(f"{metrics_path} not found – did the simulation create ros_metrics.csv?")
    sys.exit(1)

# ── load data ─────────────────────────────────────────
log_df = pd.read_csv(log_path, header=None,
                     names=["timestamp", "scenario", "result"])
metrics_df = pd.read_csv(metrics_path, header=None,
                         names=["Time", "CPU", "Memory"])

# ── compute statistics ───────────────────────────────
total_runs = len(log_df)
pass_count = (log_df["result"] == "PASS").sum()
fail_count = total_runs - pass_count
avg_cpu    = metrics_df["CPU"].mean()
avg_mem    = metrics_df["Memory"].mean()

# ── write summary ─────────────────────────────────────
with open(summary_path, "w") as f:
    f.write("Simulation Evaluation Summary\n")
    f.write(f"Generated: {datetime.now().isoformat()}\n\n")
    f.write(f"Runs recorded in log: {total_runs}\n")
    f.write(f"   PASS: {pass_count}\n")
    f.write(f"   FAIL: {fail_count}\n\n")
    f.write(f"Current-run averages:\n")
    f.write(f"   CPU   : {avg_cpu:.2f} %\n")
    f.write(f"   Memory: {avg_mem:.2f} %\n")
    f.write("\nStability: " + ("Stable\n" if fail_count == 0 else "Unstable (failures detected)\n"))


print("evaluation_summary.txt generated.")
