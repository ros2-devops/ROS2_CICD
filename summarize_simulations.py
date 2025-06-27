#!/usr/bin/env python3
"""
Summarise simulation results + AI anomaly context
─────────────────────────────────────────────────
Reads:
  simulation_log_<scenario>.csv
  ros_metrics_<scenario>.csv   (7-column version)
  anomaly_result_log_<scenario>.csv   (optional)

Writes:
  evaluation_summary_<scenario>.txt
  evaluation_log_<scenario>.csv   (appended)
"""

import os, sys
import pandas as pd
from datetime import datetime

scenario = os.getenv("SCENARIO", "unknown")

# ── paths ──────────────────────────────────────────────
log_path      = f"simulation_log_{scenario}.csv"
metrics_path  = f"ros_metrics_{scenario}.csv"
anomaly_path  = f"anomaly_result_log_{scenario}.csv"
summary_path  = f"evaluation_summary_{scenario}.txt"
eval_log_path = f"evaluation_log_{scenario}.csv"

for p in (log_path, metrics_path):
    if not os.path.exists(p):
        print(f"{p} not found"); sys.exit(1)

# ── load CSVs ──────────────────────────────────────────
log_df = pd.read_csv(log_path, header=None,
                     names=["timestamp", "scenario", "result"])

# metrics collector writes a header row – keep it
names = ["Time","CPU","Memory",
         "CPU_roll","CPU_slope","Mem_roll","Mem_slope"]
metrics_df = (pd.read_csv(metrics_path, names=names, header=0)
                .apply(pd.to_numeric, errors='coerce')
                .dropna())

if os.path.exists(anomaly_path):
    an_df = pd.read_csv(anomaly_path)
    an_sc = an_df[an_df["Scenario"] == scenario]
    latest_an = an_sc.iloc[-1] if not an_sc.empty else None
else:
    latest_an = None

# ── basic stats ───────────────────────────────────────
total_runs = len(log_df)
pass_cnt   = (log_df["result"] == "PASS").sum()
fail_cnt   = total_runs - pass_cnt

avg_cpu  = metrics_df["CPU"].mean()
max_cpu  = metrics_df["CPU"].max()
avg_mem  = metrics_df["Memory"].mean()
max_mem  = metrics_df["Memory"].max()

avg_cpu_slope = metrics_df["CPU_slope"].mean()
avg_mem_slope = metrics_df["Mem_slope"].mean()

# ── compose summary ───────────────────────────────────
lines = [
    "Simulation Evaluation Summary",
    f"Generated: {datetime.now().isoformat()}",
    "",
    f"Runs recorded: {total_runs}   PASS: {pass_cnt}   FAIL: {fail_cnt}",
    "",
    "CPU:",
    f"   Avg {avg_cpu:6.2f} %   Max {max_cpu:5.1f} %",
    f"   Avg slope {avg_cpu_slope:+.3f} %/s"
    + ("  ⚠ upward trend" if avg_cpu_slope > 0.2 else ""),
    "Memory:",
    f"   Avg {avg_mem:6.2f} %   Max {max_mem:5.1f} %",
    f"   Avg slope {avg_mem_slope:+.3f} %/s"
    + ("  ⚠ upward trend" if avg_mem_slope > 0.2 else ""),
    "",
    f"Stability: {'Stable' if fail_cnt == 0 else 'Unstable (failures detected)'}"
]

if latest_an is not None:
    lines += [
        "",
        "AI-Detected Anomalies:",
        f"   Count  : {latest_an['AnomalyScore']}",
        f"   Type   : {latest_an['AnomalyType']}",
        f"   Action : {latest_an['AI_Action']}"
    ]

summary_text = "\n".join(lines)

# ── write files ───────────────────────────────────────
with open(summary_path, "w") as f:
    f.write(summary_text)

append_header = (not os.path.exists(eval_log_path)
                 or os.stat(eval_log_path).st_size == 0)
with open(eval_log_path, "a") as f:
    if append_header:
        f.write("Timestamp,Scenario,Summary\n")
    compact = summary_text.replace("\n", " | ")
    f.write(f"{datetime.now().isoformat()},{scenario},\"{compact}\"\n")

print(f"[OK] {summary_path} written and log appended.")
