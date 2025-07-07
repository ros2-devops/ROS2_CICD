#!/usr/bin/env python3
"""
Create two artefacts for the current $SCENARIO
  • evaluation_summary_<scenario>.txt   – human-readable one-pager
  • evaluation_log_<scenario>.csv       – append-only log (for dashboard)

Expects the following files (written by metrics_collector & CI steps):
  simulation_log_<scenario>.csv
  ros_metrics_<scenario>.csv
  anomaly_result_log_<scenario>.csv      (optional – adds last AI summary)
"""

import os
import sys
from datetime import datetime

import pandas as pd

# ─────────────────────────── paths ────────────────────────────
scenario = os.getenv("SCENARIO", "unknown")

log_path      = f"simulation_log_{scenario}.csv"
metrics_path  = f"ros_metrics_{scenario}.csv"
anomaly_path  = f"anomaly_result_log_{scenario}.csv"

summary_path  = f"evaluation_summary_{scenario}.txt"
eval_log_path = f"evaluation_log_{scenario}.csv"

for p in (log_path, metrics_path):
    if not os.path.exists(p):
        print(f"{p} not found")
        sys.exit(1)

# ───────────────── simulation-level log ───────────────────────
# 7   columns (timestamp, scenario, result, cpu_viol, mem_viol, max_cpu, max_mem)
log_df = pd.read_csv(
    log_path,
    header=None,
    names=[
        "timestamp",
        "scenario",
        "result",
        "cpu_viol",
        "mem_viol",
        "max_cpu",
        "max_mem",
    ],
)

# ───────────────── metrics CSV  (9 columns, 2 comment lines) ──
expected_cols = [
    "Time",
    "CPU",
    "Memory",
    "CPU_roll",
    "CPU_slope",
    "Mem_roll",
    "Mem_slope",
]

metrics_df = (
    pd.read_csv(metrics_path, comment="#")  # skip the two provenance lines
    .apply(pd.to_numeric, errors="coerce")  # force numeric
    .dropna(subset=["Time", "CPU", "Memory"])  # keep only real data rows
)

# keep just the columns we care about (extras such as CPU_viol are ignored)
metrics_df = metrics_df[[c for c in expected_cols if c in metrics_df.columns]]

if metrics_df.empty:
    print("metrics CSV is empty after cleaning – abort")
    sys.exit(1)

# ───────────────── latest AI anomaly (optional) ───────────────
if os.path.exists(anomaly_path):
    an_df = pd.read_csv(anomaly_path)
    latest_an = an_df[an_df["Scenario"] == scenario].iloc[-1] if not an_df.empty else None
else:
    latest_an = None

# ───────────────── aggregate statistics ───────────────────────
pass_cnt = (log_df["result"] == "PASS").sum()
fail_cnt = len(log_df) - pass_cnt

avg_cpu   = metrics_df["CPU"].mean()
max_cpu   = metrics_df["CPU"].max()
avg_mem   = metrics_df["Memory"].mean()
max_mem   = metrics_df["Memory"].max()
cpu_slope = metrics_df["CPU_slope"].mean()
mem_slope = metrics_df["Mem_slope"].mean()

# ───────────────── build human report ─────────────────────────
lines = [
    f"Scenario Evaluation Report — {scenario}",
    f"Timestamp: {datetime.now().isoformat()}",
    "",
    f"Runs: {len(log_df)}   PASS: {pass_cnt}   FAIL: {fail_cnt}",
    "",
    f"CPU:    Avg {avg_cpu:.2f}%   Max {max_cpu:.1f}%   Slope {cpu_slope:+.3f}",
    f"Memory: Avg {avg_mem:.2f}%   Max {max_mem:.1f}%   Slope {mem_slope:+.3f}",
    f"Stability: {'Stable ✅' if fail_cnt == 0 else 'Unstable ⚠'}",
]

if latest_an is not None:
    lines += [
        "",
        "Latest AI Anomaly Result:",
        f"  Model:        {latest_an['Model']}",
        f"  Anomalies:    {latest_an['AnomalyCount']} "
        f"({latest_an['AnomalyPct']}%)",
        f"  Type:         {latest_an['AnomalyType']}",
        f"  Recommended:  {latest_an['AI_Action']}",
    ]

summary_txt = "\n".join(lines)

with open(summary_path, "w") as f:
    f.write(summary_txt)

# Append to global evaluation log (one line, pipe-separated)
with open(eval_log_path, "a") as f:
    if not os.path.exists(eval_log_path) or os.stat(eval_log_path).st_size == 0:
        f.write("Timestamp,Scenario,Summary\n")
    compact = summary_txt.replace("\n", " | ")
    f.write(f"{datetime.now().isoformat()},{scenario},\"{compact}\"\n")

print(f"[✓] {summary_path} and {eval_log_path} updated.")
