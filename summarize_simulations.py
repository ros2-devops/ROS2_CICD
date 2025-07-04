#!/usr/bin/env python3
import os, sys, pandas as pd
from datetime import datetime

scenario = os.getenv("SCENARIO", "unknown")

log_path      = f"simulation_log_{scenario}.csv"
metrics_path  = f"ros_metrics_{scenario}.csv"
anomaly_path  = f"anomaly_result_log_{scenario}.csv"
summary_path  = f"evaluation_summary_{scenario}.txt"
eval_log_path = f"evaluation_log_{scenario}.csv"

for p in (log_path, metrics_path):
    if not os.path.exists(p):
        print(f"{p} not found"); sys.exit(1)

# ───────── load ─────────
log_df = pd.read_csv(log_path, header=None, names=["timestamp", "scenario", "result"])
metrics_df = pd.read_csv(metrics_path)
metrics_df = metrics_df[[c for c in metrics_df.columns if c in [
    "Time", "CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope"]]]
metrics_df = metrics_df.apply(pd.to_numeric, errors="coerce").dropna()

an_df = (pd.read_csv(anomaly_path)
         if os.path.exists(anomaly_path) else pd.DataFrame())

latest_an = (an_df[an_df["Scenario"] == scenario].iloc[-1]
             if not an_df.empty else None)

# ───────── stats ─────────
pass_cnt = (log_df["result"] == "PASS").sum()
fail_cnt = len(log_df) - pass_cnt

avg_cpu = metrics_df["CPU"].mean()
max_cpu = metrics_df["CPU"].max()
avg_mem = metrics_df["Memory"].mean()
max_mem = metrics_df["Memory"].max()
cpu_slope = metrics_df["CPU_slope"].mean()
mem_slope = metrics_df["Mem_slope"].mean()

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
        f"  Anomalies:    {latest_an['AnomalyCount']} ({latest_an['AnomalyPct']}%)",
        f"  Type:         {latest_an['AnomalyType']}",
        f"  Recommended:  {latest_an['AI_Action']}"
    ]

text = "\n".join(lines)
with open(summary_path, "w") as f:
    f.write(text)

with open(eval_log_path, "a") as f:
    if not os.path.exists(eval_log_path) or os.stat(eval_log_path).st_size == 0:
        f.write("Timestamp,Scenario,Summary\n")
    f.write(f"{datetime.now().isoformat()},{scenario},\"{text.replace(chr(10), ' | ')}\"\n")

print(f"[✓] {summary_path} and {eval_log_path} updated.")
