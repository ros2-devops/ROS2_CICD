#!/usr/bin/env python3
"""
Simulation & AI-Anomaly Evaluation Summary
──────────────────────────────────────────
Generates a *stand-alone* textual digest + appends a CSV
row for longitudinal tracking across runs.
"""

from __future__ import annotations
import os, sys, pandas as pd
from datetime import datetime
from statistics import median

scenario = os.getenv("SCENARIO", "unknown")

paths = {
    "sim":   f"simulation_log_{scenario}.csv",
    "met":   f"ros_metrics_{scenario}.csv",
    "anom":  f"anomaly_result_log_{scenario}.csv",
    "sum":   f"evaluation_summary_{scenario}.txt",
    "elog":  "evaluation_log.csv"
}
for k,p in paths.items():
    if k != "anom" and not os.path.exists(p):
        sys.exit(f"Missing required input: {p}")

# ─── ingest ─────────────────────────────────────────────────────────
sim_df = pd.read_csv(paths["sim"],
                     names=["ts","scenario","res","cpu_v","mem_v","maxCPU","maxMem"],
                     header=0)

met_names = ["Time","CPU","Memory","CPU_mean","CPU_slope",
             "Mem_mean","Mem_slope","CPU_viol","Mem_viol"]
met_df = pd.read_csv(paths["met"], names=met_names, skiprows=2)

anom_df = (pd.read_csv(paths["anom"])
           if os.path.exists(paths["anom"]) else pd.DataFrame())

# ─── stats ──────────────────────────────────────────────────────────
runs = len(sim_df)
pass_rate = (sim_df["res"]=="PASS").mean()*100
p95_cpu = met_df["CPU"].quantile(0.95)
p99_mem = met_df["Memory"].quantile(0.99)
trend_cpu = met_df["CPU_slope"].mean()
trend_mem = met_df["Mem_slope"].mean()

summary = [
    f"Evaluation Summary – scenario: **{scenario}**",
    f"Generated: {datetime.now().isoformat()}",
    "",
    f"Runs     : {runs}  ·  PASS rate {pass_rate:.1f} %",
    f"CPU      : μ {met_df['CPU'].mean():.2f}%  · P95 {p95_cpu:.1f}%"
    f"  · trend {trend_cpu:+.3f}%/s",
    f"Memory   : μ {met_df['Memory'].mean():.2f}%  · P99 {p99_mem:.1f}%"
    f"  · trend {trend_mem:+.3f}%/s",
]

if not anom_df.empty:
    last = anom_df.iloc[-1]
    summary += [
        "",
        "AI Anomaly-detector:",
        f"model={last['Model']}  ·  count={last['AnomalyCount']}  "
        f"({last['AnomalyPct']} %)",
        f"action={last['AI_Action']}"
    ]

# ─── write outputs ──────────────────────────────────────────────────
with open(paths["sum"],"w") as f: f.write("\n".join(summary))
print(f"✔ summary → {paths['sum']}")

# append global evaluation_log.csv
is_new = not os.path.exists(paths["elog"])
with open(paths["elog"],"a") as f:
    if is_new:
        f.write("Timestamp,Scenario,Runs,PassRate,CPU_P95,Mem_P99\n")
    f.write(f"{datetime.now().isoformat()},{scenario},{runs},"
            f"{pass_rate:.1f},{p95_cpu:.1f},{p99_mem:.1f}\n")
print("✔ evaluation_log.csv appended")
