#!/usr/bin/env python3
"""
Simulation + AI-Anomaly Evaluation Summary
──────────────────────────────────────────
• Reads (per scenario)
      simulation_log_<SCENARIO>.csv
      ros_metrics_<SCENARIO>.csv
      anomaly_result_log_<SCENARIO>.csv  (optional)
• Writes
      evaluation_summary_<SCENARIO>.txt
      evaluation_log.csv     (global append)
Made robust against older / lighter CSV formats.
"""

from __future__ import annotations
import os, sys, pandas as pd
from datetime import datetime

SCENARIO = os.getenv("SCENARIO", "unknown")

P = {                         # centralised paths
    "sim" : f"simulation_log_{SCENARIO}.csv",
    "met" : f"ros_metrics_{SCENARIO}.csv",
    "anom": f"anomaly_result_log_{SCENARIO}.csv",
    "sum" : f"evaluation_summary_{SCENARIO}.txt",
    "elog": "evaluation_log.csv",
}

# ─── guard clauses ──────────────────────────────────────────────────
for k,p in P.items():
    if k in ("anom", "log"):  # anomaly is optional
        continue
    if not os.path.exists(p):
        sys.exit(f"Missing required input: {p}")

# ─── read simulation-log ────────────────────────────────────────────
# new format has header, old one doesn't → detect automatically
sim_cols_full = ["ts","scenario","res","cpu_v","mem_v","maxCPU","maxMem"]
sim_df = pd.read_csv(P["sim"])
if list(sim_df.columns) != sim_cols_full:              # ⇢ old 3-col log
    sim_df.columns = ["ts","scenario","res"]
    sim_df["cpu_v"]  = sim_df["mem_v"] = 0
    sim_df["maxCPU"] = sim_df["maxMem"] = float("nan")

# ─── read metrics-log ───────────────────────────────────────────────
m_cols_full = ["Time","CPU","Memory",
               "CPU_mean","CPU_slope","Mem_mean","Mem_slope",
               "CPU_viol","Mem_viol"]
met_df = pd.read_csv(P["met"])
if len(met_df.columns) == 7:                           # modern light CSV
    met_df.columns = ["Time","CPU","Memory",
                      "CPU_mean","CPU_slope","Mem_mean","Mem_slope"]
    met_df["CPU_viol"] = met_df["Mem_viol"] = 0        # fallback

# cast
met_df = met_df.apply(pd.to_numeric, errors="coerce").dropna()

# ─── anomaly (optional) ─────────────────────────────────────────────
latest_an = None
if os.path.exists(P["anom"]):
    an_df = pd.read_csv(P["anom"])
    an_df = an_df[an_df["Scenario"] == SCENARIO]
    if not an_df.empty:
        latest_an = an_df.iloc[-1]

# ─── stats ──────────────────────────────────────────────────────────
runs       = len(sim_df)
pass_rate  = (sim_df["res"]=="PASS").mean()*100
viol_cpu   = sim_df["cpu_v"].sum()
viol_mem   = sim_df["mem_v"].sum()
p95_cpu    = met_df["CPU"].quantile(0.95)
p99_mem    = met_df["Memory"].quantile(0.99)
trend_cpu  = met_df["CPU_slope"].mean()
trend_mem  = met_df["Mem_slope"].mean()

# ─── summary report ────────────────────────────────────────────────
lines = [
    f"**Evaluation Summary – {SCENARIO}**",
    f"Generated : {datetime.now().isoformat()}",
    "",
    f"Runs      : {runs} &nbsp;·&nbsp; PASS rate **{pass_rate:.1f}%**",
    f"Violations: CPU {viol_cpu} / MEM {viol_mem}",
    "",
    f"CPU       : μ {met_df['CPU'].mean():.2f}% "
    f"· p95 {p95_cpu:.1f}% · trend {trend_cpu:+.3f}%/s",
    f"Memory    : μ {met_df['Memory'].mean():.2f}% "
    f"· p99 {p99_mem:.1f}% · trend {trend_mem:+.3f}%/s",
]

if latest_an is not None:
    lines += [
        "",
        "**AI-detected anomalies**",
        f"Model  : {latest_an['Model']}",
        f"Count  : {latest_an['AnomalyCount']} "
        f"({latest_an['AnomalyPct']} %)",
        f"Action : {latest_an['AI_Action']}",
    ]

txt = "\n".join(lines)
with open(P["sum"], "w") as f:
    f.write(txt)
print("✔ summary →", P["sum"])

# ─── global log (CSV) ───────────────────────────────────────────────
first = not os.path.exists(P["elog"])
with open(P["elog"], "a") as f:
    if first:
        f.write("Timestamp,Scenario,Runs,PassRate,CPU_p95,Mem_p99,CPU_trend,Mem_trend,CPUviol,Memviol\n")
    f.write(f"{datetime.now().isoformat()},{SCENARIO},{runs},{pass_rate:.1f},"
            f"{p95_cpu:.1f},{p99_mem:.1f},{trend_cpu:+.3f},{trend_mem:+.3f},"
            f"{viol_cpu},{viol_mem}\n")
print("✔ evaluation_log.csv appended")
