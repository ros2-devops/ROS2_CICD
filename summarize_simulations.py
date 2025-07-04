#!/usr/bin/env python3
import os, sys
import pandas as pd
from datetime import datetime

scenario = os.getenv("SCENARIO", "unknown")
log_path      = f"simulation_log_{scenario}.csv"
metrics_path  = f"ros_metrics_{scenario}.csv"
anomaly_path  = f"anomaly_result_log_{scenario}.csv"
summary_path  = f"evaluation_summary_{scenario}.txt"
summary_csv   = f"evaluation_summary_{scenario}.csv"
eval_log_path = f"evaluation_log_{scenario}.csv"

for p in (log_path, metrics_path):
    if not os.path.exists(p):
        print(f"{p} not found"); sys.exit(1)

log_df = pd.read_csv(log_path)
names = ["Time","CPU","Memory","CPU_roll","CPU_slope","Mem_roll","Mem_slope"]
metrics_df = (pd.read_csv(metrics_path, names=names, header=0)
                .apply(pd.to_numeric, errors='coerce')
                .dropna())

an_df = pd.read_csv(anomaly_path) if os.path.exists(anomaly_path) else pd.DataFrame()

# Basic stats
total_runs = len(log_df)
pass_cnt   = (log_df["Result"] == "PASS").sum()
fail_cnt   = total_runs - pass_cnt
avg_cpu    = metrics_df["CPU"].mean()
max_cpu    = metrics_df["CPU"].max()
avg_mem    = metrics_df["Memory"].mean()
max_mem    = metrics_df["Memory"].max()
avg_cpu_slope = metrics_df["CPU_slope"].mean()
avg_mem_slope = metrics_df["Mem_slope"].mean()
latest_an     = an_df[an_df["Scenario"] == scenario].iloc[-1] if not an_df.empty else None

lines = [
    "Simulation Evaluation Summary",
    f"Generated: {datetime.now().isoformat()}",
    f"Runs: {total_runs}  Pass: {pass_cnt}  Fail: {fail_cnt}",
    f"Avg CPU: {avg_cpu:.2f}%  Max CPU: {max_cpu:.1f}%",
    f"Avg slope CPU: {avg_cpu_slope:+.3f}",
    f"Avg Mem: {avg_mem:.2f}%  Max Mem: {max_mem:.1f}%",
    f"Avg slope Mem: {avg_mem_slope:+.3f}",
    f"Stability: {'Stable' if fail_cnt == 0 else 'Unstable'}"
]
if latest_an is not None:
    lines += [
        f"AI: {latest_an['Model']} - {latest_an['AnomalyCount']} anomalies "
        f"({latest_an['AnomalyPct']}%)"
    ]

with open(summary_path, "w") as f:
    f.write("\n".join(lines))

pd.DataFrame([{
    "Timestamp": datetime.now().isoformat(),
    "Scenario": scenario,
    "Passes": pass_cnt,
    "Fails": fail_cnt,
    "Avg_CPU": round(avg_cpu, 2),
    "Max_CPU": round(max_cpu, 1),
    "Avg_Mem": round(avg_mem, 2),
    "Max_Mem": round(max_mem, 1),
    "CPU_Slope": round(avg_cpu_slope, 3),
    "Mem_Slope": round(avg_mem_slope, 3),
    "AI_Model": latest_an["Model"] if latest_an is not None else "N/A",
    "AnomalyCount": latest_an["AnomalyCount"] if latest_an is not None else "N/A"
}]).to_csv(summary_csv, index=False)

with open(eval_log_path, "a") as f:
    f.write(f"{datetime.now().isoformat()},{scenario},\"{'; '.join(lines)}\"\n")

print(f"[OK] {summary_path} and {summary_csv} written")
