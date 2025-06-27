#!/usr/bin/env python3
"""
AI-Powered Anomaly Detection & Logger (pipeline-aware)
• Reads ros_metrics_<scenario>.csv  (6 feature columns produced by MetricsCollector)
• Loads pipeline model (StandardScaler + IsolationForest)
• Logs anomaly summary, plot, and structured CSV
"""

import os, sys, pandas as pd, joblib, matplotlib.pyplot as plt
from datetime import datetime

# -------------------------------------------------------------------
scenario      = os.getenv("SCENARIO", "unknown")
csv_path      = f"ros_metrics_{scenario}.csv"
model_path    = "anomaly_model_iforest.pkl"   # <- NEW model
result_path   = f"anomaly_result_{scenario}.txt"
plot_path     = f"anomaly_plot_{scenario}.png"
log_path      = f"anomaly_result_log_{scenario}.csv"

feature_cols  = ["CPU","Mem","CPU_roll","CPU_slope","Mem_roll","Mem_slope"]
# -------------------------------------------------------------------

if not os.path.exists(csv_path):
    sys.exit(f"{csv_path} not found")

# read – skip header row already present
df = pd.read_csv(csv_path, names=["Time"] + feature_cols, skiprows=1)

# convert every feature col to float (handles stray strings)
for c in feature_cols:
    df[c] = pd.to_numeric(df[c], errors="coerce")
df = df.dropna(subset=feature_cols)

# -------------------------------------------------------------------
if not os.path.exists(model_path):
    sys.exit(f"Model {model_path} not found")

pipe = joblib.load(model_path)
df["anomaly"] = pipe.predict(df[feature_cols])

num_anom  = (df["anomaly"] == -1).sum()
action    = "Investigate trend" if num_anom else "No action"
anom_type = "CPU/Mem trends"    if num_anom else "None"

# -------------------------------------------------------------------
with open(result_path, "w") as f:
    if num_anom:
        f.write("ANOMALY DETECTED\n")
        f.write(f"Count: {num_anom}\n")
        f.write(f"Type: {anom_type}\n")
        f.write(f"AI Action: {action}\n")
    else:
        f.write("NO ANOMALY\nAI Action: No action\n")

print(f"✓ {result_path}")

# Plot
plt.figure()
plt.plot(df["Time"], df["CPU"], label="CPU %", lw=0.6)
plt.scatter(df.loc[df.anomaly==-1,"Time"],
            df.loc[df.anomaly==-1,"CPU"],
            c="red", s=8, label="Anomaly", zorder=5)
plt.title(f"CPU Anomalies – {scenario}")
plt.xlabel("Time (s)"); plt.ylabel("CPU (%)"); plt.legend()
plt.tight_layout(); plt.savefig(plot_path); print(f"✓ {plot_path}")

# Append structured log
ts = datetime.now().isoformat()
header = "Timestamp,Scenario,AnomalyCount,AnomalyType,AI_Action\n"
line   = f"{ts},{scenario},{num_anom},{anom_type},{action}\n"

need_header = not os.path.exists(log_path)
with open(log_path, "a") as f:
    if need_header: f.write(header)
    f.write(line)
print(f"✓ {log_path} updated")
