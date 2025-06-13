#!/usr/bin/env python3
"""
ML-based Anomaly Detection for ROS 2 Metrics
‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
Uses Isolation Forest to detect anomalous CPU / memory readings.
"""

import os
import pandas as pd
import numpy as np
from sklearn.ensemble import IsolationForest

INPUT = os.path.join(os.getcwd(), "ros_metrics.csv")
OUTPUT = os.path.join(os.getcwd(), "anomaly_result.txt")

if not os.path.exists(INPUT):
    print("ros_metrics.csv not found")
    exit(1)

# ── Load and prepare data ─────────────────────────────
df = pd.read_csv(INPUT, header=None, names=["Time", "CPU", "Memory"])
X = df[["CPU", "Memory"]]

# ── Train isolation forest ────────────────────────────
model = IsolationForest(n_estimators=100, contamination=0.1, random_state=42)
model.fit(X)

# ── Predict anomalies ─────────────────────────────────
df["anomaly"] = model.predict(X)  # -1 = anomaly, 1 = normal
anomaly_rows = df[df["anomaly"] == -1]

# ── Write result ──────────────────────────────────────
with open(OUTPUT, "w") as f:
    if not anomaly_rows.empty:
        f.write("ANOMALY DETECTED\n")
        f.write(f"{len(anomaly_rows)} anomaly points detected\n")
        f.write("Sample rows:\n")
        f.write(anomaly_rows.head(5).to_string(index=False))
    else:
        f.write("NO ANOMALY\n")

print("ML-based anomaly_result.txt written.")
