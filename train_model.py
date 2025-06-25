#!/usr/bin/env python3

"""
Train anomaly detection model from accumulated metrics
• Usage: python3 train_model.py ros_metrics_all.csv
• Output: anomaly_model.pkl
"""

import sys
import pandas as pd
import joblib
from sklearn.ensemble import IsolationForest
import os


# ── Handle CLI arg ───────────────────────────────
if len(sys.argv) != 2:
    print("Usage: python3 train_model.py <csv_path>")
    sys.exit(1)

csv_path = sys.argv[1]

if not os.path.exists(csv_path):
    print(f"File {csv_path} not found")
    sys.exit(1)


# ── Load and clean data ──────────────────────────
df = pd.read_csv(csv_path, header=None, names=["Time", "CPU", "Memory"])

# Convert CPU and Memory to numeric types (in case strings slipped in)
df["CPU"] = pd.to_numeric(df["CPU"], errors="coerce")
df["Memory"] = pd.to_numeric(df["Memory"], errors="coerce")

# Drop any rows with missing or invalid values
df = df.dropna()

df["cpu_norm"] = df["CPU"] / df["CPU"].max()
X = df[["cpu_norm"]]


df = df.dropna()
df["cpu_norm"] = df["CPU"] / df["CPU"].max()
X = df[["cpu_norm"]]

# ── Train model ──────────────────────────────────
model = IsolationForest(n_estimators=100, contamination=0.1, random_state=42)
model.fit(X)

joblib.dump(model, "anomaly_model.pkl")
print("Model trained and saved as anomaly_model.pkl")
