#!/usr/bin/env python3
"""
Convert ros_metrics_all.csv → numpy arrays for sequence models
Outputs:
  X_seq.npy   shape (N, win, feat)
  scaler.pkl  (StandardScaler fitted on all features)
"""

import pandas as pd, numpy as np, joblib, sys, os
from sklearn.preprocessing import StandardScaler

CSV = "data_store/ros_metrics_all.csv"
WIN = 30                # 30 consecutive rows ≈ 15 s with LOG_INTERVAL=0.5

df = (pd.read_csv(CSV, header=None,
        names=["Time","CPU","Memory",
               "CPU_roll","CPU_slope","Mem_roll","Mem_slope"])
        .apply(pd.to_numeric, errors="coerce").dropna())

features = df[["CPU","Memory","CPU_roll","CPU_slope",
               "Mem_roll","Mem_slope"]].values

scaler = StandardScaler().fit(features)
feat_scaled = scaler.transform(features)

# sliding windows ---------------------------------------------------------
X = []
for i in range(len(feat_scaled) - WIN):
    X.append(feat_scaled[i:i+WIN])
X = np.asarray(X, dtype=np.float32)
np.save("X_seq.npy", X)
joblib.dump(scaler, "scaler.pkl")
print("Saved:", X.shape, "→ X_seq.npy  & scaler.pkl")
