#!/usr/bin/env python3
"""
Trains Isolation-Forest on the **scaled** flat samples.
Creates: anomaly_model_iforest.pkl
"""
import numpy as np, pickle, joblib, os
from sklearn.ensemble import IsolationForest
from sklearn.metrics import roc_auc_score

X = np.load("X_scaled.npy")
iso = IsolationForest(
        n_estimators=200, max_samples='auto',
        contamination=0.02, random_state=42).fit(X)

joblib.dump(iso, "anomaly_model_iforest.pkl")
print("[iforest] model saved → anomaly_model_iforest.pkl")
