#!/usr/bin/env python3
import os
import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tensorflow.keras.models import load_model

scenario = os.getenv("SCENARIO", "unknown")
selector = os.getenv("AI_MODEL", "cnn_lstm")
csv_path = f"ros_metrics_{scenario}.csv"
MODEL_DIR = "trained_models_new"

feature_cols = [
    "CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope",
    "CPU_user", "CPU_sys", "CPU_count", "CPU_freq",
    "Disk_read", "Disk_write", "Net_sent", "Net_recv"
]

df = pd.read_csv(csv_path)
X_raw = df[feature_cols].copy()
scaler = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X = scaler.transform(X_raw)

if selector == "ae":
    model = load_model(os.path.join(MODEL_DIR, "autoencoder_model.keras"))
    recon = model.predict(X, verbose=0)
    scores = np.mean(np.square(X - recon), axis=1)
elif selector == "cnn_lstm":
    SEQ_LEN = 20
    model = load_model(os.path.join(MODEL_DIR, "cnn_lstm_model.keras"))
    if len(X) < SEQ_LEN:
        print("⚠️ Not enough data for CNN-LSTM")
        exit(0)
    X_seq = np.array([X[i:i+SEQ_LEN] for i in range(len(X) - SEQ_LEN)])
    scores = model.predict(X_seq, verbose=0).flatten()
elif selector == "iforest":
    model = joblib.load(os.path.join(MODEL_DIR, "isolation_forest_model.pkl"))
    scores = -model.decision_function(X)
else:
    raise ValueError("Unknown model selector")

np.save(f"scores_{selector}_{scenario}.npy", scores)
plt.hist(scores, bins=100)
plt.title(f"{selector.upper()} Scores – {scenario}")
plt.savefig(f"scores_{selector}_{scenario}.png")
plt.close()
