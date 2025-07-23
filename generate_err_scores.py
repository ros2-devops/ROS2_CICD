#!/usr/bin/env python3
import os
import joblib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tensorflow.keras.models import load_model

# ─── Setup ─────────────────────────────
MODEL_DIR = "trained_models_new"
CSV_PATH = "dashboard_artifacts/ros_metrics_input_demo2.csv"
SAVE_DIR = "score_debug_plots"
os.makedirs(SAVE_DIR, exist_ok=True)

# ─── Load and preprocess data ──────────
df = pd.read_csv(CSV_PATH)

feature_cols = [
    "CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope",
    "CPU_user", "CPU_sys", "CPU_count", "CPU_freq",
    "Disk_read", "Disk_write", "Net_sent", "Net_recv",
    "Temperature", "CPU_viol", "Mem_viol"
]
X_raw = df[feature_cols]
scaler = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X = scaler.transform(X_raw)

# ─── Autoencoder ───────────────────────
ae_model = load_model(os.path.join(MODEL_DIR, "autoencoder_model.keras"))
ae_threshold = joblib.load(os.path.join(MODEL_DIR, "autoencoder_threshold.pkl"))

ae_recon = ae_model.predict(X, verbose=0)
ae_scores = np.mean(np.square(X - ae_recon), axis=1)

plt.figure(figsize=(10, 4))
plt.hist(ae_scores, bins=50, alpha=0.7, label="Recon. Errors")
plt.axvline(ae_threshold, color='r', linestyle='--', label=f"Threshold = {ae_threshold:.4f}")
plt.title("Autoencoder Reconstruction Error (demo2)")
plt.xlabel("Reconstruction Error")
plt.ylabel("Frequency")
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "ae_score_histogram.png"))
print("✅ Saved AE histogram")

# ─── CNN-LSTM ──────────────────────────
SEQ_LEN = 20
cnn_model = load_model(os.path.join(MODEL_DIR, "cnn_lstm_model.keras"))
cnn_threshold = joblib.load(os.path.join(MODEL_DIR, "cnn_lstm_threshold.pkl"))

if len(X) < SEQ_LEN:
    print("❌ Not enough data rows for CNN-LSTM")
else:
    X_seq = np.array([X[i:i+SEQ_LEN] for i in range(len(X) - SEQ_LEN)])
    cnn_scores = cnn_model.predict(X_seq, verbose=0).flatten()
    # Align scores to full length
    full_scores = np.concatenate([np.full(SEQ_LEN, cnn_scores[0]), cnn_scores])[:len(X)]

    plt.figure(figsize=(10, 4))
    plt.hist(full_scores, bins=50, alpha=0.7, label="CNN-LSTM Scores")
    plt.axvline(cnn_threshold, color='r', linestyle='--', label=f"Threshold = {cnn_threshold:.4f}")
    plt.title("CNN-LSTM Output Scores (demo2)")
    plt.xlabel("Score")
    plt.ylabel("Frequency")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, "cnn_lstm_score_histogram.png"))
    print("✅ Saved CNN-LSTM histogram")
