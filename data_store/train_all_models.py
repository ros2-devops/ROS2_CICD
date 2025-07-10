#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import joblib
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import IsolationForest
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, LSTM, TimeDistributed
from tensorflow.keras.callbacks import EarlyStopping

# ───── Setup ─────
CSV = "ros_metrics_merged.csv"
MODEL_DIR = "trained_models"
os.makedirs(MODEL_DIR, exist_ok=True)

# ───── Load + Filter ─────
df = pd.read_csv(CSV)
feature_cols = ["CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope"]
df = df[df["CPU_viol"] == 0]           # Keep only non-violating rows
X = df[feature_cols].dropna().values.astype("float32")

# ───── Scale ─────
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)
joblib.dump(scaler, f"{MODEL_DIR}/scaler.pkl")

# ───── Isolation Forest ─────
iforest = IsolationForest(n_estimators=100, contamination=0.05, random_state=42)
iforest.fit(X_scaled)
joblib.dump(iforest, f"{MODEL_DIR}/anomaly_model_iforest.pkl")

# ───── Autoencoder ─────
ae = Sequential([
    Dense(12, activation='relu', input_shape=(X_scaled.shape[1],)),
    Dense(6, activation='relu'),
    Dense(12, activation='relu'),
    Dense(X_scaled.shape[1], activation='linear')
])
ae.compile(optimizer="adam", loss="mse")
ae.fit(X_scaled, X_scaled, epochs=100, batch_size=32, validation_split=0.1,
       callbacks=[EarlyStopping(patience=5, restore_best_weights=True)], verbose=1)
ae.save(f"{MODEL_DIR}/anomaly_model_ae.keras")

ae_recon = ae.predict(X_scaled)
ae_mse = np.mean(np.square(X_scaled - ae_recon), axis=1)
ae_thresh = np.percentile(ae_mse, 95)
joblib.dump(ae_thresh, f"{MODEL_DIR}/ae_threshold.pkl")

# ───── CNN-LSTM ─────
STEP = 30
pad = STEP - (len(X_scaled) % STEP)
X_pad = np.vstack([X_scaled, np.tile(X_scaled[-1], (pad, 1))])
X_seq = X_pad.reshape(-1, STEP, X_scaled.shape[1])

lstm = Sequential([
    LSTM(32, return_sequences=True, input_shape=(STEP, X_scaled.shape[1])),
    LSTM(16, return_sequences=True),
    TimeDistributed(Dense(X_scaled.shape[1]))
])
lstm.compile(optimizer="adam", loss="mse")
lstm.fit(X_seq, X_seq, epochs=50, batch_size=16, validation_split=0.1,
         callbacks=[EarlyStopping(patience=5, restore_best_weights=True)], verbose=1)
lstm.save(f"{MODEL_DIR}/anomaly_model_cnnlstm.keras")

lstm_recon = lstm.predict(X_seq).reshape(-1, X_scaled.shape[1])[:len(X_scaled)]
lstm_mse = np.mean(np.square(X_scaled - lstm_recon), axis=1)
lstm_thresh = np.percentile(lstm_mse, 95)
joblib.dump(lstm_thresh, f"{MODEL_DIR}/cnn_lstm_threshold.pkl")

print("✅ Training complete. Models saved to", MODEL_DIR)
