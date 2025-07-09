#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import joblib
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, LSTM, TimeDistributed
from tensorflow.keras import regularizers
from tensorflow.keras.callbacks import EarlyStopping

CSV_PATH = "data_store/ros_metrics_all.csv"
FEATURE_COLS = ["CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope"]
MODEL_DIR = "trained_models"
os.makedirs(MODEL_DIR, exist_ok=True)

df = pd.read_csv(CSV_PATH, usecols=FEATURE_COLS).dropna()
scaler = StandardScaler()
X_scaled = scaler.fit_transform(df)
joblib.dump(scaler, f"{MODEL_DIR}/scaler.pkl")

# Isolation Forest
iso = IsolationForest(contamination=0.05, random_state=42)
iso.fit(X_scaled)
joblib.dump(iso, f"{MODEL_DIR}/anomaly_model_iforest.pkl")

# Autoencoder
X_train, X_val = train_test_split(X_scaled, test_size=0.2, random_state=42)
input_dim = X_train.shape[1]

inp = Input(shape=(input_dim,))
enc = Dense(4, activation='relu', activity_regularizer=regularizers.l1(1e-5))(inp)
dec = Dense(input_dim, activation='linear')(enc)
ae = Model(inp, dec)
ae.compile(optimizer='adam', loss='mse')
ae.fit(X_train, X_train, epochs=100, batch_size=32, validation_data=(X_val, X_val),
       callbacks=[EarlyStopping(patience=5, restore_best_weights=True)], verbose=1)
ae.save(f"{MODEL_DIR}/anomaly_model_ae.keras")

recon = ae.predict(X_val)
mse = np.mean((X_val - recon) ** 2, axis=1)
th = np.percentile(mse, 95)
joblib.dump(th, f"{MODEL_DIR}/ae_threshold.pkl")

# CNN-LSTM
STEP = 30
pad = STEP - len(X_scaled) % STEP
X_pad = np.vstack([X_scaled, np.tile(X_scaled[-1], (pad, 1))])
X_seq = X_pad.reshape(-1, STEP, input_dim)

inp = Input(shape=(STEP, input_dim))
x = LSTM(64, return_sequences=True)(inp)
x = LSTM(32, return_sequences=True)(x)
out = TimeDistributed(Dense(input_dim))(x)
lstm = Model(inp, out)
lstm.compile(optimizer='adam', loss='mse')
lstm.fit(X_seq, X_seq, epochs=50, batch_size=16, verbose=1)
lstm.save(f"{MODEL_DIR}/anomaly_model_cnnlstm.keras")

recon_seq = lstm.predict(X_seq).reshape(-1, input_dim)[:len(X_scaled)]
mse_cnn = np.mean((X_scaled - recon_seq)**2, axis=1)
th_cnn = np.percentile(mse_cnn, 95)
joblib.dump(th_cnn, f"{MODEL_DIR}/cnn_lstm_threshold.pkl")

# Plot thresholds
plt.hist(mse, bins=50, color="gray")
plt.axvline(th, color="red", linestyle="--", label="AE Threshold")
plt.title("AE Reconstruction Error")
plt.savefig("ae_threshold_debug.png")

plt.figure()
plt.hist(mse_cnn, bins=50, color="gray")
plt.axvline(th_cnn, color="blue", linestyle="--", label="CNN-LSTM Threshold")
plt.title("CNN-LSTM Reconstruction Error")
plt.savefig("cnn_lstm_threshold_debug.png")
