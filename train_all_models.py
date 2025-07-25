#!/usr/bin/env python3
import os
import json
import joblib
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import IsolationForest
from sklearn.metrics import classification_report

import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense, LSTM, Conv1D, MaxPooling1D
from tensorflow.keras.callbacks import EarlyStopping

# ─── Config ─────────────────────────────────────────────
INPUT_CSV = "data_training_full/ros_metrics_relabelled.csv"
MODEL_DIR = "trained_models_new"
os.makedirs(MODEL_DIR, exist_ok=True)

# ─── Load and Prepare Dataset ──────────────────────────
print("📦 Loading dataset...")
df = pd.read_csv(INPUT_CSV)
X = df.drop(columns=["Time", "Scenario","Temperature" , "CPU_viol", "Mem_viol" , "Run", "Anomaly"])
y = df["Anomaly"]

scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

print("🔀 Splitting dataset...")
X_train, X_test, y_train, y_test = train_test_split(
    X_scaled, y, test_size=0.2, stratify=y, random_state=42
)
X_train_clean = X_train[y_train == 0]

# ─── Isolation Forest ──────────────────────────────────
print("🌲 Training Isolation Forest...")
iforest = IsolationForest(contamination='auto', random_state=42)
iforest.fit(X_train_clean)

if_train_scores = -iforest.decision_function(X_train_clean)
if_threshold = np.percentile(if_train_scores, 98)

if_test_scores = -iforest.decision_function(X_test)
y_pred_iforest = (if_test_scores > if_threshold).astype(int)

# ─── Autoencoder ───────────────────────────────────────
print("🧠 Training Autoencoder...")
input_dim = X_train_clean.shape[1]
input_layer = Input(shape=(input_dim,))
x = Dense(32, activation="relu")(input_layer)
x = Dense(16, activation="relu")(x)
x = Dense(32, activation="relu")(x)
decoded = Dense(input_dim, activation="linear")(x)

autoencoder = Model(input_layer, decoded)
autoencoder.compile(optimizer="adam", loss="mse")

autoencoder.fit(
    X_train_clean, X_train_clean,
    epochs=50, batch_size=64, validation_split=0.2,
    callbacks=[EarlyStopping(monitor="val_loss", patience=5, restore_best_weights=True)],
    verbose=1
)

print("📏 Setting AE threshold based on clean reconstruction errors...")
recon_train = autoencoder.predict(X_train_clean, verbose=0)
recon_errors_train = np.mean(np.square(X_train_clean - recon_train), axis=1)
ae_threshold = np.percentile(recon_errors_train, 98)

recon_test = autoencoder.predict(X_test, verbose=0)
recon_errors_test = np.mean(np.square(X_test - recon_test), axis=1)
y_pred_ae = (recon_errors_test > ae_threshold).astype(int)

# ─── CNN-LSTM ───────────────────────────────────────────
print("📈 Preparing CNN-LSTM sequences...")
SEQ_LEN = 20
features = X_scaled.shape[1]
X_seq, y_seq = [], []

for i in range(len(X_scaled) - SEQ_LEN):
    X_seq.append(X_scaled[i:i+SEQ_LEN])
    y_seq.append(y.iloc[i+SEQ_LEN-1])

X_seq = np.array(X_seq)
y_seq = np.array(y_seq)

X_seq_train, X_seq_test, y_seq_train, y_seq_test = train_test_split(
    X_seq, y_seq, test_size=0.2, stratify=y_seq, random_state=42
)
X_seq_train_clean = X_seq_train[y_seq_train == 0]

print("🌀 Training CNN-LSTM...")
model_input = Input(shape=(SEQ_LEN, features))
x = Conv1D(filters=32, kernel_size=3, activation="relu")(model_input)
x = MaxPooling1D(pool_size=2)(x)
x = LSTM(32)(x)
x = Dense(16, activation="relu")(x)
output = Dense(1, activation="sigmoid")(x)

cnn_lstm = Model(model_input, output)
cnn_lstm.compile(optimizer="adam", loss="binary_crossentropy", metrics=["accuracy"])
cnn_lstm.fit(
    X_seq_train, y_seq_train,
    epochs=20, batch_size=64, validation_split=0.2,
    callbacks=[EarlyStopping(monitor="val_loss", patience=4, restore_best_weights=True)],
    verbose=1
)

print("📏 Setting CNN-LSTM threshold from clean scores...")
cnn_train_scores = cnn_lstm.predict(X_seq_train_clean, verbose=0).flatten()
cnn_threshold = np.percentile(cnn_train_scores, 98)

cnn_test_scores = cnn_lstm.predict(X_seq_test, verbose=0).flatten()
y_pred_cnn = (cnn_test_scores > cnn_threshold).astype(int)

# ─── Evaluation ────────────────────────────────────────
print("🧾 Classification Reports")
results = {
    "Isolation Forest": classification_report(y_test, y_pred_iforest, output_dict=True),
    "Autoencoder": classification_report(y_test, y_pred_ae, output_dict=True),
    "CNN-LSTM": classification_report(y_seq_test, y_pred_cnn, output_dict=True)
}

with open(os.path.join(MODEL_DIR, "model_eval_report.json"), "w") as f:
    json.dump(results, f, indent=2)

# ─── Save All Models and Assets ────────────────────────
print("💾 Saving models and thresholds...")
autoencoder.save(os.path.join(MODEL_DIR, "autoencoder_model.keras"))
cnn_lstm.save(os.path.join(MODEL_DIR, "cnn_lstm_model.keras"))
joblib.dump(iforest, os.path.join(MODEL_DIR, "isolation_forest_model.pkl"))
joblib.dump(ae_threshold, os.path.join(MODEL_DIR, "autoencoder_threshold.pkl"))
joblib.dump(cnn_threshold, os.path.join(MODEL_DIR, "cnn_lstm_threshold.pkl"))
joblib.dump(scaler, os.path.join(MODEL_DIR, "scaler.pkl"))

print("✅ Training complete. Models saved in:", MODEL_DIR)
