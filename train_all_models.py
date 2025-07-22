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

print("📦 Loading dataset...")
df = pd.read_csv(INPUT_CSV)
df = df.drop(columns=["Time", "Scenario", "Run"])

X = df.drop(columns=["Anomaly"])
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
iforest = IsolationForest(contamination=0.36, random_state=42)
iforest.fit(X_train_clean)
y_pred_iforest = iforest.predict(X_test)
y_pred_iforest = np.where(y_pred_iforest == -1, 1, 0)


# ─── Autoencoder ───────────────────────────────────────
print("🧠 Training Autoencoder...")
input_dim = X_train_clean.shape[1]
input_layer = Input(shape=(input_dim,))
encoded = Dense(32, activation="relu")(input_layer)
encoded = Dense(16, activation="relu")(encoded)
decoded = Dense(32, activation="relu")(encoded)
decoded = Dense(input_dim, activation="linear")(decoded)
autoencoder = Model(inputs=input_layer, outputs=decoded)
autoencoder.compile(optimizer="adam", loss="mse")

autoencoder.fit(
    X_train_clean, X_train_clean,
    epochs=30, batch_size=64,
    validation_split=0.2,
    callbacks=[EarlyStopping(monitor="val_loss", patience=5, restore_best_weights=True)],
    verbose=1
)


recon_errors = np.mean(np.square(X_test - autoencoder.predict(X_test, verbose=0)), axis=1)
ae_threshold = np.percentile(recon_errors, 100 * (1 - y_test.mean()))
y_pred_ae = (recon_errors > ae_threshold).astype(int)

# ─── CNN-LSTM ───────────────────────────────────────────
print("📈 Preparing data for CNN-LSTM...")
SEQ_LEN = 20
features = X_scaled.shape[1]
X_seq, y_seq = [], []

for i in range(len(X_scaled) - SEQ_LEN):
    X_seq.append(X_scaled[i:i+SEQ_LEN])
    y_seq.append(y.iloc[i+SEQ_LEN-1])

X_seq = np.array(X_seq)
y_seq = np.array(y_seq)

X_seq_train, X_seq_test, y_seq_train, y_seq_test = train_test_split(
    X_seq, y_seq, test_size=0.2, random_state=42, stratify=y_seq
)

print("🌀 Training CNN-LSTM...")
model_input = Input(shape=(SEQ_LEN, features))
x = Conv1D(filters=32, kernel_size=3, activation="relu")(model_input)
x = MaxPooling1D(pool_size=2)(x)
x = LSTM(32)(x)
x = Dense(16, activation="relu")(x)
output = Dense(1, activation="sigmoid")(x)

cnn_lstm = Model(inputs=model_input, outputs=output)
cnn_lstm.compile(optimizer="adam", loss="binary_crossentropy", metrics=["accuracy"])
cnn_lstm.fit(
    X_seq_train, y_seq_train,
    epochs=10, batch_size=64,
    validation_split=0.2,
    callbacks=[EarlyStopping(monitor="val_loss", patience=3, restore_best_weights=True)],
    verbose=1
)

y_pred_lstm = (cnn_lstm.predict(X_seq_test, verbose=0) > 0.5).astype(int)

# ─── Evaluation ────────────────────────────────────────
print("🧾 Generating evaluation report...")
results = {
    "Isolation Forest": classification_report(y_test, y_pred_iforest, output_dict=True),
    "Autoencoder": classification_report(y_test, y_pred_ae, output_dict=True),
    "CNN-LSTM": classification_report(y_seq_test, y_pred_lstm, output_dict=True)
}

with open(os.path.join(MODEL_DIR, "model_eval_report.json"), "w") as f:
    json.dump(results, f, indent=2)

# ─── Save Isolation Forest ───
joblib.dump(iforest, os.path.join(MODEL_DIR, "isolation_forest_model.pkl"))

# ─── Save Autoencoder and threshold ───
autoencoder.save(os.path.join(MODEL_DIR, "autoencoder_model.keras"))
with open(os.path.join(MODEL_DIR, "autoencoder_threshold.pkl"), "wb") as f:
    joblib.dump(ae_threshold, f)

# ─── Save CNN-LSTM and threshold ───
cnn_lstm.save(os.path.join(MODEL_DIR, "cnn_lstm_model.keras"))

# Use the same threshold logic for CNN-LSTM
cnn_scores = cnn_lstm.predict(X_seq_test, verbose=0).flatten()
cnn_threshold = np.percentile(cnn_scores, 100 * (1 - y_seq_test.mean()))
with open(os.path.join(MODEL_DIR, "cnn_lstm_threshold.pkl"), "wb") as f:
    joblib.dump(cnn_threshold, f)

# ─── Save Scaler ───
joblib.dump(scaler, os.path.join(MODEL_DIR, "scaler.pkl"))


print("✅ Training complete. Artifacts saved to:", MODEL_DIR)
