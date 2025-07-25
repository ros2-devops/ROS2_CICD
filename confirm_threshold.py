# save_score_distributions.py
import os
import joblib
import numpy as np
import pandas as pd
from tensorflow.keras.models import load_model
import matplotlib.pyplot as plt

MODEL_DIR = "trained_models_new"
TRAIN_CSV = "data_training_full/ros_metrics_relabelled.csv"

print("🔍 Loading training set...")
df = pd.read_csv(TRAIN_CSV)
X = df.drop(columns=["Time", "Scenario", "Run", "Anomaly"])
y = df["Anomaly"]
scaler = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X_scaled = scaler.transform(X)
X_clean = X_scaled[y == 0]

# Isolation Forest scores
print("🌲 Scoring Isolation Forest...")
iforest = joblib.load(os.path.join(MODEL_DIR, "isolation_forest_model.pkl"))
isoforest_scores = -iforest.decision_function(X_clean)
np.save("score_hist_iforest.npy", isoforest_scores)
plt.hist(isoforest_scores, bins=100)
plt.title("Isolation Forest Scores (Clean Training Set)")
plt.savefig("score_hist_iforest.png")
plt.close()

# Autoencoder
print("🧠 Scoring Autoencoder...")
ae = load_model(os.path.join(MODEL_DIR, "autoencoder_model.keras"))
X_recon = ae.predict(X_clean, verbose=0)
ae_scores = np.mean(np.square(X_clean - X_recon), axis=1)
np.save("score_hist_ae.npy", ae_scores)
plt.hist(ae_scores, bins=100)
plt.title("Autoencoder Scores (Clean Training Set)")
plt.savefig("score_hist_ae.png")
plt.close()

# CNN-LSTM
print("📈 Scoring CNN-LSTM...")
SEQ_LEN = 20
X_seq = np.array([X_scaled[i:i+SEQ_LEN] for i in range(len(X_scaled) - SEQ_LEN)])
cnn = load_model(os.path.join(MODEL_DIR, "cnn_lstm_model.keras"))
cnn_scores = cnn.predict(X_seq, verbose=0).flatten()
np.save("score_hist_cnn_lstm.npy", cnn_scores)
plt.hist(cnn_scores, bins=100)
plt.title("CNN-LSTM Probabilities (Whole Training Set)")
plt.savefig("score_hist_cnn_lstm.png")
plt.close()

print("✅ Saved all histograms and score arrays.")
