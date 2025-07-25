#!/usr/bin/env python3
import os, sys, joblib, numpy as np, pandas as pd, matplotlib.pyplot as plt
from datetime import datetime
from tensorflow.keras.models import load_model

# ───── Setup ─────
scenario  = os.getenv("SCENARIO", "unknown")
selector  = os.getenv("AI_MODEL", "cnn_lstm").lower()
MODEL_DIR = "trained_models_new"

csv_path   = f"ros_metrics_{scenario}.csv"
result_txt = f"anomaly_result_{selector}_{scenario}.txt"
plot_path  = f"anomaly_plot_{selector}_{scenario}.png"
log_path   = f"anomaly_result_log_{selector}_{scenario}.csv"
recon_plot = f"recon_error_{selector}_{scenario}.png"


feature_cols = [
    "CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope",
    "CPU_user", "CPU_sys", "CPU_count", "CPU_freq",
    "Disk_read", "Disk_write", "Net_sent", "Net_recv"
]


model_files = {
    "cnn_lstm": ("cnn_lstm_model.keras", "cnn_lstm_threshold.pkl"),
    "ae":       ("autoencoder_model.keras", "autoencoder_threshold.pkl"),
    "iforest":  ("isolation_forest_model.pkl", None)
}
model_file, thresh_file = model_files.get(selector, model_files["cnn_lstm"])

# ───── Load Data ─────
if not os.path.exists(csv_path):
    sys.exit(f"Missing input CSV: {csv_path}")
df = pd.read_csv(csv_path)

if not set(feature_cols).issubset(df.columns):
    sys.exit("Missing expected columns in input CSV.")

X_raw = df[feature_cols].copy()
scaler = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X = scaler.transform(X_raw)

# ───── Load Model ─────
model_path = os.path.join(MODEL_DIR, model_file)
thresh_path = os.path.join(MODEL_DIR, thresh_file) if thresh_file else None

if selector == "iforest":
    model = joblib.load(model_path)
    preds = model.predict(X)
    is_anomaly = preds == -1
    scores = -model.decision_function(X)
    threshold = None

elif selector == "ae":
    model = load_model(model_path)
    recon = model.predict(X, verbose=0)
    scores = np.mean(np.square(X - recon), axis=1)
    threshold = joblib.load(thresh_path) if thresh_file else np.percentile(scores, 95)
    is_anomaly = scores > threshold

elif selector == "cnn_lstm":
    model = load_model(model_path)
    SEQ_LEN = 20
    if len(X) < SEQ_LEN:
        sys.exit("Not enough data rows for CNN-LSTM sequence input.")

    X_seq = np.array([X[i:i+SEQ_LEN] for i in range(len(X) - SEQ_LEN)])
    y_pred = model.predict(X_seq, verbose=0).flatten()
    full_scores = np.concatenate([np.full(SEQ_LEN, y_pred[0]), y_pred])[:len(X)]
    threshold = joblib.load(thresh_path) if thresh_file else 0.5
    is_anomaly = full_scores > threshold
    scores = full_scores

else:
    sys.exit(f"Unknown model selector: {selector}")

# ───── Result Assembly ─────
df_result = df.copy()
df_result["anomaly"] = is_anomaly
df_result["score"] = scores
df_result["Scenario"] = scenario
df_result["TimeIndex"] = df.index

# Human-readable reasons
explanations = []
for _, row in df_result.iterrows():
    if not row["anomaly"]:
        explanations.append("")
        continue
    reasons = []
    if row["CPU"] > 85: reasons.append("High CPU")
    if row["Memory"] > 80: reasons.append("High Memory")
    if row["CPU_slope"] > 20: reasons.append("CPU spike")
    if row["Mem_slope"] > 20: reasons.append("Memory spike")
    if selector != "iforest" and row["score"] > threshold:
        reasons.append("High recon error")
    explanations.append(" + ".join(reasons))

df_result["explanation"] = explanations

# ───── Output ─────
df_result.to_csv(log_path, index=False)
with open(result_txt, "w") as f:
    f.write(f"Model: {selector}\nScenario: {scenario}\nAnomalies flagged: {df_result['anomaly'].sum()} / {len(df_result)}\n")
    f.write(f"Threshold: {threshold if threshold else 'N/A'}\n")

# ───── Plot ─────
fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(df.index, df["CPU"], label="CPU")
ax.plot(df.index, df["Memory"], label="Memory")
ax.scatter(df_result[df_result["anomaly"]].index, df_result.loc[df_result["anomaly"], "CPU"], color="red", label="Anomaly", zorder=10)
ax.set_title(f"Anomaly Detection: {selector} on {scenario}")
ax.set_xlabel("Time Index")
ax.set_ylabel("Usage (%)")
ax.legend()
plt.tight_layout()
plt.savefig(plot_path)

if selector != "iforest":
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(df_result.index, df_result["score"], label="Recon/Prob Score")
    ax2.axhline(y=threshold, color="red", linestyle="--", label="Threshold")
    ax2.scatter(df_result[df_result["anomaly"]].index,
                df_result.loc[df_result["anomaly"], "score"],
                color="orange", label="Flagged")
    ax2.set_title("Anomaly Scores over Time")
    ax2.set_xlabel("Time Index")
    ax2.set_ylabel("Score")
    ax2.legend()
    plt.tight_layout()
    plt.savefig(recon_plot)

print(f"✅ Anomaly results saved to {log_path} with {df_result['anomaly'].sum()} anomalies.")
