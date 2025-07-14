#!/usr/bin/env python3
import os, sys, joblib, numpy as np, pandas as pd, matplotlib.pyplot as plt
from datetime import datetime
from tensorflow.keras.models import load_model

scenario  = os.getenv("SCENARIO", "unknown")
selector  = os.getenv("AI_MODEL", "cnn_lstm").lower()
MODEL_DIR = "trained_models"

csv_path   = f"ros_metrics_{scenario}.csv"
result_txt = f"anomaly_result_{selector}_{scenario}.txt"
plot_path  = f"anomaly_plot_{selector}_{scenario}.png"
log_path   = f"anomaly_result_log_{selector}_{scenario}.csv"
recon_plot = f"recon_error_{selector}_{scenario}.png"

feature_cols = ["CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope"]
model_files = {
    "cnn_lstm": ("anomaly_model_cnnlstm.keras", "cnn_lstm_threshold.pkl"),
    "ae":       ("anomaly_model_ae.keras",       "ae_threshold.pkl"),
    "iforest":  ("anomaly_model_iforest.pkl",    None)
}
model_file, thresh_file = model_files.get(selector, model_files["cnn_lstm"])

# ───────── Load data ─────────
if not os.path.exists(csv_path):
    sys.exit(f"Missing input CSV: {csv_path}")
df = pd.read_csv(csv_path)
df = df[feature_cols].dropna()

# ───────── Load model ─────────
model_path = os.path.join(MODEL_DIR, model_file)
thresh_path = os.path.join(MODEL_DIR, thresh_file) if thresh_file else None

if selector == "iforest":
    model = joblib.load(model_path)
    scores = -model.decision_function(df)
    preds  = model.predict(df)
    is_anomaly = preds == 1
    threshold = None
else:
    model = load_model(model_path)
    X = df.values.astype(np.float32)
    X = X.reshape((X.shape[0], 1, X.shape[1]))
    recon = model.predict(X, verbose=0)
    errors = np.mean(np.square(X.squeeze() - recon.squeeze()), axis=1)
    if thresh_file:
        threshold = joblib.load(thresh_path)
    else:
        threshold = np.percentile(errors, 95)
    is_anomaly = errors > threshold
    scores = errors

# ───────── Add context to anomalies ─────────
df_result = df.copy()
df_result["anomaly"] = is_anomaly
if selector != "iforest":
    df_result["recon_error"] = scores
else:
    df_result["iforest_score"] = scores

# Add basic context
if "Time" in df.columns:
    df_result["Time"] = pd.to_datetime(df["Time"], errors="coerce")
else:
    df_result["Time"] = pd.date_range(start=datetime.now(), periods=len(df_result), freq="S")

df_result["Scenario"] = scenario

# Explanation per anomaly row
explanations = []
for idx, row in df_result.iterrows():
    if not row["anomaly"]:
        explanations.append("")
        continue
    reasons = []
    if row["CPU"] > 80:
        reasons.append("High CPU")
    if row["CPU_slope"] > 20:
        reasons.append("CPU spike")
    if row["Memory"] > 80:
        reasons.append("High Memory")
    if row["Mem_slope"] > 20:
        reasons.append("Memory spike")
    if selector != "iforest" and row["recon_error"] > threshold:
        reasons.append("High recon error")
    explanations.append(" + ".join(reasons))

df_result["explanation"] = explanations

# ───────── Output files ─────────
anom_count = df_result["anomaly"].sum()
with open(result_txt, "w") as f:
    f.write(f"Model: {selector}\nScenario: {scenario}\nAnomalies flagged: {anom_count} / {len(df_result)}\n")
    f.write(f"Threshold: {threshold if threshold else 'N/A'}\n")

# Save detailed anomaly log
df_result.to_csv(log_path, index=False)

# ───────── Plots ─────────
fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(df_result.index, df_result["CPU"], label="CPU")
ax.plot(df_result.index, df_result["Memory"], label="Memory")
ax.scatter(df_result[df_result["anomaly"]].index,
           df_result[df_result["anomaly"]]["CPU"],
           color="red", label="Anomaly", zorder=10)
ax.set_title(f"Anomaly Detection: {selector} on {scenario}")
ax.set_xlabel("Time Index")
ax.set_ylabel("% Usage")
ax.legend()
plt.tight_layout()
plt.savefig(plot_path)

# Optional: plot reconstruction error if DL model
if selector != "iforest":
    fig2, ax2 = plt.subplots(figsize=(10, 4))
    ax2.plot(df_result.index, df_result["recon_error"], label="Recon Error")
    ax2.axhline(y=threshold, color="red", linestyle="--", label="Threshold")
    ax2.scatter(df_result[df_result["anomaly"]].index,
                df_result[df_result["anomaly"]]["recon_error"],
                color="orange", label="Flagged")
    ax2.set_title("Reconstruction Error over Time")
    ax2.set_xlabel("Time Index")
    ax2.set_ylabel("MSE")
    ax2.legend()
    plt.tight_layout()
    plt.savefig(recon_plot)

print(f"Anomaly results saved to {log_path} with {anom_count} anomalies.")
