#!/usr/bin/env python3
"""
Unified AI-powered anomaly detector & logger
────────────────────────────────────────────
• Reads  ros_metrics_<SCENARIO>.csv  (6 feature columns)
• Chooses the pipeline by `AI_MODEL` env  →  iforest | ae | cnn_lstm
• All models + scaler live in   trained_models/
• Writes
      anomaly_result_<scenario>.txt
      anomaly_plot_<scenario>.png
      anomaly_result_log_<scenario>.csv   (appended)
"""

import os, sys, joblib, numpy as np, pandas as pd, matplotlib.pyplot as plt
from datetime import datetime

# ─────── runtime settings ─────────────────────────────────────────────
scenario  = os.getenv("SCENARIO", "unknown")
selector  = os.getenv("AI_MODEL", "cnn_lstm").lower()     # cnn_lstm | ae | iforest
MODEL_DIR = "trained_models"                              # <── central folder

csv_path   = f"ros_metrics_{scenario}.csv"
result_txt = f"anomaly_result_{scenario}.txt"
plot_path  = f"anomaly_plot_{scenario}.png"
log_path   = f"anomaly_result_log_{scenario}.csv"

feature_cols = ["CPU", "Memory", "CPU_roll",
                "CPU_slope", "Mem_roll", "Mem_slope"]

model_files = {
    "cnn_lstm": ("trained_models/anomaly_model_cnnlstm.keras",
                 "trained_models/cnn_lstm_threshold.pkl"),
    "ae":       ("trained_models/anomaly_model_ae.keras",
                 "trained_models/ae_threshold.pkl"),
    "iforest":  ("trained_models/anomaly_model_iforest.pkl", None)
}
model_file, thresh_file = model_files.get(selector, model_files["cnn_lstm"])

# ─────── path checks ──────────────────────────────────────────────────
def need(path, msg):
    if not os.path.exists(path):
        sys.exit(msg)

need(csv_path,   f"{csv_path} not found")
need(MODEL_DIR, "trained_models/ folder missing — commit the pickles")

model_path  = os.path.join(MODEL_DIR, model_file)
need(model_path, f"{model_path} not found")

scaler_path = "trained_models/scaler.pkl"
need(scaler_path, "trained_models/scaler.pkl not found")

if thresh_file:
    thresh_path = os.path.join(MODEL_DIR, thresh_file)
    need(thresh_path, f"{thresh_path} missing")

# ─────── load + clean csv ─────────────────────────────────────────────
df = pd.read_csv(csv_path, names=["Time"] + feature_cols, skiprows=1)

for col in feature_cols + ["Time"]:
    df[col] = pd.to_numeric(df[col], errors="coerce")
df = df.dropna(subset=feature_cols + ["Time"])
if df.empty:
    sys.exit("No valid rows after cleaning — aborting")

# ─────── scale features ───────────────────────────────────────────────
scaler   = joblib.load(scaler_path)
X_scaled = scaler.transform(df[feature_cols])

# ─────── model-specific inference ─────────────────────────────────────
if selector == "iforest":
    model = joblib.load(model_path)
    df["anomaly"] = model.predict(X_scaled)          # -1 = anomaly

else:  # Auto-Encoder or CNN-LSTM
    model  = joblib.load(model_path)
    thresh = float(joblib.load(thresh_path))

    if selector == "cnn_lstm":
        STEP = 30
        pad  = STEP - (len(X_scaled) % STEP)
        Xp   = np.vstack([X_scaled,
                          np.tile(X_scaled[-1], (pad, 1))])
        Xseq = Xp.reshape(-1, STEP, X_scaled.shape[1])
        recon = model.predict(Xseq, verbose=0).reshape(-1, X_scaled.shape[1])[:len(X_scaled)]
    else:                                   # auto-encoder
        recon = model.predict(X_scaled, verbose=0)

    errs = np.mean((recon - X_scaled) ** 2, axis=1)
    df["anomaly"] = (errs > thresh).astype(int) * -1

# ─────── summarise ────────────────────────────────────────────────────
n_anom   = int((df["anomaly"] == -1).sum())
p_anom   = n_anom / len(df) * 100
has_anom = n_anom > 0
action   = "Investigate trend" if has_anom else "No action"
atype    = "Resource-trend"    if has_anom else "None"

# plain-text
with open(result_txt, "w") as f:
    if has_anom:
        f.write(f"ANOMALY DETECTED\nCount: {n_anom}\nShare: {p_anom:.2f} %\n"
                f"Type:  {atype}\nAI Action: {action}\n")
    else:
        f.write("NO ANOMALY\nAI Action: No action\n")
print("✓", result_txt)

# plot
t, cpu  = df["Time"].to_numpy(), df["CPU"].to_numpy()
mask    = df["anomaly"].to_numpy() == -1

plt.figure(figsize=(8,3))
plt.plot(t, cpu, lw=0.6, label="CPU %")
plt.scatter(t[mask], cpu[mask], c="red", s=8, zorder=5, label="Anomaly")
plt.title(f"CPU anomalies – {scenario} ({selector})")
plt.xlabel("Time (s)"); plt.ylabel("CPU %"); plt.legend()
plt.tight_layout(); plt.savefig(plot_path)
print("✓", plot_path)

# csv log
ts     = datetime.now().isoformat()
header = "Timestamp,Scenario,Model,AnomalyCount,AnomalyPct,AI_Action\n"
row    = f"{ts},{scenario},{selector},{n_anom},{p_anom:.2f},{action}\n"

write_head = not os.path.exists(log_path)
with open(log_path, "a") as f:
    if write_head:
        f.write(header)
    f.write(row)
print("✓", log_path, "updated")
