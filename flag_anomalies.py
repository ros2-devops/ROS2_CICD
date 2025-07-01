#!/usr/bin/env python3
"""
Unified AI-powered anomaly detector & logger
────────────────────────────────────────────
• Reads  ros_metrics_<SCENARIO>.csv  (6 feature columns)
• Chooses pipeline by $AI_MODEL  →  iforest | ae | cnn_lstm
• All models + scaler live in   trained_models/
• Outputs
      anomaly_result_<scenario>.txt
      anomaly_plot_<scenario>.png
      anomaly_result_log_<scenario>.csv   (append-only)
"""

import os, sys, joblib, numpy as np, pandas as pd, matplotlib.pyplot as plt
from datetime import datetime
from tensorflow.keras.models import load_model          # ← single import

# ───────── runtime params ──────────────────────────────────────────────
scenario  = os.getenv("SCENARIO", "unknown")
selector  = os.getenv("AI_MODEL", "cnn_lstm").lower()        # cnn_lstm | ae | iforest
MODEL_DIR = "trained_models"

csv_path   = f"ros_metrics_{scenario}.csv"
result_txt = f"anomaly_result_{selector}_{scenario}.txt"
plot_path  = f"anomaly_plot_{selector}_{scenario}.png"
log_path   = f"anomaly_result_log_{selector}_{scenario}.csv"

feature_cols = ["CPU", "Memory", "CPU_roll",
                "CPU_slope", "Mem_roll", "Mem_slope"]

model_files = {
    "cnn_lstm": ("anomaly_model_cnnlstm.keras", "cnn_lstm_threshold.pkl"),
    "ae"      : ("anomaly_model_ae.keras",       "ae_threshold.pkl"),
    "iforest" : ("anomaly_model_iforest.pkl",    None)
}
model_file, thresh_file = model_files.get(selector, model_files["cnn_lstm"])

# ───────── guard helpers ───────────────────────────────────────────────
def need(path, msg): 
    if not os.path.exists(path):
        sys.exit(msg)

need(csv_path,   f"{csv_path} not found")
need(MODEL_DIR, "trained_models/ folder missing — commit the pickles")
need(os.path.join(MODEL_DIR, model_file),  f"{model_file} not found")
need(os.path.join(MODEL_DIR, "scaler.pkl"),"trained_models/scaler.pkl not found")
if thresh_file:
    need(os.path.join(MODEL_DIR, thresh_file), f"{thresh_file} missing")

# ───────── ingest CSV ──────────────────────────────────────────────────
df = pd.read_csv(csv_path, names=["Time"] + feature_cols, skiprows=1)
for c in feature_cols + ["Time"]:
    df[c] = pd.to_numeric(df[c], errors="coerce")
df = df.dropna(subset=feature_cols + ["Time"])
if df.empty:
    sys.exit("No usable rows after cleaning — abort")

# ───────── scale features ──────────────────────────────────────────────
scaler   = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X_scaled = scaler.transform(df[feature_cols])

# ───────── inference ──────────────────────────────────────────────────
if selector == "iforest":
    model = joblib.load(os.path.join(MODEL_DIR, model_file))
    df["anomaly"] = model.predict(X_scaled)           # -1 = anomaly

else:
    model  = load_model(os.path.join(MODEL_DIR, model_file))
    thresh = float(joblib.load(os.path.join(MODEL_DIR, thresh_file)))

    if selector == "cnn_lstm":                        # needs 3-D window
        STEP = 30
        pad  = STEP - (len(X_scaled) % STEP)
        Xp   = np.vstack([X_scaled,
                          np.tile(X_scaled[-1], (pad, 1))])
        Xseq = Xp.reshape(-1, STEP, X_scaled.shape[1])
        recon = model.predict(Xseq, verbose=0)\
                      .reshape(-1, X_scaled.shape[1])[:len(X_scaled)]
    else:                                             # ⇢ AE expects (20,6)
        X3d   = X_scaled.reshape(-1, 20, X_scaled.shape[1])
        recon = model.predict(X3d, verbose=0)\
                      .reshape(-1, X_scaled.shape[1])[:len(X_scaled)]

    errs = np.mean((recon - X_scaled) ** 2, axis=1)
    df["anomaly"] = (errs > thresh).astype(int) * -1

# ───────── reporting ───────────────────────────────────────────────────
n_anom   = int((df["anomaly"] == -1).sum())
p_anom   = n_anom / len(df) * 100
action   = "Investigate trend" if n_anom else "No action"
atype    = "Resource-trend"    if n_anom else "None"

with open(result_txt, "w") as f:
    if n_anom:
        f.write(f"ANOMALY DETECTED\nCount: {n_anom}\nShare: {p_anom:.2f} %\n"
                f"Type:  {atype}\nAI Action: {action}\n")
    else:
        f.write("NO ANOMALY\nAI Action: No action\n")
print("✓", result_txt)

# ───────── quick plot ──────────────────────────────────────────────────
t, cpu = df["Time"].to_numpy(), df["CPU"].to_numpy()
mask   = df["anomaly"].to_numpy() == -1

plt.figure(figsize=(8,3))
plt.plot(t, cpu, lw=0.6, label="CPU %")
plt.scatter(t[mask], cpu[mask], c="red", s=8, label="Anomaly", zorder=5)
plt.xlabel("Time (s)"); plt.ylabel("CPU %")
plt.title(f"CPU anomalies – {scenario} ({selector})")
plt.tight_layout(); plt.legend(); plt.savefig(plot_path)
print("✓", plot_path)

# ───────── append CSV log ──────────────────────────────────────────────
ts   = datetime.now().isoformat()
head = "Timestamp,Scenario,Model,AnomalyCount,AnomalyPct,AI_Action\n"
row  = f"{ts},{scenario},{selector},{n_anom},{p_anom:.2f},{action}\n"
first = not os.path.exists(log_path)
with open(log_path, "a") as f:
    if first: f.write(head)
    f.write(row)
print("✓", log_path, "updated")
