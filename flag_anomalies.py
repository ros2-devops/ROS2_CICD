#!/usr/bin/env python3
import os, sys, joblib, numpy as np, pandas as pd, matplotlib.pyplot as plt
from datetime import datetime
from tensorflow.keras.models import load_model

# Runtime params
scenario  = os.getenv("SCENARIO", "unknown")
selector  = os.getenv("AI_MODEL", "cnn_lstm").lower()
MODEL_DIR = "trained_models"

# Files
csv_path   = f"ros_metrics_{scenario}.csv"
result_txt = f"anomaly_result_{selector}_{scenario}.txt"
plot_path  = f"anomaly_plot_{selector}_{scenario}.png"
log_path   = f"anomaly_result_log_{selector}_{scenario}.csv"
summary_csv = f"anomaly_summary_{scenario}.csv"

feature_cols = ["CPU", "Memory", "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope"]
model_files = {
    "cnn_lstm": ("anomaly_model_cnnlstm.keras", "cnn_lstm_threshold.pkl"),
    "ae"      : ("anomaly_model_ae.keras", "ae_threshold.pkl"),
    "iforest": ("anomaly_model_iforest.pkl", None)
}
model_file, thresh_file = model_files.get(selector, model_files["cnn_lstm"])

# Safeguards
def need(path, msg): 
    if not os.path.exists(path): sys.exit(msg)

need(csv_path, f"{csv_path} not found")
need(os.path.join(MODEL_DIR, model_file),  f"{model_file} missing")
need(os.path.join(MODEL_DIR, "scaler.pkl"), "scaler.pkl missing")
if thresh_file: need(os.path.join(MODEL_DIR, thresh_file), f"{thresh_file} missing")

# Load and clean
df = pd.read_csv(csv_path, names=["Time"] + feature_cols, skiprows=1)
df = df.dropna(subset=feature_cols + ["Time"])

scaler = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X_scaled = scaler.transform(df[feature_cols])

if selector == "iforest":
    model = joblib.load(os.path.join(MODEL_DIR, model_file))
    df["anomaly"] = model.predict(X_scaled)
else:
    model = load_model(os.path.join(MODEL_DIR, model_file))
    thresh = float(joblib.load(os.path.join(MODEL_DIR, thresh_file)))
    if selector == "cnn_lstm":
        STEP = 30
        pad = STEP - (len(X_scaled) % STEP)
        Xp = np.vstack([X_scaled, np.tile(X_scaled[-1], (pad, 1))])
        Xseq = Xp.reshape(-1, STEP, X_scaled.shape[1])
        recon = model.predict(Xseq, verbose=0).reshape(-1, X_scaled.shape[1])[:len(X_scaled)]
    else:
        X3d = X_scaled.reshape(-1, 20, X_scaled.shape[1])
        recon = model.predict(X3d, verbose=0).reshape(-1, X_scaled.shape[1])[:len(X_scaled)]
    errs = np.mean((recon - X_scaled)**2, axis=1)
    df["anomaly"] = (errs > thresh).astype(int) * -1

# Reporting
n_anom = int((df["anomaly"] == -1).sum())
p_anom = n_anom / len(df) * 100
atype  = "Resource-trend" if n_anom else "None"
action = "Investigate trend" if n_anom else "No action"
ts     = datetime.now().isoformat()

with open(result_txt, "w") as f:
    f.write(f"{'ANOMALY DETECTED' if n_anom else 'NO ANOMALY'}\n")
    f.write(f"Count: {n_anom}\nShare: {p_anom:.2f} %\n")
    f.write(f"Type:  {atype}\nAI Action: {action}\n")
print("✓", result_txt)

plt.figure(figsize=(8,3))
plt.plot(df["Time"], df["CPU"], lw=0.6, label="CPU %")
plt.scatter(df["Time"][df["anomaly"] == -1], df["CPU"][df["anomaly"] == -1],
            c="red", s=8, label="Anomaly", zorder=5)
plt.xlabel("Time (s)"); plt.ylabel("CPU %")
plt.title(f"CPU anomalies – {scenario} ({selector})")
plt.tight_layout(); plt.legend(); plt.savefig(plot_path)
print("✓", plot_path)

# Append detailed log
head = "Timestamp,Scenario,Model,AnomalyCount,AnomalyPct,AnomalyType,AI_Action\n"
row = f"{ts},{scenario},{selector},{n_anom},{p_anom:.2f},{atype},{action}\n"
first = not os.path.exists(log_path)
with open(log_path, "a") as f:
    if first: f.write(head)
    f.write(row)
print("✓", log_path, "updated")

# Write clean dashboard-ready summary CSV
pd.DataFrame([{
    "Timestamp": ts,
    "Scenario": scenario,
    "Model": selector,
    "AnomalyCount": n_anom,
    "AnomalyPct": f"{p_anom:.2f}",
    "AnomalyType": atype,
    "AI_Action": action
}]).to_csv(summary_csv, index=False)
print("✓", summary_csv, "created")
