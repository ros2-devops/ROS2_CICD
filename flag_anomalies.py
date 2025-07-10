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

# ───────── check input ─────────
def need(path, msg): 
    if not os.path.exists(path): sys.exit(msg)

need(csv_path, f"{csv_path} not found")
need(MODEL_DIR, "trained_models/ folder missing")
need(os.path.join(MODEL_DIR, model_file),  f"{model_file} not found")
need(os.path.join(MODEL_DIR, "scaler.pkl"), "trained_models/scaler.pkl not found")
if thresh_file:
    need(os.path.join(MODEL_DIR, thresh_file), f"{thresh_file} missing")

# ───────── ingest ─────────
df = (
    pd.read_csv(csv_path, usecols=["Time"] + feature_cols)
      .apply(pd.to_numeric, errors="coerce")
      .dropna()
)
if df.empty: sys.exit("No usable rows in cleaned DataFrame")

scaler = joblib.load(os.path.join(MODEL_DIR, "scaler.pkl"))
X_scaled = scaler.transform(df[feature_cols])

# ───────── inference ─────────
if selector == "iforest":
    model = joblib.load(os.path.join(MODEL_DIR, model_file))
    df["anomaly"] = model.predict(X_scaled)  # -1 = anomaly

else:
    
    model = load_model(os.path.join(MODEL_DIR, model_file))

    if selector == "cnn_lstm":
        STEP = 20
        pad = STEP - (len(X_scaled) % STEP)
        Xp = np.vstack([X_scaled, np.tile(X_scaled[-1], (pad, 1))])
        Xseq = Xp.reshape(-1, STEP, X_scaled.shape[1])
        recon = model.predict(Xseq, verbose=0).reshape(-1, X_scaled.shape[1])[:len(X_scaled)]

    elif selector == "ae":
        recon = model.predict(X_scaled, verbose=0)

    errs = np.mean((recon - X_scaled) ** 2, axis=1)
    # Dynamically set threshold to 99th percentile
    thresh = np.percentile(errs, 99)

    df["anomaly"] = (errs > thresh).astype(int) * -1

    #errs = np.mean((recon - X_scaled) ** 2, axis=1)
    #df["anomaly"] = (errs > thresh).astype(int) * -1

    # ───────── diagnostics ─────────
    print(f"[{selector}] Threshold = {thresh:.4f}")
    print(f"[{selector}] Mean MSE  = {errs.mean():.4f}")
    print(f"[{selector}] Max  MSE  = {errs.max():.4f}")
    print(f"[{selector}] Anomaly % = {(df['anomaly'] == -1).mean() * 100:.2f}%")
    print("\nSample inputs vs reconstructions:")
    for i in range(min(3, len(X_scaled))):
        print(f"  Input {i}: {X_scaled[i]}")
        print(f"  Recon {i}: {recon[i]}\n")

    # ───────── plot reconstruction error ─────────
    plt.figure(figsize=(6, 3))
    plt.hist(errs, bins=50, color="gray")
    plt.axvline(thresh, color="red", linestyle="--", label="Threshold")
    plt.title(f"Reconstruction Error ({selector} - {scenario})")
    plt.xlabel("MSE"); plt.ylabel("Count")
    plt.tight_layout()
    plt.savefig(recon_plot)

    plt.hist(errs, bins=50, alpha=0.7)
    plt.axvline(thresh, color='r', linestyle='--', label='Threshold')
    plt.title(f'MSE Histogram – {selector} ({scenario})')
    plt.xlabel("MSE"); plt.ylabel("Count")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"debug_mse_{selector}_{scenario}.png")

# ───────── reporting ─────────
n_anom = int((df["anomaly"] == -1).sum())
p_anom = n_anom / len(df) * 100
action = "Investigate resource trend" if n_anom else "No action"
atype  = "Resource-trend" if n_anom else "None"

with open(result_txt, "w") as f:
    f.write(f"ANOMALY DETECTED\nCount: {n_anom}\nShare: {p_anom:.2f} %\nType: {atype}\nAI Action: {action}\n" if n_anom
            else "NO ANOMALY\nAI Action: No action\n")

# ───────── plot anomalies ─────────
t, cpu = df["Time"].to_numpy(), df["CPU"].to_numpy()
mask   = df["anomaly"].to_numpy() == -1

plt.switch_backend("Agg")
plt.figure(figsize=(8,3))
plt.plot(t, cpu, lw=0.6, label="CPU %")
plt.scatter(t[mask], cpu[mask], c="red", s=8, label="Anomaly", zorder=5)
plt.xlabel("Time (s)"); plt.ylabel("CPU %")
plt.title(f"CPU Anomalies – {scenario} ({selector})")
plt.legend(); plt.tight_layout()
plt.savefig(plot_path)




# ───────── log ─────────
log_head = "Timestamp,Scenario,Model,AnomalyCount,AnomalyPct,AnomalyType,AI_Action\n"
log_row  = f"{datetime.now().isoformat()},{scenario},{selector},{n_anom},{p_anom:.2f},{atype},{action}\n"
first = not os.path.exists(log_path)

with open(log_path, "a") as f:
    if first: f.write(log_head)
    f.write(log_row)
