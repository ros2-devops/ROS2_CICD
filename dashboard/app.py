#!/usr/bin/env python3
import os
import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt

st.set_page_config(layout="wide")
st.title("📡 ROS 2 AI Simulation Dashboard")

artifact_dir = "../dashboard_artifacts"
os.makedirs(artifact_dir, exist_ok=True)

# ───── Helpers ─────
def load_text_file(path):
    try:
        with open(path, "r") as f:
            return f.read()
    except Exception as e:
        return f"❌ Failed to load {path}: {e}"

def load_dataframe(path):
    try:
        return pd.read_csv(path)
    except Exception as e:
        st.error(f"❌ Failed to read {path}: {e}")
        return None

# ───── Sidebar: Scenario Picker ─────
scenarios = sorted([
    f.replace("evaluation_summary_", "").replace(".txt", "")
    for f in os.listdir(artifact_dir)
    if f.startswith("evaluation_summary_") and f.endswith(".txt")
])
selected_scenario = st.sidebar.selectbox("🧪 Select Scenario", scenarios)

# ───── Section 1: Simulation Summary ─────
st.header("🧾 Simulation Summary")
summary_path = os.path.join(artifact_dir, f"evaluation_summary_{selected_scenario}.txt")
log_path     = os.path.join(artifact_dir, f"simulation_log_{selected_scenario}.csv")

if os.path.exists(summary_path):
    st.code(load_text_file(summary_path), language="text")
else:
    st.warning("⚠️ No simulation summary found.")

if os.path.exists(log_path):
    df_log = pd.read_csv(log_path)
    st.subheader("📜 Recent Simulation Logs")
    st.dataframe(df_log.tail(10), use_container_width=True)
else:
    st.info("ℹ️ No simulation log file available.")

# ───── Section 2: Anomaly Detection Results ─────
st.header("🚨 Anomaly Detection Results")

for model in ["iforest", "ae", "cnn_lstm"]:
    st.subheader(f"🔎 {model.upper()} Results")

    log_csv = os.path.join(artifact_dir, f"anomaly_result_log_{model}_{selected_scenario}.csv")
    recon_plot = os.path.join(artifact_dir, f"recon_error_{model}_{selected_scenario}.png")
    plot_png = os.path.join(artifact_dir, f"anomaly_plot_{model}_{selected_scenario}.png")

    # ─ Logs
    if os.path.exists(log_csv):
        df_anom = load_dataframe(log_csv)
        if df_anom is not None and "anomaly" in df_anom.columns:
            count = df_anom["anomaly"].sum()
            st.markdown(f"🔴 **{count} anomalies flagged** by `{model}`")
            st.dataframe(df_anom[["TimeIndex", "CPU", "Memory", "anomaly", "explanation"]].tail(15), use_container_width=True)
        else:
            st.warning("No valid anomaly log data.")

    # ─ Score Plots
    if os.path.exists(recon_plot):
        st.image(recon_plot, caption=f"{model.upper()} Score or Reconstruction Error", use_column_width=True)
    if os.path.exists(plot_png):
        st.image(plot_png, caption=f"{model.upper()} Anomaly Overlay", use_column_width=True)

# ───── Section 3: Best Model Summary ─────
st.header("🏆 Best Model Recommendation")
best_model_md = os.path.join(artifact_dir, f"best_model_{selected_scenario}.md")
if os.path.exists(best_model_md):
    st.markdown(load_text_file(best_model_md))
else:
    st.info("❓ No best model markdown file found.")

# ───── Section 4: Evaluation Summary Table ─────
st.header("📊 Evaluation Summary (All Models)")
eval_log = os.path.join(artifact_dir, "evaluation_log.csv")
if os.path.exists(eval_log):
    df_eval = load_dataframe(eval_log)
    if df_eval is not None:
        filtered = df_eval[df_eval["Scenario"] == selected_scenario]
        st.dataframe(filtered.sort_values("Timestamp", ascending=False), use_container_width=True)
else:
    st.info("🔎 No evaluation log found.")
