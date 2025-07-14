# app.py
import os
import re
import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
from glob import glob

st.set_page_config(layout="wide")
st.title("ROS 2 AI Simulation Dashboard")

artifact_dir = "../dashboard_artifacts"
os.makedirs(artifact_dir, exist_ok=True)

def load_text_file(path):
    with open(path, "r") as f:
        return f.read()

# ───────── Sidebar: Scenario & Model Picker ─────────
scenarios = sorted([
    f.replace("evaluation_summary_", "").replace(".txt", "")
    for f in os.listdir(artifact_dir)
    if f.startswith("evaluation_summary_") and f.endswith(".txt")
])
selected_scenario = st.sidebar.selectbox("Select Scenario", scenarios)

models_available = ["iforest", "ae", "cnn_lstm"]
selected_models = st.sidebar.multiselect("Select Models", models_available, default=models_available)

# ───────── Section 1: Simulation Summary ─────────
st.header("📊 Simulation Summary")

summary_path = os.path.join(artifact_dir, f"evaluation_summary_{selected_scenario}.txt")
log_path = os.path.join(artifact_dir, f"simulation_log_{selected_scenario}.csv")
if os.path.exists(summary_path):
    st.code(load_text_file(summary_path), language="text")
if os.path.exists(log_path):
    df_log = pd.read_csv(log_path, header=None, names=["Time", "Scenario", "Result"])
    st.dataframe(df_log.tail(10), use_container_width=True)

# ───────── Section 2: Per-Model Anomaly Viewer ─────────
st.header("🚨 Anomaly Detection Viewer")

for model in selected_models:
    log_file = os.path.join(artifact_dir, f"anomaly_result_log_{model}_{selected_scenario}.csv")
    plot_file = os.path.join(artifact_dir, f"anomaly_plot_{model}_{selected_scenario}.png")
    recon_plot = os.path.join(artifact_dir, f"recon_error_{model}_{selected_scenario}.png")

    if os.path.exists(log_file):
        df = pd.read_csv(log_file)
        st.subheader(f"🔍 Model: {model.upper()}")

        # Summary
        total = len(df)
        flagged = df["anomaly"].sum()
        st.markdown(f"**{flagged} anomalies out of {total} data points**")
        if flagged > 0:
            st.markdown(f"**Flagging rate:** {100*flagged/total:.2f}%")

        # Optional filters
        show_expl = st.checkbox(f"Show flagged rows with explanations for `{model}`", key=model)
        if show_expl:
            st.dataframe(df[df["anomaly"]][["Time", "CPU", "Memory", "explanation"]], use_container_width=True)

        # Time-series plot
        st.image(plot_file, caption=f"{model.upper()} Anomaly Plot")

        if os.path.exists(recon_plot):
            st.image(recon_plot, caption=f"{model.upper()} Reconstruction Error Plot")
    else:
        st.info(f"No anomaly log found for `{model}`.")

# ───────── Section 3: Best Model Summary ─────────
st.header("🏆 Best Model Recommendation")
best_model_path = os.path.join(artifact_dir, f"best_model_{selected_scenario}.md")
if os.path.exists(best_model_path):
    st.markdown(load_text_file(best_model_path))
else:
    st.warning("No best model recommendation available.")

# ───────── Section 4: Flagged Anomaly Summary CSVs ─────────
st.header("📍 Flagged Data Summary")

for model in selected_models:
    summary_path = os.path.join(artifact_dir, f"anomaly_summary_{model}_{selected_scenario}.csv")
    if os.path.exists(summary_path):
        df_flagged = pd.read_csv(summary_path)
        st.subheader(f"Flagged Summary – `{model}`")
        st.dataframe(df_flagged, use_container_width=True)
    else:
        st.info(f"No flagged summary for `{model}`.")
