import os
import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt

st.set_page_config(layout="wide")
st.title("📡 ROS 2 AI Simulation Dashboard")

artifact_dir = "../dashboard_artifacts"
os.makedirs(artifact_dir, exist_ok=True)

# ───── Helper ─────
def load_text_file(path):
    with open(path, "r") as f:
        return f.read()

def load_dataframe(path):
    try:
        return pd.read_csv(path)
    except Exception as e:
        st.error(f"Failed to read {path}: {e}")
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
    st.warning("No simulation summary found.")

if os.path.exists(log_path):
    df_log = pd.read_csv(log_path, header=None, names=["Time", "Scenario", "Result"])
    st.dataframe(df_log.tail(10), use_container_width=True)
else:
    st.info("No simulation log found.")

# ───── Section 2: Anomaly Detection Results ─────
st.header("🚨 Anomaly Detection Results")

for model in ["iforest", "ae", "cnn_lstm"]:
    st.subheader(f"🔎 {model.upper()} Results")
    log_csv = os.path.join(artifact_dir, f"anomaly_result_log_{model}_{selected_scenario}.csv")
    recon_plot = os.path.join(artifact_dir, f"recon_error_{model}_{selected_scenario}.png")
    plot_png = os.path.join(artifact_dir, f"anomaly_plot_{model}_{selected_scenario}.png")
    summary_csv = os.path.join(artifact_dir, f"anomaly_summary_{model}_{selected_scenario}.csv")

    if os.path.exists(log_csv):
        df_log = load_dataframe(log_csv)
        if df_log is not None:
            st.dataframe(df_log[["Time", "CPU", "Memory", "anomaly", "explanation"]].tail(15), use_container_width=True)

    if os.path.exists(summary_csv):
        df_flagged = load_dataframe(summary_csv)
        if df_flagged is not None:
            st.markdown(f"`{len(df_flagged)}` anomalies flagged by `{model}`")
            st.dataframe(df_flagged, use_container_width=True)

    if os.path.exists(plot_png):
        st.image(plot_png, caption=f"{model.upper()} Anomaly Overlay", use_column_width=True)

    if os.path.exists(recon_plot):
        st.image(recon_plot, caption=f"{model.upper()} Reconstruction Error", use_column_width=True)

# ───── Section 3: Best Model Report ─────
st.header("🏆 Best Model Recommendation")
best_model_md = os.path.join(artifact_dir, f"best_model_{selected_scenario}.md")
if os.path.exists(best_model_md):
    st.markdown(load_text_file(best_model_md))
else:
    st.info("No best model report available.")

# ───── Section 4: Overall Evaluation Metrics ─────
st.header("📊 Evaluation Summary (All Models)")
eval_csv = os.path.join(artifact_dir, "evaluation_summary_all_models.csv")
if os.path.exists(eval_csv):
    df_eval = load_dataframe(eval_csv)
    if df_eval is not None:
        st.dataframe(df_eval, use_container_width=True)
else:
    st.info("No global evaluation results available.")
