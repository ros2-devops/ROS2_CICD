# app.py
import os
import streamlit as st
import pandas as pd
import matplotlib.pyplot as plt
from glob import glob

st.set_page_config(layout="wide")
st.title("ROS 2 AI Simulation Dashboard")

artifact_dir = "dashboard_artifacts"
os.makedirs(artifact_dir, exist_ok=True)

# Helper
def load_text_file(path):
    with open(path, "r") as f:
        return f.read()

# Sidebar: scenario picker
scenarios = sorted({
    f.split("_")[-1].split(".")[0]
    for f in glob(os.path.join(artifact_dir, "*_*.txt")) + 
             glob(os.path.join(artifact_dir, "*_*.md")) + 
             glob(os.path.join(artifact_dir, "*.csv"))
})
selected_scenario = st.sidebar.selectbox("Select Scenario", scenarios)

# Section 1: Simulation Summary
summary_path = os.path.join(artifact_dir, f"evaluation_summary_{selected_scenario}.txt")
log_path     = os.path.join(artifact_dir, f"simulation_log_{selected_scenario}.csv")

st.header("📊 Simulation Summary")
if os.path.exists(summary_path):
    st.code(load_text_file(summary_path), language="text")
else:
    st.warning("No simulation summary found.")

if os.path.exists(log_path):
    df = pd.read_csv(log_path, header=None, names=["Time", "Scenario", "Result"])
    st.dataframe(df.tail(10), use_container_width=True)
else:
    st.warning("No simulation log found.")

# Section 2: Anomaly Detection Results
st.header("🚨 Anomaly Detection Results")

for model in ["iforest", "ae", "cnn_lstm"]:
    path = os.path.join(artifact_dir, f"anomaly_result_log_{model}_{selected_scenario}.csv")
    if os.path.exists(path):
        st.subheader(f"{model.upper()} Anomalies")
        try:
            df = pd.read_csv(path)
            st.dataframe(df.tail(10), use_container_width=True)
        except Exception as e:
            st.error(f"Could not load {model} log: {e}")

        img_path = os.path.join(artifact_dir, f"anomaly_plot_{model}_{selected_scenario}.png")
        if os.path.exists(img_path):
            st.image(img_path, caption=f"{model.upper()} anomaly plot", use_column_width=True)
    else:
        st.info(f"No results for {model.upper()}")

# Section 3: Best Model Summary
st.header("🏆 Best Model Recommendation")
best_model_md = os.path.join(artifact_dir, f"best_model_{selected_scenario}.md")
if os.path.exists(best_model_md):
    with open(best_model_md) as f:
        st.markdown(f.read())
else:
    st.warning("No best model report available.")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)
