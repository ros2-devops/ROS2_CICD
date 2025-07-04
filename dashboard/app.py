
import streamlit as st, glob, pandas as pd, pathlib, re

st.title("ROS 2 CI • Anomaly Dashboard")

logs = []
for fp in glob.glob("**/anomaly_result_log_*_*.csv", recursive=True):
    df = pd.read_csv(fp)
    model, scen = re.search(r"log_(.+?)_(.+?)\.csv", fp).groups()
    df["model"]     = model
    df["scenario"]  = scen
    logs.append(df)
if not logs:
    st.warning("No logs found – run at least one pipeline.")
    st.stop()

df = pd.concat(logs, ignore_index=True)
st.dataframe(df.tail(200))

st.subheader("Anomaly count by commit")
chart = df.groupby(["scenario","model"])["AnomalyCount"].mean().unstack()
st.line_chart(chart)