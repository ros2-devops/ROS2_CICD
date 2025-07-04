# dashboards/anomaly_dashboard.py
# ──────────────────────────────
"""
Streamlit Dashboard – Dissertation Edition
• interactive filters + KPIs
• timeline & small-multiples for anomalies
Run locally:  streamlit run dashboards/anomaly_dashboard.py
"""

import re, glob, pathlib, datetime as dt
import pandas as pd
import streamlit as st
import plotly.express as px

DATA_GLOB = "**/anomaly_result_log_*_*.csv"  # works from repo root or artefact folder
THEME_COL  = "#4F8BF9"

st.set_page_config(page_title="ROS 2 CI ▸ Anomaly Dashboard",
                   layout="wide", page_icon="📊")

# ╭──────────────────────────────────────────────────────────────╮
# | 1) Data ingest & caching                                     |
# ╰──────────────────────────────────────────────────────────────╯
@st.cache_data(show_spinner="Loading anomaly logs …")
def load_logs() -> pd.DataFrame:
    records = []
    for fp in glob.glob(DATA_GLOB, recursive=True):
        m = re.search(r"log_(.+?)_(.+?)\.csv$", fp)
        if not m: continue
        model, scenario = m.groups()
        df = pd.read_csv(fp)
        df["Model"] = model
        df["Scenario"] = scenario
        # if Timestamp col absent -> inject dummy
        if "Timestamp" not in df.columns:
            df["Timestamp"] = pathlib.Path(fp).stat().st_mtime
        records.append(df)
    if not records:
        return pd.DataFrame()
    df = pd.concat(records, ignore_index=True)
    df["Timestamp"] = pd.to_datetime(df["Timestamp"])
    return df.sort_values("Timestamp")

df = load_logs()
if df.empty:
    st.warning("No anomaly logs found – run at least one CI pipeline.")
    st.stop()

all_scen = sorted(df["Scenario"].unique())
all_mod  = sorted(df["Model"].unique())

# ╭───────────────────────── Sidebar filters ────────────────────╮
with st.sidebar:
    st.header("🔍 Filter")
    scen_sel = st.multiselect("Scenario", all_scen, default=all_scen)
    mod_sel  = st.multiselect("Model", all_mod,  default=all_mod)
    min_date, max_date = df["Timestamp"].min(), df["Timestamp"].max()
    drange = st.date_input("Date range",
                           value=(min_date.date(), max_date.date()))
    dr_start = pd.to_datetime(drange[0]); dr_end = pd.to_datetime(drange[1])+pd.Timedelta(days=1)

mask = (df["Scenario"].isin(scen_sel) & df["Model"].isin(mod_sel) &
        (df["Timestamp"]>=dr_start) & (df["Timestamp"]<dr_end))
df_f = df.loc[mask]

st.title("📊 ROS 2 CI • Anomaly Dashboard")

# ╭──────────────────────────────────────────────────────────────╮
# | 2) KPI cards                                                |
# ╰──────────────────────────────────────────────────────────────╯
def kpi(label:str, value, help_txt:str=""):
    st.metric(label, value, help=help_txt)

col1, col2, col3, col4 = st.columns(4)
kpi("Runs analysed", len(df_f), "Rows in anomaly log")
kpi("Mean anomalies / run", f"{df_f['AnomalyCount'].mean():.2f}")
best_row = (df_f.sort_values(["AnomalyCount","Model"])
                 .iloc[0][["Model","AnomalyCount"]])
kpi("📌 Best model", f"{best_row['Model']} ({best_row['AnomalyCount']})")
kpi("Latest log date", df_f["Timestamp"].max().strftime("%Y-%m-%d"))

# ╭──────────────────────── Tabs layout ─────────────────────────╮
tab_over, tab_time, tab_table = st.tabs(["Overview 📈", "Timeline ⏱️", "Data Table 📄"])

# ── Overview (bar chart) ───────────────────────────────────────
with tab_over:
    st.subheader("Average anomaly count per model")
    bar = (df_f.groupby("Model")["AnomalyCount"].mean()
                 .sort_values().reset_index())
    st.plotly_chart(px.bar(bar, x="Model", y="AnomalyCount",
                           color="Model", color_discrete_sequence=px.colors.qualitative.Set2),
                    use_container_width=True)

# ── Timeline – one line per model ──────────────────────────────
with tab_time:
    st.subheader("Anomaly trend over time (per commit / run)")
    time_df = df_f.copy()
    time_df["Run"]=time_df.groupby(["Scenario","Model"]).cumcount()+1
    fig = px.line(time_df, x="Timestamp", y="AnomalyCount",
                  color="Model", line_group="Model",
                  symbol="Scenario", markers=True,
                  title=None, hover_data=["Scenario"])
    fig.update_layout(height=450, legend_title=None,
                      margin=dict(l=10,r=10,t=10,b=40))
    st.plotly_chart(fig, use_container_width=True)

# ── Detailed table ─────────────────────────────────────────────
with tab_table:
    st.subheader("Raw anomaly logs")
    st.dataframe(df_f.reset_index(drop=True), use_container_width=True)
    csv = df_f.to_csv(index=False).encode()
    st.download_button("Download CSV", csv,
                       file_name="anomaly_logs_filtered.csv", mime="text/csv")
