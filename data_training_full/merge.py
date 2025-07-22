#!/usr/bin/env python3

import pandas as pd
import glob
import os
import re

# ─── Settings ─────────────────────────────────────────────
INPUT_GLOB = "ros_metrics_*_*.csv"
OUTPUT_CSV = "ros_metrics_training_full.csv"

# ─── Gather all metric files ──────────────────────────────
files = sorted(glob.glob(INPUT_GLOB))
if not files:
    raise FileNotFoundError("No matching ros_metrics_*_*.csv files found.")

all_rows = []
expected_cols = {"CPU_viol", "Mem_viol"}

# ─── Process each CSV ─────────────────────────────────────
for filepath in files:
    filename = os.path.basename(filepath)
    match = re.match(r"ros_metrics_(.+)_(\d+)\.csv", filename)
    if not match:
        print(f"❌ Skipping malformed filename: {filename}")
        continue

    scenario = match[1]
    run_id = int(match[2])

    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"❌ Failed to read {filename}: {e}")
        continue

    if not expected_cols.issubset(df.columns):
        print(f"⚠️  Skipping {filename}: missing required columns")
        continue

    df["Scenario"] = scenario
    df["Run"] = run_id
    df["Anomaly"] = ((df["CPU_viol"] > 0) | (df["Mem_viol"] > 0)).astype(int)

    all_rows.append(df)

# ─── Combine and Save ─────────────────────────────────────
if not all_rows:
    raise RuntimeError("No valid CSV files with expected columns found.")

full_df = pd.concat(all_rows, ignore_index=True)
full_df.to_csv(OUTPUT_CSV, index=False)

print(f"✅ Combined {len(all_rows)} files into {OUTPUT_CSV}")
print(f"   Total rows: {len(full_df)}")
print(f"   Features: {list(full_df.columns)}")
