#!/usr/bin/env python3
import pandas as pd
import glob

files = glob.glob("ros_metrics_*_*.csv")
expected_cols = ["Time", "CPU", "Memory", "CPU_roll", "CPU_slope",
                 "Mem_roll", "Mem_slope", "CPU_viol", "Mem_viol"]

merged = []

for f in files:
    try:
        df = pd.read_csv(f)
        if not all(col in df.columns for col in expected_cols):
            print(f"[⚠️] Skipping {f}: missing columns")
            continue
        merged.append(df)
        print(f"[✓] Loaded {f} ({len(df)} rows)")
    except Exception as e:
        print(f"[❌] Failed to read {f}: {e}")

if merged:
    out_df = pd.concat(merged, ignore_index=True)
    out_df.to_csv("ros_metrics_merged_full.csv", index=False)
    print(f"\n✅ Merged {len(merged)} files → ros_metrics_merged_full.csv ({len(out_df)} total rows)")
else:
    print("🚫 No valid files found with required columns.")
