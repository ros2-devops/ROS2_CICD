#!/usr/bin/env python3
"""
prepare_dataset.py  – clean & enrich raw metrics
Usage: python3 prepare_dataset.py data_store/ros_metrics_all.csv
Result: data_store/features.csv
"""

import sys, os, pandas as pd

if len(sys.argv)!=2:
    print("Usage: prepare_dataset.py <raw_csv>"); sys.exit(1)

raw   = sys.argv[1]
clean = os.path.join(os.path.dirname(raw), "features.csv")

cols  = ["Time","CPU","Memory","CPU_roll","CPU_slope","Mem_roll","Mem_slope"]
df    = pd.read_csv(raw, names=cols)

# drop bad / duplicates
df = df.dropna().drop_duplicates()

# min-max normalise level features
for c in ["CPU","Memory","CPU_roll","Mem_roll"]:
    df[f"{c}_norm"] = (df[c]-df[c].min())/(df[c].max()-df[c].min())

df.to_csv(clean, index=False)
print(" cleaned →", clean, len(df), "rows")
