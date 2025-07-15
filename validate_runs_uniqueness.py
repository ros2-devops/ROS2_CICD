#!/usr/bin/env python3
import glob, os, pandas as pd, hashlib
from collections import defaultdict

pattern = "data_store/ros_metrics_*_*.csv"
files = sorted(glob.glob(pattern))
scenario_runs = defaultdict(list)

for f in files:
    parts = os.path.basename(f).split("_")
    scenario = parts[2]
    scenario_runs[scenario].append(f)

print("🔍 Semantic duplicate check per scenario (ignores Time, compares rounded core metrics)…")

def hash_df(df):
    df = df.drop(columns=["Time"], errors="ignore")
    df = df.round(3)
    df = df.sort_index(axis=1).reset_index(drop=True)
    return hashlib.md5(pd.util.hash_pandas_object(df, index=False).values).hexdigest()

for scenario, filelist in scenario_runs.items():
    seen_hashes = {}
    dup_count = 0

    for f in filelist:
        try:
            df = pd.read_csv(f)
            h = hash_df(df)
            if h in seen_hashes:
                print(f"  ⚠️ DUPLICATE in {scenario}: {f} == {seen_hashes[h]}")
                dup_count += 1
            else:
                seen_hashes[h] = f
        except Exception as e:
            print(f"  ❌ Failed to read {f}: {e}")

    total = len(filelist)
    print(f"▶ Scenario: {scenario}")
    print(f"   Total runs: {total}")
    print(f"   Duplicates: {dup_count}")
    print(f"   Duplicate rate: {dup_count / total:.2%}\n")
