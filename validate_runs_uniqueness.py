#!/usr/bin/env python3
import glob, os, pandas as pd
from collections import defaultdict

pattern = "data_store/ros_metrics_*_*.csv"
files = sorted(glob.glob(pattern))
scenario_runs = defaultdict(list)

for f in files:
    parts = os.path.basename(f).split("_")
    scenario = parts[2]
    scenario_runs[scenario].append(f)

print("🔍 Debugging duplicate check (row-by-row, value-by-value, no Time)...\n")

def load_data(path):
    try:
        df = pd.read_csv(path).drop(columns=["Time"], errors="ignore")
        values = df.round(3).values.tolist()
        print(f"  📄 Loaded {path}: {len(values)} rows, shape {df.shape}")
        return values
    except Exception as e:
        print(f"  ❌ Failed to load {path}: {e}")
        return None

for scenario, filelist in scenario_runs.items():
    print(f"\n▶ Scenario: {scenario}")
    seen = []
    dups = 0

    for idx, f in enumerate(filelist):
        rows = load_data(f)
        if rows is None:
            continue

        found_duplicate = False

        for j, prev in enumerate(seen):
            if len(rows) != len(prev):
                print(f"     ⚠️ Row count mismatch: {f} vs previous file {filelist[j]}")
                continue

            all_match = True
            for r1, r2 in zip(rows, prev):
                if r1 != r2:
                    all_match = False
                    break

            if all_match:
                print(f"  ⚠️ DUPLICATE: {f} is identical to {filelist[j]}")
                dups += 1
                found_duplicate = True
                break

        if not found_duplicate:
            seen.append(rows)

    total = len(filelist)
    print(f"  Total runs: {total}")
    print(f"  Duplicates: {dups}")
    print(f"  Duplicate rate: {dups / total:.2%}")
