#!/usr/bin/env python3
import glob, csv, sys, os, json
from pathlib import Path

SCENARIO = os.getenv("SCENARIO", "unknown")

rows = []
for log in glob.glob(f"anomaly_result_log_*_{SCENARIO}.csv"):
    model = log.replace("anomaly_result_log_", "").replace(f"_{SCENARIO}.csv", "")
    with open(log) as f:
        last = list(csv.reader(f))[-1]
    try:
        anom_count = int(float(last[3]))
        anom_pct = float(last[4])
    except ValueError:
        # fallback: look for matching "AnomalyCount" line
        for line in reversed(lines):
            if line.strip().startswith("Timestamp"):
                continue
            parts = line.strip().split(",")
            if len(parts) >= 5:
                try:
                    anom_count = int(float(parts[3]))
                    anom_pct = float(parts[4])
                    break
                except:
                    pass
    rows.append((model, anom_count, anom_pct))



if not rows:
    sys.exit("No logs found!")

priority = {"iforest": 0, "ae": 1, "cnn_lstm": 2}
rows.sort(key=lambda r: (r[1], priority[r[0]]))
best = rows[0]

Path(f"best_model_{SCENARIO}.txt").write_text(
    f"best={best[0]}  anomalies={best[1]} ({best[2]} %)\n")

Path(f"best_model_{SCENARIO}.md").write_text(
    "|Model|Anoms|Pct|\n|---|---|---|\n" +
    "\n".join(f"|{m}|{c}|{p:.2f} %|" for m,c,p in rows) +
    f"\n\n**Best → `{best[0]}`**\n")

Path(f"compare_models_{SCENARIO}.json").write_text(json.dumps({
    "scenario": SCENARIO,
    "best_model": best[0],
    "scores": [{"model": m, "anoms": c, "pct": round(p, 2)} for m,c,p in rows]
}, indent=2))

print(f"Best model: {best[0]} with {best[1]} anomalies ({best[2]:.2f}%)")
