#!/usr/bin/env python3
"""
Pick the best detector for one SCENARIO.
Rules (keep it simple first):
    •   lowest AnomalyCount wins
    •   tie → prefer: iforest  >  ae  >  cnn_lstm
Outputs
    best_model_<scenario>.txt   (one-liner)
    best_model_<scenario>.md    (pretty table for dashboard / summary)
"""

import glob, csv, sys, os, json
from pathlib import Path

SCENARIO = os.getenv("SCENARIO", "unknown")

rows = []  # [(model, count, pct)]
for log in glob.glob(f"anomaly_result_log_*_{SCENARIO}.csv"):
    model = log.split("_")[3]        # anomaly_result_log_<model>_<scen>.csv
    with open(log) as f:
        *_, last = csv.reader(f).readlines()   # last appended row
    parts = last.strip().split(",")
    rows.append((model, int(parts[3]), float(parts[4])))

if not rows:
    sys.exit("No logs found!")

# sort by count, then our manual priority
priority = {"iforest": 0, "ae": 1, "cnn_lstm": 2}
rows.sort(key=lambda r: (r[1], priority[r[0]]))
best = rows[0]

# ---------- write artifacts ----------
txt = f"best={best[0]}  anomalies={best[1]} ({best[2]} %)\n"
Path(f"best_model_{SCENARIO}.txt").write_text(txt)

md  = "|Model|Anoms|Pct|\n|---|---|---|\n" + \
      "\n".join(f"|{m}|{c}|{p:.2f} %|" for m,c,p in rows) + "\n" + \
      f"\n**Best → `{best[0]}`**\n"
Path(f"best_model_{SCENARIO}.md").write_text(md)

print(txt.strip())
