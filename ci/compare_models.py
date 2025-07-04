#!/usr/bin/env python3
"""
Select the ‘best’ anomaly-detector model *per scenario*.

Ranking = 1) fewer anomalies  2) deterministic priority
Priority order:  iforest  <  ae  <  cnn_lstm   (0 = best).

Produces
  best_model_<scenario>.txt   (single line machine-readable)
  best_model_<scenario>.md    (pretty Markdown table)
"""

import glob, csv, sys, os
from pathlib import Path
from datetime import datetime

SCENARIO = os.getenv("SCENARIO", "unknown")
logs = sorted(glob.glob(f"anomaly_result_log_*_{SCENARIO}.csv"))
if not logs:
    sys.exit(f"No anomaly logs found for {SCENARIO}")

rows = []
for log in logs:
    model = log.split("_", 3)[2]        # anomaly_result_log_<model>_<scenario>.csv
    with open(log) as f:
        *_, last = list(csv.reader(f))   # last non-header row
    rows.append((model, int(last[3]), float(last[4])))

priority = {"iforest": 0, "ae": 1, "cnn_lstm": 2}
rows.sort(key=lambda r: (r[1], priority.get(r[0], 9)))
best = rows[0]

# ─── write artifacts ────────────────────────────────────────────────
ts = datetime.now().isoformat()
Path(f"best_model_{SCENARIO}.txt").write_text(
    f"{ts}  best={best[0]}  anomalies={best[1]}  pct={best[2]:.2f}\n"
)

md_lines = [
    f"*Model selection for* **`{SCENARIO}`**  — {ts}",
    "", "| Model | # Anoms | % | Rank |", "|---|---:|---:|---:|"
]
for rank, (m,c,p) in enumerate(rows, 1):
    md_lines.append(f"| {m} | {c} | {p:.2f}% | {rank} |")
md_lines += ["", f"**Winner → `{best[0]}`**"]

Path(f"best_model_{SCENARIO}.md").write_text("\n".join(md_lines))
print(f"✔ Best model: {best[0]}  (anoms={best[1]})")
