#!/usr/bin/env bash
set -euo pipefail

# CONFIG ----------------------------------------------------------------
SCENARIOS=(demo1 demo2 arena maze dynamic multibot)
REPEAT=30               # 30 runs per scenario
DURATION=90             # sec
INTERVAL=0.5            # LOG_INTERVAL (higher sample rate)
STORE_DIR="data_store"
MERGED="$STORE_DIR/ros_metrics_all.csv"
# -----------------------------------------------------------------------

mkdir -p "$STORE_DIR"
rm -f "$MERGED"
echo "Starting batch collection …"

for sc in "${SCENARIOS[@]}"; do
  for ((i=1;i<=REPEAT;i++)); do
    echo "▶ $sc  run $i/$REPEAT"
    SCENARIO=$sc SIM_DURATION=$DURATION LOG_INTERVAL=$INTERVAL \
      ros2 launch sim_demo webots_sim.launch.py --noninteractive >/dev/null 2>&1

    csv="ros_metrics_${sc}.csv"
    awk 'NR>1' "$csv" >> "$MERGED"   # append, skip header
  done
done

lines=$(wc -l <"$MERGED")
echo " Combined dataset → $MERGED  ($lines rows)"
