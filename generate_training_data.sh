#!/usr/bin/env bash
set -euo pipefail

# CONFIG ----------------------------------------------------------------
SCENARIOS=(demo1 demo2 arena dynamic maze multibot)  # Add more scenarios if needed
REPEAT=30                                            # Number of simulation runs per scenario
DURATION=90                                          # Duration of each run (seconds)
INTERVAL=0.5                                         # Logging interval (seconds)
STORE_DIR="data_store"
MERGED="$STORE_DIR/ros_metrics_all.csv"
BACKUP_RAW=true                                      # Set to false to skip per-run backup
# -----------------------------------------------------------------------

mkdir -p "$STORE_DIR"
rm -f "$MERGED"
echo "📦 Starting batch collection …"

for sc in "${SCENARIOS[@]}"; do
  for ((i = 1; i <= REPEAT; i++)); do
    echo "▶ $sc run $i/$REPEAT"

    # Run simulation
    SCENARIO=$sc SIM_DURATION=$DURATION LOG_INTERVAL=$INTERVAL \
      ros2 launch sim_demo webots_sim.launch.py --noninteractive

    csv="ros_metrics_${sc}.csv"

    # Check for expected output
    if [[ ! -f "$csv" ]]; then
      echo "⚠️  Warning: $csv not found. Skipping."
      continue
    fi

    # Optionally archive per-run copy
    if [[ "$BACKUP_RAW" == true ]]; then
      cp "$csv" "$STORE_DIR/ros_metrics_${sc}_${i}.csv"
    fi

    # Add scenario/run columns and append to combined dataset
    awk -v scenario="$sc" -v run="$i" 'NR>1 { print $0 "," scenario "," run }' "$csv" >> "$MERGED"
  done
done

# Final header with scenario/run metadata
head -n1 "ros_metrics_${SCENARIOS[0]}.csv" | awk '{print $0 ",Scenario,Run"}' > "$STORE_DIR/tmp_header.csv"
cat "$STORE_DIR/tmp_header.csv" "$MERGED" > "$STORE_DIR/tmp_combined.csv"
mv "$STORE_DIR/tmp_combined.csv" "$MERGED"
rm "$STORE_DIR/tmp_header.csv"

lines=$(wc -l <"$MERGED")
echo "✅ Combined dataset → $MERGED ($lines rows)"
