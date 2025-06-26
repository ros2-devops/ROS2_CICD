#!/bin/bash

# CONFIG
SCENARIOS=("demo1" "demo2")
REPEAT=25
DURATION=60
INTERVAL=0.3
STORE_DIR=~/ROS2_CICD/data_store
MERGED_FILE="$STORE_DIR/ros_metrics_cleaned.csv"

mkdir -p "$STORE_DIR"
rm -f "$MERGED_FILE"
echo "Time,CPU,Memory,Scenario,Run" >> "$MERGED_FILE"

echo "Starting simulation batch to collect training data..."

for scenario in "${SCENARIOS[@]}"; do
  for ((i=1; i<=REPEAT; i++)); do
    echo "▶ [$scenario] Run $i/$REPEAT"
    export SCENARIO=$scenario
    export SIM_DURATION=$DURATION
    export LOG_INTERVAL=$INTERVAL

    ros2 launch sim_demo webots_sim.launch.py > /dev/null 2>&1

    csv_file="ros_metrics_${scenario}.csv"
    if [ -f "$csv_file" ]; then
      awk -v s="$scenario" -v i="$i" -F',' 'NF==3 {print $1 "," $2 "," $3 "," s "," i}' "$csv_file" >> "$MERGED_FILE"
    else
      echo "✗ Missing $csv_file"
    fi
  done
done

echo "✅ Training dataset generated: $MERGED_FILE"
echo "🔢 Total lines: $(($(wc -l < "$MERGED_FILE") - 1)) (excluding header)"
