#!/usr/bin/env bash
set -euo pipefail

export LC_ALL=C.UTF-8
export LANG=C.UTF-8
export RMW_FASTRTPS_USE_SHM=0
export QT_QPA_PLATFORM=offscreen
# ───── Configuration ─────
SCENARIOS=(demo2 multibot demo1 maze dynamic arena)
REPEAT=60
DURATION=90
INTERVAL=0.5
STORE_DIR="data_training_new"
CLEAN_FILE="$STORE_DIR/ros_metrics_clean_training.csv"
LABELED_FILE="$STORE_DIR/ros_metrics_with_labels.csv"
TMP_CLEAN="tmp_clean.csv"
TMP_LABEL="tmp_label.csv"
BACKUP_RAW=true

# ───── Setup ─────
mkdir -p "$STORE_DIR"
rm -f "$CLEAN_FILE" "$LABELED_FILE" "$TMP_CLEAN" "$TMP_LABEL"
echo "📦 Starting full training data generation…"

declare -A row_counts

# ───── Data Collection ─────
for sc in "${SCENARIOS[@]}"; do
  for ((i = 1; i <= REPEAT; i++)); do
    echo "▶ Running $sc [$i/$REPEAT]"

    rm -f "ros_metrics_${sc}.csv"

    export QT_QPA_PLATFORM=offscreen
    xvfb-run -a webots --stdout --batch --no-rendering "ros2_ws/src/sim_demo/worlds/${sc}.wbt" > webots_output.log 2>&1 &
    WEBOTS_PID=$!

    sleep 2  # Allow Webots to initialize

    (
      set +eu
      source /opt/ros/humble/setup.bash
      source ros2_ws/install/setup.bash

      # Launch metrics_collector in background
      SCENARIO=${sc} SIM_DURATION=${DURATION} LOG_INTERVAL=${INTERVAL} \
        ros2 run ros2_observability metrics_collector &
      COLLECTOR_PID=$!

      SECONDS=0
      while kill -0 $COLLECTOR_PID 2>/dev/null; do
        sleep 2
        if [[ "$SECONDS" -gt 95 ]]; then
          echo "⏱️ Timeout hit for $sc [$i/$REPEAT]"
          # Force kill Webots after timeout or normal completion
          kill -TERM $WEBOTS_PID 2>/dev/null || true
          sleep 2
          kill -KILL $WEBOTS_PID 2>/dev/null || true
          wait $WEBOTS_PID 2>/dev/null || true

          # Clean stray Webots/Xvfb processes just in case
          pkill -f webots 2>/dev/null || true
          pkill -f Xvfb 2>/dev/null || true

          kill -TERM $COLLECTOR_PID
          wait $COLLECTOR_PID || true
          break
        fi
      done

      wait $COLLECTOR_PID
      COLLECTOR_STATUS=$?

      if [[ $COLLECTOR_STATUS -ne 0 && $COLLECTOR_STATUS -ne 143 ]]; then
        echo "❌ metrics_collector failed with code $COLLECTOR_STATUS for $sc [$i/$REPEAT]"
      fi

    )


    csv="ros_metrics_${sc}.csv"

    if [[ ! -f "$csv" ]]; then
      echo "⚠️  Missing output: $csv"
      continue
    fi

    if [[ "$BACKUP_RAW" == true ]]; then
      cp "$csv" "$STORE_DIR/ros_metrics_${sc}_${i}.csv"
    fi

    # Extract clean rows with scenario tag
    awk -v s="$sc" 'BEGIN{FS=OFS=","} NR==1 {print $0,"Scenario"} NR>1 && $8==0 && $9==0 {print $0,s}' "$csv" >> "$TMP_CLEAN"

    # Full labeled version
    awk -v s="$sc" 'BEGIN{FS=OFS=","}
      NR==1 {print $0,"Anomaly","Scenario"}
      NR>1 {
        label = ($8 > 0 || $9 > 0 || s ~ /demo/) ? 1 : 0;
        print $0, label, s
      }' "$csv" >> "$TMP_LABEL"

    count=$(wc -l < "$csv")
    row_counts["$sc"]=$(( ${row_counts["$sc"]:-0} + count ))
  done
done

# ───── Final Output ─────
head -n1 "$TMP_CLEAN" > "$CLEAN_FILE"
tail -n +2 "$TMP_CLEAN" | sort | uniq >> "$CLEAN_FILE"

head -n1 "$TMP_LABEL" > "$LABELED_FILE"
tail -n +2 "$TMP_LABEL" | sort | uniq >> "$LABELED_FILE"

# ───── Summary ─────
echo -e "\n📊 Distribution Summary:"
for sc in "${SCENARIOS[@]}"; do
  echo "  $sc: ${row_counts[$sc]:-0} rows"
done

echo -e "\n✅ Clean data → $CLEAN_FILE"
echo "✅ Full labeled data → $LABELED_FILE"
wc -l "$CLEAN_FILE" "$LABELED_FILE"
