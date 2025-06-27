# clean_mixed_csv.sh ─ keeps only rows with exactly 7 commas (= 7 columns)
in=data_store/ros_metrics_all.csv
out=data_store/ros_metrics_all_clean.csv
awk -F',' 'NF==7' "$in" > "$out"
mv "$out" "$in"
echo "Cleaned → $in ( $(wc -l <"$in") rows remain )"
