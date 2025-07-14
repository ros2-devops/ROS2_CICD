#!/usr/bin/env python3
import pandas as pd
import os
import glob

# ───── Config ─────
artifact_dir = "dashboard_artifacts"
merged_data = "data_store/ros_metrics_merged_full.csv"
results = []

# ───── Load Ground Truth ─────
if not os.path.exists(merged_data):
    raise FileNotFoundError(f"Ground truth file not found: {merged_data}")

df_truth = pd.read_csv(merged_data)
df_truth["true_anomaly"] = ((df_truth["CPU_viol"] > 0) | (df_truth["Mem_viol"] > 0)).astype(int)
print(f"[INFO] Ground truth loaded: {len(df_truth)} rows")

# ───── Evaluate Each Model ─────
for model in ["iforest", "ae", "cnn_lstm"]:
    pattern = os.path.join(artifact_dir, f"anomaly_result_log_{model}_*.csv")
    for file in glob.glob(pattern):
        scenario = file.split(f"anomaly_result_log_{model}_")[1].replace(".csv", "")
        try:
            df_pred = pd.read_csv(file)
        except Exception as e:
            print(f"[❌] Could not load {file}: {e}")
            continue

        print(f"[…] Evaluating {file} (rows={len(df_pred)})")

        # Check alignment
        if len(df_pred) != len(df_truth):
            print(f"[⚠️] Skipping {file}: length mismatch (pred={len(df_pred)} vs truth={len(df_truth)})")
            continue

        # Evaluate
        df_eval = df_pred.copy()
        df_eval["true_anomaly"] = df_truth["true_anomaly"].values

        tp = ((df_eval["anomaly"] == 1) & (df_eval["true_anomaly"] == 1)).sum()
        fp = ((df_eval["anomaly"] == 1) & (df_eval["true_anomaly"] == 0)).sum()
        tn = ((df_eval["anomaly"] == 0) & (df_eval["true_anomaly"] == 0)).sum()
        fn = ((df_eval["anomaly"] == 0) & (df_eval["true_anomaly"] == 1)).sum()

        precision = tp / (tp + fp) if tp + fp > 0 else 0.0
        recall    = tp / (tp + fn) if tp + fn > 0 else 0.0
        f1        = 2 * (precision * recall) / (precision + recall) if precision + recall > 0 else 0.0

        results.append({
            "Model": model,
            "Scenario": scenario,
            "TP": tp, "FP": fp, "TN": tn, "FN": fn,
            "Precision": round(precision, 3),
            "Recall": round(recall, 3),
            "F1": round(f1, 3)
        })

# ───── Output Results ─────
if results:
    df_results = pd.DataFrame(results)
    df_results.to_csv(f"{artifact_dir}/evaluation_summary_all_models.csv", index=False)
    print(f"\n✅ Evaluation complete → {artifact_dir}/evaluation_summary_all_models.csv")
else:
    print("🚫 No results to write. All evaluations skipped due to length mismatch.")
