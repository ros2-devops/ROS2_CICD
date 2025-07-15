#!/usr/bin/env python3
import pandas as pd
import os
import glob

# ───── Config ─────
artifact_dir = "dashboard_artifacts"
results = []

# ───── Evaluate Each Model's Output ─────
for model in ["iforest", "ae", "cnn_lstm"]:
    pattern = os.path.join(artifact_dir, f"anomaly_result_log_{model}_*.csv")
    for pred_file in glob.glob(pattern):
        scenario = pred_file.split(f"anomaly_result_log_{model}_")[1].replace(".csv", "")
        input_file = os.path.join(artifact_dir, f"ros_metrics_input_{scenario}.csv")

        print(f"[…] Evaluating {pred_file}")
        if not os.path.exists(input_file):
            print(f"[⚠️] Skipping {pred_file}: input file {input_file} not found")
            continue

        try:
            df_pred = pd.read_csv(pred_file)
            df_truth = pd.read_csv(input_file)
        except Exception as e:
            print(f"[❌] Failed to load files for {scenario}: {e}")
            continue

        if len(df_pred) != len(df_truth):
            print(f"[⚠️] Skipping {pred_file}: length mismatch (pred={len(df_pred)} vs truth={len(df_truth)})")
            continue

        df_truth["true_anomaly"] = ((df_truth["CPU_viol"] > 0) | (df_truth["Mem_viol"] > 0)).astype(int)
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
    out_path = os.path.join(artifact_dir, "evaluation_summary_all_models.csv")
    df_results.to_csv(out_path, index=False)
    print(f"✅ Evaluation complete → {out_path}")
else:
    print("🚫 No results to write. All evaluations skipped.")
