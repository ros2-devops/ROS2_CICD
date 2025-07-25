# compare_feature_distributions.py
import pandas as pd

train_df = pd.read_csv("data_training_full/ros_metrics_relabelled.csv")
inference_df = pd.read_csv("ros_metrics_demo2.csv")

train_stats = train_df.describe().T[["mean", "std", "min", "max"]]
inference_stats = inference_df.describe().T[["mean", "std", "min", "max"]]

summary = train_stats.join(inference_stats, lsuffix="_train", rsuffix="_inference")
summary.to_csv("feature_distribution_shift.csv")
print(summary)
