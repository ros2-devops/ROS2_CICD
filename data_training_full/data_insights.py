import pandas as pd
import numpy as np
from scipy.stats import zscore

# ─── Configuration ───
INPUT_CSV = "ros_metrics_training_full.csv"
OUTPUT_CSV = "ros_metrics_relabelled.csv"
CPU_VIOL_THRESHOLD = 10
MEM_VIOL_THRESHOLD = 10

print(f"📂 Loading: {INPUT_CSV}")
df = pd.read_csv(OUTPUT_CSV)



# ─── Quality Check ───
print("\n📊 Label distribution:")
print(df["Anomaly"].value_counts())

print("\n🧪 Missing values:")
print(df.isna().sum())

print("\n📉 Outlier summary (|z| > 3):")
numeric_cols = df.select_dtypes(include=np.number).columns.tolist()
# Fixed outlier detection
z_scores = np.abs(zscore(df[numeric_cols]))
outlier_counts = pd.Series((z_scores > 3).sum(axis=0), index=numeric_cols)
print(outlier_counts[outlier_counts > 0].sort_values(ascending=False))
