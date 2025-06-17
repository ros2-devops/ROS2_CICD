# train_model.py
import pandas as pd
from sklearn.ensemble import IsolationForest
import joblib

# Load data
df = pd.read_csv("datasets/borg_tru_small.csv")

# Keep timestamp and CPU column, drop any NaNs
df = df[['timestamp', 'mean_CPU_usage_rate']].dropna()

# Normalize CPU to 0–1 scale
df['cpu_norm'] = df['mean_CPU_usage_rate'] / df['mean_CPU_usage_rate'].max()

# Isolation Forest expects 2D input
X = df[['cpu_norm']]

# Train Isolation Forest
model = IsolationForest(contamination=0.05, random_state=42)
model.fit(X)

# Save trained model
joblib.dump(model, "anomaly_model.pkl")
print("✅ Model trained and saved as anomaly_model.pkl")
