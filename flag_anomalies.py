# flag_anomalies.py
import os
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import numpy as np

# Paths
ros_metrics_path = "ros_metrics.csv"
model_path = "anomaly_model.pkl"
result_path = "anomaly_result.txt"

# Load runtime metrics
if not os.path.exists(ros_metrics_path):
    print("ros_metrics.csv not found")
    exit(1)

df = pd.read_csv(ros_metrics_path, header=None, names=["Time", "CPU", "Memory"])

# Preprocess: normalize CPU only (match training)
df["cpu_norm"] = df["CPU"] / df["CPU"].max()
X = df[["cpu_norm"]]

# Load model
if not os.path.exists(model_path):
    print("Trained model not found (anomaly_model.pkl)")
    exit(1)

model = joblib.load(model_path)

# Predict: -1 = anomaly, 1 = normal
df["anomaly"] = model.predict(X)

# Count anomalies
num_anomalies = (df["anomaly"] == -1).sum()

# Write result file
with open(result_path, "w") as f:
    if num_anomalies > 0:
        f.write(f"ANOMALY DETECTED\n")
        f.write(f"Count: {num_anomalies} rows\n")
    else:
        f.write("NO ANOMALY\n")

print("anomaly_result.txt written")



#  Ensure time is numeric
df["Time"] = pd.to_numeric(df["Time"], errors="coerce")
df = df.dropna(subset=["Time", "CPU", "anomaly"])

# Convert to numpy explicitly (prevents Series indexing issues)
times = df["Time"].to_numpy()
cpu = df["CPU"].to_numpy()
anomalies = df["anomaly"].to_numpy()

#  Plot
plt.figure()
plt.plot(times, cpu, label="CPU %", alpha=0.7)
plt.scatter(times[anomalies == -1], cpu[anomalies == -1],
            color="red", label="Anomaly", zorder=5)
plt.xlabel("Time (s)")
plt.ylabel("CPU Usage (%)")
plt.title("CPU Anomalies Over Time")
plt.legend()
plt.tight_layout()
# Save plot
plt.savefig(f"anomaly_plot_{scenario}.png")
print(f"anomaly_plot_{scenario}.png saved")

# Scenario and timestamp for structured log
from datetime import datetime
timestamp = datetime.now().isoformat()
scenario = os.getenv("SCENARIO", "unknown")
log_file = f"anomaly_log_{scenario}.csv"

# Structured logging per anomaly row
df["Timestamp"] = timestamp
df["Scenario"] = scenario
df["AnomalyScore"] = (df["anomaly"] == -1).astype(int)

# Create/append structured log
header = not os.path.exists(log_file)
df[["Timestamp", "Scenario", "CPU", "Memory", "AnomalyScore"]].to_csv(
    log_file, mode='a', header=header, index=False
)
print(f"Structured anomaly log written to: {log_file}")
