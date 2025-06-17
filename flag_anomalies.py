# flag_anomalies.py
import os
import pandas as pd
import joblib
import matplotlib.pyplot as plt

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
df["CPU_norm"] = df["CPU"] / df["CPU"].max()
X = df[["CPU_norm"]]

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

# Optional: Plot
plt.figure()
plt.plot(df["Time"], df["CPU"], label="CPU %", alpha=0.7)
plt.scatter(df["Time"][df["anomaly"] == -1], df["CPU"][df["anomaly"] == -1],
            color="red", label="Anomaly", zorder=5)
plt.xlabel("Time (s)")
plt.ylabel("CPU Usage (%)")
plt.title("CPU Anomalies Over Time")
plt.legend()
plt.tight_layout()
plt.savefig("anomaly_plot.png")
print("anomaly_plot.png saved")
