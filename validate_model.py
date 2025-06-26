import pandas as pd
import joblib
import matplotlib.pyplot as plt
import os

CSV_PATH = "data_store/ros_metrics_all.csv"
MODEL_PATH = "anomaly_model.pkl"

# Load data
df = pd.read_csv(CSV_PATH, header=None, names=["Time", "CPU", "Memory"])

# Convert numeric
df["CPU"] = pd.to_numeric(df["CPU"], errors="coerce")
df["Memory"] = pd.to_numeric(df["Memory"], errors="coerce")

# Drop bad rows
df = df.dropna()
print(f" Loaded {len(df)} rows after cleaning")

# Normalize
df["cpu_norm"] = df["CPU"] / df["CPU"].max()

# Check empty
if df.empty:
    print(" ERROR: DataFrame is empty after cleaning. Cannot evaluate model.")
    exit(1)

# Load model
if not os.path.exists(MODEL_PATH):
    print(f" {MODEL_PATH} not found")
    exit(1)

model = joblib.load(MODEL_PATH)
df["anomaly"] = model.predict(df[["cpu_norm"]])

# Print summary
total = len(df)
anomalies = (df["anomaly"] == -1).sum()
print(f" Model evaluated on {total} samples")
print(f" Anomalies detected: {anomalies} ({100 * anomalies / total:.1f}%)")

# Plot
plt.figure()
plt.plot(df["Time"], df["CPU"], label="CPU %")
plt.scatter(df["Time"][df["anomaly"] == -1],
            df["CPU"][df["anomaly"] == -1],
            color="red", label="Anomaly", zorder=5)
plt.xlabel("Time")
plt.ylabel("CPU (%)")
plt.title("Anomaly Detection – Training Data")
plt.legend()
plt.tight_layout()
plt.savefig("training_anomaly_plot.png")
print(" Saved training_anomaly_plot.png")
