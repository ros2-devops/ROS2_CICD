#!/usr/bin/env python3
import joblib, pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.ensemble import IsolationForest

CSV = "data_store/ros_metrics_all_clean.csv"
COLS = ["Time", "CPU", "Mem",
        "CPU_roll", "CPU_slope", "Mem_roll", "Mem_slope"]

df = pd.read_csv(CSV, names=COLS)
X  = df[["CPU","Mem","CPU_roll",
         "CPU_slope","Mem_roll","Mem_slope"]]

# train/validation split
X_train, X_val = train_test_split(X, test_size=0.20, random_state=42)

pipe = Pipeline([
    ("scale", StandardScaler()),
    ("if", IsolationForest(
            n_estimators=200,
            contamination=0.02,   # start small (≈2 %)
            random_state=42))
])

pipe.fit(X_train)

val_pred = pipe.predict(X_val)
rate = (val_pred == -1).mean()*100
print(f"Validation anomaly rate: {rate:.2f} % "
      f"({val_pred.sum()}/{len(val_pred)})")

joblib.dump(pipe, "anomaly_model_iforest.pkl")
print("Saved → anomaly_model_iforest.pkl")
