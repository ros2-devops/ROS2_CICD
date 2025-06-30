#!/usr/bin/env python3
"""
Train simple fully-connected AutoEncoder on per-row feature vectors
Outputs:  anomaly_model_ae.pkl  (contains scaler + model)
"""

import numpy as np, pandas as pd, joblib, tensorflow as tf, os
from sklearn.metrics import roc_auc_score
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

CSV = "data_store/ros_metrics_all.csv"

df = (pd.read_csv(CSV, header=None,
        names=["Time","CPU","Memory",
               "CPU_roll","CPU_slope","Mem_roll","Mem_slope"])
        .apply(pd.to_numeric, errors="coerce").dropna())

X_raw = df[["CPU","Memory","CPU_roll","CPU_slope",
            "Mem_roll","Mem_slope"]].values.astype("float32")

scaler = StandardScaler().fit(X_raw)
X = scaler.transform(X_raw)

X_train, X_val = train_test_split(X, test_size=0.2, shuffle=True, random_state=42)

inp = tf.keras.Input(shape=(X.shape[1],))
h = tf.keras.layers.Dense(16, activation="relu")(inp)
h = tf.keras.layers.Dense(8 , activation="relu")(h)
encoded = tf.keras.layers.Dense(4 , activation="relu")(h)
h = tf.keras.layers.Dense(8 , activation="relu")(encoded)
h = tf.keras.layers.Dense(16, activation="relu")(h)
out = tf.keras.layers.Dense(X.shape[1], activation=None)(h)

ae = tf.keras.Model(inp, out)
ae.compile(optimizer="adam", loss="mse")
ae.fit(X_train, X_train,
       epochs=20, batch_size=256,
       validation_data=(X_val, X_val), verbose=2)

# reconstruction error distribution on val set ----------------------------
recon = ae.predict(X_val, verbose=0)
mse   = np.mean(np.square(X_val - recon), axis=1)
th    = np.percentile(mse, 95)          # 95-th pct as anomaly threshold

joblib.dump({"scaler": scaler,
             "model" : ae,
             "thresh": th},
            "anomaly_model_ae.pkl")
print("AE saved. 95-th-pct threshold =", th)
