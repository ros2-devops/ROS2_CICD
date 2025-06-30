#!/usr/bin/env python3
"""
Train CNN-LSTM on sliding-window tensors produced by prepare_seq_dataset.py
Outputs: anomaly_model_cnnlstm.pkl
"""

import numpy as np, tensorflow as tf, joblib, os, json
from sklearn.model_selection import train_test_split

SEQ = "X_seq.npy"
SCALER = "trained_models/scaler.pkl"
assert os.path.exists(SEQ) and os.path.exists(SCALER), \
       "Run prepare_seq_dataset.py first."

X = np.load(SEQ)               # shape (N, win, feat)
scaler = joblib.load(SCALER)

X_train, X_val = train_test_split(X, test_size=0.2, shuffle=True, random_state=42)

win, feat = X.shape[1:3]
inp = tf.keras.Input(shape=(win, feat))
h = tf.keras.layers.Conv1D(32, 3, padding="same", activation="relu")(inp)
h = tf.keras.layers.MaxPool1D(2)(h)
h = tf.keras.layers.LSTM(32, return_sequences=False)(h)
h = tf.keras.layers.Dense(16, activation="relu")(h)
out = tf.keras.layers.Dense(feat * win, activation=None)(h)
out = tf.keras.layers.Reshape((win, feat))(out)

model = tf.keras.Model(inp, out)
model.compile(optimizer="adam", loss="mse")
model.fit(X_train, X_train,
          epochs=15, batch_size=128,
          validation_data=(X_val, X_val), verbose=2)

recon = model.predict(X_val, verbose=0)
mse   = np.mean(np.square(X_val - recon), axis=(1,2))
th    = np.percentile(mse, 95)

joblib.dump({"scaler": scaler,
             "model" : model,
             "thresh": th,
             "window": win},
            "anomaly_model_cnnlstm.pkl")
print("CNN-LSTM saved. 95-th-pct threshold =", th)
