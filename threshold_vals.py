import joblib

print("AE threshold =", joblib.load("trained_models_new/autoencoder_threshold.pkl"))
print("LSTM threshold =", joblib.load("trained_models_new/cnn_lstm_threshold.pkl"))
