# inspect_columns.py
import pandas as pd

df = pd.read_csv("datasets/borg_tru_small.csv")
print("Columns:\n", df.columns)
print("\nFirst few rows:\n", df.head())
