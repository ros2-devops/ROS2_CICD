import pandas as pd

input_file = "data_store/ros_metrics_all.csv"
output_file = "ros_metrics_cleaned.csv"

# Read file assuming 2-row alternating format
with open(input_file, 'r') as f:
    lines = f.readlines()

# Group lines in pairs
records = []
for i in range(0, len(lines), 2):
    if i+1 >= len(lines): continue  # Skip if no pair
    data_line = lines[i].strip().split(',')
    meta_line = lines[i+1].strip().split(',')

    if len(data_line) != 3 or len(meta_line) != 2:
        continue  # Skip malformed

    time, cpu, mem = data_line
    scenario, run = meta_line
    records.append([float(time), float(cpu), float(mem), scenario.strip(), int(run)])

# Save as DataFrame
df = pd.DataFrame(records, columns=["Time", "CPU", "Memory", "Scenario", "Run"])
df.to_csv(output_file, index=False)
print(f" Cleaned CSV saved as {output_file} with {len(df)} rows")
