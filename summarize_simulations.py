import pandas as pd
from datetime import datetime

# Load simulation log
log_path = '/home/runner/simulation_log.csv'
try:
    log_df = pd.read_csv(log_path, header=None, names=['timestamp', 'scenario', 'result'])
except Exception as e:
    print(f"Failed to read log: {e}")
    exit(1)

# Count total runs and result breakdown
total_runs = len(log_df)
pass_count = (log_df['result'] == 'PASS').sum()
fail_count = (log_df['result'] == 'FAIL').sum()

# Load last metrics
metrics_path = '/home/runner/ros_metrics.csv'
df = pd.read_csv(metrics_path, header=None, names=['Time', 'CPU', 'Memory'])
avg_cpu = df['CPU'].mean()
avg_mem = df['Memory'].mean()

# Save summary
summary_path = 'evaluation_summary.txt'
with open(summary_path, 'w') as f:
    f.write(f"Simulation Evaluation Summary\n")
    f.write(f"Date: {datetime.now().isoformat()}\n")
    f.write(f"Total Runs: {total_runs}\n")
    f.write(f"Pass: {pass_count}, Fail: {fail_count}\n")
    f.write(f"Average CPU: {avg_cpu:.2f}%\n")
    f.write(f"Average Memory: {avg_mem:.2f}%\n")
    if fail_count > 0:
        f.write("Stability:  Unstable (Failures Detected)\n")
    else:
        f.write("Stability:  Stable\n")

print(" Evaluation summary generated.")
