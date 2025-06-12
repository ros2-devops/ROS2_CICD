import pandas as pd
import matplotlib.pyplot as plt

input_path = '/home/runner/ros_metrics.csv'
df = pd.read_csv(input_path, header=None, names=['Time', 'CPU', 'Memory'])

# CPU Plot
plt.figure()
df.plot(x='Time', y='CPU', title='CPU Usage Over Time')
plt.xlabel('Time (s)')
plt.ylabel('CPU (%)')
plt.savefig('cpu_usage.png')

# Memory Plot
plt.figure()
df.plot(x='Time', y='Memory', title='Memory Usage Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Memory (%)')
plt.savefig('memory_usage.png')
