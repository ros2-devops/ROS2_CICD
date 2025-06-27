import pandas as pd, matplotlib.pyplot as plt, seaborn as sns
df = pd.read_csv('data_store/ros_metrics_all.csv',
                 names=['Time','CPU','Mem','CPU_roll','CPU_slope','Mem_roll','Mem_slope'])
sns.pairplot(df[['CPU','Mem','CPU_roll','CPU_slope','Mem_roll','Mem_slope']])
plt.savefig(f"feature_dist.png")
