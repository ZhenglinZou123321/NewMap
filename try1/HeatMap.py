import xml.etree.ElementTree as ET
import csv

import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# 加载交通数据，假设数据中包含 'road_id', 'time', 'vehicle_count' 等列
data = pd.read_csv('traffic_data.csv')

# 按道路和时间汇总数据
pivot_data = data.pivot_table(index='road_id', columns='time', values='vehicle_count', aggfunc='mean')

# 绘制热力图
plt.figure(figsize=(3600*24/600, 20))
sns.heatmap(pivot_data, cmap="YlOrRd", cbar=True)
plt.title("Traffic Flow Heatmap")
plt.xlabel("Time")
plt.ylabel("Road Segment")
plt.show()