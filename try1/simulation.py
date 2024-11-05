import traci
import csv
import time
import xml.etree.ElementTree as ET
import csv

import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

time_gap = 600

# 打开CSV文件准备写入
with open('traffic_data.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])

    # 启动SUMO仿真
    traci.start(["sumo-gui", "-c", "Traffic_Sim.sumocfg"])

    # 仿真循环
    step = 0
    while step < 3600*24:  # 仿真时间，例如1小时
        traci.simulationStep()  # 每步执行仿真
        if step%time_gap == 0:
            for edge_id in traci.edge.getIDList():
                if edge_id[0] != 'j':
                    continue
                vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
                avg_speed = traci.edge.getLastStepMeanSpeed(edge_id)
                writer.writerow([step/time_gap, edge_id, vehicle_count, avg_speed])
        step += 1

    traci.close()

# 加载交通数据，假设数据中包含 'road_id', 'time', 'vehicle_count' 等列
data = pd.read_csv('traffic_data.csv')

# 按道路和时间汇总数据
pivot_data = data.pivot_table(index='road_id', columns='time', values='vehicle_count', aggfunc='mean')

# 绘制热力图
plt.figure(figsize=(3600*24/time_gap, 8))
sns.heatmap(pivot_data, cmap="YlOrRd", cbar=True)
plt.title("Traffic Flow Heatmap")
plt.xlabel("Time")
plt.ylabel("Road Segment")
plt.show()
