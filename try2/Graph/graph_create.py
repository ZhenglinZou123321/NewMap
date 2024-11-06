import traci
import networkx as nx
import numpy as np
import sumolib
import pandas as pd
import json

# 启动 SUMO 仿真（假设已正确配置并启动 SUMO）
traci.start(["sumo", "-c", "../Gaussian_trip.sumocfg"])  # 替换为您的配置文件

# 获取所有车道的 ID
lane_ids_temp = traci.lane.getIDList()

lane_ids = [lane for lane in lane_ids_temp if lane[0] == 'j']


# 加载网络文件
net = sumolib.net.readNet("../Map_new.net.xml")  # 替换为您的 .net.xml 文件路径

# 获取所有路口和边
junction_ids = [j.getID() for j in net.getNodes() if j.getID()[0] == 'j']
edge_ids = [e.getID() for e in net.getEdges() if e.getID()[0] == 'j']

# 构建邻接矩阵
num_junctions = len(junction_ids)
adj_matrix = np.zeros((num_junctions, num_junctions), dtype=int)

# 创建一个字典，用于快速索引路口在邻接矩阵中的位置
junction_index = {junction_id: idx for idx, junction_id in enumerate(junction_ids)}

# 遍历每条边，根据边的起点和终点确定路口的连接关系
for edge in net.getEdges():
    from_junction = edge.getFromNode().getID()
    to_junction = edge.getToNode().getID()

    # 检查起点和终点是否是路口
    if from_junction in junction_index and to_junction in junction_index:
        adj_matrix[junction_index[from_junction], junction_index[to_junction]] = 1


# 使用 pandas 将邻接矩阵保存为带表头的 CSV 文件
df = pd.DataFrame(adj_matrix, index=junction_ids, columns=junction_ids)
df.to_csv("junction_adj_matrix.csv", index=True)

# 保存字典为 JSON 文件
with open("junction_index.json", "w") as f:
    json.dump(junction_index, f)


# 构建邻接矩阵
num_lanes = len(lane_ids)
adj_matrix = np.zeros((num_lanes, num_lanes), dtype=int)

# 创建一个字典，用于快速索引车道在邻接矩阵中的位置
lane_index = {lane_id: idx for idx, lane_id in enumerate(lane_ids)}

# 遍历每个车道，获取它的相连车道并填充邻接矩阵
for lane_id in lane_ids:
    links = traci.lane.getLinks(lane_id)
    for link in links:
        target_lane = link[0]
        if target_lane in lane_index:
            adj_matrix[lane_index[lane_id], lane_index[target_lane]] = 1

# 使用 pandas 将邻接矩阵保存为带表头的 CSV 文件
df = pd.DataFrame(adj_matrix, index=lane_ids, columns=lane_ids)
df.to_csv("lane_adj_matrix.csv", index=True)

# 保存字典为 JSON 文件
with open("lane_index.json", "w") as f:
    json.dump(lane_index, f)
# 关闭仿真
traci.close()