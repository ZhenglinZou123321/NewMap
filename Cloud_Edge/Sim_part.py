'''import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque'''
import sumolib
import traci
import json
import csv
import pandas as pd
'''import seaborn as sns
import matplotlib.pyplot as plt
import threading
import re
import time
import copy'''
#from Solver_utils import QP_solver
from  network_utils import check_and_kill_port
#import socket

# 车辆参数
MAX_SPEED = 11  # 最大速度 (m/s)
MIN_SPEED = 0
MAX_ACCEL = 3  # 最大加速度 (m/s^2)
MIN_ACCEL = -20


# 信号灯周期和时间参数
GREEN_TIME = 30  # 绿灯时间
RED_TIME = 30  # 红灯时间
CYCLE_TIME = GREEN_TIME + RED_TIME
dt = 0.2
N=40
L_safe = 4 + 3 #4米车长，3米间距
# 初始化交叉口车辆列表
vehicles = []


# IDM模型参数
V_0 = MAX_SPEED  # 期望速度 (m/s)，可根据实际情况调整
T = 1.5  # 期望车头时距 (s)，可根据实际情况调整
a_max = MAX_ACCEL  # 最大加速度 (m/s²)，与前面已定义的保持一致或按需调整
b = -1*MIN_ACCEL  # 舒适制动减速度 (m/s²)，可根据实际情况调整
s_0 = 2  # 最小间距 (m)，可根据实际情况调整
delta = 4  # 速度影响指数参数，可根据实际情况调整

if __name__ == '__main__':
    # 启动SUMO仿真
    check_and_kill_port(14491)
    #traci.start(["sumo-gui", "-c", "Gaussian_trip.sumocfg","--start","--step-length",'0.2',"--remote-port",'8813'])
    #traci.start(["sumo-gui", "-c", "Gaussian_trip.sumocfg", "--start", "--step-length", '0.2','--port','14491'])
    sumo_cmd = [
        "sumo-gui",
        "-c", "Gaussian_trip.sumocfg",
        "--step-length", "0.2",
        "--num-clients","8"
    ]


    # 使用增强配置启动连接
    traci.start(
        sumo_cmd,
        port= 14491,
        numRetries=5  # 最多重试5次
    )
    #sock = traci.getConnection("default")._socket  # 获取 TraCI 的底层 socket
    #sock.setblocking(False)  # 设置为非阻塞

    print('started')
    traci.setOrder(2)
    step = 0
    while step < 3600*1:  # 仿真时间，例如1小时
        traci.simulationStep()  # 每步执行仿真

    traci.close()

'''
    net = sumolib.net.readNet("Map_new.net.xml")  # 替换为您的 .net.xml 文件路径

    with open("Graph/junction_index.json", "r") as f:
        junction_index_dict = json.load(f)

    with open("Graph/lane_index.json", "r") as f:
        lane_index_dict = json.load(f)
        index_lane_dict = {v: k for k, v in lane_index_dict.items()}  # 构造反向字典


    df = pd.read_csv("Graph/junction_adj_matrix.csv", index_col=0)
    junc_adj_matrix = df.values

    df = pd.read_csv("Graph/lane_adj_matrix.csv", index_col=0)
    lane_adj_matrix = df.values

    with open("Graph/traffic_light_info.json", "r") as f:
        data = json.load(f)
    # 获取 lane_to_traffic_light 和 traffic_light_to_lanes 字典
    lane_to_traffic_light = data["lane_to_traffic_light"]
    traffic_light_to_lanes = data["traffic_light_to_lanes"]

    with open('traffic_data_gaussian.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])

    # 仿真循环
    Action_list = [10,15,20,25,30,35,40,45,50,55,60,65,70,75,80]
    Intelligent_Sigal_List = ['j5', 'j6', 'j7', 'j10','j11','j12']

    Intersection_Edge_Dict = {'j20': {'in': ['j26tj20', 'j21tj20', 'j13tj20', 'j19tj20'], 'out': ['j20tj26', 'j20tj21', 'j20tj13', 'j20tj19']}}
    Agent_List = {}
    Least_Check_Time = 3
    train_batchsize = 32
    train_gap = 20
    arrived_vehicles = set()
    vehicle_depart_times = {}
    total_travel_time = 0


    heat_gap = 300


    state_size = 1+3*3+3*3+1+3*3+3*3+2

    shared_model = True
    shared_model_location = 'models/agent_model.pth'
    Train_Or_Not = False


    waiting_time_dict={}
    junction_counts = {k.getID():0 for k in net.getNodes()}  # 用于记录每个 junction 的车辆通过计数

    previous_vehicle_edges = {}  # 用于记录每个车辆的上一个 edge



    vehicle_threads = {}'''

