import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import random
from collections import deque
import sumolib
import traci
import json
import csv
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import threading
import re
import time
import copy
from planning_test import QP_solver




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

def idm_acceleration(current_speed, front_vehicle_speed, gap,  front_vehicle_id=None):
    """
    根据IDM模型计算车辆的加速度。

    参数:
    current_speed (float): 当前车辆速度 (m/s)
    front_vehicle_speed (float): 前车速度 (m/s)
    gap (float): 当前车与前车的间距 (m)
    front_vehicle_id (str, 可选): 前车的ID，用于调试或其他可能的拓展需求，默认为None

    返回:
    float: 根据IDM模型计算出的当前车辆加速度 (m/s²)
    """
    relative_speed = current_speed - front_vehicle_speed
    s_star = s_0 + current_speed * T + (current_speed * relative_speed) / (2 * np.sqrt(a_max * b))
    acceleration = a_max * (1 - (current_speed / V_0) ** delta - (s_star / gap) ** 2)
    return min(max(acceleration,MIN_ACCEL),MAX_ACCEL)

# 线程类定义
class VehicleController(threading.Thread):
    def __init__(self, vehicle_id_):
        threading.Thread.__init__(self)
        self.vehicle_id = vehicle_id_
        self.running = True
        self.acc_control = 0
        self.idm_acc = 0
    def run(self):
        while self.running and traci.simulation.getMinExpectedNumber() > 0 and self.vehicle_id[0:3] == "CAV":
            try:
                # 检查车辆是否仍然在仿真中
                if self.vehicle_id in traci.vehicle.getIDList():
                    # 示例控制逻辑：设置车辆速度
                    self.speed = traci.vehicle.getSpeed(self.vehicle_id)
                    print("iiiii")
                    self.front_info = traci.vehicle.getLeader(self.vehicle_id)
                    print("ddddd")
                    self.idm_acc = None
                    if self.front_info != None:
                        self.front_id,self.gap = self.front_info
                        print(f"{self.vehicle_id}的前车是{self.front_id}")
                    if self.front_info != None and traci.vehicle.getLaneID(self.vehicle_id) == traci.vehicle.getLaneID(self.front_id):
                        self.idm_acc = idm_acceleration(self.speed, traci.vehicle.getSpeed(self.front_id), self.gap, front_vehicle_id=None)
                    else:
                        self.lane_now = traci.vehicle.getLaneID(self.vehicle_id)
                        self.phase,self.remaining_time = get_remaining_phase_and_time(self.lane_now)
                        if self.phase == 'r' or self.phase == 'y':
                            self.lane_length = traci.lane.getLength(self.lane_now)
                            self.idm_acc = idm_acceleration(self.speed, 0.0,
                                                        self.lane_length-traci.vehicle.getLanePosition(self.vehicle_id),
                                                        front_vehicle_id=None)
                            print("kkk")
                            print(self.lane_now)
                            print(self.vehicle_id)
                            print(self.lane_length)
                            print(traci.vehicle.getLanePosition(self.vehicle_id))
                            print(self.idm_acc)
                    if self.vehicle_id[0:3] == "CAV":
                        if len(control_signal[self.vehicle_id].control_list) != 0:
                            traci.vehicle.setSpeedMode(self.vehicle_id, 00000)  # 关闭跟驰模型
                            self.mpc_acc = control_signal[self.vehicle_id].control_signal(time_now, dt)
                            if self.idm_acc != None:
                                #self.acc_control = control_signal[self.vehicle_id].control_signal(time_now, dt)
                                print(f"{self.vehicle_id} MPC控制量为：{self.mpc_acc}, IDM控制量为：{self.idm_acc}")
                                if self.mpc_acc != None:
                                    if self.mpc_acc < self.idm_acc:
                                        self.acc_control = control_signal[self.vehicle_id].control_signal(time_now, dt)
                                        print(f"{self.vehicle_id} 施加MPC控制量为：{self.acc_control}")
                                        traci.vehicle.setColor(self.vehicle_id, (0, 0, 255)) # MPC = blue
                                    else:
                                        self.acc_control = self.idm_acc
                                        print(f"{self.vehicle_id} 施加IDM控制量为：{self.acc_control}")
                                        traci.vehicle.setColor(self.vehicle_id, (255, 0, 0))  # IDM = red
                                else:
                                    self.acc_control = self.idm_acc
                                    print(f"{self.vehicle_id} 施加IDM控制量为：{self.acc_control}")
                                    traci.vehicle.setColor(self.vehicle_id, (255, 0, 0))  # IDM = red
                            else:
                                self.acc_control = self.mpc_acc
                                traci.vehicle.setColor(self.vehicle_id, (0, 0, 255))  # MPC =  blue
                            #print(self.acc_control)
                            if self.acc_control != None:
                                traci.vehicle.setAcceleration(self.vehicle_id, self.acc_control, 1)
                            else:
                                traci.vehicle.setAcceleration(self.vehicle_id, 0.0, 1)
                            #print(control_signal[self.vehicle_id].control_list_show())
                            print(f"{self.vehicle_id}已施加加速度控制量：{self.acc_control}")
                # new_speed = speed + 1  # 简单地增加速度
                # traci.vehicle.setSpeed(self.vehicle_id, new_speed)
                # print(f"Vehicle {self.vehicle_id} speed updated to {new_speed}")
                else:
                    # 如果车辆离开仿真，停止线程
                    print(f"Vehicle {self.vehicle_id} has left the simulation.")
                    self.running = False
            except Exception as e:
                print(f"Error controlling vehicle {self.vehicle_id}: {e}")
                self.running = False

            # 控制逻辑运行间隔
            time.sleep(0.0001)

    def stop(self):
        self.running = False


class control_info:
    def __init__(self,vehicle_id):
        self.vehicle_id = vehicle_id
        self.time_when_cal = 0
        self.control_list = []
        self.state_initial = [0.0,0.0]
    def set_time(self,time):
        self.time_when_cal = time
    def control_list_clear(self):
        self.control_list = []
    def control_list_append(self,control_signal):
        self.control_list.append(control_signal)
    def control_signal(self,time,dt):
        if int((time-self.time_when_cal)/dt) <= len(self.control_list)-1:
            return self.control_list[int((time-self.time_when_cal)/dt)]
        else:
            return None
    def control_list_show(self):
        return self.control_list
    def set_state(self,state):
        self.state_initial[0] = state[0]
        self.state_initial[1] = state[1]

def get_remaining_phase_and_time(lane_id): #获取信号灯当前相位和剩余时间
    # 按照固定字符进行分割
    x, rest = lane_id.split("t", 1)  # 分割出 X 和剩余部分
    intersection_id, z = rest.split("_", 1)  # 分割出 Y 和 Z
    # 获取当前仿真时间
    current_time = traci.simulation.getTime()
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(intersection_id)
    # 计算剩余时间 秒
    remaining_time = next_switch_time - current_time
    current_phase = traci.trafficlight.getRedYellowGreenState(intersection_id)[traci.trafficlight.getControlledLanes(intersection_id).index(lane_id)]
    return current_phase.lower(),max(remaining_time, 0)  # 防止负值


#得到每条edge上的车辆列表以及后1/4的车辆列表
def get_all_edge_vehicles_and_last_quarter():
    """
    获取仿真中所有路段上的车辆，以及位于每条车道后四分之一段的车辆
    :return: 两个字典
        - edge_vehicles: {edgeID: {laneID: [vehicleID, ...]}}
        - last_quarter_vehicles: {edgeID: {laneID: [vehicleID, ...]}}
    """
    edge_vehicles = {}  # 每条路段上的车辆
    last_quarter_vehicles = {}  # 每条路段后四分之一段的车辆

    # 获取仿真中的所有路段
    edge_ids = traci.edge.getIDList()

    for edgeID in edge_ids:
        edge_vehicles[edgeID] = {}
        last_quarter_vehicles[edgeID] = {}

        # 获取路段上的所有车道
        #lane_ids = traci.edge.getLanes(edgeID)
        lane_ids = [f"{edgeID}_{i}" for i in range(traci.edge.getLaneNumber(edgeID))]
        for laneID in lane_ids:
            # 获取当前车道上的车辆
            vehicles_on_lane = traci.lane.getLastStepVehicleIDs(laneID)
            edge_vehicles[edgeID][laneID] = list(vehicles_on_lane)

            # 获取车道长度
            lane_length = traci.lane.getLength(laneID)

            # 筛选后四分之一段的车辆
            vehicles_in_last_quarter = [
                vehID for vehID in vehicles_on_lane
                if traci.vehicle.getLanePosition(vehID) > 0.5 * lane_length
            ]
            last_quarter_vehicles[edgeID][laneID] = vehicles_in_last_quarter #idx=0的车是最靠近lane终点的，以此类推

    return edge_vehicles, last_quarter_vehicles

# 更新 CAV 速度控制
def update_cav_speeds(intersection_id,traffic_light_to_lanes,last_quarter_vehicles,N,dt,L_safe):
    start_time = time.time()
    #in_range_vehicles = get_vehicles_in_range("j3")
    #in_range_vehicles = get_vehicles_in_range(intersection_id,traffic_light_to_lanes)
    #in_range_vehicles_lanes = last_quarter_vehicles[intersection_id]
    now_time = traci.simulation.getTime()
    for lane_id in traffic_light_to_lanes[intersection_id]:
        num_CAV = 0
        num_HDV = 0
        initial_state_CAV = []  # 最后要转换为np.array() [位置,速度]
        initial_state_HDV = []  # 最后要转换为np.array() [位置,速度]
        CAV_id_list = []
        HDV_id_list = []
        type_list = []
        for temp in [i for i, value in enumerate(lane_adj_matrix[lane_index_dict[lane_id]]) if value != 0]:
            lane_towards = index_lane_dict[temp]
        #print(lane_id)
        #print(last_quarter_vehicles)
        vehicles_list_this_lane = copy.deepcopy(last_quarter_vehicles[lane_id[:-2]][lane_id])
        for vehicle_id in vehicles_list_this_lane:
            print('1')
            if vehicle_id not in control_signal.keys():
                control_signal[vehicle_id] = control_info(vehicle_id)
            control_signal[vehicle_id].set_time(now_time)
            control_signal[vehicle_id].control_list_clear()
            if vehicle_id[0:3] == "CAV":
                num_CAV +=1
                type_list.append('CAV')
                state = [traci.vehicle.getLanePosition(vehicle_id), traci.vehicle.getSpeed(vehicle_id)]
                control_signal[vehicle_id].set_state(state)
                initial_state_CAV.append(state)
                CAV_id_list.append(vehicle_id)
            elif vehicle_id[0:3] == "HDV":
                num_HDV +=1
                type_list.append('HDV')
                state = [traci.vehicle.getLanePosition(vehicle_id), traci.vehicle.getSpeed(vehicle_id)]
                control_signal[vehicle_id].set_state(state)
                initial_state_HDV.append(state)
                HDV_id_list.append(vehicle_id)
            #CAV HDV的状态都要收集 但是只控制CAV
                #target_speed = MAX_SPEED if is_green_light("j3") else 0
                #optimized_speed, optimized_accel = optimize_speed(vehicle_id, target_speed, MAX_SPEED, MAX_ACCEL, REACTION_TIME)
        if len(initial_state_CAV) != 0:
            print('2')
            type_info = (type_list,num_CAV,num_HDV)
            initial_state_CAV = np.array(initial_state_CAV)
            initial_state_HDV = np.array(initial_state_HDV)
            #mpc_control(initial_state, 0, weights=[1.0,0.5], N=20, dt=dt, bounds=(MIN_ACCEL,MAX_ACCEL,0,MAX_SPEED),type_info=type_info,now_lane = lane_id,lane_towards = lane_towards,last_quarter_vehicles=last_quarter_vehicles)
            QP_solver(initial_state_CAV, initial_state_HDV, vehicles_list_this_lane, N, dt, v_max=MAX_SPEED, v_min=MIN_SPEED, a_max=MAX_ACCEL,a_min=MIN_ACCEL, L_safe=L_safe, lane_now=lane_id, CAV_id_list=CAV_id_list, HDV_id_list=HDV_id_list)

            print('3')
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"计算耗时: {elapsed_time:.4f} 秒")



# 合并所有智能体的 memory
def merge_and_save_memory(agent_list, file_path):
    merged_memory = []

    for agent in agent_list.values():  # 遍历每个智能体
        for experience in agent.memory:
            state, action, reward, next_state = experience
            merged_memory.append({
                'state': state.tolist(),  # 将 numpy 数组转换为列表
                'action': action,
                'reward': reward,
                'next_state': next_state.tolist(),  # 将 numpy 数组转换为列表
            })

    # 保存合并后的 memory 到 JSON 文件
    with open(file_path, 'a') as file:
        json.dump(merged_memory, file)

def match_strings(str1, str2):
    # 使用正则表达式将字符串按 "tj" 分成三部分：j开头部分、中间部分、t结尾部分
    pattern = r'j(\d+)tj(\d+)'
    match1 = re.match(pattern, str1)
    match2 = re.match(pattern, str2)

    # 检查是否匹配成功，并且两者的分隔结果相同（忽略顺序）
    if match1 and match2:
        part1_1, part2_1 = match1.groups()
        part1_2, part2_2 = match2.groups()
        # 比较是否相同
        return (part1_1 == part2_2 and part2_1 == part1_2) or (part1_1 == part1_2 and part2_1 == part2_2)
    return False



def is_incoming_lane(lane_id):
    # 获取该车道的链接信息
    links = traci.lane.getLinks(lane_id)
    if not links:
        return False  #如果没有链接，则不属于驶入车道

    # 如果第一段链接是进入路口的（交叉口），则该车道为驶入车道
    next_lane_id, via_edge_id, signal_index, traffic_light_id = links[0]
    if traffic_light_id:
        return True  #有信号灯控制则为驶入路口车道
    return False

def get_remaining_phase_time(traffic_light_id): #获取信号灯剩余时间
    # 获取当前仿真时间
    current_time = traci.simulation.getTime()
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(traffic_light_id)
    # 计算剩余时间
    remaining_time = next_switch_time - current_time
    return max(remaining_time, 0)  # 防止负值

def get_lane_state(lane_id,lane_dict,lane_m):
    traffic_signal_dict = {'r':0,'g':1,'y':2}
    edge_id = traci.lane.getEdgeID(lane_id)
    to_junction = traci.edge.getToJunction(edge_id)
    if to_junction in traffic_light_to_lanes.keys():
        controlled_lanes = traci.trafficlight.getControlledLanes(to_junction)
        current_phase_state = traci.trafficlight.getRedYellowGreenState(to_junction)
        lane_index = controlled_lanes.index(lane_id)
        lane_phase = current_phase_state[lane_index]
        remain_time = get_remaining_phase_time(to_junction)
        return traffic_signal_dict[lane_phase.lower()], remain_time
    else:
        lane_phase = 'g'
        remain_time = 99


    return traffic_signal_dict[lane_phase],remain_time

def get_state(intersection_id,Intersection_Edge_Dict,lane_index_dict,lane_adj,nowphase_index):
    reversed_lane_dict = {str(v): k for k, v in lane_index_dict.items()}
    next_state_of_last = []
    new_state = []
    traffic_signal_dict = {'r':0,'g':1,'y':2}
    checked_lane = []
    dentisy_self = 0#不处理右转车道
    dentisy_from = 0
    dentisy_to = 0#目标车道的车辆密度
    next_green_density_last = []
    next_green_density_new = []
    current_phase_state = traci.trafficlight.getRedYellowGreenState(intersection_id)
    next_phase_state = traci.trafficlight.getCompleteRedYellowGreenDefinition(intersection_id)[0].phases[(nowphase_index + 1) % 8].state
    next_3_phase_state = traci.trafficlight.getCompleteRedYellowGreenDefinition(intersection_id)[0].phases[(nowphase_index + 3) % 8].state
    #for edge in Intersection_Edge_Dict[intersection_id]['in']:
    for (index,lane) in enumerate(traci.trafficlight.getControlledLanes(intersection_id)):
        if lane  in checked_lane:
            continue
        checked_lane.append(lane)
        if lane[-1]=='0':#不处理右转车道
            continue

        now_signal_state = traffic_signal_dict[current_phase_state[index].lower()]
        next_signal_state = traffic_signal_dict[next_phase_state[index].lower()]
        next_3_signal_state = traffic_signal_dict[next_3_phase_state[index].lower()]
        if now_signal_state == 2:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length/traci.lane.getLength(lane) #self占用率
            #links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(lane_adj[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(lane_adj[:, lane_index])[0]#来这个lane的索引
            next_state_of_last.extend([dentisy_self])
            for one_lane in to_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_to = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                next_state_of_last.extend([dentisy_to,signal_index,remain_time])

            from_temp = []
            for one_lane in from_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_from = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                from_temp.extend([dentisy_to, signal_index, remain_time])
            from_temp += [0] * (3 * 3 - len(from_temp))
            next_state_of_last.extend(from_temp)

        elif now_signal_state == 0 and next_signal_state == 1:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length/traci.lane.getLength(lane) #self占用率
            #links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(lane_adj[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(lane_adj[:, lane_index])[0]#来这个lane的索引
            new_state.extend([dentisy_self])
            next_green_density_last.append(dentisy_self)
            for one_lane in to_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_to = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                new_state.extend([dentisy_to,signal_index,remain_time])

            from_temp = []
            for one_lane in from_list:
                if match_strings(reversed_lane_dict[str(one_lane)][:-2],lane[:-2]):
                    continue
                one_lane = reversed_lane_dict[str(one_lane)]
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_from = vehicle_occupancy_length/traci.lane.getLength(one_lane)
                signal_index,remain_time = get_lane_state(one_lane, lane_index_dict, lane_adj)
                from_temp.extend([dentisy_to,signal_index,remain_time])
            from_temp += [0]*(3*3 - len(from_temp))
            new_state.extend(from_temp)


        elif now_signal_state == 0 and next_signal_state == 0 and next_3_signal_state == 1:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self = vehicle_occupancy_length / traci.lane.getLength(lane)  # self占用率
            next_green_density_new.append(dentisy_self)
    next_state_of_last.extend(next_green_density_last)
    new_state.extend(next_green_density_new)

    if len(next_state_of_last) !=40 or len(new_state) != 40:
        print('wrong')


    return np.array(next_state_of_last, dtype=np.float32),np.array(new_state, dtype=np.float32)



def get_reward(intersection_id,agent,Action_list,junction_counts):
    reward = 0
    checked_lane = []
    traffic_signal_dict = {'r': 0, 'g': 1, 'y':2}
    passed_count = junction_counts[intersection_id] - agent.passed_count
    passed_vel = passed_count/Action_list[agent.action]
    agent.passed_count = junction_counts[intersection_id]
    reward = passed_vel*100
    return reward


class GATLayer(nn.Module):
    def __init__(self, in_features, out_features):
        super(GATLayer, self).__init__()
        self.W = nn.Linear(in_features, out_features, bias=False)
        self.a = nn.Linear(2 * out_features, 1, bias=False)

    def forward(self, x, adj):
        h = self.W(x)
        N = h.size(0)

        a_input = torch.cat([h.repeat(1, N).view(N * N, -1), h.repeat(N, 1)], dim=1)
        e = self.a(a_input).squeeze(1)

        attention = torch.nn.functional.softmax(e, dim=1)
        h_prime = torch.matmul(attention, h)

        return h_prime

class GraphSAGEWithBiGRU(nn.Module):
    def __init__(self, in_features, out_features):
        super(GraphSAGEWithBiGRU, self).__init__()
        self.gru = nn.GRU(in_features, out_features, bidirectional=True)

    def forward(self, x, adj):
        h, _ = self.gru(x)
        h = torch.sum(h, dim=0)
        return h

class QNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super(QNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, action_size)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        return self.fc4(x)


class DDQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size #[10,15,20,25,30,35,40] 7
        self.memory = deque(maxlen=2000)
        self.gamma = 0.99
        self.epsilon = 0.5
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.01
        self.learning_rate = 0.001
        self.q_network = QNetwork(state_size, action_size)
        self.target_network = QNetwork(state_size, action_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.learning_rate)
        self.update_target_network()
        self.now_Duration = 0
        self.ChangeOrNot = False
        self.CheckOrNot = False
        self.state = np.zeros(state_size, dtype=np.float32)
        self.action = 0
        self.total_reward = 0
        self.reward_delta = 0
        self.step = 0
        self.Trained_time = 0
        self.passed_count = 0

    def update_target_network(self):
        self.target_network.load_state_dict(self.q_network.state_dict())

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        state = torch.FloatTensor(state).unsqueeze(0)
        act_values = self.q_network(state)
        return torch.argmax(act_values, dim=1).item()

    def train(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)

        for state, action, reward, next_state in minibatch:
            target = reward

            next_action = torch.argmax(self.q_network(torch.FloatTensor(next_state).unsqueeze(0)), dim=1)
            target += self.gamma * self.target_network(torch.FloatTensor(next_state).unsqueeze(0))[0][next_action]

            target_f = self.q_network(torch.FloatTensor(state).unsqueeze(0)).detach().clone()
            target_f[0][action] = target

            self.optimizer.zero_grad()
            loss = nn.MSELoss()(self.q_network(torch.FloatTensor(state).unsqueeze(0)), target_f)
            loss.backward()
            self.optimizer.step()
        self.Trained_time += 1

        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay


def train_agent(episodes, agent, batch_size=32):
    for episode in range(episodes):

        state = get_state("intersection_id")
        total_reward = 0

        while traci.simulation.getMinExpectedNumber() > 0:
            action = agent.act(state)
            traci.trafficlight.setPhase("intersection_id", action)

            next_state = get_state("intersection_id")
            reward = get_reward("intersection_id")
            done = traci.simulation.getMinExpectedNumber() == 0
            agent.memory.append((state, action, reward, next_state, done))
            state = next_state
            total_reward += reward

            if len(agent.memory) > batch_size:
                agent.train(batch_size)

        agent.update_target_network()
        print(f"Episode: {episode}, Total Reward: {total_reward}")



if __name__ == '__main__':

    edge_vehicles = {}
    lane_previous_vehicles = {}  # 用于存储每条车道上一时刻的车辆
    control_signal = {}
    Lower_control_signal = {}
    step = 0
    time_now = 0
    vehicle_threads = {}

    # 启动SUMO仿真
    traci.start(["sumo-gui", "-c", "Gaussian_trip.sumocfg","--start"])

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
    step = 0
    arrived_vehicles = set()
    vehicle_depart_times = {}
    total_travel_time = 0

    heat_gap = 300

    state_size = 1+3*3+3*3+1+3*3+3*3+2

    shared_model = True
    shared_model_location = 'models/agent_model.pth'
    Train_Or_Not = False
    for Traffic_Signal_id in Intelligent_Sigal_List:
        Agent_List[Traffic_Signal_id] = DDQNAgent(state_size=state_size ,action_size=len(Action_list))
        if shared_model:
            try:
                Agent_List[Traffic_Signal_id].q_network.load_state_dict(torch.load(shared_model_location))
                Agent_List[Traffic_Signal_id].target_network.load_state_dict(
                    torch.load(shared_model_location))
            except:
                pass
        else:
            try:
                Agent_List[Traffic_Signal_id].q_network.load_state_dict(torch.load(f'models/{Traffic_Signal_id}_model.pth'))
                Agent_List[Traffic_Signal_id].target_network.load_state_dict(torch.load(f'models/{Traffic_Signal_id}_model.pth'))
            except:
                pass

    waiting_time_dict={}
    junction_counts = {k.getID():0 for k in net.getNodes()}  # 用于记录每个 junction 的车辆通过计数

    previous_vehicle_edges = {}  # 用于记录每个车辆的上一个 edge



    vehicle_threads = {}



    while step < 3600*5:  # 仿真时间，例如1小时
        traci.simulationStep()  # 每步执行仿真
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:

            if vehicle_id not in vehicle_threads:
                # 如果发现新车辆，启动一个控制线程
                if vehicle_id[0:3] == "CAV":
                    traci.vehicle.setColor(vehicle_id, (255, 0, 0))
                controller = VehicleController(vehicle_id)
                controller.start()
                vehicle_threads[vehicle_id] = controller
                print(f"Started thread for vehicle {vehicle_id}")
            if vehicle_id not in control_signal.keys():
                continue
            try:
                if vehicle_id[0:3] == "CAV":
                    traci.vehicle.setSpeedMode(vehicle_id, 00000)  # 关闭跟驰模型
                    '''acc_control = control_signal[vehicle_id].control_signal(time_now, dt)
                    print(acc_control)
                    traci.vehicle.setAcceleration(vehicle_id, acc_control,1)
                    print(control_signal[vehicle_id].control_list_show())
                    print(f"{vehicle_id}已施加加速度控制量：{acc_control}")'''
            except Exception as e:
                traci.vehicle.setSpeedMode(vehicle_id, 00000)  # 关闭跟驰模型
                if vehicle_id[0:3] == "CAV":
                    print(f"{vehicle_id}施加了 0 ")
                    traci.vehicle.setAcceleration(vehicle_id, 0, 1)
                # 捕获异常并打印详细信息
                print(f"An error occurred: {e}")
                print(f"{vehicle_id}加速度施加失败")
                pass


            # 获取车辆当前所在的 edge
            current_edge = traci.vehicle.getRoadID(vehicle_id)
            # 如果车辆在网络的有效 edge 上
            if current_edge and current_edge[0] != ":":
                # 获取车辆的上一个 edge
                previous_edge = previous_vehicle_edges.get(vehicle_id)
                # 检查车辆是否从一个 edge 转移到另一个 edge
                if previous_edge and previous_edge != current_edge:
                    # 获取 previous_edge 的终点 junction
                    to_junction = net.getEdge(previous_edge).getToNode().getID()
                    # 在 junction_counts 中递增计数
                    if to_junction in junction_counts:
                        junction_counts[to_junction] += 1
                    else:
                        junction_counts[to_junction] = 1
                # 更新车辆的上一个 edge 为当前 edge
                previous_vehicle_edges[vehicle_id] = current_edge

        if step % 2 == 0:
            # global last_quarter_vehicles
            edge_vehicles, last_quarter_vehicles =get_all_edge_vehicles_and_last_quarter()
            #print(last_quarter_vehicles)
            # 获取刚刚离开每个车道的车辆
            #just_left_vehicles, lane_previous_vehicles = get_vehicles_just_left(lane_previous_vehicles)
            update_cav_speeds('j5', traffic_light_to_lanes, last_quarter_vehicles, N, dt, L_safe)  # 更新 CAV 的速度控制
            update_cav_speeds('j6', traffic_light_to_lanes, last_quarter_vehicles, N, dt, L_safe)  # 更新 CAV 的速度控制
            update_cav_speeds('j7', traffic_light_to_lanes, last_quarter_vehicles, N, dt, L_safe)  # 更新 CAV 的速度控制
            update_cav_speeds('j10', traffic_light_to_lanes, last_quarter_vehicles, N, dt, L_safe)  # 更新 CAV 的速度控制
            update_cav_speeds('j11', traffic_light_to_lanes, last_quarter_vehicles, N, dt, L_safe)  # 更新 CAV 的速度控制
            update_cav_speeds('j12', traffic_light_to_lanes, last_quarter_vehicles, N, dt, L_safe)  # 更新 CAV 的速度控制





        for Traffic_Signal_id in Intelligent_Sigal_List:
            if traci.trafficlight.getPhase(Traffic_Signal_id) in [1,3,5,7] and get_remaining_phase_time(Traffic_Signal_id)<Least_Check_Time and Agent_List[Traffic_Signal_id].CheckOrNot is False:
                next_state,new_state = get_state(Traffic_Signal_id,Intersection_Edge_Dict,lane_index_dict,lane_adj_matrix,traci.trafficlight.getPhase(Traffic_Signal_id))
                reward = get_reward(Traffic_Signal_id,Agent_List[Traffic_Signal_id],Action_list,junction_counts)
                Agent_List[Traffic_Signal_id].memory.append((Agent_List[Traffic_Signal_id].state, Agent_List[Traffic_Signal_id].action, reward, next_state))
                Agent_List[Traffic_Signal_id].step += 1
                Agent_List[Traffic_Signal_id].action = Agent_List[Traffic_Signal_id].act(next_state)
                Agent_List[Traffic_Signal_id].state = new_state
                #traci.trafficlight.setPhase(Traffic_Signal_id, Agent_List[action])
                Agent_List[Traffic_Signal_id].reward_delta = reward
                Agent_List[Traffic_Signal_id].total_reward += reward
                Agent_List[Traffic_Signal_id].CheckOrNot = True

            if traci.trafficlight.getPhase(Traffic_Signal_id) in [0,2,4,6] and Agent_List[Traffic_Signal_id].CheckOrNot is True and get_remaining_phase_time(Traffic_Signal_id)>Least_Check_Time:

                temp_duration = get_remaining_phase_time(Traffic_Signal_id)
                traci.trafficlight.setPhaseDuration(Traffic_Signal_id, float(Action_list[Agent_List[Traffic_Signal_id].action]))
                #print(f"Agent: {Traffic_Signal_id} 原:{temp_duration} 现在:{get_remaining_phase_time(Traffic_Signal_id)} ")
                Agent_List[Traffic_Signal_id].CheckOrNot = False
                if Agent_List[Traffic_Signal_id].step%train_gap == 0 and Agent_List[Traffic_Signal_id].step>=train_batchsize and Train_Or_Not:
                    Agent_List[Traffic_Signal_id].train(train_batchsize)
                    Agent_List[Traffic_Signal_id].update_target_network()
                    torch.save(Agent_List[Traffic_Signal_id].q_network.state_dict(), f'models/{Traffic_Signal_id}_model.pth')
                    print(f"Agent: {Traffic_Signal_id} Reward = {Agent_List[Traffic_Signal_id].reward_delta} Epsilon = {Agent_List[Traffic_Signal_id].epsilon} Trained_time = {Agent_List[Traffic_Signal_id].Trained_time}")

        if step%heat_gap == 0:
            with open('traffic_data_gaussian.csv', mode='a', newline='') as file:
                file.write('\n')
                writer = csv.writer(file)
                for edge_id in traci.edge.getIDList():
                    if edge_id[0] != 'j':
                        continue
                    vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
                    avg_speed = traci.edge.getLastStepMeanSpeed(edge_id)
                    writer.writerow([step / heat_gap, edge_id, vehicle_count, avg_speed])

        current_time = traci.simulation.getTime()
        vehicle_ids = traci.vehicle.getIDList()
        # 记录每辆车的出发时间
        for vehicle_id in vehicle_ids:
            if vehicle_id not in vehicle_depart_times:
                vehicle_depart_times[vehicle_id] = traci.vehicle.getDeparture(vehicle_id)
            waiting_time_dict[vehicle_id] = traci.vehicle.getAccumulatedWaitingTime(vehicle_id)
        arrived_vehicle_ids = traci.simulation.getArrivedIDList()
        for vehicle_id in arrived_vehicle_ids:
            if vehicle_id in vehicle_depart_times:
                # 计算该车的总行驶时间
                travel_time = current_time - vehicle_depart_times[vehicle_id]
                #waiting_time = traci.vehicle.getAccumulatedWaitingTime(vehicle_id)
                total_travel_time += waiting_time_dict[vehicle_id]
                # 从记录中删除该车辆
                del vehicle_depart_times[vehicle_id]


        step += 1

    data_total = []
    merge_and_save_memory(Agent_List, 'Datas/data.json')

    try:
        with open('trained_data_gaussian.json','r',encoding='utf-8') as file:
            trained_data = json.load(file)
        trained_data['times'] = trained_data['times']+1
        trained_data[str(trained_data['times'])] = total_travel_time
        with open('trained_data_gaussian.json', 'w', encoding='utf-8') as file:
            json.dump(trained_data,file,ensure_ascii=False,indent=4)
    except:
        trained_data = {}
        trained_data['times'] = 1
        trained_data[str(trained_data['times'])] = total_travel_time
        with open('trained_data_gaussian.json', 'w', encoding='utf-8') as file:
            json.dump(trained_data,file,ensure_ascii=False,indent=4)

    # 加载交通数据，假设数据中包含 'road_id', 'time', 'vehicle_count' 等列
    data = pd.read_csv('traffic_data_gaussian.csv')

    # 按道路和时间汇总数据
    pivot_data = data.pivot_table(index='road_id', columns='time', values='vehicle_count', aggfunc='mean')

    # 绘制热力图
    plt.figure(figsize=(3600/heat_gap, 8))
    sns.heatmap(pivot_data, cmap="YlOrRd", cbar=True)
    plt.title("Traffic Flow Heatmap")
    plt.xlabel("Time")
    plt.ylabel("Road Segment")
    plt.show()


    traci.close()


