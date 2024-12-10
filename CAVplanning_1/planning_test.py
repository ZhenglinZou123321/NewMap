import traci
import sumolib
import numpy as np
import json
import gurobipy as gp
from gurobipy import GRB
import pandas as pd
import matplotlib.pyplot as plt
import csv
from scipy.optimize import minimize
import math
import re
import threading
import queue
import time

def update_cav_control_thread_func(traffic_light_to_lanes, lane_previous_vehicles, control_signal):
    ''' 这个函数在独立线程中运行，负责周期性地更新CAV车辆的速度控制
    :param traffic_light_to_lanes:
    :param lane_previous_vehicles:
    :param control_signal:
    :param last_quarter_vehicles:
    :return:
    '''
    while 1:
        if step%5==0:
            #global last_quarter_vehicles
            edge_vehicles, last_quarter_vehicles = get_all_edge_vehicles_and_last_quarter()
            print(last_quarter_vehicles)
            # 获取刚刚离开每个车道的车辆
            just_left_vehicles, lane_previous_vehicles = get_vehicles_just_left(lane_previous_vehicles)
            update_cav_speeds('j3', traffic_light_to_lanes,last_quarter_vehicles)  # 更新 CAV 的速度控制


# SUMO 仿真配置
SUMO_BINARY = "sumo-gui"  # 使用 sumo-gui 以便可视化
CONFIG_FILE = "Gaussian_trip.sumocfg"  # 仿真配置文件路径


# 启动 SUMO 仿真
def start_sumo():
    traci.start([SUMO_BINARY, "-c", CONFIG_FILE])


# 车辆参数
MAX_SPEED = 13.89  # 最大速度 (m/s)
MAX_ACCEL = 2.6  # 最大加速度 (m/s^2)
REACTION_TIME = 1.0  # 反应时间 (s)

# IDM模型的参数
params = {

    'v_0': MAX_SPEED,  # 期望速度30 m/s
    'delta': 4,  # 加速因子
    's_0': 2,  # 最小跟车距离
    'T_g': 1.5,  # 期望时间跟车距离
    'a_max': MAX_ACCEL,  # 最大加速度
    'b': 2.0,  # 最大减速度

    #下面两个参数是要进行计算的
    'v_lead': 18.0,  # 前车速度（假设为18 m/s）
    's': 50  # 当前车辆与前车的距离（假设为50米）

}

# 信号灯周期和时间参数
GREEN_TIME = 30  # 绿灯时间
RED_TIME = 30  # 红灯时间
CYCLE_TIME = GREEN_TIME + RED_TIME
dt = 0.2
# 初始化交叉口车辆列表
vehicles = []

class control_info:
    def __init__(self,vehicle_id):
        self.vehicle_id = vehicle_id
        self.time_when_cal = 0
        self.control_list = []
    def set_time(self,time):
        self.time_when_cal = time
    def control_list_clear(self):
        self.control_list = []
    def control_list_append(self,control_signal):
        self.control_list.append(control_signal)
    def control_signal(self,time,dt):
        return self.control_list[int((time-self.time_when_cal)/dt)]
    def control_list_show(self):
        return self.control_list


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


#获取刚刚离开某条lane的车辆
def get_vehicles_just_left(lane_previous_vehicles):
    """
    获取所有车道中刚刚离开车道的车辆 ID
    :param lane_previous_vehicles: 上一时刻每条车道的车辆列表字典
    :return: dict {laneID: [vehicleID, ...]}
    """
    just_left_vehicles = {}
    current_lane_vehicles = {}  # 记录当前时刻每条车道的车辆

    # 获取所有车道 ID
    lane_ids = traci.lane.getIDList()

    for laneID in lane_ids:
        # 获取当前车道的车辆列表
        current_vehicles = set(traci.lane.getLastStepVehicleIDs(laneID))
        current_lane_vehicles[laneID] = current_vehicles

        # 计算刚刚离开车道的车辆
        previous_vehicles = lane_previous_vehicles.get(laneID, set())
        just_left = previous_vehicles - current_vehicles
        just_left_vehicles[laneID] = list(just_left)

    return just_left_vehicles, current_lane_vehicles

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
                if traci.vehicle.getLanePosition(vehID) > 0.75 * lane_length
            ]
            last_quarter_vehicles[edgeID][laneID] = vehicles_in_last_quarter

    return edge_vehicles, last_quarter_vehicles

'''# 在检查交叉口相关车道上的车辆信息
def get_vehicles_in_range(intersection_id, traffic_light_to_lanes):

    #vehicle_ids = traci.vehicle.getIDList()
    lanes_pair = []
    for lanes_id in traffic_light_to_lanes[intersection_id]:
        lanes_leave_id =
    in_range_vehicles = []
    for vid in vehicle_ids:


            in_range_vehicles.append(vid)
    return in_range_vehicles'''

# 车辆动力学模型
def vehicle_dynamics(state, control, dt,type_car,params=None):
    """
    计算所有车辆下一时刻的状态
    HDV以IDM模型预测
    :param state: 当前状态 [位置, 速度]
    :param control: 当前加速度
    :param dt: 时间步长
    :return: 下一时刻状态
    """
    position, velocity = state
    if type_car == 'CAV':
        acceleration = control
        next_position = position + velocity * dt + 0.5 * acceleration * dt**2
        next_velocity = velocity + acceleration * dt
    elif type_car == 'HDV':
        # 对于HDV，使用IDM模型计算加速度
        v_0 = params['v_0']  # 期望速度（目标速度）
        delta = params['delta']  # 加速因子
        s_0 = params['s_0']  # 最小跟车距离
        T_g = params['T_g']  # 期望时间跟车距离
        a_max = params['a_max']  # 最大加速度
        b = params['b']  # 最大减速度
        # 计算当前车辆与前车之间的相对速度
        v_lead = params['v_lead']  # 前车速度（由环境或前车状态提供）
        s = params['s']  # 当前车辆与前车之间的距离

        delta_v = v_lead - velocity
        # 计算IDM模型的加速度
        s_star = s_0 + velocity * T_g + (velocity * delta_v) / (2 * np.sqrt(a_max * b))

        # 限制 s 的最小值避免除以零
        epsilon = 1e-6  # 一个非常小的值，防止除以零
        s_safe = max(s, epsilon)  # 保证 s 不为零

        # 限制计算中的比值，防止过大数值
        velocity_ratio = min(velocity / v_0, 10)
        s_ratio = min(s_star / s_safe, 10)

        acceleration = a_max * (1 - velocity_ratio ** delta - s_ratio ** 2)
        #acceleration = a_max * (1 - (velocity / v_0) ** delta - (s_star / s) ** 2)
        # 更新位置和速度
        next_position = position + velocity * dt + 0.5 * acceleration * dt ** 2
        next_velocity = velocity + acceleration * dt
    return np.array([next_position, next_velocity])

# 目标函数
def objective(control, state, target, weights, dt, N,type_info):
    """
    MPC的优化目标函数
    :param control: 控制变量（加速度序列）
    :param state: 当前状态
    :param target: 目标状态
    :param weights: 权重 [距离权重, 能耗权重]
    :param dt: 时间步长
    :param N: 优化步长
    :return: 总成本
    """
    w_distance, w_energy = weights
    total_cost = 0
    type_list = type_info[0]

    for t in range(N):
        for i in range(len(type_list)):
            if type_list[i] == 'CAV':
                state[i] = vehicle_dynamics(state[i], control[t * len(type_list) + i], dt, type_car=type_list[i],
                                            params=params)
                distance_cost = -w_distance * state[i][0]  # 行驶距离最大化
                energy_cost = w_energy * control[t]**2  # 能耗最小化
                total_cost += distance_cost + energy_cost
            elif type_list[i] == 'HDV':
                if i != len(type_list)-1:
                    params['v_lead'] = state[i + 1][1]
                    params['s'] = state[i + 1][0] - state[i][0]
                else:
                    params['v_lead'] = MAX_SPEED
                    params['s'] = 10


                state[i] = vehicle_dynamics(state[i], 0, dt, type_car=type_list[i],
                                            params=params)
                distance_cost = -w_distance * state[i][0]  # 行驶距离最大化
                distance_cost = -w_distance * state[i][0]  # 行驶距离最大化
                total_cost += distance_cost
    return total_cost

# 约束条件
def constraints(control, state, dt, N, vmin, vmax, amin, amax,type_info,now_lane,lane_towards):
    """
    定义优化问题的约束
    """
    constraints = []
    type_list = type_info[0]
    current_phase,remaining_time = get_remaining_phase_and_time(now_lane)
    lane_length = traci.lane.getLength(now_lane)
    state_houche = []
    for t in range(N):
        remaining_time = remaining_time - t*dt
        if remaining_time<=0:
            if current_phase == 'r':
                current_phase = 'g'
                remaining_time = 10
            elif current_phase == 'g':
                current_phase = 'y'
                remaining_time = 3
            elif current_phase == 'y':
                current_phase = 'r'
                remaining_time = 10

        for i in range(len(type_list)):
            if type_list[i] == 'CAV':
                state[i] = vehicle_dynamics(state[i], control[t * len(type_list) + i], dt,type_car=type_list[i],params=params)
                # 添加速度限制
                constraints.append({'type': 'ineq', 'fun': lambda c: vmax - state[i][1]})
                constraints.append({'type': 'ineq', 'fun': lambda c: state[i][1] - vmin})
                # 添加加速度限制
                constraints.append({'type': 'ineq', 'fun': lambda c: amax - control[t]})
                constraints.append({'type': 'ineq', 'fun': lambda c: control[t * len(type_list) + i] - amin})
                # 添加安全距离限制
                if t > 0:
                    if (current_phase == 'r' or current_phase == 'y') and state[i][0] < lane_length:
                        constraints.append({'type': 'ineq', 'fun': lambda c: lane_length - state[i][0]})
                    if i > 0:
                        if type_list[i-1] == 'CAV':
                            safety_distance = 2
                            constraints.append({'type': 'ineq', 'fun': lambda c: state[i][0] - state_houche[0] - safety_distance})
                        '''if len(just_left_vehicles[now_lane]) !=0 and just_left_vehicles[now_lane][0] in edge_vehicles[lane_towards[:-2]][lane_towards]:
                            state_qianche = vehicle_dynamics(state, control[t], dt, type_car='HDV', params=params)
                            constraints.append({'type': 'ineq', 'fun': lambda c: state[0] - safety_distance})'''
                state_houche = state
            elif type_list[i] == 'HDV':
                if i != len(type_list)-1:
                    params['v_lead'] = state[i + 1][1]
                    params['s'] = state[i + 1][0] - state[i][0]
                else:
                    params['v_lead'] = MAX_SPEED
                    params['s'] = 10
                state[i] = vehicle_dynamics(state[i], 0, dt,type_car=type_list[i],params=params)
                if t > 0:
                    if i > 0:
                        if type_list[i-1] == 'CAV':
                            safety_distance = 2
                            constraints.append({'type': 'ineq', 'fun': lambda c: state[i][0]-state_houche[0]-safety_distance})
                        '''if len(just_left_vehicles[now_lane]) !=0 and just_left_vehicles[now_lane][0] in edge_vehicles[lane_towards[:-2]][lane_towards]:
                            state_qianche = vehicle_dynamics(state, control[t], dt, type_car='HDV', params=params)
                            constraints.append({'type': 'ineq', 'fun': lambda c: state[0] - safety_distance})'''
                state_houche = state[i]
                    #edge_vehicles[lane_towards[:-2]][lane_towards]
                    #constraints.append({'type': 'ineq', 'fun': lambda c: state[0] - safety_distance})
    return constraints

# MPC主循环
def mpc_control(initial_state, target_state, weights, N, dt, bounds,type_info,now_lane,lane_towards,last_quarter_vehicles):
    """
    基于MPC优化得到最优控制序列
    :param initial_state: 初始状态 [位置, 速度]
    :param target_state: 目标状态 [位置, 速度]
    :param weights: 权重
    :param N: 优化步长
    :param dt: 时间步长
    :param bounds: 控制变量的上下限
    """
    type_list = type_info[0]

    # 初始猜测
    u0 = np.zeros(N*len(type_list))  # 初始控制序列
    # 约束
    cons = constraints(u0, initial_state.copy(), dt, N, bounds[2],bounds[3],bounds[0],bounds[1],type_info,now_lane,lane_towards)
    # 优化求解
    result = minimize(
        objective, u0, args=(initial_state.copy(), target_state, weights, dt, N,type_info),
        constraints=cons, method='SLSQP', bounds=[(-3.0, MAX_ACCEL)] * (N * len(type_list))
    )
    i = 0
    while i < N*len(type_list):
        for vehicle in last_quarter_vehicles[now_lane[:-2]][now_lane]:
            control_signal[vehicle].control_list_append(result.x[i])
            i += 1
    return result.x  # 返回最优控制序列




# 更新 CAV 速度控制
def update_cav_speeds(intersection_id,traffic_light_to_lanes,last_quarter_vehicles):
    start_time = time.time()
    #in_range_vehicles = get_vehicles_in_range("j3")
    #in_range_vehicles = get_vehicles_in_range(intersection_id,traffic_light_to_lanes)
    #in_range_vehicles_lanes = last_quarter_vehicles[intersection_id]
    now_time = traci.simulation.getTime()
    for lane_id in traffic_light_to_lanes[intersection_id]:
        num_CAV = 0
        num_HDV = 0
        initial_state = []  # 最后要转换为np.array() [位置,速度]
        type_list = []
        for temp in [i for i, value in enumerate(lane_adj_matrix[lane_index_dict[lane_id]]) if value != 0]:
            lane_towards = index_lane_dict[temp]
        #print(lane_id)
        print(last_quarter_vehicles)
        for vehicle_id in last_quarter_vehicles[lane_id[:-2]][lane_id]:
            print('1')
            if vehicle_id not in control_signal.keys():
                control_signal[vehicle_id] = control_info(vehicle_id)
            control_signal[vehicle_id].set_time(now_time)
            control_signal[vehicle_id].control_list_clear()
            if vehicle_id[0:3] == "CAV":
                num_CAV +=1
                type_list.append('CAV')
            elif vehicle_id[0:3] == "HDV":
                num_HDV +=1
                type_list.append('HDV')
            #CAV HDV的状态都要收集 但是只控制CAV
            state = [traci.vehicle.getLanePosition(vehicle_id),traci.vehicle.getSpeed(vehicle_id)]
                #target_speed = MAX_SPEED if is_green_light("j3") else 0
                #optimized_speed, optimized_accel = optimize_speed(vehicle_id, target_speed, MAX_SPEED, MAX_ACCEL, REACTION_TIME)
            initial_state.append(state)
        if len(initial_state) != 0:
            print('2')
            type_info = (type_list,num_CAV,num_HDV)
            initial_state = np.array(initial_state)
            mpc_control(initial_state, 0, weights=[1.0,0.5], N=50, dt=dt, bounds=(-3.0,MAX_ACCEL,0,MAX_SPEED),type_info=type_info,now_lane = lane_id,lane_towards = lane_towards,last_quarter_vehicles=last_quarter_vehicles)
    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"计算耗时: {elapsed_time:.4f} 秒")

def record_data(step):
    with open("traffic_data.csv", mode="a", newline="") as file:
        writer = csv.writer(file)
        vehicle_ids = traci.vehicle.getIDList()
        for vehicle_id in vehicle_ids:
            speed = traci.vehicle.getSpeed(vehicle_id)
            delay = max(0, MAX_SPEED - speed)
            emissions = traci.vehicle.getCO2Emission(vehicle_id)
            writer.writerow([step, vehicle_id, speed, delay, emissions])




edge_vehicles = {}

lane_previous_vehicles = {}  # 用于存储每条车道上一时刻的车辆
control_signal = {}
step = 0
if __name__ == "__main__":
    start_sumo()
    # 读取SUMO网络
    net = sumolib.net.readNet("Map_new.net.xml")

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

    # 创建并启动后台线程来更新 CAV 速度
    update_thread = threading.Thread(target=update_cav_control_thread_func, args=(traffic_light_to_lanes, lane_previous_vehicles, control_signal))
    update_thread.daemon = True  # 设置为守护线程，确保程序退出时自动关闭
    update_thread.start()

    Sampling_T = 5
    t_tick = 0

    while step < 3600*3:  # 仿真 3 小时
        sim_start = time.time()
        traci.simulationStep()  # 仿真步进
        sim_end = time.time()
        time_now = traci.simulation.getTime()
        vehicles_list = traci.vehicle.getIDList()
        for vehicle_id in vehicles_list:
            if vehicle_id not in control_signal.keys():
                continue
            try:
                if vehicle_id[0:3] == "CAV":
                    acc_control = control_signal[vehicle_id].control_signal(time_now, dt)
                    traci.vehicle.setAcceleration(vehicle_id, acc_control)

                    print(control_signal[vehicle_id].control_list_show())
                    print(f"{vehicle_id}已施加加速度控制量：{acc_control}")
            except:
                print(f"{vehicle_id}加速度施加失败")
                pass

        elapsed_time = sim_end - sim_start
        if elapsed_time < 1:
            time.sleep(1-elapsed_time)


            #update_cav_speeds('j3',traffic_light_to_lanes)  # 更新 CAV 的速度控制

        record_data(step)  # 记录数据
        step += 1

    traci.close()

    # 加载并分析数据
    data = pd.read_csv("traffic_data.csv")
    plt.plot(data["step"], data["delay"], label="Delay")
    plt.plot(data["step"], data["emissions"], label="Emissions")
    plt.legend()
    plt.xlabel("Simulation Step")
    plt.ylabel("Value")
    plt.title("Traffic Efficiency and Emissions Over Time")
    plt.show()

