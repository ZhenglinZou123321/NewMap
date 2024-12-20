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
import copy
import cvxpy as cp

def update_cav_control_thread_func(traffic_light_to_lanes, lane_previous_vehicles, control_signal,N,dt,L_safe):
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
            update_cav_speeds('j3', traffic_light_to_lanes,last_quarter_vehicles,N,dt,L_safe)  # 更新 CAV 的速度控制



# SUMO 仿真配置
SUMO_BINARY = "sumo-gui"  # 使用 sumo-gui 以便可视化
CONFIG_FILE = "Gaussian_trip.sumocfg"  # 仿真配置文件路径


# 启动 SUMO 仿真
def start_sumo():
    traci.start([SUMO_BINARY, "-c", CONFIG_FILE,"--step-length",'0.2'])


# 车辆参数
MAX_SPEED = 15  # 最大速度 (m/s)
MIN_SPEED = 0
MAX_ACCEL = 10  # 最大加速度 (m/s^2)
MIN_ACCEL = -10
REACTION_TIME = 1.0  # 反应时间 (s)

# IDM模型的参数
params = {

    'v_0': MAX_SPEED,  # 期望速度30 m/s
    'delta': 4,  # 加速因子
    's_0': 2,  # 最小跟车距离
    'T_g': 1.5,  # 期望时间跟车距离
    'a_max': MAX_ACCEL,  # 最大加速度
    'b': MIN_ACCEL,  # 最大减速度

    #下面两个参数是要进行计算的
    'v_lead': 18.0,  # 前车速度（假设为18 m/s）
    's': 50  # 当前车辆与前车的距离（假设为50米）

}

# 信号灯周期和时间参数
GREEN_TIME = 30  # 绿灯时间
RED_TIME = 30  # 红灯时间
CYCLE_TIME = GREEN_TIME + RED_TIME
dt = 0.2
N=40
L_safe = 2
# 初始化交叉口车辆列表
vehicles = []

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
        return self.control_list[int((time-self.time_when_cal)/dt)]
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
                if traci.vehicle.getLanePosition(vehID) > 0.5 * lane_length
            ]
            last_quarter_vehicles[edgeID][laneID] = vehicles_in_last_quarter #idx=0的车是最靠近lane终点的，以此类推

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
        print(last_quarter_vehicles)
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


def construct_block_diagonal_matrix(alpha, N):
    # alpha 是一个 3x2 的矩阵
    rows_a, cols_a = alpha.shape  # 获取 alpha 的形状

    #N是对角块的数量

    # 创建一个 rows x cols 的零矩阵
    result = np.zeros((rows_a*N, cols_a*N))

    # 填充对角线
    for i in range(N):
        result[rows_a * i:rows_a * (i + 1), cols_a * i:cols_a * (i + 1)] = alpha

    return result


def construct_HDV_block_matrix(num_CAV, m):
    #m是紧跟这个HDV的CAV的序号（在CAV里的序号）
    m = m+1 #序号为0的cav其实对应第1个块
    # 创建一个 2 x (2 * num_CAV) 的零矩阵
    result = np.zeros((2, 2 * num_CAV))

    # 创建一个 2x2 的单位矩阵
    identity_matrix = np.eye(2)

    # 将第 m 个块设置为单位矩阵
    # 每个块的宽度是 2，因此第 m 个块开始的位置是 2*(m-1)
    result[:, 2 * (m - 1): 2 * m] = identity_matrix

    return result

def construct_CAV_block_matrix(num_CAV, m):
    #m是紧跟这个HDV的CAV的序号（在CAV里的序号）
    m = m+1 #序号为0的cav其实对应第1个块
    # 创建一个 2 x (2 * num_CAV) 的零矩阵
    result = np.zeros((2, 2 * num_CAV))

    # 创建一个 2x2 的单位矩阵
    identity_matrix = np.eye(2)

    # 将第 m 个块设置为单位矩阵
    # 每个块的宽度是 2，因此第 m 个块开始的位置是 2*(m-1)
    result[:, 2 * (m - 1): 2 * m] =  identity_matrix
    m = m - 1
    result[:, 2 * (m - 1): 2 * m] = -1 *identity_matrix

    return result

def QP_solver(initial_state_CAV,initial_state_HDV,vehicles_list_this_lane,N,dt,v_max,v_min,a_max,a_min,L_safe,lane_now,CAV_id_list,HDV_id_list):
    v_best = 13
    CAV_id_list.reverse()
    HDV_id_list.reverse()
    vehicles_list_this_lane.reverse()
    initial_state_CAV = np.flipud(initial_state_CAV)
    initial_state_HDV = np.flipud(initial_state_HDV)
    X0 = initial_state_CAV.flatten()

    X0 = X0.reshape(-1, 1) #变成列向量

    alpha_c = np.array([[1,dt],
                        [0,1]])
    beta_c = np.array([[0.5*dt*dt],[dt]])
    num_CAV = len(initial_state_CAV)

    A = construct_block_diagonal_matrix(alpha_c,num_CAV)

    B = construct_block_diagonal_matrix(beta_c,num_CAV)

    h = np.array([[0,0],
                 [0,1]])
    epsilon = np.array([0,1])

    #H_t = np.block([[h if i == j else np.zeros_like(h) for j in range(num_CAV)] for i in range(num_CAV)])
    H_t = construct_block_diagonal_matrix(h,num_CAV)

    C_t = np.hstack([epsilon]*(num_CAV))

    Q = np.block([[H_t if i == j else np.zeros_like(H_t) for j in range(N)] for i in range(N)])

    C = np.hstack([C_t]*(N))
    C = C.reshape(1, -1) #变成行向量

    A_tilde = np.vstack([np.linalg.matrix_power(A, i) for i in range(1, N + 1)])

    B_tilde = np.zeros((2*N*num_CAV,N*num_CAV))

    # 计算B_tilde
    for i in range(N):
        for j in range(i + 1):
            # 计算 A^(i-j) * B
            power_A = np.linalg.matrix_power(A, i - j)
            # 将 A^(i-j) * B 填充到相应的位置
            B_tilde[2 *num_CAV* i: 2 *num_CAV* (i + 1), num_CAV * j: num_CAV * (j + 1)] = np.dot(power_A, B)

    #QP问题的参数
    half_H_qp = B_tilde.T @ Q @ B_tilde

    C_T = 2*X0.T @ A_tilde.T @ Q @ B_tilde - 2*v_best * C @ B_tilde

    u = cp.Variable((num_CAV*N,1))




    #不等式约束

    constraints = []

    constraints.append(u>=a_min)
    constraints.append(u<=a_max)
    #避碰
    HDVcons_left = np.array([1,0])
    CAVcons_left = np.array([1,0])
    phase,remaining_time = get_remaining_phase_and_time(lane_now)

    #生成红绿灯约束矩阵
    big_M = 9999
    traffic_signal_list = []
    for i in range(N):
        if remaining_time >= 0 :
            pass
        else:
            if phase == 'r':
                phase = 'g'
                remaining_time = 10
            elif phase == 'y':
                phase = 'r'
                remaining_time = 10
            elif phase == 'g':
                phase = 'y'
                remaining_time = 3
        if phase == 'r':
            traffic_signal_list.append(0)
        elif phase == 'y':
            traffic_signal_list.append(0)
        elif phase == 'g':
            traffic_signal_list.append(big_M)
        remaining_time = remaining_time - dt

    signal_matrix = np.array(traffic_signal_list).reshape(-1, 1)


    for (idx,vehicle_id) in enumerate(vehicles_list_this_lane):
        if idx == 0 and (vehicle_id in CAV_id_list): #0 是最靠近路口的
            print('加入红灯停约束')
            #红灯停的约束
            HDVcons_right = construct_HDV_block_matrix(num_CAV,0)
            HDVcons = HDVcons_left @ HDVcons_right
            HDVcons = HDVcons.reshape(1,-1)
            HDVconsM = construct_block_diagonal_matrix(HDVcons, N)
            lane_length = traci.lane.getLength(lane_now)
            LHDVsafe = np.vstack([lane_length-L_safe] * N)
            Inequal_with_u = HDVconsM @ B_tilde
            Inequal_right = LHDVsafe - HDVconsM @ A_tilde @ X0 + signal_matrix

            #硬约束
            # 加入不等式约束~~~~~~~~~~~~~
            #constraints.append(Inequal_with_u @ u <= Inequal_right)
            print(Inequal_right)

            #软约束
            #加入等式约束 同时更改目标函数
            Soft = cp.Variable((N,1), nonneg=False)
            constraints.append(Inequal_with_u @ u + Soft <= Inequal_right)

            #constraints.append(u <= np.linalg.inv(Inequal_with_u) @ Inequal_right)

            continue
        if vehicle_id in CAV_id_list:
            if vehicles_list_this_lane[idx-1] in HDV_id_list:
                HDVcons = HDVcons_left @ construct_HDV_block_matrix(num_CAV,CAV_id_list.index(vehicle_id))
                HDVcons = HDVcons.reshape(1, -1)
                HDVconsM = construct_block_diagonal_matrix(HDVcons,N)
                LHDVsafe = np.vstack([initial_state_HDV[HDV_id_list.index(vehicles_list_this_lane[idx-1])][0]-L_safe]*N)
                Inequal_with_u = HDVconsM @ B_tilde
                Inequal_right = LHDVsafe - HDVconsM @ A_tilde @ X0
                # 加入不等式约束~~~~~~~~~~~~~
                constraints.append(Inequal_with_u @ u <= Inequal_right)
            elif vehicles_list_this_lane[idx-1] in CAV_id_list:
                CAVcons = CAVcons_left @ construct_CAV_block_matrix(num_CAV,CAV_id_list.index(vehicle_id))
                CAVcons = CAVcons.reshape(1, -1)
                CAVconsM = construct_block_diagonal_matrix(CAVcons,N)
                LCAVsafe = np.vstack([-L_safe]*N)
                Inequal_with_u = CAVconsM @ B_tilde
                Inequal_right = -1 *CAVconsM @ A_tilde @ X0 + LCAVsafe
                # 加入不等式约束~~~~~~~~~~~~~
                constraints.append(Inequal_with_u @ u <= Inequal_right)

    #限速
    speed_little_matrix = np.array([0,1]).reshape(1,-1)
    VmaxM = np.vstack([v_max]*(num_CAV*N))
    VminM = np.vstack([v_min]*(num_CAV*N))
    Vtake = construct_block_diagonal_matrix(speed_little_matrix,num_CAV)
    VtakeM = construct_block_diagonal_matrix(Vtake,N)

    #v_max:
    Inequal_with_u = VtakeM @ B_tilde
    Inequal_right = VmaxM - VtakeM @ A_tilde @ X0
    #加入不等式约束~~~~~
    constraints.append(Inequal_with_u @ u <= Inequal_right)
    #v_min:
    Inequal_with_u = -1 * VtakeM @ B_tilde
    Inequal_right = VtakeM @ A_tilde @ X0 - VminM
    # 加入不等式约束~~~~~~~~~~~~~
    constraints.append(Inequal_with_u @ u <= Inequal_right)

    #objective = cp.Minimize(cp.quad_form(u,half_H_qp)+ C_T @ u)   硬约束
    objective = 0
    #objective = cp.Minimize(cp.quad_form(u, half_H_qp) + C_T @ u + 100 * cp.norm(Soft, 2))
    try:
        print('有CAV位于首位')
        objective = cp.Minimize(cp.quad_form(u, half_H_qp) + C_T @ u + 100*cp.norm(Soft,2))
    except:
        print('无CAV位于首位')
        objective = cp.Minimize(cp.quad_form(u, half_H_qp) + C_T @ u)

    problem = cp.Problem(objective, constraints)
    problem.solve(verbose=True)
    # 输出结果
    print("Solver status:", problem.status)
    if problem.status == 'infeasible':
        for i, constraint in enumerate(constraints):
            try:
                lhs = constraint.args[0].value
                rhs = constraint.args[1].value
                print(f"Constraint {i}: {lhs} <= {rhs}")
            except Exception as e:
                print(f"Constraint {i} could not be evaluated: {e}")
        print('11111111111111111111')
    print("Optimal value:", problem.value)
    print("Optimal u:", u.value)

    i = 0
    while i < N*len(CAV_id_list):
        for vehicle in CAV_id_list:
            control_signal[vehicle].control_list_append(u.value[i])
            i += 1

#表面上是数学优化问题上约束满足的问题。所以我从数学上考虑加入软约束。但是这个问题的根本原因在于车芸协同控制/云边协同控制的最终的决定权在车还是在云。这种是一种强动态
#环境下必须要考虑，但大家容易忽略的问题。我们认为在车，因为他直接面对动态环境。这是机制性的问题。只要有时延存在，就一定会遇到，只是概率大小。
#根本上解决，应该把决定权给车。即车端如何融合云端信号。如何研究车云融合控制。CBF 不偏离云端控制的前提下，保证现实动态约束的安全性。
#在云端控制的时候，考虑信号灯未来相位的变化。目前，信号灯的未来状态并没有传递给云端控制器。即，车路云协同还没有深层。尤其是在MPC预测控制框架下。他们数据和处理事实上都在
#共同的边缘云设备上，不传递是不应该的。这样会形成混合整数规划问题。无论是可解性还是计算时间，都是新的挑战。参考邓宇的研究。
#要尽可能地降低采样时间，不要让人为的时延大于计算时延+通信时延。不然这个时延是人为设置引起的。这个和软阈值约束不矛盾，可以独立优化。
#对比实验中，把考虑了车路协同和完全没有考虑的对比。体现机制性的区别。insight也体现出来了。很妙。国内的期刊就是喜欢这种以小见大的感觉。佐证政策。


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
    update_thread = threading.Thread(target=update_cav_control_thread_func, args=(traffic_light_to_lanes, lane_previous_vehicles, control_signal,N,dt,L_safe))
    update_thread.daemon = True  # 设置为守护线程，确保程序退出时自动关闭
    #update_thread.start()

    Sampling_T = 5
    t_tick = 0

    while step < 3600*3:  # 仿真 3 小时
        sim_start = time.time()
        traci.simulationStep()  # 仿真步进
        sim_end = time.time()
        time_now = traci.simulation.getTime()
        vehicles_list = traci.vehicle.getIDList()
        if step%2==0:
            #global last_quarter_vehicles
            edge_vehicles, last_quarter_vehicles = get_all_edge_vehicles_and_last_quarter()
            print(last_quarter_vehicles)
            # 获取刚刚离开每个车道的车辆
            just_left_vehicles, lane_previous_vehicles = get_vehicles_just_left(lane_previous_vehicles)
            update_cav_speeds('j3', traffic_light_to_lanes,last_quarter_vehicles,N,dt,L_safe)  # 更新 CAV 的速度控制
        for vehicle_id in vehicles_list:
            if vehicle_id not in control_signal.keys():
                continue
            try:
                if vehicle_id[0:3] == "CAV":
                    traci.vehicle.setSpeedMode(vehicle_id, 00000) #关闭跟驰模型
                    acc_control = control_signal[vehicle_id].control_signal(time_now, dt)
                    print(acc_control)
                    traci.vehicle.setAcceleration(vehicle_id, acc_control,1)

                    print(control_signal[vehicle_id].control_list_show())
                    print(f"{vehicle_id}已施加加速度控制量：{acc_control}")
            except Exception as e:
                traci.vehicle.setSpeedMode(vehicle_id, 00000) #关闭跟驰模型
                if vehicle_id[0:3] == "CAV":
                    print(f"{vehicle_id}施加了 0 ")
                    traci.vehicle.setAcceleration(vehicle_id, 0,1)
                # 捕获异常并打印详细信息
                print(f"An error occurred: {e}")
                print(f"{vehicle_id}加速度施加失败")
                pass

        elapsed_time = sim_end - sim_start
        if elapsed_time < 0.2:
            time.sleep(0.2-elapsed_time)


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

