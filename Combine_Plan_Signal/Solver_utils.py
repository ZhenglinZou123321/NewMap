#encoding: utf-8
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
import os
import sys


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

def QP_solver(initial_state_CAV,initial_state_HDV,vehicles_list_this_lane,N,dt,v_max,v_min,a_max,a_min,L_safe,lane_now,CAV_id_list,HDV_id_list,control_signal):
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
    big_M = 999
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
            LHDVsafe = np.vstack([lane_length-L_safe-6] * N)
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
            #idm_acc = idm_acceleration(current_speed, front_vehicle_speed, gap, front_vehicle_id=None):

            if vehicles_list_this_lane[idx-1] in HDV_id_list:
                #idm_acc = idm_acceleration(initial_state_CAV[vehicle_id][0], initial_state_HDV[vehicles_list_this_lane[idx-1]][0],initial_state_HDV[vehicles_list_this_lane[idx-1]][1]-initial_state_CAV[vehicle_id][1], front_vehicle_id=None)
                HDVcons = HDVcons_left @ construct_HDV_block_matrix(num_CAV,CAV_id_list.index(vehicle_id))
                HDVcons = HDVcons.reshape(1, -1)
                HDVconsM = construct_block_diagonal_matrix(HDVcons,N)
                LHDVsafe = np.vstack([initial_state_HDV[HDV_id_list.index(vehicles_list_this_lane[idx-1])][0]-L_safe]*N)
                Inequal_with_u = HDVconsM @ B_tilde
                Inequal_right = LHDVsafe - HDVconsM @ A_tilde @ X0
                # 加入不等式约束~~~~~~~~~~~~~
                constraints.append(Inequal_with_u @ u <= Inequal_right)
                #IDM lower_control


            elif vehicles_list_this_lane[idx-1] in CAV_id_list:
                #idm_acc = idm_acceleration(initial_state_CAV[vehicle_id][0], initial_state_CAV[vehicles_list_this_lane[idx-1]][0],initial_state_CAV[vehicles_list_this_lane[idx-1]][1]-initial_state_CAV[vehicle_id][1], front_vehicle_id=None)
                CAVcons = CAVcons_left @ construct_CAV_block_matrix(num_CAV,CAV_id_list.index(vehicle_id))
                CAVcons = CAVcons.reshape(1, -1)
                CAVconsM = construct_block_diagonal_matrix(CAVcons,N)
                LCAVsafe = np.vstack([-L_safe]*N)
                Inequal_with_u = CAVconsM @ B_tilde
                Inequal_right = -1 *CAVconsM @ A_tilde @ X0 + LCAVsafe
                # 加入不等式约束~~~~~~~~~~~~~
                constraints.append(Inequal_with_u @ u <= Inequal_right)
            #Lower_control_signal[vehicle_id] = idm_acc

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
        objective = cp.Minimize(cp.quad_form(u, half_H_qp) + C_T @ u + 10*cp.norm(Soft,2))
    except:
        print('无CAV位于首位')
        objective = cp.Minimize(cp.quad_form(u, half_H_qp) + C_T @ u)

    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.GUROBI, verbose=True, solver_opts={"LogFile": "gurobi.log"})
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
    try:
        while i < N*len(CAV_id_list):
            for vehicle in CAV_id_list:
                control_signal[vehicle].control_list_append(u.value[i])
                i += 1
        print(f'{lane_now} 求解成功')
        return control_signal
    except:
        print(f'{lane_now} 求解失败')
        return control_signal
        pass

#表面上是数学优化问题上约束满足的问题。所以我从数学上考虑加入软约束。但是这个问题的根本原因在于车芸协同控制/云边协同控制的最终的决定权在车还是在云。这种是一种强动态
#环境下必须要考虑，但大家容易忽略的问题。我们认为在车，因为他直接面对动态环境。这是机制性的问题。只要有时延存在，就一定会遇到，只是概率大小。
#根本上解决，应该把决定权给车。即车端如何融合云端信号。如何研究车云融合控制。CBF 不偏离云端控制的前提下，保证现实动态约束的安全性。
#在云端控制的时候，考虑信号灯未来相位的变化。目前，信号灯的未来状态并没有传递给云端控制器。即，车路云协同还没有深层。尤其是在MPC预测控制框架下。他们数据和处理事实上都在
#共同的边缘云设备上，不传递是不应该的。这样会形成混合整数规划问题。无论是可解性还是计算时间，都是新的挑战。参考邓宇的研究。
#要尽可能地降低采样时间，不要让人为的时延大于计算时延+通信时延。不然这个时延是人为设置引起的。这个和软阈值约束不矛盾，可以独立优化。
#对比实验中，把考虑了车路协同和完全没有考虑的对比。体现机制性的区别。insight也体现出来了。很妙。国内的期刊就是喜欢这种以小见大的感觉。佐证政策。
