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

import re
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







# 启动SUMO仿真
traci.start(["sumo-gui", "-c", "Gaussian_trip.sumocfg","--start"])

net = sumolib.net.readNet("Map_new.net.xml")  # 替换为您的 .net.xml 文件路径

with open("Graph/junction_index.json", "r") as f:
    junction_index_dict = json.load(f)

with open("Graph/lane_index.json", "r") as f:
    lane_index_dict = json.load(f)

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
while step < 3600*10:  # 仿真时间，例如1小时
    traci.simulationStep()  # 每步执行仿真
    vehicle_ids = traci.vehicle.getIDList()
    for vehicle_id in vehicle_ids:
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


