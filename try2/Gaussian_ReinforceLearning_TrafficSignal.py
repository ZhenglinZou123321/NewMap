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

def get_state(intersection_id,Intersection_Edge_Dict,junc_dict,junc_m):
    state = []
    traffic_signal_dict = {'r':0,'g':1}
    checked_lane = []
    dentisy_self = 0#不处理右转车道
    dentisy_from = 0
    dentisy_to = 0#目标车道的车辆密度
    #for edge in Intersection_Edge_Dict[intersection_id]['in']:
    for (index,lane) in enumerate(traci.trafficlight.getControlledLanes(intersection_id)):
        if lane  in checked_lane:
            continue
        checked_lane.append(lane)
        if lane[-1]=='0':#不处理右转车道
            continue
        current_phase_state = traci.trafficlight.getRedYellowGreenState(intersection_id)
        signal_state = traffic_signal_dict[current_phase_state[index].lower()]
        if signal_state == 1:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
            dentisy_self += vehicle_occupancy_length/traci.lane.getLength(lane)
            links = traci.lane.getLinks(lane)
            lane_index = lane_index_dict[lane]
            to_list = np.nonzero(junc_m[lane_index])[0] #这个lane要去的lane的索引
            from_list = np.nonzero(junc_m[:, lane_index])[0]#来这个lane的索引

            for one_lane in to_list:
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_to += vehicle_occupancy_length/traci.lane.getLength(one_lane)

            for one_lane in from_list:
                vehicle_ids = traci.lane.getLastStepVehicleIDs(one_lane)
                vehicle_occupancy_length = sum(traci.vehicle.getLength(vehicle_id) for vehicle_id in vehicle_ids)
                dentisy_from += vehicle_occupancy_length/traci.lane.getLength(one_lane)

        #waiting_time = traci.lane.getWaitingTime(lane)

        # 将等待车辆数量和等待时间添加到状态向量
        # 获取车道上所有车辆的 ID 列表
        first_vehicle_delay = 0
        waiting_vehicle_count = traci.lane.getLastStepHaltingNumber(lane)
        try:
            vehicle_ids = traci.lane.getLastStepVehicleIDs(lane)
            # 获取第一辆车的 ID
            first_vehicle_id = vehicle_ids[0]
            first_vehicle_delay = traci.vehicle.getWaitingTime(first_vehicle_id)
        except:
            first_vehicle_delay = 0

        #state.extend([dentisy_from, dentisy_to])
    state = [dentisy_from, dentisy_to]
    return np.array(state, dtype=np.float32)



def get_reward(intersection_id,agent,Action_list):
    reward = 0
    checked_lane = []
    traffic_signal_dict = {'r': 0, 'g': 1}
    for (index,lane) in enumerate(traci.trafficlight.getControlledLanes(intersection_id)):
        if lane  in checked_lane:
            continue
        checked_lane.append(lane)
        if lane[-1]=='0':#不处理右转车道
            continue
        current_phase_state = traci.trafficlight.getRedYellowGreenState(intersection_id)
        signal_state = traffic_signal_dict[current_phase_state[index].lower()]
        if signal_state == 1:
            waiting_time = 2*traci.lane.getWaitingTime(lane)
        else:
            waiting_time = traci.lane.getWaitingTime(lane)
        reward -= waiting_time

    reward -= Action_list[agent.action]
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
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
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



def get_remaining_phase_time(traffic_light_id): #获取信号灯剩余时间
    # 获取当前仿真时间
    current_time = traci.simulation.getTime()
    # 获取下一个信号切换的时间
    next_switch_time = traci.trafficlight.getNextSwitch(traffic_light_id)
    # 计算剩余时间
    remaining_time = next_switch_time - current_time
    return max(remaining_time, 0)  # 防止负值



# 启动SUMO仿真
traci.start(["sumo-gui", "-c", "Gaussian_trip.sumocfg","--start"])


with open("Graph/junction_index.json", "r") as f:
    junction_index_dict = json.load(f)

with open("Graph/lane_index.json", "r") as f:
    lane_index_dict = json.load(f)

df = pd.read_csv("Graph/junction_adj_matrix.csv", index_col=0)
junc_adj_matrix = df.values

df = pd.read_csv("Graph/lane_adj_matrix.csv", index_col=0)
lane_adj_matrix = df.values



with open('traffic_data_gaussian.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'road_id', 'vehicle_count', 'average_speed'])

# 仿真循环
Action_list = [10,15,20,25,30,35,40]
Intelligent_Sigal_List = ['j5', 'j6', 'j7', 'j10','j11','j12']

Intersection_Edge_Dict = {'j20': {'in': ['j26tj20', 'j21tj20', 'j13tj20', 'j19tj20'], 'out': ['j20tj26', 'j20tj21', 'j20tj13', 'j20tj19']}}
Agent_List = {}
Least_Check_Time = 5
train_batchsize = 32
train_gap = 20
step = 0
arrived_vehicles = set()
vehicle_depart_times = {}
total_travel_time = 0

heat_gap = 300

for Traffic_Signal_id in Intelligent_Sigal_List:
    Agent_List[Traffic_Signal_id] = DDQNAgent(state_size=2 ,action_size=7)
    try:
        Agent_List[Traffic_Signal_id].q_network.load_state_dict(torch.load(f'models/{Traffic_Signal_id}_model.pth'))
        Agent_List[Traffic_Signal_id].target_network.load_state_dict(torch.load(f'models/{Traffic_Signal_id}_model.pth'))
    except:
        pass

waiting_time_dict={}
while step < 3600*10:  # 仿真时间，例如1小时
    traci.simulationStep()  # 每步执行仿真
    for Traffic_Signal_id in Intelligent_Sigal_List:
        if traci.trafficlight.getPhase(Traffic_Signal_id) in [0,2] and get_remaining_phase_time(Traffic_Signal_id)<Least_Check_Time and Agent_List[Traffic_Signal_id].CheckOrNot is False:
            next_state = get_state(Traffic_Signal_id,Intersection_Edge_Dict,junction_index_dict,junc_adj_matrix)
            reward = get_reward(Traffic_Signal_id,Agent_List[Traffic_Signal_id],Action_list)
            Agent_List[Traffic_Signal_id].memory.append((Agent_List[Traffic_Signal_id].state, Agent_List[Traffic_Signal_id].action, reward, next_state))
            Agent_List[Traffic_Signal_id].step += 1
            Agent_List[Traffic_Signal_id].action = Agent_List[Traffic_Signal_id].act(next_state)
            Agent_List[Traffic_Signal_id].state = next_state
            #traci.trafficlight.setPhase(Traffic_Signal_id, Agent_List[action])
            Agent_List[Traffic_Signal_id].reward_delta = reward
            Agent_List[Traffic_Signal_id].total_reward += reward
            Agent_List[Traffic_Signal_id].CheckOrNot = True

        if traci.trafficlight.getPhase(Traffic_Signal_id) in [0,2] and Agent_List[Traffic_Signal_id].CheckOrNot is True and get_remaining_phase_time(Traffic_Signal_id)>Least_Check_Time:

            temp_duration = get_remaining_phase_time(Traffic_Signal_id)
            traci.trafficlight.setPhaseDuration(Traffic_Signal_id, float(Action_list[Agent_List[Traffic_Signal_id].action]))
            #print(f"Agent: {Traffic_Signal_id} 原:{temp_duration} 现在:{get_remaining_phase_time(Traffic_Signal_id)} ")
            Agent_List[Traffic_Signal_id].CheckOrNot = False
            if Agent_List[Traffic_Signal_id].step%train_gap == 0 and Agent_List[Traffic_Signal_id].step>=train_batchsize:
                Agent_List[Traffic_Signal_id].train(train_batchsize)
                Agent_List[Traffic_Signal_id].update_target_network()
                torch.save(Agent_List[Traffic_Signal_id].q_network.state_dict(), f'models/{Traffic_Signal_id}_model.pth')
                print(f"Agent: {Traffic_Signal_id} Reward = {Agent_List[Traffic_Signal_id].reward_delta} Trained_time = {Agent_List[Traffic_Signal_id].Trained_time}")

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
