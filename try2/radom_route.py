import traci
import random

# 定义路网中的边列表
edges = ["edge_1", "edge_2", "edge_3", "edge_4", "edge_5", "edge_6"]  # 替换为实际的 edge ID 列表
edges_in = ['j4tj5','j1tj5','j2tj6','j3tj7','j8tj7','j13tj12','j16tj12','j15tj11','j14tj10','j9tj10']
edges_out = ['j5tj4','j5tj1','j6tj2','j7tj3','j7tj8','j12tj13','j12tj16','j11tj15','j10tj14','j10tj9']
# 启动 SUMO 仿真
traci.start(["sumo", "-c", "your_config.sumocfg"])  # 替换为您的配置文件

step = 0
vehicle_count = 0

while step < 3600:  # 模拟1小时，即3600步
    traci.simulationStep()  # 执行一个仿真步骤

    # 每隔一定步数生成随机车辆
    if step % 10 == 0:  # 每10步添加一个车辆
        from_edge = random.choice(edges)
        to_edge = random.choice([edge for edge in edges if edge != from_edge])
        vehicle_id = f"vehicle_{vehicle_count}"

        # 添加车辆并设置起点和终点
        traci.vehicle.add(vehID=vehicle_id, routeID="", typeID="car")

        # 为车辆设置起点和终点，SUMO 将自动生成路线
        try:
            traci.vehicle.setRoute(vehicle_id, [from_edge, to_edge])
            vehicle_count += 1
            print(f"Vehicle {vehicle_id} assigned route from {from_edge} to {to_edge}")
        except traci.TraCIException:
            print(f"Could not find a route from {from_edge} to {to_edge} for {vehicle_id}")

    step += 1

# 关闭仿真
traci.close()
