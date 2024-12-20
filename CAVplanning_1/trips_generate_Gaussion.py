import numpy as np


# 配置参数
total_hours = 3            # 总仿真时间
total_seconds = total_hours * 3600
mean_time = total_seconds / 2  # 高斯分布的均值，5小时为高峰
std_dev = total_seconds / 3    # 标准差，控制分布的宽度
num_vehicles = 400          # 生成的车辆数量
Permeability = 0.3


# 生成符合高斯分布的出发时间序列
depart_times = np.random.normal(loc=mean_time, scale=std_dev, size=num_vehicles)
depart_times = np.clip(depart_times, 0, total_seconds)  # 将出发时间限制在 [0, total_seconds] 范围内
depart_times = np.sort(depart_times)  # 排序出发时间


# 定义出发和到达边
entry_edges = ['j2tj3','j5tj3','j6tj4','j8tj1']    # 替换为实际的入口边
exit_edges =  ['j3tj2','j3tj5','j4tj6','j1tj8']   # 替换为实际的出口边


time_list = {}
# 写入到 .trips.xml 文件
with open("custom_gaussian.trips.xml", "w") as f:
    # 定义车辆类型
    f.write('<types>\n')
    f.write('    <type id="CAV" length="3" />\n')  # CAV车长为4.5米
    f.write('    <type id="HDV" length="3" />\n')   # HDV车长为12米
    f.write('</types>\n')
    f.write('<routes>\n')
    for i, depart_time in enumerate(depart_times):

        from_edge = np.random.choice(entry_edges)
        from_edge_index = entry_edges.index(from_edge)
        to_edge = np.random.choice(exit_edges)
        to_edge_index = exit_edges.index(to_edge)
        while abs(from_edge_index - to_edge_index)<=1:
            from_edge = np.random.choice(entry_edges)
            from_edge_index = entry_edges.index(from_edge)
            to_edge = np.random.choice(exit_edges)
            to_edge_index = exit_edges.index(to_edge)
        if i%(1//Permeability)==0:
            vehicle_type = 'CAV'
        else:
            vehicle_type = 'HDV'

        f.write(f'    <trip id="{vehicle_type}_{i}" depart="{int(depart_time)}" from="{from_edge}" to="{to_edge} type="{vehicle_type}" />\n')
        if str(depart_time//3600+1) not in time_list.keys():
            time_list[str(depart_time//3600+1)] = 1
        else:
            time_list[str(depart_time//3600+1)] += 1
    f.write('</routes>\n')

for i in time_list.keys():
    print(f"hour {i} counts: {time_list[i]}")