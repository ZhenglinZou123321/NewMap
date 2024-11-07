import numpy as np

# 配置参数
total_hours = 10            # 总仿真时间
total_seconds = total_hours * 3600
mean_time = total_seconds / 2  # 高斯分布的均值，5小时为高峰
std_dev = total_seconds / 6    # 标准差，控制分布的宽度
num_vehicles = 10000           # 生成的车辆数量

# 生成符合高斯分布的出发时间序列
depart_times = np.random.normal(loc=mean_time, scale=std_dev, size=num_vehicles)
depart_times = np.clip(depart_times, 0, total_seconds)  # 将出发时间限制在 [0, total_seconds] 范围内
depart_times = np.sort(depart_times)  # 排序出发时间

# 定义出发和到达边
entry_edges = ['j4tj5','j1tj5','j2tj6','j3tj7','j8tj7','j13tj12','j16tj12','j15tj11','j14tj10','j9tj10']    # 替换为实际的入口边
exit_edges =  ['j5tj4','j5tj1','j6tj2','j7tj3','j7tj8','j12tj13','j12tj16','j11tj15','j10tj14','j10tj9']   # 替换为实际的出口边


time_list = {}
# 写入到 .trips.xml 文件
with open("custom_gaussian.trips.xml", "w") as f:
    f.write('<routes>\n')
    for i, depart_time in enumerate(depart_times):

        from_edge = np.random.choice(entry_edges)
        from_edge_index = entry_edges.index(from_edge)
        to_edge = np.random.choice(exit_edges)
        to_edge_index = exit_edges.index(to_edge)
        while abs(from_edge_index - to_edge_index)<=3:
            from_edge = np.random.choice(entry_edges)
            from_edge_index = entry_edges.index(from_edge)
            to_edge = np.random.choice(exit_edges)
            to_edge_index = exit_edges.index(to_edge)
        f.write(f'    <trip id="vehicle_{i}" depart="{int(depart_time)}" from="{from_edge}" to="{to_edge}" />\n')
        if str(depart_time//3600+1) not in time_list.keys():
            time_list[str(depart_time//3600+1)] = 1
        else:
            time_list[str(depart_time//3600+1)] += 1
    f.write('</routes>\n')

for i in time_list.keys():
    print(f"hour {i} counts: {time_list[i]}")