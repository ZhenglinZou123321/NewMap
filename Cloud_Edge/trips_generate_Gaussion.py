import numpy as np


# 配置参数
total_hours = 5            # 总仿真时间
total_seconds = total_hours * 3600
mean_time = total_seconds / 2  # 高斯分布的均值，5小时为高峰
std_dev = total_seconds / 1    # 标准差，控制分布的宽度
num_vehicles = 20000          # 生成的车辆数量
Permeability = 0.3


# 生成符合高斯分布的出发时间序列
depart_times = np.random.normal(loc=mean_time, scale=std_dev, size=num_vehicles)
#min_depart_time = np.min(depart_times)
#depart_times = depart_times - min_depart_time
# 筛选出在 [0, total_seconds] 范围内的出发时间
depart_times = depart_times[depart_times >= 0]  # 保证出发时间大于等于0
depart_times = depart_times[depart_times <= total_seconds]  # 保证出发时间小于等于 total_seconds
depart_times = np.sort(depart_times)  # 排序出发时间


# 定义出发和到达边
entry_edges = ['j4tj5','j1tj5','j2tj6','j3tj7','j8tj7','j13tj12','j16tj12','j15tj11','j14tj10','j9tj10']    # 替换为实际的入口边
exit_edges =  ['j5tj4','j5tj1','j6tj2','j7tj3','j7tj8','j12tj13','j12tj16','j11tj15','j10tj14','j10tj9']   # 替换为实际的出口边



time_list = {}
# 写入到 .trips.xml 文件
with open("custom_gaussian.trips.xml", "w") as f:

    f.write('<routes>\n')
    # 定义车辆类型
    f.write('<vTypeDistribution id="typedist1">\n')
    f.write('    <vType id="CAV" length="4" />\n')  # CAV车长为4.5米
    f.write('    <vType id="HDV" length="4" />\n')   # HDV车长为12米
    f.write('</vTypeDistribution>\n')
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

        f.write(f'    <trip id="{vehicle_type}_{i}" depart="{int(depart_time)}" from="{from_edge}" to="{to_edge}" type="{vehicle_type}"/>\n')
        if str(depart_time//3600+1) not in time_list.keys():
            time_list[str(depart_time//3600+1)] = 1
        else:
            time_list[str(depart_time//3600+1)] += 1
    f.write('</routes>\n')

for i in time_list.keys():
    print(f"hour {i} counts: {time_list[i]}")