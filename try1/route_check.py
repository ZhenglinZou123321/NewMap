import xml.etree.ElementTree as ET
from collections import Counter
# 加载 .net.xml 文件
tree = ET.parse('routes_1.rou.xml')
root = tree.getroot()

id_map = {}
target_edge_list = ['j21tj20','j20tj21','j19tj20','j20tj19']
edge_counts = Counter()
# 遍历并修改ID
for vehicle in root.findall('vehicle'):
    route = vehicle.find('route')
    edges = route.get('edges')

    for target in target_edge_list:
        if target in edges.split():
            for edge in edges.split():
                edge_counts[edge] +=1



for edge, count in edge_counts.most_common():
    print(f"{edge}，  出现次数：{count}")