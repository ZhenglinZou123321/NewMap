import xml.etree.ElementTree as ET

# 加载 .net.xml 文件
tree = ET.parse('one_intersection.net.xml')
root = tree.getroot()

id_map = {}
lane_id_map = {}
def replace_all_references(root, id_map):
    for element in root.iter():
        for attr in element.attrib:
            if element.get(attr) in id_map:
                element.set(attr, id_map[element.get(attr)])
# 遍历并修改ID
for edge in root.findall('edge'):
    try:
        if edge.get('function') == 'internal':
            continue
    except:
        pass
    edge_id = edge.get('id')
    edge_from = edge.get('from')
    edge_to = edge.get('to')
    if edge_from[0]=='j':
        old_id = edge.get('id')
        new_id = f"{edge_from}t{edge_to}"
        id_map[edge_id] = new_id
        edge.set('id', new_id)
        for index, lane in enumerate(edge.findall('lane')):
            old_lane_id = lane.get('id')
            new_lane_id = f"{new_id}_{index}"
            lane.set('id', new_lane_id)
            # 将旧的 lane ID 映射到新的 lane ID
            lane_id_map[old_lane_id] = new_lane_id

def replace_all_references(root, lane_id_map):
    for element in root.iter():
        for attr in element.attrib:
            if attr == 'incLanes' or attr == 'intLanes':
                lanes = element.get(attr).split()
                new_lanes = [lane_id_map.get(lane, lane) for lane in lanes]
                element.set(attr, ' '.join(new_lanes))
            else:
                value = element.get(attr)
                if value in lane_id_map:
                    element.set(attr, lane_id_map[value])

# 替换所有引用中的 lane ID
replace_all_references(root, lane_id_map)

for conn in root.findall('connection'):
    from_edge = conn.get('from')
    to_edge = conn.get('to')

    # 更新 connection 中的 from 引用
    if from_edge in id_map:
        conn.set('from', id_map[from_edge])

    # 更新 connection 中的 to 引用
    if to_edge in id_map:
        conn.set('to', id_map[to_edge])

#replace_all_references(root, id_map)



#设置变道实线：以车道的后1/4作为实线
for lane in root.findall(".//lane"):
    # 获取车道长度
    length = float(lane.get("length"))
    if length > 0:
        # 计算后四分之一的起点
        start_pos = length * 0.75
        # 添加新的实线标线
        marking = ET.Element("marking", {
            "type": "solid",
            "startPos": str(start_pos),
            "endPos": str(length)
        })
        lane.append(marking)


# 保存修改后的文件
tree.write('Map_new.net.xml')
