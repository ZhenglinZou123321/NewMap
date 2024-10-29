import xml.etree.ElementTree as ET

# 加载 .net.xml 文件
tree = ET.parse('Map.net.xml')
root = tree.getroot()

id_map = {}

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

for conn in root.findall('connection'):
    from_edge = conn.get('from')
    to_edge = conn.get('to')

    # 更新 connection 中的 from 引用
    if from_edge in id_map:
        conn.set('from', id_map[from_edge])

    # 更新 connection 中的 to 引用
    if to_edge in id_map:
        conn.set('to', id_map[to_edge])



# 保存修改后的文件
tree.write('Map_new.net.xml')
