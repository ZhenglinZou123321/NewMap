import xml.etree.ElementTree as ET

# 加载 .net.xml 文件
tree = ET.parse('Map.net.xml')
root = tree.getroot()



# 遍历并修改ID
for edge in root.findall('edge'):
    edge_id = edge.get('id')
    edge_from = edge.get('from')
    edge_to = edge.get('to')
    if edge_from[0]=='j':
        edge.set('id', edge_from+'t'+edge_to)

# 保存修改后的文件
tree.write('Map_new.net.xml')
