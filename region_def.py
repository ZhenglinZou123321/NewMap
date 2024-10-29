import xml.etree.ElementTree as ET
import random as r
# 加载 .net.xml 文件
tree = ET.parse('Map_new.net.xml')
root = tree.getroot()
# 开始程序交互
def main():
    string = ""
    for edge in root.findall('edge'):
        lstr = ''
        try:
            if edge.get('function') == 'internal':
                continue
        except:
            pass
        edge_id = edge.get('id')
        # 询问 Label 是商业区还是居民区
        label_input = input(f"{edge_id} 是商业区还是居民区？请输入 0 表示商业区或 1 表示居民区: ")

        # 根据输入判断 Label
        if label_input == "0":
            region_type = "商业区"
        elif label_input == "1":
            region_type = "居民区"
        else:
            print("无效输入，请输入 0 或 1。")
            return

        # 询问大还是小
        size_input = input(f"{edge_id} 是大还是小？请输入 0 表示大 或 1 表示中 或 2 表示小: ")

        # 根据输入判断大小
        if size_input == "0":
            region_size = "大"
        elif size_input == "1":
            region_size = "中"
        elif size_input == '2':
            region_size = "小"
        else:
            print("无效输入，请输入 0 或 1。")
            return

        # 输出结果
        population = 0
        workPosition = 0
        if region_type == "商业区":
            if region_size == "大":
                population = r.randint(25,30)
                workPosition = r.randint(90,100)
            elif region_size == '中':
                population = r.randint(20,25)
                workPosition = r.randint(70,90)
            elif region_size == '小':
                population = r.randint(20,22)
                workPosition = r.randint(50,70)
        elif region_type == "居民区":
            if region_size == "大":
                workPosition = r.randint(25,30)
                population = r.randint(90,100)
            elif region_size == '中':
                 workPosition = r.randint(20,25)
                 population= r.randint(70,90)
            elif region_size == '小':
                 workPosition = r.randint(20,22)
                 population = r.randint(50,70)




        lstr = f'<street edge="{edge_id}" population="{str(population)}" workPosition="{str(workPosition)}"/>\n'
        string = string + lstr

    with open("region_def_output_1.xml", "w") as file:
        # 写入指定内容
        file.write(string)
    print("文件创建并写入成功")



# 运行程序
main()
