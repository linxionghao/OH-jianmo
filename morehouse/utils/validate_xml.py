import mujoco_py
import os

# 设置XML文件路径
xml_path = 'three_bedroom_apartment.xml'

try:
    # 尝试加载XML文件
    model = mujoco_py.load_model_from_path(xml_path)
    print("XML文件加载成功！")
    
    # 创建模拟环境
    sim = mujoco_py.MjSim(model)
    print("模拟环境创建成功！")
    
    # 运行一步模拟
    sim.step()
    print("模拟运行成功！")
    
except Exception as e:
    print(f"错误信息: {e}")
    import traceback
    traceback.print_exc()