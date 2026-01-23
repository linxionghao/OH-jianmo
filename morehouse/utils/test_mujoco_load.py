import os

# 尝试导入mujoco-py
try:
    import mujoco_py
    print("成功导入mujoco-py模块")
    
    # 检查是否安装了MuJoCo库
    try:
        # 尝试加载XML文件
        model_path = os.path.join(os.path.dirname(__file__), 'three_bedroom_apartment.xml')
        model = mujoco_py.load_model_from_path(model_path)
        print("✓ 成功加载XML模型")
        
        # 创建模拟环境
        sim = mujoco_py.MjSim(model)
        print("✓ 成功创建模拟环境")
        
        # 运行一步模拟
        sim.step()
        print("✓ 成功运行一步模拟")
        
        print("\n🎉 模型加载和模拟运行成功！")
        print(f"模型名称: {model.modelname}")
        print(f"自由度: {model.nq}")
        print(f"接触点: {model.ncon}")
        
    except Exception as e:
        print(f"✗ 加载或运行模型时出错: {e}")
        import traceback
        traceback.print_exc()
        
except ImportError as e:
    print(f"✗ 导入mujoco-py失败: {e}")
    print("请确保已正确安装MuJoCo和mujoco-py")
    print("安装指南: https://github.com/openai/mujoco-py#install-mujoco")
    
except Exception as e:
    print(f"✗ 发生其他错误: {e}")
    import traceback
    traceback.print_exc()