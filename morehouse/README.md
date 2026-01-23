# 扫地机器人模型运行指南

## 项目简介

本项目是一个基于MuJoCo物理引擎的扫地机器人模拟系统，具有以下功能：

- 避障功能：遇到障碍物时自动右转避障
- 匀速移动：保持稳定的移动速度

## 环境设置

### 1. 安装Python

确保你的系统已经安装了Python 3.7或更高版本。

### 2. 安装MuJoCo

1. 从[MuJoCo官方网站](https://mujoco.org/)下载适合你系统的MuJoCo版本
2. 按照官方文档安装MuJoCo
3. 设置环境变量：
   - Windows: `set MUJOCO_PATH=C:\path\to\mujoco`
   - Linux/Mac: `export MUJOCO_PATH=/path/to/mujoco`

### 3. 安装依赖

使用项目根目录下的虚拟环境，或创建新的虚拟环境并安装依赖：

```bash
# 使用现有的虚拟环境
.venv\Scripts\activate

# 或创建新的虚拟环境
python -m venv venv
venv\Scripts\activate

# 安装依赖
pip install mujoco-python
pip install numpy
```

## 运行模型

### 1. 运行扫地机器人控制脚本

```bash
python control_robot_simple.py
```

### 2. 脚本功能说明

- 启动后，机器人会自动进行清扫
- 遇到障碍物时，机器人会自动右转避障
- 机器人会以匀速移动，速度为：
  - 前进速度：0.75
  - 后退速度：1.25
  - 转向速度：1.25

## 项目结构

```
more house/
├── control_robot_simple.py    # 核心控制脚本
├── three_bedroom_apartment.xml # 房屋模型文件
├── utils/                      # 辅助工具脚本
│   ├── check_xml_syntax.py
│   ├── test_mujoco_load.py
│   ├── validate_mujoco_xml.py
│   └── validate_xml.py
├── microwave/                  # 微波炉模型
├── oven/                       # 烤箱模型
├── studyTable/                 # 书桌模型
├── bookcase_besta_0170/        # 书柜模型
├── bookcase_billy_0191/        # 书柜模型
├── chair_balser_0115/          # 椅子模型
├── chair_ivar_0668/            # 椅子模型
├── tvunit_0406/                # 电视柜模型
└── README.md                   # 本运行指南
```

## 注意事项

1. 确保MuJoCo已正确安装并配置环境变量
2. 确保所有依赖包已正确安装
3. 运行时会打开MuJoCo的可视化窗口，显示机器人的运动状态
4. 控制台会输出机器人的位置、导航状态和避障信息

## 故障排除

- **无法找到MuJoCo库**：请检查MuJoCo的安装路径和环境变量设置
- **缺少依赖包**：请运行`pip install`安装所需的依赖包
- **XML文件错误**：请检查`three_bedroom_apartment.xml`文件的语法是否正确
- **机器人无法移动**：请检查控制脚本中的电机ID是否正确

## 扩展功能

如需扩展功能，可以修改`control_robot_simple.py`文件：

- 调整速度参数：修改`MOVE_SPEED`、`BACK_SPEED`和`TURN_SPEED`变量
- 添加新的房间：在`rooms`字典中添加新的房间信息
- 修改避障策略：调整`turn`状态中的避障逻辑
- 添加新的传感器：在XML文件中添加传感器定义，并在控制脚本中处理传感器数据
