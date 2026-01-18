# 工业车间Mujoco仿真场景 - 使用说明

## 概述

这是一个高精度、高真实性的工业车间Mujoco仿真场景，包含完整的机器人工作站、流水线系统、物料搬运设备等工业元素。场景适用于机器人控制、路径规划、抓取任务、多机协作等研究。

## 场景特性

- **物理仿真精度**: 采用Newton求解器，时间步长0.002s，公差1e-10
- **场景规模**: 20m × 16m × 8m工业车间
- **设备数量**: 2台6轴机械臂、1条流水线、1台AGV、多个工作站
- **可控自由度**: 16个执行器（两台机械臂各8个自由度）
- **传感器系统**: 位置、速度、姿态等20+传感器

## 文件信息

- **文件名**: `workshop.xml`
- **格式**: Mujoco XML
- **Mujoco版本**: 2.0+
- **文件大小**: 约40KB

---

## 场景组成

### 1. 机械臂系统

#### 第一台机械臂（橙色）
- **位置**: (-2, 3, 0)
- **自由度**: 6轴 + 2指夹爪
- **关节名称**: joint1 ~ joint6, gripper_left, gripper_right
- **工作空间**: 半径约1.25m
- **负载能力**: 约5kg（末端）

**关节范围**:
- Joint 1（基座旋转）: ±180°
- Joint 2（肩部）: ±110°
- Joint 3（肘部）: ±110°
- Joint 4（手腕旋转1）: ±180°
- Joint 5（手腕旋转2）: ±110°
- Joint 6（手腕旋转3）: ±180°
- 夹爪: 0-40mm

#### 第二台机械臂（蓝色）
- **位置**: (2, 3, 0)
- **配置**: 与第一台相同
- **关节名称**: robot2_joint1 ~ robot2_joint6
- **用途**: 协作作业、物料传递

### 2. 流水线系统

- **类型**: 辊筒式传送带
- **尺寸**: 8m（长）× 1.2m（宽）× 0.05m（厚）
- **高度**: 0.85m（离地）
- **组件**:
  - 4根支撑立柱（直径0.1m）
  - 5个传送滚轮（均匀分布）
  - 两侧黄色安全护栏
  - 黑色传送带表面

### 3. 物料系统

#### 物料箱（3个）
- **尺寸**: 0.3m × 0.3m × 0.24m
- **质量**: 2kg/个
- **位置**:
  - box1: (-2, 0, 1.05)
  - box2: (0, 0, 1.05)
  - box3: (2, 0, 1.05)
- **特性**: 可自由移动（freejoint）

### 4. AGV自动导引车

- **位置**: (-4, 0.5, 0)
- **底盘**: 1m × 0.8m × 0.2m
- **质量**: 30kg
- **组件**:
  - 4个独立轮子（直径0.2m）
  - 导航传感器（顶部）
  - 货物托盘（0.8m × 0.6m）
- **用途**: 物料搬运和运输

### 5. 工作台

#### 工作台1（左侧）
- **位置**: (-5, -3, 0)
- **台面**: 1.6m × 1m × 0.06m
- **高度**: 0.78m

#### 工作台2（右侧）
- **位置**: (5, -3, 0)
- **配置**: 与工作台1相同

### 6. 货架系统

- **位置**: (-7, -5, 0)
- **尺寸**: 1.5m × 2m × 4m（高）
- **层数**: 4层（高度: 1m, 2m, 3m, 4m）
- **结构**: 4根立柱 + 4层金属层板

### 7. 质检工位

- **位置**: (6, -5, 0)
- **设备**:
  - 检测台（1.2m × 1.2m）
  - 显微镜设备
  - 照明灯具
- **高度**: 0.8m

### 8. 工具车

- **位置**: (4, -5, 0)
- **结构**: 3层货架
- **组件**:
  - 4个轮子（直径0.16m）
  - 3层金属托盘
  - 工具存放区
- **层高**: 0.3m, 0.6m, 0.9m

### 9. 辅助设施

#### 控制面板
- **位置**: (6, 2, 0)
- **显示屏**: 0.6m × 0.8m
- **控制按钮**: 绿色、黄色、红色各1个

#### 消防设备
- **位置**: (-8, 6, 0)
- **类型**: 灭火器 + 支架

#### 管道系统
- **主管道**: 10m长，直径0.08m
- **分支管道**: 3条，各1.5m

#### 照明系统
- **天花板灯具**: 3个（均匀分布）
- **定向光源**: 3个（多角度照明）

---

## 使用方法

### 方法1: Mujoco Simulate（推荐新手）

```bash
# Windows
simulate workshop.xml

# Linux/Mac
./simulate workshop.xml
```

### 方法2: Python API（推荐开发）

#### 基础示例

```python
import mujoco
import mujoco.viewer

# 加载模型
model = mujoco.MjModel.from_xml_path('workshop.xml')
data = mujoco.MjData(model)

# 启动可视化查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 执行物理仿真步
        mujoco.mj_step(model, data)
        # 同步显示
        viewer.sync()
```

#### 控制机械臂

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path('workshop.xml')
data = mujoco.MjData(model)

# 设置关节目标位置（示例：机械臂回到零位）
target_positions = [0, 0, 0, 0, 0, 0, 0, 0]  # 6个关节 + 2个夹爪

# 获取执行器索引
actuator_names = [
    'joint1_actuator', 'joint2_actuator', 'joint3_actuator',
    'joint4_actuator', 'joint5_actuator', 'joint6_actuator',
    'gripper_left_actuator', 'gripper_right_actuator'
]

# 控制循环
for i in range(1000):
    # 设置控制信号
    for j, name in enumerate(actuator_names):
        actuator_id = model.actuator(name).id
        data.ctrl[actuator_id] = target_positions[j]

    # 执行仿真步
    mujoco.mj_step(model, data)
```

#### 抓取物体示例

```python
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path('workshop.xml')
data = mujoco.MjData(model)

def control_robot(data, joint_positions):
    """控制机械臂关节"""
    actuator_names = [
        'joint1_actuator', 'joint2_actuator', 'joint3_actuator',
        'joint4_actuator', 'joint5_actuator', 'joint6_actuator',
    ]
    for i, name in enumerate(actuator_names):
        actuator_id = model.actuator(name).id
        data.ctrl[actuator_id] = joint_positions[i]

def control_gripper(data, open_distance):
    """控制夹爪开合
    Args:
        open_distance: 0-0.04 (0=闭合, 0.04=完全打开)
    """
    left_id = model.actuator('gripper_left_actuator').id
    right_id = model.actuator('gripper_right_actuator').id
    data.ctrl[left_id] = open_distance
    data.ctrl[right_id] = open_distance

# 抓取序列
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 1. 打开夹爪
    for _ in range(500):
        control_gripper(data, 0.04)
        mujoco.mj_step(model, data)
        viewer.sync()

    # 2. 移动到物体上方（需要逆运动学计算实际角度）
    target_joints = [0, -0.5, -0.5, 0, 0.5, 0]
    for _ in range(1000):
        control_robot(data, target_joints)
        control_gripper(data, 0.04)
        mujoco.mj_step(model, data)
        viewer.sync()

    # 3. 闭合夹爪抓取
    for _ in range(500):
        control_robot(data, target_joints)
        control_gripper(data, 0.01)  # 闭合
        mujoco.mj_step(model, data)
        viewer.sync()

    # 4. 抬起物体
    lift_joints = [0, -0.3, -0.3, 0, 0.3, 0]
    for _ in range(1000):
        control_robot(data, lift_joints)
        control_gripper(data, 0.01)
        mujoco.mj_step(model, data)
        viewer.sync()
```

#### 读取传感器数据

```python
import mujoco

model = mujoco.MjModel.from_xml_path('workshop.xml')
data = mujoco.MjData(model)

# 读取关节位置
joint1_pos_sensor = model.sensor('joint1_pos').id
joint1_position = data.sensordata[joint1_pos_sensor]

# 读取关节速度
joint1_vel_sensor = model.sensor('joint1_vel').id
joint1_velocity = data.sensordata[joint1_vel_sensor]

# 读取末端执行器位置
gripper_pos_sensor = model.sensor('gripper_pos').id
gripper_position = data.sensordata[gripper_pos_sensor:gripper_pos_sensor+3]
print(f"夹爪位置: {gripper_position}")

# 读取物料箱位置
box1_pos_sensor = model.sensor('box1_pos').id
box1_position = data.sensordata[box1_pos_sensor:box1_pos_sensor+3]
print(f"物料箱1位置: {box1_position}")
```

---

## 相机视角

场景提供4个预设相机视角：

1. **overview** - 鸟瞰全景
   - 位置: (8, -8, 6)
   - 视角: 45度俯视
   - 用途: 观察整体布局

2. **robot_view** - 机械臂视角
   - 位置: (-2, 5, 2)
   - 视角: 聚焦机械臂工作区
   - 用途: 观察机械臂操作细节

3. **conveyor_view** - 流水线视角
   - 位置: (0, -3, 3)
   - 视角: 侧面观察流水线
   - 用途: 监控物料流动

4. **tracking** - 物体跟踪视角
   - 目标: box1（物料箱1）
   - 模式: 自动跟随
   - 用途: 追踪物料移动

**切换相机**:
- Simulate界面: 右键点击场景 → Select Camera
- Python API: `viewer.cam.fixedcamid = camera_id`

---

## 执行器列表

### 第一台机械臂（橙色）

| 执行器名称 | 类型 | 控制范围 | 增益(kp) |
|-----------|------|---------|---------|
| joint1_actuator | position | ±180° | 200 |
| joint2_actuator | position | ±110° | 200 |
| joint3_actuator | position | ±110° | 150 |
| joint4_actuator | position | ±180° | 100 |
| joint5_actuator | position | ±110° | 100 |
| joint6_actuator | position | ±180° | 50 |
| gripper_left_actuator | position | 0-40mm | 50 |
| gripper_right_actuator | position | 0-40mm | 50 |

### 第二台机械臂（蓝色）

| 执行器名称 | 类型 | 控制范围 | 增益(kp) |
|-----------|------|---------|---------|
| robot2_joint1_actuator | position | ±180° | 200 |
| robot2_joint2_actuator | position | ±110° | 200 |
| robot2_joint3_actuator | position | ±110° | 150 |
| robot2_joint4_actuator | position | ±180° | 100 |
| robot2_joint5_actuator | position | ±110° | 100 |
| robot2_joint6_actuator | position | ±180° | 50 |
| robot2_gripper_left_actuator | position | 0-40mm | 50 |
| robot2_gripper_right_actuator | position | 0-40mm | 50 |

---

## 传感器列表

### 关节传感器

| 传感器类型 | 数量 | 命名格式 | 数据维度 |
|-----------|------|---------|---------|
| 关节位置 | 12 | jointX_pos, robot2_jointX_pos | 1D |
| 关节速度 | 6 | jointX_vel | 1D |
| 夹爪位置 | 4 | gripper_left/right_pos | 1D |

### 位姿传感器

| 传感器名称 | 类型 | 数据维度 | 说明 |
|-----------|------|---------|------|
| gripper_pos | framepos | 3D | 第一台机械臂末端位置(x,y,z) |
| gripper_quat | framequat | 4D | 第一台机械臂末端姿态(四元数) |
| box1_pos | framepos | 3D | 物料箱1位置 |
| box2_pos | framepos | 3D | 物料箱2位置 |
| box3_pos | framepos | 3D | 物料箱3位置 |

---

## 应用场景

### 1. 机器人抓取研究
- 单臂抓取
- 双臂协作
- 视觉伺服控制
- 力控制

### 2. 路径规划
- 避障规划
- 轨迹优化
- 多机协调
- 碰撞检测

### 3. 强化学习
- 抓取任务学习
- 物料分拣
- 装配任务
- 多智能体协作

### 4. 工业仿真
- 生产线模拟
- 工艺验证
- 布局优化
- 周期时间分析

### 5. 教学演示
- 机器人运动学
- 动力学仿真
- 控制系统设计
- 工业自动化

---

## 性能参数

### 仿真参数
- **时间步长**: 0.002s (500Hz)
- **求解器**: Newton
- **迭代次数**: 50
- **求解精度**: 1e-10
- **实时因子**: ~1.0（取决于硬件）

### 物理参数
- **重力**: 9.81 m/s²
- **摩擦系数**: 1.0（切向）, 0.5（侧向）
- **材料密度**: 1000 kg/m³（默认）
- **接触边界**: 0.001m

### 可视化参数
- **阴影质量**: 4096×4096
- **渲染分辨率**: 800×800（离屏）
- **视野距离**: 50m
- **雾效**: 启用（增强深度感）

---

## 高级功能

### 1. 逆运动学求解

```python
import mujoco
import numpy as np

def inverse_kinematics(model, data, body_name, target_pos, target_quat=None):
    """简单的逆运动学求解器"""
    body_id = model.body(body_name).id
    jac_pos = np.zeros((3, model.nv))
    jac_rot = np.zeros((3, model.nv))

    for _ in range(100):  # 迭代求解
        mujoco.mj_jacBody(model, data, jac_pos, jac_rot, body_id)

        # 计算位置误差
        current_pos = data.body(body_id).xpos
        error_pos = target_pos - current_pos

        # 更新关节角度
        delta_q = np.linalg.pinv(jac_pos) @ error_pos * 0.1
        data.qpos[:6] += delta_q[:6]

        mujoco.mj_forward(model, data)

        # 检查收敛
        if np.linalg.norm(error_pos) < 0.01:
            break

    return data.qpos[:6]
```

### 2. 碰撞检测

```python
def check_collisions(model, data):
    """检查场景中的碰撞"""
    collisions = []
    for i in range(data.ncon):
        contact = data.contact[i]
        geom1 = model.geom(contact.geom1).name
        geom2 = model.geom(contact.geom2).name
        collisions.append((geom1, geom2, contact.dist))
    return collisions
```

### 3. 轨迹记录

```python
class TrajectoryRecorder:
    def __init__(self):
        self.trajectory = []

    def record(self, data):
        """记录当前状态"""
        state = {
            'time': data.time,
            'qpos': data.qpos.copy(),
            'qvel': data.qvel.copy(),
            'ctrl': data.ctrl.copy()
        }
        self.trajectory.append(state)

    def save(self, filename):
        """保存轨迹到文件"""
        np.save(filename, self.trajectory)

    def load(self, filename):
        """从文件加载轨迹"""
        self.trajectory = np.load(filename, allow_pickle=True)
```

---

## 常见问题

### Q1: 场景加载缓慢或卡顿？
**A**: 降低阴影质量或关闭实时渲染：
```xml
<!-- 修改 visual/quality 标签 -->
<quality shadowsize="2048"/>
```

### Q2: 机械臂抖动不稳定？
**A**: 增加关节阻尼或降低增益：
```xml
<joint damping="5"/>  <!-- 增加阻尼 -->
<position kp="100"/>  <!-- 降低增益 -->
```

### Q3: 物体穿透或碰撞异常？
**A**: 调整碰撞参数：
```xml
<geom margin="0.002"/>  <!-- 增加碰撞边界 -->
<option solver="CG" iterations="100"/>  <!-- 增加迭代次数 -->
```

### Q4: 如何添加新的物体？
**A**: 在worldbody中添加新的body：
```xml
<body name="new_object" pos="x y z">
    <freejoint/>
    <geom type="box" size="0.1 0.1 0.1" rgba="1 0 0 1"/>
</body>
```

### Q5: 如何修改材质颜色？
**A**: 修改对应geom的rgba属性：
```xml
<geom rgba="R G B A"/>  <!-- 范围: 0-1 -->
```

---

## 性能优化建议

### 1. 减少计算负载
- 降低时间步长精度（增大timestep）
- 减少求解器迭代次数
- 简化碰撞几何体

### 2. 提升渲染性能
- 降低阴影分辨率
- 减少光源数量
- 使用简单材质

### 3. 批量仿真
```python
# 无渲染模式（最快）
for _ in range(10000):
    mujoco.mj_step(model, data)
    # 不调用viewer.sync()
```

---

## 扩展开发

### 添加ROS接口

```python
# 需要安装: pip install rclpy
import rclpy
from sensor_msgs.msg import JointState

def publish_joint_states(data):
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    msg.position = data.qpos[:6].tolist()
    msg.velocity = data.qvel[:6].tolist()
    publisher.publish(msg)
```

### 集成深度学习

```python
import torch
import gymnasium as gym

class MujocoWorkshopEnv(gym.Env):
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path('workshop.xml')
        self.data = mujoco.MjData(self.model)
        # 定义观察空间和动作空间

    def step(self, action):
        # 执行动作
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        # 返回观察、奖励、终止标志
        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_done()
        return obs, reward, done, {}
```

---

## 技术规格

### 环境要求
- **操作系统**: Windows 10+, Ubuntu 20.04+, macOS 10.15+
- **Python**: 3.8+
- **Mujoco**: 2.0+
- **内存**: 建议4GB+
- **显卡**: 支持OpenGL 3.3+

### 依赖库
```bash
pip install mujoco
pip install numpy
pip install matplotlib  # 可选：数据可视化
```

### 文件结构
```
build/
├── workshop.xml          # 主场景文件
└── README.md            # 本说明文档
```

---

## 参考资料

- [Mujoco官方文档](https://mujoco.readthedocs.io/)
- [Mujoco Python绑定](https://github.com/deepmind/mujoco)
- [XML模型格式](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [机器人学基础](https://www.coursera.org/learn/robotics-basics)

---

## 更新日志

### v1.0 (2026-01-15)
- 初始版本发布
- 包含2台6轴机械臂
- 完整流水线系统
- AGV和辅助设备
- 16个执行器和20+传感器

---

## 许可证

本项目仅供学习和研究使用。

---

## 联系方式

如有问题或建议，欢迎反馈。

**享受你的Mujoco仿真之旅！** 🤖🏭
