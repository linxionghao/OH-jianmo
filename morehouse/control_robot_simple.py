import mujoco
import mujoco.viewer
import numpy as np
import time

# 加载模型
model = mujoco.MjModel.from_xml_path('three_bedroom_apartment.xml')
data = mujoco.MjData(model)

# 查找扫地机器人的电机ID
robot_motors = []
for i in range(model.nu):
    motor_name = model.actuator(i).name
    if 'robot_' in motor_name:
        robot_motors.append(i)
        print(f"找到扫地机器人电机: {motor_name}, ID: {i}")

if len(robot_motors) == 0:
    print("错误: 找不到扫地机器人的电机!")
    exit(1)

# 房间布局数据
rooms = {
    "living_room": {
        "center": [0, -3.5],
        "bounds": {"x_min": -3, "x_max": 3, "y_min": -6, "y_max": -1}
    },
    "master_bedroom": {
        "center": [4.2, 4.5],
        "bounds": {"x_min": 2.5, "x_max": 6, "y_min": 1, "y_max": 6}
    },
    "study_room": {
        "center": [0, 3.5],
        "bounds": {"x_min": -3, "x_max": 3, "y_min": 1, "y_max": 6}
    },
    "bedroom_2": {
        "center": [-4.6, 3.5],
        "bounds": {"x_min": -6, "x_max": -2.5, "y_min": 1, "y_max": 6}
    },
    "kitchen": {
        "center": [-4.2, -3],
        "bounds": {"x_min": -6, "x_max": -2.5, "y_min": -6, "y_max": -1}
    },
    "bathroom": {
        "center": [4.2, -2.5],
        "bounds": {"x_min": 2.5, "x_max": 6, "y_min": -6, "y_max": -1}
    },
    "hallway": {
        "center": [0, 0],
        "bounds": {"x_min": -3, "x_max": 3, "y_min": -1, "y_max": 1}
    }
}

# 状态机
state = "forward"  # forward, turn, back, navigate
state_timer = 0
state_duration = 2.0  # 状态持续时间（秒）

# 随机方向
current_direction = np.random.uniform(-1, 1, 2)
current_direction = current_direction / np.linalg.norm(current_direction)

# 碰撞检测阈值
collision_threshold = 1.0

# 导航目标
nav_target_room = None
nav_target_pos = None
nav_timer = 0
nav_timeout = 30.0  # 导航超时时间（秒）

# 房间清扫状态 - 只包含客厅和厨房
rooms_cleaned = {
    "living_room": False,
    "kitchen": False
}
current_room = "hallway"

# 判断机器人当前所在房间
def get_current_room(robot_pos):
    x, y = robot_pos[0], robot_pos[1]
    for room_name, room_data in rooms.items():
        bounds = room_data["bounds"]
        if bounds["x_min"] <= x <= bounds["x_max"] and bounds["y_min"] <= y <= bounds["y_max"]:
            return room_name
    return None

# 选择下一个要清扫的房间 - 只考虑客厅和厨房
def select_next_room():
    for room_name, cleaned in rooms_cleaned.items():
        if not cleaned:
            return room_name
    # 所有房间都已清扫，返回None
    return None

# 移动速度参数
MOVE_SPEED = 0.375
BACK_SPEED = 0.625
TURN_SPEED = 0.625

# 启动模拟器
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 打开所有门到90度
    door_motors = []
    for i in range(model.nu):
        motor_name = model.actuator(i).name
        if 'door' in motor_name:
            door_motors.append(i)
    
    # 设置所有门打开到90度
    for i in range(model.njnt):
        joint_name = model.joint(i).name
        if 'hinge' in joint_name:
            data.qpos[i] = np.pi / 2  # 90度
    
    # 应用状态
    mujoco.mj_forward(model, data)
    
    start = time.time()
    while viewer.is_running():
        step_start = time.time()
        
        # 获取当前时间
        current_time = time.time() - start
        
        # 检测碰撞
        collision = False
        collision_direction = np.array([0.0, 0.0])
        collision_count = 0
        for i in range(data.ncon):
            contact = data.contact[i]
            try:
                geom1_name = model.geom(contact.geom1).name
                geom2_name = model.geom(contact.geom2).name
            except:
                continue
            
            # 跳过没有名称的几何形状
            if not geom1_name or not geom2_name:
                continue
            
            # 检查是否是扫地机器人与其他物体的碰撞
            # 排除与地面和地毯的碰撞
            if ('body' in geom1_name or 'body' in geom2_name) and \
               'floor' not in geom1_name and 'floor' not in geom2_name and \
               'carpet' not in geom1_name and 'carpet' not in geom2_name:
                collision = True
                print(f"碰撞: {geom1_name} 与 {geom2_name}")
                
                # 计算碰撞方向
                # 找到扫地机器人的几何形状
                robot_geom_id = contact.geom1 if 'body' in geom1_name else contact.geom2
                other_geom_id = contact.geom2 if 'body' in geom1_name else contact.geom1
                
                # 获取几何形状的位置
                robot_geom_pos = data.geom_xpos[robot_geom_id]
                other_geom_pos = data.geom_xpos[other_geom_id]
                
                # 计算碰撞方向（从机器人指向障碍物）
                direction = other_geom_pos[:2] - robot_geom_pos[:2]
                if np.linalg.norm(direction) > 0:
                    direction = direction / np.linalg.norm(direction)
                    collision_direction += direction
                    collision_count += 1
                # 不break，继续检测所有碰撞点
        
        # 状态机逻辑
        if collision and state != "back":
            # 遇到障碍物，先后退再转向
            state = "back"
            state_timer = 0
            print(f"[{current_time:.2f}] 检测到障碍物，切换到后退状态")
            
            # 根据碰撞方向计算后退方向
            if collision_count > 0:
                collision_direction = collision_direction / collision_count
                # 后退方向与碰撞方向相反
                current_direction = -collision_direction
                print(f"[{current_time:.2f}] 避障方向: {current_direction}")
        elif state_timer > state_duration:
            # 状态持续时间结束，切换状态
            if state == "forward":
                state = "turn"
                print(f"[{current_time:.2f}] 切换到转向状态")
            elif state == "back":
                state = "turn"
                print(f"[{current_time:.2f}] 后退完成，切换到转向状态")
            else:
                state = "forward"
                # 随机新方向
                current_direction = np.random.uniform(-1, 1, 2)
                current_direction = current_direction / np.linalg.norm(current_direction)
                print(f"[{current_time:.2f}] 切换到前进状态，新方向: {current_direction}")
            state_timer = 0
        
        # 检查机器人当前所在房间
        robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot_vacuum')
        robot_pos = data.xpos[robot_body_id]
        robot_current_room = get_current_room(robot_pos)
        
        # 更新当前房间
        if robot_current_room:
            current_room = robot_current_room
        
        # 导航逻辑：如果没有导航目标，选择下一个要清扫的房间
        if nav_target_room is None:
            nav_target_room = select_next_room()
            if nav_target_room:
                nav_target_pos = np.array(rooms[nav_target_room]["center"])
                nav_timer = 0
                state = "navigate"
                print(f"[{current_time:.2f}] 开始导航到 {nav_target_room}，目标位置: {nav_target_pos}")
            else:
                # 所有房间都已清扫完成
                print(f"[{current_time:.2f}] 所有房间都已清扫完成！")
                # 保持前进状态，继续随机移动
                if state != "forward":
                    state = "forward"
                    current_direction = np.random.uniform(-1, 1, 2)
                    current_direction = current_direction / np.linalg.norm(current_direction)
        
        # 执行状态动作
        if state == "forward":
            # 前进：沿当前方向移动
            data.ctrl[robot_motors[0]] = current_direction[0] * MOVE_SPEED
            data.ctrl[robot_motors[1]] = current_direction[1] * MOVE_SPEED
            data.ctrl[robot_motors[2]] = 0.0  # z方向力
        elif state == "back":
            # 后退：向相反方向移动，增加后退力
            data.ctrl[robot_motors[0]] = current_direction[0] * BACK_SPEED
            data.ctrl[robot_motors[1]] = current_direction[1] * BACK_SPEED
            data.ctrl[robot_motors[2]] = 0.0
        elif state == "turn":
            # 转向：根据碰撞方向转向
            if collision_count > 0:
                # 有碰撞时，向与碰撞方向垂直的右侧方向转向
                # 计算垂直方向（右转）
                perpendicular_direction = np.array([-current_direction[1], current_direction[0]])
                data.ctrl[robot_motors[0]] = perpendicular_direction[0] * TURN_SPEED
                data.ctrl[robot_motors[1]] = perpendicular_direction[1] * TURN_SPEED
            else:
                # 无碰撞时，随机转向，使用TURN_SPEED参数
                data.ctrl[robot_motors[0]] = np.random.uniform(-TURN_SPEED, TURN_SPEED)
                data.ctrl[robot_motors[1]] = np.random.uniform(-TURN_SPEED, TURN_SPEED)
            data.ctrl[robot_motors[2]] = 0.0
        elif state == "navigate":
            # 导航状态：向目标房间中心移动
            if nav_target_pos is not None:
                # 计算到目标位置的方向向量
                direction_to_target = nav_target_pos - robot_pos[:2]
                distance_to_target = np.linalg.norm(direction_to_target)
                
                # 检查是否到达目标位置
                if distance_to_target < 0.5:  # 到达目标位置的阈值
                    print(f"[{current_time:.2f}] 到达 {nav_target_room}！开始清扫...")
                    # 标记房间为已清扫
                    rooms_cleaned[nav_target_room] = True
                    # 清除导航目标，切换到前进状态
                    nav_target_room = None
                    nav_target_pos = None
                    state = "forward"
                    current_direction = np.random.uniform(-1, 1, 2)
                    current_direction = current_direction / np.linalg.norm(current_direction)
                else:
                    # 向目标位置移动
                    direction_to_target = direction_to_target / distance_to_target
                    # 打印导航信息，以便调试
                    print(f"[{current_time:.2f}] 导航中: 当前位置: {robot_pos[:2]}, 目标位置: {nav_target_pos}, 方向: {direction_to_target}, 距离: {distance_to_target}")
                    # 保持匀速移动，不加速
                    data.ctrl[robot_motors[0]] = direction_to_target[0] * MOVE_SPEED
                    data.ctrl[robot_motors[1]] = direction_to_target[1] * MOVE_SPEED
                    data.ctrl[robot_motors[2]] = 0.0
                    
                    # 检查导航超时
                    nav_timer += model.opt.timestep
                    if nav_timer > nav_timeout:
                        print(f"[{current_time:.2f}] 导航到 {nav_target_room} 超时，重新规划路径")
                        nav_target_room = None
                        nav_target_pos = None
                        state = "forward"
                        current_direction = np.random.uniform(-1, 1, 2)
                        current_direction = current_direction / np.linalg.norm(current_direction)
        
        # 更新状态计时器
        state_timer += model.opt.timestep
        
        # 步进模拟
        mujoco.mj_step(model, data)
        
        # 获取扫地机器人的位置
        robot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'robot_vacuum')
        robot_pos = data.xpos[robot_body_id]
        
        # 每1秒输出一次位置
        if int(current_time) % 1 == 0 and current_time - int(current_time) < model.opt.timestep:
            print(f"[{current_time:.2f}] 扫地机器人位置: [{robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f}]")
        
        # 同步 viewer
        viewer.sync()
        
        # 保持实时模拟
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("模拟结束")
