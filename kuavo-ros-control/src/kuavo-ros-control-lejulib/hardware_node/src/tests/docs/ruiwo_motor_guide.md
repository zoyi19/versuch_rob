# Ruiwo 手臂电机控制跟踪测试

`Ruiwo`电机的独立测试模块，提供基于 ROS 话题的控制与状态跟踪测试。

## 基本信息

- `/ruiwo_motor/command`: 订阅电机控制命令, 单位弧度
- `/ruiwo_motor/state`：发布电机状态，单位弧度
- 屏蔽电机可通过修改`~.config.lejuconfig/config.yaml`中的电机ID实现，将其设置成`0x00`即屏蔽，通过该方可实现只对单个电机进行测试
- 电机控制线程频率 200Hz，接收反馈消息线程频率 200Hz

### 支持的机器人类型

- **Kuavo机器人**：14个关节（左右手臂各6个+头部2个）
- **Roban机器人**：10个关节（左右手臂各4个+头部2个）

### Launch文件参数

- `cali_arm`：是否启用手臂校准（默认：false）
- `robot_type`：机器人类型（默认：kuavo，选项：kuavo, roban）
- `can_protocol`：CAN协议类型（默认：false）

## 编译

```bash
catkin build hardware_node
```

## 运行

### 运行电机控制程序

**C++ 版本：**
```bash
# 测试 Kuavo 机械臂
roslaunch hardware_node ruiwo_motor_cxx_test.launch robot_type:=kuavo

# 测试 Roban 机械臂
roslaunch hardware_node ruiwo_motor_cxx_test.launch robot_type:=roban

# 带校准参数
roslaunch hardware_node ruiwo_motor_cxx_test.launch robot_type:=kuavo cali_arm:=true
```

**Python 版本：**
```bash
# 测试 Kuavo 机械臂
roslaunch hardware_node ruiwo_motor_py_test.launch robot_type:=kuavo

# 测试 Roban 机械臂
roslaunch hardware_node ruiwo_motor_py_test.launch robot_type:=roban

# 带校准参数
roslaunch hardware_node ruiwo_motor_py_test.launch robot_type:=kuavo cali_arm:=true
```

### 运行轨迹脚本

**Kuavo机械臂测试：**
```bash
source devel/setup.bash
python3 scripts/kuavo_arm_tracing_test.py
```

**Roban机械臂测试：**
```bash
source devel/setup.bash
python3 scripts/roban_arm_tracing_test.py
```

**注意事项：**
- 运行脚本前确保已启动对应的手臂控制程序
- 结束控制请先使用`CTRL+C`结束 Python 轨迹脚本，手臂会自动回到零点
- 待回到零点后，可使用`CTRL+C`停止控制程序

## 查看效果

使用`plotjuggler`或其他工具查看控制指令与电机状态:
```bash
rosrun plotjuggler plotjuggler
```

## 在仿真中预览控制曲线和动作

先在 rviz 中运行可视化：
```bash
# kuavo 4pro
cp -r src/kuavo_assets/models/biped_s45 ./src
catkin build biped_s45
source devel/setup.bash
roslaunch biped_s45 display.launch

# roban2
cp -r src/kuavo_assets/models/biped_s13 ./src
catkin build biped_s13
source devel/setup.bash
roslaunch biped_s13 display.launch
```

运行测试脚本：
```bash
source devel/setup.bash
# kuavo 4pro
python3 scripts/kuavo_arm_tracing_test.py
# roban2
python3 scripts/roban_arm_tracing_test.py
```

## 文件结构

```
hardware_node/src/tests/
├── ruiwo_controller_ros_test.cc        # 主要的电机控制器测试程序
├── scripts/
│   ├── kuavo_arm_tracing_test.py      # Kuavo机械臂轨迹测试
│   ├── roban_arm_tracing_test.py      # Roban机械臂轨迹测试
│   └── record_arm_dexhand_topics.sh   # ROS话题录制脚本
└── docs/
    └── ruiwo_motor_guide.md           # 本指南文档
```