# KUAVO机器人DDS话题接口文档

**非常重要的提示**：
**非常重要的提示**：
**非常重要的提示**：
- 本文档提供的 DDS 控制话题中，所有数据皆为关节空间（Joint Space）.

## 概述

HardwareDdsPlant模块基于Eclipse Cyclone DDS实现机器人的实时通信，提供低延迟的控制命令接收和状态数据发布功能。该模块继承自HardwarePlant，在保持所有原有硬件控制功能的同时，增加了DDS通信能力。

### 重要说明：协议兼容性

- 本模块使用`unitree_hg::msg::dds_::LowCmd_`和`unitree_hg::msg::dds_::LowState_`
- 这些是Unitree公司定义的标准机器人控制协议
- **无法修改IDL消息结构**，只能在现有字段基础上进行适配

**字段复用策略**:
- KUAVO机器人的28个关节 → 使用DDS协议的35个电机槽位中的0-27
- KUAVO特有数据(如free_acc) → 复用类似语义的字段(如rpy)
- 未使用的字段保持零值或默认值，确保协议兼容性

## 1. DDS话题配置

### 1.1 话题列表

| 话题名称 | 消息类型 | 方向 | 功能描述 | 频率 |
|---------|----------|------|-----------|------|
| `rt/lowcmd` | `unitree_hg::msg::dds_::LowCmd_` | 订阅 | 接收机器人控制命令 | 500Hz |
| `rt/lowstate` | `unitree_hg::msg::dds_::LowState_` | 发布 | 发布机器人状态数据 | 500Hz |

### 1.2 系统常量

| 常量名 | 值 | 物理含义 | 备注 |
|-------|-----|----------|------|
| `DDS_MOTOR_COUNT` | 35 | DDS协议支持的最大电机数量 | 兼容Unitree协议 |
| `KUAVO_JOINT_COUNT` | 28 | KUAVO机器人实际关节数量 | 映射到电机0-27 |


---

## 2. 控制命令接收接口

### 2.1 LowCmd_ 消息结构

**话题**: `rt/lowcmd`  
**消息类型**: `unitree_hg::msg::dds_::LowCmd_`

```cpp
class LowCmd_ {
    uint8_t mode_pr_;                              // 程序运行模式
    uint8_t mode_machine_;                         // 机器状态模式
    std::array<MotorCmd_, 35> motor_cmd_;          // 35个电机控制命令
    std::array<uint32_t, 4> reserve_;             // 保留字段
    uint32_t crc_;                                // CRC32校验码
};
```

#### 消息字段物理含义

| 字段名 | 类型 | 物理含义 | 单位 | 使用状态 | 实际用途 |
|-------|------|----------|------|----------|----------|
| `mode_pr_` | uint8_t | 程序运行模式控制 | - | **未使用** | 接收但不处理 |
| `mode_machine_` | uint8_t | 机器状态模式控制 | - | **已使用** | 更新内部状态：`mode_machine_` |
| `motor_cmd_[35]` | MotorCmd_[] | 35个电机的控制命令数组 | 各电机单位 | **部分使用** | 仅使用0-27索引(KUAVO关节数) |
| `reserve_[4]` | uint32_t[] | 协议扩展保留字段 | - | **未使用** | 协议保留，无实际处理 |
| `crc_` | uint32_t | 数据完整性校验码 | - | **已使用** | CRC32校验，验证失败丢弃消息 |

### 2.2 MotorCmd_ 控制命令结构

```cpp
class MotorCmd_ {
    uint8_t mode_;        // 电机控制模式
    float q_;             // 位置目标值  对于leju，为关节
    float dq_;            // 速度目标值  对于leju，为关节
    float tau_;           // 力矩目标值  对于leju，为关节
    float kp_;            // 位置增益
    float kd_;            // 阻尼增益
    uint32_t reserve_;    // 保留字段
};
```

#### 电机控制参数物理含义

| 字段名 | 物理含义 | 单位 | 使用状态 |
|-------|----------|------|----------|
| `mode_` | 电机控制模式选择 | - | **已使用** |
| `q_` | 目标关节位置 | rad | **已使用** |
| `dq_` | 目标关节角速度 | rad/s | **已使用** |
| `tau_` | 目标关节力矩 | N·m | **已使用** |
| `kp_` | 位置比例增益(刚度) | N·m/rad | **已使用** |
| `kd_` | 速度阻尼增益 | N·m·s/rad | **已使用** |
| `reserve_` | 扩展字段 | - | **未使用** |

#### 硬编码的控制参数

| 参数名 | 固定值 | 物理含义 |
|-------|--------|----------|
| `tau_max` | 1000.0 | 最大力矩限制 |
| `tau_ratio` | 1.0 | 力矩比例系数 |

#### 控制模式说明

| mode_ 值 | 控制模式 |
|----------|----------|
| 0 | 扭矩控制模式 |
| 1 | 速度控制模式 |
| 2 | 位置控制模式 |

---

## 3. 状态数据发布接口

### 3.1 LowState_ 消息结构

**话题**: `rt/lowstate`  
**消息类型**: `unitree_hg::msg::dds_::LowState_`

```cpp
class LowState_ {
    std::array<uint32_t, 2> version_;                          // 协议版本信息
    uint8_t mode_pr_;                                          // 程序运行模式状态
    uint8_t mode_machine_;                                     // 机器当前状态模式
    uint32_t tick_;                                            // 时间戳计数器
    IMUState_ imu_state_;                                      // IMU传感器状态
    std::array<MotorState_, 35> motor_state_;                  // 35个电机状态
    std::array<uint8_t, 40> wireless_remote_;                 // 无线遥控器数据
    std::array<uint32_t, 4> reserve_;                         // 保留字段
    uint32_t crc_;                                            // CRC32校验码
};
```

#### 状态消息字段物理含义

| 字段名 | 类型 | 物理含义 | 使用状态 |
|-------|------|----------|----------|
| `version_[2]` | uint32_t[] | DDS协议版本号 | **未设置** |
| `mode_pr_` | uint8_t | 当前程序运行模式 | **未设置** |
| `mode_machine_` | uint8_t | 当前机器状态模式 | **未设置** |
| `tick_` | uint32_t | 系统时间戳计数器 | **已使用** |
| `imu_state_` | IMUState_ | IMU传感器完整状态 | **已使用** |
| `motor_state_[35]` | MotorState_[] | 35个电机的状态反馈 | **部分使用** |
| `wireless_remote_[40]` | uint8_t[] | 无线遥控器原始数据 | **未使用** |
| `reserve_[4]` | uint32_t[] | 时间戳等扩展数据 | **部分使用** |
| `crc_` | uint32_t | 状态数据完整性校验 | **已使用** |

### 3.2 MotorState_ 电机状态结构

```cpp
class MotorState_ {
    uint8_t mode_;                        // 当前控制模式
    float q_;                             // 当前位置
    float dq_;                            // 当前速度
    float ddq_;                           // 当前加速度
    float tau_est_;                       // 估计力矩
    std::array<int16_t, 2> temperature_;  // 温度传感器
    float vol_;                           // 电机电压
    std::array<uint32_t, 2> sensor_;      // 传感器原始数据
    uint32_t motorstate_;                 // 电机状态位
    std::array<uint32_t, 4> reserve_;     // 保留字段
};
```

#### 电机状态参数物理含义

| 字段名 | 物理含义 | 单位 | 使用状态 |
|-------|----------|------|----------|
| `mode_` | 电机当前工作模式 | - | **已设置** |
| `q_` | 实际关节位置反馈 | rad | **已使用** |
| `dq_` | 实际关节角速度反馈 | rad/s | **已使用** |
| `ddq_` | 实际关节角加速度 | rad/s² | **已使用** |
| `tau_est_` | 估计关节力矩 | N·m | **已使用** |
| `temperature_[2]` | 电机温度监测 | °C | **未使用** |
| `vol_` | 电机供电电压 | V | **未使用** |
| `sensor_[2]` | 编码器等传感器原始值 | 计数 | **未使用** |
| `motorstate_` | 电机状态标志位 | 位标志 | **未使用** |
| `reserve_[4]` | 扩展状态信息 | - | **未设置** |

### 3.3 IMUState_ 惯性测量单元状态

```cpp
class IMUState_ {
    std::array<float, 4> quaternion_;      // 四元数姿态
    std::array<float, 3> gyroscope_;       // 陀螺仪数据
    std::array<float, 3> accelerometer_;   // 加速度计数据
    std::array<float, 3> rpy_;             // 欧拉角姿态
    int16_t temperature_;                  // IMU温度
};
```

#### IMU参数物理含义

| 字段名 | 物理含义 | 单位 | 使用状态 |
|-------|----------|------|----------|
| `quaternion_[4]` | 四元数姿态表示(x,y,z,w) | 无量纲 | **已使用** |
| `gyroscope_[3]` | 三轴角速度(X,Y,Z) | rad/s | **已使用** |
| `accelerometer_[3]` | 三轴加速度(X,Y,Z) | m/s² | **已使用** |
| `rpy_[3]` | 欧拉角(Roll,Pitch,Yaw) | rad | **复用** |
| `temperature_` | IMU传感器温度 | °C | **未使用** |

---

## 4. 重要发现：实际使用情况分析

### 4.1 协议兼容性导致的未使用字段

**LowCmd接收时未使用的字段** (Unitree协议兼容性考虑):
- `mode_pr_` - Unitree程序模式，KUAVO不需要此字段
- `reserve_[4]` - 协议保留字段，用于未来扩展
- `motor_cmd_[].reserve_` - 电机命令保留字段

**LowState发布时未设置的字段** (避免与Unitree系统冲突):  
- `version_[2]` - Unitree版本信息，KUAVO保持零值
- `mode_pr_` - Unitree程序模式，KUAVO未实现
- `mode_machine_` - Unitree机器状态，KUAVO未同步
- `wireless_remote_[40]` - Unitree遥控器数据，KUAVO不使用
- 电机状态中的硬件相关字段 - KUAVO未实现或数据不可用

### 4.2 协议适配导致的字段复用

**字段复用问题**: IMU的`rpy_`字段被复用存储其他数据
```cpp  
// 源码第377-379行 - 字段复用
dds_state.imu_state().rpy()[0] = static_cast<float>(sensor_data.free_acc.x());
dds_state.imu_state().rpy()[1] = static_cast<float>(sensor_data.free_acc.y());  
dds_state.imu_state().rpy()[2] = static_cast<float>(sensor_data.free_acc.z());
```
**原因**: 使用Unitree的IDL协议，无法自定义消息结构，只能复用现有字段
- `rpy_`字段本意是欧拉角(Roll-Pitch-Yaw)
- 实际被复用来传输`free_acc`(自由加速度)数据
- 这是协议适配的权宜之计，而非设计错误

### 4.3 硬编码参数

**控制命令处理中的固定值**:
```cpp
cmd_input[num_joint * 3 + kuavo_idx] = 1000.0; // tau_max - 固定最大力矩
cmd_input[num_joint * 4 + kuavo_idx] = 1.0;    // tau_ratio - 固定力矩比例
```

**状态发布中的固定值**:
```cpp
motor_state.mode(1);           // 启用模式 - 0-27关节
motor_state.mode(0);           // 禁用模式 - 28-34关节  
motor_state.temperature({{0, 0}}); // TODO: 温度数据未实现
```

### 4.4 关节映射策略

**正确实现的映射**:
- KUAVO 28个关节 → DDS电机 0-27 (直接映射)
- DDS电机 28-34 → 全零填充(未使用槽位)
- 控制命令和状态反馈都遵循此映射关系

---

## 5. 数据映射和转换

### KUAVO适配Unitree协议策略

由于使用Unitree的标准IDL定义，KUAVO机器人需要将自身的数据结构映射到Unitree协议格式：

**完全匹配的字段**:
- 关节位置、速度、力矩 → 直接映射
- IMU四元数、陀螺仪、加速度计 → 语义一致，直接映射
- CRC校验、时间戳 → 通信机制相同

**复用字段策略**:
- `rpy_[3]` ← `free_acc` (自由加速度复用欧拉角字段)
- `reserve_[0-1]` ← 时间戳存储 (复用保留字段)
- `motor_state_[28-34]` ← 零值填充 (未使用的电机槽位)

**保持兼容性**:
- Unitree特有字段保持默认值，避免协议冲突
- 数据范围和类型转换确保通信稳定
