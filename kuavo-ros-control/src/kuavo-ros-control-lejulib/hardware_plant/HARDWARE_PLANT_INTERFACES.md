# HardwarePlant 接口文档

本文档列出了 HardwarePlant 类中所有关于硬件相关的读写接口，包含函数签名、返回值和参数。

**非常重要的提示**：
**非常重要的提示**：
**非常重要的提示**：
- 本文档提供的接口中，存在电机空间(motor space)与关节空间(joint space)的概念，区别在与是否经过内部的脚踝解算等操作。
- 请务必在已经清楚接口对应的数据空间的情况下，使用它。


## 重要数据结构

### JointParam_t / MotorParam_t 结构体
用于关节/电机控制和反馈的核心数据结构：
```cpp
typedef struct {
  double position;        // 位置指令/反馈 (rad)
  double velocity;        // 速度指令/反馈 (rad/s)  
  double torque;          // 扭矩指令/反馈 (N·m)
  double maxTorque;       // 最大扭矩限制 (N·m)
  double positionOffset;  // 位置偏移 (rad)
  double velocityOffset;  // 速度偏移 (rad/s)
  double torqueOffset;    // 扭矩偏移 (N·m)
  double acceleration;    // 加速度 (rad/s²)
  double kp;              // 比例增益
  double kd;              // 微分增益
  uint8_t status;         // 电机状态标志位
  uint16_t status_word;   // 电机状态字
  uint16_t error_code;    // 错误代码
  double torque_demand_trans; // 扭矩需求转换值 (N·m)
} MotorParam_t;
```

### SensorData_t 结构体
用于传感器数据传输和状态反馈的核心数据结构：
```cpp
typedef struct {
  double time;                               // 时间戳 (s)
  Eigen::VectorXd joint_q;                   // 位置 (rad)        !!! 注意该字段即可以用来表示joint_q也可用表示motor_position, 这取决与对应的接口 !!!
  Eigen::VectorXd joint_v;                   // 速度 (rad/s)      !!! 注意该字段即可以用来表示joint_v也可用表示motor_velocity, 这取决与对应的接口 !!!
  Eigen::VectorXd joint_vd;                  // 加速度 (rad/s²)    !!! 注意该字段即可以用来表示joint_vd也可用表示motor_velocity_d, 这取决与对应的接口 !!!
  Eigen::VectorXd joint_current;             // 电流 (A) - 实际为扭矩反馈值  
  Eigen::Vector3d gyro;                      // 陀螺仪数据 (rad/s) 
  Eigen::Vector3d acc;                       // 加速度计数据 (m/s²)
  Eigen::Vector3d free_acc;                  // 去重力加速度 (m/s²)
  Eigen::Vector4d quat;                      // 四元数姿态 [w,x,y,z]
  Eigen::Vector3d gyro_W;                    // 陀螺仪数据 (rad/s)
  Eigen::Vector3d acc_W;                     // 加速度计数据 (m/s²)
  Eigen::Vector3d free_acc_W;                // 去重力加速度 (m/s²)
  std::vector<EndEffectorData> end_effectors_data; // 末端执行器数据
  
} SensorData_t;
```

### 控制模式常量
```cpp
#define CST 0  // 扭矩控制模式 (Cyclic Synchronous Torque)
#define CSV 1  // 速度控制模式 (Cyclic Synchronous Velocity)  
#define CSP 2  // 位置控制模式 (Cyclic Synchronous Position)
```

### 核心控制接口

```cpp
void writeCommand(Eigen::VectorXd cmd_r, 
                  uint32_t na_r, 
                  std::vector<int> control_modes, 
                  Eigen::VectorXd &joint_kp, 
                  Eigen::VectorXd &joint_kd);
```
- **数据空间**: 电机空间 (Motor Space)
- **返回值**: void
- **作用**: 构造不同控制模式下的 joint_data 向电机发送控制指令
- **参数**: 
  - `cmd_r`: 电机指令向量，长度为na_r * 5，结构化存储5组数据：
    - `cmd_r[0 ~ na_r-1]`: 位置指令 (rad)
    - `cmd_r[na_r ~ 2*na_r-1]`: 速度指令 (rad/s)  
    - `cmd_r[2*na_r ~ 3*na_r-1]`: 扭矩指令 (N·m)
    - `cmd_r[3*na_r ~ 4*na_r-1]`: 最大扭矩限制 (N·m)
    - `cmd_r[4*na_r ~ 5*na_r-1]`: 扭矩比例系数
  - `na_r`: 执行器数量 (count)
  - `control_modes`: 每个电机的控制模式数组：
    - `0 (CST)`: 扭矩控制模式
    - `1 (CSV)`: 速度控制模式  
    - `2 (CSP)`: 位置控制模式
  - `joint_kp`: 关节比例增益系数(用于力矩控制, 下肢电机)
  - `joint_kd`: 关节微分增益系数(用于力矩控制，下肢电机)


Notes: 可使用`cmds2Cmdr()`接口完成 JointSpace 2 MotorSpace 之后下发电机指令。

### 传感器和状态管理

```cpp
bool readSensor(SensorData_t &sensor_data);
```
- **数据空间**: 电机空间 (Motor Space)
- **返回值**: bool（成功/失败）
- **作用**: 从硬件读取传感器数据，包括电机状态和IMU数据
- **参数**: 
  - `sensor_data`: 传感器数据结构（引用），包含以下字段：
    - `time`: 时间戳 (s)
    - `joint_q`: 电机位置 (rad)
    - `joint_v`: 电机速度 (rad/s)
    - `joint_vd`: 电机速度导数 (rad/s²)
    - `joint_current`: 电机电流 (A)
    - `gyro`: 陀螺仪数据 (rad/s)
    - `acc`: 加速度计数据 (m/s²)
    - `free_acc`: 去重力加速度 (m/s²)
    - `quat`: 四元数姿态 
    - `end_effectors_data`: 末端执行器数据

notes: 可使用`motor2joint()`接口完成 MotorSpace 2 JointSpace 之后转为关节数据。

### 关节控制和运动

```cpp
void jointMoveTo(std::vector<double> goal_pos, 
                 double speed, 
                 double dt = 1e-3, 
                 double current_limit = -1);
```

- **数据空间**: 关节空间 (Joint Space)
- **返回值**: void
- **作用**: 控制关节移动到指定位置，常用于校准和初始化定位
- **参数**: 
  - `goal_pos`: 目标关节位置 (deg)，单位为度
  - `speed`: 运动速度 (deg/s)，表示关节运动的最大速度
  - `dt`: 控制周期时间步长 (s, default: 1e-3)
  - `current_limit`: 电流限制 (A, default: -1表示使用默认限制)
- **实际使用**: 
  - 校准运动: `jointMoveTo(moving_pos, 60.0)` - 使用60度/秒速度
  - 初始定位: `jointMoveTo(goal_pos, 45.0)` - 使用45度/秒速度
  - 服务调用: `jointMoveTo(req.goal_position, req.speed, req.dt)` - 自定义参数

```cpp
inline void SetMotorVelocity(const std::vector<uint8_t> &joint_ids, 
                             std::vector<MotorParam_t> &motor_data);
```

- **数据空间**: 电机空间 (Motor Space)
- **返回值**: void
- **作用**: 设置电机速度指令，按驱动器类型分发到相应底层接口
- **参数**: 
  - `joint_ids`: 电机对应的关节ID
  - `motor_data`: 电机数据, 由 `WriteCommand` 构造
- **底层调用**: 
  - **EC_MASTER电机**: `actuators.setJointVelocity()` - 支持EtherCAT速度控制
  - **其他驱动器**: 当前未实现速度控制接口

```cpp
inline void SetMotorTorque(const std::vector<uint8_t> &joint_ids, 
                           std::vector<MotorParam_t> &motor_data);
```

- **数据空间**: 电机空间 (Motor Space)
- **返回值**: void
- **作用**: 设置电机指令，按驱动器类型分发到相应底层接口
- **参数**: 
  - `joint_ids`: 电机对应的关节ID
  - `motor_data`: 电机数据 `WriteCommand` 构造
- **底层调用**: 
  - **EC_MASTER电机**: `actuators.setJointTorque()` - 支持EtherCAT扭矩控制
  - **DYNAMIXEL电机**: 当前接口已注释，未激活
  - **REALMAN电机**: 当前接口已注释，未激活

```cpp
inline void SetMotorPosition(const std::vector<uint8_t> &joint_ids, 
                             std::vector<MotorParam_t> &motor_data);
```

- **数据空间**: 电机空间 (Motor Space)
- **返回值**: void
- **作用**: 设置电机位置指令，按驱动器类型分发到相应底层接口
- **参数**: 
  - `joint_ids`: 电机对应的关节ID
  - `motor_data`: 电机数据, 由 `WriteCommand` 构造
- **底层调用**: 
  - **EC_MASTER电机**: `actuators.setJointPosition()` - EtherCAT位置控制
  - **RUIWO电机**: `ruiwo_actuator->set_positions()` - 手臂电机位置控制
  - **DYNAMIXEL电机**: 当前接口已注释，未激活
  - **REALMAN电机**: 当前接口已注释，未激活

```cpp
inline void GetMotorData(const std::vector<uint8_t> &joint_ids, 
                         std::vector<MotorParam_t> &motor_data);
```

- **WRAN**: 函数名称中的`Joint`与实际不符，实际本接口为`Motor`.
- **数据空间**: 电机空间 (Motor Space)
- **返回值**: void
- **作用**: 获取电机反馈数据，按驱动器类型分发到相应底层接口
- **参数**: 
  - `joint_ids`: 电机对应的关节ID
  - `motor_data`: (引用)电机数据
- **底层调用**: 
  - **EC_MASTER电机**: `actuators.getJointData()` - 获取EtherCAT电机状态
  - **RUIWO电机**: `ruiwo_actuator->get_positions/get_velocity/get_torque()` - 获取手臂电机状态
  - **DYNAMIXEL电机**: 当前接口已注释，未激活
  - **REALMAN电机**: 当前接口已注释，未激活
- **数据转换**: RUIWO电机数据从弧度转换为度数后存储到joint_data


### 末端执行器控制

```cpp
void endEffectorCommand(std::vector<EndEffectorData> &end_effector_cmd);
```

- **返回值**: void
- **作用**: 发送末端执行器控制指令，支持不同类型的末端执行器
- **参数**: 
  - `end_effector_cmd`: 末端执行器指令数据向量（引用）
- **实现**: 
  - 遍历指令向量，根据末端执行器类型分发到对应控制器
  - `EndEffectorType::jodell`: 提取位置数据发送到夹爪控制器
  - `EndEffectorType::qiangnao/qiangnao_touch`: 提取位置数据发送到灵巧手控制器

### EndEffectorData 结构体
```cpp
typedef struct {
  EndEffectorType type;      // 末端执行器类型
  Eigen::VectorXd position;  // 位置指令 (具体单位依赖执行器类型)
  Eigen::VectorXd velocity;  // 速度指令 (具体单位依赖执行器类型)  
  Eigen::VectorXd torque;    // 扭矩指令 (具体单位依赖执行器类型)
} EndEffectorData;
```

### EndEffectorType 枚举
```cpp
enum EndEffectorType {
  none,              // 无末端执行器
  jodell,            // 乐聚夹爪
  qiangnao,          // 强脑灵巧手
  lejuclaw,          // 乐聚夹爪(新版本)
  qiangnao_touch,    // 强脑触觉灵巧手
  revo2              // Revo2灵巧手
};
```

## 末端执行器类型配置说明

### 不同末端执行器的数组长度和关节映射

| 末端执行器类型 | position长度 | velocity长度 | torque长度 | 关节映射 | 备注 |
|---------------|-------------|-------------|-----------|---------|------|
| **none** | 0 | 0 | 0 | 无关节 | 无末端执行器配置 |
| **jodell** | 2 | 2 | 2 | 单关节夹爪 | 乐聚机械夹爪，单自由度开合 |
| **qiangnao** | 12 | 12 | 12 | 每只手6个关节 | 强脑灵巧手，双手共12个自由度 |
| **lejuclaw** | 2 | 2 | 2 | 乐聚夹爪 | 乐聚夹爪，单个夹爪控制 |
| **qiangnao_touch** | 12 | 12 | 12 | 每只手6个关节 | 强脑触觉灵巧手，双手共12个自由度 |
| **revo2** | 12 | 12 | 12 | 每只手6个关节 | Revo2灵巧手，双手共12个自由度 |

### 末端执行器控制方式说明

**重要**: 所有末端执行器仅支持 **position 控制**，position 范围统一为 **0-100**：
- **0**: 张开状态
- **100**: 闭合状态

### 双手配置时的数据组织方式

**灵巧手类型** (qiangnao, qiangnao_touch, revo2):
- **总长度**: 12 (左手6 + 右手6)
- **数据组织**: `[左手0-5, 右手0-5]`
- **单手关节**: 6个手指关节
- **控制方式**: 每只手独立控制6个自由度，范围0-100

**夹爪类型** (jodell, lejuclaw):
- **jodell**: 2关节，控制夹爪开合
- **lejuclaw**: 2个关节，控制双夹爪
- **控制方式**: 位置控制，范围0-100 (0=张开, 100=闭合)

### 获取灵巧手末端执行器状态
```cpp
eef_controller::FingerStatusArray getHandControllerStatus();
```

- **返回值**: eef_controller::FingerStatusArray（手部控制器状态）
- **作用**: 获取灵巧手手指状态
- **参数**: 无

### FingerStatusArray 类型定义
```cpp
using FingerStatusArray = std::array<dexhand::FingerStatus, 2>;
```
- **说明**: 数组包含2个元素，分别对应左手和右手的手指状态
- **内容**: 每个 `dexhand::FingerStatus` 包含手指的位置、速度、力矩等状态信息

```cpp
eef_controller::ClawState getLejuClawState();
```

- **返回值**: eef_controller::ClawState（当前夹爪状态）
- **作用**: 获取当前乐聚夹爪状态信息
- **参数**: 无

### ClawState 结构体
```cpp
struct ClawState {
    JointState data;                   // 关节状态数据
    std::vector<int8_t> state = {0, 0}; // 夹爪状态 {左爪, 右爪}
};

struct JointState {
    std::vector<std::string> name;     // 关节名称
    std::vector<double> position;      // 位置 (0-100, 0=张开, 100=闭合)
    std::vector<double> velocity;      // 速度 (未使用)
    std::vector<double> effort;        // 力矩 (未使用)
};
```

### 末端执行器控制总结

**统一控制模式**:
- 所有末端执行器仅支持位置控制模式
- 位置范围统一为 0-100 (0=张开状态, 100=闭合状态)
- velocity 和 torque 字段在数据结构中定义但实际未使用
- 不同类型末端执行器的关节数量不同，但控制逻辑统一
