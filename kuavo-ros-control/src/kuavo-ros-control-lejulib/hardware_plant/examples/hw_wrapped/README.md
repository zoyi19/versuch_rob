# hw_wrapped 示例

hw_wrapped 是对 hardware_plant 库的封装，提供更简洁易用的硬件接口。

## 目录结构

```
hw_wrapped/
├── CMakeLists.txt              # 顶层 cmake，构建示例程序
├── wrapped_example.cpp         # 基础示例程序
├── imu_example.cpp             # IMU 数据读取示例
├── joint_sin_example.cpp       # 正弦波关节控制示例
├── README.md
└── wrapped/                    # hw_wrapped 静态库
    ├── CMakeLists.txt
    ├── include/
    │   └── hw_wrapped/
    │       ├── hw_types.h      # 数据类型定义
    │       └── hw_wrapped.h    # 硬件接口类声明
    └── src/
        ├── hw_wrapped.cpp      # 输出流运算符实现
        ├── hw_wrapped_impl.cc  # HardwareInterface 实现
        └── hw_wrapped_log.h    # 日志宏定义
```

## 环境配置

在构建和运行前，需要先配置环境变量：

```bash
# 设置库路径和资源路径
source hardware_lib_setup.sh

# 设置机器人版本（Kuavo4Pro 使用 45）
export ROBOT_VERSION=45
```

## 构建

```bash
cd examples/hw_wrapped
mkdir -p build && cd build
cmake ..
make
```

## 运行

```bash
# 基础示例：初始化硬件并读取关节状态
./wrapped_example

# IMU 示例：循环读取并打印 IMU 数据
./imu_example

# 关节控制示例：正弦波位置控制
./joint_sin_example
```

## API 说明

### HardwareInterface

单例模式的硬件接口类，提供以下方法：

| 方法 | 说明 |
|------|------|
| `GetInstance()` | 获取单例实例 |
| `Initialize()` | 初始化硬件接口 |
| `Shutdown()` | 关闭硬件接口 |
| `GetMotorNumber()` | 获取电机数量 |
| `GetSensorsState(JointData_t&, ImuData_t&)` | 获取传感器状态（关节+IMU） |
| `SetJointCommand(JointCommand_t&)` | 设置关节控制命令 |
| `JointMoveTo(goal_pos, speed, dt, current_limit)` | 移动关节到目标位置 |

### 数据类型

- `JointData_t`: 关节状态数据（位置、速度、加速度、力矩）
- `JointCommand_t`: 关节控制命令（目标位置、速度、力矩、增益等）
- `ImuData_t`: IMU 数据（角速度、加速度、四元数）
- `HwErrorType`: 错误类型枚举

## 使用示例

```cpp
#include "hw_wrapped/hw_wrapped.h"

using namespace leju::hw;

int main() {
    // 获取硬件接口单例
    auto& hw = HardwareInterface::GetInstance();

    // 初始化
    if (hw.Initialize() != HwErrorType::Success) {
        return 1;
    }

    // 读取传感器状态
    JointData_t joint_data;
    ImuData_t imu_data;
    hw.GetSensorsState(joint_data, imu_data);
    std::cout << joint_data << std::endl;

    // 关闭
    hw.Shutdown();
    return 0;
}
```
