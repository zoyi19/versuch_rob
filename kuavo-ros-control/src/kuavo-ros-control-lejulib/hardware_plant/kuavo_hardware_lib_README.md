# Kuavo Hardware Lib

本项目是基于 KUAVO 硬件的 SDK，提供电机控制、传感器数据处理和末端执行器管理的统一接口，包含测试用例以及 CMake 示例，方便用户快速上手。

## 项目结构

```bash
kuavo_hardware_lib/
├── bin
├── docs
├── examples            # 示例代码
│   ├── hw_wrapped      # 硬件接口封装示例
│   └── quick_start
├── include
│   └── hardware_plant/    # 对外接口头文件
├── lib
├── share
└── tests
    ├── CMakeLists.txt
    ├── hardware_plant_test.cpp
    └── ...
```

## 安全须知

- 操作前确保机器人已正确吊装，双脚离地
- 确保周围无人员和障碍物，保持安全距离
- 熟悉急停按钮位置，遇到异常立即按下急停
- 调试过程中避免站在机器人运动范围内

## 快速开始

### 1. 克隆代码

```bash
git clone https://www.lejuhub.com/highlydynamic/kuavo_hardware_lib.git
```

### 2. 编译测试程序

```bash
cd kuavo_hardware_lib
source hardware_lib_setup.sh # !!! IMPORTANT: 需设置一些环境变量，编译需要
mkdir -p tests/build && cd tests/build
cmake .. && make -j$(nproc)
```

### 3. 运行测试

> **运行前准备**：
> - 将机器人吊起，确保双脚伸直不触地
> - 开启电源并松开急停按钮（无需使机器人站立）
> - 将手臂摆正到零点位置

```bash
# 运行测试程序
sudo ./tests/build/hardware_plant_test
```

## 示例代码

本库提供了丰富的示例代码，帮助快速上手。

**推荐示例**: `hw_wrapped` - 提供简洁易用的硬件接口封装，是使用 Kuavo 硬件库的最佳实践。

### 快速运行示例

```bash
# 配置环境
source hardware_lib_setup.sh
export ROBOT_VERSION=45

# 构建 hw_wrapped 示例
cd examples/hw_wrapped
mkdir -p build && cd build
cmake .. && make

# 运行示例
./wrapped_example    # 基础示例
./imu_example        # IMU 数据读取
./joint_example      # 正弦波关节控制
```

详细文档请参阅 [examples/README.md](examples/README.md)

## 在项目中使用

> **注意**: 本库仅公开 `hardware_plant/hardware_plant.h` 作为对外接口，控制硬件应当使用该文件中提供的接口，请勿直接使用其他内部头文件。

### CMakeLists.txt 配置

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(your_project)

# 查找 hardware_plant 库
find_package(hardware_plant REQUIRED)

# 添加可执行文件
add_executable(your_executable main.cpp)

# 链接库
target_link_libraries(your_executable
    PUBLIC
        hardware_plant::hardware_plant
)
```

### 代码示例

```cpp
#include "hardware_plant/hardware_plant.h"

using namespace HighlyDynamic;

int main() {
    // 配置硬件参数
    HardwareParam param;
    param.robot_version = RobotVersion(4, 5);  // v4.5
    param.kuavo_assets_path = std::getenv("KUAVO_ASSETS_PATH");

    // 初始化硬件
    HardwarePlant plant(0.001, param);  // 1ms 控制周期
    plant.HWPlantInit();

    // 读取传感器数据
    SensorData_t sensor_data;
    plant.readSensor(sensor_data);

    // 关节运动控制
    std::vector<double> target_pos(28, 0.0);
    plant.jointMoveTo(target_pos, 30.0, 0.02);  // 速度 30 deg/s

    plant.HWPlantDeInit();
    return 0;
}
```

## 文档

- [硬件接口文档](docs/HARDWARE_PLANT_INTERFACES.md) - HardwarePlant 类接口详细说明
