# quick_start 示例

quick_start 是最基础的入门示例，展示如何初始化 `HardwarePlant` 硬件接口。

## 目录结构

```
quick_start/
├── CMakeLists.txt              # 构建配置
├── quick_start_example.cpp     # 基础初始化示例
└── README.md
```

## 环境配置

在构建和运行前，需要先配置环境变量：

```bash
# 设置库路径和资源路径
source hardware_lib_setup.sh

# 设置机器人版本
export ROBOT_VERSION=45
```

## 构建

```bash
cd examples/quick_start
mkdir -p build && cd build
cmake ..
make
```

## 运行

```bash
./quick_start_example
```

预期输出：
```
使用机器人版本: 4.5
kuavo_assets_path: /path/to/share/kuavo_assets
准备初始化硬件...
硬件初始化成功!
```

## 代码说明

该示例展示了 `HardwarePlant` 的最小初始化流程：

1. **获取机器人版本**：从环境变量 `ROBOT_VERSION` 读取
2. **配置硬件参数**：设置 `HardwareParam` 结构体
3. **初始化硬件**：创建 `HardwarePlant` 实例并调用 `HWPlantInit()`

## 核心类

### HardwarePlant

核心硬件抽象类，位于 `hardware_plant.h`，管理所有硬件接口：

```cpp
namespace HighlyDynamic {

class HardwarePlant {
public:
    HardwarePlant(double dt, const HardwareParam& param, const std::string& project_dir);
    void HWPlantInit();
    // ... 更多方法
};

}
```

### HardwareParam

硬件初始化参数结构体：

```cpp
struct HardwareParam {
    RobotVersion robot_version;  // 机器人版本
    std::string kuavo_assets_path;  // 资源文件路径
    // ... 其他参数
};
```

## 使用示例

```cpp
#include "hardware_plant.h"

using namespace HighlyDynamic;

int main() {
    // 1. 配置参数
    HardwareParam hardware_param;
    hardware_param.robot_version = RobotVersion::create(45);
    hardware_param.kuavo_assets_path = KUAVO_ASSETS_PATH;

    // 2. 创建 HardwarePlant (1ms 控制周期)
    auto hardware_plant = std::make_unique<HardwarePlant>(
        0.001, hardware_param, std::string(PROJECT_SOURCE_DIR)
    );

    // 3. 初始化硬件
    hardware_plant->HWPlantInit();

    // 4. 在此添加控制逻辑...

    return 0;
}
```