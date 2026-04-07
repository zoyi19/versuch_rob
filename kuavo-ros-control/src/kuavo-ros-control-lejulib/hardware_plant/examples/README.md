# Kuavo Hardware Library 示例

本目录包含 Kuavo 硬件库的使用示例。

## 推荐示例：hw_wrapped

**强烈推荐使用 `hw_wrapped` 作为参考示例。** 它提供了简洁易用的硬件接口封装，是使用 Kuavo 硬件库的最佳实践。

### 为什么选择 hw_wrapped？

- **简洁的 API**: 对 hardware_plant 库的封装，提供更简洁易用的硬件接口
- **类型安全**: 清晰的数据结构定义（JointData_t、JointCommand_t、ImuData_t）
- **错误处理**: 统一的错误码返回，便于调试

### 快速开始

```bash
# 1. 配置环境
source hardware_lib_setup.sh
export ROBOT_VERSION=45  # Kuavo4Pro

# 2. 构建
cd examples/hw_wrapped
mkdir -p build && cd build
cmake .. && make

# 3. 运行
./wrapped_example    # 基础示例
./imu_example        # IMU 数据读取
./joint_example      # 正弦波关节控制 (CSP模式)
```

### 代码示例

```cpp
#include "hw_wrapped/hw_wrapped.h"

using namespace leju::hw;

int main() {
    // 获取硬件接口
    auto& hw = HardwareInterface::GetInstance();

    // 初始化
    if (hw.Initialize() != HwErrorType::Success) {
        return 1;
    }

    // 读取关节状态
    JointData_t joint_data;
    ImuData_t imu_data;
    hw.GetSensorsState(joint_data, imu_data);
    
    std::cout << joint_data << std::endl;
    std::cout << imu_data << std::endl;

    // 关闭
    hw.Shutdown();
    return 0;
}
```

### hw_wrapped 目录结构

```
hw_wrapped/
├── CMakeLists.txt
├── README.md                   # 详细文档
├── wrapped_example.cpp         # 基础示例
├── imu_example.cpp             # IMU 读取示例
├── joint_example.cpp           # 正弦波关节控制示例
└── wrapped/                    # 静态库源码
    ├── include/hw_wrapped/
    │   ├── hw_types.h          # 数据类型
    │   └── hw_wrapped.h        # 接口声明
    └── src/
```

详细文档请参阅 [hw_wrapped/README.md](hw_wrapped/README.md)

---

## 其他示例

| 目录 | 说明 |
|------|------|
| [hw_wrapped](hw_wrapped/) | **推荐** - 硬件接口封装库及示例 |
| [quick_start](quick_start/) | 快速开始示例（直接使用 hardware_plant） |

## 环境要求

- 已安装 kuavo_hardware_lib
- 已配置环境变量（通过 `source hardware_lib_setup.sh`）
- 已设置 `ROBOT_VERSION` 环境变量
