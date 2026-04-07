# 灵巧手SDK

对强脑灵巧手SDK的统一封装，提供统一的C++接口，支持Modbus、Protobuf、CAN总线通信。

基于 BrainCo STARK SDK v1.1.3，统一 0~1000 控制量程（API 层保持 0~100）。

```
dexhand_sdk/
├── src/                              # 实现文件
│   ├── stark_dexhand.cpp             # StarkDexhand 自动检测协议实现
│   ├── modbus.cpp                    # ModbusDexhand 基类实现
│   ├── touch_dexhand.cpp             # 触觉灵巧手实现
│   ├── revo1_hand.cpp                # Revo1 Modbus RS485 实现
│   ├── revo2_hand.cpp                # Revo2 Modbus RS485 实现
│   ├── revo1_hand_can_customed.cpp   # Revo1 CAN 实现
│   ├── revo2_hand_can_customed.cpp   # Revo2 CAN 实现
│   └── protobuf_dexhand.cpp          # (已废弃，保留未编译)
├── dexhand_base.h                    # 基础类定义
├── stark_dexhand.h                   # StarkDexhand 自动检测接口
├── modbus.h                          # ModbusDexhand 基类接口
├── touch_dexhand.h                   # 触觉灵巧手接口
├── revo1_hand.h                      # Revo1 Modbus 接口
├── revo2_hand.h                      # Revo2 Modbus 接口
├── revo1_hand_can_customed.h         # Revo1 CAN 接口
├── revo2_hand_can_customed.h         # Revo2 CAN 接口
├── protobuf_dexhand.h                # (已废弃，保留未编译)
├── modbus_sdk/                       # STARK SDK 库 (libbc_stark_sdk.so v1.1.3)
├── protobuf_sdk/                     # 旧 Protobuf SDK 库 (保留)
├── third_party/                      # 第三方库 (libserialport，保留)
├── tests/                            # 测试程序
└── scripts/                          # 脚本文件
```

## 设备类型

#### 1. StarkDexhand (自动检测)
- **类名**: StarkDexhand
- **中文名称**: 通用灵巧手（自动检测协议）
- **头文件**: `stark_dexhand.h`
- **通信协议**: 自动检测 — 先 `stark_auto_detect` (Modbus)，失败回退 Protobuf
- **支持设备**: Revo1 (Protobuf)、Revo1Advanced (Modbus)、Revo2 (Modbus)
- **说明**: 替代原 ProtobufDexhand，用于 `qiangnao` 类型的 RS485 非触觉手

#### 2. TouchDexhand (V1_TOUCH)
- **类名**: TouchDexhand
- **中文名称**: 触觉灵巧手
- **头文件**: `touch_dexhand.h`
- **通信协议**: 自动检测 — 先 `stark_auto_detect`，失败回退 `modbus_open`
- **支持设备**: Revo1Touch、Revo1AdvancedTouch
- **说明**: 继承 ModbusDexhand，额外提供触觉传感器接口

#### 3. Revo2Dexhand (V2_STANDARD)
- **类名**: Revo2Dexhand
- **中文名称**: Roban Revo2 二代灵巧手 (RS485)
- **头文件**: `revo2_hand.h`
- **通信协议**: Modbus RTU

#### 4. Revo1CanDexhand (V1_STANDARD)
- **类名**: Revo1CanDexhand
- **中文名称**: Revo1 灵巧手 (CAN)
- **头文件**: `revo1_hand_can_customed.h`
- **通信协议**: CAN 2.0

#### 5. Revo2CanDexhand (V2_STANDARD)
- **类名**: Revo2CanDexhand
- **中文名称**: Roban Revo2 二代灵巧手 (CAN)
- **头文件**: `revo2_hand_can_customed.h`
- **通信协议**: CAN FD

## 量程说明

API 层统一使用 **0~100** 范围（位置/速度），SDK 内部统一使用 **0~1000**。

| 协议 | setFingerPositions | getFingerStatus |
|------|-------------------|-----------------|
| Modbus / CAN | API 0~100 x10 -> SDK 0~1000 | SDK 0~1000 /10 -> API 0~100 |
| Protobuf | API 0~100 x10 -> SDK 0~1000 | SDK 0~1000 /10 -> API 0~100 |

SDK v1.1.3 所有协议统一 0~1000 量程，SDK 内部自动处理协议差异。

## 编译

由于 dexhand_sdk 依赖 canbus_sdk，需要先编译 canbus_sdk：

```bash
# 编译canbus_sdk
cd ../canbus_sdk
mkdir build && cd build
cmake ..
make

# 返回dexhand_sdk目录并编译
cd ../../dexhand_sdk
mkdir build && cd build
cmake ..
make
```

## 示例和测试

### 运行示例和测试程序

在项目根目录中提供了可运行的示例程序：

```bash
cd build

# 运行自动检测测试 (RS485，支持 revo1/revo1adv/revo2)
./stark_dexhand_test

# 运行CAN Revo2通信测试
sudo ./revo2_can_test

# 运行CAN Revo1通信测试
sudo ./revo1_can_test

# 运行基础测试程序
./dexhand_test
# Usage: ./dexhand_test --touch|--normal|--revo2 [--test [round]] [--scan]
# Options:
#   --touch: 触觉手测试模式
#   --normal: 普通手测试模式
#   --revo2: Roban2 灵巧手测试模式
#   --test [round]: 测试灵巧手运动, round 为测试次数，默认为 5 次
#   --scan: 扫描设备, 识别 ttyUSB 设备, 并尝试控制设备运动
```

## 工具

### 设备规则生成工具
```bash
# 运行设备规则生成脚本
./scripts/gen_dexhand_serial_rules.sh
```

SDK支持自动设备识别，需要创建udev规则。

## 支持和文档
- **revo1 一代手文档**：https://www.brainco-hz.com/docs/revolimb-hand/revo1/parameters.html
- **revo2 二代手文档**：https://www.brainco-hz.com/docs/revolimb-hand/index.html
- **原生 SDK 示例**: https://github.com/BrainCoTech/stark-serialport-example
