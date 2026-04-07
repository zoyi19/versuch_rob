电机跟随性测试脚本使用说明 v2.0
========================================

## 概述

motor_follow_test 是一个电机跟随性能测试工具，支持实物和仿真两种模式。
测试完成后会自动运行对称性分析脚本，生成详细报告和波形图。

## 快速开始

###  编译项目

```bash
cd ~/kuavo_ws
catkin build hardware_node
source devel/setup.bash
```

###  运行测试

#### 实物模式（默认）

```bash
# 全身测试
roslaunch hardware_node motor_follow_test.launch mode:=real

# 腿部测试
roslaunch hardware_node motor_follow_test.launch mode:=real test_mode:=1

# 手臂测试
roslaunch hardware_node motor_follow_test.launch mode:=real test_mode:=2
```

**注意**: 机器人初始化会先进入标定姿态，启动程序前请将机器人抬高！

#### 仿真模式

```bash
# 全身测试（固定base）
roslaunch hardware_node motor_follow_test.launch mode:=sim fixed_base:=true

# 腿部测试（不固定base）
roslaunch hardware_node motor_follow_test.launch mode:=sim test_mode:=1 fixed_base:=false
```

## 测试参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | real | 测试模式：`real`(实物) 或 `sim`(仿真) |
| `test_mode` | 0 | 测试范围：0=全身, 1=腿部, 2=手臂 |
| `robot_version` | 13 | 机器人版本号 |
| `fixed_base` | true | 仿真模式下是否固定base |

## 自动分析功能 ✨

测试完成后会自动运行对称性分析，无需手动操作！

### 自动生成的文件

1. **测试数据** (`motorTest/file/`)
   - `motorPair_<索引>_L<左关节>_R<右关节>.txt`

2. **分析报告** (当前目录)
   - `symmetry_analysis_report_<时间戳>.txt`
   - 包含详细的测试结果和失败原因

3. **波形图** (`motorTest/pics/`)
   - `motor_pair_<索引>_L<左关节>_R<右关节>.png`
   - 每个关节对的目标位置vs实际位置对比图

### 查看结果

```bash
cd ~/kuavo_ws/src/kuavo-ros-control-lejulib/hardware_node/src/tests/motorTest

# 查看分析报告
cat symmetry_analysis_report_*.txt

# 查看波形图
cd pics
ls *.png
```

## 手动运行分析（可选）

如果需要重新分析已有数据：

```bash
cd ~/kuavo_ws/src/kuavo-ros-control-lejulib/hardware_node/src/tests/motorTest

# 使用默认数据目录
python3 symmetry_analysis.py

# 指定数据目录和机器人版本
python3 symmetry_analysis.py ./file --robot-version 45

# 不保存图片（仅显示）
python3 symmetry_analysis.py --no-save
```

## 配置文件

测试参数配置文件位置：
```
hardware_node/src/tests/motorTest/config/joint_control_params_v<版本号>.json
```

可调整参数：
- `action_scale_k`: 各关节的运动幅度
- `action_scale_W`: 各关节的运动频率 (Hz)
- `joint_kp`: 各关节的比例增益
- `joint_kd`: 各关节的微分增益

## 分析阈值调整

编辑 `symmetry_analysis.py` 修改测试标准：

```python
ERROR_VAR_THRESHOLD = 0.2      # 误差方差阈值
SIMILARITY_THRESHOLD = 0.95    # 相似性阈值
SYMMETRY_THRESHOLD = 0.7      # 对称性阈值
```

## 常见问题

### Q: 测试失败，显示关节异常？
A: 查看生成的报告文件，检查失败原因：
   - 误差方差过高：检查关节控制参数
   - 相似性不足：检查左右关节是否对称
   - 数据异常：检查传感器连接

### Q: Python脚本执行失败？
A: 确保已安装必要的Python包：
```bash
pip3 install numpy matplotlib scipy
```

### Q: 如何禁用自动分析？
A: 编辑 `motor_follow_test.cpp`，注释掉 `runSymmetryAnalysis()` 调用

### Q: 仿真时机器人倒地？
A: 使用 `fixed_base:=true` 参数固定base

## 配置文件向下兼容 ✨

程序会自动查找兼容的配置文件：

**规则**：
- 如果找不到当前版本的配置文件，会自动向下查找同一十位数的版本
- 例如：版本47找不到，会依次尝试46, 45, 44, 43, 42, 41, 40
- 如果该十位数范围内都找不到，则报错

**示例**：
```bash
# 版本47，但只有v40的配置文件
export ROBOT_VERSION=47
roslaunch hardware_node motor_follow_test.launch mode:=real

# 程序会自动使用v40的配置文件
# 输出：版本 47 的配置文件不存在，使用兼容版本 40
```

**适用范围**：
- kuavo.json 配置文件（机器人参数）
- joint_control_params_v*.json 配置文件（控制参数）

## 版本历史

- v2.0: 添加自动分析功能，支持仿真模式，配置文件向下兼容
- v1.0: 初始版本，仅支持实物测试

## 更多文档

详细文档请参考：
- `motor_follow_test_usage.md` - 完整使用说明
- `mujoco_constraint_mechanism.md` - MuJoCo固定机制
- `ankle_data_flow_analysis.md` - 踝关节数据流分析
