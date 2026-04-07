# 电机跟随测试对称性评估系统

## 概述

本系统为电机跟随测试提供了完整的对称性评估功能，包括数据记录、保存和分析。

## 系统架构

### C++ Core模块
- **数据记录**: 实时记录关节命令和响应数据
- **数据保存**: 将测试数据保存到本地文件
- **测试管理**: 管理测试流程和状态

### Python分析脚本
- **对称性评估**: 计算误差方差、频谱相似度和对称性指标
- **报告生成**: 生成详细的测试报告
- **可视化**: 绘制测试结果图表

## 文件结构

```
motorTest/
├── motor_follow_test_core.h          # Core模块头文件
├── motor_follow_test_core.cpp        # Core模块实现
├── symmetry_analysis.py              # Python对称性分析脚本
├── README_symmetry.md               # 本说明文件
└── test_data/                       # 测试数据目录（运行时创建）
    ├── inputData_0_0.txt            # 测试对0左关节输入数据
    ├── responseData_0_0.txt         # 测试对0左关节响应数据
    ├── inputData_0_1.txt            # 测试对0右关节输入数据
    ├── responseData_0_1.txt         # 测试对0右关节响应数据
    └── ...
```

## 使用方法

### 1. 运行电机跟随测试

```bash
# 仿真模式
roslaunch hardware_node motor_follow_test.launch mode:=sim robot_version:=13

# 实物模式
roslaunch hardware_node motor_follow_test.launch mode:=real robot_version:=13
```

### 2. 数据记录

测试过程中，Core模块会自动：
- 记录每个关节对的命令和响应数据
- 在每个测试对完成后保存数据到文件
- 数据格式：`timestamp\tvalue`

### 3. 对称性分析

测试完成后，运行Python分析脚本：

```bash
# 使用默认数据目录
python3 symmetry_analysis.py

# 指定数据目录
python3 symmetry_analysis.py /path/to/test_data

# 不显示图表
python3 symmetry_analysis.py --no-plot

# 不保存报告和图表
python3 symmetry_analysis.py --no-save
```

## 评估指标

### 1. 误差方差 (Error Variance)
- **定义**: 输入信号与响应信号之间误差的方差
- **阈值**: 0.05
- **意义**: 衡量跟踪精度，值越小表示跟踪越准确

### 2. 频谱相似度 (Spectral Similarity)
- **定义**: 左右关节响应信号的相关系数
- **阈值**: 0.98
- **意义**: 衡量左右关节响应的一致性，值越接近1表示越一致

### 3. 对称性指标 (Symmetry Metrics)
- **定义**: 基于左右误差信号差分积分的对称性评估
- **阈值**: 0.85
- **意义**: 衡量左右关节的对称性，值越接近1表示越对称

## 输出文件

### 数据文件
- `inputData_{pair_index}_{joint_id}.txt`: 关节输入数据
- `responseData_{pair_index}_{joint_id}.txt`: 关节响应数据

### 分析报告
- `symmetry_analysis_report_{timestamp}.txt`: 详细分析报告
- `symmetry_analysis_plots_{timestamp}.png`: 结果图表

## 报告内容

分析报告包含：
1. **测试信息**: 时间、数据目录、测试对数
2. **详细结果**: 每个测试对的各项指标
3. **统计结果**: 平均值、异常数量等

## 依赖库

### Python依赖
```bash
pip install numpy scipy matplotlib pandas
```

## 配置参数

可以在Core模块中调整以下参数：
- `data_save_path_`: 数据保存路径
- 评估阈值：误差方差、相似度、对称性阈值

## 故障排除

### 常见问题

1. **数据文件不存在**
   - 检查测试是否正常运行
   - 确认数据保存路径正确

2. **Python脚本运行失败**
   - 检查依赖库是否安装
   - 确认数据文件格式正确

3. **图表不显示**
   - 检查matplotlib后端配置
   - 使用`--no-plot`参数跳过图表显示


