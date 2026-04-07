#!/bin/bash
# 
# 手臂臂与灵巧手ROS话题录制脚本

# 该脚本用于录制机械臂和灵巧手控制相关的ROS话题数据，保存为rosbag文件。
# 这些数据可以用于后续的分析、调试和回放测试。

# 使用方法：
# 1. 确保已经启动相应的ROS节点（如ruiwo_motor测试节点和灵巧手测试节点）
# 2. 运行此脚本开始录制：
#    ./record_arm_dexhand_topics.sh
# 3. 按Ctrl+C停止录制

# 录制的话题：
# - /ruiwo_motor/command：机械臂控制命令
# - /ruiwo_motor/state：机械臂状态反馈
# - /dexhand/command：灵巧手控制命令
# - /dexhand/state：灵巧手状态反馈

# 输出文件：
# - 保存到：$HOME/rosbags/ruiwo_motor/
# - 文件名格式：ruiwo_motor_dexhand_YYYYMMDD_HHMMSS.bag
# 

# 设置变量
FILENAME="ruiwo_motor_dexhand_$(date +%Y%m%d_%H%M%S).bag"

# 创建输出目录（如果不存在）
mkdir -p $OUTPUT_DIR

# 输出信息
echo "机械臂与灵巧手ROS话题录制开始"
echo "录制话题："
echo "  - /ruiwo_motor/command (机械臂控制命令)"
echo "  - /ruiwo_motor/state (机械臂状态反馈)"
echo "  - /dexhand/command (灵巧手控制命令)"
echo "  - /dexhand/state (灵巧手状态反馈)"
echo "输出文件: $FILENAME"
echo "按Ctrl+C停止录制"
echo "=================================="

# 开始录制指定话题
rosbag record -O "$FILENAME" /ruiwo_motor/command /ruiwo_motor/state /dexhand/command /dexhand/state

echo "录制完成。Bag文件保存到: $FILENAME"
