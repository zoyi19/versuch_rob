#!/usr/bin/env python3
"""
灵巧手波浪运动跟踪测试脚本

该脚本用于测试灵巧手的波浪运动跟踪控制，通过控制手指的渐进式位置变化
来实现波浪运动效果。测试包含正向和反向波浪运动，用于验证灵巧手的
位置控制精度和响应性能。

使用方法：
1. 先启动灵巧手控制器测试节点（如 dexhand_ctrl_test --revo2 --ros）
2. 然后运行此脚本:
    source devel/setup.bash
    python3 dexhand_tracing_test.py
"""

import rospy
from kuavo_msgs.msg import dexhandCommand
import signal
import sys

def signal_handler(sig, frame):
    print("\nCtrl+C detected, exiting...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('dexhand_tracing_test')

    # 创建双手控制发布器
    dual_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)

    # Wait for publishers to be ready
    rospy.sleep(1)

    # 创建控制命令消息
    cmd = dexhandCommand()
    cmd.control_mode = dexhandCommand.POSITION_CONTROL

    # 波浪运动位置序列（每步6个手指位置）
    # 位置值范围：0-100，分别对应6个手指，从0开始表示完全张开
    positions_list = [
        [0, 0, 0, 0, 0, 0],        # 初始位置：完全张开
        [20, 0, 100, 0, 0, 0],     # 手指1弯曲到20%
        [30, 0, 100, 100, 0, 0],    # 手指2弯曲到30%，手指3弯曲到100%
        [40, 0, 100, 100, 100, 0],   # 手指1弯曲到40%，手指4弯曲到100%
        [50, 0, 100, 100, 100, 100], # 手指1弯曲到50%，其他手指完全闭合
    ]
    num_steps = len(positions_list)
    delay_sec = 1.0

    print("\033[36m[dexhand_tracing_test] INFO: 开始灵巧手波浪运动跟踪测试\033[0m")

    # 持续执行波浪运动（不停止）
    j = 1
    while not rospy.is_shutdown():
        print(f"\033[36m[dexhand_tracing_test] INFO: 执行第 {j} 轮波浪运动\033[0m")
        j += 1

        # 正向波浪运动
        for i in range(num_steps):
            # 设置双手位置（12个值：左6个+右6个）
            cmd.data = positions_list[i] * 2  # 复制为双手数据

            print(f"正向波浪步进 {i+1}/{num_steps} - 双手位置: {cmd.data}")
            dual_pub.publish(cmd)
            rospy.sleep(delay_sec)

        # 反向波浪运动
        for i in range(num_steps-1, -1, -1):
            cmd.data = positions_list[i] * 2  # 复制为双手数据

            print(f"反向波浪步进 {num_steps-i}/{num_steps} - 双手位置: {cmd.data}")
            dual_pub.publish(cmd)
            rospy.sleep(delay_sec)

    print("\033[36m[dexhand_tracing_test] INFO: 灵巧手波浪运动跟踪测试完成\033[0m")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass