#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
import sys
import os
import rospkg

rospack_path = rospkg.RosPack().get_path("hardware_node")
sys.path.append(os.path.join(rospack_path, "lib/ruiwo_controller"))
from ruiwo_actuator import RuiWoActuator

from hardware_node.msg import robotHeadMotionData, horizontalMotionByStep

class HeadControllerNode:
    def __init__(self):
        self.joint_control = RuiWoActuator()
        time.sleep(1)

        self.joint_limit = self.get_joint_limit()

        rospy.init_node("head_controller_node")
        rospy.on_shutdown(self.close_canbus)

        self.head_motion_sub = rospy.Subscriber("/robot_head_motion_data", robotHeadMotionData, self.head_motion_callback)
        self.horizontal_motion_sub = rospy.Subscriber("/horizontal_motion_by_step", horizontalMotionByStep, self.horizontal_motion_by_step)
        rospy.spin()
    
    def get_joint_limit(self):
        # Todo 从urdf中获取关节限制

        joint_limit = {
            9: [-60, 60],
            10: [0, 30]
        }

        return joint_limit
    
    def get_joint_state(self):
        # 获取当前电机值，底层返回值是长度为 10 的列表
        # 单个列表元素包含（电机ID, 电机位置，电机速度，电机力矩，故障码,驱动板温度）
        # 示例: [9, 0.015259021896696368, -0.012210012210012167, -0.0366300366300365, 0, 36]

        raw_data = self.joint_control.get_joint_state()
        if raw_data is None:
            rospy.loginfo("Failed to get joint state")
            return [0, 0]
        
        # 将读取的弧度制数据转换为角度制
        head_joint_9 = raw_data[8][1] * 180 / math.pi
        head_joint_10 = raw_data[9][1] * 180 / math.pi
        return [head_joint_9, head_joint_10]
    
    def is_joint_over_limit(self, joint_id, target_pos):
        # 判断目标位置收否超出限位，如果超出则运动到最接近的位置
        # 最终的结果转换为弧度值

        if joint_id in self.joint_limit:
            if target_pos < self.joint_limit[joint_id][0]:
                target_pos = self.joint_limit[joint_id][0]
            elif target_pos > self.joint_limit[joint_id][1]:
                target_pos = self.joint_limit[joint_id][1]
            
            # 将目标位置转换为弧度值
            rospy.loginfo("joint_id: %d, target_pos: %d", joint_id, target_pos)
            result = target_pos * math.pi / 180
            rospy.loginfo("result: %f", result)
            return result
        else:
            rospy.loginfo("Invalid joint id: %d", joint_id)
            return 0
            
    
    def head_motion_callback(self, msg):
        # 头部对应电机 id 为 9、10，9 之前的电机直接填充 0
        target_pos = msg.target_position
        head_joint_9 = self.is_joint_over_limit(9, target_pos[0])
        head_joint_10 = self.is_joint_over_limit(10, target_pos[1])

        motion_data = [0] * 10
        motion_data[8] = head_joint_9
        motion_data[9] = head_joint_10

        self.joint_control.go_to_target(motion_data)
    
    def horizontal_motion_by_step(self, msg):
        # 水平左右转动，step 为步长，单位为度
        step = msg.step
        
        # 当前角度
        current_joint_state = self.get_joint_state()
        current_joint_9 = current_joint_state[0]
        current_joint_10 = current_joint_state[1]

        next_joint_9 = self.is_joint_over_limit(9, current_joint_9 + step)
        next_joint_10 = self.is_joint_over_limit(10, current_joint_10)

        motion_data = [0] * 10
        motion_data[8] = next_joint_9
        motion_data[9] = next_joint_10

        self.joint_control.go_to_target(motion_data)
        
    def close_canbus(self):
        # 结束程序时关闭电机使能
        rospy.loginfo("Exit program, close canbus....")
        self.joint_control.close()


if __name__ == "__main__":
    head_controller_node = HeadControllerNode()









