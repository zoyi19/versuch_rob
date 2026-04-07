#!/usr/bin/env python3
"""
Kuavo手臂电机正弦运动跟踪测试脚本

该脚本用于测试Kuavo机器人机械臂的正弦运动跟踪控制，通过控制各个关节的
渐进式正弦变化来实现流畅的机械臂运动效果。测试包含左右臂和头部的
协调运动，用于验证机械臂的位置控制精度和响应性能。

使用方法：
1. 先启动瑞沃电机控制器测试节点：
   ./devel/lib/hardware_node/ruiwo_motor_ros_cxx_test _robot_type:=kuavo

2. 然后运行此脚本：
   source devel/setup.bash
   python3 kuavo_arm_tracing_test.py
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
import time
import signal
import sys

class ArmRuiwoMotorTest:
    def __init__(self):
        rospy.init_node('ruiwo_motor_arm_test', anonymous=True)
        self.joint_pub = rospy.Publisher('/ruiwo_motor/command', JointState, queue_size=10)
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # ===== CONFIGURABLE PARAMETERS =====
        self.enable_pub_joint_state = False
        self.rate = rospy.Rate(200)  # 200Hz
        self.joint_names = [
            'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint', 'zarm_l5_joint', 'zarm_l6_joint', 'zarm_l7_joint',  # Left arm joints
            'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint', 'zarm_r5_joint', 'zarm_r6_joint', 'zarm_r7_joint',   # Right arm joints
            'zhead_1_joint', 'zhead_2_joint'  # Head joints
        ]
        # Define arm and head joint limits
        self.arm_head_joint_limits = [
            # Left arm joints
            (10, 90),    # zarm_l2
            (-45, 45),  # zarm_l3
            (-45,0),    # zarm_l4
            (-45, 45),  # zarm_l5
            (-20, 20),  # zarm_l6
            (-45, 30),  # zarm_l7
            # Right arm joints
            (-90, -10),       # zarm_r2
            (-45, 45),      # zarm_r3
            (-45, 0),        # zarm_r4
            (-45, 45),    # zarm_r5
            (-20, 20),    # zarm_r6
            (-30, 45),    # zarm_r7
            # Head joints
            (-25, 25),    # zhead_1 (±25 degrees)
            (-25, 25),    # zhead_2 (±25 degrees)
        ]
        # Speed multiplier to adjust movement speed
        self.speed_multiplier = 2.0  # Default to double speed
        self.arm_head_frequencies = [0.3*self.speed_multiplier] * len(self.joint_names) # Base frequencies for arm and head joints 0.3Hz

        # Phase offsets for arm and head joints
        self.arm_head_phase_offsets = [
            # Left arm joints
            0.0, np.pi/6, np.pi/3, np.pi/2, 2*np.pi/3, 5*np.pi/6,
            # Right arm joints (opposite phase)
            np.pi, 7*np.pi/6, 4*np.pi/3, 3*np.pi/2, 5*np.pi/3, 11*np.pi/6,
            # Head joints
            0.0,            # zhead_1
            np.pi/2,        # zhead_2 (quarter cycle offset)
        ]
        # ===== END OF CONFIGURABLE PARAMETERS =====
        
        signal.signal(signal.SIGINT, self.signal_handler)
        self.start_time = time.time()
        self.running = True
            
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C to stop the node gracefully"""
        print("\nStopping joint movement and shutting down...")
        self.running = False
        self.move_to_zero_position()
        sys.exit(0)
    
    def move_to_zero_position(self):
        """Gradually move all joints to zero position"""
        print("Moving to zero position...")
        # Get current positions
        current_positions = self.calculate_joint_positions(time.time() - self.start_time)
        target_positions = [0.0] * len(self.joint_names)
        
        # Gradually move to zero position
        steps = 400  # Number of steps for smooth transition
        
        for step in range(steps + 1):
            # Calculate intermediate positions with easing
            t = step / steps  # Normalized time from 0 to 1
            easing_factor = (1 - np.cos(t * np.pi)) / 2  # Sinusoidal easing
            
            intermediate_positions = [
                current + (target - current) * easing_factor
                for current, target in zip(current_positions, target_positions)
            ]
            
            self.publish_joint_command(intermediate_positions)
            self.rate.sleep()  # Use the defined rate instead of fixed delay
        
        print("Reached zero position")
    
    def move_to_start_position(self):
        """Gradually move all joints to the starting position of the sinusoidal movement"""
        print("Moving to sinusoidal starting position...")
        # Get current positions
        current_positions = [0.0] * len(self.joint_names)  # Assume we're at zero
        
        # Calculate the starting positions (t=0 in the sinusoidal movement)
        start_positions = self.calculate_joint_positions(0)
        
        # Gradually move to start position
        steps = 400  # Number of steps for smooth transition
        
        for step in range(steps + 1):
            # Calculate intermediate positions with easing
            t = step / steps  # Normalized time from 0 to 1
            easing_factor = (1 - np.cos(t * np.pi)) / 2  # Sinusoidal easing
            
            intermediate_positions = [
                current + (target - current) * easing_factor
                for current, target in zip(current_positions, start_positions)
            ]
            
            self.publish_joint_command(intermediate_positions)
            self.rate.sleep()  # Use the defined rate instead of fixed delay
        
        print("Reached sinusoidal starting position")

    def calculate_joint_positions(self, t):
        """Calculate joint positions using sinusoidal functions"""
        positions = []
        
        # Calculate positions for all joints
        for i in range(len(self.joint_names)):
            min_val, max_val = self.arm_head_joint_limits[i]
            # Convert angle limits from degrees to radians if needed
            min_val = min_val * np.pi / 180.0
            max_val = max_val * np.pi / 180.0
            mid_val = (max_val + min_val) / 2
            amplitude = (max_val - min_val) / 2
            
            pos = mid_val + amplitude * np.sin(self.arm_head_frequencies[i] * t + self.arm_head_phase_offsets[i])
            positions.append(pos)
        
        return positions
    
    def publish_joint_command(self, positions):
        """Create and publish joint state message"""
        # Check if positions length matches joint_names length
        if len(positions) != len(self.joint_names):
            rospy.logwarn(f"Position array length ({len(positions)}) does not match joint names length ({len(self.joint_names)})")
            return
        
        # Also publish to joint_states topic
        joint_cmd = JointState()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.name = self.joint_names
        joint_cmd.position = positions

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()

        other_joint_names = [ 
            'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint',
            'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint',
            'zarm_l1_joint', 'zarm_r1_joint']
        joint_state.name = other_joint_names + self.joint_names
        joint_state.position = [0.0]* len(other_joint_names) + positions

        # radians
        self.joint_pub.publish(joint_cmd)
        if self.enable_pub_joint_state:
            self.joint_states_pub.publish(joint_state)
        
    def run(self):
        """Main loop to publish joint positions"""
        print(f"Starting joint movement with speed multiplier {self.speed_multiplier}x. Press Ctrl+C to stop.")
                
        self.move_to_start_position()
        
        # Main control loop
        self.start_time = time.time()
        while not rospy.is_shutdown() and self.running:
            t = time.time() - self.start_time
            positions = self.calculate_joint_positions(t)
            self.publish_joint_command(positions)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ArmRuiwoMotorTest()
        controller.run()
    except rospy.ROSInterruptException:
        pass
