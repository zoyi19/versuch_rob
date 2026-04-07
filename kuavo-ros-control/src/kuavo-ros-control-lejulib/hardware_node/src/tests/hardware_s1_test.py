#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import jointCmd
import time
import signal
import sys

class HardwareS1Test:
    def __init__(self):
        rospy.init_node('hardware_s1_test', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_cmd', jointCmd, queue_size=10)
        self.joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # ===== CONFIGURABLE PARAMETERS =====
        # Control rate
        self.rate = rospy.Rate(500)  # 500Hz
        
        # Speed multiplier to adjust movement speed
        self.speed_multiplier = 2.0  # Default to double speed
        
        # Control flags for arms and legs
        self.control_legs = True
        self.control_arms = True
        
        # Base frequencies
        self.base_leg_frequency = 0.4  # 0.4Hz for active leg joints
        self.base_arm_frequency = 0.3  # 0.3Hz for arm joints
        
        # Define joint limits for the 12 leg joints
        self.joint_limits = [
            (-0.50, 0.50),  # leg_l1
            (-0.10, 0.10),  # leg_l2
            (-0.20, 0.20),  # leg_l3
            (0.0, 0.8),     # leg_l4
            (0.0, 0.0),     # leg_l5
            (0.0, 0.0),     # leg_l6
            (0.50, -0.50),  # leg_r1
            (0.10, -0.10),  # leg_r2
            (-0.20, 0.20),  # leg_r3
            (0.0, 0.8),     # leg_r4
            (0.0, 0.0),     # leg_r5
            (0.0, 0.0),     # leg_r6
        ]
        
        # Define arm joint limits
        self.arm_joint_limits = [
            (-0.8, 0.8),    # zarm_l1
            (0.1, 0.2),     # zarm_l2
            (-0.1, 0.1),    # zarm_l3
            (0.0, -0.9),    # zarm_l4
            (-0.8, 0.8),    # zarm_r1
            (-0.2, -0.1),   # zarm_r2
            (-0.1, 0.1),    # zarm_r3
            (0.0, -0.9),    # zarm_r4
        ]
        
        # Phase offsets for leg joints to create walking-like motion
        # Left and right legs have π (180°) phase difference for alternating movement
        self.phase_offsets = [
            0.0, np.pi/3, np.pi/2, 2*np.pi/3, 0.0, 0.0,  # Left leg
            np.pi, 4*np.pi/3, 3*np.pi/2, 5*np.pi/3, 0.0, 0.0  # Right leg (opposite phase)
        ]
        
        # Phase offsets for arm joints
        self.arm_phase_offsets = [
            0.0, np.pi/4, np.pi/2, 3*np.pi/4,  # Left arm
            np.pi, 5*np.pi/4, 3*np.pi/2, 7*np.pi/4  # Right arm (opposite phase)
        ]
        # ===== END OF CONFIGURABLE PARAMETERS =====
        
        # Define all 21 joint names including legs, arms, and waist
        self.joint_names = [
            'leg_l1_joint', 'leg_l2_joint', 'leg_l3_joint', 'leg_l4_joint', 'leg_l5_joint', 'leg_l6_joint',
            'leg_r1_joint', 'leg_r2_joint', 'leg_r3_joint', 'leg_r4_joint', 'leg_r5_joint', 'leg_r6_joint',
            'waist_joint',  # Waist joint (13th)
            'zarm_l1_joint', 'zarm_l2_joint', 'zarm_l3_joint', 'zarm_l4_joint',  # Left arm joints
            'zarm_r1_joint', 'zarm_r2_joint', 'zarm_r3_joint', 'zarm_r4_joint'   # Right arm joints
        ]
        
        # Base frequencies for leg joints
        self.frequencies = [self.base_leg_frequency] * 4 + [0.0] * 2 + [self.base_leg_frequency] * 4 + [0.0] * 2
        
        # Base frequencies for arm joints
        self.arm_frequencies = [self.base_arm_frequency] * 4 + [self.base_arm_frequency] * 4
        
        # Apply speed multiplier
        self.update_frequencies()
        
        signal.signal(signal.SIGINT, self.signal_handler)
        self.start_time = time.time()
        self.running = True
    
    def update_frequencies(self):
        """Update frequencies based on speed multiplier"""
        # Update leg frequencies
        for i in range(len(self.frequencies)):
            if self.frequencies[i] > 0:  # Only update non-zero frequencies
                self.frequencies[i] = self.base_leg_frequency * self.speed_multiplier
        
        # Update arm frequencies
        for i in range(len(self.arm_frequencies)):
            if self.arm_frequencies[i] > 0:  # Only update non-zero frequencies
                self.arm_frequencies[i] = self.base_arm_frequency * self.speed_multiplier
    
    def set_speed_multiplier(self, multiplier):
        """Set the speed multiplier and update frequencies"""
        self.speed_multiplier = multiplier
        self.update_frequencies()
        print(f"Speed multiplier set to {multiplier}x")
    
    def set_control_mode(self, control_legs=None, control_arms=None):
        """Set which parts of the robot to control"""
        if control_legs is not None:
            self.control_legs = control_legs
            print(f"Leg control {'enabled' if control_legs else 'disabled'}")
        
        if control_arms is not None:
            self.control_arms = control_arms
            print(f"Arm control {'enabled' if control_arms else 'disabled'}")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C to stop the node gracefully"""
        print("\nStopping joint movement and shutting down...")
        self.running = False
        sys.exit(0)
    
    def publish_zero_positions(self):
        """Publish zero positions for all joints"""
        positions = [0.0] * 21  # Zero for all 21 joints
        self.publish_joint_state(positions)
        rospy.sleep(0.5)  # Give time for the command to be processed
    
    def calculate_joint_positions(self, t):
        """Calculate joint positions using sinusoidal functions"""
        positions = []
        
        # Calculate positions for the 12 leg joints
        for i in range(12):
            if self.control_legs:
                min_val, max_val = self.joint_limits[i]
                mid_val = (max_val + min_val) / 2
                amplitude = (max_val - min_val) / 2
                
                pos = mid_val + amplitude * np.sin(self.frequencies[i] * t + self.phase_offsets[i])
            else:
                pos = 0.0  # Zero position when legs are not controlled
            positions.append(pos)
        
        # Add zero position for waist joint
        positions.append(0.0)
        
        # Calculate positions for the 8 arm joints
        for i in range(8):
            if self.control_arms:
                min_val, max_val = self.arm_joint_limits[i]
                mid_val = (max_val + min_val) / 2
                amplitude = (max_val - min_val) / 2
                
                pos = mid_val + amplitude * np.sin(self.arm_frequencies[i] * t + self.arm_phase_offsets[i])
            else:
                pos = 0.0  # Zero position when arms are not controlled
            positions.append(pos)
        
        return positions
    
    def publish_joint_state(self, positions):
        """Create and publish joint state message"""
        joint_cmd = jointCmd()
        joint_cmd.header.stamp = rospy.Time.now()
        joint_cmd.joint_q = positions
        joint_cmd.joint_v = [0.0] * 21
        joint_cmd.tau = [0.0] * 21
        joint_cmd.tau_max = [2.0] * 21
        joint_cmd.tau_ratio = [0.0] * 21
        joint_cmd.joint_kp = [0.0] * 21
        joint_cmd.joint_kd = [0.0] * 21
        joint_cmd.control_modes = [2] * 21
        
        self.joint_pub.publish(joint_cmd)
        
        # Also publish to joint_states topic
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names if hasattr(self, 'joint_names') else [f"joint_{i}" for i in range(len(positions))]
        joint_state.position = positions
        joint_state.velocity = [0.0] * len(positions)
        joint_state.effort = [0.0] * len(positions)
        self.joint_states_pub.publish(joint_state)
        
    def run(self):
        """Main loop to publish joint positions"""
        print(f"Starting joint movement with speed multiplier {self.speed_multiplier}x. Press Ctrl+C to stop.")
        print(f"Control mode: Legs {'enabled' if self.control_legs else 'disabled'}, Arms {'enabled' if self.control_arms else 'disabled'}")
        
        # Calculate initial positions with phase offsets for leg joints
        initial_positions = []
        for i in range(12):
            if self.control_legs:
                min_val, max_val = self.joint_limits[i]
                mid_val = (max_val + min_val) / 2
                amplitude = (max_val - min_val) / 2
                pos = mid_val + amplitude * np.sin(self.phase_offsets[i])
            else:
                pos = 0.0
            initial_positions.append(pos)
        
        # Add zero position for waist joint
        initial_positions.append(0.0)
        
        # Calculate initial positions for arm joints
        for i in range(8):
            if self.control_arms:
                min_val, max_val = self.arm_joint_limits[i]
                mid_val = (max_val + min_val) / 2
                amplitude = (max_val - min_val) / 2
                pos = mid_val + amplitude * np.sin(self.arm_phase_offsets[i])
            else:
                pos = 0.0
            initial_positions.append(pos)
        
        # Gradually move to initial position
        print("Gradually moving to initial position...")
        steps = 50  # Number of steps for smooth transition
        current_positions = [0.0] * 21  # Start from zero positions for all 21 joints
        
        for step in range(steps + 1):
            # Calculate intermediate positions with easing
            t = step / steps  # Normalized time from 0 to 1
            easing_factor = (1 - np.cos(t * np.pi)) / 2  # Sinusoidal easing
            
            intermediate_positions = [
                current + (target - current) * easing_factor
                for current, target in zip(current_positions, initial_positions)
            ]
            
            self.publish_joint_state(intermediate_positions)
            rospy.sleep(0.05)  # 50ms delay between steps
        
        print("Reached initial position")
        
        # Main control loop
        self.start_time = time.time()
        while not rospy.is_shutdown() and self.running:
            t = time.time() - self.start_time
            positions = self.calculate_joint_positions(t)
            self.publish_joint_state(positions)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = HardwareS1Test()
        controller.run()
    except rospy.ROSInterruptException:
        pass
