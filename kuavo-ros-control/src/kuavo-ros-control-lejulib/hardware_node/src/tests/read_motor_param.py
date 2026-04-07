#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from kuavo_msgs.srv import getMotorParam

def read_motor_param():
    rospy.init_node('read_motor_param_node')
    
    try:
        # Wait for service to become available
        rospy.wait_for_service('hardware/get_motor_param')
        
        # Create service proxy
        get_motor_param = rospy.ServiceProxy('hardware/get_motor_param', getMotorParam)
        
        # Call service
        response = get_motor_param()
        
        if response.success:
            print("Successfully got motor parameters:")
            for param in response.data:
                print(f"Motor ID: {param.id}, Kp: {param.Kp}, Kd: {param.Kd}")
        else:
            print(f"Failed to get motor parameters: {response.message}")
            
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        read_motor_param()
    except rospy.ROSInterruptException:
        pass
