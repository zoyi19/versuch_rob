#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import motorParam
from kuavo_msgs.srv import changeMotorParam, changeMotorParamRequest

def change_motor_param():
    rospy.init_node('change_motor_param_client')
    
    # Wait for service to become available
    rospy.wait_for_service('hardware/change_motor_param')
    
    try:
        # Create service proxy
        change_param = rospy.ServiceProxy('hardware/change_motor_param', changeMotorParam)
        
        # Create request
        req = changeMotorParamRequest()
        req.data = []
        req.data.append(motorParam(id=1, Kp=221, Kd=2979))
        print(req)
        # Call service
        response = change_param(req)
        
        if response.success:
            rospy.loginfo("Successfully changed motor parameters: %s", response.message)
        else:
            rospy.logerr("Failed to change motor parameters: %s", response.message)
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        change_motor_param()
    except rospy.ROSInterruptException:
        pass
