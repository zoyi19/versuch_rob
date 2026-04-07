import rospy
import signal
import sys
import os
import time
from ocs2_msgs.msg import mpc_observation

msg_arrived = False
timeout_seconds = 60
interval_seconds = 0.5

def obs_callback(msg: mpc_observation):
    global msg_arrived
    msg_arrived = True

if __name__ == "__main__":

    rospy.init_node("check_robot_launch")
    rospy.loginfo("check_robot_launch node started")
    obs_sub = rospy.Subscriber("/humanoid_mpc_observation", mpc_observation, obs_callback)
    def timeout_handler(signum, frame):
        print(f"机器人启动失败，未在{timeout_seconds}秒内收到 mpc_observation 消息")
        os._exit(1)

    # Set 60 second timeout
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(timeout_seconds)

    try:
        # Print ROS environment variables
        print("--------------------------------------------------------------------------------------------------------")
        print("Check Robot Launch")
        print("--------------------------------------------------------------------------------------------------------")
        print("ROS Environment Variables:")
        print(f"ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI', 'Not set')}")
        print(f"ROS_HOSTNAME: {os.environ.get('ROS_HOSTNAME', 'Not set')}")
        print(f"ROS_IP: {os.environ.get('ROS_IP', 'Not set')}")
        print(f"每间隔{interval_seconds}秒检查一次，如果{timeout_seconds}秒内没有收到 mpc_observation 消息，则认为机器人启动失败")
        print("--------------------------------------------------------------------------------------------------------")

        # Wait for /build_cppad_state to be 2
        while not msg_arrived:
            try:
                rospy.loginfo("Waiting for mpc_observation message...")
                time.sleep(interval_seconds)
            except rospy.ROSInterruptException:
                rospy.loginfo("ROSInterruptException occurred")
                break
        rospy.loginfo("收到 mpc_observation 消息，机器人启动成功")
        sys.exit(0)
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
        sys.exit(1)

