# 仿真启动
cd /home/lab/ziyi/versuch_rob/kuavo-ros-control
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
（rviz）
cd /home/lab/ziyi/versuch_rob/kuavo-ros-control
source devel/setup.bash
rosrun rviz rviz -d $(rospack find humanoid_wheel_interface_ros)/rviz/mobile_manipulator.rviz
（验证）
rosrun tf tf_echo base_link zarm_l7_link
rosrun tf tf_echo base_link zhead_2_link

开启umi数据采集
夹爪检测：
cd /home/lab/ziyi/versuch_rob/gripper_calibration
python3 gripper_percent_node.py --calibration gripper_range.json

遥操
cd /home/lab/ziyi/versuch_rob/kuavo-ros-control
source devel/setup.bash
cd src/demo/umi_replay/scripts

# 不使用夹爪：
python3 umi_realtime_teleop.py --no-gripper rate 50 fhan-r 1.0 fhan-h0-scale 5.0 delta-scale 0.5 max-delta 0.35

# 使用夹爪：
python3 umi_realtime_teleop.py \
    --head-frame zhead_2_link \
    --delta-scale 0.5 \
    --rate 20 \
    --fhan-r 2.0 \
    --fhan-h0-scale 5.0 \
    --max-delta 0.35
