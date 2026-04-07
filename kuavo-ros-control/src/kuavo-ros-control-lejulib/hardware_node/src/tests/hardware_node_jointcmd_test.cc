#include <ros/ros.h>
#include "hardware_node.h"
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <signal.h>

#define LB_s60 1

std::unique_ptr<HighlyDynamic::HardwareNode> hardware_node_ptr;

void controlLoop()
{
    if(hardware_node_ptr) {
        hardware_node_ptr->real_init_wait();
    }
}

int main(int argc, char **argv)
{
    double dt = 0.002;
    ros::init(argc, argv, "hardware_test");
    ros::NodeHandle nh;
    hardware_node_ptr = std::make_unique<HighlyDynamic::HardwareNode>(nh, dt);
    hardware_node_ptr->init();

    std::thread control_thread(controlLoop);

    control_thread.detach();

    nh.setParam("build_cppad_state", 2);

#ifdef LB_s60

    std::vector<double> state_vector;
    state_vector.resize(20, 0);
    
    nh.setParam("/initial_state", state_vector);
    nh.setParam("/squat_initial_state", state_vector);
    nh.setParam("/default_joint_pos", state_vector);

    bool move_to_default_pose = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--default-pose" && i + 1 < argc && std::string(argv[i+1]) == "1") {
            move_to_default_pose = true;
            break;
        }
    }
    
    if(move_to_default_pose){

        std::cout << "移动到准备姿态 ..." << std::endl;
        std::vector<double> moving_pos(hardware_node_ptr->get_num_actuated(), 0);
        moving_pos = std::vector<double>(hardware_node_ptr->get_num_actuated(), 0);
        moving_pos[0] = 18.0;
        moving_pos[1] = -10.0;
        moving_pos[2] = -8.9;
        moving_pos[4] = -20;
        moving_pos[11] = -20;
        hardware_node_ptr->jointMoveTo(moving_pos, 20.0);
    }
    
#endif
    ros::spin();

    return 0;
}
