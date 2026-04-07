#include <ros/ros.h>
#include "hardware_node.h"
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <atomic>

std::atomic<bool> running(true);
bool ready = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_test");
    ros::NodeHandle nh;
    
    // 设置信号处理函数
    signal(SIGINT, [](int sig) {
        std::cout << "\nCTRL+C received, exiting..." << std::endl;
        running = false;
    });
    
    HighlyDynamic::HardwareNode *hardware_plant_ptr = new HighlyDynamic::HardwareNode(nh);

    /* Hardware initialization */
    hardware_plant_ptr->HWPlantInit();
    std::thread spin_thread([]()
                            { ros::spin(); });
    spin_thread.detach();

    /* ROS PARAM */
    // Set the build_cppad_state parameter to 2
    // This indicates that cppad has completed building
    nh.setParam("build_cppad_state", 2);
    // Set the /initial_state parameter
    std::vector<double> initial_state = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0101207, 1.12683e-05, 0.634255, 0, 0.0523599, 0,
        // leg
        -0.644679, -0.133286, -0.429381, 0.796557, -0.350946, 0.0222689, 
        0.644702, 0.133237, 0.429419, 0.796559, -0.350953, -0.0222094
    };
    // Set the /squat_initial_state parameter
    std::vector<double> squat_initial_state = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0318145, 1.20197e-05, 0.493235, 0, 0.0523599, 0,
        // leg
        -1.37486, -0.608303, -0.733005, 2.01772, -0.762585, 0.0352346,
        1.37489, 0.608285, 0.733089, 2.01772, -0.762585, -0.0351503
    };

    nh.setParam("/squat_initial_state", squat_initial_state);
    nh.setParam("/initial_state", initial_state);
    // Print confirmation message that all parameters have been set
    std::cout << "\033[33m\nAll parameters have been successfully set:" << std::endl;
    std::cout << "  - build_cppad_state: 2" << std::endl;
    std::cout << "  - /initial_state: [";
    for (size_t i = 0; i < initial_state.size(); ++i) {
        std::cout << initial_state[i];
        if (i < initial_state.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "  - /squat_initial_state: [";
    for (size_t i = 0; i < squat_initial_state.size(); ++i) {
        std::cout << squat_initial_state[i];
        if (i < squat_initial_state.size() - 1) std::cout << ", ";
    }
    std::cout << "]\033[0m" << std::endl;
    /*****************************************************************/

    hardware_plant_ptr->real_init_wait();

    std::vector<double> ready_inital_pos(hardware_plant_ptr->get_num_actuated(), 0); // 进入反馈的初始位置

    hardware_plant_ptr->jointMoveTo(ready_inital_pos, 60.0, 0.002);

    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << " Node is ready for test script execution." << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;

    // 等待用户输入 'x' 退出或 CTRL+C 中断
    std::cout << "Press 'x' to exit or CTRL+C to interrupt..." << std::endl;
    bool exit_requested = false;
    
    while (running && !exit_requested)
    {
        if (kbhit())
        {
            char input = getchar();
            if (input == 'x')
            {
                std::cout << "Exit command received" << std::endl;
                exit_requested = true;
            }
        }
        usleep(50000);
    }

    std::cout << "Finished sending commands" << std::endl;
    if (hardware_plant_ptr && !hardware_plant_ptr->is_deinitialized_) {
        hardware_plant_ptr->HWPlantDeInit();
    }
    running = false;

    return 0;
}
