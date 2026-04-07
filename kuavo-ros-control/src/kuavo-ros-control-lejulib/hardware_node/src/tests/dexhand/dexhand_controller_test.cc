
#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <csignal>

#include "touch_hand_controller.h"
#include "touch_dexhand_ros1.h"
#include "kuavo_assets/include/package_path.h"
#include "kuavo_common/common/common.h"
#include "kuavo_common/kuavo_common.h"

using namespace dexhand;
bool running = true;

void signal_handler(int signal) {
    // Handle the signal
    running = false; // Example action
}

int normal_main(int argc, char** argv, bool touch_mode) {    
    using namespace eef_controller;
    std::string asserts_path = ocs2::kuavo_assets::getPath();
    std::string config_path = asserts_path + "/config/gesture/preset_gestures.json";
    
    // 定义手势位置常量
    const UnsignedDualHandsArray kDualHandClosePositions = {kCloseFingerPositions, kCloseFingerPositions};
    const UnsignedDualHandsArray kDualHandOpenPositions = {kOpenFingerPositions, kOpenFingerPositions};
  
    const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
    RobotVersion rb_version_ = RobotVersion(4, 0);
    auto kuavo_common_ptr = HighlyDynamic::KuavoCommon::getInstancePtr(rb_version_, kuavo_assets_path);
    auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
    HighlyDynamic::HandProtocolType hand_protocol_type = kuavo_settings.hardware_settings.getHandProtocolType();
    bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;

    auto hand_controller = DexhandController::Create(config_path, touch_mode, is_can_protocol);
    if(!hand_controller->init()) {
        std::cout << "[ERROR] Failed to init touch dexhand controller.\n";
        return -1;
    }

    signal(SIGINT, signal_handler);

    std::thread status_thread([&]() {
        while (running) {
            auto finger_status = hand_controller->get_finger_status();
            if (touch_mode) {
                auto touch_status = hand_controller->get_touch_status();
            }
            std::cout <<"[Left Hand]:" << finger_status[0] << std::endl;
            std::cout <<"[Right Hand]:" << finger_status[1] << std::endl;
            // std::cout <<  touch_status[1].at(1) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        std::cout << "status thread exit \n";
    });

    FingerArray close_speed = {0, 30, 30, 30, 30, 30};
    FingerArray open_speed = {0, -30, -30, -30, -30, -30};
    while (running) {
        // Close both hands
        if (!running) break;
        std::cout << "\033[33mDual Hand Position --> [Close]\033[0m \n";
        hand_controller->send_position(kDualHandClosePositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Open both hands
        if (!running) break;
        std::cout << "\033[33mDual Hand Position --> [Open]\033[0m \n";
        hand_controller->send_right_position(kOpenFingerPositions);
        hand_controller->send_left_position(kOpenFingerPositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        if (!running) break;
        std::cout << "\033[32mLeft Hand Speed --> [Close]\033[0m \n";
        hand_controller->send_left_speed(close_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        if (!running) break;
        std::cout << "\033[32mLeft Hand Speed --> [Open]\033[0m \n";
        hand_controller->send_left_speed(open_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        if (!running) break;
        std::cout << "\033[32mRight Hand Speed --> [Close]\033[0m \n";
        hand_controller->send_right_speed(close_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        if (!running) break;
        std::cout << "\033[32mRight Hand Speed --> [Open]\033[0m \n";
        hand_controller->send_right_speed(open_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    if(status_thread.joinable()) {
        status_thread.join();
    }

    return 0;
}

int ros1_main(int argc, char** argv, bool touch_mode) {
    ros::init(argc, argv, "dexhand_controller_test_node");
    ros::NodeHandle nh;
    using namespace eef_controller;

    DexhandRosNode node;
    
    std::string asserts_path = ocs2::kuavo_assets::getPath();
    std::string config_path = asserts_path + "/config/gesture/preset_gestures.json";

    // 从配置文件读取手部协议类型
    const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
    RobotVersion rb_version_ = RobotVersion(4, 0);
    auto kuavo_common_ptr = HighlyDynamic::KuavoCommon::getInstancePtr(rb_version_, kuavo_assets_path);
    auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
    HighlyDynamic::HandProtocolType hand_protocol_type = kuavo_settings.hardware_settings.getHandProtocolType();
    bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;

    std::unique_ptr<DexhandController> controller = nullptr;
    if (!node.init(nh, config_path, 1000, controller, touch_mode, is_can_protocol)) {
        std::cout << "[ERROR] Failed to init dexhand node.\n";
        return -1;
    }

    std::cout << "\n\n----- Dexhand ROS Node ----- \n";
    std::cout << "Topics:\n";
    std::cout << " - /control_robot_hand_position \n";
    std::cout << " - /dexhand/command \n";
    std::cout << " - /dexhand/right/command \n";
    std::cout << " - /dexhand/left/command \n";
    std::cout << " - /dexhand/state \n";
    if (touch_mode) {
        std::cout << " - /dexhand/touch_state \n";
    }
    std::cout << "Services:\n";
    std::cout << "----- Touch Dexhand ROS Node ----- \n";
    std::cout << "/dexhand/change_force_level \n";
    if (touch_mode) {
        std::cout << "/dexhand/left/enable_touch_sensor \n";
        std::cout << "/dexhand/right/enable_touch_sensor \n";
    }
    std::cout << "/gesture/execute \n";
    std::cout << "/gesture/execute_state \n";
    std::cout << "/gesture/list \n";
    ros::spin();
    
    return 0;
}


void print_usage(std::string argv0) {
    std::cout << "\033[32m使用方法: " << argv0 << " --touch|--normal [--ros]\033[0m" << std::endl;
    std::cout << "例如(基于ROS控制触觉手): " << argv0 << " --touch --ros" << std::endl;
    std::cout << "例如(控制触觉手): " << argv0 << " --touch" << std::endl;
    std::cout << "例如(基于ROS控制非触觉手): " << argv0 << " --normal --ros" << std::endl;
    std::cout << "例如(控制非触觉手): " << argv0 << " --normal" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return -1;
    }

    std::string mode = argv[1];
    if (mode != "--touch" && mode != "--normal") {
        std::cout << "First argument must be --touch or --normal" << std::endl;
        print_usage(argv[0]);
        return -1;
    }

    bool touch_mode = (mode == "--touch");
    bool ros_mode = false;

    if (argc > 2 && std::string(argv[2]) == "--ros") {
        ros_mode = true;
    }

    if (ros_mode) {
        return ros1_main(argc, argv, touch_mode);
    } else {
        return normal_main(argc, argv, touch_mode);
    }

    return 0;
}