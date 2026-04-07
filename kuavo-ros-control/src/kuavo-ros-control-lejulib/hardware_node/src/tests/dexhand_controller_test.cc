
/**
 * @file dexhand_controller_test.cc
 * @brief 灵巧手控制器测试应用
 *
 * 本应用程序为灵巧手控制器提供全面的测试套件，支持多种机器人平台和操作模式：
 * - Kuavo（Revo1）平台（触觉/非触觉版本）
 * - Roban2（Revo2）平台
 * - 支持ROS节点模式和独立运行模式
 *
 * 测试应用演示了基本的手部控制操作，包括：
 * - 位置控制（张开/闭合手势）
 * - 速度控制（单个手指）
 * - 双手协调控制
 * - 状态监控和反馈
 *
 * @使用方法
 * ./dexhand_controller_test --touch|--normal|revo2 [--ros]
 * -- touch : Kuavo一代触觉手
 * -- normal : Kuavo一代非触觉手
 * -- revo2 : Roban二代非触觉手
 * -- ros : 启用ROS模式（可选）
 */

#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <csignal>

#include "revo2_dexhand_ros.h"
#include "revo2_hand_controller.h"
#include "touch_dexhand_ros1.h"
#include "touch_hand_controller.h"
#include "kuavo_assets/include/package_path.h"
#include "kuavo_common/common/common.h"
#include "kuavo_common/kuavo_common.h"

using namespace dexhand;
bool running = true;

void signal_handler(int signal) {
    // Handle the signal
    running = false; // Example action
}

int kuavo_normal_main(int argc, char** argv, bool touch_mode) {    
    using namespace eef_controller;
    
    // 定义手势位置常量
    const UnsignedDualHandsArray kDualHandClosePositions = {kCloseFingerPositions, kCloseFingerPositions};
    const UnsignedDualHandsArray kDualHandOpenPositions = {kOpenFingerPositions, kOpenFingerPositions};
    
    std::string asserts_path = ocs2::kuavo_assets::getPath();
    std::string config_path = asserts_path + "/config/gesture/preset_gestures.json";

    // : is can protocol
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
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[33mDual Hand Position --> [Close]\033[0m \n";
        hand_controller->send_position(kDualHandClosePositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Open both hands
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[33mDual Hand Position --> [Open]\033[0m \n";
        hand_controller->send_right_position(kOpenFingerPositions);
        hand_controller->send_left_position(kOpenFingerPositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[32mLeft Hand Speed --> [Close]\033[0m \n";
        hand_controller->send_left_speed(close_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[32mLeft Hand Speed --> [Open]\033[0m \n";
        hand_controller->send_left_speed(open_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[32mRight Hand Speed --> [Close]\033[0m \n";
        hand_controller->send_right_speed(close_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // cppcheck-suppress oppositeInnerCondition
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

int roban2_normal_main(int argc, char** argv) {    
    using namespace eef_controller;
    
    // 定义手势位置常量
    const UnsignedDualHandsArray kDualHandClosePositions = {kCloseFingerPositions, kCloseFingerPositions};
    const UnsignedDualHandsArray kDualHandOpenPositions = {kOpenFingerPositions, kOpenFingerPositions};
    
    std::string asserts_path = ocs2::kuavo_assets::getPath();
    std::string config_path = asserts_path + "/config/gesture/preset_gestures.json";

    // : is can protocol
    const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
    RobotVersion rb_version_ = RobotVersion(4, 0);
    auto kuavo_common_ptr = HighlyDynamic::KuavoCommon::getInstancePtr(rb_version_, kuavo_assets_path);
    auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
    HighlyDynamic::HandProtocolType hand_protocol_type = kuavo_settings.hardware_settings.getHandProtocolType();
    bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;

    auto hand_controller = Revo2HandController::Create(config_path, is_can_protocol);
    if(!hand_controller->init()) {
        std::cout << "[ERROR] Failed to init touch dexhand controller.\n";
        return -1;
    }

    signal(SIGINT, signal_handler);

    std::thread status_thread([&]() {
        while (running) {
            auto finger_status = hand_controller->get_finger_status();
            std::cout <<"[Left Hand]:" << finger_status[0] << std::endl;
            std::cout <<"[Right Hand]:" << finger_status[1] << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        std::cout << "status thread exit \n";
    });

    FingerArray close_speed = {0, 30, 30, 30, 30, 30};
    FingerArray open_speed = {0, -30, -30, -30, -30, -30};
    while (running) {
        // Close both hands
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[33mDual Hand Position --> [Close]\033[0m \n";
        hand_controller->send_position(kDualHandClosePositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Open both hands
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[33mDual Hand Position --> [Open]\033[0m \n";
        hand_controller->send_right_position(kOpenFingerPositions);
        hand_controller->send_left_position(kOpenFingerPositions);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[32mLeft Hand Speed --> [Close]\033[0m \n";
        hand_controller->send_left_speed(close_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[32mLeft Hand Speed --> [Open]\033[0m \n";
        hand_controller->send_left_speed(open_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // cppcheck-suppress oppositeInnerCondition
        if (!running) break;
        std::cout << "\033[32mRight Hand Speed --> [Close]\033[0m \n";
        hand_controller->send_right_speed(close_speed);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // cppcheck-suppress oppositeInnerCondition
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


int kuavo_ros1_main(int argc, char** argv, bool touch_mode) {
    ros::init(argc, argv, "dexhand_controller_test_node");
    ros::NodeHandle nh;
    using namespace eef_controller;

    DexhandRosNode node;
    
    std::string asserts_path = ocs2::kuavo_assets::getPath();
    std::string config_path = asserts_path + "/config/gesture/preset_gestures.json";

    std::unique_ptr<DexhandController> controller = nullptr;
    const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
    RobotVersion rb_version_ = RobotVersion(4, 0);
    auto kuavo_common_ptr = HighlyDynamic::KuavoCommon::getInstancePtr(rb_version_, kuavo_assets_path);
    auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
    HighlyDynamic::HandProtocolType hand_protocol_type = kuavo_settings.hardware_settings.getHandProtocolType();
    bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;
    if (!node.init(nh, config_path, 200, controller, touch_mode, is_can_protocol)) {
        std::cout << "[ERROR] Failed to init dexhand node.\n";
        return -1;
    }

    std::cout << "\n\n----- Kuavo Revo1 Dexhand ROS Node ----- \n";
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

int roban2_ros1_main(int argc, char** argv) {
    ros::init(argc, argv, "dexhand_controller_test_node");
    ros::NodeHandle nh;
    using namespace eef_controller;

    Revo2DexhandRosNode node;
    
    std::string asserts_path = ocs2::kuavo_assets::getPath();
    std::string config_path = asserts_path + "/config/gesture/preset_gestures.json";

    std::unique_ptr<Revo2HandController> controller = nullptr;
    const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
    RobotVersion rb_version_ = RobotVersion(4, 0);
    auto kuavo_common_ptr = HighlyDynamic::KuavoCommon::getInstancePtr(rb_version_, kuavo_assets_path);
    auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
    HighlyDynamic::HandProtocolType hand_protocol_type = kuavo_settings.hardware_settings.getHandProtocolType();
    bool is_can_protocol = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;
    if (!node.init(nh, config_path, 200, controller, is_can_protocol)) {
        std::cout << "[ERROR] Failed to init dexhand node.\n";
        return -1;
    }

    std::cout << "\n\n----- Roban2 Revo2 Dexhand ROS Node ----- \n";
    std::cout << "Topics:\n";
    std::cout << " - /control_robot_hand_position \n";
    std::cout << " - /dexhand/command \n";
    std::cout << " - /dexhand/right/command \n";
    std::cout << " - /dexhand/left/command \n";
    std::cout << " - /dexhand/state \n";
    std::cout << "Services:\n";
    std::cout << "/gesture/execute \n";
    std::cout << "/gesture/execute_state \n";
    std::cout << "/gesture/list \n";
    ros::spin();
    
    return 0;
}

void print_usage(const std::string& argv0) {
    std::cout << "\033[32m使用方法: " << argv0 << " --touch|--normal|revo2 [--ros]\033[0m" << std::endl;
    std::cout << "-- touch : kuavo 一代触觉手\n";
    std::cout << "-- normal : kuavo 一代非触觉手\n";
    std::cout << "-- revo2 : roban 二代非触觉手\n";
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
    if (mode != "--touch" && mode != "--normal" && mode != "--revo2") {
        std::cout << "第一个参数必须是 --touch、--normal 或 --revo2" << std::endl;
        print_usage(argv[0]);
        return -1;
    }

    bool touch_mode = (mode == "--touch");
    bool revo2_mode = (mode == "--revo2");
    bool ros_mode = false;

    if (argc > 2 && std::string(argv[2]) == "--ros") {
        ros_mode = true;
    }

    if (ros_mode) {
        if(revo2_mode) {
            printf("\033[33mroban2 ros1 main\033[0m\n");
            return roban2_ros1_main(argc, argv);
        }else {
            printf("\033[33mkuavo ros1 main\033[0m\n");
            return kuavo_ros1_main(argc, argv, touch_mode);
        }
    } else {
        std::cout << "已选择非ROS模式" << std::endl;
        if(revo2_mode) {
            return roban2_normal_main(argc, argv);
        }
        else {
            printf("\033[33mkuavo normal main\033[0m\n");
            return kuavo_normal_main(argc, argv, touch_mode);
        }
    }

    return 0;
}
