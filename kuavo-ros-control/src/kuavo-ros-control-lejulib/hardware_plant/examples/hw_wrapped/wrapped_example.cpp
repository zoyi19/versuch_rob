// Copyright 2025 Lejurobot. All rights reserved.
//
// 基础示例：演示 hw_wrapped 库的基本用法
// - 初始化硬件接口
// - 获取电机数量
// - 读取关节状态
// - 关闭硬件接口

#include <iostream>
#include <thread>
#include <chrono>
#include "hw_wrapped/hw_wrapped.h"

using namespace leju::hw;

int main(int argc, char* argv[]) {
    std::cout << "=== hw_wrapped 示例程序 ===" << std::endl;

    // 获取硬件接口单例
    auto& hw = HardwareInterface::GetInstance();

    // 初始化硬件接口
    std::cout << "正在初始化硬件..." << std::endl;
    HwErrorType ret = hw.Initialize();
    if (ret != HwErrorType::Success) {
        std::cerr << "硬件初始化失败, 错误码: " << static_cast<int>(ret) << std::endl;
        return 1;
    }
    std::cout << "硬件初始化成功!" << std::endl;

    // 获取电机数量
    uint32_t motor_num = hw.GetMotorNumber();
    std::cout << "电机数量: " << motor_num << std::endl;

    // 读取传感器状态
    JointData_t joint_data;
    ImuData_t imu_data;
    ret = hw.GetSensorsState(joint_data, imu_data);
    if (ret == HwErrorType::Success) {
        std::cout << "当前关节状态:" << std::endl;
        std::cout << joint_data << std::endl;
    } else {
        std::cerr << "读取传感器状态失败" << std::endl;
    }

    // 示例：移动关节到初始位置（全零位置）
    // std::vector<double> goal_pos(motor_num, 0.0);
    // double speed = 10.0;  // 10 度/秒
    // ret = hw.JointMoveTo(goal_pos, speed);
    // if (ret == HwErrorType::Success) {
    //     std::cout << "关节移动命令已发送" << std::endl;
    // }

    // 关闭硬件接口
    std::cout << "正在关闭硬件..." << std::endl;
    hw.Shutdown();
    std::cout << "硬件已关闭" << std::endl;

    return 0;
}
