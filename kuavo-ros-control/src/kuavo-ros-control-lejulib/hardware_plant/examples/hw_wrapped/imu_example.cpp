// Copyright 2025 Lejurobot. All rights reserved.
//
// IMU 数据读取示例：循环读取并实时显示 IMU 数据
// - 100Hz 刷新率
// - 按 Ctrl+C 退出

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include "hw_wrapped/hw_wrapped.h"

using namespace leju::hw;

static volatile bool running = true;

void signalHandler(int signum) {
    std::cout << "\n收到中断信号，正在退出..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    std::cout << "=== IMU 数据读取示例 ===" << std::endl;

    // 注册信号处理
    signal(SIGINT, signalHandler);

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

    // 循环读取 IMU 数据
    std::cout << "开始读取 IMU 数据 (按 Ctrl+C 退出)..." << std::endl;
    std::cout << std::endl;

    JointData_t joint_data;
    ImuData_t imu_data;
    while (running) {
        ret = hw.GetSensorsState(joint_data, imu_data);
        if (ret == HwErrorType::Success) {
            // 清屏并打印 IMU 数据
            std::cout << "\033[2J\033[H";  // 清屏
            std::cout << "=== IMU 数据 ===" << std::endl;
            std::cout << imu_data << std::endl;
            std::cout << "按 Ctrl+C 退出" << std::endl;
        } else if (ret == HwErrorType::ImuUnavailable) {
            std::cerr << "IMU 不可用" << std::endl;
        } else {
            std::cerr << "读取 IMU 数据失败, 错误码: " << static_cast<int>(ret) << std::endl;
        }

        // 100Hz 读取频率
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 关闭硬件接口
    std::cout << "正在关闭硬件..." << std::endl;
    hw.Shutdown();
    std::cout << "硬件已关闭" << std::endl;

    return 0;
}
