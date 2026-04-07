// Copyright 2025 Lejurobot. All rights reserved.

#include <iostream>
#include <vector>
#include <cstdlib>
#include "hardware_plant.h"

using namespace HighlyDynamic;

int main(int argc, char* argv[]) {
    // 获取机器人版本
    const char* robot_version_env = std::getenv("ROBOT_VERSION");
    RobotVersion rb_version;
    if (robot_version_env == nullptr) {
        std::cerr << "警告：未设置环境变量 ROBOT_VERSION，使用默认版本 42" << std::endl;
        rb_version = RobotVersion::create(42);
    } else {
        rb_version = RobotVersion::create(std::atoi(robot_version_env));
        std::cout << "使用机器人版本: " << rb_version.to_string() << std::endl;
    }

    // 配置硬件参数
    HardwareParam hardware_param;
    hardware_param.robot_version = rb_version;
    hardware_param.kuavo_assets_path = KUAVO_ASSETS_PATH;

    std::cout << "kuavo_assets_path: " << hardware_param.kuavo_assets_path << std::endl;

    // 初始化硬件
    double dt = 0.001;  // 1ms 控制周期
    std::cout << "准备初始化硬件..." << std::endl;

    auto hardware_plant = std::make_unique<HardwarePlant>(dt, hardware_param, std::string(PROJECT_SOURCE_DIR));
    hardware_plant->HWPlantInit();

    if (hardware_plant == nullptr) {
        std::cerr << "硬件初始化失败" << std::endl;
        return 1;
    }

    std::cout << "硬件初始化成功!" << std::endl;

    // 在这里添加你的控制逻辑
    // ...

    return 0;
}
