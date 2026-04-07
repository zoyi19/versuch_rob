// Copyright 2025 Lejurobot. All rights reserved.

#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <string>
#include <atomic>
#include <ostream>
#include "hw_wrapped/hw_types.h"

namespace HighlyDynamic {
class HardwarePlant;
struct HardwareParam;
struct HardwareSettings;
}

namespace leju {
namespace hw {

// 硬件接口类，提供与底层硬件的统一通信接口。
//
// 该类使用单例模式管理机器人硬件的初始化、数据读取和控制命令发送，
// 包括电机控制、IMU 数据采集等功能。
class HardwareInterface {
public:
    HardwareInterface(const HardwareInterface&) = delete;
    HardwareInterface& operator=(const HardwareInterface&) = delete;

    // 获取硬件接口单例实例。
    // @return HardwareInterface 单例引用
    static HardwareInterface& GetInstance();

    // 初始化硬件接口。
    //
    // 该方法初始化底层硬件通信接口，包括电机控制器、IMU 等硬件设备。
    // 必须在使用其他功能前调用。
    //
    // @return HwErrorType 初始化结果状态码
    //         - Success: 初始化成功
    //         - NotInitialized: 未初始化
    //         - ImuInitFailed: IMU 初始化失败
    //         - ImuUnavailable: IMU 不可用
    //         - ConfigMissing: 配置文件缺失
    //         - Failed: 初始化失败
    HwErrorType Initialize();

    // 关闭硬件接口。
    //
    // 释放硬件资源，断开与底层硬件的连接，清理内部状态并安全关闭所有硬件设备。
    void Shutdown();

    // 获取电机数量。
    // @return 电机总数
    uint32_t GetMotorNumber() const;

    // 设置关节控制命令。
    //
    // 向所有关节发送位置、速度和力矩控制命令，
    // 命令将被发送到底层电机控制器执行。
    //
    // @param joint_command 关节命令结构体，包含所有关节的目标位置、速度、力矩等信息
    // @return HwErrorType 设置结果状态码
    //         - Success: 命令设置成功
    //         - NotInitialized: 接口未初始化
    //         - DimensionMismatch: 数据维度不匹配
    //         - Failed: 命令发送失败
    HwErrorType SetJointCommand(const JointCommand_t &joint_command);

    // 获取传感器状态数据。
    //
    // 读取所有关节和 IMU 的当前状态信息，
    // 包括关节位置、速度、力矩，以及 IMU 加速度、角速度、姿态四元数等。
    //
    // @param joint_data 用于存储关节状态数据的结构体引用
    // @param imu_data 用于存储 IMU 数据的结构体引用
    // @return HwErrorType 读取结果状态码
    //         - Success: 读取成功
    //         - NotInitialized: 接口未初始化
    //         - ReadSensorFailed: 传感器读取失败
    //         - Failed: 读取失败
    HwErrorType GetSensorsState(JointData_t &joint_data, ImuData_t &imu_data);

    // 将所有关节以指定速度移动到目标位置。
    //
    // @param goal_pos 所有关节的目标位置（弧度），向量大小必须与机器人关节数匹配
    // @param speed 移动速度（度/秒）
    // @param dt 轨迹插值时间步长（秒），默认值: 1e-3
    // @param current_limit 移动过程中所有关节的最大电流限制，默认值: -1（无电流限制）
    // @return HwErrorType 命令结果状态码
    //         - Success: 移动命令执行成功
    //         - NotInitialized: 硬件接口未初始化
    HwErrorType JointMoveTo(const std::vector<double> &goal_pos, double speed, double dt = 1e-3, double current_limit=-1);

private:
    // 私有构造函数，实现单例模式
    HardwareInterface() = default;

    // 私有析构函数
    ~HardwareInterface() = default;

    // 初始化状态标志，true 表示已初始化
    std::atomic<bool> initialized_{false};

    // 底层硬件控制对象指针
    HighlyDynamic::HardwarePlant* hardware_plant_ = nullptr;
    HighlyDynamic::HardwareSettings* hardware_settings_ = nullptr;
};

}  // namespace hw
}  // namespace leju

std::ostream& operator<<(std::ostream& os, const leju::hw::JointData_t& joint_data);
std::ostream& operator<<(std::ostream& os, const leju::hw::ImuData_t& imu_data);
std::ostream& operator<<(std::ostream& os, const leju::hw::JointCommand_t& joint_command);
