// Copyright 2025 Lejurobot. All rights reserved.

#include "hw_wrapped/hw_wrapped.h"
#include "hw_wrapped_log.h"

#include "hardware_plant.h"

#include <cstdlib> // for getenv
#include <functional>
#include <vector>
#include <iostream>

namespace leju {
namespace hw {

HardwareInterface& HardwareInterface::GetInstance() {
    static HardwareInterface instance;
    return instance;
}

HwErrorType HardwareInterface::Initialize() {
    
    // 从环境变量获取配置路径: 这些环境变量通过 source hardware_lib_setup.sh 设置
    const char* kuavo_assets_path = std::getenv("KUAVO_ASSETS_PATH");
    const char* hardware_plant_path = std::getenv("HARDWARE_PLANT_PATH");
    if (kuavo_assets_path == nullptr || hardware_plant_path == nullptr) {
        HWLOG_FAILURE("环境变量 KUAVO_ASSETS_PATH 或 HARDWARE_PLANT_PATH 未设置, 请先执行`source hardware_lib_setup.sh`");
        return HwErrorType::ConfigMissing;
    }

    // 从环境变量读取 ROBOT_VERSION, Kuavo4Pro 请设置为: export ROBOT_VERSION=45
    const char* robot_version_env = std::getenv("ROBOT_VERSION");
    if (robot_version_env == nullptr) {
       HWLOG_FAILURE("环境变量 ROBOT_VERSION 未设置, 请先执行`export ROBOT_VERSION=<version>`");
       return HwErrorType::ConfigMissing;
    }

    // 1. 初始化硬件参数
    HighlyDynamic::HardwareParam hardware_param;
    hardware_param.cali_leg = false;
    hardware_param.cali_arm = false;
    hardware_param.cali     = false;
    hardware_param.teach_pendant_ = false;
    hardware_param.only_half_up_body = false;
    hardware_param.robot_version = RobotVersion::create(std::atoi(robot_version_env));;
    hardware_param.kuavo_assets_path = kuavo_assets_path;
    
    HWLOG_I("Hardware version: %s", hardware_param.robot_version.to_string().c_str());
    
    double dt = 0.001;  // 1ms 控制周期
    hardware_plant_ = new HighlyDynamic::HardwarePlant(dt, hardware_param, hardware_plant_path);

    if (hardware_plant_->HWPlantInit() != 0) {
        return HwErrorType::Failed;
    }

    hardware_settings_ = new HighlyDynamic::HardwareSettings(hardware_plant_->get_motor_info());

    initialized_.store(true);

    JointData_t joint_data;
    ImuData_t imu_data;
    if(this->GetSensorsState(joint_data, imu_data) != HwErrorType::Success) {
        HWLOG_FAILURE("Init, Failed to get sensors state.");
        initialized_.store(false);
        return HwErrorType::Failed;
    }

    return HwErrorType::Success;
}

void HardwareInterface::Shutdown() {
    initialized_.store(false);
    if(hardware_plant_) {
        hardware_plant_->HWPlantDeInit();
        delete hardware_plant_;
        hardware_plant_ = nullptr;
        delete hardware_settings_;
        hardware_settings_ = nullptr;
    }
}

HwErrorType HardwareInterface::SetJointCommand(const JointCommand_t &joint_command) {
    // 检查是否已初始化
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }
    // 检查维度
    if (!joint_command.isValid()) {
        std::cerr << "JointCommand 维度不匹配: "
                  << "q=" << joint_command.q.size()
                  << ", v=" << joint_command.v.size()
                  << ", tau=" << joint_command.tau.size()
                  << ", kp=" << joint_command.kp.size()
                  << ", kd=" << joint_command.kd.size()
                  << ", modes=" << joint_command.modes.size() << std::endl;
        return HwErrorType::DimensionMismatch;
    }

    /////////////// JointSpace 2 MotorSpace /////////////////////
    uint32_t num_joint = joint_command.q.size();
    Eigen::VectorXd cmd_out(num_joint * 5);
    Eigen::VectorXd cmd_input(num_joint * 5);
    std::vector<int> control_modes(num_joint);
    Eigen::VectorXd joint_kp(num_joint);
    Eigen::VectorXd joint_kd(num_joint);

    for (uint32_t i = 0; i < num_joint; ++i) {
        joint_kp(i) =joint_command.kp.at(i);
        joint_kd(i) = joint_command.kd.at(i);
        control_modes[i] = static_cast<int>(joint_command.modes.at(i));

        cmd_input[num_joint * 0 + i] = joint_command.q.at(i);          
        cmd_input[num_joint * 1 + i] = joint_command.v.at(i);     
        cmd_input[num_joint * 2 + i] = joint_command.tau.at(i);       
        cmd_input[num_joint * 3 + i] = hardware_settings_->max_current.at(i);  // tau_max  
        cmd_input[num_joint * 4 + i] = 1;   // tau_ratio
        
    }

    // Use HardwarePlant's command processing
    // 关键步骤: joint space to motor space
    hardware_plant_->cmds2Cmdr(cmd_input, num_joint, cmd_out, num_joint);

    // Handle NaN values
    if (cmd_out.hasNaN()) {
        std::cerr << "Command output has NaN values, setting to zero" << std::endl;
        return HwErrorType::Failed;
    }
    ////////////////////////////////////////////////////////////////

    // Motor Command Write
    hardware_plant_->writeCommand(cmd_out, num_joint, control_modes, joint_kp, joint_kd);

    return HwErrorType::Success;
    #undef CHECK_JOINT_COMMAND_DIMS
}

HwErrorType HardwareInterface::GetSensorsState(JointData_t &joint_data, ImuData_t &imu_data) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    SensorData_t sensor_data_motor, sensor_data_joint;
    sensor_data_joint.resizeJoint(hardware_plant_->get_num_actuated());
    // Read current sensor data using parent class method
    if (!hardware_plant_->readSensor(sensor_data_motor)) {
        return HwErrorType::Failed;
    }

    // 关键转换: motor space to joint space
    hardware_plant_->motor2joint(sensor_data_motor, sensor_data_joint);
    hardware_plant_->setState(sensor_data_motor, sensor_data_joint); // 必要的步骤

    // Fill joint_data from sensor_data_joint
    size_t num_joints = sensor_data_joint.joint_q.size();
    joint_data.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        joint_data.q.at(i) = sensor_data_joint.joint_q(i);
        joint_data.v.at(i) = sensor_data_joint.joint_v(i);
        joint_data.vd.at(i) = sensor_data_joint.joint_vd(i);
        joint_data.tau.at(i) = sensor_data_joint.joint_current(i);
    }

    // Convert sensor_data.gyro, sensor_data.acc, sensor_data.quat, sensor_data.free_acc To ImuData_t
    imu_data.reset();

    // Convert angular velocity (rad/s) - from Eigen::Vector3d to std::array<double, 3>
    imu_data.gyro[0] = sensor_data_motor.gyro(0);  // gyro_x
    imu_data.gyro[1] = sensor_data_motor.gyro(1);  // gyro_y
    imu_data.gyro[2] = sensor_data_motor.gyro(2);  // gyro_z

    // Convert acceleration (m/s²) - from Eigen::Vector3d to std::array<double, 3>
    imu_data.acc[0] = sensor_data_motor.acc(0);     // acc_x
    imu_data.acc[1] = sensor_data_motor.acc(1);     // acc_y
    imu_data.acc[2] = sensor_data_motor.acc(2);     // acc_z

    // Convert free acceleration (m/s²) - from Eigen::Vector3d to std::array<double, 3>
    imu_data.free_acc[0] = sensor_data_motor.free_acc(0);  // free_acc_x
    imu_data.free_acc[1] = sensor_data_motor.free_acc(1);  // free_acc_y
    imu_data.free_acc[2] = sensor_data_motor.free_acc(2);  // free_acc_z

    // Convert quaternion [w, x, y, z] - from Eigen::Vector4d to std::array<double, 4>
    imu_data.quat[0] = sensor_data_motor.quat[0];  // quat_w
    imu_data.quat[1] = sensor_data_motor.quat[1];  // quat_x
    imu_data.quat[2] = sensor_data_motor.quat[2];  // quat_y
    imu_data.quat[3] = sensor_data_motor.quat[3];  // quat_z

    return HwErrorType::Success;
}

HwErrorType HardwareInterface::JointMoveTo(const std::vector<double> &goal_pos, 
    double speed, double dt /*= 1e-3*/, 
    double current_limit /*=-1*/) {
    if (!initialized_) {
        return HwErrorType::NotInitialized;
    }

    if(goal_pos.size() != hardware_plant_->get_num_actuated()) {
        return HwErrorType::DimensionMismatch;
    }

    hardware_plant_->jointMoveTo(goal_pos, speed, dt, current_limit);

    return HwErrorType::Success;
}

uint32_t HardwareInterface::GetMotorNumber() const {
    if (!initialized_) {
        return 0; // warning: not initialized
    }
    return hardware_plant_->get_num_actuated();
}

} // namespace hw
} // namespace leju