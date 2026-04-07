#include <ros/ros.h>
#include <kuavo_msgs/jointCmd.h>
#include <kuavo_msgs/sensorsData.h>
#include <kuavo_common/common/sensor_data.h>
#include "motor_follow_test_core.h"
#include <mutex>
#include <chrono>
#include <thread>
#include <signal.h>
#include <csignal>
#include <fstream>
#include <filesystem>
#include <jsoncpp/json/json.h>
#include <ros/package.h>
#include <time.h>
#include <algorithm>

// 硬件接口相关头文件
#include "kuavo_common/common/common.h"
#include "EcDemoApp.h"  // 包含driver_type声明

#include "kuavo_common/kuavo_common.h"
#include "kuavo_common/common/kuavo_settings.h"

// 添加hardware_plant头文件
#include "hardware_plant.h"


// 硬件相关常量
const int kEcMasterNilId = 254;

// 使用已存在的MotorDriveType枚举
using namespace HighlyDynamic;  // 在kuavo5分支中仍然需要

// 使用hardware_plant中定义的数据结构

class MotorFollowTest {
public:
    // 静态信号处理函数
    static void signalHandler(int signal) {
        std::cout << "\n收到信号 " << signal << "，开始安全退出..." << std::endl;
        
        // 直接调用静态的清理方法
        emergencyStopStatic();
        
        std::cout << "安全退出完成" << std::endl;
        exit(0);
    }
    
    // 静态紧急停止方法
    static void emergencyStopStatic() {
        // 这里可以添加一些全局的清理逻辑
        // 比如停止所有电机、清理资源等
        std::cout << "执行紧急停止..." << std::endl;
        
        // 等待一段时间确保电机停止
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "硬件清理完成" << std::endl;
    }

    MotorFollowTest() : nh_("~") {
        // 读取参数
        nh_.param("robot_version", robot_version_, 13);
        nh_.param("joint_control_config_file", config_file_path_, std::string(""));
        
        // 从文件路径读取机器人参数（参考sim版本）
        loadRobotParameters();
        
        // 如果没有指定配置文件或文件不存在，尝试查找兼容版本
        if (config_file_path_.empty() || !std::filesystem::exists(config_file_path_)) {
            const std::string hardware_node_path = ros::package::getPath("hardware_node");
            const std::string base_path = hardware_node_path + "/src/tests/motorTest";
            
            int compatible_version = findCompatibleVersion(base_path, robot_version_, "control_params");
            if (compatible_version != -1) {
                config_file_path_ = base_path + "/config/joint_control_params_v" + std::to_string(compatible_version) + ".json";
            }
        }
        
        std::cout << "MotorFollowTest 构造函数" << std::endl;
        std::cout << "机器人版本: " << robot_version_ << std::endl;
        std::cout << "配置文件: " << config_file_path_ << std::endl;
        
        // 只创建motorcore对象，不进行初始化
        std::cout << "MotorFollowTest 构造函数完成" << std::endl;
    }
    
    ~MotorFollowTest() {
        motor_core_.stopTest();
        
        // hardware_plant析构时会自动清理硬件资源
        // 不需要手动调用cleanupHardware()
    }
    
    void initialize() {
        std::cout << "开始初始化MotorFollowTest..." << std::endl;
        
        // 设置硬件路径
        const std::string hardware_node_path = ros::package::getPath("hardware_node");
        const std::string hardware_plant_path = std::filesystem::path(hardware_node_path).parent_path().string() + "/hardware_plant";
        ::setHardwarePlantPath(hardware_plant_path);
        hardware_plant_path_ = hardware_plant_path;
        
        // 加载硬件配置
        loadHardwareConfig();
        
        // 获取测试模式
        motor_follow_test::MotorFollowTestCore::TestMode test_mode = getTestModeFromParam();
        
        // 初始化核心类
        motor_follow_test::MotorFollowTestCore::TestConfig config;
        config.robot_version = robot_version_;
        config.num_joint = num_joint_;
        config.num_waist_joints = num_waist_joints_;
        config.num_arm_joints = num_arm_joints_;
        config.num_head_joints = num_head_joints_;
        config.na_foot = na_foot_;
        config.config_file_path = config_file_path_;
        config.dt = 0.002;
        config.test_duration_ms = 10000; // 10秒测试
        config.test_mode = test_mode;
        
        if (!motor_core_.initialize(config)) {
            std::cout << "MotorFollowTestCore 初始化失败" << std::endl;
            return;
        }
        
        // 设置ROS话题
        joint_cmd_pub_ = nh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
        
        // 设置ROS话题发布器
        sensor_data_pub_ = nh_.advertise<kuavo_msgs::sensorsData>("/sensors_data_raw", 10);
        sensor_data_pub_orig_ = nh_.advertise<kuavo_msgs::sensorsData>("/sensors_data_raw_orig", 10);
        
        std::cout << "MotorFollowTest 初始化完成" << std::endl;
    }

    void run() {
        std::cout << "====================== start motor following check =====================" << std::endl;
        
        // 初始化
        initialize();
        
        double dt = 0.002;
        double controlFrequency = 1 / dt;
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        std::cout << "start control loop." << std::endl;

        // 定义关节位置
        int num_joints = num_joint_;
        std::vector<double> cali_pos(num_joints, 0.0);
        
        // 初始化阶段
        std::cout << "size: " << joint_ids_.size() << std::endl;

        std::vector<JointParam_t> jointData(num_joints);
        hardware_plant_->GetMotorData(joint_ids_, jointData);

        // 移动到初始位置（参考hardware_plant.cc的calibrateLoop）
        double cali_tau_limit = 3.0;
        hardware_plant_->jointMoveTo(cali_pos, 60, dt, cali_tau_limit);

        // 初始化绝对时间基准
        clock_gettime(CLOCK_MONOTONIC, &next_time);

        // 添加3秒延时
        next_time.tv_sec += 3;

        // 等待到指定时间
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        std::cout << "开始电机跟随测试." << std::endl;
        
        // 启动测试
        motor_core_.startTest();

        // 重新初始化时间基准，准备进入控制循环
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        
        // 确保控制频率是正确的浮点计算
        double controlPeriod = 1.0 / controlFrequency;                           // 单位：秒
        long long controlPeriodNs = static_cast<long long>(controlPeriod * 1e9); // 单位：纳秒

        while (ros::ok()) {
            const auto currentTime = std::chrono::high_resolution_clock::now();

            // 处理ROS回调
            ros::spinOnce();
            
            // 硬件状态由hardware_plant统一管理
            
            // 检查测试是否完成
            if (motor_core_.isTestComplete()) {
                std::cout << "电机跟随测试完成" << std::endl;
                break;
            }
            
            // 输出测试进度
            static int progress_counter = 0;
            if (progress_counter++ % 1000 == 0) { // 每2秒输出一次进度
                double progress = motor_core_.getTestProgress();
            }
            
            // 读取传感器数据并更新缓存
            SensorData_t sensor_data_motor;
            if (!hardware_plant_->readSensor(sensor_data_motor)) {
                std::cerr << "读取传感器数据失败" << std::endl;
                continue;
            }
            
            // motor_follow_test测试的是单个电机，不需要motor2joint转换
            // 直接使用电机空间的数据进行测试
            // SensorData_t sensor_data_joint;
            // sensor_data_joint.resizeJoint(num_joint_);
            // hardware_plant_->motor2joint(sensor_data_motor, sensor_data_joint);
            
            // 更新缓存数据（只使用电机空间数据）
            setState(sensor_data_motor, sensor_data_motor);
            
            // 执行控制逻辑（参考备份版本）
            {
                std::lock_guard<std::mutex> lock(motor_joint_data_mtx_);
                if (sensor_data_joint_last_.joint_q.size() > 0) {
                    // 生成电机指令
                    JointCmd_t joint_cmd;
                    if (motor_core_.generateMotorCommand(sensor_data_joint_last_, joint_cmd)) {
                        // 根据驱动器类型设置控制模式
                        for (int i = 0; i < num_joint_; i++) {
                            if (motor_info_.driver[i] == RUIWO) {
                                // RUIWO使用位置控制
                                joint_cmd.control_mode[i] = MOTOR_CONTROL_MODE_POSITION;
                                joint_cmd.joint_pos[i] = motor_core_.getCurrentTargetPosition(i);
                                joint_cmd.joint_torque[i] = 0.0;
                            } else if (motor_info_.driver[i] == EC_MASTER) {
                                // EC使用力矩控制
                                joint_cmd.control_mode[i] = MOTOR_CONTROL_MODE_TORQUE;
                            }
                        }
                        
                        // 记录测试数据（目标位置和实际位置）
                        recordTestData(sensor_data_joint_last_, joint_cmd);
                        
                        // 转换为ROS消息并发布
                        kuavo_msgs::jointCmd ros_joint_cmd;
                        convertToRosMessage(joint_cmd, ros_joint_cmd);
                        
                        joint_cmd_pub_.publish(ros_joint_cmd);
                        
                        // 调用硬件控制函数
                        jointMoveServo(ros_joint_cmd);
                    }
                }
            }

            // 更新下一次执行时间
            next_time.tv_nsec += controlPeriodNs;
            while (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }

            // 等待到下一个周期
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

            // 检查周期时间
            const auto cycleTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - currentTime).count();
            if (cycleTime > 1000) {
                // 控制周期时间超过1秒的警告
            }
        }
        
        // 测试结束后的清理工作
        std::cout << "实物电机跟随测试结束，开始清理..." << std::endl;
        
        // 停止测试
        motor_core_.stopTest();
        
        // 执行紧急停止，确保所有电机停止
        emergencyStop();
        
        // 等待一段时间确保电机完全停止
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // hardware_plant析构时会自动清理硬件资源
        
        std::cout << "硬件清理完成，程序安全退出" << std::endl;
        
        // 自动运行对称性分析脚本
        runSymmetryAnalysis();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher joint_cmd_pub_;
    
    motor_follow_test::MotorFollowTestCore motor_core_;
    
    int robot_version_;
    std::string config_file_path_;
    
    // 机器人参数
    int num_joint_;
    int num_waist_joints_;
    int num_arm_joints_;
    int num_head_joints_;
    int na_foot_;
    
    // 硬件相关成员变量
    std::string hardware_plant_path_;
    
    // 添加HardwarePlant实例用于复用硬件控制方法
    std::unique_ptr<HighlyDynamic::HardwarePlant> hardware_plant_;
    
    // 硬件配置数据（保留必要的配置）
    HardwareSettings motor_info_;
    std::vector<uint8_t> joint_ids_;
    
    // 线程安全
    std::mutex motor_joint_data_mtx_;
    
    // 传感器数据缓存
    SensorData_t sensor_data_motor_last_;
    SensorData_t sensor_data_joint_last_;
    
    ros::Publisher sensor_data_pub_;
    ros::Publisher sensor_data_pub_orig_;
    
    
    
    void setState(const SensorData_t &sensor_data_motor, const SensorData_t &sensor_data_joint) {
        std::lock_guard<std::mutex> lock(motor_joint_data_mtx_);
        sensor_data_motor_last_ = sensor_data_motor;
        sensor_data_joint_last_ = sensor_data_joint;
    }
    
    void jointMoveServo(const kuavo_msgs::jointCmd &jointCmdMsg) {
        // 参考原始motor_follow_test.cpp的jointMoveServo实现
        kuavo_msgs::jointCmd modifiedCmdMsg = jointCmdMsg;

    std::vector<uint8_t> joint_tau_ids;
    std::vector<uint8_t> joint_vel_ids;
    std::vector<uint8_t> joint_pos_ids;
    std::vector<JointParam_t> joint_tau_cmd;
    std::vector<JointParam_t> joint_vel_cmd;
    std::vector<JointParam_t> joint_pos_cmd;

    uint8_t num_tau = 0, num_vel = 0, num_pos = 0;

        for (uint32_t i = 0; i < num_joint_; i++) {
        JointParam_t joint_tau_temp;
        joint_tau_temp.status = 0;
            switch (modifiedCmdMsg.control_modes[i]) {
            case MOTOR_CONTROL_MODE_TORQUE: {
            joint_tau_temp.torqueOffset = 0;
            joint_tau_temp.torque = modifiedCmdMsg.tau[i] / motor_info_.c2t_coeff_default[i];
            joint_tau_temp.maxTorque = modifiedCmdMsg.tau_max[i];
            joint_tau_temp.position = modifiedCmdMsg.joint_q[i] * (180.0 / M_PI);
            joint_tau_temp.velocityOffset = modifiedCmdMsg.joint_v[i] * (180.0 / M_PI);
            joint_tau_temp.kp = modifiedCmdMsg.joint_kp[i];
            joint_tau_temp.kd = modifiedCmdMsg.joint_kd[i];
            
            joint_tau_cmd.push_back(joint_tau_temp);
            joint_tau_ids.push_back(joint_ids_[i]);
            num_tau++;
            break;
        }
            case MOTOR_CONTROL_MODE_VELOCITY: {
            joint_tau_temp.velocityOffset = 0;
                joint_tau_temp.velocity = modifiedCmdMsg.joint_v[i] * (180.0 / M_PI);
            joint_tau_temp.torqueOffset = modifiedCmdMsg.tau[i] / motor_info_.c2t_coeff_default[i];
            joint_tau_temp.maxTorque = modifiedCmdMsg.tau_max[i];
            joint_vel_cmd.push_back(joint_tau_temp);
            joint_vel_ids.push_back(joint_ids_[i]);
            num_vel++;
            break;
        }
            case MOTOR_CONTROL_MODE_POSITION: {
            joint_tau_temp.positionOffset = 0;
            joint_tau_temp.position = modifiedCmdMsg.joint_q[i] * (180.0 / M_PI);
            joint_tau_temp.velocityOffset = modifiedCmdMsg.joint_v[i] * (180.0 / M_PI);
            joint_tau_temp.maxTorque = modifiedCmdMsg.tau_max[i];

            auto tmp_tau = modifiedCmdMsg.tau[i] / motor_info_.c2t_coeff_default[i] * modifiedCmdMsg.tau_ratio[i];
                if (tmp_tau > joint_tau_temp.maxTorque) {
                joint_tau_temp.torqueOffset = joint_tau_temp.maxTorque;
                } else if (tmp_tau < -joint_tau_temp.maxTorque) {
                joint_tau_temp.torqueOffset = -joint_tau_temp.maxTorque;
                } else {
                joint_tau_temp.torqueOffset = tmp_tau;
            }

            joint_pos_cmd.push_back(joint_tau_temp);
            joint_pos_ids.push_back(joint_ids_[i]);
            num_pos++;
            break;
        }
        default:
            break;
        }
    }

        if (num_tau > 0) {
        hardware_plant_->SetMotorTorque(joint_tau_ids, joint_tau_cmd);
    }
        if (num_pos > 0) {
        hardware_plant_->SetMotorPosition(joint_pos_ids, joint_pos_cmd);
    }

}


    void recordTestData(const SensorData_t& sensor_data, const JointCmd_t& joint_cmd) {
        // 获取当前测试的关节对
        auto test_pairs = motor_core_.getTestPairs();
        if (motor_core_.getCurrentPairIndex() < test_pairs.size()) {
            auto current_pair = test_pairs[motor_core_.getCurrentPairIndex()];
            int l_index = current_pair.left_joint;
            int r_index = current_pair.right_joint;
            
            // 记录命令数据（目标位置）
            if (l_index < joint_cmd.joint_pos.size()) {
                motor_core_.recordJointCommand(l_index, joint_cmd.joint_pos[l_index]);
            }
            if (r_index < joint_cmd.joint_pos.size()) {
                motor_core_.recordJointCommand(r_index, joint_cmd.joint_pos[r_index]);
            }
            
            // 记录响应数据（实际位置）
            if (l_index < sensor_data.joint_q.size()) {
                motor_core_.recordJointResponse(l_index, sensor_data.joint_q[l_index]);
            }
            if (r_index < sensor_data.joint_q.size()) {
                motor_core_.recordJointResponse(r_index, sensor_data.joint_q[r_index]);
            }
        }
    }
    
    int findCompatibleVersion(const std::string& kuavo_assets_path, int version, const std::string& file_type) {
        // 向下兼容查找配置文件版本号
        // file_type: "kuavo" 或 "control_params"
        // 返回找到的版本号，如果找不到返回-1
        
        int major_version = version / 10;  // 十位数
        
        // 从当前版本向下查找到该十位数的最小版本
        for (int v = version; v >= major_version * 10; v--) {
            bool exists = false;
            
            if (file_type == "kuavo") {
                std::string robot_name = "kuavo_v" + std::to_string(v);
                std::string config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
                exists = std::filesystem::exists(config_path);
            } else if (file_type == "control_params") {
                std::string params_file = kuavo_assets_path + "/config/joint_control_params_v" + std::to_string(v) + ".json";
                exists = std::filesystem::exists(params_file);
            }
            
            if (exists) {
                if (v != version) {
                    std::cout << "使用兼容版本 v" << v << " 的" << file_type << "配置文件" << std::endl;
                }
                return v;
            }
        }
        
        return -1;  // 未找到
    }
    
    void loadRobotParameters() {
        // 参考KuavoCommon的做法，直接从文件路径读取kuavo.json
        try {
            // 获取kuavo_assets路径 - 从ROS参数读取
            std::string kuavo_assets_path;
            if (!nh_.getParam("/repo_root_path", kuavo_assets_path)) {
                std::cerr << "无法获取/repo_root_path参数，无法继续" << std::endl;
                setDefaultParameters();
                return;
            }
            kuavo_assets_path += "/src/kuavo_assets";
            
            // 构建配置文件路径
            std::string robot_name = "kuavo_v" + std::to_string(robot_version_);
            std::string config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
            
            // 检查文件是否存在，如果不存在则查找兼容版本
            if (!std::filesystem::exists(config_path)) {
                int compatible_version = findCompatibleVersion(kuavo_assets_path, robot_version_, "kuavo");
                if (compatible_version == -1) {
                    std::cerr << "错误：在版本范围 " << (robot_version_ / 10) << "0-" << robot_version_ 
                              << " 内未找到kuavo.json配置文件" << std::endl;
                    setDefaultParameters();
                    return;
                }
                // 使用兼容版本的路径
                robot_name = "kuavo_v" + std::to_string(compatible_version);
                config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
            }
            
            std::cout << "使用配置文件: kuavo_v" << robot_name.substr(7) << "/kuavo.json" << std::endl;
            
            // 读取并解析JSON文件
            std::ifstream config_file(config_path);
            if (!config_file.is_open()) {
                std::cerr << "无法打开配置文件: " << config_path << std::endl;
                setDefaultParameters();
                return;
            }
            
            Json::Value root;
            Json::Reader reader;
            if (!reader.parse(config_file, root)) {
                std::cerr << "JSON解析失败: " << reader.getFormattedErrorMessages() << std::endl;
                setDefaultParameters();
                return;
            }
            
            // 读取关节数量参数
            if (root.isMember("NUM_JOINT")) {
                num_joint_ = root["NUM_JOINT"].asInt();
        } else {
                num_joint_ = 29; // 默认值
            }
            
            if (root.isMember("NUM_WAIST_JOINT")) {
                num_waist_joints_ = root["NUM_WAIST_JOINT"].asInt();
            } else {
                num_waist_joints_ = 1; // 默认值
            }
            
            if (root.isMember("NUM_ARM_JOINT")) {
                num_arm_joints_ = root["NUM_ARM_JOINT"].asInt();
            } else {
                num_arm_joints_ = 14; // 默认值
            }
            
            if (root.isMember("NUM_HEAD_JOINT")) {
                num_head_joints_ = root["NUM_HEAD_JOINT"].asInt();
            } else {
                num_head_joints_ = 2; // 默认值
            }
            
            // 计算腿部关节数
            na_foot_ = num_joint_ - num_waist_joints_ - num_arm_joints_ - num_head_joints_;
            
            std::cout << "成功从文件读取机器人参数:" << std::endl;
            std::cout << "  总关节数: " << num_joint_ << std::endl;
            std::cout << "  腰部关节数: " << num_waist_joints_ << std::endl;
            std::cout << "  手臂关节数: " << num_arm_joints_ << std::endl;
            std::cout << "  头部关节数: " << num_head_joints_ << std::endl;
            std::cout << "  腿部关节数: " << na_foot_ << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "读取机器人参数时发生异常: " << e.what() << std::endl;
            setDefaultParameters();
        }
    }
    
    void setDefaultParameters() {
        num_joint_ = 29;
        num_waist_joints_ = 1;
        num_arm_joints_ = 14;
        num_head_joints_ = 2;
        na_foot_ = 12;
    }
    
    void loadHardwareConfig() {
        // 参考原脚本的loadConfig函数实现
        std::cout << "========= 加载硬件配置 =========" << std::endl;
        
        try {
            // 参考原始脚本的版本获取逻辑
            RobotVersion version(5, 1); // 默认版本
            int robot_version_int = 51; // 默认版本
            
            // 从ROS参数获取版本
            if (nh_.hasParam("/robot_version")) {
                nh_.getParam("/robot_version", robot_version_int);
                std::cout << "从ROS参数获取robot_version: " << robot_version_int << std::endl;
                int major = robot_version_int / 10;
                int minor = robot_version_int % 10;
        version = RobotVersion(major, minor);
                robot_version_ = robot_version_int; // 更新成员变量
    }

            // 从环境变量获取版本（优先级更高）
    const char* robot_version_env = std::getenv("ROBOT_VERSION");
            if (robot_version_env) {
                std::cout << "从环境变量获取ROBOT_VERSION: " << robot_version_env << std::endl;
                robot_version_int = std::stoi(robot_version_env);
                int major = robot_version_int / 10;
                int minor = robot_version_int % 10;
        version = RobotVersion(major, minor);
                robot_version_ = robot_version_int; // 更新成员变量
            } else {
                std::cout << "ROBOT_VERSION 环境变量未设置，使用ROS参数或默认值" << std::endl;
            }
            
            std::cout << "最终使用的robot_version: " << robot_version_ << std::endl;
            
            // 获取kuavo_assets路径 - 从ROS参数读取
            std::string kuavo_assets_path;
            if (!nh_.getParam("/repo_root_path", kuavo_assets_path)) {
                std::cerr << "无法获取/repo_root_path参数，无法继续" << std::endl;
                return;
            }
            kuavo_assets_path += "/src/kuavo_assets";
            
            // 获取KuavoCommon实例 (参考hardware_plant.cc的实现)
            auto kuavo_common_ptr = KuavoCommon::getInstancePtr(version, kuavo_assets_path);
            auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
            
            // 加载硬件设置
            motor_info_ = kuavo_settings.hardware_settings;
            
            // 设置关节ID
            joint_ids_ = motor_info_.joint_ids;
            
            // 配置踝关节求解器
            JSONConfigReader *robot_config = kuavo_common_ptr->getRobotConfig();
            // AnkleSolverType ankleSolverType = static_cast<AnkleSolverType>(robot_config->getValue<int>("ankle_solver_type"));
            // ankleSolver配置由hardware_plant管理
            // ankleSolver_.getconfig(ankleSolverType);
            
            std::cout << "硬件配置加载完成" << std::endl;
            std::cout << "  关节ID数量: " << joint_ids_.size() << std::endl;
            
            // 初始化HardwarePlant实例
            std::cout << "开始初始化HardwarePlant实例..." << std::endl;
            
            // 创建HardwareParam配置
            HighlyDynamic::HardwareParam hw_param;
            hw_param.robot_version = RobotVersion(robot_version_ / 10, robot_version_ % 10);
            hw_param.cali = false;
            hw_param.cali_arm = false;
            hw_param.cali_leg = false;
            
            // 创建HardwarePlant实例
            hardware_plant_ = std::make_unique<HighlyDynamic::HardwarePlant>(
                1e-3,  // dt
                hw_param,
                hardware_plant_path_,
                MOTOR_CONTROL_MODE_TORQUE,
                0,     // num_actuated
                7,     // nq_f
                6      // nv_f
            );
            
            // 初始化HardwarePlant
            int init_result = hardware_plant_->HWPlantInit();
            if (init_result != 0) {
                std::cout << "HardwarePlant初始化失败，错误代码: " << init_result << std::endl;
                hardware_plant_.reset();
                return;
            }
            
            std::cout << "HardwarePlant初始化成功" << std::endl;
            std::cout << "硬件配置和初始化完成" << std::endl;
            
        } catch (const std::exception& e) {
            std::cerr << "加载硬件配置时发生异常: " << e.what() << std::endl;
            // 使用默认配置
            joint_ids_.resize(num_joint_);
        }
    }
    

    motor_follow_test::MotorFollowTestCore::TestMode getTestModeFromParam() {
        int test_mode_param;
        nh_.param("test_mode", test_mode_param, 0); // 默认全身测试

        std::cout << "\n=== 电机跟随测试模式 ===" << std::endl;

        switch (test_mode_param) {
            case 0:
                std::cout << "测试模式：全身测试 (腰部 + 腿部 + 手臂)" << std::endl;
                return motor_follow_test::MotorFollowTestCore::TestMode::FULL_BODY;
            case 1:
                std::cout << "测试模式：腿部测试 (腰部 + 腿部)" << std::endl;
                return motor_follow_test::MotorFollowTestCore::TestMode::LEGS_ONLY;
            case 2:
                std::cout << "测试模式：手臂测试 (手臂)" << std::endl;
                return motor_follow_test::MotorFollowTestCore::TestMode::ARMS_ONLY;
            default:
                std::cout << "无效的测试模式参数 (" << test_mode_param << ")，使用默认：全身测试" << std::endl;
                return motor_follow_test::MotorFollowTestCore::TestMode::FULL_BODY;
        }
    }
    
    void convertToRosMessage(const JointCmd_t& joint_cmd, kuavo_msgs::jointCmd& ros_joint_cmd) {
        // 设置消息头
        ros_joint_cmd.header.stamp = ros::Time::now();
        ros_joint_cmd.header.frame_id = "base_link";

        // 设置关节数据
        ros_joint_cmd.joint_q = joint_cmd.joint_pos;
        ros_joint_cmd.joint_v = joint_cmd.joint_vel;

        // 初始化所有必需的数组
        size_t num_joints = joint_cmd.joint_pos.size();
        ros_joint_cmd.tau = joint_cmd.joint_torque;  // 使用计算好的力矩
        ros_joint_cmd.tau_max.resize(num_joints, 10.0);  // 默认最大力矩
        ros_joint_cmd.tau_ratio.resize(num_joints, 1.0); // 默认力矩比例
        ros_joint_cmd.control_modes.resize(num_joints);
        ros_joint_cmd.joint_kp.resize(num_joints);
        ros_joint_cmd.joint_kd.resize(num_joints);

        // 根据关节类型设置控制模式和KP/KD值
        for (size_t i = 0; i < num_joints; ++i) {
            try {
                double kp = motor_core_.getJointKp(i);
                double kd = motor_core_.getJointKd(i);
                ros_joint_cmd.joint_kp[i] = kp;
                ros_joint_cmd.joint_kd[i] = kd;
                
                // 根据驱动器类型和测试关节状态设置控制模式
                if (motor_info_.driver[i] == RUIWO) {
                    // RUIWO驱动器：始终使用位置控制
                    ros_joint_cmd.control_modes[i] = MOTOR_CONTROL_MODE_POSITION;
                    if (!motor_core_.isCurrentTestJoint(i)) {
                        ros_joint_cmd.tau[i] = 0.0;  // 非测试关节力矩置0
                    }
                } else if (motor_info_.driver[i] == EC_MASTER) {
                    // EC_MASTER驱动器：根据测试关节状态设置
                    if (motor_core_.isCurrentTestJoint(i)) {
                        // 测试关节：使用力矩模式，保持计算出的力矩
                        ros_joint_cmd.control_modes[i] = MOTOR_CONTROL_MODE_TORQUE;
                    } else {
                        // 非测试关节：使用位置模式，力矩置0
                        ros_joint_cmd.control_modes[i] = MOTOR_CONTROL_MODE_POSITION;
                        ros_joint_cmd.tau[i] = 0.0;  // 非测试关节力矩置0
                    }
                } else {
                    // 其他驱动器：默认位置模式
                    ros_joint_cmd.control_modes[i] = MOTOR_CONTROL_MODE_POSITION;
                    ros_joint_cmd.tau[i] = 0.0;
                }
            } catch (const std::exception& e) {
                std::cout << "[ERROR] 关节 " << i << " 获取KP/KD失败: " << e.what() << std::endl;
                ros_joint_cmd.joint_kp[i] = 5.0;  // 默认值
                ros_joint_cmd.joint_kd[i] = 0.01;   // 默认值
                ros_joint_cmd.control_modes[i] = MOTOR_CONTROL_MODE_POSITION; // 默认位置模式
                ros_joint_cmd.tau[i] = 0.0;  // 默认力矩置0
            }
        }
    }
    

    // 紧急停止函数 - 直接触发hardware_plant析构
    void emergencyStop() {
        std::cout << "执行紧急停止..." << std::endl;
        
        try {
            // 直接触发hardware_plant析构，与hardware_plant保持一样的实现
            if (hardware_plant_) {
                hardware_plant_.reset(); // 触发析构函数
                std::cout << "HardwarePlant已析构，紧急停止完成" << std::endl;
            } else {
                std::cout << "HardwarePlant未初始化，无需紧急停止" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cout << "紧急停止时发生异常: " << e.what() << std::endl;
        }
    }
    
    void runSymmetryAnalysis() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "开始运行对称性分析脚本..." << std::endl;
        std::cout << "========================================\n" << std::endl;
        
        // 获取脚本路径
        const std::string hardware_node_path = ros::package::getPath("hardware_node");
        const std::string script_path = hardware_node_path + "/src/tests/motorTest/symmetry_analysis.py";
        const std::string data_dir = hardware_node_path + "/src/tests/motorTest/file";
        
        // 检查脚本是否存在
        if (!std::filesystem::exists(script_path)) {
            std::cerr << "警告：对称性分析脚本不存在: " << script_path << std::endl;
            return;
        }
        
        // 检查数据目录是否存在
        if (!std::filesystem::exists(data_dir)) {
            std::cerr << "警告：测试数据目录不存在: " << data_dir << std::endl;
            return;
        }
        
        // 构建Python命令
        std::string command = "python3 " + script_path + " " + data_dir + 
                            " --robot-version " + std::to_string(robot_version_);
        
        std::cout << "执行命令: " << command << std::endl;
        
        // 执行Python脚本
        int result = system(command.c_str());
        
        if (result == 0) {
            std::cout << "\n========================================" << std::endl;
            std::cout << "✅ 对称性分析完成！" << std::endl;
            std::cout << "========================================\n" << std::endl;
        } else {
            std::cerr << "\n========================================" << std::endl;
            std::cerr << "⚠️  对称性分析脚本执行失败，返回码: " << result << std::endl;
            std::cerr << "========================================\n" << std::endl;
        }
    }
    
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "MotorFollowTest");
    
    // 注册信号处理函数
    signal(SIGINT, MotorFollowTest::signalHandler);   // Ctrl+C
    signal(SIGTERM, MotorFollowTest::signalHandler);  // 终止信号
    
    try {
        MotorFollowTest motor_test;
        motor_test.run();
    } catch (const std::exception& e) {
        std::cout << "MotorFollowTest 异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}


