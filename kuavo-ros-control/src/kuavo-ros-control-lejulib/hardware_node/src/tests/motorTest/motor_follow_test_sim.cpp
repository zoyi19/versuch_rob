#include <ros/ros.h>
#include <kuavo_msgs/jointCmd.h>
#include <kuavo_msgs/sensorsData.h>
#include <kuavo_common/common/sensor_data.h>
#include <std_srvs/SetBool.h>
#include "motor_follow_test_core.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <jsoncpp/json/json.h>

class MotorFollowTestSim {
public:
    MotorFollowTestSim() : nh_("~") {
        // 读取参数
        nh_.param("is_real", is_real_, false);
        nh_.param("use_mujoco", use_mujoco_, true);
        nh_.param("robot_version", robot_version_, 13);
        nh_.param("joint_control_config_file", config_file_path_, std::string(""));
        
        // 从ROS参数服务器读取机器人参数
        loadRobotParameters();
        
        std::cout << "MotorFollowTestSim 初始化 - 机器人版本: " << robot_version_ 
                  << ", 模式: " << (is_real_ ? "实物" : "MuJoCo仿真") << std::endl;
        
        // 从ROS参数获取测试模式
        motor_follow_test::MotorFollowTestCore::TestMode test_mode = getTestModeFromParam();
        
        // 初始化核心类 - 传递所有必要的参数
        motor_follow_test::MotorFollowTestCore::TestConfig config;
        config.robot_version = robot_version_;
        config.num_joint = num_joint_;
        config.num_waist_joints = num_waist_joints_;
        config.num_arm_joints = num_arm_joints_;
        config.num_head_joints = num_head_joints_;
        config.na_foot = na_foot_;
        config.config_file_path = config_file_path_;
        config.is_real = is_real_;
        config.dt = 0.002;
        config.test_duration_ms = 10000; // 10秒测试
        config.test_mode = test_mode;
        
        try {
            if (!motor_core_.initialize(config)) {
                std::cout << "[ERROR] MotorFollowTestCore 初始化失败" << std::endl;
                core_initialized_ = false;
                return;
            }
            core_initialized_ = true;
            
            // 初始化传感器数据 - 参考humanoidController.cpp的做法
            latest_sensor_data_.time = 0.0;
            latest_sensor_data_.resizeJoint(num_joint_);  // 使用resizeJoint方法
            latest_sensor_data_.gyro.setZero();
            latest_sensor_data_.acc.setZero();
            latest_sensor_data_.free_acc.setZero();
            latest_sensor_data_.quat.setZero();
            latest_sensor_data_.gyro_W.setZero();
            latest_sensor_data_.acc_W.setZero();
            latest_sensor_data_.free_acc_W.setZero();
            latest_sensor_data_.quat_W.setZero();
            latest_sensor_data_.end_effectors_data.clear();
        } catch (const std::exception& e) {
            std::cout << "[ERROR] MotorFollowTestCore 初始化异常: " << e.what() << std::endl;
            core_initialized_ = false;
            return;
        } catch (...) {
            std::cout << "[ERROR] MotorFollowTestCore 初始化未知异常" << std::endl;
            core_initialized_ = false;
            return;
        }
    }
    
    ~MotorFollowTestSim() {
        motor_core_.stopTest();
        if (sensor_thread_.joinable()) {
            sensor_thread_.join();
        }
    }
    
    void run() {
        // 初始化MuJoCo仿真接口
        initMujocoSimulation();
        
        // 设置初始姿态参数
        setInitialPose();
        
        // 启动测试（只有在core初始化成功时才启动）
        if (core_initialized_) {
            if (!motor_core_.startTest()) {
                std::cout << "[ERROR] 无法启动电机跟随测试" << std::endl;
                return;
            }
        } else {
            std::cout << "[WARNING] Core初始化失败，跳过电机跟随测试启动" << std::endl;
        }
        
        // 启动传感器线程
        sensor_thread_ = std::thread(&MotorFollowTestSim::sensorThreadFunc, this);
        
        // 等待MuJoCo完全初始化后再调用sim_start服务
        ros::Duration(2.0).sleep();
        
        // 只有在Core初始化成功时才调用sim_start服务
        if (core_initialized_) {
            callSimStartSrv();
        }
        
        ros::Rate rate(500); // 500Hz
        
        while (ros::ok()) {
            // 处理传感器数据并生成电机指令
            if (motor_core_.isTestComplete()) {
                std::cout << "电机跟随测试完成" << std::endl;
                
                // 测试完成，数据已经在每个测试对完成时保存了
                std::cout << "测试数据已保存到文件" << std::endl;
                
                // 生成并打印测试报告
                std::string report = motor_core_.generateTestReport();
                std::cout << report << std::endl;
                
                break;
            }
            
            // 获取传感器数据
            SensorData_t sensor_data;
            if (getSensorData(sensor_data)) {
                // 生成电机指令
                JointCmd_t joint_cmd;
                if (motor_core_.generateMotorCommand(sensor_data, joint_cmd)) {
                    // 记录测试关节的命令和响应数据
                    recordTestData(sensor_data, joint_cmd);
                    
                    // 转换为ROS消息并发送
                    kuavo_msgs::jointCmd ros_joint_cmd;
                    convertToRosMessage(joint_cmd, ros_joint_cmd);
                    
                    
                    sendMotorCommand(ros_joint_cmd);
                }
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void recordTestData(const SensorData_t& sensor_data, const JointCmd_t& joint_cmd) {
        // 获取当前测试的关节对
        auto test_pairs = motor_core_.getTestPairs();
        if (motor_core_.getCurrentPairIndex() < test_pairs.size()) {
            auto current_pair = test_pairs[motor_core_.getCurrentPairIndex()];
            int l_index = current_pair.left_joint;
            int r_index = current_pair.right_joint;
            
            // 记录命令数据（目标位置） - 从joint_cmd获取目标位置
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
            
            // 调试输出
            static int debug_count = 0;
            if (debug_count++ % 1000 == 0) {
                std::cout << "[DEBUG] 记录测试数据: L=" << l_index << " target=" << joint_cmd.joint_pos[l_index] 
                          << " actual=" << (l_index < sensor_data.joint_q.size() ? sensor_data.joint_q[l_index] : -999)
                          << " R=" << r_index << " target=" << joint_cmd.joint_pos[r_index] 
                          << " actual=" << (r_index < sensor_data.joint_q.size() ? sensor_data.joint_q[r_index] : -999) << std::endl;
            }
        }
    }
    
    
    void loadRobotParameters() {
        // 参考KuavoCommon的做法，直接从文件路径读取kuavo.json
        try {
            // 获取kuavo_assets路径
            std::string kuavo_assets_path;
            if (!nh_.getParam("/repo_root_path", kuavo_assets_path)) {
                kuavo_assets_path = "/root/kuavo_ws/src/kuavo_assets";
            } else {
                kuavo_assets_path += "/src/kuavo_assets";
            }
            
            // 构建配置文件路径
            std::string robot_name = "kuavo_v" + std::to_string(robot_version_);
            std::string config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
            
            std::cout << "尝试从文件路径读取配置: " << config_path << std::endl;
            
            // 检查文件是否存在
            if (!std::filesystem::exists(config_path)) {
                std::cerr << "配置文件不存在: " << config_path << std::endl;
                setDefaultParameters();
                return;
            }
            
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
                num_joint_ = 23; // 默认值
            }
            
            if (root.isMember("NUM_WAIST_JOINT")) {
                num_waist_joints_ = root["NUM_WAIST_JOINT"].asInt();
            } else {
                num_waist_joints_ = 1; // 默认值
            }
            
            if (root.isMember("NUM_ARM_JOINT")) {
                num_arm_joints_ = root["NUM_ARM_JOINT"].asInt();
            } else {
                num_arm_joints_ = 8; // 默认值
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
        num_joint_ = 23;
        num_waist_joints_ = 1;
        num_arm_joints_ = 8;
        num_head_joints_ = 2;
        na_foot_ = 12;
        
    }
    
    void setInitialPose() {
        std::cout << "设置仿真初始姿态" << std::endl;
        
        // 创建初始姿态参数向量
        // 格式: [x, y, z, qw, qx, qy, qz, joint1, joint2, ..., jointN]
        std::vector<double> robot_init_state_param;
        
        // 位置: x=0, y=0, z=1.0 (站立高度)
        robot_init_state_param.push_back(0.0);  // x
        robot_init_state_param.push_back(0.0);  // y
        robot_init_state_param.push_back(1.0);  // z
        
        // 四元数: 无旋转 (qw=1, qx=0, qy=0, qz=0)
        robot_init_state_param.push_back(1.0);  // qw
        robot_init_state_param.push_back(0.0);  // qx
        robot_init_state_param.push_back(0.0);  // qy
        robot_init_state_param.push_back(0.0);  // qz
        
        // 所有关节位置设为0 (站立姿态)
        for (int i = 0; i < num_joint_; ++i) {
            robot_init_state_param.push_back(0.0);
        }
        
        // 设置ROS参数 - 参考humanoidController.cpp的参数设置方式
        ros::param::set("robot_init_state_param", robot_init_state_param);
        ros::param::set("/humanoid/init_q", robot_init_state_param);
        
        // 设置腰部关节数量参数，供MuJoCo使用
        ros::param::set("waistRealDof", num_waist_joints_);
        
        // 设置MuJoCo固定约束参数 - 参考mujoco_node.cc的固定方式
        setMujocoConstraints();
        
        // 注意：在构造函数中不调用sleep，避免ROS初始化问题
        
        std::cout << "设置初始姿态参数，总长度: " << robot_init_state_param.size() << std::endl;
        std::cout << "位置: [" << robot_init_state_param[0] << ", " << robot_init_state_param[1] << ", " << robot_init_state_param[2] << "]" << std::endl;
        std::cout << "四元数: [" << robot_init_state_param[3] << ", " << robot_init_state_param[4] << ", " << robot_init_state_param[5] << ", " << robot_init_state_param[6] << "]" << std::endl;
        std::cout << "关节数量: " << num_joint_ << std::endl;
        std::cout << "腰部关节数量: " << num_waist_joints_ << std::endl;
    }
    
    void setMujocoConstraints() {
        std::cout << "设置MuJoCo约束参数" << std::endl;
        
        // 不使用约束，让机器人自由运动
        // 注意：MuJoCo现有实现只支持only_half_up_body模式（同时固定躯干和腿部）
        // 为了motor_follow_test，我们不启用任何约束，依赖控制器保持稳定
        bool only_half_up_body = true;
        ros::param::set("/only_half_up_body", only_half_up_body);
        
        std::cout << "约束配置：" << std::endl;
        std::cout << "  only_half_up_body: false - 不固定，机器人完全自由" << std::endl;
        std::cout << "  注意：依赖PD控制器保持机器人稳定性" << std::endl;
        
        std::cout << "MuJoCo约束参数设置完成" << std::endl;
    }
    
    void initMujocoSimulation() {
        std::cout << "初始化MuJoCo仿真接口" << std::endl;
        
        // 订阅传感器数据
        mujoco_sensor_sub_ = nh_.subscribe("/sensors_data_raw", 10, 
                                          &MotorFollowTestSim::mujocoSensorDataCallback, this);
        
        // 发布电机指令
        mujoco_cmd_pub_ = nh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
        
        // 注意：sim_start服务调用已移到所有初始化完成后，参考humanoidController.cpp的实现
        // 注意：传感器线程在run()函数中启动，避免在构造函数中启动线程
        
        std::cout << "MuJoCo仿真接口初始化完成" << std::endl;
    }
    
    void callSimStartSrv() {
        std_srvs::SetBool srv;
        srv.request.data = true;
        
        // 等待服务可用
        bool service_available = ros::service::waitForService("/sim_start", ros::Duration(5.0));
        
        if (service_available) {
            // 使用全局NodeHandle来访问全局命名空间的服务
            ros::NodeHandle global_nh;
            ros::ServiceClient sim_start_client = global_nh.serviceClient<std_srvs::SetBool>("/sim_start");
            
            // 等待客户端连接并调用服务
            if (sim_start_client.waitForExistence(ros::Duration(3.0))) {
                if (sim_start_client.call(srv)) {
                    if (srv.response.success) {
                        std::cout << "sim_start Service call succeeded" << std::endl;
                    } else {
                        ROS_WARN("sim_start Service call failed: %s", srv.response.message.c_str());
                    }
                } else {
                    ROS_WARN("Failed to call sim_start service");
                }
            } else {
                ROS_WARN("sim_start client failed to connect");
            }
        } else {
            ROS_WARN("sim_start Service not available");
        }
    }
    
    bool getSensorData(SensorData_t& sensor_data) {
        // 从MuJoCo仿真获取传感器数据 - 参考humanoidController.cpp的做法
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        
        if (latest_sensor_data_.joint_q.size() == 0) {
            // 如果没有传感器数据，返回零位置
            sensor_data.resizeJoint(num_joint_);
            return true;
        }
        
        // 直接赋值，SensorData_t结构体应该支持拷贝赋值
        sensor_data = latest_sensor_data_;
        return true;
    }
    
    void sendMotorCommand(const kuavo_msgs::jointCmd& joint_cmd) {
        mujoco_cmd_pub_.publish(joint_cmd);
    }
    
    // 将独立的数据结构转换为ROS消息
    void convertToRosMessage(const JointCmd_t& joint_cmd, kuavo_msgs::jointCmd& ros_joint_cmd) {
        // 设置消息头
        ros_joint_cmd.header.stamp = ros::Time::now();
        ros_joint_cmd.header.frame_id = "base_link";
        
        // 初始化所有必需的数组
        size_t num_joints = joint_cmd.joint_torque.size();
        
        // 设置关节数据 - 位置和速度设为0
        ros_joint_cmd.joint_q.resize(num_joints, 0.0);
        ros_joint_cmd.joint_v.resize(num_joints, 0.0);
        ros_joint_cmd.tau = joint_cmd.joint_torque;  // 使用计算好的力矩
        ros_joint_cmd.tau_max.resize(num_joints, 10.0);  // 默认最大力矩
        ros_joint_cmd.tau_ratio.resize(num_joints, 1.0); // 默认力矩比例
        ros_joint_cmd.control_modes = joint_cmd.control_mode;  // 使用指定的控制模式
        ros_joint_cmd.joint_kp.resize(num_joints);
        ros_joint_cmd.joint_kd.resize(num_joints);
        
        // 设置KP和KD值 - 参考mujoco_node.cc的控制策略
        for (size_t i = 0; i < num_joints; ++i) {
            try {
                double kp = motor_core_.getJointKp(i);
                double kd = motor_core_.getJointKd(i);
                
                // 根据是否为测试关节设置不同的控制参数
                if (motor_core_.isCurrentTestJoint(i)) {
                    // 测试关节：使用原始KP/KD值，允许正常运动
                    ros_joint_cmd.joint_kp[i] = kp;
                    ros_joint_cmd.joint_kd[i] = kd;
                } else {
                    // 非测试关节：使用高增益保持位置，增强稳定性
                    // 参考mujoco_node.cc中固定关节的控制策略
                    ros_joint_cmd.joint_kp[i] = kp * 10.0;  // 提高位置增益
                    ros_joint_cmd.joint_kd[i] = kd * 20.0;  // 提高阻尼增益
                }
            } catch (const std::exception& e) {
                std::cout << "[ERROR] 关节 " << i << " 获取KP/KD失败: " << e.what() << std::endl;
                // 设置安全的默认值
                if (motor_core_.isCurrentTestJoint(i)) {
                    ros_joint_cmd.joint_kp[i] = 100.0;   // 测试关节默认KP
                    ros_joint_cmd.joint_kd[i] = 10.0;    // 测试关节默认KD
                } else {
                    ros_joint_cmd.joint_kp[i] = 1000.0;  // 非测试关节高KP
                    ros_joint_cmd.joint_kd[i] = 200.0;   // 非测试关节高KD
                }
            }
        }
    }
    
    void sensorThreadFunc() {
        // 传感器数据处理线程
        ros::Rate rate(500); // 500Hz
        
        
        while (ros::ok()) {
            // 检查core是否初始化成功
            if (!core_initialized_) {
                rate.sleep();
                continue;
            }
            
            // 检查测试是否完成
            if (motor_core_.isTestComplete()) {
                break;
            }
            
            // 从MuJoCo仿真获取传感器数据
            SensorData_t sensor_data;
            if (getSensorData(sensor_data)) {
                // 生成电机指令
                JointCmd_t joint_cmd;
                if (motor_core_.generateMotorCommand(sensor_data, joint_cmd)) {
                    // 转换为ROS消息并发布
                    kuavo_msgs::jointCmd ros_joint_cmd;
                    convertToRosMessage(joint_cmd, ros_joint_cmd);
                    mujoco_cmd_pub_.publish(ros_joint_cmd);
                }
            }
            
            rate.sleep();
        }
        
        std::cout << "[INFO] 传感器线程结束" << std::endl;
    }
    
    // MuJoCo传感器数据回调函数 - 参考humanoidController.cpp的实现
    void mujocoSensorDataCallback(const kuavo_msgs::sensorsData::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(sensor_data_mutex_);
        
        auto &joint_data = msg->joint_data;
        const auto &end_effector_data = msg->end_effector_data;
        
        // 使用resizeJoint方法初始化传感器数据
        latest_sensor_data_.resizeJoint(num_joint_);
        
        // 设置时间戳
        latest_sensor_data_.time = msg->sensor_time.toSec();
        
        // 复制关节数据 - 参考humanoidController.cpp的做法
        for (size_t i = 0; i < num_joint_; ++i) {
            if (i < joint_data.joint_q.size()) {
                latest_sensor_data_.joint_q[i] = joint_data.joint_q[i];
            }
            if (i < joint_data.joint_v.size()) {
                latest_sensor_data_.joint_v[i] = joint_data.joint_v[i];
            }
            if (i < joint_data.joint_vd.size()) {
                latest_sensor_data_.joint_vd[i] = joint_data.joint_vd[i];
            }
            if (i < joint_data.joint_torque.size()) {
                latest_sensor_data_.joint_current[i] = joint_data.joint_torque[i];
            }
        }
        
        // 复制IMU数据
        latest_sensor_data_.gyro << msg->imu_data.gyro.x,
                                   msg->imu_data.gyro.y,
                                   msg->imu_data.gyro.z;
        
        latest_sensor_data_.acc << msg->imu_data.acc.x,
                                  msg->imu_data.acc.y,
                                  msg->imu_data.acc.z;
        
        latest_sensor_data_.free_acc << msg->imu_data.free_acc.x,
                                       msg->imu_data.free_acc.y,
                                       msg->imu_data.free_acc.z;
        
        latest_sensor_data_.quat << msg->imu_data.quat.w,
                                   msg->imu_data.quat.x,
                                   msg->imu_data.quat.y,
                                   msg->imu_data.quat.z;
        
        // 复制末端执行器数据 - 暂时跳过，因为类型不匹配
        // latest_sensor_data_.end_effectors_data = end_effector_data;
    }

private:
    ros::NodeHandle nh_;
    
    // 配置参数
    bool is_real_;
    
    /**
     * @brief 从ROS参数获取测试模式
     * @return 测试模式
     */
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
    bool use_mujoco_;
    int robot_version_;
    std::string config_file_path_;
    
    // 机器人参数
    int num_joint_;
    int num_waist_joints_;
    int num_arm_joints_;
    int num_head_joints_;
    int na_foot_;
    
    // 核心类
    motor_follow_test::MotorFollowTestCore motor_core_;
    bool core_initialized_;
    
    // ROS接口
    ros::Subscriber mujoco_sensor_sub_;
    ros::Publisher mujoco_cmd_pub_;
    
    // 传感器数据
    SensorData_t latest_sensor_data_;
    std::mutex sensor_data_mutex_;
    
    // 线程
    std::thread sensor_thread_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motorFollowTestSim");
    
    try {
        MotorFollowTestSim motor_test;
        motor_test.run();
    } catch (const std::exception& e) {
        ROS_ERROR("MotorFollowTestSim 异常: %s", e.what());
        return -1;
    }
    
    return 0;
}