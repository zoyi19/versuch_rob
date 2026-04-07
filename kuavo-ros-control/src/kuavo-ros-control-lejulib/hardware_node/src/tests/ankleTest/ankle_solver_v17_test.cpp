#include <ros/ros.h>
#include <kuavo_msgs/jointCmd.h>
#include <kuavo_msgs/sensorsData.h>
#include <kuavo_common/common/sensor_data.h>
#include <mutex>
#include <chrono>
#include <thread>
#include <atomic>
#include <signal.h>
#include <csignal>
#include <fstream>
#include <filesystem>
#include <jsoncpp/json/json.h>
#include <ros/package.h>
#include <time.h>
#include <algorithm>
#include <iomanip>
#include <sstream>

// 硬件接口相关头文件
#include "kuavo_common/common/common.h"
#include "kuavo_common/kuavo_common.h"
#include "kuavo_common/common/kuavo_settings.h"
#include "hardware_plant.h"
#include "kuavo_solver/ankle_solver.h"

// 使用hardware_plant中定义的数据结构
using namespace HighlyDynamic;

// 脚踝关节索引定义
const int LEFT_PITCH_IDX = 4;
const int LEFT_ROLL_IDX = 5;
const int RIGHT_PITCH_IDX = 10;
const int RIGHT_ROLL_IDX = 11;

// 工作空间限制（S2GEN_2）
const double PITCH_MIN = -0.872664625997165;  // 约 -50°
const double PITCH_MAX = 0.523598775598299;  // 约 30°
const double ROLL_MIN = -0.261799387799149;  // 约 -15°
const double ROLL_MAX = 0.261799387799149;   // 约 15°

// CST模式定义
#define CST 0

class AnkleSolverV17Test {
public:
    // 静态指针用于信号处理
    static AnkleSolverV17Test* instance_ptr_;
    
    // 静态信号处理函数
    static void signalHandler(int signal) {
        if (instance_ptr_) {
            instance_ptr_->should_stop_ = true;
            std::cout << "\n收到信号 " << signal << "，设置中断标志..." << std::endl;
            // 触发ROS shutdown，使ros::ok()返回false
            ros::shutdown();
        } else {
            // 如果实例不存在，直接退出
            std::cout << "\n收到信号 " << signal << "，直接退出..." << std::endl;
            ros::shutdown();
            emergencyStopStatic();
            exit(0);
        }
    }
    
    // 静态紧急停止方法
    static void emergencyStopStatic() {
        std::cout << "执行紧急停止..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "硬件清理完成" << std::endl;
    }

    AnkleSolverV17Test() : nh_("~"), should_stop_(false) {
        // 设置静态指针
        instance_ptr_ = this;
        
        // 读取参数
        nh_.param("robot_version", robot_version_, 17);
        
        // 从文件路径读取机器人参数
        loadRobotParameters();
        
        std::cout << "AnkleSolverV17Test 构造函数" << std::endl;
        std::cout << "机器人版本: " << robot_version_ << std::endl;
    }
    
    ~AnkleSolverV17Test() {
        emergencyStop();
    }
    
    void initialize() {
        std::cout << "开始初始化AnkleSolverV17Test..." << std::endl;
        
        // 设置硬件路径
        const std::string hardware_node_path = ros::package::getPath("hardware_node");
        const std::string hardware_plant_path = std::filesystem::path(hardware_node_path).parent_path().string() + "/hardware_plant";
        ::setHardwarePlantPath(hardware_plant_path);
        hardware_plant_path_ = hardware_plant_path;
        
        // 加载硬件配置
        loadHardwareConfig();
        
        // 初始化ankle solver
        ankle_solver_.getconfig(AnkleSolverType::ANKLE_SOLVER_TYPE_S2GEN_2);
        std::cout << "AnkleSolver已配置为S2GEN_2模式" << std::endl;
        
        std::cout << "AnkleSolverV17Test 初始化完成" << std::endl;
    }

    /** 纯 solver 力矩 round-trip 验证：不依赖硬件，仅检查 joint2motor 与 motor2joint 数值一致性 */
    void runSolverRoundTripVerification() {
        const double kTauTolerance = 1e-5;
        const double kCurrentTolerance = 1e-5;
        std::cout << "\n--- 纯 solver 力矩 round-trip 验证 (阈值 tau=" << kTauTolerance
                  << ", i=" << kCurrentTolerance << ") ---" << std::endl;

        Eigen::VectorXd q(12), p(12), tau(12), i_cmd(12);
        q.setZero();
        p.setZero();
        tau.setZero();
        i_cmd.setZero();

        // 多组 (q_ankle, p_ankle, tau_ankle) 在工作空间内采样
        struct Sample { double q4, q5, q10, q11, p4, p5, p10, p11, tau4, tau5, tau10, tau11; };
        std::vector<Sample> samples = {
            {0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.05, 0.1, 0.05},
            {0.05, 0.02, -0.05, -0.02, 0.1, 0.05, -0.1, -0.05, 0.2, 0.1, -0.2, -0.1},
            {(PITCH_MIN + PITCH_MAX) * 0.5, (ROLL_MIN + ROLL_MAX) * 0.5,
             (PITCH_MIN + PITCH_MAX) * 0.5, (ROLL_MIN + ROLL_MAX) * 0.5,
             0, 0, 0, 0, 0.15, 0.08, 0.15, 0.08},
        };

        int n_fail = 0;
        for (size_t s = 0; s < samples.size(); s++) {
            const auto& S = samples[s];
            q[4] = S.q4;  q[5] = S.q5;  q[10] = S.q10;  q[11] = S.q11;
            p[4] = S.p4;  p[5] = S.p5;  p[10] = S.p10;  p[11] = S.p11;
            tau[4] = S.tau4;  tau[5] = S.tau5;  tau[10] = S.tau10;  tau[11] = S.tau11;

            // joint2motor 纯净验证: tau -> i -> tau'
            Eigen::VectorXd i_out = ankle_solver_.joint_to_motor_current(q, p, tau);
            Eigen::VectorXd tau_back = ankle_solver_.motor_to_joint_torque(q, p, i_out);
            double tau_err = 0;
            tau_err += std::abs(tau_back[4] - tau[4]) + std::abs(tau_back[5] - tau[5])
                     + std::abs(tau_back[10] - tau[10]) + std::abs(tau_back[11] - tau[11]);
            if (tau_err > kTauTolerance || std::isnan(tau_err)) {
                std::cerr << "  [round-trip tau] sample " << s << " tau_error=" << tau_err << " (max allowed " << kTauTolerance << ")" << std::endl;
                n_fail++;
            }

            // motor2joint 纯净验证: i -> tau -> i'
            Eigen::VectorXd i_in(12);
            i_in.setZero();
            i_in[4] = i_out[4];  i_in[5] = i_out[5];  i_in[10] = i_out[10];  i_in[11] = i_out[11];
            Eigen::VectorXd tau_m2j = ankle_solver_.motor_to_joint_torque(q, p, i_in);
            Eigen::VectorXd i_back = ankle_solver_.joint_to_motor_current(q, p, tau_m2j);
            double i_err = 0;
            i_err += std::abs(i_back[4] - i_in[4]) + std::abs(i_back[5] - i_in[5])
                   + std::abs(i_back[10] - i_in[10]) + std::abs(i_back[11] - i_in[11]);
            if (i_err > kCurrentTolerance || std::isnan(i_err)) {
                std::cerr << "  [round-trip i] sample " << s << " i_error=" << i_err << " (max allowed " << kCurrentTolerance << ")" << std::endl;
                n_fail++;
            }
        }
        if (n_fail == 0) {
            std::cout << "  纯 solver round-trip 验证通过 (" << samples.size() << " 组样本)" << std::endl;
        } else {
            std::cerr << "  纯 solver round-trip 验证失败: " << n_fail << " 项未通过" << std::endl;
        }
        std::cout << "--- 纯 solver 力矩 round-trip 验证结束 ---\n" << std::endl;
    }

    void run() {
        std::cout << "====================== 开始脚踝解算验证测试 =====================" << std::endl;
        
        // 初始化
        initialize();
        
        double dt = 0.002;
        double controlFrequency = 1 / dt;
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        std::cout << "开始控制循环，频率: " << controlFrequency << " Hz" << std::endl;

        // 定义关节位置
        int num_joints = num_joint_;
        std::vector<double> cali_pos(num_joints, 0.0);
        
        // 初始化阶段
        std::cout << "关节数量: " << joint_ids_.size() << std::endl;

        std::vector<JointParam_t> jointData(num_joints);
        hardware_plant_->GetMotorData(joint_ids_, jointData);

        // 移动到初始位置
        double cali_tau_limit = 3.0;
        hardware_plant_->jointMoveTo(cali_pos, 60, dt, cali_tau_limit);

        // 初始化绝对时间基准
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        next_time.tv_sec += 3;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        std::cout << "开始脚踝解算测试." << std::endl;

        // 先执行纯 solver 力矩 round-trip 验证（不依赖硬件）
        if (ros::ok() && !should_stop_) {
            runSolverRoundTripVerification();
        }
        
        // 获取测试类型
        int test_type = 0;
        nh_.param("test_type", test_type, 0);
        
        // 获取左右脚测试模式
        nh_.param("ankle_test_foot", ankle_test_foot_, 0);
        std::cout << "左右脚测试模式: " << ankle_test_foot_ 
                  << " (0=both_same, 1=left_only, 2=right_only, 3=both_separate)" << std::endl;
        
        // 根据测试类型执行相应的测试（3=依次执行位置→速度→力矩）
        switch (test_type) {
            case 0:
                std::cout << "执行位置控制验证测试" << std::endl;
                testPositionControl();
                break;
            case 1:
                std::cout << "执行速度控制验证测试" << std::endl;
                testVelocityControl();
                break;
            case 2:
                std::cout << "执行力矩控制验证测试" << std::endl;
                testTorqueControl();
                break;
            case 3:
                std::cout << "依次执行：位置 → 速度 → 力矩 验证测试" << std::endl;
                if (ros::ok() && !should_stop_) {
                    std::cout << "\n--- [1/3] 位置控制验证测试 ---" << std::endl;
                    testPositionControl();
                }
                if (ros::ok() && !should_stop_) {
                    std::cout << "\n--- [2/3] 速度控制验证测试 ---" << std::endl;
                    testVelocityControl();
                }
                if (ros::ok() && !should_stop_) {
                    std::cout << "\n--- [3/3] 力矩控制验证测试 ---" << std::endl;
                    testTorqueControl();
                }
                break;
            default:
                std::cerr << "无效的测试类型: " << test_type << std::endl;
                break;
        }
        
        // 测试结束后的清理工作
        if (!ros::ok() || should_stop_) {
            std::cout << "脚踝解算测试被中断，开始清理..." << std::endl;
        } else {
            std::cout << "脚踝解算测试结束，开始清理..." << std::endl;
        }
        emergencyStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "硬件清理完成，程序安全退出" << std::endl;
        
        // 自动运行分析脚本
        runAnalysisScript();
    }

private:
    ros::NodeHandle nh_;
    
    int robot_version_;
    
    // 机器人参数
    int num_joint_;
    int num_waist_joints_;
    int num_arm_joints_;
    int num_head_joints_;
    int na_foot_;
    
    // 硬件相关成员变量
    std::string hardware_plant_path_;
    std::unique_ptr<HighlyDynamic::HardwarePlant> hardware_plant_;
    HardwareSettings motor_info_;
    std::vector<uint8_t> joint_ids_;
    
    // 保存默认kp/kd值（用于EC_MASTER电机位置控制）
    std::vector<double> default_joint_kp_;
    std::vector<double> default_joint_kd_;
    
    // AnkleSolver
    AnkleSolver ankle_solver_;
    
    // 线程安全
    std::mutex data_mtx_;
    
    // 传感器数据缓存
    SensorData_t sensor_data_motor_last_;
    SensorData_t sensor_data_joint_last_;
    
    // 数据记录文件
    std::ofstream csv_file_;
    
    // 中断标志
    std::atomic<bool> should_stop_;
    
    // 测试模式：0=both_same, 1=left_only, 2=right_only, 3=both_separate
    int ankle_test_foot_;
    
    void loadRobotParameters() {
        try {
            std::string kuavo_assets_path;
            if (!nh_.getParam("/repo_root_path", kuavo_assets_path)) {
                std::cerr << "无法获取/repo_root_path参数，无法继续" << std::endl;
                setDefaultParameters();
                return;
            }
            kuavo_assets_path += "/src/kuavo_assets";
            
            std::string robot_name = "kuavo_v" + std::to_string(robot_version_);
            std::string config_path = kuavo_assets_path + "/config/" + robot_name + "/kuavo.json";
            
            if (!std::filesystem::exists(config_path)) {
                std::cerr << "错误：未找到kuavo.json配置文件: " << config_path << std::endl;
                setDefaultParameters();
                return;
            }
            
            std::cout << "使用配置文件: " << config_path << std::endl;
            
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
            
            if (root.isMember("NUM_JOINT")) {
                num_joint_ = root["NUM_JOINT"].asInt();
            } else {
                num_joint_ = 29;
            }
            
            if (root.isMember("NUM_WAIST_JOINT")) {
                num_waist_joints_ = root["NUM_WAIST_JOINT"].asInt();
            } else {
                num_waist_joints_ = 1;
            }
            
            if (root.isMember("NUM_ARM_JOINT")) {
                num_arm_joints_ = root["NUM_ARM_JOINT"].asInt();
            } else {
                num_arm_joints_ = 14;
            }
            
            if (root.isMember("NUM_HEAD_JOINT")) {
                num_head_joints_ = root["NUM_HEAD_JOINT"].asInt();
            } else {
                num_head_joints_ = 2;
            }
            
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
        std::cout << "========= 加载硬件配置 =========" << std::endl;
        
        try {
            RobotVersion version(1, 7);
            int robot_version_int = 17;
            
            if (nh_.hasParam("/robot_version")) {
                nh_.getParam("/robot_version", robot_version_int);
                std::cout << "从ROS参数获取robot_version: " << robot_version_int << std::endl;
                int major = robot_version_int / 10;
                int minor = robot_version_int % 10;
                version = RobotVersion(major, minor);
                robot_version_ = robot_version_int;
            }
            
            const char* robot_version_env = std::getenv("ROBOT_VERSION");
            if (robot_version_env) {
                std::cout << "从环境变量获取ROBOT_VERSION: " << robot_version_env << std::endl;
                robot_version_int = std::stoi(robot_version_env);
                int major = robot_version_int / 10;
                int minor = robot_version_int % 10;
                version = RobotVersion(major, minor);
                robot_version_ = robot_version_int;
            }
            
            std::cout << "最终使用的robot_version: " << robot_version_ << std::endl;
            
            std::string kuavo_assets_path;
            if (!nh_.getParam("/repo_root_path", kuavo_assets_path)) {
                std::cerr << "无法获取/repo_root_path参数，无法继续" << std::endl;
                return;
            }
            kuavo_assets_path += "/src/kuavo_assets";
            
            auto kuavo_common_ptr = KuavoCommon::getInstancePtr(version, kuavo_assets_path);
            auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
            
            motor_info_ = kuavo_settings.hardware_settings;
            joint_ids_ = motor_info_.joint_ids;
            
            // 保存默认kp/kd值（用于EC_MASTER电机位置控制）
            // 注意：joint_kp和joint_kd是int类型，需要转换为double
            default_joint_kp_.clear();
            default_joint_kd_.clear();
            for (const auto& kp : kuavo_settings.running_settings.joint_kp) {
                default_joint_kp_.push_back(static_cast<double>(kp));
            }
            for (const auto& kd : kuavo_settings.running_settings.joint_kd) {
                default_joint_kd_.push_back(static_cast<double>(kd));
            }
            
            std::cout << "硬件配置加载完成" << std::endl;
            std::cout << "  关节ID数量: " << joint_ids_.size() << std::endl;
            
            // 初始化HardwarePlant实例
            std::cout << "开始初始化HardwarePlant实例..." << std::endl;
            
            HighlyDynamic::HardwareParam hw_param;
            hw_param.robot_version = RobotVersion(robot_version_ / 10, robot_version_ % 10);
            hw_param.cali = false;
            hw_param.cali_arm = false;
            hw_param.cali_leg = false;
            
            hardware_plant_ = std::make_unique<HighlyDynamic::HardwarePlant>(
                1e-3,  // dt
                hw_param,
                hardware_plant_path_,
                MOTOR_CONTROL_MODE_TORQUE,
                0,     // num_actuated
                7,     // nq_f
                6      // nv_f
            );
            
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
            joint_ids_.resize(num_joint_);
        }
    }
    
    void testPositionControl() {
        std::cout << "\n=== 位置控制验证测试 ===" << std::endl;
        
        // 创建CSV文件
        std::string filename = createCSVFile("position");
        
        // 生成测试轨迹（网格采样）
        int grid_size = 10;
        std::vector<std::pair<double, double>> test_points;
        
        for (int i = 0; i < grid_size; ++i) {
            double pitch = PITCH_MIN + (PITCH_MAX - PITCH_MIN) * i / (grid_size - 1);
            for (int j = 0; j < grid_size; ++j) {
                double roll = ROLL_MIN + (ROLL_MAX - ROLL_MIN) * j / (grid_size - 1);
                test_points.push_back({pitch, roll});
            }
        }
        
        std::cout << "生成了 " << test_points.size() << " 个测试点" << std::endl;
        
        double dt = 0.002;
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        double controlPeriod = dt;
        long long controlPeriodNs = static_cast<long long>(controlPeriod * 1e9);
        
        // 对每个测试点进行测试
        for (size_t point_idx = 0; point_idx < test_points.size() && ros::ok() && !should_stop_; ++point_idx) {
            ros::spinOnce(); // 处理ROS回调，包括信号
            
            if (!ros::ok() || should_stop_) {
                std::cout << "检测到中断标志，停止测试..." << std::endl;
                break;
            }
            
            auto& point = test_points[point_idx];
            double pitch_cmd = point.first;
            double roll_cmd = point.second;
            
            std::cout << "测试点 " << (point_idx + 1) << "/" << test_points.size() 
                      << ": pitch=" << pitch_cmd << ", roll=" << roll_cmd << std::endl;
            
            // 根据测试模式构建命令
            std::vector<std::pair<Eigen::VectorXd, std::string>> test_configs;
            
            if (ankle_test_foot_ == 0) {  // both_same
                Eigen::VectorXd q_cmd(12);
                q_cmd.setZero();
                q_cmd[LEFT_PITCH_IDX] = pitch_cmd;
                q_cmd[LEFT_ROLL_IDX] = roll_cmd;
                q_cmd[RIGHT_PITCH_IDX] = pitch_cmd;
                q_cmd[RIGHT_ROLL_IDX] = roll_cmd;
                test_configs.push_back({q_cmd, "both"});
            } else if (ankle_test_foot_ == 1) {  // left_only
                Eigen::VectorXd q_cmd(12);
                q_cmd.setZero();
                q_cmd[LEFT_PITCH_IDX] = pitch_cmd;
                q_cmd[LEFT_ROLL_IDX] = roll_cmd;
                test_configs.push_back({q_cmd, "left"});
            } else if (ankle_test_foot_ == 2) {  // right_only
                Eigen::VectorXd q_cmd(12);
                q_cmd.setZero();
                q_cmd[RIGHT_PITCH_IDX] = pitch_cmd;
                q_cmd[RIGHT_ROLL_IDX] = roll_cmd;
                test_configs.push_back({q_cmd, "right"});
            } else if (ankle_test_foot_ == 3) {  // both_separate
                Eigen::VectorXd q_cmd_l(12);
                q_cmd_l.setZero();
                q_cmd_l[LEFT_PITCH_IDX] = pitch_cmd;
                q_cmd_l[LEFT_ROLL_IDX] = roll_cmd;
                test_configs.push_back({q_cmd_l, "left"});
                
                Eigen::VectorXd q_cmd_r(12);
                q_cmd_r.setZero();
                q_cmd_r[RIGHT_PITCH_IDX] = pitch_cmd;
                q_cmd_r[RIGHT_ROLL_IDX] = roll_cmd;
                test_configs.push_back({q_cmd_r, "right"});
            }
            
            // 对每个配置进行测试
            for (auto& config : test_configs) {
                ros::spinOnce(); // 处理ROS回调
                
                if (!ros::ok() || should_stop_) {
                    std::cout << "检测到中断标志，停止当前配置测试..." << std::endl;
                    break;
                }
                
                Eigen::VectorXd& q_cmd = config.first;
                std::string foot_type = config.second;
                
                // 转换为motor命令
                Eigen::VectorXd p_cmd = ankle_solver_.joint_to_motor_position(q_cmd);
                
                // 发送motor位置命令（CST模式）
                sendMotorPositionCommand(p_cmd);
                
                // 等待稳定期间持续发送命令，支持中断
                // 使用ros::ok()检查，确保ROS信号可以中断
                struct timespec wait_start_time;
                clock_gettime(CLOCK_MONOTONIC, &wait_start_time);
                wait_start_time.tv_sec += 2; // 等待2秒
                
                int wait_count = 0;
                while (ros::ok() && !should_stop_) {
                    ros::spinOnce(); // 处理ROS回调，包括信号处理
                    
                    if (!ros::ok() || should_stop_) {
                        std::cout << "检测到中断，停止等待..." << std::endl;
                        break;
                    }
                    
                    struct timespec current_time;
                    clock_gettime(CLOCK_MONOTONIC, &current_time);
                    if (current_time.tv_sec >= wait_start_time.tv_sec) {
                        break; // 等待时间到
                    }
                    
                    // 持续发送命令保持电机使能
                    sendMotorPositionCommand(p_cmd);
                    
                    // 使用短时间等待，确保能及时响应中断
                    usleep(2000); // 2ms
                    wait_count++;
                }
            
                if (should_stop_) break;
                
                // 读取实际motor位置反馈
                SensorData_t sensor_data_motor;
                if (!hardware_plant_->readSensor(sensor_data_motor)) {
                    std::cerr << "读取传感器数据失败" << std::endl;
                    continue;
                }
                
                // 转换为Eigen向量
                Eigen::VectorXd p_feedback(12);
                for (int i = 0; i < 12; ++i) {
                    p_feedback[i] = sensor_data_motor.joint_q[i];
                }
                
                // 转换回joint位置
                Eigen::VectorXd q_feedback = ankle_solver_.motor_to_joint_position(p_feedback);
                // 由指令电机角正解得到的关节角，用于解算误差分解：解算往返误差 = q_cmd - q_from_cmd，电机传播误差 = q_from_cmd - q_fb
                Eigen::VectorXd q_from_cmd = ankle_solver_.motor_to_joint_position(p_cmd);
                
                // 计算误差（总误差 = 解算往返误差 + 电机传播误差）
                Eigen::VectorXd error(4);
                error[0] = q_cmd[LEFT_PITCH_IDX] - q_feedback[LEFT_PITCH_IDX];
                error[1] = q_cmd[LEFT_ROLL_IDX] - q_feedback[LEFT_ROLL_IDX];
                error[2] = q_cmd[RIGHT_PITCH_IDX] - q_feedback[RIGHT_PITCH_IDX];
                error[3] = q_cmd[RIGHT_ROLL_IDX] - q_feedback[RIGHT_ROLL_IDX];
                
                // 记录数据（含 q_from_cmd 用于解算误差分析）
                recordPositionData(csv_file_, point_idx, pitch_cmd, roll_cmd,
                                 p_cmd, p_feedback, q_from_cmd, q_feedback, error, foot_type);
            }
            
            if (!ros::ok() || should_stop_) {
                std::cout << "检测到中断标志，停止测试..." << std::endl;
                break;
            }
        }
        
        csv_file_.close();
        if (!ros::ok() || should_stop_) {
            std::cout << "位置控制测试被中断，数据已保存到: " << filename << std::endl;
        } else {
            std::cout << "位置控制测试完成，数据已保存到: " << filename << std::endl;
        }
    }
    
    void testVelocityControl() {
        std::cout << "\n=== 速度控制验证测试 ===" << std::endl;
        
        // 创建CSV文件
        std::string filename = createCSVFile("velocity");
        
        // 生成测试轨迹（正弦波）
        double test_duration = 10.0;  // 10秒
        double dt = 0.002;
        int num_steps = static_cast<int>(test_duration / dt);
        
        std::cout << "生成正弦波测试轨迹，持续时间: " << test_duration << " 秒" << std::endl;
        
        double t = 0.0;
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        double controlPeriod = dt;
        long long controlPeriodNs = static_cast<long long>(controlPeriod * 1e9);
        
        for (int step = 0; step < num_steps && ros::ok() && !should_stop_; ++step) {
            ros::spinOnce(); // 处理ROS回调，包括信号
            
            if (!ros::ok() || should_stop_) break;
            
            // 生成正弦波位置和速度
            double pitch_cmd = 0.1 * std::sin(3.14 * t);
            double roll_cmd = 0.05 * std::sin(3.14 * t);
            double pitch_vel_cmd = 0.1  * std::cos(3.14 * t);
            double roll_vel_cmd = 0.05  * std::cos(3.14 * t);
            
            // 限制在工作空间内
            pitch_cmd = std::max(PITCH_MIN, std::min(PITCH_MAX, pitch_cmd));
            roll_cmd = std::max(ROLL_MIN, std::min(ROLL_MAX, roll_cmd));
            
            // 构建12维joint命令
            Eigen::VectorXd q_cmd(12);
            Eigen::VectorXd dq_cmd(12);
            q_cmd.setZero();
            dq_cmd.setZero();
            q_cmd[LEFT_PITCH_IDX] = pitch_cmd;
            q_cmd[LEFT_ROLL_IDX] = roll_cmd;
            q_cmd[RIGHT_PITCH_IDX] = pitch_cmd;
            q_cmd[RIGHT_ROLL_IDX] = roll_cmd;
            dq_cmd[LEFT_PITCH_IDX] = pitch_vel_cmd;
            dq_cmd[LEFT_ROLL_IDX] = roll_vel_cmd;
            dq_cmd[RIGHT_PITCH_IDX] = pitch_vel_cmd;
            dq_cmd[RIGHT_ROLL_IDX] = roll_vel_cmd;
            
            // 读取当前motor位置
            SensorData_t sensor_data_motor;
            if (!hardware_plant_->readSensor(sensor_data_motor)) {
                std::cerr << "读取传感器数据失败" << std::endl;
                continue;
            }
            
            Eigen::VectorXd p_current(12);
            for (int i = 0; i < 12; ++i) {
                p_current[i] = sensor_data_motor.joint_q[i];
            }
            
            // 转换为motor速度命令
            Eigen::VectorXd dp_cmd = ankle_solver_.joint_to_motor_velocity(q_cmd, p_current, dq_cmd);
            
            // 发送motor速度命令（CST模式）
            sendMotorVelocityCommand(dp_cmd);
            
            // 等待到下一个周期，支持中断
            if (should_stop_) break;
            
            next_time.tv_nsec += controlPeriodNs;
            while (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }
            
            // 使用可中断的等待方式
            struct timespec current_time;
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            while (current_time.tv_sec < next_time.tv_sec || 
                   (current_time.tv_sec == next_time.tv_sec && current_time.tv_nsec < next_time.tv_nsec)) {
                if (should_stop_) break;
                usleep(100); // 100us检查一次
                clock_gettime(CLOCK_MONOTONIC, &current_time);
            }
            
            // 读取实际motor速度反馈
            if (!hardware_plant_->readSensor(sensor_data_motor)) {
                continue;
            }
            
            Eigen::VectorXd p_feedback(12);
            Eigen::VectorXd dp_feedback(12);
            for (int i = 0; i < 12; ++i) {
                p_feedback[i] = sensor_data_motor.joint_q[i];
                dp_feedback[i] = sensor_data_motor.joint_v[i];
            }
            
            // 转换回joint速度
            Eigen::VectorXd q_feedback = ankle_solver_.motor_to_joint_position(p_feedback);
            Eigen::VectorXd dq_feedback = ankle_solver_.motor_to_joint_velocity(q_feedback, p_feedback, dp_feedback);
            // 由指令电机速度正解得到的关节速度，用于解算误差分解
            Eigen::VectorXd dq_from_cmd = ankle_solver_.motor_to_joint_velocity(q_cmd, p_current, dp_cmd);
            
            // 计算误差（总误差 = 解算往返误差 + 电机传播误差）
            Eigen::VectorXd vel_error(4);
            vel_error[0] = dq_cmd[LEFT_PITCH_IDX] - dq_feedback[LEFT_PITCH_IDX];
            vel_error[1] = dq_cmd[LEFT_ROLL_IDX] - dq_feedback[LEFT_ROLL_IDX];
            vel_error[2] = dq_cmd[RIGHT_PITCH_IDX] - dq_feedback[RIGHT_PITCH_IDX];
            vel_error[3] = dq_cmd[RIGHT_ROLL_IDX] - dq_feedback[RIGHT_ROLL_IDX];
            
            // 记录数据（每25步记录一次，与力矩一致）
            if (step % 25 == 0) {
                std::string foot_type = (ankle_test_foot_ == 0) ? "both" : 
                                       (ankle_test_foot_ == 1) ? "left" : 
                                       (ankle_test_foot_ == 2) ? "right" : "both";
                recordVelocityData(csv_file_, step, t, pitch_cmd, roll_cmd,
                                 pitch_vel_cmd, roll_vel_cmd,
                                 dp_cmd, dp_feedback, dq_from_cmd, dq_feedback, vel_error, foot_type);
            }
            
            t += dt;
            if (!ros::ok() || should_stop_) break;
        }
        
        csv_file_.close();
        if (!ros::ok() || should_stop_) {
            std::cout << "速度控制测试被中断，数据已保存到: " << filename << std::endl;
        } else {
            std::cout << "速度控制测试完成，数据已保存到: " << filename << std::endl;
        }
    }
    
    void testTorqueControl() {
        std::cout << "\n=== 力矩控制验证测试 ===" << std::endl;
        
        // 创建CSV文件
        std::string filename = createCSVFile("torque");
        
        // 生成测试轨迹（正弦波力矩）
        double test_duration = 10.0;  // 10秒
        double dt = 0.002;
        int num_steps = static_cast<int>(test_duration / dt);
        
        std::cout << "生成正弦波力矩测试轨迹，持续时间: " << test_duration << " 秒" << std::endl;
        
        double t = 0.0;
        struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC, &next_time);
        double controlPeriod = dt;
        long long controlPeriodNs = static_cast<long long>(controlPeriod * 1e9);
        
        for (int step = 0; step < num_steps && ros::ok() && !should_stop_; ++step) {
            ros::spinOnce(); // 处理ROS回调，包括信号
            
            if (!ros::ok() || should_stop_) break;
            
            // 生成正弦波位置和力矩
            // 生成正弦波位置和速度
            double pitch_cmd = 0.0 * std::sin(3.14 * t);
            double roll_cmd = 0.0 * std::sin(3.14 * t);
            double pitch_tau_cmd = 0.2 * std::cos(3.14 * t);
            double roll_tau_cmd = 0.2 * std::cos(3.14 * t);
            
            // 限制在工作空间内
            pitch_cmd = std::max(PITCH_MIN, std::min(PITCH_MAX, pitch_cmd));
            roll_cmd = std::max(ROLL_MIN, std::min(ROLL_MAX, roll_cmd));
            
            // 构建12维joint命令
            Eigen::VectorXd q_cmd(12);
            Eigen::VectorXd tau_cmd(12);
            q_cmd.setZero();
            tau_cmd.setZero();
            q_cmd[LEFT_PITCH_IDX] = pitch_cmd;
            q_cmd[LEFT_ROLL_IDX] = roll_cmd;
            q_cmd[RIGHT_PITCH_IDX] = pitch_cmd;
            q_cmd[RIGHT_ROLL_IDX] = roll_cmd;
            tau_cmd[LEFT_PITCH_IDX] = pitch_tau_cmd;
            tau_cmd[LEFT_ROLL_IDX] = roll_tau_cmd;
            tau_cmd[RIGHT_PITCH_IDX] = pitch_tau_cmd;
            tau_cmd[RIGHT_ROLL_IDX] = roll_tau_cmd;
            
            // 读取当前motor位置
            SensorData_t sensor_data_motor;
            if (!hardware_plant_->readSensor(sensor_data_motor)) {
                std::cerr << "读取传感器数据失败" << std::endl;
                continue;
            }
            
            Eigen::VectorXd p_current(12);
            for (int i = 0; i < 12; ++i) {
                p_current[i] = sensor_data_motor.joint_q[i];
            }
            
            // 转换为motor电流命令
            Eigen::VectorXd i_cmd = ankle_solver_.joint_to_motor_current(q_cmd, p_current, tau_cmd);
            
            // 发送motor力矩命令（CST模式）
            sendMotorTorqueCommand(i_cmd);
            
            // 等待到下一个周期，使用ros::ok()检查
            if (!ros::ok() || should_stop_) break;
            
            next_time.tv_nsec += controlPeriodNs;
            while (next_time.tv_nsec >= 1000000000) {
                next_time.tv_nsec -= 1000000000;
                next_time.tv_sec++;
            }
            
            // 等待到下一个周期（ROS会自动处理中断）
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
            
            // 读取实际motor电流反馈
            if (!hardware_plant_->readSensor(sensor_data_motor)) {
                continue;
            }
            
            Eigen::VectorXd p_feedback(12);
            Eigen::VectorXd tau_feedback_motor(12);
            
            // 获取motor电流值（12维，仅腿部）
            Eigen::VectorXd motor_current(12);
            for (int i = 0; i < 12; ++i) {
                p_feedback[i] = sensor_data_motor.joint_q[i];
                motor_current[i] = sensor_data_motor.joint_current[i];
            }
            
            // GetC2Tcoeff 内部按 motor_cul.size()（全关节数 num_joint_）循环访问 motor_cur[i]，
            // 必须传入全尺寸电流向量，否则 i>=12 会越界导致段错误
            Eigen::VectorXd motor_current_full(num_joint_);
            for (uint32_t i = 0; i < num_joint_; ++i) {
                motor_current_full[i] = (i < 12) ? sensor_data_motor.joint_current[i] : 0.0;
            }
            Eigen::VectorXd c2t_coeff = hardware_plant_->GetC2Tcoeff(motor_current_full);
            
            // 将电流值乘以c2t系数得到力矩值（N·m），仅取前12路
            for (int i = 0; i < 12; ++i) {
                tau_feedback_motor[i] = motor_current[i] * c2t_coeff[i];
            }
            
            // 转换回joint力矩
            Eigen::VectorXd q_feedback = ankle_solver_.motor_to_joint_position(p_feedback);
            Eigen::VectorXd tau_feedback = ankle_solver_.motor_to_joint_torque(q_feedback, p_feedback, tau_feedback_motor);
            
            // 计算误差
            Eigen::VectorXd tau_error(4);
            tau_error[0] = tau_cmd[LEFT_PITCH_IDX] - tau_feedback[LEFT_PITCH_IDX];
            tau_error[1] = tau_cmd[LEFT_ROLL_IDX] - tau_feedback[LEFT_ROLL_IDX];
            tau_error[2] = tau_cmd[RIGHT_PITCH_IDX] - tau_feedback[RIGHT_PITCH_IDX];
            tau_error[3] = tau_cmd[RIGHT_ROLL_IDX] - tau_feedback[RIGHT_ROLL_IDX];
            
            // 记录数据（每25步记录一次，10s/0.002s=5000步 -> 200个点）
            if (step % 25 == 0) {
                // i_cmd是电流命令，motor_current是电流反馈
                std::string foot_type = (ankle_test_foot_ == 0) ? "both" : 
                                       (ankle_test_foot_ == 1) ? "left" : 
                                       (ankle_test_foot_ == 2) ? "right" : "both";
                recordTorqueData(csv_file_, step, t, pitch_cmd, roll_cmd, 
                                pitch_tau_cmd, roll_tau_cmd,
                                i_cmd, motor_current, tau_feedback_motor, tau_feedback, tau_error, foot_type);
            }
            
            t += dt;
            if (!ros::ok() || should_stop_) break;
        }
        
        csv_file_.close();
        if (!ros::ok() || should_stop_) {
            std::cout << "力矩控制测试被中断，数据已保存到: " << filename << std::endl;
        } else {
            std::cout << "力矩控制测试完成，数据已保存到: " << filename << std::endl;
        }
    }
    
    void sendMotorPositionCommand(const Eigen::VectorXd& p_cmd) {
        // 使用writeCommand方法，构建cmd_r向量
        // cmd_r格式: [位置(na_r), 速度(na_r), 力矩(na_r), 最大力矩(na_r), tau_ratio(na_r)]
        Eigen::VectorXd cmd_r(num_joint_ * 5);
        std::vector<int> control_modes(num_joint_, MOTOR_CONTROL_MODE_POSITION);
        Eigen::VectorXd joint_kp(num_joint_);
        Eigen::VectorXd joint_kd(num_joint_);
        
        // 计算EC_MASTER电机的索引映射（用于获取默认kp/kd）
        int ec_master_count = 0;
        
        for (uint32_t i = 0; i < num_joint_; i++) {
            if (i < 12) {  // 只控制腿部关节
                cmd_r[i] = p_cmd[i];  // 位置 (rad)
                cmd_r[num_joint_ + i] = 0.0;  // 速度 (rad/s)
                cmd_r[num_joint_ * 2 + i] = 0.0;  // 力矩 (N·m)
                cmd_r[num_joint_ * 3 + i] = 10.0;  // 最大力矩 (N·m)
                cmd_r[num_joint_ * 4 + i] = 0.0;  // tau_ratio
            } else {
                // 非腿部关节保持当前位置
                cmd_r[i] = 0.0;
                cmd_r[num_joint_ + i] = 0.0;
                cmd_r[num_joint_ * 2 + i] = 0.0;
                cmd_r[num_joint_ * 3 + i] = 10.0;
                cmd_r[num_joint_ * 4 + i] = 0.0;
                control_modes[i] = MOTOR_CONTROL_MODE_POSITION;
            }
            
            // 为EC_MASTER电机设置默认kp/kd（与jointMoveTo保持一致）
            if (i < motor_info_.driver.size() && 
                motor_info_.driver[i] == EC_MASTER && 
                motor_info_.motors_exist[i]) {
                // 使用EC_MASTER索引获取默认kp/kd
                if (ec_master_count < static_cast<int>(default_joint_kp_.size()) &&
                    ec_master_count < static_cast<int>(default_joint_kd_.size())) {
                    joint_kp[i] = default_joint_kp_[ec_master_count];
                    joint_kd[i] = default_joint_kd_[ec_master_count];
                } else {
                    joint_kp[i] = 0.0;
                    joint_kd[i] = 0.0;
                }
                ec_master_count++;
            } else {
                joint_kp[i] = 0.0;
                joint_kd[i] = 0.0;
            }
        }
        
        hardware_plant_->writeCommand(cmd_r, num_joint_, control_modes, joint_kp, joint_kd);
    }
    
    void sendMotorVelocityCommand(const Eigen::VectorXd& dp_cmd) {
        // 使用writeCommand方法，构建cmd_r向量
        // cmd_r格式: [位置(na_r), 速度(na_r), 力矩(na_r), 最大力矩(na_r), tau_ratio(na_r)]
        Eigen::VectorXd cmd_r(num_joint_ * 5);
        std::vector<int> control_modes(num_joint_, MOTOR_CONTROL_MODE_VELOCITY);
        Eigen::VectorXd joint_kp(num_joint_);
        Eigen::VectorXd joint_kd(num_joint_);
        
        // 读取当前位置作为反馈位置
        SensorData_t sensor_data_motor;
        if (!hardware_plant_->readSensor(sensor_data_motor)) {
            return;
        }
        
        for (uint32_t i = 0; i < num_joint_; i++) {
            if (i < 12) {  // 只控制腿部关节
                cmd_r[i] = sensor_data_motor.joint_q[i];  // 当前位置 (rad)
                cmd_r[num_joint_ + i] = dp_cmd[i];  // 速度 (rad/s)
                cmd_r[num_joint_ * 2 + i] = 0.0;  // 力矩 (N·m)
                cmd_r[num_joint_ * 3 + i] = 10.0;  // 最大力矩 (N·m)
                cmd_r[num_joint_ * 4 + i] = 0.0;  // tau_ratio
            } else {
                // 非腿部关节保持当前位置
                cmd_r[i] = sensor_data_motor.joint_q[i];
                cmd_r[num_joint_ + i] = 0.0;
                cmd_r[num_joint_ * 2 + i] = 0.0;
                cmd_r[num_joint_ * 3 + i] = 10.0;
                cmd_r[num_joint_ * 4 + i] = 0.0;
                control_modes[i] = MOTOR_CONTROL_MODE_POSITION;
            }
            joint_kp[i] = 0.0;
            joint_kd[i] = 0.0;
        }
        
        hardware_plant_->writeCommand(cmd_r, num_joint_, control_modes, joint_kp, joint_kd);
    }
    
    void sendMotorTorqueCommand(const Eigen::VectorXd& i_cmd) {
        // 使用writeCommand方法，构建cmd_r向量
        // cmd_r格式: [位置(na_r), 速度(na_r), 力矩(na_r), 最大力矩(na_r), tau_ratio(na_r)]
        // 注意：i_cmd是电流值(A)，需要转换为力矩值(N·m)
        Eigen::VectorXd cmd_r(num_joint_ * 5);
        std::vector<int> control_modes(num_joint_, MOTOR_CONTROL_MODE_TORQUE);
        Eigen::VectorXd joint_kp(num_joint_);
        Eigen::VectorXd joint_kd(num_joint_);
        
        // 读取当前位置和速度作为反馈
        SensorData_t sensor_data_motor;
        if (!hardware_plant_->readSensor(sensor_data_motor)) {
            return;
        }
        
        // 构建完整的电流向量（包含所有关节，非腿部关节为0）
        Eigen::VectorXd motor_current_full(num_joint_);
        for (uint32_t i = 0; i < num_joint_; i++) {
            if (i < 12) {
                motor_current_full[i] = i_cmd[i];
            } else {
                motor_current_full[i] = 0.0;
            }
        }
        
        // 获取c2t系数（根据电流值动态查找）
        Eigen::VectorXd c2t_coeff = hardware_plant_->GetC2Tcoeff(motor_current_full);
        
        for (uint32_t i = 0; i < num_joint_; i++) {
            if (i < 12) {  // 只控制腿部关节
                cmd_r[i] = sensor_data_motor.joint_q[i];  // 当前位置 (rad)
                cmd_r[num_joint_ + i] = sensor_data_motor.joint_v[i];  // 当前速度 (rad/s)
                // 将电流值转换为力矩值 (N·m)
                cmd_r[num_joint_ * 2 + i] = i_cmd[i] * c2t_coeff[i];  // 力矩 (N·m)
                cmd_r[num_joint_ * 3 + i] = 10.0;  // 最大力矩 (N·m)
                cmd_r[num_joint_ * 4 + i] = 0.0;  // tau_ratio
            } else {
                // 非腿部关节保持当前位置
                cmd_r[i] = sensor_data_motor.joint_q[i];
                cmd_r[num_joint_ + i] = sensor_data_motor.joint_v[i];
                cmd_r[num_joint_ * 2 + i] = 0.0;
                cmd_r[num_joint_ * 3 + i] = 10.0;
                cmd_r[num_joint_ * 4 + i] = 0.0;
                control_modes[i] = MOTOR_CONTROL_MODE_POSITION;
            }
            joint_kp[i] = 0.0;
            joint_kd[i] = 0.0;
        }
        
        hardware_plant_->writeCommand(cmd_r, num_joint_, control_modes, joint_kp, joint_kd);
    }
    
    std::string createCSVFile(const std::string& test_type) {
        // 获取时间戳
        auto now = std::time(nullptr);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();
        
        // 创建文件路径
        const std::string hardware_node_path = ros::package::getPath("hardware_node");
        const std::string data_dir = hardware_node_path + "/src/tests/ankleTest/data";
        std::filesystem::create_directories(data_dir);
        
        std::string filename = data_dir + "/ankle_solver_v17_test_" + test_type + "_" + timestamp + ".csv";
        csv_file_.open(filename);
        
        // 写入CSV头部
        if (test_type == "position") {
            csv_file_ << "point_idx,time,foot,pitch_cmd,roll_cmd,"
                      << "p_cmd_4,p_cmd_5,p_cmd_10,p_cmd_11,"
                      << "p_fb_4,p_fb_5,p_fb_10,p_fb_11,"
                      << "q_from_cmd_pitch_l,q_from_cmd_roll_l,q_from_cmd_pitch_r,q_from_cmd_roll_r,"
                      << "q_fb_pitch_l,q_fb_roll_l,q_fb_pitch_r,q_fb_roll_r,"
                      << "error_pitch_l,error_roll_l,error_pitch_r,error_roll_r\n";
        } else if (test_type == "velocity") {
            csv_file_ << "step,time,foot,pitch_cmd,roll_cmd,pitch_vel_cmd,roll_vel_cmd,"
                      << "dp_cmd_4,dp_cmd_5,dp_cmd_10,dp_cmd_11,"
                      << "dp_fb_4,dp_fb_5,dp_fb_10,dp_fb_11,"
                      << "dq_from_cmd_pitch_l,dq_from_cmd_roll_l,dq_from_cmd_pitch_r,dq_from_cmd_roll_r,"
                      << "dq_fb_pitch_l,dq_fb_roll_l,dq_fb_pitch_r,dq_fb_roll_r,"
                      << "error_pitch_l,error_roll_l,error_pitch_r,error_roll_r\n";
        } else if (test_type == "torque") {
            csv_file_ << "step,time,foot,pitch_cmd,roll_cmd,pitch_tau_cmd,roll_tau_cmd,"
                      << "i_cmd_4,i_cmd_5,i_cmd_10,i_cmd_11,"
                      << "i_fb_4,i_fb_5,i_fb_10,i_fb_11,"
                      << "tau_motor_fb_4,tau_motor_fb_5,tau_motor_fb_10,tau_motor_fb_11,"
                      << "tau_fb_pitch_l,tau_fb_roll_l,tau_fb_pitch_r,tau_fb_roll_r,"
                      << "error_pitch_l,error_roll_l,error_pitch_r,error_roll_r\n";
        }
        
        return filename;
    }
    
    void recordPositionData(std::ofstream& file, int point_idx,
                          double pitch_cmd, double roll_cmd,
                          const Eigen::VectorXd& p_cmd, const Eigen::VectorXd& p_fb,
                          const Eigen::VectorXd& q_from_cmd, const Eigen::VectorXd& q_fb,
                          const Eigen::VectorXd& error, const std::string& foot_type = "both") {
        file << point_idx << ","
             << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count() << ","
             << foot_type << ","
             << pitch_cmd << "," << roll_cmd << ","
             << p_cmd[4] << "," << p_cmd[5] << "," << p_cmd[10] << "," << p_cmd[11] << ","
             << p_fb[4] << "," << p_fb[5] << "," << p_fb[10] << "," << p_fb[11] << ","
             << q_from_cmd[LEFT_PITCH_IDX] << "," << q_from_cmd[LEFT_ROLL_IDX] << ","
             << q_from_cmd[RIGHT_PITCH_IDX] << "," << q_from_cmd[RIGHT_ROLL_IDX] << ","
             << q_fb[LEFT_PITCH_IDX] << "," << q_fb[LEFT_ROLL_IDX] << ","
             << q_fb[RIGHT_PITCH_IDX] << "," << q_fb[RIGHT_ROLL_IDX] << ","
             << error[0] << "," << error[1] << "," << error[2] << "," << error[3] << "\n";
    }
    
    void recordVelocityData(std::ofstream& file, int step, double t,
                           double pitch_cmd, double roll_cmd,
                           double pitch_vel_cmd, double roll_vel_cmd,
                           const Eigen::VectorXd& dp_cmd, const Eigen::VectorXd& dp_fb,
                           const Eigen::VectorXd& dq_from_cmd, const Eigen::VectorXd& dq_fb,
                           const Eigen::VectorXd& error, const std::string& foot_type = "both") {
        file << step << "," << t << ","
             << foot_type << ","
             << pitch_cmd << "," << roll_cmd << ","
             << pitch_vel_cmd << "," << roll_vel_cmd << ","
             << dp_cmd[4] << "," << dp_cmd[5] << "," << dp_cmd[10] << "," << dp_cmd[11] << ","
             << dp_fb[4] << "," << dp_fb[5] << "," << dp_fb[10] << "," << dp_fb[11] << ","
             << dq_from_cmd[LEFT_PITCH_IDX] << "," << dq_from_cmd[LEFT_ROLL_IDX] << ","
             << dq_from_cmd[RIGHT_PITCH_IDX] << "," << dq_from_cmd[RIGHT_ROLL_IDX] << ","
             << dq_fb[LEFT_PITCH_IDX] << "," << dq_fb[LEFT_ROLL_IDX] << ","
             << dq_fb[RIGHT_PITCH_IDX] << "," << dq_fb[RIGHT_ROLL_IDX] << ","
             << error[0] << "," << error[1] << "," << error[2] << "," << error[3] << "\n";
    }
    
    void recordTorqueData(std::ofstream& file, int step, double t,
                         double pitch_cmd, double roll_cmd,
                         double pitch_tau_cmd, double roll_tau_cmd,
                         const Eigen::VectorXd& i_cmd, const Eigen::VectorXd& i_fb,
                         const Eigen::VectorXd& tau_fb_motor,
                         const Eigen::VectorXd& tau_fb, const Eigen::VectorXd& error,
                         const std::string& foot_type = "both") {
        file << step << "," << t << ","
             << foot_type << ","
             << pitch_cmd << "," << roll_cmd << ","
             << pitch_tau_cmd << "," << roll_tau_cmd << ","
             << i_cmd[4] << "," << i_cmd[5] << "," << i_cmd[10] << "," << i_cmd[11] << ","
             << i_fb[4] << "," << i_fb[5] << "," << i_fb[10] << "," << i_fb[11] << ","
             << tau_fb_motor[4] << "," << tau_fb_motor[5] << "," << tau_fb_motor[10] << "," << tau_fb_motor[11] << ","
             << tau_fb[LEFT_PITCH_IDX] << "," << tau_fb[LEFT_ROLL_IDX] << ","
             << tau_fb[RIGHT_PITCH_IDX] << "," << tau_fb[RIGHT_ROLL_IDX] << ","
             << error[0] << "," << error[1] << "," << error[2] << "," << error[3] << "\n";
    }
    
    void emergencyStop() {
        std::cout << "执行紧急停止..." << std::endl;
        if (hardware_plant_) {
            hardware_plant_.reset();
            std::cout << "HardwarePlant已析构，紧急停止完成" << std::endl;
        }
    }
    
    void runAnalysisScript() {
        std::cout << "\n========================================" << std::endl;
        std::cout << "开始运行分析脚本..." << std::endl;
        std::cout << "========================================\n" << std::endl;
        
        // 获取脚本路径
        const std::string hardware_node_path = ros::package::getPath("hardware_node");
        const std::string script_path = hardware_node_path + "/src/tests/ankleTest/ankle_solver_analysis.py";
        const std::string data_dir = hardware_node_path + "/src/tests/ankleTest/data";
        
        // 检查脚本是否存在
        if (!std::filesystem::exists(script_path)) {
            std::cerr << "警告：分析脚本不存在: " << script_path << std::endl;
            return;
        }
        
        // 检查数据目录是否存在
        if (!std::filesystem::exists(data_dir)) {
            std::cerr << "警告：测试数据目录不存在: " << data_dir << std::endl;
            return;
        }
        
        // 构建Python命令
        std::string command = "python3 " + script_path + " " + data_dir;
        
        std::cout << "执行命令: " << command << std::endl;
        
        // 执行Python脚本
        int result = system(command.c_str());
        
        if (result == 0) {
            std::cout << "\n========================================" << std::endl;
            std::cout << "✅ 分析完成！" << std::endl;
            std::cout << "========================================\n" << std::endl;
        } else {
            std::cerr << "\n========================================" << std::endl;
            std::cerr << "⚠️  分析脚本执行失败，返回码: " << result << std::endl;
            std::cerr << "========================================\n" << std::endl;
        }
    }
};

// 静态指针定义（必须在类定义之后）
AnkleSolverV17Test* AnkleSolverV17Test::instance_ptr_ = nullptr;

int main(int argc, char** argv) {
    ros::init(argc, argv, "ankle_solver_v17_test");
    
    // 注册信号处理函数
    signal(SIGINT, AnkleSolverV17Test::signalHandler);
    signal(SIGTERM, AnkleSolverV17Test::signalHandler);
    
    try {
        AnkleSolverV17Test test;
        test.run();
    } catch (const std::exception& e) {
        std::cout << "AnkleSolverV17Test 异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
