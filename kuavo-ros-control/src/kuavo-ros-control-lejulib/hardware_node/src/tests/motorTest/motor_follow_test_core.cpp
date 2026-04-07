#include "motor_follow_test_core.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <jsoncpp/json/json.h>

namespace motor_follow_test {

MotorFollowTestCore::MotorFollowTestCore() 
    : config_()
    , num_joint_(0)
    , num_waist_joints_(0)
    , num_arm_joints_(0)
    , num_head_joints_(0)
    , na_foot_(0)
    , joint_indices_()
    , index_(0)
    , test_running_(false)
    , test_complete_(false)
    , test_start_time_(0.0)
    , test_progress_(0.0)
    , time_offset_(0.0)
    , cycleCountL_(0)
    , cycleCountR_(0)
    , prevSinL_(MAX_VALUE)
    , prevSinR_(MAX_VALUE)
    , sinNum_(0)
    , data_save_path_("./test_data")
{
    // 初始化数据记录缓冲区
    l_txBuffer_.clear();
    r_txBuffer_.clear();
    l_rxBuffer_.clear();
    r_rxBuffer_.clear();
    test_results_.clear();
}

MotorFollowTestCore::~MotorFollowTestCore() {
    stopTest();
}

bool MotorFollowTestCore::initialize(const TestConfig& config) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    config_ = config;
    
    std::cout << "========= MotorFollowTestCore 初始化 =========" << std::endl;
    std::cout << "机器人版本: " << config_.robot_version << std::endl;
    std::cout << "配置文件: " << config_.config_file_path << std::endl;
    std::cout << "模式: " << (config_.is_real ? "实物" : "仿真") << std::endl;
    std::cout << "时间步长: " << config_.dt << std::endl;
    
    // 直接使用传入的机器人参数
    num_joint_ = config_.num_joint;
    num_waist_joints_ = config_.num_waist_joints;
    num_arm_joints_ = config_.num_arm_joints;
    num_head_joints_ = config_.num_head_joints;
    na_foot_ = config_.na_foot;
    
    std::cout << "机器人参数:" << std::endl;
    std::cout << "  总关节数: " << num_joint_ << std::endl;
    std::cout << "  腰部关节数: " << num_waist_joints_ << std::endl;
    std::cout << "  腿部关节数: " << na_foot_ << std::endl;
    std::cout << "  手臂关节数: " << num_arm_joints_ << std::endl;
    std::cout << "  头部关节数: " << num_head_joints_ << std::endl;
    
    // 初始化关节组索引
    initializeJointGroupIndices();
    
    // 加载配置文件
    if (!loadConfigFile()) {
        std::cerr << "配置文件加载失败" << std::endl;
        return false;
    }
    
    // 生成电机对配置
    generateMotorPairs();
    
    // 生成控制参数
    generateControlParameters();
    
    // 初始化轨迹生成相关变量
    action_scale_W_.resize(num_joint_, 0.5); // 默认频率 0.5Hz (2秒周期)
    is_testing_completed_.resize(num_joint_, false);
    current_target_positions_.resize(num_joint_, 0.0);
    current_target_velocities_.resize(num_joint_, 0.0);
    
    std::cout << "MotorFollowTestCore 初始化完成" << std::endl;
    return true;
}


void MotorFollowTestCore::initializeJointGroupIndices() {
    // 根据机器人版本设置关节索引顺序
    if (config_.robot_version >= 50) {
        // 版本50+：左腿(0-5) -> 右腿(6-11) -> 腰部(12) -> 左臂(13-19) -> 右臂(20-26) -> 头部(27-28)
        joint_indices_.left_leg_start = 0;
        joint_indices_.left_leg_end = 5; // 6个关节
        
        joint_indices_.right_leg_start = 6;
        joint_indices_.right_leg_end = 11; // 6个关节
        
        // 腰部关节索引（在腿部之后）
        if (num_waist_joints_ > 0) {
            joint_indices_.waist_start = 12;
            joint_indices_.waist_end = 12 + num_waist_joints_ - 1;
        } else {
            joint_indices_.waist_start = -1;
            joint_indices_.waist_end = -1;
        }
    } else {
        // 版本13等旧版本：腰部(0) -> 左腿(1-6) -> 右腿(7-12) -> 左臂(13-19) -> 右臂(20-26) -> 头部(27-28)
        if (num_waist_joints_ > 0) {
            joint_indices_.waist_start = 0;
            joint_indices_.waist_end = num_waist_joints_ - 1;
        } else {
            joint_indices_.waist_start = -1;
            joint_indices_.waist_end = -1;
        }
        
        // 腿部关节索引（从腰部关节之后开始）
        joint_indices_.left_leg_start = num_waist_joints_;
        joint_indices_.left_leg_end = joint_indices_.left_leg_start + 5; // 6个关节
        
        joint_indices_.right_leg_start = joint_indices_.left_leg_end + 1;
        joint_indices_.right_leg_end = joint_indices_.right_leg_start + 5; // 6个关节
    }
    
    // 手臂和头部关节索引（根据版本设置）
    if (config_.robot_version >= 50) {
        // 版本50+：左臂(13-19) -> 右臂(20-26) -> 头部(27-28)
        joint_indices_.left_arm_start = 13;
        joint_indices_.left_arm_end = 19; // 7个关节
        
        joint_indices_.right_arm_start = 20;
        joint_indices_.right_arm_end = 26; // 7个关节
        
        joint_indices_.head_start = 27;
        joint_indices_.head_end = 28; // 2个关节
    } else {
        // 版本13等旧版本：手臂在腿部之后
        joint_indices_.left_arm_start = joint_indices_.right_leg_end + 1;
        joint_indices_.left_arm_end = joint_indices_.left_arm_start + (num_arm_joints_ / 2) - 1;
        
        joint_indices_.right_arm_start = joint_indices_.left_arm_end + 1;
        joint_indices_.right_arm_end = joint_indices_.right_arm_start + (num_arm_joints_ / 2) - 1;
        
        joint_indices_.head_start = joint_indices_.right_arm_end + 1;
        joint_indices_.head_end = joint_indices_.head_start + num_head_joints_ - 1;
    }
    
    std::cout << "关节索引范围:" << std::endl;
    std::cout << "  腰部: " << joint_indices_.waist_start << " - " << joint_indices_.waist_end << std::endl;
    std::cout << "  左腿: " << joint_indices_.left_leg_start << " - " << joint_indices_.left_leg_end << std::endl;
    std::cout << "  右腿: " << joint_indices_.right_leg_start << " - " << joint_indices_.right_leg_end << std::endl;
    std::cout << "  左臂: " << joint_indices_.left_arm_start << " - " << joint_indices_.left_arm_end << std::endl;
    std::cout << "  右臂: " << joint_indices_.right_arm_start << " - " << joint_indices_.right_arm_end << std::endl;
    std::cout << "  头部: " << joint_indices_.head_start << " - " << joint_indices_.head_end << std::endl;
}

bool MotorFollowTestCore::loadConfigFile() {
    std::ifstream config_file(config_.config_file_path);
    if (!config_file.is_open()) {
        std::cerr << "无法打开配置文件: " << config_.config_file_path << std::endl;
        return false;
    }
    
    Json::Value root;
    Json::Reader reader;
    
    if (!reader.parse(config_file, root)) {
        std::cerr << "配置文件解析失败: " << reader.getFormattedErrorMessages() << std::endl;
        return false;
    }
    
    // 检查配置文件版本
    if (root.isMember("version")) {
        int config_version = root["version"].asInt();
        if (config_version != config_.robot_version) {
            std::cerr << "配置文件版本不匹配: 期望 " << config_.robot_version 
                      << ", 实际 " << config_version << std::endl;
            return false;
        }
    }
    
    // 加载action_scale_k配置
    if (root.isMember("action_scale_k")) {
        action_scale_k_config_ = root["action_scale_k"];
    } else {
        std::cerr << "配置文件中缺少action_scale_k字段" << std::endl;
        return false;
    }
    
    // 加载joint_kp配置
    if (root.isMember("joint_kp")) {
        joint_kp_config_ = root["joint_kp"];
    } else {
        std::cerr << "配置文件中缺少joint_kp字段" << std::endl;
        return false;
    }
    
    // 加载joint_kd配置
    if (root.isMember("joint_kd")) {
        joint_kd_config_ = root["joint_kd"];
    } else {
        std::cerr << "配置文件中缺少joint_kd字段" << std::endl;
        return false;
    }
    
    // 加载偏移关节配置
    if (root.isMember("joint_offset")) {
        joint_offset_config_ = root["joint_offset"];
    } else {
        std::cout << "配置文件中未找到joint_offset字段，使用默认偏移关节配置" << std::endl;
    }
    
    std::cout << "配置文件加载成功" << std::endl;
    return true;
}

void MotorFollowTestCore::generateMotorPairs() {
    lr_leg_.clear();
    
    // 根据测试模式生成电机对
    switch (config_.test_mode) {
        case TestMode::FULL_BODY:
            // 全身测试：腰部 + 腿部 + 手臂
            generateFullBodyPairs();
            break;
        case TestMode::LEGS_ONLY:
            // 腿部测试：腰部 + 腿部
            generateLegPairs();
            break;
        case TestMode::ARMS_ONLY:
            // 手臂测试：手臂
            generateArmPairs();
            break;
        default:
            std::cerr << "未知的测试模式，使用默认全身测试" << std::endl;
            generateFullBodyPairs();
            break;
    }
    
    std::cout << "生成电机对配置 (" << getTestModeName() << "):" << std::endl;
    std::cout << "机器人版本: " << config_.robot_version << ", 腰部关节数: " << num_waist_joints_ << std::endl;
    int count = 0;
    for (size_t i = 0; i < lr_leg_.size(); ++i) {
        std::cout << "  " << i << ": " << lr_leg_[i].name 
                  << " [" << lr_leg_[i].left_joint << ", " << lr_leg_[i].right_joint << "]" << std::endl;
        count ++;
    }
    std::cout << "测试对数量:" << count << std::endl;
}

void MotorFollowTestCore::generateFullBodyPairs() {
    // 全身测试：腰部 + 腿部 + 手臂
    
    // 根据关节数量判断机器人类型
    bool has_waist = (num_waist_joints_ > 0);
    int arm_joints_per_side = num_arm_joints_ / 2;
    
    // 添加腰部关节（如果有）
    if (has_waist && joint_indices_.waist_start >= 0) {
        lr_leg_.push_back({joint_indices_.waist_start, joint_indices_.waist_start, "waist"});
    }
    
    // 添加腿部关节（6个关节）
    lr_leg_.push_back({joint_indices_.left_leg_start, joint_indices_.right_leg_start, "hip_yaw"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 1, joint_indices_.right_leg_start + 1, "hip_roll"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 2, joint_indices_.right_leg_start + 2, "hip_pitch"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 3, joint_indices_.right_leg_start + 3, "knee"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 4, joint_indices_.right_leg_start + 4, "ankle_pitch"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 5, joint_indices_.right_leg_start + 5, "ankle_roll"});
    
    // 添加手臂关节（根据关节数量）
    lr_leg_.push_back({joint_indices_.left_arm_start, joint_indices_.right_arm_start, "shoulder_pitch"});
    lr_leg_.push_back({joint_indices_.left_arm_start + 1, joint_indices_.right_arm_start + 1, "shoulder_roll"});
    lr_leg_.push_back({joint_indices_.left_arm_start + 2, joint_indices_.right_arm_start + 2, "shoulder_yaw"});
    lr_leg_.push_back({joint_indices_.left_arm_start + 3, joint_indices_.right_arm_start + 3, "elbow"});
    
    // 如果手臂有7个关节，添加额外的手臂关节
    if (arm_joints_per_side > 4) {
        lr_leg_.push_back({joint_indices_.left_arm_start + 4, joint_indices_.right_arm_start + 4, "wrist_yaw"});
        lr_leg_.push_back({joint_indices_.left_arm_start + 5, joint_indices_.right_arm_start + 5, "wrist_roll"});
        lr_leg_.push_back({joint_indices_.left_arm_start + 6, joint_indices_.right_arm_start + 6, "wrist_pitch"});
    }
}

void MotorFollowTestCore::generateLegPairs() {
    // 腿部测试：腰部 + 腿部
    
    // 根据关节数量判断机器人类型
    bool has_waist = (num_waist_joints_ > 0);
    
    // 添加腰部关节（如果有）
    if (has_waist && joint_indices_.waist_start >= 0) {
        lr_leg_.push_back({joint_indices_.waist_start, joint_indices_.waist_start, "waist"});
    }
    
    // 添加腿部关节（6个关节）
    lr_leg_.push_back({joint_indices_.left_leg_start, joint_indices_.right_leg_start, "hip_roll"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 1, joint_indices_.right_leg_start + 1, "hip_yaw"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 2, joint_indices_.right_leg_start + 2, "hip_pitch"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 3, joint_indices_.right_leg_start + 3, "knee"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 4, joint_indices_.right_leg_start + 4, "ankle_pitch"});
    lr_leg_.push_back({joint_indices_.left_leg_start + 5, joint_indices_.right_leg_start + 5, "ankle_roll"});
}

void MotorFollowTestCore::generateArmPairs() {
    // 手臂测试：手臂
    
    int arm_joints_per_side = num_arm_joints_ / 2;
    
    // 添加手臂关节（根据关节数量）
    lr_leg_.push_back({joint_indices_.left_arm_start, joint_indices_.right_arm_start, "shoulder_yaw"});
    lr_leg_.push_back({joint_indices_.left_arm_start + 1, joint_indices_.right_arm_start + 1, "shoulder_pitch"});
    lr_leg_.push_back({joint_indices_.left_arm_start + 2, joint_indices_.right_arm_start + 2, "shoulder_roll"});
    lr_leg_.push_back({joint_indices_.left_arm_start + 3, joint_indices_.right_arm_start + 3, "elbow"});
    
    // 如果手臂有7个关节，添加额外的手臂关节
    if (arm_joints_per_side > 4) {
        lr_leg_.push_back({joint_indices_.left_arm_start + 4, joint_indices_.right_arm_start + 4, "wrist_yaw"});
        lr_leg_.push_back({joint_indices_.left_arm_start + 5, joint_indices_.right_arm_start + 5, "wrist_pitch"});
        lr_leg_.push_back({joint_indices_.left_arm_start + 6, joint_indices_.right_arm_start + 6, "wrist_roll"});
    }
}

std::string MotorFollowTestCore::getTestModeName() const {
    switch (config_.test_mode) {
        case TestMode::FULL_BODY:
            return "全身测试";
        case TestMode::LEGS_ONLY:
            return "腿部测试";
        case TestMode::ARMS_ONLY:
            return "手臂测试";
        default:
            return "未知模式";
    }
}

void MotorFollowTestCore::generateControlParameters() {
    action_scale_k_.resize(num_joint_, 0.5); // 默认值
    action_scale_W_.resize(num_joint_, 0.5); // 默认频率 0.5Hz (2秒周期)
    
    // 从配置文件生成参数
    if (!action_scale_k_config_.isNull() && !action_scale_k_config_.empty()) {
        // 设置腰部参数
        if (action_scale_k_config_.isMember("waist") && joint_indices_.waist_start >= 0) {
            const Json::Value& waist = action_scale_k_config_["waist"];
            for (int i = joint_indices_.waist_start; i <= joint_indices_.waist_end; ++i) {
                int array_index = i - joint_indices_.waist_start;
                if (array_index < (int)waist.size()) {
                    action_scale_k_[i] = waist[array_index].asDouble();
                }
            }
        }
        
        // 设置左腿参数
        if (action_scale_k_config_.isMember("left_leg")) {
            const Json::Value& left_leg = action_scale_k_config_["left_leg"];
            for (int i = 0; i < 6 && i < (int)left_leg.size(); ++i) {
                int joint_id = joint_indices_.left_leg_start + i;
                action_scale_k_[joint_id] = left_leg[i].asDouble();
            }
        }
        
        // 设置右腿参数
        if (action_scale_k_config_.isMember("right_leg")) {
            const Json::Value& right_leg = action_scale_k_config_["right_leg"];
            for (int i = 0; i < 6 && i < (int)right_leg.size(); ++i) {
                int joint_id = joint_indices_.right_leg_start + i;
                action_scale_k_[joint_id] = right_leg[i].asDouble();
            }
        }
        
        // 设置手臂参数
        if (action_scale_k_config_.isMember("left_arm")) {
            const Json::Value& left_arm = action_scale_k_config_["left_arm"];
            for (int i = 0; i < 7 && i < (int)left_arm.size(); ++i) {
                int joint_id = joint_indices_.left_arm_start + i;
                action_scale_k_[joint_id] = left_arm[i].asDouble();
            }
        }
        
        if (action_scale_k_config_.isMember("right_arm")) {
            const Json::Value& right_arm = action_scale_k_config_["right_arm"];
            for (int i = 0; i < 7 && i < (int)right_arm.size(); ++i) {
                int joint_id = joint_indices_.right_arm_start + i;
                action_scale_k_[joint_id] = right_arm[i].asDouble();
            }
        }
        
        // 设置头部参数
        if (action_scale_k_config_.isMember("head")) {
            const Json::Value& head = action_scale_k_config_["head"];
            for (int i = 0; i < 2 && i < (int)head.size(); ++i) {
                int joint_id = joint_indices_.head_start + i;
                action_scale_k_[joint_id] = head[i].asDouble();
            }
        }
    }
    
    // 从配置文件读取action_scale_W参数
    if (!action_scale_k_config_.isNull() && action_scale_k_config_.isMember("action_scale_W")) {
        const Json::Value& action_scale_W_config = action_scale_k_config_["action_scale_W"];
        
        // 设置腰部频率
        if (action_scale_W_config.isMember("waist") && joint_indices_.waist_start >= 0) {
            for (int i = joint_indices_.waist_start; i <= joint_indices_.waist_end; ++i) {
                action_scale_W_[i] = action_scale_W_config["waist"].asDouble();
            }
        }
        
        // 设置左腿频率
        if (action_scale_W_config.isMember("left_leg")) {
            const Json::Value& left_leg = action_scale_W_config["left_leg"];
            for (int i = 0; i < 6 && i < (int)left_leg.size(); ++i) {
                int joint_id = joint_indices_.left_leg_start + i;
                action_scale_W_[joint_id] = left_leg[i].asDouble();
            }
        }
        
        // 设置右腿频率
        if (action_scale_W_config.isMember("right_leg")) {
            const Json::Value& right_leg = action_scale_W_config["right_leg"];
            for (int i = 0; i < 6 && i < (int)right_leg.size(); ++i) {
                int joint_id = joint_indices_.right_leg_start + i;
                action_scale_W_[joint_id] = right_leg[i].asDouble();
            }
        }
        
        // 设置手臂频率
        if (action_scale_W_config.isMember("left_arm")) {
            const Json::Value& left_arm = action_scale_W_config["left_arm"];
            for (int i = 0; i < 7 && i < (int)left_arm.size(); ++i) {
                int joint_id = joint_indices_.left_arm_start + i;
                action_scale_W_[joint_id] = left_arm[i].asDouble();
            }
        }
        
        if (action_scale_W_config.isMember("right_arm")) {
            const Json::Value& right_arm = action_scale_W_config["right_arm"];
            for (int i = 0; i < 7 && i < (int)right_arm.size(); ++i) {
                int joint_id = joint_indices_.right_arm_start + i;
                action_scale_W_[joint_id] = right_arm[i].asDouble();
            }
        }
        
        // 设置头部频率
        if (action_scale_W_config.isMember("head")) {
            const Json::Value& head = action_scale_W_config["head"];
            for (int i = 0; i < 2 && i < (int)head.size(); ++i) {
                int joint_id = joint_indices_.head_start + i;
                action_scale_W_[joint_id] = head[i].asDouble();
            }
        }
    }
    
    // 生成偏移关节配置
    joint_offset_.resize(num_joint_, 0); // 默认不偏移
    
    if (!joint_offset_config_.isNull() && !joint_offset_config_.empty()) {
        // 设置腰部偏移
        if (joint_offset_config_.isMember("waist") && joint_indices_.waist_start >= 0) {
            const Json::Value& waist = joint_offset_config_["waist"];
            for (int i = joint_indices_.waist_start; i <= joint_indices_.waist_end; ++i) {
                int array_index = i - joint_indices_.waist_start;
                if (array_index < (int)waist.size()) {
                    joint_offset_[i] = waist[array_index].asInt();
                }
            }
        }
        
        // 设置左腿偏移
        if (joint_offset_config_.isMember("left_leg")) {
            const Json::Value& left_leg = joint_offset_config_["left_leg"];
            for (int i = 0; i < 6 && i < (int)left_leg.size(); ++i) {
                int joint_id = joint_indices_.left_leg_start + i;
                joint_offset_[joint_id] = left_leg[i].asInt();
            }
        }
        
        // 设置右腿偏移
        if (joint_offset_config_.isMember("right_leg")) {
            const Json::Value& right_leg = joint_offset_config_["right_leg"];
            for (int i = 0; i < 6 && i < (int)right_leg.size(); ++i) {
                int joint_id = joint_indices_.right_leg_start + i;
                joint_offset_[joint_id] = right_leg[i].asInt();
            }
        }
        
        // 设置左臂偏移
        if (joint_offset_config_.isMember("left_arm")) {
            const Json::Value& left_arm = joint_offset_config_["left_arm"];
            for (int i = 0; i < 7 && i < (int)left_arm.size(); ++i) {
                int joint_id = joint_indices_.left_arm_start + i;
                joint_offset_[joint_id] = left_arm[i].asInt();
            }
        }
        
        // 设置右臂偏移
        if (joint_offset_config_.isMember("right_arm")) {
            const Json::Value& right_arm = joint_offset_config_["right_arm"];
            for (int i = 0; i < 7 && i < (int)right_arm.size(); ++i) {
                int joint_id = joint_indices_.right_arm_start + i;
                joint_offset_[joint_id] = right_arm[i].asInt();
            }
        }
        
        // 设置头部偏移
        if (joint_offset_config_.isMember("head")) {
            const Json::Value& head = joint_offset_config_["head"];
            for (int i = 0; i < 2 && i < (int)head.size(); ++i) {
                int joint_id = joint_indices_.head_start + i;
                joint_offset_[joint_id] = head[i].asInt();
            }
        }
    }
    
    std::cout << "控制参数生成完成" << std::endl;
    
    // 调试输出：显示action_scale_W的实际值
    std::cout << "\n=== action_scale_W 调试信息 ===" << std::endl;
    for (int i = 0; i < std::min(10, (int)action_scale_W_.size()); ++i) {
        std::cout << "关节 " << i << ": action_scale_W = " << action_scale_W_[i] 
                  << " Hz (周期 " << (1.0/action_scale_W_[i]) << " 秒)" << std::endl;
    }
    std::cout << "===============================" << std::endl;
}

bool MotorFollowTestCore::generateMotorCommand(const SensorData_t& sensor_data, JointCmd_t& joint_cmd) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (!test_running_) {
        return false;
    }
    
    // 处理传感器数据
    SensorData_t processed_data = processSensorData(sensor_data);
    
    // 生成电机指令
    joint_cmd.joint_pos.clear();
    joint_cmd.joint_vel.clear();
    joint_cmd.joint_torque.clear();
    joint_cmd.control_mode.clear();
    
    // 初始化所有关节指令 - 设置位置、速度、力矩和控制模式
    for (int i = 0; i < num_joint_; ++i) {
        joint_cmd.joint_pos.push_back(0.0);
        joint_cmd.joint_vel.push_back(0.0);
        joint_cmd.joint_torque.push_back(0.0);
        joint_cmd.control_mode.push_back(0);
    }
    
    // 打印测试关节信息
    printTestJointInfo();
    
    // 为所有关节计算PD控制力矩
    for (int i = 0; i < num_joint_; ++i) {
        bool is_test_joint = isCurrentTestJoint(i);
        
        if (is_test_joint) {
            // 测试关节：使用正弦波轨迹
            generateTestJointCommand(i, processed_data, joint_cmd);
        } else {
            // 非测试关节：保持零位置
            generateNonTestJointCommand(i, processed_data, joint_cmd);
        }
    }
    
    // 执行前向旋转输入处理
    int forward_result = forwardRotationInput();
    if (forward_result == 0) {
        // 当前关节对测试完成，继续到下一个关节对
        return true;
    } else if (forward_result == -1) {
        // 所有关节对测试完成
        test_complete_ = true;
        test_running_ = false;
        return false;
    }
    
    return true;
}

SensorData_t MotorFollowTestCore::processSensorData(const SensorData_t& sensor_data) {
    // 这里可以添加传感器数据处理逻辑
    // 比如滤波、坐标转换等
    return sensor_data;
}

bool MotorFollowTestCore::startTest() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (test_running_) {
        return false;
    }
    
    test_running_ = true;
    test_complete_ = false;
    test_start_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count() / 1000.0;
    index_ = 0;
    
    std::cout << "电机跟随测试开始" << std::endl;
    return true;
}

double MotorFollowTestCore::getCurrentTargetPosition(int joint_id) const {
    if (joint_id >= 0 && joint_id < static_cast<int>(current_target_positions_.size())) {
        return current_target_positions_[joint_id];
    }
    return 0.0;
}

void MotorFollowTestCore::stopTest() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    test_running_ = false;
    std::cout << "电机跟随测试停止" << std::endl;
}

bool MotorFollowTestCore::isTestComplete() const {
    return test_complete_;
}

double MotorFollowTestCore::getTestProgress() const {
    if (!test_running_ || lr_leg_.empty()) {
        return 0.0;
    }
    
    return static_cast<double>(index_) / lr_leg_.size();
}

double MotorFollowTestCore::getJointKp(int joint_id) const {
    // 从配置文件读取KP参数
    if (!joint_kp_config_.isNull()) {
        const Json::Value& kp_params = joint_kp_config_;
        
        // 根据关节类型返回对应的KP值
        if (joint_indices_.waist_start >= 0 && joint_id >= joint_indices_.waist_start && joint_id <= joint_indices_.waist_end) {
            if (kp_params.isMember("waist")) {
                const Json::Value& waist = kp_params["waist"];
                int idx = joint_id - joint_indices_.waist_start;
                if (idx < (int)waist.size()) {
                    return waist[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.left_leg_start && joint_id <= joint_indices_.left_leg_end) {
            if (kp_params.isMember("left_leg")) {
                const Json::Value& left_leg = kp_params["left_leg"];
                int idx = joint_id - joint_indices_.left_leg_start;
                if (idx < (int)left_leg.size()) {
                    return left_leg[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.right_leg_start && joint_id <= joint_indices_.right_leg_end) {
            if (kp_params.isMember("right_leg")) {
                const Json::Value& right_leg = kp_params["right_leg"];
                int idx = joint_id - joint_indices_.right_leg_start;
                if (idx < (int)right_leg.size()) {
                    return right_leg[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.left_arm_start && joint_id <= joint_indices_.left_arm_end) {
            if (kp_params.isMember("left_arm")) {
                const Json::Value& left_arm = kp_params["left_arm"];
                int idx = joint_id - joint_indices_.left_arm_start;
                if (idx < (int)left_arm.size()) {
                    return left_arm[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.right_arm_start && joint_id <= joint_indices_.right_arm_end) {
            if (kp_params.isMember("right_arm")) {
                const Json::Value& right_arm = kp_params["right_arm"];
                int idx = joint_id - joint_indices_.right_arm_start;
                if (idx < (int)right_arm.size()) {
                    return right_arm[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.head_start && joint_id <= joint_indices_.head_end) {
            if (kp_params.isMember("head")) {
                const Json::Value& head = kp_params["head"];
                int idx = joint_id - joint_indices_.head_start;
                if (idx < (int)head.size()) {
                    return head[idx].asDouble();
                }
            }
        }
    }
    
    // 默认值
    return 100.0;
}

double MotorFollowTestCore::getJointKd(int joint_id) const {
    // 从配置文件读取KD参数
    if (!joint_kd_config_.isNull()) {
        const Json::Value& kd_params = joint_kd_config_;
        
        // 根据关节类型返回对应的KD值
        if (joint_indices_.waist_start >= 0 && joint_id >= joint_indices_.waist_start && joint_id <= joint_indices_.waist_end) {
            if (kd_params.isMember("waist")) {
                const Json::Value& waist = kd_params["waist"];
                int idx = joint_id - joint_indices_.waist_start;
                if (idx < (int)waist.size()) {
                    return waist[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.left_leg_start && joint_id <= joint_indices_.left_leg_end) {
            if (kd_params.isMember("left_leg")) {
                const Json::Value& left_leg = kd_params["left_leg"];
                int idx = joint_id - joint_indices_.left_leg_start;
                if (idx < (int)left_leg.size()) {
                    return left_leg[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.right_leg_start && joint_id <= joint_indices_.right_leg_end) {
            if (kd_params.isMember("right_leg")) {
                const Json::Value& right_leg = kd_params["right_leg"];
                int idx = joint_id - joint_indices_.right_leg_start;
                if (idx < (int)right_leg.size()) {
                    return right_leg[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.left_arm_start && joint_id <= joint_indices_.left_arm_end) {
            if (kd_params.isMember("left_arm")) {
                const Json::Value& left_arm = kd_params["left_arm"];
                int idx = joint_id - joint_indices_.left_arm_start;
                if (idx < (int)left_arm.size()) {
                    return left_arm[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.right_arm_start && joint_id <= joint_indices_.right_arm_end) {
            if (kd_params.isMember("right_arm")) {
                const Json::Value& right_arm = kd_params["right_arm"];
                int idx = joint_id - joint_indices_.right_arm_start;
                if (idx < (int)right_arm.size()) {
                    return right_arm[idx].asDouble();
                }
            }
        } else if (joint_id >= joint_indices_.head_start && joint_id <= joint_indices_.head_end) {
            if (kd_params.isMember("head")) {
                const Json::Value& head = kd_params["head"];
                int idx = joint_id - joint_indices_.head_start;
                if (idx < (int)head.size()) {
                    return head[idx].asDouble();
                }
            }
        }
    }
    
    // 默认值
    return 10.0;
}

int MotorFollowTestCore::forwardRotationInput() {
    // 使用真实时间而不是固定的dt累积
    static auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
    time_offset_ = elapsed.count() / 1000.0; // 转换为秒
    
    // 检查索引是否超出范围
    if (index_ >= static_cast<int>(lr_leg_.size())) {
        return -1; // 所有测试完成
    }
    
    // 检查是否两个关节都完成了10个周期（仿真和实物都同时测试两个电机）
    if (cycleCountL_ >= 10 && cycleCountR_ >= 10) {
        // 获取当前测试的关节对
        int current_l_index = lr_leg_[index_].left_joint;
        int current_r_index = lr_leg_[index_].right_joint;
        
        // 标记当前测试关节为已完成
        is_testing_completed_[current_l_index] = true;
        is_testing_completed_[current_r_index] = true;
        
        std::cout << "=== 测试完成标记 ===" << std::endl;
        std::cout << "完成测试的关节对: " << lr_leg_[index_].name 
                  << " [L:" << current_l_index << ", R:" << current_r_index << "]" << std::endl;
        
        // 重置周期计数器
        cycleCountL_ = 0;
        cycleCountR_ = 0;
        prevSinL_ = MAX_VALUE;
        prevSinR_ = MAX_VALUE;
        time_offset_ = 0.0;
        sinNum_ = 0;

        // 保存当前测试对的数据
        std::cout << "开始保存测试对 " << index_ << " 的数据..." << std::endl;
        
        // 保存当前测试对的数据
        std::cout << "开始保存测试对 " << index_ << " 的数据..." << std::endl;
        saveTestDataToFile(index_);
        
        // 切换到下一对关节
        index_ = index_ + 1;
        if (index_ < static_cast<int>(lr_leg_.size())) {
            int l_index = lr_leg_[index_].left_joint;
            int r_index = lr_leg_[index_].right_joint;
            std::cout << "切换到关节对 " << index_ << ", l_index: " << l_index << ", r_index: " << r_index << std::endl;
            return 0; // 当前关节对测试完成，继续下一个
        } else {
            std::cout << "所有关节对测试完成, index: " << index_ << std::endl;
            return -1; // 所有测试完成
        }
    }

    int l_index = lr_leg_[index_].left_joint;
    int r_index = lr_leg_[index_].right_joint;
    
    // 计算当前正弦值
    double currentSinL = std::sin(action_scale_W_[l_index] * time_offset_ * 2 * M_PI);
    double currentSinR = std::sin(action_scale_W_[r_index] * time_offset_ * 2 * M_PI);
    // 为偏移关节使用相位后移 -π/2 的波形，使 (sin(· - π/2) + 1) 在 t=0 起始为 0
    double currentSinL_shift = std::sin(action_scale_W_[l_index] * time_offset_ * 2 * M_PI - M_PI / 2.0);
    double currentSinR_shift = std::sin(action_scale_W_[r_index] * time_offset_ * 2 * M_PI - M_PI / 2.0);
    
    // 计算目标速度（正弦信号的导数）
    double target_vel_L = action_scale_W_[l_index] * 2 * M_PI * std::cos(action_scale_W_[l_index] * time_offset_ * 2 * M_PI);
    double target_vel_R = action_scale_W_[r_index] * 2 * M_PI * std::cos(action_scale_W_[r_index] * time_offset_ * 2 * M_PI);
    // 偏移关节的目标速度
    double target_vel_L_shift = action_scale_W_[l_index] * 2 * M_PI * std::cos(action_scale_W_[l_index] * time_offset_ * 2 * M_PI - M_PI / 2.0);
    double target_vel_R_shift = action_scale_W_[r_index] * 2 * M_PI * std::cos(action_scale_W_[r_index] * time_offset_ * 2 * M_PI - M_PI / 2.0);

    // 使用配置文件中的偏移关节设置
    bool is_offset_joint_l = isOffsetJoint(l_index);
    bool is_offset_joint_r = isOffsetJoint(r_index);
    
    // 计算目标位置和目标速度（统一逻辑）
    double l_target_pos, r_target_pos;
    double l_target_vel, r_target_vel;
    
    // 根据偏移关节配置计算目标位置和速度
    if (is_offset_joint_l) {
        l_target_pos = action_scale_k_[l_index] * (currentSinL_shift +1.0);
        l_target_vel = action_scale_k_[l_index] * target_vel_L_shift;
    } else {
        l_target_pos = action_scale_k_[l_index] * currentSinL;
        l_target_vel = action_scale_k_[l_index] * target_vel_L;
    }
    
    if (is_offset_joint_r) {
        r_target_pos = action_scale_k_[r_index] * (currentSinR_shift +1.0);
        r_target_vel = action_scale_k_[r_index] * target_vel_R_shift;
    } else {
        r_target_pos = action_scale_k_[r_index] * currentSinR;
        r_target_vel = action_scale_k_[r_index] * target_vel_R;
    }
    
    // 存储目标位置和速度供generateMotorCommand使用
    current_target_positions_[l_index] = l_target_pos;
    current_target_velocities_[l_index] = l_target_vel;
    
    // 如果左右关节不同，才设置右关节的目标位置
    if (l_index != r_index) {
        current_target_positions_[r_index] = r_target_pos;
        current_target_velocities_[r_index] = r_target_vel;
    }
    
    // 周期计数逻辑
    if (currentSinL > 0 && prevSinL_ <= 0) {
        cycleCountL_++;
    }
    if (currentSinR > 0 && prevSinR_ <= 0) {
        cycleCountR_++;
    }
    
    prevSinL_ = currentSinL;
    prevSinR_ = currentSinR;
    
    return 1; // 继续当前测试
}

void MotorFollowTestCore::motor2joint(const SensorData_t& sensor_data_motor, SensorData_t& sensor_data_joint) {
    // 初始化关节传感器数据结构
    sensor_data_joint.joint_q.resize(num_joint_);
    sensor_data_joint.joint_v.resize(num_joint_);
    sensor_data_joint.joint_vd.resize(num_joint_);
    
    // 直接复制电机数据到关节数据（motor_follow_test直接使用电机空间数据）
    for (int i = 0; i < num_joint_; ++i) {
        if (i < sensor_data_motor.joint_q.size()) {
            sensor_data_joint.joint_q[i] = sensor_data_motor.joint_q[i];
            sensor_data_joint.joint_v[i] = sensor_data_motor.joint_v[i];
            sensor_data_joint.joint_vd[i] = sensor_data_motor.joint_vd[i];
        } else {
            sensor_data_joint.joint_q[i] = 0.0;
            sensor_data_joint.joint_v[i] = 0.0;
            sensor_data_joint.joint_vd[i] = 0.0;
        }
    }
    
    // 注意：motor_follow_test直接使用电机空间数据，不进行ankle solver转换
    // 这样可以避免ankle solver可能引入的符号错误问题
}




bool MotorFollowTestCore::isOffsetJoint(int joint_id) const {
    if (joint_id < 0 || joint_id >= static_cast<int>(joint_offset_.size())) {
        return false;
    }
    return joint_offset_[joint_id] == 1;
}

void MotorFollowTestCore::recordJointCommand(int joint_index, double command_value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 数据有效性检查：过滤异常值
    if (std::isnan(command_value) || std::isinf(command_value) || 
        std::abs(command_value) > 1000.0) {
        std::cout << "警告：关节 " << joint_index << " 命令数据异常: " << command_value << std::endl;
        return; // 跳过异常数据
    }
    
    TimedValue cmd_data{command_value, getMillisecondTimestamp()};
    
    // 获取当前测试的关节对
    if (index_ < lr_leg_.size()) {
        int l_index = lr_leg_[index_].left_joint;
        int r_index = lr_leg_[index_].right_joint;
        
        if (joint_index == l_index) {
            l_txBuffer_.push_back(cmd_data);
        }
        if (joint_index == r_index) {
            r_txBuffer_.push_back(cmd_data);
        }
    }
}

void MotorFollowTestCore::recordJointResponse(int joint_index, double response_value) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 数据有效性检查：过滤异常值
    if (std::isnan(response_value) || std::isinf(response_value) || 
        std::abs(response_value) > 1000.0) {
        std::cout << "警告：关节 " << joint_index << " 响应数据异常: " << response_value << std::endl;
        return; // 跳过异常数据
    }
    
    TimedValue resp_data{response_value, getMillisecondTimestamp()};
    
    // 获取当前测试的关节对
    if (index_ < lr_leg_.size()) {
        int l_index = lr_leg_[index_].left_joint;
        int r_index = lr_leg_[index_].right_joint;
        
        if (joint_index == l_index) {
            l_rxBuffer_.push_back(resp_data);
        }
        if (joint_index == r_index) {
            r_rxBuffer_.push_back(resp_data);
        }
    }
}

long long MotorFollowTestCore::getMillisecondTimestamp() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return duration.count();
}

void MotorFollowTestCore::saveTestDataToFile(int pair_index) {
    // 注意：调用此函数时，调用者必须已经持有data_mutex_锁
    
    if (pair_index >= lr_leg_.size()) {
        std::cerr << "错误：测试对索引超出范围" << std::endl;
        return;
    }
    
    std::cout << "保存测试对 " << pair_index << " 数据 (L:" << lr_leg_[pair_index].left_joint 
              << ", R:" << lr_leg_[pair_index].right_joint << ")" << std::endl;
    
    // 限制数据点数量到5000，就像原始脚本一样
    std::vector<TimedValue> l_tx_limited, l_rx_limited, r_tx_limited, r_rx_limited;
    
    // 左输入数据
    if (l_txBuffer_.size() > 5000) {
        l_tx_limited.assign(l_txBuffer_.end() - 5000, l_txBuffer_.end());
    } else {
        l_tx_limited = l_txBuffer_;
    }
    
    // 左响应数据
    if (l_rxBuffer_.size() > 5000) {
        l_rx_limited.assign(l_rxBuffer_.end() - 5000, l_rxBuffer_.end());
    } else {
        l_rx_limited = l_rxBuffer_;
    }
    
    // 右输入数据
    if (r_txBuffer_.size() > 5000) {
        r_tx_limited.assign(r_txBuffer_.end() - 5000, r_txBuffer_.end());
    } else {
        r_tx_limited = r_txBuffer_;
    }
    
    // 右响应数据
    if (r_rxBuffer_.size() > 5000) {
        r_rx_limited.assign(r_rxBuffer_.end() - 5000, r_rxBuffer_.end());
    } else {
        r_rx_limited = r_rxBuffer_;
    }
    
    // 按电机对保存数据：每个电机对保存成一个文件，包含时间戳和四个数据列
    std::string pair_filename = "motorPair_" + std::to_string(pair_index) + "_L" + 
                               std::to_string(lr_leg_[pair_index].left_joint) + "_R" + 
                               std::to_string(lr_leg_[pair_index].right_joint) + ".txt";
    std::string pair_file = buildFilePath(pair_filename);
    
    std::cout << "准备保存电机对数据文件，约" << l_tx_limited.size() << "个数据点" << std::endl;
    
    // 保存电机对数据到单个文件
    saveMotorPairToFile(l_tx_limited, l_rx_limited, r_tx_limited, r_rx_limited, pair_file);
    
    // 清空当前测试对的数据缓冲区
    l_txBuffer_.clear();
    r_txBuffer_.clear();
    l_rxBuffer_.clear();
    r_rxBuffer_.clear();
    
    std::cout << "测试对 " << pair_index << " 数据保存完成" << std::endl;
}


void MotorFollowTestCore::saveMotorPairToFile(const std::vector<TimedValue>& l_tx, 
                                             const std::vector<TimedValue>& l_rx,
                                             const std::vector<TimedValue>& r_tx,
                                             const std::vector<TimedValue>& r_rx,
                                             const std::string& savePath) const {
    // 确保目录存在
    std::filesystem::path path(savePath);
    std::filesystem::path dir = path.parent_path();
    if (!dir.empty() && !std::filesystem::exists(dir)) {
        try {
            std::filesystem::create_directories(dir);
        } catch (const std::exception& e) {
            std::cerr << "创建目录失败: " << e.what() << std::endl;
            return;
        }
    }
    
    // 写入文件
    std::ofstream file(savePath);
    if (!file) {
        std::cerr << "无法打开文件: " << savePath << std::endl;
        return;
    }
    
    // 写入表头
    file << "timestamp\tleft_target_pos\tleft_actual_pos\tright_target_pos\tright_actual_pos\n";
    
    // 确定数据点数量（取最小值）
    size_t data_count = std::min({l_tx.size(), l_rx.size(), r_tx.size(), r_rx.size()});
    
    // 写入数据：时间戳、左输入、左响应、右输入、右响应
    for (size_t i = 0; i < data_count; ++i) {
        file << l_tx[i].timestamp << '\t' 
             << l_tx[i].value << '\t'
             << l_rx[i].value << '\t'
             << r_tx[i].value << '\t'
             << r_rx[i].value << '\n';
    }
    
    file.close();
    std::cout << "电机对数据已保存: " << data_count << " 个数据点到 " << savePath << std::endl;
}

std::string MotorFollowTestCore::buildFilePath(const std::string& filename) const {
    // 获取当前源文件路径
    std::string current_path = __FILE__;
    
    // 找到源文件目录
    size_t last_slash = current_path.find_last_of("/");
    if (last_slash != std::string::npos) {
        std::string source_dir = current_path.substr(0, last_slash);
        return source_dir + "/file/" + filename;
    }
    
    // 如果无法获取路径，使用当前目录下的file文件夹
    return "./file/" + filename;
}

std::string MotorFollowTestCore::generateTestReport() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::stringstream report;
    report << "========= 电机跟随测试报告 =========" << std::endl;
    report << "测试时间: " << std::chrono::system_clock::now().time_since_epoch().count() << std::endl;
    report << "机器人版本: " << config_.robot_version << std::endl;
    report << "测试模式: " << (config_.test_mode == TestMode::FULL_BODY ? "全身" : 
                              config_.test_mode == TestMode::LEGS_ONLY ? "腿部" : "手臂") << std::endl;
    report << "总测试对数: " << lr_leg_.size() << std::endl;
    report << "完成测试对数: " << test_results_.size() << std::endl;
    report << "数据保存路径: " << data_save_path_ << std::endl;
    report << std::endl;
    
    if (!test_results_.empty()) {
        report << "========= 测试结果 =========" << std::endl;
        for (const auto& result : test_results_) {
            report << "测试对 " << result.pair_index << " (关节 " << result.left_joint << "-" << result.right_joint << "): " << result.status << std::endl;
        }
    }
    
    return report.str();
}

bool MotorFollowTestCore::isCurrentTestJoint(int joint_id) const {
    if (index_ >= static_cast<int>(lr_leg_.size())) {
        return false;
    }
    
    const MotorPair& current_pair = lr_leg_[index_];
    return (joint_id == current_pair.left_joint || joint_id == current_pair.right_joint);
}

void MotorFollowTestCore::printTestJointInfo() {
    static auto last_print_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time);
    
    // 每1秒打印一次测试关节信息
    if (time_diff.count() >= 1) {
        if (index_ < static_cast<int>(lr_leg_.size())) {
            const MotorPair& current_pair = lr_leg_[index_];
            std::cout << "[测试关节] 当前测试对: " << current_pair.name 
                      << " (左关节: " << current_pair.left_joint 
                      << ", 右关节: " << current_pair.right_joint << ")" << std::endl;
        } else {
            std::cout << "[测试关节] 当前无测试关节对" << std::endl;
        }
        last_print_time = current_time;
    }
}

void MotorFollowTestCore::generateTestJointCommand(int joint_id, const SensorData_t& sensor_data, JointCmd_t& joint_cmd) {
    // 检查响应数据是否异常
    if (joint_id < sensor_data.joint_q.size()) {
        double response_value = sensor_data.joint_q[joint_id];
        if (std::isnan(response_value) || std::isinf(response_value) || 
            std::abs(response_value) > 1000.0) {
            std::cout << "警告：关节 " << joint_id << " 响应数据异常，暂时不控制: " << response_value << std::endl;
            // 异常数据时，保持零位置和零力矩
            joint_cmd.joint_pos[joint_id] = 0.0;
            joint_cmd.joint_vel[joint_id] = 0.0;
            joint_cmd.joint_torque[joint_id] = 0.0;
            return;
        }
    }
    
    // 测试关节：使用正弦波轨迹
    double target_pos = current_target_positions_[joint_id];
    double target_vel = current_target_velocities_[joint_id];
    
    // 计算PD控制力矩
    double kp = getJointKp(joint_id);
    double kd = getJointKd(joint_id);
    
    // 位置误差和速度误差
    double pos_error = target_pos - sensor_data.joint_q[joint_id];
    double vel_error = target_vel - sensor_data.joint_v[joint_id];
    
    // PD控制力矩计算
    joint_cmd.joint_torque[joint_id] = kp * pos_error + kd * vel_error;
    
    // 设置目标位置和速度（用于记录）
    joint_cmd.joint_pos[joint_id] = target_pos;
    joint_cmd.joint_vel[joint_id] = target_vel;
}

void MotorFollowTestCore::generateNonTestJointCommand(int joint_id, const SensorData_t& sensor_data, JointCmd_t& joint_cmd) {
    // 检查响应数据是否异常
    if (joint_id < sensor_data.joint_q.size()) {
        double response_value = sensor_data.joint_q[joint_id];
        if (std::isnan(response_value) || std::isinf(response_value) || 
            std::abs(response_value) > 100.0) {
            std::cout << "警告：关节 " << joint_id << " 响应数据异常，暂时不控制: " << response_value << std::endl;
            // 异常数据时，保持零位置和零力矩
            joint_cmd.joint_pos[joint_id] = 0.0;
            joint_cmd.joint_vel[joint_id] = 0.0;
            joint_cmd.joint_torque[joint_id] = 0.0;
            return;
        }
    }
    
    // 非测试关节：保持零位置
    double target_pos = 0.0;
    double target_vel = 0.0;
    
    // 计算PD控制力矩
    double kp = getJointKp(joint_id);
    double kd = getJointKd(joint_id);
    
    // 位置误差和速度误差
    double pos_error = target_pos - sensor_data.joint_q[joint_id];
    double vel_error = target_vel - sensor_data.joint_v[joint_id];
    
    // PD控制力矩计算
    joint_cmd.joint_torque[joint_id] = kp * pos_error + kd * vel_error;
    
    // 设置目标位置和速度
    joint_cmd.joint_pos[joint_id] = target_pos;
    joint_cmd.joint_vel[joint_id] = target_vel;
}

} // namespace motor_follow_test
