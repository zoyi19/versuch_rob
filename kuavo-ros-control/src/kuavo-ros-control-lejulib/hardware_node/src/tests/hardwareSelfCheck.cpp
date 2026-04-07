#include "hardwareSelfCheck.h"
#include "kuavo_assets/include/package_path.h"

#include <ros/package.h>
#include <limits>

// ANSI转义序列定义颜色
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string RESET = "\033[0m";

hardwareSelfCheck::hardwareSelfCheck(double dt, ros::NodeHandle nh) : dt_(dt), nh_(nh)
{

    std::cout << "Set dt: " << dt_ << std::endl;
    joint_cmd_.resize(num_joint_);
    joint_data_.resize(num_joint_);
    loadConfig();

    int ec_count = 0;
    for (int i = 0; i < motor_info.driver.size(); ++i)
    {
        if (motor_info.driver[i] == EC_MASTER)
        {
            if (motor_info.motors_exist[i])
            {
                ec_count++;
                ec_index_map_[i] = ec_count;
            }
            else
            {
                ec_index_map_[i] = kEcMasterNilId; // EC master not exist
            }
        }
    }
}

hardwareSelfCheck::~hardwareSelfCheck()
{
    // 释放通过 new 创建的原始指针
    delete ruiwo_actuator_;
    ruiwo_actuator_ = nullptr;
}

void hardwareSelfCheck::SetJointPosition(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data)
{
    // Classify IDs based on their types
    std::vector<uint8_t> ecMasterIds;
    std::vector<uint8_t> ruiwoIds;
    std::vector<JointParam_t> EC_joint_data;
    std::vector<double> dyn_joint_data;
    std::vector<double> rm_joint_data;
    RuiWoJointData rw_joint_data;

    // Add more vectors for other types if needed...
    for (size_t i = 0; i < joint_ids.size(); ++i)
    {
        size_t index = joint_ids[i] - 1;
        // Check the driver type and add to the appropriate vector
        if (motor_info.driver[index] == EC_MASTER)
        {
            // Only add the motor that exist
            if (motor_info.motors_exist[index])
            {
                EC_joint_data.push_back(joint_data.at(i));
                ecMasterIds.push_back(ec_index_map_[index]);
            }
        }
        else if (motor_info.driver[index] == DYNAMIXEL)
        {
            dyn_joint_data.push_back(joint_data.at(i).position * TO_RADIAN);
        }
        else if (motor_info.driver[index] == REALMAN)
        {
            rm_joint_data.push_back(joint_data.at(i).position);
        }
        else if (motor_info.driver[index] == RUIWO)
        {
            ruiwoIds.push_back(ruiwoIds.size());
            rw_joint_data.pos.push_back(joint_data.at(i).position);
            rw_joint_data.vel.push_back(joint_data.at(i).velocityOffset);
            rw_joint_data.torque.push_back(joint_data.at(i).torqueOffset);
        }
    }
    if (ecMasterIds.size())
    {
        std::vector<uint16_t> ecMasterIds16(ecMasterIds.size());
        convert_uint8_to_uint16(ecMasterIds.data(), ecMasterIds16.data(), ecMasterIds.size());
        actuators_.setJointPosition(ecMasterIds16.data(), driver_type, ecMasterIds.size(), EC_joint_data.data());
    }
    if (ruiwoIds.size())
    {
        // 接口传入的是角度, c++sdk中转为弧度
        ruiwo_actuator_->set_positions(ruiwoIds, rw_joint_data.pos, rw_joint_data.torque, rw_joint_data.vel);
    }
}

void hardwareSelfCheck::GetJointData(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data)
{
    // Classify IDs based on their types
    std::vector<uint8_t> ecMasterIds;
    std::vector<uint8_t> ecExistIds;
    std::vector<uint8_t> ruiwoIds;
    std::vector<JointParam_t> EC_joint_data;
    std::vector<double> dyn_joint_data;
    std::vector<double> rm_joint_data;
    RuiWoJointData rw_joint_data;
    // Add more vectors for other types if needed...

    for (size_t i = 0; i < joint_ids.size(); ++i)
    {
        size_t index = joint_ids[i] - 1;
        // Check the driver type and add to the appropriate vector
        if (motor_info.driver[index] == EC_MASTER)
        {
            ecMasterIds.push_back(ec_index_map_[index]);
            // EC_joint_data.push_back(joint_data.at(i));
            if (motor_info.motors_exist[i])
            {
                ecExistIds.push_back(ec_index_map_[index]);
            }
        }
        else if (motor_info.driver[index] == DYNAMIXEL)
        {
            dyn_joint_data.push_back(joint_data.at(i).position);
        }
        else if (motor_info.driver[index] == REALMAN)
        {
            rm_joint_data.push_back(joint_data.at(i).position);
        }
        else if (motor_info.driver[index] == RUIWO)
        {
            ruiwoIds.push_back(ruiwoIds.size());
            rw_joint_data.pos.push_back(joint_data.at(i).position);
            rw_joint_data.vel.push_back(joint_data.at(i).velocity);
            rw_joint_data.torque.push_back(joint_data.at(i).torque);
        }

        // Add more conditions for other types if needed...
    }
    if (ecMasterIds.size())
    {
        EC_joint_data.resize(ecMasterIds.size());
        std::vector<JointParam_t> exsit_ec_joint_data(ecExistIds.size());
        std::vector<uint16_t> ecMasterIds16(ecExistIds.size());
        convert_uint8_to_uint16(ecExistIds.data(), ecMasterIds16.data(), ecExistIds.size());
        actuators_.getJointData(ecMasterIds16.data(), driver_type, ecExistIds.size(), exsit_ec_joint_data.data());

        int ec_exist_index = 0;
        for (size_t i = 0; i < EC_joint_data.size(); i++)
        {
            if (ecMasterIds[i] != kEcMasterNilId)
            {
                EC_joint_data[i] = exsit_ec_joint_data.at(ec_exist_index++);
            }
            else
            {
                // !!! Motor does not exist, set all values default.
                EC_joint_data[i] = JointParam_t();
            }
        }
    }
    if (ruiwoIds.size())
    {
        // 获取到弧度值
        rw_joint_data.pos = ruiwo_actuator_->get_positions();
        rw_joint_data.vel = ruiwo_actuator_->get_velocity();
        rw_joint_data.torque = ruiwo_actuator_->get_torque();
        for (size_t i = 0; i < rw_joint_data.pos.size(); i++)
        {
            rw_joint_data.pos.at(i) = (rw_joint_data.pos.at(i) * 180) / M_PI;
            rw_joint_data.vel.at(i) = (rw_joint_data.vel.at(i) * 180) / M_PI;
            rw_joint_data.torque.at(i) = rw_joint_data.torque.at(i);
        }
    }

    uint8_t num_ec_data = 0;
    uint8_t num_rm_data = 0;
    uint8_t num_dyn_data = 0;
    uint8_t num_rw_data = 0;
    for (size_t i = 0; i < joint_ids.size(); ++i)
    {
        size_t index = joint_ids[i] - 1;
        // Check the driver type and add to the appropriate vector
        if (motor_info.driver[index] == EC_MASTER)
        {
            // std::cout << ":i:" << i << " EC_joint_data[num_ec_data]:" << EC_joint_data[num_ec_data].position << std::endl;
            joint_data.at(i) = EC_joint_data.at(num_ec_data);
            num_ec_data++;
        }
        else if (motor_info.driver[index] == DYNAMIXEL)
        {
            joint_data.at(i).position = dyn_joint_data.at(num_dyn_data++);
            // joint_data[i].position = 0;
        }
        else if (motor_info.driver[index] == REALMAN)
        {
            joint_data.at(i).position = rm_joint_data.at(num_rm_data++);
        }
        else if (motor_info.driver[index] == RUIWO)
        {
            if (rw_joint_data.pos.size() <= num_rw_data)
            {
                std::cerr << "[HardwareNode::GetJointData] rw_joint_data.pos.size() " << rw_joint_data.pos.size() << " <= num_rw_data " << num_rw_data << std::endl;
                continue;
            }
            joint_data.at(i).position = rw_joint_data.pos.at(num_rw_data);
            joint_data.at(i).velocity = rw_joint_data.vel.at(num_rw_data);
            joint_data.at(i).torque = rw_joint_data.torque.at(num_rw_data);
            ++num_rw_data;
        }
    }
}

bool hardwareSelfCheck::jointMoveTo(std::vector<double> goal_pos, double speed, double dt, double current_limit)
{
    Eigen::VectorXd cmd_s(goal_pos.size());
    Eigen::VectorXd motor_position(na_foot_);
    for (size_t i = 0; i < goal_pos.size(); i++)
    {
        cmd_s(i) = goal_pos[i] * TO_RADIAN;
    }
    motor_position = ankleSolver_.joint_to_motor_position(cmd_s.head(na_foot_));
    for (size_t i = 0; i < na_foot_; i++)
    {
        goal_pos.at(i) = motor_position(i) * TO_DEGREE;
    }

    GetJointData(joint_ids_, joint_data_);
    usleep(10000);
    std::vector<double> start_pos(num_joint_, 0);
    std::vector<double> T(num_joint_, 0);
    for (uint32_t i = 0; i < num_joint_; i++)
    {
        start_pos[i] = joint_data_[i].position;
        T[i] = fabs(goal_pos[i] - start_pos[i]) / speed;
        printf("Joint %u from %f to %f\n", i + 1, start_pos[i], goal_pos[i]);
    }
    double max_T = *max_element(T.begin(), T.end());
    if (max_T < 0.5)
    {
        max_T = 0.5;
    }
    printf("Duration %f\n", max_T);
    uint32_t total_cnt = ceil(max_T / dt);
    uint32_t count = 0;
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    while (1)
    {
        for (uint32_t i = 0; i < num_joint_; i++)
        {
            joint_cmd_[i].position = calcCos(start_pos[i], goal_pos[i], total_cnt, count);
            if (current_limit > 0)
            {
                joint_cmd_[i].maxTorque = current_limit;
            }
            else
            {
                joint_cmd_[i].maxTorque = motor_info.max_current[i];
            }
        }
        SetJointPosition(joint_ids_, joint_cmd_);

        count++;
        if (count > total_cnt)
        {
            OsSleep(4);
            GetJointData(joint_ids_, joint_data_);
            break;
        }

        next_time.tv_sec += (next_time.tv_nsec + dt * 1e9) / 1e9;
        next_time.tv_nsec = (int)(next_time.tv_nsec + dt * 1e9) % (int)1e9;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    return true;
    // wait move done
    //  return waitMoveDone(goal_pos);
}

bool hardwareSelfCheck::detectYawDrift()
{
    const int SAMPLE_COUNT = 1000;         // 检测样本数（可根据需求调整）
    const double MAX_ALLOWED_DRIFT = 0.1;  // 允许的最大漂移（弧度，约5.7度）

    std::vector<double> yawAngles;         // 存储所有采样的yaw角
    int readFailedCount = 0;               // 记录读取失败次数

    // 仅当IMU支持时间戳时使用频率统计相关变量
    bool isTimestampSupported = (imu_type_str_ == "xsens");
    std::vector<double> acc_intervals;     // 加速度计时间间隔（仅xsens使用）
    std::vector<double> gyro_intervals;    // 陀螺仪时间间隔（仅xsens使用）
    std::vector<double> quat_intervals;    // 四元数时间间隔（仅xsens使用）
    struct timespec last_acc_time = {0}, last_gyro_time = {0}, last_quat_time = {0};

    std::cout << "正在检测Yaw角稳定性（保持静止）..." << std::endl;
    Eigen::Vector3d acc, gyro;
    Eigen::Quaterniond quat;
    struct timespec current_time, current_acc_time, current_gyro_time, current_quat_time;

    // 首次读取数据
    bool readflag = false;
    if (isTimestampSupported)
    {
        // Xsens IMU（支持时间戳）
        readflag = xsens_IMU::getImuDataFrame(acc, gyro, quat,
                                              current_time, current_acc_time,
                                              current_gyro_time, current_quat_time);
    }
    else
    {
        // HIPNUC IMU（不支持时间戳，无时间相关参数）
        readflag = HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
    }

    if (!readflag)
    {
        std::cerr << "IMU数据读取失败!" << std::endl;
        readFailedCount++;
    }
    else
    {
        // 记录Yaw角
        Eigen::Vector3d eulerAngles = quat.toRotationMatrix().eulerAngles(2, 1, 0);
        yawAngles.push_back(eulerAngles[0]);
        std::cout << "采样 " << 1 << "/" << SAMPLE_COUNT << "，Yaw: " << yawAngles.back() << " rad" << std::endl;

        // 初始化时间戳（仅xsens）
        if (isTimestampSupported)
        {
            last_acc_time = current_acc_time;
            last_gyro_time = current_gyro_time;
            last_quat_time = current_quat_time;
        }
    }

    // 主采样循环
    readflag = false;
    struct timespec start_time, current_check_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    bool first_non_zero_found = false;
    int last_printed_sample = 0;
    
    for (int i = 1; i < SAMPLE_COUNT; ++i)
    {
        if (isTimestampSupported)
        {
            readflag = xsens_IMU::getImuDataFrame(acc, gyro, quat,
                                                  current_time, current_acc_time,
                                                  current_gyro_time, current_quat_time);
        }
        else
        {
            readflag = HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
        }

        if (!readflag)
        {
            std::cerr << "采样 " << i + 1 << "/" << SAMPLE_COUNT << "：IMU数据读取失败!" << std::endl;
            readFailedCount++;
            continue;
        }

        // 记录Yaw角
        Eigen::Vector3d eulerAngles = quat.toRotationMatrix().eulerAngles(2, 1, 0);
        double current_yaw = eulerAngles[0];
        
        // 检查是否为第一个非零值
        if (!first_non_zero_found && std::abs(current_yaw) > 1e-6)
        {
            first_non_zero_found = true;
            std::cout << "✅ IMU数据初始化成功，检测到第一个有效Yaw值: " << current_yaw << " rad" << std::endl;
        }
        
        // 如果还没有找到非零值，检查超时
        if (!first_non_zero_found)
        {
            clock_gettime(CLOCK_MONOTONIC, &current_check_time);
            double elapsed_time = (current_check_time.tv_sec - start_time.tv_sec) + 
                                  (current_check_time.tv_nsec - start_time.tv_nsec) / 1e9;
            
            if (elapsed_time > 5.0) // 5秒超时
            {
                std::cout << RED << "❌ IMU初始化失败！等待有效数据超时（5秒）" << RESET << std::endl;
                std::cout << "可能原因：IMU设备未连接、驱动问题或设备故障" << std::endl;
                return false;
            }
            
            // 每100ms打印一次等待状态
            if (i % 100 == 0)
            {
                std::cout << "⏳ 等待IMU数据初始化... (" << std::fixed << std::setprecision(1) << elapsed_time << "s)" << std::endl;
            }
            continue; // 跳过当前采样，继续等待
        }
        
        yawAngles.push_back(current_yaw);
        
        // 减少刷屏：每100个采样打印一次进度
        if (i % 100 == 0 || i == SAMPLE_COUNT - 1)
        {
            std::cout << "采样 " << i + 1 << "/" << SAMPLE_COUNT << "，Yaw: " << current_yaw << " rad" << std::endl;
            last_printed_sample = i;
        }

        // 计算时间间隔（仅xsens）
        if (isTimestampSupported) {
            double acc_interval = (current_acc_time.tv_sec - last_acc_time.tv_sec) + 
                                  (current_acc_time.tv_nsec - last_acc_time.tv_nsec) / 1e9;
            double gyro_interval = (current_gyro_time.tv_sec - last_gyro_time.tv_sec) + 
                                   (current_gyro_time.tv_nsec - last_gyro_time.tv_nsec) / 1e9;
            double quat_interval = (current_quat_time.tv_sec - last_quat_time.tv_sec) + 
                                   (current_quat_time.tv_nsec - last_quat_time.tv_nsec) / 1e9;

            acc_intervals.push_back(acc_interval);
            gyro_intervals.push_back(gyro_interval);
            quat_intervals.push_back(quat_interval);

            // 更新上一次时间戳
            last_acc_time = current_acc_time;
            last_gyro_time = current_gyro_time;
            last_quat_time = current_quat_time;
        }
    }

    // 计算Yaw角漂移
    double maxYaw = *std::max_element(yawAngles.begin(), yawAngles.end());
    double minYaw = *std::min_element(yawAngles.begin(), yawAngles.end());
    double yawDrift = maxYaw - minYaw;

    // 输出采样统计
    std::cout << "采样统计:" << std::endl;
    std::cout << "IMU 总采样次数: " << SAMPLE_COUNT << ", 采样周期：" << 1 << " ms" << std::endl;
    std::cout << "数据读取失败次数: " << readFailedCount << std::endl;
    std::cout << "数据读取成功率: " << 
        (SAMPLE_COUNT > 0 ? (100.0 * (SAMPLE_COUNT - readFailedCount) / SAMPLE_COUNT) : 0.0) 
        << "%" << std::endl;

    // 频率统计（仅xsens）
    if (isTimestampSupported) 
    {
        auto calculateFrequencyStats = [](const std::vector<double>& intervals) {
            std::vector<double> frequencies;
            for (double interval : intervals) {
                if (interval > 0) {
                    frequencies.push_back(1.0 / interval);
                }
            }
            
            double min_freq = DBL_MAX;
            double max_freq = 0.0;
            double sum_freq = 0.0;
            
            for (double freq : frequencies) {
                min_freq = std::min(min_freq, freq);
                max_freq = std::max(max_freq, freq);
                sum_freq += freq;
            }
            
            double mean_freq = frequencies.empty() ? 0.0 : sum_freq / frequencies.size();
            
            return std::make_tuple(mean_freq, min_freq, max_freq);
        };

        // 计算并输出频率
        auto [acc_mean, acc_min, acc_max] = calculateFrequencyStats(acc_intervals);
        auto [gyro_mean, gyro_min, gyro_max] = calculateFrequencyStats(gyro_intervals);
        auto [quat_mean, quat_min, quat_max] = calculateFrequencyStats(quat_intervals);

        // 输出频率统计信息
        std::cout << "IMU更新频率统计:" << std::endl;
        std::cout << "加速度计: 均值=" << acc_mean << " Hz, 最小值=" << acc_min << " Hz, 最大值=" << acc_max << " Hz" << std::endl;
        std::cout << "陀螺仪:   均值=" << gyro_mean << " Hz, 最小值=" << gyro_min << " Hz, 最大值=" << gyro_max << " Hz" << std::endl;
        std::cout << "四元数:   均值=" << quat_mean << " Hz, 最小值=" << quat_min << " Hz, 最大值=" << quat_max << " Hz" << std::endl;

    } 
    else 
    {
        std::cout << YELLOW <<"当前IMU（HIPNUC）不支持时间戳，无法统计更新频率." << RESET << std::endl;
    }

    // Yaw漂移检测
    if (yawDrift > MAX_ALLOWED_DRIFT) 
    {
        std::cout << YELLOW << "Yaw角漂移检测失败！漂移量：" << yawDrift << " rad（允许最大值：" << MAX_ALLOWED_DRIFT << " rad）" << RESET << std::endl;
        std::cout << "可能原因：IMU未校准、陀螺仪零偏过大或设备晃动" << std::endl;
        return false;
    } 
    else 
    {
        std::cout << GREEN << "Yaw角稳定性检测通过！漂移量：" << yawDrift << " rad" << RESET << std::endl;
    }

    return true;
}

void hardwareSelfCheck::loadConfig()
{
    std::cout << "========= load config =========" << std::endl;
    RobotVersion rb_version(4, 5);
    if (nh_.hasParam("/robot_version"))
    {
        int rb_version_int;
        nh_.getParam("/robot_version", rb_version_int);
        rb_version = RobotVersion::create(rb_version_int);
    }
    ecmaster_type_ = motor_info.getEcmasterType(rb_version);
    setEcmasterDriverType(ecmaster_type_);

    kuavo_common_ptr_ = KuavoCommon::getInstancePtr(rb_version, ocs2::kuavo_assets::getPath());
    kuavo_settings_ = kuavo_common_ptr_->getKuavoSettings();
    motor_info = kuavo_settings_.hardware_settings;
    joint_ids_ = motor_info.joint_ids;
    imu_type_str_ = motor_info.getIMUType(rb_version);
    JSONConfigReader *robot_config = kuavo_common_ptr_->getRobotConfig();
    AnkleSolverType ankleSolverType = static_cast<AnkleSolverType>(robot_config->getValue<int>("ankle_solver_type"));
    ankleSolver_.getconfig(ankleSolverType);
    HighlyDynamic::HandProtocolType hand_protocol_type = motor_info.getHandProtocolType();
    is_can_protocol_ = hand_protocol_type == HighlyDynamic::HandProtocolType::PROTO_CAN;  
}

bool hardwareSelfCheck::initEndEffector()
{
    const auto &eef_types = motor_info.end_effector_type;

    // auto has_jodell = std::find(eef_types.begin(), eef_types.end(), EndEffectorType::jodell);
    auto has_qiangnao = std::find(eef_types.begin(), eef_types.end(), EndEffectorType::qiangnao);
    auto has_qiangnao_touch = std::find(eef_types.begin(), eef_types.end(), EndEffectorType::qiangnao_touch);

    std::vector<bool> eef_init_ok;
    // Init Qiangnao DexterousHand
    if (has_qiangnao != eef_types.end())
    {
        std::string gesture_filepath = kuavo_settings_.kuavo_assets_path + "/config/gesture/preset_gestures.json";
        dexhand_actuator_ = eef_controller::DexhandController::Create(gesture_filepath, false, is_can_protocol_);
        if (!dexhand_actuator_->init())
        {
            printf("############### initEndEffector Qiangnao DexterousHand init failed ###############\n");
            eef_init_ok.push_back(false);
        }
    }

    // FIXME: 当前不支持触觉手和非触觉手同时使用!!!!
    if (has_qiangnao_touch != eef_types.end())
    {
        if (has_qiangnao != eef_types.end())
        {
            // FIXME: 当前不支持触觉手和非触觉手同时使用!!!!
            std::cerr << "\033[33m当前不支持触觉手和非触觉手同时使用, 放弃初始化触觉灵巧手！\033[0m" << std::endl;
        }
        else
        {
            std::string gesture_filepath = kuavo_settings_.kuavo_assets_path + "/config/gesture/preset_gestures.json";
            // TODO: qiangnao_touch 触觉手固定使用 485 协议 (is_can_protocol = false)
            dexhand_actuator_ = eef_controller::DexhandController::Create(gesture_filepath, true, false);

            if (!dexhand_actuator_->init())
            {
                printf("[HardwarePlant] initEndEffector TouchDexhand init failed \n");
                eef_init_ok.push_back(false);
            }
            else
            {
                std::cout << "[HardwarePlant] initEndEffector TouchDexhand init success!! \r\n";
            }
        }
    }

    // WARNNING and TIPS, wait for user input 'enter' or 'Ctrl+C' to continue.
    if (!eef_init_ok.empty())
    {
        printf("\033[33m************************** !!! WARNING 警告 WARNING !!!! **************************\033[0m\n");
        printf("\033[33m[WARN] 机器人末端执行器初始化失败 \033[0m\n");
        return false;
    }
    return true;
}

bool hardwareSelfCheck::initMotor()
{
    // 初始化电机
    // 初始ruiwo电机

    const std::string hardware_node_path = ros::package::getPath("hardware_node");
    const std::string hardware_plant_path = std::filesystem::path(hardware_node_path).parent_path().string() + "/hardware_plant";

    setHardwarePlantPath(hardware_plant_path);
    auto has_ruiwo = std::find(motor_info.driver.begin(), motor_info.driver.end(), MotorDriveType::RUIWO);
    if (has_ruiwo != motor_info.driver.end())
    {
        bool cali_arm = false;
        if (nh_.hasParam("cali_arm"))
        {
            nh_.getParam("cali_arm", cali_arm);
        }
        ruiwo_actuator_ = new RuiWoActuator(GetAbsolutePathHW("lib/ruiwo_controller"), cali_arm);
        auto ret = ruiwo_actuator_->initialize();
        if (ret != 0)
        {
            std::cout << "############### ruiwo_actuator init failed ###############" << std::endl;
            return false;
        }
        auto motor_state_vec = ruiwo_actuator_->get_motor_state();
        bool ruiwo_enable_successed{true};
        std::cout << "ruiwo motor state[";
        for (const auto &motor : motor_state_vec)
        {
            if (RuiWoActuator::State::Enabled != motor.state)
            {
                ruiwo_enable_successed = false;
                std::cout << "FLASE ";
                continue;
            }
            std::cout << "TRUE ";
        }
        std::cout << "]" << std::endl;
        if(!ruiwo_enable_successed)
        {
            std::cout << "############### ruiwo_actuator motor state abnormal ###############" << std::endl;
            return false;
        }
        std::cout << "============= ruiwo_actuator init success ==============" << std::endl;
    }

    // 初始ECMaster电机
    actuatorsInterfaceSetup("real", &actuators_);
    // set ec_master encoder range
    std::vector<uint32_t> ecMasterEncoderRange;
    std::vector<uint16_t> ec_master_nil_ids{};
    uint32_t countECMasters = 0;

    for (size_t i = 0; i < motor_info.driver.size(); ++i)
    {
        if (motor_info.driver[i] == EC_MASTER)
        {
            int id = ec_index_map_[i];
            if (motor_info.motors_exist[i])
            {
                ecMasterEncoderRange.push_back(motor_info.encoder_range[i]);
                countECMasters++;
            }
            if (motor_info.motors_disable[i])
            {
                ec_master_nil_ids.push_back(id);
            }
        }
    }

    // add ignore ids for ec master.
    actuators_.addIgnore(ec_master_nil_ids.data(), ec_master_nil_ids.size());
    actuators_.setEncoderRange(ecMasterEncoderRange.data(), ecMasterEncoderRange.size());

    const auto &joint_kp_source =
        (kuavo_settings_.running_settings.use_vr_arm_kpkd &&
         !kuavo_settings_.running_settings.vr_joint_kp.empty() &&
         !kuavo_settings_.running_settings.vr_joint_kd.empty() &&
         kuavo_settings_.running_settings.vr_joint_kp.size() == kuavo_settings_.running_settings.vr_joint_kd.size())
            ? kuavo_settings_.running_settings.vr_joint_kp
            : kuavo_settings_.running_settings.joint_kp;
    const auto &joint_kd_source =
        (kuavo_settings_.running_settings.use_vr_arm_kpkd &&
         !kuavo_settings_.running_settings.vr_joint_kp.empty() &&
         !kuavo_settings_.running_settings.vr_joint_kd.empty() &&
         kuavo_settings_.running_settings.vr_joint_kp.size() == kuavo_settings_.running_settings.vr_joint_kd.size())
            ? kuavo_settings_.running_settings.vr_joint_kd
            : kuavo_settings_.running_settings.joint_kd;

    std::vector<int32_t> robot_running_mode_joint_kp(joint_kp_source.begin(), joint_kp_source.end());
    std::vector<int32_t> robot_running_mode_joint_kd(joint_kd_source.begin(), joint_kd_source.end());
    actuators_.setJointKp(robot_running_mode_joint_kp);
    actuators_.setJointKd(robot_running_mode_joint_kd);

    std::cout << "actuators_InterfaceSetup DONE." << std::endl;
    std::cout << "actuators_InterfaceSetup countECMasters: " << countECMasters << std::endl;

    EcActuatorParams params;
    params.num_actuators = countECMasters;
    params.ec_type = ecmaster_type_;
    params.robot_module = motor_info.robot_module; // 从motor_info中获取robot_module
    if (actuators_.init(params) != 0)
    {
        std::cout << "############### actuators_ init failed ###############" << std::endl;
        return false;
    }

    return true;
}

void hardwareSelfCheck::setEcmasterDriverType(const std::string &type)
{
    if (type != "elmo" && type != "youda")
    {
        std::cerr << "Unsupported ecmaster driver type: " << type << std::endl;
        exit(1);
    }
    std::cout << "set ecmaster driver type to: " << type << std::endl;
    size_t driver_type_length = sizeof(driver_type) / sizeof(driver_type[0]);
    for (size_t i = 0; i < driver_type_length; i++)
    {
        if (type == "elmo")
        {
            driver_type[i] = EcMasterType::ELMO;
        }
        else
        {
            driver_type[i] = EcMasterType::YD;
        }
    }
}

bool hardwareSelfCheck::initIMU()
{
    std::cout << "start init imu." << std::endl;

    bool ret = (0 == ((imu_type_str_ == "xsens")? xsens_IMU::imu_init(): HIPNUC_IMU::imu_init())) ? true : false; // TODO: better imu init
    if (!ret)
    {
        std::cerr << "############### imu init failed ###############" << std::endl;
        return false;
    }
    std::cout << "============= imu init done =============" << std::endl;

    ret = detectYawDrift();
    if (!ret)
    {
        return false;
    }

    std::cout << "================ imu init success ===============" << std::endl;
    return true;
}

bool hardwareSelfCheck::waitMoveDone(const std::vector<double> &tarPos)
{
    std::vector<JointParam_t> cur_joint_data(num_joint_);
    bool bIsMoveDone{false};
    bool bIsMotionTimeOut{false};
    ros::Time startTime = ros::Time::now();
    while (!bIsMoveDone && !bIsMotionTimeOut)
    {
        bIsMoveDone = true;
        GetJointData(joint_ids_, cur_joint_data);
        for (int i = 0; i < num_joint_; i++)
        {
            auto tarQ = tarPos[i];
            auto curQ = cur_joint_data[i].position;
            if (fabs(tarQ - curQ) > 1)
            {
                bIsMoveDone = false;
            }
            // std::cout << "joint " << i << " pos:[" << curQ << "] to tar pos[" << tarQ << "]" << std::endl;
        }

        bIsMotionTimeOut = (ros::Time::now() - startTime > ros::Duration(5));
        usleep(1000);
    }

    if (bIsMotionTimeOut)
    {
        std::cout << YELLOW << "motion timeout." << RESET << std::endl;
        for (int i = 0; i < num_joint_; i++)
        {
            auto tarQ = tarPos[i];
            auto curQ = cur_joint_data[i].position;
            if (fabs(tarQ - curQ) > 1)
            {
                std::cout << YELLOW << "joint " << i << " pos:[" << curQ << "] did not reach the tar pos[" << tarQ << "]" << RESET << std::endl;
            }
        }
        return false;
    }

    std::cout << GREEN << "move done" << RESET << std::endl;
    return true;
}

int main(int argc, char **argv)
{
    // 初始ros节点
    ros::init(argc, argv, "hardwareSelfCheck");
    ros::NodeHandle nh;

    double dt{0.002};
    std::unique_ptr<hardwareSelfCheck> self_check = std::make_unique<hardwareSelfCheck>(dt, nh);

    std::cout << "====================== start self-check =====================" << std::endl;

    usleep(2000000);
    //初始化IMU
    bool bInitIMU= self_check->initIMU();

    //初始化灵巧手
    bool bInitEndEffector= self_check->initEndEffector();

    //初始化Motor
    bool bInitMotor = self_check->initMotor();
    if (bInitIMU)
    {
        std::cout << GREEN << "IMU 初始化成功!!!" << RESET << std::endl;
    }
    else
    {
        std::cout << RED << "IMU 初始化失败!!!" << RESET << std::endl;
    }

    if (bInitEndEffector)
    {
        std::cout << GREEN << "末端执行器(灵巧手)初始化成功!!!" << RESET << std::endl;
    }
    else
    {
        std::cout << RED << "末端执行器(灵巧手)初始化失败!!!" << RESET << std::endl;
    }

    if (bInitMotor)
    {
        std::cout << GREEN << "电机初始化成功!!!" << RESET << std::endl;
    }
    else
    {
        std::cout << RED << "电机初始化失败!!!" << RESET << std::endl;
        return 0;
    }

    // if (!bInitIMU || !bInitEndEffector || !bInitMotor)
    // {
    //     std::cout << RED << "Robot init failed, exit self-check." << RESET << std::endl;
    //     return 1;
    // }

    int num_joints{28};
    auto joint_ids = self_check->getJointIds();
    std::vector<JointParam_t> joint_data(num_joints);
    self_check->GetJointData(joint_ids, joint_data);

    std::cout << "测试手臂电机>>" << std::endl;
    std::vector<double> test_arm_pos1(num_joints);
    test_arm_pos1 =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      -5.0, 5.0, -5.0, -5.0, -5.0, 5.0, -5.0,
                      -5.0, -5.0, 5.0, -5.0, 5.0, -5.0, -5.0,
                      -3.0, -3.0};

    auto initPos = self_check->getCurJointPos();
    self_check->jointMoveTo(test_arm_pos1, 60, dt);

    auto curPos = self_check->getCurJointPos();
    bool bIsArmMoveTestFailedFlag{false};
    for (uint32_t i = 12; i < 28; i++)
    {
        if (fabs(curPos[i] - initPos[i]) < 0.1)
        {
            std::cout << YELLOW << "手臂关节 " << i << " 没有移动, 起点位置: " << initPos[i] << ", 当前位置: "<<curPos[i] << RESET << std::endl;
            bIsArmMoveTestFailedFlag = true;
        }
        else
        {
            std::cout << "手臂关节 " << i << " 成功移动, 起点位置: " << initPos[i] << ", 当前位置: "<<curPos[i] << std::endl;
        }
    }

    usleep(2000000);
    if(!bIsArmMoveTestFailedFlag)
    {
        std::cout << GREEN << "测试手臂电机成功!!!" << RESET << std::endl;
    }
    else
    {
        std::cout << RED << "测试手臂电机失败, 请用户确认相应关节是否使能!!!" << RESET << std::endl;
    }

    std::cout << "测试腿部电机>>" << std::endl;
    std::cout << "当前cali位置是否正确，[y/n] 输入[y]进入到准备姿态(执行完后可以输入[x]退出程序)" << std::endl;
    std::vector<double> cali_pos(num_joints, 0.0);
    self_check->jointMoveTo(cali_pos, 60, dt);

    // 等待用户输入判断运动位置是否正确
    char user_input;
    std::cin >> user_input;
    // 清空输入缓冲区，避免读取到换行符
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (user_input == 'y' || user_input == 'Y')
    {
        std::cout << GREEN << "用户确认校准位置正确，继续后续操作。" << RESET << std::endl;
        std::cout << "移动到准备姿态 ..." << std::endl;
        std::vector<double> Ready_pos = {
            1.61187, 0.0843823, -60.5829, 107.461, -49.8766, -1.6141,
            -1.45677, -0.076402, -60.6254, 107.512, -49.8855, 1.45869,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // 人工判断姿态2是否正确
        std::cout << "准备姿态位置是否正确，[y/n] 输入[y]确认腿部电机测试成功" << std::endl;
        self_check->jointMoveTo(Ready_pos, 60, dt);

        std::cin >> user_input;
        // 清空输入缓冲区，避免读取到换行符
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        if (user_input == 'y' || user_input == 'Y')
        {
            std::cout << GREEN << "用户确认准备姿态正确 - 腿部电机测试通过!!!" << RESET << std::endl;
        }
        else if (user_input == 'n' || user_input == 'N')
        {
            std::cout << RED << "用户确认准备姿态不正确 - 腿部电机测试失败!!!" << RESET << std::endl;
        }
        else
        {
            std::cout << YELLOW << "输入无效，请输入 'y' 或 'n'。" << RESET << std::endl;
            return 0;
        }
    }
    else if (user_input == 'n' || user_input == 'N')
    {
        std::cout << RED << "用户确认校准位置不正确，退出自检。" << RESET << std::endl;
        return 0;
    }
    else
    {
        std::cout << YELLOW << "输入无效，请输入 'y' 或 'n'。" << RESET << std::endl;
        return 0;
    }
    
    std::cout << "输入x退出程序：";
    std::cin >> user_input;
    // 清空输入缓冲区，避免读取到换行符
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "测试完毕.";
    usleep(1000000);

    return 0;
}
