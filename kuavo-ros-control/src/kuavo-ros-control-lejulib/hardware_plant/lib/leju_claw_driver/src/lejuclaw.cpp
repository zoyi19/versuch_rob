#include "lejuclaw.h"
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <iomanip>
#define DISABLE_ADDRESS 0x00

const float dt = 0.001;    // 控制周期
const float max_speed = 200; // 插值规划的速度
const float velocity_factor = 0.01;
RUIWOTools ruiwo;

LeJuClaw::LeJuClaw(std::string unused)
{
    // 初始化VR控制相关变量
    last_target_update_time = std::chrono::steady_clock::now();
    is_vr_control_mode = false;
    movement_start_time = std::chrono::steady_clock::now();
    movement_timeout_enabled = false;
}

LeJuClaw::~LeJuClaw()
{
    thread_running = false;
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
}

int LeJuClaw::initialize(bool init_bmlib)
{
    initialize_ruiwoSDK(&ruiwo, true);
    std::string path = get_home_path();
    if (path.empty())
    {
        std::cout << "Failed to get home path." << std::endl;
        return 1;
    }
    std::string config_file = "config.yaml";
    std::string config_path = path + "/" + config_file;
    std::string leju_param_file = "lejuclaw_config.yaml";
    std::string leju_param_path = path + "/" + leju_param_file;

    std::filesystem::path yaml_file_path(config_path);
    if (!std::filesystem::exists(yaml_file_path))
    {
        std::cout << "[LEJU claw] config file is not exist, path:" << config_path << "\n";
        std::cout << "[LEJU claw] 请从`lib/ruiwo_controller` 或 `lib/ruiwo_controller_cxx` 拷贝对应的 config.yaml 到配置路径\n";
        return 1;
    }

    std::cout << "[LEJU claw] config_path:" << config_path << "\n";
    // using motor info from config file
    // get_parameter();
    get_config(config_path);

    // 加载可选的夹爪运行参数文件（不存在则忽略，保持默认值）
    try {
        std::filesystem::path leju_yaml_path(leju_param_path);
        if (std::filesystem::exists(leju_yaml_path)) {
            std::cout << "[LEJU claw] lejuclaw_config_path:" << leju_param_path << "\n";
            load_leju_params(leju_param_path);
        } else {
            std::cout << "[LEJU claw] lejuclaw_config.yaml 未找到，使用头文件默认参数" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cout << "[LEJU claw] 读取 lejuclaw_config.yaml 发生异常: " << e.what() << std::endl;
    }
    thread_running = true;
    thread_end = false;

    std::cout << "[LEJU claw]:Control mode:" << Control_mode << std::endl;
    std::cout << "[LEJU claw]:Negtive joint ID:";
    for (int id : Negtive_joint_address_list)
    {
        std::cout << id << " ";
    }
    std::cout << std::endl;

    std::cout << "---------------INTIALIZED START---------------" << std::endl;
    unsigned char version[4];
    version[0] = 0x02;
    version[1] = 0x02;
    version[2] = 0x04;
    version[3] = 0x0B;
    auto errorcode = open_canbus(&ruiwo, (init_bmlib ? 1: 0), version);
    if (errorcode != 0)
    {
        std::cout << "[LEJU claw]:Canbus status: [ " << errorcode << " ]" << std::endl;
        return 1;
    }
    std::cout << "[LEJU claw]:Canbus status: [ Success ]" << std::endl;
    target_update = false;
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    this->set_claw_kpkd(0, 0);
    go_to_zero_with_current_control();
    
    // 初始化完成后，确保所有电流都清零
    clear_all_torque();
    
    std::cout << "---------------INTIALIZED DONE---------------" << std::endl;

    // 启动控制线程
    control_thread_ = std::thread(&LeJuClaw::control_thread, this);
    
    // 等待控制线程启动
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // 使用move_paw发布位置指令，让夹爪打开到中间位置
    std::cout << "[LEJU claw]:初始化完成后，发布位置指令，目标位置中间位置" << std::endl;
    
    // 创建目标位置向量
    std::vector<double> target_positions(Joint_address_list.size(), 50.0);
    std::vector<double> target_velocity(Joint_address_list.size(), 0.0);
    std::vector<double> target_torque(Joint_address_list.size(), 0.0);
    auto result = move_paw(target_positions, target_velocity, target_torque);
    
    return 0;
}

std::vector<int> LeJuClaw::get_joint_addresses(const YAML::Node &config, const std::string &joint_type, int count)
{
    std::vector<int> addresses;
    for (int i = 0; i < count; ++i)
    {
        std::string key = joint_type + "_" + std::to_string(i + 1);
        addresses.push_back(config["address"][key].as<int>());
    }
    return addresses;
}

std::vector<bool> LeJuClaw::get_joint_online_status(const YAML::Node &config, const std::string &joint_type, int count)
{
    std::vector<bool> online_status;
    for (int i = 0; i < count; ++i)
    {
        std::string key = joint_type + "_" + std::to_string(i + 1);
        online_status.push_back(config["online"][key].as<bool>());
    }
    return online_status;
}

std::vector<std::vector<int>> LeJuClaw::get_joint_parameters(const YAML::Node &config, const std::string &joint_type, int count)
{
    std::vector<std::vector<int>> parameters;
    for (int i = 0; i < count; ++i)
    {
        std::string key = joint_type + "_" + std::to_string(i + 1);
        parameters.push_back(config["parameter"][key].as<std::vector<int>>());
    }
    return parameters;
}

// void LeJuClaw::get_parameter()
// {
//     // Left_joint_address = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
//     // Right_joint_address = {0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
//     // Head_joint_address = {0x0D, 0x0E};
//     // Left_joint_parameter = {{0, 25, 8, 0, 0, 0, 0},
//     //                         {0, 20, 6, 0, 0, 0, 0},
//     //                         {0, 20, 6, 0, 0, 0, 0},
//     //                         {0, 10, 3, 0, 0, 0, 0},
//     //                         {0, 10, 3, 0, 0, 0, 0},
//     //                         {0, 10, 3, 0, 0, 0, 0}};
//     // Right_joint_parameter = {{0, 25, 8, 0, 0, 0, 0},
//     //                          {0, 20, 6, 0, 0, 0, 0},
//     //                          {0, 20, 6, 0, 0, 0, 0},
//     //                          {0, 10, 3, 0, 0, 0, 0},
//     //                          {0, 10, 3, 0, 0, 0, 0},
//     //                          {0, 10, 3, 0, 0, 0, 0}};
//     // Head_joint_parameter = {{0, 4, 3, 0, 0, 0, 0},
//     //                         {0, 10, 6, 0, 0, 0, 0}};
//     // Negtive_joint_address_list = {};
//     // Unnecessary_go_zero_list = {0x04, 0x05, 0x06, 0x0A, 0x0B, 0x0C};
//     // Control_mode = "ptm";

//     Left_joint_online = {false, false, false, false, false, false};
//     Right_joint_online = {false, false, false, false, false, false};
//     Head_joint_online = {false, false};
//     Joint_parameter_list = Left_joint_parameter;
//     Joint_parameter_list.insert(Joint_parameter_list.end(), Right_joint_parameter.begin(), Right_joint_parameter.end());
//     Joint_parameter_list.insert(Joint_parameter_list.end(), Head_joint_parameter.begin(), Head_joint_parameter.end());
//     Joint_address_list = Left_joint_address;
//     Joint_address_list.insert(Joint_address_list.end(), Right_joint_address.begin(), Right_joint_address.end());
//     Joint_address_list.insert(Joint_address_list.end(), Head_joint_address.begin(), Head_joint_address.end());
//     Joint_online_list = Left_joint_online;
//     Joint_online_list.insert(Joint_online_list.end(), Right_joint_online.begin(), Right_joint_online.end());
//     Joint_online_list.insert(Joint_online_list.end(), Head_joint_online.begin(), Head_joint_online.end());
// }

void LeJuClaw::get_config(const std::string &config_file)
{
     try {
        YAML::Node motor_config = YAML::LoadFile(config_file);

        auto address_items = motor_config["address"].as<std::map<std::string, int>>();

        auto online_items  = motor_config["online"].as<std::map<std::string, bool>>();
        auto parameter_items = motor_config["parameter"].as<std::map<std::string, std::vector<int>>>();
        
        Claw_joint_address.clear();
        Claw_joint_online.clear();

        Joint_online_list.clear();
        Joint_parameter_list.clear();
        Joint_address_list.clear();
        Negtive_joint_address_list.clear();

        for (auto online_item : online_items) {
            if (online_item.first.find("Claw_joint") != std::string::npos) {
                Claw_joint_online.push_back(online_item.second);
                
            }
            Joint_online_list.push_back(online_item.second);
        }
        

        for (auto parameter_item : parameter_items) {
            if (parameter_item.first.find("Claw_joint") != std::string::npos) {
                Joint_parameter_list.push_back(parameter_item.second);
            }
        }

        // 只处理夹爪电机的地址
        for (auto address_item : address_items) {
            if (address_item.first.find("Claw_joint") != std::string::npos) {
                Claw_joint_address.push_back(address_item.second);
                Joint_address_list.push_back(address_item.second);
            }
            
        }
        
        // 其他配置项        
        // const YAML::Node& negtive_address = motor_config["negtive_address"];
        // if(negtive_address.size()) {
        //     const YAML::Node& negtive_motor_list = negtive_address[0];
        //     for (size_t j = 0; j < negtive_motor_list.size(); ++j) {
        //        int motor_id = negtive_motor_list[j].as<int>();
        //        Negtive_joint_address_list.push_back(motor_id); 
        //     }
        // }
        
        // 控制模式
        Control_mode = motor_config["control_mode"].as<std::string>();

        for (auto const &id: disable_joint_ids) {
            Joint_address_list[id - 1] = DISABLE_ADDRESS;
        }

    }catch (const YAML::Exception &e) {
        std::cout << "[LEJU claw]:YAML::Exception:" << e.what() << std::endl;
    }
    catch (const std::exception& e) {
        std::cout << "[LEJU claw] Exception:" << e.what() << std::endl;
    }
}

void LeJuClaw::load_leju_params(const std::string &config_file)
{
    try {
        YAML::Node cfg = YAML::LoadFile(config_file);
        // 可采用平铺或lejuclaw命名空间两种写法
        YAML::Node root = cfg["lejuclaw"].IsDefined() ? cfg["lejuclaw"] : cfg;

        auto set_float = [&](const char* key, float &dst){ if (root[key].IsDefined()) dst = root[key].as<float>(); };
        auto set_int   = [&](const char* key, int   &dst){ if (root[key].IsDefined()) dst = root[key].as<int>(); };
        auto set_bool  = [&](const char* key, bool  &dst){ if (root[key].IsDefined()) dst = root[key].as<bool>(); };

        set_float("kp", cfg_KP);
        set_float("kd", cfg_KD);
        set_float("alpha", cfg_ALPHA);
        set_float("max_current", cfg_MAX_CURRENT);
        set_float("min_error_percent", cfg_MIN_ERROR_PERCENT);
        set_float("dt", cfg_DT);

        set_float("limit_range_percent", cfg_LIMIT_RANGE_PERCENT);
        set_float("impact_current", cfg_IMPACT_CURRENT);
        set_float("impact_duration_ms", cfg_IMPACT_DURATION_MS);
        set_float("impact_interval_ms", cfg_IMPACT_INTERVAL_MS);

        set_float("target_position_threshold", cfg_TARGET_POSITION_THRESHOLD);
        set_float("target_position_detection_time_ms", cfg_TARGET_POSITION_DETECTION_TIME_MS);
        set_float("stable_position_threshold", cfg_STABLE_POSITION_THRESHOLD);
        set_float("stable_velocity_threshold", cfg_STABLE_VELOCITY_THRESHOLD);
        set_float("stable_current_threshold", cfg_STABLE_CURRENT_THRESHOLD);
        set_float("stable_detection_time_ms", cfg_STABLE_DETECTION_TIME_MS);
        set_float("grip_holding_current", cfg_GRIP_HOLDING_CURRENT);
        set_bool("enable_100_position_holding_current", cfg_ENABLE_100_POSITION_HOLDING_CURRENT);
        set_int("move_paw_timeout_ms", cfg_MOVE_PAW_TIMEOUT_MS);

        set_int("vr_control_timeout_ms", cfg_VR_CONTROL_TIMEOUT_MS);
        set_int("vr_stuck_detection_cycles", cfg_VR_STUCK_DETECTION_CYCLES);
        set_float("vr_stuck_position_threshold", cfg_VR_STUCK_POSITION_THRESHOLD);
        set_float("vr_target_position_threshold", cfg_VR_TARGET_POSITION_THRESHOLD);
        set_float("vr_grip_position_threshold", cfg_VR_GRIP_POSITION_THRESHOLD);
        set_float("vr_grip_holding_current", cfg_VR_GRIP_HOLDING_CURRENT);

        set_float("zero_control_kp", cfg_ZERO_CONTROL_KP);
        set_float("zero_control_kd", cfg_ZERO_CONTROL_KD);
        set_float("zero_control_alpha", cfg_ZERO_CONTROL_ALPHA);
        set_float("zero_control_max_current", cfg_ZERO_CONTROL_MAX_CURRENT);
        set_float("stall_current_threshold", cfg_STALL_CURRENT_THRESHOLD);
        set_float("stall_velocity_threshold", cfg_STALL_VELOCITY_THRESHOLD);
        set_float("zero_control_dt", cfg_ZERO_CONTROL_DT);
        set_int("zero_find_timeout_ms", cfg_ZERO_FIND_TIMEOUT_MS);
        set_int("zero_wait_ms", cfg_ZERO_WAIT_MS);
        set_float("open_limit_adjustment", cfg_OPEN_LIMIT_ADJUSTMENT);
        set_float("close_limit_adjustment", cfg_CLOSE_LIMIT_ADJUSTMENT);
        set_float("target_velocity", cfg_TARGET_VELOCITY);
        set_float("open_position_change_threshold", cfg_OPEN_POSITION_CHANGE_THRESHOLD);
        set_float("close_stuck_current_threshold", cfg_CLOSE_STUCK_CURRENT_THRESHOLD);
        set_float("close_stuck_detection_threshold", cfg_CLOSE_STUCK_DETECTION_THRESHOLD);
        set_int("close_startup_delay_ms", cfg_CLOSE_STARTUP_DELAY_MS);
        set_float("close_impact_current", cfg_CLOSE_IMPACT_CURRENT);
        set_int("close_impact_duration_ms", cfg_CLOSE_IMPACT_DURATION_MS);
        set_int("close_impact_interval_ms", cfg_CLOSE_IMPACT_INTERVAL_MS);
        set_int("close_max_attempts", cfg_CLOSE_MAX_ATTEMPTS);

        // 渐变刹车参数（1.2.2机制）
        set_float("brake_range_percent", cfg_BRAKE_RANGE_PERCENT);
        set_float("brake_min_speed_factor", cfg_BRAKE_MIN_SPEED_FACTOR);
        set_float("brake_curve_exponent", cfg_BRAKE_CURVE_EXPONENT);
        set_float("target_proximity_brake_percent", cfg_TARGET_PROXIMITY_BRAKE_PERCENT);
        set_float("target_proximity_min_factor", cfg_TARGET_PROXIMITY_MIN_FACTOR);

        // STUCK params
        set_float("stuck_detection_delay_ms", cfg_STUCK_DETECTION_DELAY_MS);
        set_float("stuck_detection_time_ms", cfg_STUCK_DETECTION_TIME_MS);
        set_float("stuck_position_threshold", cfg_STUCK_POSITION_THRESHOLD);

        std::cout << "[LEJU claw] lejuclaw_config.yaml 参数加载完成" << std::endl;
    } catch (const YAML::Exception &e) {
        std::cout << "[LEJU claw] 解析 lejuclaw_config.yaml 出错: " << e.what() << std::endl;
    } catch (const std::exception &e) {
        std::cout << "[LEJU claw] 读取 lejuclaw_config.yaml 异常: " << e.what() << std::endl;
    }
}

std::string LeJuClaw::get_home_path()
{
    const char *sudo_user = getenv("SUDO_USER");
    if (sudo_user != nullptr)
    {
        passwd *pw = getpwnam(sudo_user);
        if (pw != nullptr)
        {
            std::string path = pw->pw_dir;
            path += "/.config/lejuconfig";
            return path;
        }
    }
    else
    {
        uid_t uid = getuid();
        passwd *pw = getpwuid(uid);
        if (pw != nullptr)
        {
            std::string path = pw->pw_dir;
            path += "/.config/lejuconfig";
            return path;
        }
    }
    return "";
}

void LeJuClaw::control_thread()
{
    std::cout << "[LEJU claw]:Threadstart succeed" << std::endl;
    std::vector<float> target_positions_copy;
    std::vector<float> target_torque_copy;
    std::vector<float> target_velocity_copy;

    // 定期读取反馈的定时器（20ms读取一次，即50Hz）
    const int FEEDBACK_READ_INTERVAL_MS = 20;
    auto last_feedback_read_time = std::chrono::steady_clock::now();

    try
    {
        std::vector<float> current_positions_copy(Joint_address_list.size(), 0);
        std::vector<float> current_torque_copy(Joint_address_list.size(), 0);
        std::vector<float> current_velocity_copy(Joint_address_list.size(), 0);

        while (thread_running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1)));    

            // 读取当前位置
            {
                std::lock_guard<std::mutex> lock(state_lock);
                std::vector<std::vector<float>> joint_state_copy = joint_status;

                for (size_t i = 0; i < Joint_address_list.size(); ++i)
                {
                    if (Joint_online_list[i])
                    {
                        const std::vector<float> &motor = joint_state_copy[i];
                        current_positions_copy[i] = motor[1];
                        current_velocity_copy[i] = motor[2];
                        current_torque_copy[i] = motor[3];
                    }
                }
            }

            {
                std::lock_guard<std::mutex> lock(recvpos_lock);
                current_positions = current_positions_copy;
            }
            {
                std::lock_guard<std::mutex> lock(recvtor_lock);
                current_torque = current_torque_copy;
            }
            {
                std::lock_guard<std::mutex> lock(recvvel_lock);
                current_velocity = current_velocity_copy;
            }

            // 检查是否需要持续发送保持电流（即使target_update为false）
            bool need_send_holding_current = false;
            {
                std::lock_guard<std::mutex> lock(sendtor_lock);
                for (size_t i = 0; i < Joint_address_list.size(); ++i)
                {
                    if (Joint_online_list[i] && is_holding_object_global[i])
                    {
                        // 需要保持电流，使用存储的维持电流值
                        target_torque[i] = holding_current_values[i];
                        need_send_holding_current = true;
                    }
                }
            }
            
            // 检查是否需要定期读取反馈（即使没有控制命令）
            auto current_time = std::chrono::steady_clock::now();
            auto time_since_last_read = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_feedback_read_time).count();
            bool need_read_feedback = (time_since_last_read >= FEEDBACK_READ_INTERVAL_MS);
            
            // 如果只需要读取反馈（没有控制命令），使用enter_motor_state只读取状态
            if (need_read_feedback && !target_update && !need_send_holding_current)
            {
                // 只读取反馈，不发送控制命令
                {
                    std::lock_guard<std::mutex> lock(can_lock);
                    for (size_t i = 0; i < Joint_address_list.size(); ++i)
                    {
                        if (Joint_online_list[i])
                        {
                            float state_list[6];
                            int errorcode = enter_motor_state(&ruiwo, Joint_address_list[i], state_list, NULL);
                            if (errorcode == 0)
                            {
                                std::vector<float> state;
                                state.assign(state_list, state_list + 6);
                                set_joint_state(i, state);
                            }
                        }
                    }
                }
                last_feedback_read_time = current_time;
                continue;
            }
            
            // 如果有控制命令需要发送，正常处理
            if (!target_update && !need_send_holding_current)
            {
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(sendpos_lock);
                target_positions_copy = target_positions;
                target_torque_copy = target_torque;
                target_velocity_copy = target_velocity;
                if (target_update) {
                    target_update = false;  // 只在有更新时重置标志
                }
            }

            std::vector<int> index(Joint_address_list.size());
            for (size_t i = 0; i < Joint_address_list.size(); ++i)
            {
                index[i] = i;
            }
            
            {
                std::lock_guard<std::mutex> lock(can_lock);
                send_positions(index, target_positions_copy, target_torque_copy, target_velocity_copy);
            }
            
            // 更新反馈读取时间（发送命令时也会更新状态，所以也更新读取时间）
            if (need_read_feedback || target_update || need_send_holding_current) {
                last_feedback_read_time = current_time;
            }
        
        }
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    thread_end = true;
    std::cout << "[LEJU claw]:Threadend succeed" << std::endl;
}

// void LeJuClaw::join() {
//     if (control_thread_.joinable()) {
//         control_thread_.join();
//     }
// }

std::vector<std::vector<float>> LeJuClaw::interpolate_positions_with_speed(
    const std::vector<float> &a, 
    const std::vector<float> &b, 
    const std::vector<float> &speeds, 
    float dt)
{
    std::vector<float> a_vec = a;
    std::vector<float> b_vec = b;
    std::vector<float> c_vec = speeds;
    int num_joints = a.size();

    // // 打印起始位置
    // std::cout << "Start positions: ";
    // for (int i = 0; i < num_joints; ++i)
    // {
    //     std::cout << a_vec[i] << " ";
    // }
    // std::cout << std::endl;

    // // 打印终点位置
    // std::cout << "End positions: ";
    // for (int i = 0; i < num_joints; ++i)
    // {
    //     std::cout << b_vec[i] << " ";
    // }
    // std::cout << std::endl;
    //     // 打印终点位置
    // std::cout << "speed: ";
    // for (int i = 0; i < num_joints; ++i)
    // {
    //     std::cout << speeds[i] << " ";
    // }
    // std::cout << std::endl;
    // 计算每个关节的时间
    std::vector<float> times(num_joints, 0.0f);
    for (int i = 0; i < num_joints; ++i)
    {
        float distance = std::abs(b_vec[i] - a_vec[i]);
        times[i] = distance / speeds[i];  // 计算每个关节的时间
    }

    // 总时间为所有关节中最大的时间
    float total_time = *std::max_element(times.begin(), times.end());

    // 根据总时间和时间步长计算实际的时间步数
    int steps = static_cast<int>(total_time / dt) + 1;
    //std::cout << "Total steps:\n " << steps << std::endl;
     // 打印最大时间
    //std::cout << "Total time: " << total_time << std::endl;
    // 使用线性插值计算每个时间步的关节位置
    std::vector<std::vector<float>> interpolation(steps, std::vector<float>(num_joints));
    for (int i = 0; i < steps; ++i)
    {
        float t = static_cast<float>(i) / (steps - 1); // 归一化时间比例 [0, 1]
        for (int j = 0; j < num_joints; ++j)
        {
            float joint_time = times[j];
            float joint_t = std::min(t * (total_time / joint_time), 1.0f); // 对每个关节单独插值
            interpolation[i][j] = a_vec[j] + joint_t * (b_vec[j] - a_vec[j]);
        }
    }

    return interpolation;
}

void LeJuClaw::go_to_zero_with_current_control()
{
    if (!thread_running)
    {
        return;
    }

    std::cout << "[LEJU claw]:开始夹爪限位寻找初始化..." << std::endl;

    // 初始化电机状态
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            float state_list[6];
            std::lock_guard<std::mutex> lock(can_lock);
            int errorcode = enter_motor_state(&ruiwo, Joint_address_list[i], state_list, NULL);
            if (errorcode != 0)
            {
                std::cerr << "Joint " << Joint_address_list[i] << " 初始化失败" << std::endl;
                return;
            }
            set_joint_state(i, std::vector<float>(state_list, state_list + 6));
        }
    }

    // 记录初始位置
    std::vector<float> initial_positions = create_zero_vector(Joint_address_list.size());
    std::vector<float> last_positions = create_zero_vector(Joint_address_list.size());
    
    std::vector<std::vector<float>> state = get_joint_state();
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            initial_positions[i] = state[i][1];
            last_positions[i] = state[i][1];
        }
    }

    // 寻找开爪限位（负速度）
    std::cout << "[LEJU claw]:开始寻找开爪限位，超时时间设置为: " << cfg_ZERO_FIND_TIMEOUT_MS << " ms..." << std::endl;
    
    // 记录开爪开始位置，用于后续分析位置变化
    std::vector<float> open_start_positions(Joint_address_list.size(), 0.0f);
    state = get_joint_state();
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            open_start_positions[i] = state[i][1];
            std::cout << "[LEJU claw]:关节 " << Joint_address_list[i] << " 开爪开始位置: " << open_start_positions[i] << std::endl;
        }
    }
    
    bool open_limit_found = find_claw_limit_velocity_control(true, cfg_ZERO_CONTROL_KP, cfg_ZERO_CONTROL_KD, cfg_ZERO_CONTROL_ALPHA, cfg_ZERO_CONTROL_MAX_CURRENT, 
                                                           cfg_STALL_CURRENT_THRESHOLD, cfg_STALL_VELOCITY_THRESHOLD, cfg_ZERO_CONTROL_DT, cfg_ZERO_FIND_TIMEOUT_MS);
    if (!open_limit_found)
    {
        std::cout << "[LEJU claw]:开爪限位寻找超时，认为当前位置为开爪限位，继续寻找关爪限位" << std::endl;
    }

    // 记录开爪限位位置并分析位置变化
    std::vector<float> open_limit_positions(Joint_address_list.size(), 0.0f);
    state = get_joint_state();
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            open_limit_positions[i] = state[i][1];
            float position_change = std::abs(open_limit_positions[i] - open_start_positions[i]);
            std::cout << "[LEJU claw]:关节 " << Joint_address_list[i] << " 开爪限位位置: " << open_limit_positions[i] << std::endl;
            std::cout << "[LEJU claw]:关节 " << Joint_address_list[i] << " 开爪位置变化: " << position_change << " rad" << std::endl;
        }
    }

    // 等待一段时间后开始关爪
    std::this_thread::sleep_for(std::chrono::milliseconds(cfg_ZERO_WAIT_MS));
    
    // 检查开爪位置变化情况，决定是否执行高电流冲关爪
    std::vector<bool> can_perform_3a_impact(Joint_address_list.size(), false);
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            float open_position_change = std::abs(open_limit_positions[i] - open_start_positions[i]);
            can_perform_3a_impact[i] = (open_position_change > cfg_OPEN_POSITION_CHANGE_THRESHOLD);
            
            if (can_perform_3a_impact[i]) {
                std::cout << "[LEJU claw]:关节 " << Joint_address_list[i] << " 开爪运动正常，位置变化(" << open_position_change << " rad) > " << cfg_OPEN_POSITION_CHANGE_THRESHOLD << " rad，若关爪时卡死，将启用高电流冲关爪" << std::endl;
            } else {
                std::cout << "[LEJU claw]:关节 " << Joint_address_list[i] << " 开爪位置变化异常(" << open_position_change << " rad) < " << cfg_OPEN_POSITION_CHANGE_THRESHOLD << " rad，处于卡死状态，方向不明，将禁用高电流冲关爪，仅使用正常限位检测，请手动松开夹爪摆脱限位" << std::endl;
            }
        }
    }
    
    // 寻找关爪限位（正速度）
    std::cout << "[LEJU claw]:开始寻找关爪限位，超时时间设置为: " << cfg_ZERO_FIND_TIMEOUT_MS << " ms..." << std::endl;
    bool close_limit_found = find_claw_limit_velocity_control(false, cfg_ZERO_CONTROL_KP, cfg_ZERO_CONTROL_KD, cfg_ZERO_CONTROL_ALPHA, cfg_ZERO_CONTROL_MAX_CURRENT,
                                                           cfg_STALL_CURRENT_THRESHOLD, cfg_STALL_VELOCITY_THRESHOLD, cfg_ZERO_CONTROL_DT, cfg_ZERO_FIND_TIMEOUT_MS, can_perform_3a_impact);
    if (!close_limit_found)
    {
        std::cout << "[LEJU claw]:关爪限位寻找超时，认为当前位置为关爪限位，完成夹爪初始化" << std::endl;
    }

    // 记录关爪限位位置
    std::vector<float> close_limit_positions(Joint_address_list.size(), 0.0f);
    state = get_joint_state();
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            close_limit_positions[i] = state[i][1];
            std::cout << "[LEJU claw]:关节 " << Joint_address_list[i] << " 关爪限位位置: " << close_limit_positions[i] << std::endl;
        }
    }

    // 停止所有电机
    std::vector<int> index;
    std::vector<float> pos = create_zero_vector(Joint_address_list.size());
    std::vector<float> torque = create_zero_vector(Joint_address_list.size());
    std::vector<float> velocity = create_zero_vector(Joint_address_list.size());

    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            index.push_back(i);
        }
    }
    
    // 发送一次零电流停止电机
    send_positions(index, pos, torque, velocity);
    
    // 设置关节位置映射范围
    joint_start_positions.resize(Joint_online_list.size(), 0.0f);
    joint_end_positions.resize(Joint_online_list.size(), 0.0f);
    
    // 可调节的映射参数（百分比）（正数往行程外扩展，负数往行程内收缩）
    const float open_limit_adjustment = cfg_OPEN_LIMIT_ADJUSTMENT;
    const float close_limit_adjustment = cfg_CLOSE_LIMIT_ADJUSTMENT;
    
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            // 原始开爪和关爪位置
            float original_open_position = open_limit_positions[i];
            float original_close_position = close_limit_positions[i];
            float total_travel_range = std::abs(original_close_position - original_open_position);
            
            // 计算调整量（基于总行程范围的百分比）
            float open_adjustment = total_travel_range * (open_limit_adjustment / 100.0f);
            float close_adjustment = total_travel_range * (close_limit_adjustment / 100.0f);
            
            float actual_start_position, actual_end_position;
            
            if (original_close_position > original_open_position) {
                // 关爪位置大于开爪位置（正向运动）
                actual_start_position = original_open_position - open_adjustment;
                actual_end_position = original_close_position + close_adjustment;
            } else {
                // 关爪位置小于开爪位置（负向运动）
                actual_start_position = original_open_position + open_adjustment;
                actual_end_position = original_close_position - close_adjustment;
            }
            
            // 存储实际使用的限位位置
            joint_start_positions[i] = actual_start_position; 
            joint_end_positions[i] = actual_end_position;   
            
            float actual_travel_range = std::abs(actual_end_position - actual_start_position);
        }
    }
    
    std::cout << "[LEJU claw]:夹爪限位寻找完成，位置映射设置完成" << std::endl;
    
    // 等待系统稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool LeJuClaw::find_claw_limit_velocity_control(bool is_open_direction, float kp, float kd, float alpha,
                                               float max_current, float stall_current_threshold, 
                                               float stall_velocity_threshold, float dt, 
                                               int timeout_ms,
                                               const std::vector<bool>& can_perform_3a_impact)
{
    // 期望速度设置（开爪为负速度，关爪为正速度）
    const float target_velocity = is_open_direction ? -cfg_TARGET_VELOCITY : cfg_TARGET_VELOCITY;  // 期望速度（rad/s）
    
    std::vector<float> last_positions = create_zero_vector(Joint_address_list.size());
    std::vector<float> prev_torque = create_zero_vector(Joint_address_list.size());
    std::vector<bool> joint_limit_found(Joint_address_list.size(), false);
    
    // 关爪特殊处理相关变量
    std::vector<float> close_start_positions(Joint_address_list.size(), 0.0f);   // 关爪开始位置
    std::vector<bool> close_attempt_started(Joint_address_list.size(), false);   // 是否开始关爪尝试
    std::vector<int> close_attempt_count(Joint_address_list.size(), 0);          // 关爪尝试次数
    std::vector<std::chrono::steady_clock::time_point> close_attempt_start_times(Joint_address_list.size());
    std::vector<bool> close_high_current_mode(Joint_address_list.size(), false); // 是否处于高电流模式
    std::vector<bool> close_impact_active(Joint_address_list.size(), false);     // 是否处于冲击阶段
    std::vector<bool> close_stuck_detection_mode(Joint_address_list.size(), false); // 是否处于关爪卡死检测模式
    std::vector<bool> close_normal_limit_detection_enabled(Joint_address_list.size(), false); // 是否启用正常限位检测
    std::vector<std::chrono::steady_clock::time_point> close_startup_start_times(Joint_address_list.size()); // 关爪启动开始时间
    
    // 初始化位置记录
    std::vector<std::vector<float>> state = get_joint_state();
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            last_positions[i] = state[i][1];
            if (!is_open_direction)  // 关爪时记录开始位置
            {
                close_start_positions[i] = state[i][1];
                close_startup_start_times[i] = std::chrono::steady_clock::now(); // 记录启动开始时间
                close_stuck_detection_mode[i] = true; // 关爪时直接进入卡死检测模式
                close_normal_limit_detection_enabled[i] = false; // 初始时禁用正常限位检测
            }
        }
    }

    auto start_time = std::chrono::steady_clock::now();
    bool all_joints_found_limit = false;

    while (!all_joints_found_limit && thread_running)
    {
        // 超时检查
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed > timeout_ms)
        {
            std::cerr << "[LEJU claw]:限位寻找超时 (" << (is_open_direction ? "开爪" : "关爪") << ")，超时时间: " << timeout_ms << " ms，实际运行时间: " << elapsed << " ms" << std::endl;
            return false;
        }

        // 读取当前状态
        state = get_joint_state();
        std::vector<float> current_torques(Joint_address_list.size(), 0.0f);
        
        // 计算当前速度和电流控制输出
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i] && !joint_limit_found[i])
            {
                float current_pos = state[i][1];
                float current_velocity = (current_pos - last_positions[i]) / dt;
                float current_torque_feedback = state[i][3];  // 当前电流反馈
                
                // 关爪处理逻辑
                if (!is_open_direction && close_stuck_detection_mode[i] && !close_high_current_mode[i])
                {
                    float position_change = std::abs(current_pos - close_start_positions[i]);
                    float current_torque_feedback = state[i][3];  // 当前电流反馈
                    
                    // 检查启动延时是否已过
                    auto startup_elapsed = std::chrono::steady_clock::now() - close_startup_start_times[i];
                    long long startup_ms = std::chrono::duration_cast<std::chrono::milliseconds>(startup_elapsed).count();
                    
                    // 只有在启动延时过后才开始卡死检测
                    if (startup_ms >= cfg_CLOSE_STARTUP_DELAY_MS)
                    {
                        // 检查是否满足卡死条件：电流达到阈值且位置变化小于阈值
                        if (current_torque_feedback >= cfg_CLOSE_STUCK_CURRENT_THRESHOLD && position_change < cfg_CLOSE_STUCK_DETECTION_THRESHOLD)
                        {
                            // 检查是否允许执行高电流冲击模式
                            bool can_impact = (i < can_perform_3a_impact.size()) ? can_perform_3a_impact[i] : true;
                            
                            if (can_impact) {
                                // 满足卡死条件且允许冲击，开始冲击模式
                                close_attempt_started[i] = true;
                                close_attempt_count[i] = 0;
                                close_high_current_mode[i] = true;
                                close_attempt_start_times[i] = std::chrono::steady_clock::now();
                                close_impact_active[i] = true;  // 开始冲击阶段
                                std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 检测到卡死（电流:" << current_torque_feedback << "A, 位置变化:" << position_change << "），开始" << cfg_CLOSE_IMPACT_CURRENT << "A冲击模式" << std::endl;
                            } else {
                                // 满足卡死条件但不允许冲击，直接启用正常限位检测
                                close_stuck_detection_mode[i] = false;
                                close_normal_limit_detection_enabled[i] = true;
                                std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 检测到卡死但开爪位置变化异常，禁用高电流冲击模式，启用正常限位检测" << std::endl;
                            }
                        }
                        // 如果位置变化大于CLOSE_STUCK_DETECTION_THRESHOLD，说明正常运动，退出卡死检测模式和冲击模式，启用正常限位检测
                        else if (position_change >= cfg_CLOSE_STUCK_DETECTION_THRESHOLD)
                        {
                            close_stuck_detection_mode[i] = false;
                            close_high_current_mode[i] = false;  // 确保退出冲击模式
                            close_attempt_started[i] = false;
                            close_impact_active[i] = false;
                            close_normal_limit_detection_enabled[i] = true;
                            // 重置滤波器状态，确保平滑过渡到正常速度控制
                            prev_torque[i] = 0.0f;
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 位置变化正常，位置从 " << close_start_positions[i] << " 变化到 " << current_pos << "，总变化量:" << position_change << " rad，退出卡死检测模式，启用正常限位检测" << std::endl;
                        }
                    }
                    else
                    {
                        // // 启动延时内，输出调试信息
                        // if (startup_ms % 100 == 0) // 定期输出调试信息
                        // {
                        //     std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 启动卡死检测延时中，剩余 " << (CLOSE_STARTUP_DELAY_MS - startup_ms) << " ms，位置变化:" << position_change << " rad" << std::endl;
                        // }
                    }
                }
                
                // 高电流尝试模式
                if (close_high_current_mode[i])
                {
                    auto attempt_elapsed = std::chrono::steady_clock::now() - close_attempt_start_times[i];
                    long long attempt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(attempt_elapsed).count();
                    
                    // 冲击和间隔的循环逻辑
                    if (close_impact_active[i])
                    {
                        // 冲击阶段
                        if (attempt_ms < cfg_CLOSE_IMPACT_DURATION_MS) 
                        {
                            current_torques[i] = cfg_CLOSE_IMPACT_CURRENT;  // 冲击电流
                            // 只在冲击开始时输出一次日志
                            if (attempt_ms == 0)
                            {
                                three_amp_current_usage_count++;
                                std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 进入" << cfg_CLOSE_IMPACT_CURRENT << "A冲击模式 - 第" << three_amp_current_usage_count << "次" << std::endl;
                            }
                        }
                        else
                        {
                            // 冲击结束，进入间隔阶段
                            close_impact_active[i] = false;
                            current_torques[i] = 0.0f;
                            close_attempt_start_times[i] = std::chrono::steady_clock::now();  // 重置计时器
                        }
                    }
                    else
                    {
                        // 间隔阶段
                        if (attempt_ms < cfg_CLOSE_IMPACT_INTERVAL_MS)
                        {
                            current_torques[i] = 0.0f;  // 零电流间隔
                        }
                        else
                        {
                            // 间隔结束，检查位置变化
                            float position_change = std::abs(current_pos - close_start_positions[i]);
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 第" << close_attempt_count[i] << "次冲击间隔结束，位置变化:" << position_change << " rad" << std::endl;
                            if (position_change > cfg_CLOSE_STUCK_DETECTION_THRESHOLD)
                            {
                                // 位置变化超过阈值，退出高电流模式，启用正常限位检测
                                close_high_current_mode[i] = false;
                                close_attempt_started[i] = false;
                                close_stuck_detection_mode[i] = false;
                                close_normal_limit_detection_enabled[i] = true;
                                // 重置滤波器状态，确保平滑过渡到正常速度控制
                                prev_torque[i] = 0.0f;
                                std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 冲击成功，位置从 " << close_start_positions[i] << " 变化到 " << current_pos << "，总变化量:" << position_change << " rad，退出冲击模式，启用正常限位检测" << std::endl;
                            }
                            else
                            {
                                // 位置变化不大，开始下一次冲击
                                close_attempt_count[i]++;
                                if (close_attempt_count[i] >= cfg_CLOSE_MAX_ATTEMPTS)
                                {
                                    // 最大尝试次数完成，认为到达限位
                                    joint_limit_found[i] = true;
                                }
                                else
                                {
                                    // 开始下一次冲击
                                    close_impact_active[i] = true;
                                    close_attempt_start_times[i] = std::chrono::steady_clock::now();
                                }
                            }
                        }
                    }
                }
                else
                {
                    // 正常速度控制模式
                    float velocity_error = target_velocity - current_velocity;
                    float torque_out = kd * velocity_error;
                    
                    // 电流限幅：开爪时限制在-max_current，关爪时限制在+max_current
                    if (is_open_direction)
                    {
                        if (torque_out < -max_current)
                        {
                            torque_out = -max_current;
                        }
                    }
                    else
                    {
                        if (torque_out > max_current)
                        {
                            torque_out = max_current;
                        }
                    }
                    
                    // 滤波处理
                    current_torques[i] = lowPassFilter(torque_out, prev_torque[i], alpha);
                    prev_torque[i] = current_torques[i];
                    
                    // 开爪时的限位检测
                    if (is_open_direction && current_torque_feedback < -stall_current_threshold && std::abs(current_velocity) < stall_velocity_threshold)
                    {
                        joint_limit_found[i] = true;
                    }
                    
                    // 关爪时的限位检测 - 只有在启用正常限位检测且不在冲击模式时才执行
                    if (!is_open_direction && close_normal_limit_detection_enabled[i] && !close_high_current_mode[i] && current_torque_feedback > stall_current_threshold && std::abs(current_velocity) < stall_velocity_threshold)
                    {
                        joint_limit_found[i] = true;
                        std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 正常限位检测触发" << std::endl;
                    }
                }
                
                last_positions[i] = current_pos;
            }
        }
        
        // 发送电流指令
        std::vector<int> index;
        std::vector<float> pos(Joint_address_list.size(), 0.0f);
        std::vector<float> torque(Joint_address_list.size(), 0.0f);
        std::vector<float> velocity(Joint_address_list.size(), 0.0f);
        
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i])
            {
                index.push_back(i);
                // 如果关节已经找到限位，发送0电流停止运动
                if (joint_limit_found[i])
                {
                    torque[i] = 0.0f;
                }
                else
                {
                    torque[i] = current_torques[i];
                }
            }
        }
        
        send_positions(index, pos, torque, velocity);
        
        // 检查是否所有关节都找到限位
        all_joints_found_limit = true;
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i] && !joint_limit_found[i])
            {
                all_joints_found_limit = false;
                break;
            }
        }
        
        // 如果所有关节都找到限位，立即退出循环
        if (all_joints_found_limit)
        {
            std::cout << "[LEJU claw]:所有关节都到达" << (is_open_direction ? "开爪" : "关爪") << "限位，退出寻找过程" << std::endl;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }
    
    // 超时退出
    if (!all_joints_found_limit)
    {
        std::cout << "[LEJU claw]:" << (is_open_direction ? "开爪" : "关爪") << "限位寻找超时，认为当前位置为限位" << std::endl;
    }
    
    return all_joints_found_limit;
}

void LeJuClaw::interpolate_move(const std::vector<float> &start_positions, const std::vector<float> &target_positions, const std::vector<float> &speeds, float dt)
{
    // 插值计算
    std::vector<std::vector<float>> interpolation_list = interpolate_positions_with_speed(start_positions, target_positions, speeds, dt);

    std::vector<std::chrono::steady_clock::time_point> exceed_start_times(Joint_address_list.size()); 
    std::vector<bool> joint_exceeded_flags(Joint_address_list.size(), false); 
    std::vector<bool> joint_limit_reached(Joint_address_list.size(), false); 
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            int errorcode = 1;
            float state_list[6];
            std::lock_guard<std::mutex> lock(can_lock);
            errorcode = enter_motor_state(&ruiwo, Joint_address_list[i], state_list, NULL);
            if (errorcode == 0)
            {
                std::vector<float> state;
                state.assign(state_list, state_list + 6);
                set_joint_state(i, state);
                
            }
        }
    }

    for (const auto &target_position : interpolation_list)
    {
        
        std::vector<int> index(Joint_address_list.size());
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
                index[i] = i;
        }
        send_positions(index, target_position, target_torque, target_velocity);
        old_target_positions = target_position;

        
        std::vector<std::vector<float>> state = get_joint_state();

        bool all_joints_reached_limit = true; 

        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i]) // 
            {
                current_positions[i] = state[i][1];
                current_velocity[i] = state[i][2];
                current_torque[i] = state[i][3]; // 获取电流（或力矩）

                //std::cout << "Joint " << Joint_address_list[i] << " - Current torque: " << current_torque[i] << std::endl;

                
                if (abs(current_torque[i]) > 1.5) // 电流超过阈值
                {
                    if (!joint_exceeded_flags[i]) // 如果电流首次超标
                    {
                        exceed_start_times[i] = std::chrono::steady_clock::now(); 
                        joint_exceeded_flags[i] = true;
                        //std::cout << "Joint " << Joint_address_list[i] << " - Torque exceeded threshold, starting timer..." << std::endl;
                    }

                    
                    auto exceed_duration = std::chrono::steady_clock::now() - exceed_start_times[i];
                    long long exceed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(exceed_duration).count();

                    // 如果电流持续超过一定时间，标记为达到了限位
                    if(!joint_limit_reached[i]){
                        if (exceed_ms >= 200)
                        {
                            joint_limit_reached[i] = true; // 达到限位
                            std::cout << "Joint " << Joint_address_list[i] << " - Torque exceeded threshold for " << exceed_ms << " milliseconds, reaching limit." << std::endl;
                            float state_list[6];
                            int retcode;
                            std::lock_guard<std::mutex> lock(can_lock);
                            uint8_t errcode = 0;
                            retcode = enter_reset_state(&ruiwo, Joint_address_list[i], state_list, &errcode);
                            std::vector<float> state;
                            state.assign(state_list, state_list + 6);
                            set_joint_state(i, state);
                        }
                    }
                }
                else
                {
                    if (joint_exceeded_flags[i]) 
                    {
                        joint_exceeded_flags[i] = false; 
                        exceed_start_times[i] = std::chrono::steady_clock::time_point(); // 重置计时器
                        //std::cout << "Joint " << Joint_address_list[i] << " - Torque back to normal." << std::endl;
                    }
                    
                }
            }
        }

        all_joints_reached_limit = true;
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i] && !joint_limit_reached[i]) 
            {
                all_joints_reached_limit = false;
                break;
            }
        }

        // 如果所有关节都达到了限位，退出插值
        if (all_joints_reached_limit)
        {
            std::cout << "All joints reached the limit, exiting interpolation." << std::endl;

            
            std::cout << "Current positions of joints at exit:" << std::endl;
            for (size_t i = 0; i < Joint_address_list.size(); ++i)
            {
                if (Joint_online_list[i])
                {
                    std::cout << "Joint " << Joint_address_list[i] << " - Position: " << current_positions[i] << std::endl;
                }
            }

            return; // 退出插值过程
        }

        // 等待下一次插值
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    std::cout << "[LEJU claw]:Interpolation completed." << std::endl;
}


void LeJuClaw::enable()
{
    disable();
    target_positions.assign(Joint_address_list.size(), 0);
    target_velocity.assign(Joint_address_list.size(), 0);
    target_torque.assign(Joint_address_list.size(), 0);
    target_pos_kp.assign(Joint_address_list.size(), 0);
    target_pos_kd.assign(Joint_address_list.size(), 0);
    target_vel_kp.assign(Joint_address_list.size(), 0);
    target_vel_kd.assign(Joint_address_list.size(), 0);
    target_vel_ki.assign(Joint_address_list.size(), 0);

    for (size_t i = 0; i < Joint_parameter_list.size(); ++i)
    {
        target_pos_kp[i] = Joint_parameter_list[i][1];
        target_pos_kd[i] = Joint_parameter_list[i][2];
        target_vel_kp[i] = Joint_parameter_list[i][4];
        target_vel_kd[i] = Joint_parameter_list[i][5];
        target_vel_ki[i] = Joint_parameter_list[i][6];
    }

    old_target_positions.assign(Joint_address_list.size(), 0);
    current_positions.assign(Joint_address_list.size(), 0);
    current_torque.assign(Joint_address_list.size(), 0);
    current_velocity.assign(Joint_address_list.size(), 0);
    joint_status.assign(Joint_address_list.size(), std::vector<float>(6, 0));
    
    // 初始化全局保持电流机制
    is_holding_object_global.assign(Joint_address_list.size(), false);
    holding_current_values.assign(Joint_address_list.size(), 0.0f);
    
    // 初始化move_paw状态记录
    last_move_paw_target_positions.assign(Joint_address_list.size(), -1.0);  // -1表示未设置
    last_exit_with_grip_current.assign(Joint_address_list.size(), false);
    
    float state_list[6];
    int failed_count = 0;  // 失败计数
    int total_claw_count = 0;  // 总夹爪数量
    
    for (int i = 0; i < Joint_address_list.size(); ++i)
    {
        int errorcode = 1;
        if (Joint_address_list[i] == DISABLE_ADDRESS)
        {
            Joint_online_list[i] = false;
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Enable: [Failed], Errorcode: [Disable address]" << std::endl;
            failed_count++;
            continue;
        }
        
        total_claw_count++;
        errorcode = enter_reset_state(&ruiwo, Joint_address_list[i], state_list, NULL);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        errorcode = enter_motor_state(&ruiwo, Joint_address_list[i], state_list, NULL);
        if (errorcode == 0)
        {
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            set_joint_state(i, state);
            Joint_online_list[i] = true;
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Enable: [Succeed]" << std::endl;
        }
        else
        {
            Joint_online_list[i] = false;
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Enable: [Failed], Errorcode: [" << errorcode << "]" << std::endl;
            failed_count++;
        }
    }
    
    // 检查是否所有夹爪都连接失败
    if (failed_count == total_claw_count && total_claw_count > 0)
    {
        std::cout << "\n" << std::string(60, '=') << std::endl;
        std::cout << "❌ 夹爪连接失败！" << std::endl;
        std::cout << "请检查以下项目：" << std::endl;
        std::cout << "1. 夹爪电源是否正确连接" << std::endl;
        std::cout << "2. 夹爪通信线缆是否正确连接" << std::endl;
        std::cout << "3. 夹爪ID配置是否正确" << std::endl;
        std::cout << "4. 夹爪是否处于正常工作状态" << std::endl;
        std::cout << std::string(60, '=') << std::endl;
        std::cout << "\n程序将退出..." << std::endl;
        throw std::runtime_error("所有夹爪连接失败，请检查硬件连接");
    }
}

void LeJuClaw::disable()
{
    std::vector<bool> Joint_online_list_reload;
    std::vector<int> Joint_address_list_reload;
    std::vector<std::vector<int>> Joint_parameter_list_reload;

    int errorcode = 1;
    float state_list[6];
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_address_list[i] == DISABLE_ADDRESS)
        {
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Disable: [Failed], Errorcode: [Disable address]" << std::endl;
            continue;
        }
        errorcode = enter_reset_state(&ruiwo, Joint_address_list[i], state_list, NULL);
        if (errorcode == 0)
        {
            Joint_online_list[i] = false;
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Disable: [Succeed]" << std::endl;
            for (int j = 0; j < 5; ++j)
            {
                run_ptm_mode(&ruiwo, Joint_address_list[i], 0, 0, 0, 0, 0, state_list);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        else
        {
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Disable: [Failed], Errorcode: [" << errorcode << "]" << std::endl;
        }
    }

}

void LeJuClaw::set_zero()
{
    float state_list[6];
    int errorcode = 1;
    for (int i = 0; i < Joint_address_list.size(); ++i)
    {
        errorcode = set_zero_position(&ruiwo, Joint_address_list[i], state_list);
        if (errorcode == 0)
        {
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            set_joint_state(i, state);
        }
        std::cout << "[LEJU claw]:Set all motor zero position " << state_list[1] << std::endl;
    }
}

void LeJuClaw::send_positions(const std::vector<int> &index, const std::vector<float> &pos, const std::vector<float> &torque, const std::vector<float> &velocity)
{
    if (!thread_running)
    {
        return;
    }

    auto target_torque = torque;
    for (int i : index)
    {
        if (!Joint_online_list[i])
        {
            //std::cout << "Joint " << Joint_address_list[i] << " is offline, skipping." << std::endl;
            continue;
        }
        // if (Joint_address_list[i] == Head_joint_address[1])
        // {
        //     target_torque[i] = head_high_torque;
        // }
        float state_list[6];
        int errorcode = 1;
        if (Control_mode == "ptm")
        {
            // // 打印所有参数的值
            // std::cout << "[LEJU claw]:Calling run_ptm_mode with parameters:" << std::endl;
            // std::cout << "Joint Address: " << Joint_address_list[i] << std::endl;
            // std::cout << "Position: " << pos[i] << std::endl;
            // std::cout << "Velocity: " << velocity[i] << std::endl;
            // std::cout << "Target Position Kp: " << target_pos_kp[i] << std::endl;
            // std::cout << "Target Position Kd: " << target_pos_kd[i] << std::endl;
            // std::cout << "Target Torque: " << target_torque[i] << std::endl;
            errorcode = run_ptm_mode(&ruiwo, Joint_address_list[i], pos[i], velocity[i], target_pos_kp[i], target_pos_kd[i], target_torque[i], state_list);
        }
        else if (Control_mode == "servo")
        {
            errorcode = run_servo_mode(&ruiwo, Joint_address_list[i], pos[i], velocity[i], target_pos_kp[i], target_pos_kd[i], target_vel_kp[i], target_vel_kd[i], target_vel_ki[i], state_list);
        }
        if (errorcode == 0)
        {
            std::vector<float> state;
            state.assign(state_list, state_list + 6);
            //  //打印state的内容
            // std::cout << "[LEJU claw]:Setting joint state for Joint--------------------- " << Joint_address_list[i] << " - State: ";
            // for (const auto &val : state)
            // {
            //     std::cout << val << " ";  // 打印每个state的值
            // }
            // std::cout << std::endl;
            set_joint_state(i, state);
            
        }
        else
        {
            std::cout << "[LEJU claw]:ID:" << Joint_address_list[i] << " Send position: [Failed], Errorcode: [" << errorcode << "]" << std::endl;
        }
    }
}

void LeJuClaw::set_positions(const std::vector<uint8_t> &index, const std::vector<double> &positions, const std::vector<double> &torque, const std::vector<double> &velocity)
{
    if (!thread_running)
    {
        return;
    }
    std::vector<double> rad_position;
    std::vector<double> rad_velocity;
    rad_position = positions;
    rad_velocity = velocity;
    for (size_t i = 0; i < positions.size(); i++)
    {
        rad_position[i] = (positions[i] * 3.14159265358979323846) / 180;
        rad_velocity[i] = (velocity[i] * 3.14159265358979323846) / 180;
    }
    {
        std::lock_guard<std::mutex> lock(sendpos_lock);
        auto new_positions = target_positions;
        auto new_torque = target_torque;
        auto new_velocity = target_velocity;

        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_address_list[i] == DISABLE_ADDRESS)
            {
                continue;
            }
            
            for (size_t j = 0; j < index.size(); ++j)
            {
                if (Joint_address_list[i] - 1 == static_cast<int>(index[j]))
                {
                    if (std::find(Negtive_joint_address_list.begin(), Negtive_joint_address_list.end(), Joint_address_list[i]) != Negtive_joint_address_list.end())
                    {
                        new_positions[i] = -rad_position[j];
                        new_torque[i] = std::max(-10.0, -torque[j]);
                        new_velocity[i] = velocity_factor * -rad_velocity[j];
                    }
                    else
                    {
                        new_positions[i] = rad_position[j];
                        new_torque[i] = std::min(10.0, torque[j]);
                        new_velocity[i] = velocity_factor * rad_velocity[j];
                    }
                    break;
                }
            }
        }

        target_positions = new_positions;
        target_torque = new_torque;
        target_velocity = new_velocity;
        target_update = true;
    }
}

LeJuClaw::PawMoveState LeJuClaw::move_paw(const std::vector<double>& positions, const std::vector<double>& velocity, const std::vector<double>& torque)
{
    // 调用带VR模式参数的版本，默认非VR模式
    return move_paw(positions, velocity, torque, false);
}

LeJuClaw::PawMoveState LeJuClaw::move_paw(const std::vector<double>& positions, const std::vector<double>& velocity, const std::vector<double>& torque, bool is_vr_mode)
{
    if (!thread_running)
    {
        return PawMoveState::ERROR;
    }

    // 输入参数安全检查
    if (positions.size() != Joint_address_list.size() ||
        velocity.size() != Joint_address_list.size() ||
        torque.size() != Joint_address_list.size())
    {
        std::cerr << "[LEJU claw]:Input vector sizes do not match Joint_address_list size." << std::endl;
        return PawMoveState::ERROR;
    }

    std::vector<double> safe_positions = positions;
    for (size_t i = 0; i < safe_positions.size(); ++i)
    {
        safe_positions[i] = std::max(0.0, std::min(100.0, safe_positions[i]));
    }

    // 保存当前目标位置（百分比）
    std::vector<double> current_target_positions = safe_positions;

    // 检查当前位置是否已经在目标位置的误差允许范围内
    std::vector<std::vector<float>> current_state = get_joint_state();
    bool all_in_tolerance = true;
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            // 获取当前位置
            float current_pos = current_state[i][1];
            
            // 将目标位置（百分比）转换为实际位置（弧度）
            float start = joint_start_positions[i];
            float end = joint_end_positions[i];
            float target_pos = start + (safe_positions[i] * (end - start)) / 100.0f;
            
            // 计算误差阈值（基于行程百分比）
            float total_travel_range = std::abs(end - start);
            float error_threshold = total_travel_range * (cfg_MIN_ERROR_PERCENT / 100.0f);
            
            // 检查是否在误差范围内
            bool is_in_tolerance = false;
            if (target_pos > current_pos) {
                is_in_tolerance = (current_pos >= target_pos - error_threshold);
            } else {
                is_in_tolerance = (current_pos <= target_pos + error_threshold);
            }
            
            if (!is_in_tolerance)
            {
                all_in_tolerance = false;
                break;
            }
        }
    }
    
    // 如果所有夹爪都在目标位置的误差允许范围内，跳过move_paw，保持之前的状态
    if (all_in_tolerance)
    {
        // 记录本次目标位置
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i])
            {
                last_move_paw_target_positions[i] = current_target_positions[i];
            }
        }
        
        // std::cout << "[LEJU claw] 所有夹爪已在目标位置误差允许范围内，跳过move_paw，保持之前的状态" << std::endl;
        return PawMoveState::LEFT_REACHED_RIGHT_REACHED;
    }

    // 检查上次退出时是否处于维持夹持状态，决定是否跳过控制
    bool all_skip_control = true;
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            // [修改] 移除 VR 模式的特殊处理，VR 和非 VR 模式使用相同逻辑
            // VR 模式下清除之前的夹持状态，正常执行 move_paw
            if (is_vr_mode)
            {
                all_skip_control = false;
                // 清除之前可能存在的夹持状态
                is_holding_object_global[i] = false;
                holding_current_values[i] = 0.0f;
                last_exit_with_grip_current[i] = false;
                continue;  // VR 模式直接进入正常控制逻辑
            }

            // 非VR模式的正常逻辑
            // 非VR模式：如果启用了"100位置则维持电流"功能，且目标位置是100，且上次退出时处于维持夹持状态，跳过控制
            if (!is_vr_mode && cfg_ENABLE_100_POSITION_HOLDING_CURRENT && safe_positions[i] >= 99.9 && last_exit_with_grip_current[i] && 
                last_move_paw_target_positions[i] >= 0.0)
            {
                // 目标位置是100，且上次处于维持夹持状态，跳过控制，继续维持夹持电流
                // std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] 
                //           << " 目标位置100%，且上次退出时处于维持夹持状态，跳过控制，继续维持夹持电流" << std::endl;
            }
            // 检查上次是否处于维持夹持电流状态
            else if (last_exit_with_grip_current[i] && 
                last_move_paw_target_positions[i] >= 0.0)  // 上次有记录目标位置
            {
                // 检查目标位置变化方向
                double target_diff = safe_positions[i] - last_move_paw_target_positions[i];
                
                // 如果目标位置相同（允许0.1%误差）或更大（关爪方向），继续维持夹持电流
                if (std::abs(target_diff) < 0.1 || target_diff > 0.0)
                {
                    // 跳过控制，继续维持夹持电流
                    // std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] 
                    //           << " 上次退出时处于维持夹持状态，且目标位置相同或更大（上次: " << last_move_paw_target_positions[i] 
                    //           << "%，本次: " << safe_positions[i] << "%），跳过控制，继续维持夹持电流" << std::endl;
                }
                else
                {
                    // 目标位置变小了（开爪方向），清除维持夹持状态，正常执行move_paw
                    all_skip_control = false;
                    is_holding_object_global[i] = false;
                    holding_current_values[i] = 0.0f;  // 清除维持电流值
                    last_exit_with_grip_current[i] = false;  // 清除维持夹持状态标记
                    // std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] 
                    //           << " 目标位置变小（上次: " << last_move_paw_target_positions[i] 
                    //           << "%，本次: " << safe_positions[i] << "%），开爪方向，清除维持夹持状态，正常执行move_paw" << std::endl;
                }
            }
            else
            {
                all_skip_control = false;
                // 如果上次不是维持夹持电流状态，清除标记
                if (!last_exit_with_grip_current[i])
                {
                    is_holding_object_global[i] = false;
                    holding_current_values[i] = 0.0f;  // 清除维持电流值
                }
            }
        }
    }
    
    // 如果所有夹爪都跳过控制，直接退出
    if (all_skip_control)
    {
        // 记录本次目标位置
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i])
            {
                last_move_paw_target_positions[i] = current_target_positions[i];
            }
        }
        
        // 触发一次发送，确保保持电流被发送
        {
            std::lock_guard<std::mutex> lock(sendtor_lock);
            target_update = true;
        }
        
        return PawMoveState::LEFT_REACHED_RIGHT_REACHED;
    }

    // 初始化电机状态
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            float state_list[6];
            std::lock_guard<std::mutex> lock(can_lock);
            int errorcode = enter_motor_state(&ruiwo, Joint_address_list[i], state_list, NULL);
            if (errorcode != 0)
            {
                std::cerr << "Joint " << Joint_address_list[i] << " 初始化失败" << std::endl;
                return PawMoveState::ERROR;
            }
            set_joint_state(i, std::vector<float>(state_list, state_list + 6));
        }
    }

    // 使用传入的VR模式状态
    bool is_vr_control_mode = is_vr_mode;
    // std::cout << "[LEJU claw] 使用传入的VR模式状态: " << (is_vr_control_mode ? "VR模式" : "正常模式") << std::endl;
    
    // 映射目标位置
    std::vector<float> mapped_positions;
    for (size_t i = 0; i < safe_positions.size(); ++i)
    {
        float start = joint_start_positions[i];
        float end = joint_end_positions[i];
        mapped_positions.push_back(start + (safe_positions[i] * (end - start)) / 100.0f);
    }

    // 初始化PD控制相关变量
    std::vector<float> prev_torque = create_zero_vector(Joint_address_list.size());
    std::vector<float> last_positions = create_zero_vector(Joint_address_list.size());  // 上一时刻位置
    std::vector<float> last_velocities = create_zero_vector(Joint_address_list.size()); // 上一时刻速度
    
    // 卡死检测相关变量
    std::vector<std::chrono::steady_clock::time_point> stuck_start_times(Joint_address_list.size());
    std::vector<std::chrono::steady_clock::time_point> stuck_detection_start_times(Joint_address_list.size());
    std::vector<bool> stuck_detected(Joint_address_list.size(), false);
    std::vector<bool> in_impact_mode(Joint_address_list.size(), false);
    std::vector<std::chrono::steady_clock::time_point> impact_start_times(Joint_address_list.size());
    std::vector<bool> impact_active(Joint_address_list.size(), false);
    std::vector<float> stuck_check_positions = create_zero_vector(Joint_address_list.size());
    std::vector<bool> stuck_detection_enabled(Joint_address_list.size(), false);
    
    // 目标位置检测相关变量
    std::vector<std::chrono::steady_clock::time_point> target_position_detection_start_times(Joint_address_list.size());
    std::vector<bool> target_position_detection_enabled(Joint_address_list.size(), false);
    std::vector<float> target_position_check_positions = create_zero_vector(Joint_address_list.size());
    
    // 稳定检测相关变量
    std::vector<std::chrono::steady_clock::time_point> stable_detection_start_times(Joint_address_list.size());
    std::vector<bool> stable_detection_enabled(Joint_address_list.size(), false);
    std::vector<bool> is_stable(Joint_address_list.size(), false);
    std::vector<float> stable_check_positions = create_zero_vector(Joint_address_list.size());
    std::vector<bool> is_holding_object(Joint_address_list.size(), false);  // 记录是否夹到东西
    std::vector<float> stable_detection_max_current(Joint_address_list.size(), 0.0f);  // 稳定检测期间的最大电流值
    
    // VR模式卡死检测相关变量
    std::vector<int> vr_stuck_cycle_count(Joint_address_list.size(), 0);  // VR模式下卡死检测周期计数
    std::vector<float> vr_stuck_check_positions = create_zero_vector(Joint_address_list.size());  // VR模式下卡死检测起始位置
    
    // 获取初始状态
    std::vector<std::vector<float>> state = get_joint_state();
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            last_positions[i] = state[i][1];
            stuck_check_positions[i] = state[i][1];
            target_position_check_positions[i] = state[i][1];  // 初始化目标位置检测起始位置
            stable_check_positions[i] = state[i][1];
            vr_stuck_check_positions[i] = state[i][1];  // 初始化VR卡死检测起始位置
            stuck_detection_start_times[i] = std::chrono::steady_clock::now();
            target_position_detection_start_times[i] = std::chrono::steady_clock::now();
            stable_detection_start_times[i] = std::chrono::steady_clock::now();
        }
    }

    // 主循环
    auto start_time = std::chrono::steady_clock::now();
    bool all_reached = false;
    
    // VR控制模式：进入快速响应模式，使用超时时间
    if (is_vr_control_mode) {
        // std::cout << "[LEJU claw] VR模式：进入快速响应模式，超时时间=" << cfg_VR_CONTROL_TIMEOUT_MS << "ms" << std::endl;
        {
            std::lock_guard<std::mutex> lock(sendpos_lock);
            for (size_t i = 0; i < Joint_address_list.size(); ++i) {
                if (Joint_online_list[i]) {
                    target_positions[i] = mapped_positions[i];
                }
            }
            target_update = true;
        }
        
        // VR模式启用超时机制
        movement_timeout_enabled = true;
        movement_start_time = std::chrono::steady_clock::now();
    } else {
        // std::cout << "[LEJU claw] 非VR模式：进入正常控制循环，等待稳定检测或超时" << std::endl;
        // 非VR模式也启用超时机制
        movement_timeout_enabled = true;
        movement_start_time = std::chrono::steady_clock::now();
    }

    while (!all_reached && thread_running)
    {
        // 超时检查：VR模式和非VR模式都检查超时
        if (movement_timeout_enabled) {
            auto current_loop_time = std::chrono::steady_clock::now();
            auto movement_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_loop_time - movement_start_time).count();
            
            int timeout_ms = is_vr_control_mode ? cfg_VR_CONTROL_TIMEOUT_MS : cfg_MOVE_PAW_TIMEOUT_MS;
            if (movement_elapsed > timeout_ms) {
                if (is_vr_control_mode) {
                    // std::cout << "[LEJU claw] VR控制模式超时 (" << timeout_ms << " ms)，强制退出" << std::endl;
                } else {
                    std::cout << "[LEJU claw] 非VR模式超时 (" << timeout_ms << " ms)，清零电流退出move_paw" << std::endl;
                }
                all_reached = true;  // 超时时认为到达目标位置
                break;
            }
        }
        
        // 读取当前状态
        state = get_joint_state();
        std::vector<float> current_velocities(Joint_address_list.size(), 0.0f);
        
        // 计算当前速度和加速度
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_online_list[i])
            {
                float current_pos = state[i][1];
                // 计算当前速度：位置差分
                current_velocities[i] = (current_pos - last_positions[i]) / cfg_DT;
                last_positions[i] = current_pos;
            }
        }

        // PD控制计算
        std::vector<float> new_torque(Joint_address_list.size(), 0.0);
        all_reached = true;

        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (!Joint_online_list[i]) continue;

            // [回退1.2.2] 移除VR模式夹持电流优先级逻辑，统一使用PD控制

            float current_pos = state[i][1];
            float target_pos = mapped_positions[i];
            float error = target_pos - current_pos;           // 位置误差
            float current_velocity = current_velocities[i];   // 当前速度
            float target_velocity = 0.0f;                     // 目标速度（期望速度为0）
            float current_torque = state[i][3];               // 当前电流

            // 所有电机都适用夹爪逻辑
            bool is_claw_joint = true;

            // 计算行程范围（用于百分比计算）
            float total_travel_range = std::abs(joint_end_positions[i] - joint_start_positions[i]);
            
            // 1. 检查是否到达目标位置（位置在误差范围内，直接退出，清零电流）
            // 基于行程百分比计算误差阈值
            float error_threshold = total_travel_range * (cfg_MIN_ERROR_PERCENT / 100.0f);
            bool is_in_tolerance = false;
            if (target_pos > current_pos) {
                is_in_tolerance = (current_pos >= target_pos - error_threshold);
            } else {
                is_in_tolerance = (current_pos <= target_pos + error_threshold);
            }
            
            // 如果位置在误差范围内，直接退出（到达目标位置），清零电流
            if (is_in_tolerance)
            {
                new_torque[i] = 0.0f;
                // 重置所有检测状态
                stuck_detected[i] = false;
                in_impact_mode[i] = false;
                impact_active[i] = false;
                stuck_detection_enabled[i] = false;
                vr_stuck_cycle_count[i] = 0;  // 重置VR卡死检测计数
                vr_stuck_check_positions[i] = current_pos;  // 重置VR卡死检测位置
                is_stable[i] = true;  // 标记为稳定，允许退出
                is_holding_object[i] = false;  // 到达目标位置，未夹到东西
                // 清除全局保持电流状态（到达目标位置，未夹到东西）
                is_holding_object_global[i] = false;
                holding_current_values[i] = 0.0f;
                continue;
            }
            
            // 计算当前位置距离开/关限位的距离（使用百分比）
            float limit_range_rad = total_travel_range * (cfg_LIMIT_RANGE_PERCENT / 100.0f);
            float dist_to_open = std::abs(current_pos - joint_start_positions[i]);
            float dist_to_close = std::abs(current_pos - joint_end_positions[i]);
            
            // 根据运动方向确定冲击范围（卡死检测范围）
            bool is_closing_direction = (error > 0);  // 正误差表示向关爪方向运动
            bool in_impact_range = false;  // 卡死检测范围
            bool in_closing_limit_range = (dist_to_close < limit_range_rad);  // 在关爪限位范围内
            bool in_opening_limit_range = (dist_to_open < limit_range_rad);   // 在开爪限位范围内
            
            if (is_closing_direction) {
                // 关闭方向：在开限位附近范围内才进行卡死检测
                in_impact_range = in_opening_limit_range;
            } else {
                // 打开方向：在关限位附近范围内才进行卡死检测
                in_impact_range = in_closing_limit_range;
            }

            // 目标位置检测和稳定检测是独立的、并行的两条退出路径
            // 目标位置检测：在误差范围内进行
            // 稳定检测：在误差范围外进行
            // 两者互补且互斥，覆盖不同场景
            
            // 检测条件：不在卡死检测范围内，或者在同向限位范围内
            bool can_perform_detection = !in_impact_range || 
                (is_closing_direction && in_closing_limit_range) ||  // 关爪方向时，在关爪限位范围内可以进行检测（不进行卡死检测）
                (!is_closing_direction && in_opening_limit_range);    // 开爪方向时，在开爪限位范围内可以进行检测（不进行卡死检测）
            
            // [回退1.2.2] VR模式禁用稳定检测，使用超时快速响应
            if (!is_vr_control_mode && !is_stable[i] && can_perform_detection) {
                float current_magnitude = std::abs(current_torque);
                
                // 路径1：目标位置检测（在误差范围内，100ms内位置变化 < TARGET_POSITION_THRESHOLD）
                if (is_in_tolerance) {
                    float target_position_change = std::abs(current_pos - target_position_check_positions[i]);
                    
                    if (!target_position_detection_enabled[i]) {
                        // 开始目标位置检测
                        target_position_detection_enabled[i] = true;
                        target_position_detection_start_times[i] = std::chrono::steady_clock::now();
                        target_position_check_positions[i] = current_pos;
                    } else {
                        // 检查位置变化是否超过阈值
                        if (target_position_change >= cfg_TARGET_POSITION_THRESHOLD) {
                            // 位置变化超过阈值，重置目标位置检测
                            target_position_check_positions[i] = current_pos;
                            target_position_detection_start_times[i] = std::chrono::steady_clock::now();
                        } else {
                            // 检查是否达到检测时间
                            auto target_detection_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - target_position_detection_start_times[i]).count();
                            if (static_cast<float>(target_detection_elapsed) >= cfg_TARGET_POSITION_DETECTION_TIME_MS) {
                                // 通过目标位置检测，可以退出move_paw
                                is_stable[i] = true;
                                // 退出时检查电流是否 > 1A来判断是否夹取
                                if (current_magnitude >= cfg_STABLE_CURRENT_THRESHOLD) {
                                    is_holding_object[i] = true;
                                }
                                // std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 通过目标位置检测（" << cfg_TARGET_POSITION_DETECTION_TIME_MS << "ms内位置变化<" << cfg_TARGET_POSITION_THRESHOLD << "rad），位置: " << current_pos << ", 目标: " << target_pos << ", 当前电流: " << current_torque << "A" << std::endl;
                            }
                        }
                    }
                } else {
                    // 不在误差范围内，重置目标位置检测
                    target_position_detection_enabled[i] = false;
                    target_position_check_positions[i] = current_pos;
                }
                
                // 路径2：稳定检测（在误差范围外，200ms内位置 < STABLE_POSITION_THRESHOLD 且速度 < STABLE_VELOCITY_THRESHOLD）
                if (!is_in_tolerance) {
                float position_change = std::abs(current_pos - stable_check_positions[i]);
                float velocity_magnitude = std::abs(current_velocity);
                
                if (!stable_detection_enabled[i]) {
                    // 开始稳定检测
                    stable_detection_enabled[i] = true;
                    stable_detection_start_times[i] = std::chrono::steady_clock::now();
                    stable_check_positions[i] = current_pos;
                    stable_detection_max_current[i] = current_magnitude;  // 初始化最大电流值
                } else {
                    // 如果位置和速度都很小，继续稳定检测
                    if (position_change < cfg_STABLE_POSITION_THRESHOLD && velocity_magnitude < cfg_STABLE_VELOCITY_THRESHOLD) {
                        // 在稳定检测期间，持续记录最大电流值
                        if (current_magnitude > stable_detection_max_current[i]) {
                            stable_detection_max_current[i] = current_magnitude;
                        }
                        
                        auto stable_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - stable_detection_start_times[i]).count();
                        if (static_cast<float>(stable_elapsed) >= cfg_STABLE_DETECTION_TIME_MS) {
                            // 通过稳定检测，可以退出move_paw
                            is_stable[i] = true;
                            // 退出时检查电流是否 > 1A来判断是否夹取
                            if (stable_detection_max_current[i] >= cfg_STABLE_CURRENT_THRESHOLD) {
                                is_holding_object[i] = true;
                            }
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 通过稳定检测（" << cfg_STABLE_DETECTION_TIME_MS << "ms内位置变化<" << cfg_STABLE_POSITION_THRESHOLD << "rad且速度<" << cfg_STABLE_VELOCITY_THRESHOLD << "rad/s），位置: " << current_pos << ", 目标: " << target_pos << ", 稳定检测期间最大电流: " << stable_detection_max_current[i] << "A, 当前电流: " << current_torque << "A" << std::endl;
                        }
                    } else {
                        // 位置或速度有变化，重置稳定检测
                        stable_detection_enabled[i] = false;
                        stable_check_positions[i] = current_pos;
                        stable_detection_max_current[i] = 0.0f;  // 重置最大电流值
                    }
                }
                } else {
                    // 在误差范围内，重置稳定检测
                    stable_detection_enabled[i] = false;
                    stable_check_positions[i] = current_pos;
                    stable_detection_max_current[i] = 0.0f;  // 重置最大电流值
                }
            // [回退1.2.2] VR模式下重置稳定检测状态
            } else if (is_vr_control_mode || !can_perform_detection) {
                // 不能进行检测时（在卡死检测范围内且不在同向限位范围内），重置所有检测状态
                target_position_detection_enabled[i] = false;
                target_position_check_positions[i] = current_pos;
                stable_detection_enabled[i] = false;
                stable_check_positions[i] = current_pos;
                stable_detection_max_current[i] = 0.0f;  // 重置最大电流值
            }
            
            // 如果已经稳定但位置或速度有变化，重置所有检测状态
            // [回退1.2.2] 只在非VR模式下检查稳定状态重置
            if (!is_vr_control_mode && is_stable[i] && can_perform_detection) {
                float position_change = std::abs(current_pos - stable_check_positions[i]);
                float velocity_magnitude = std::abs(current_velocity);
                // 如果位置或速度变化较大，重置稳定状态
                if (position_change >= cfg_STABLE_POSITION_THRESHOLD || velocity_magnitude >= cfg_STABLE_VELOCITY_THRESHOLD) {
                    is_stable[i] = false;
                    is_holding_object[i] = false;
                    target_position_detection_enabled[i] = false;
                    target_position_check_positions[i] = current_pos;
                    stable_detection_enabled[i] = false;
                    stable_check_positions[i] = current_pos;
                    stable_detection_max_current[i] = 0.0f;
                }
            }
            
            // 如果夹爪已经稳定，检查是否需要保持电流
            // VR 模式下不清零电流，继续 PD 控制（VR 是持续控制，不应因稳定而停止）
            // 非 VR 模式下，稳定后清零电流或保持夹持电流
            if (is_stable[i] && !is_vr_control_mode) {
                // 检查是否是因为夹到东西而进入稳定状态
                if (is_holding_object[i]) {
                    // 3. 夹取到物体：保持电流，退出move_paw后一直保持电流
                    // 检查运动方向：只有关爪方向（正误差）才发布保持电流
                    if (error > 0) {  // 正误差表示关爪方向
                        // 夹到东西且在关爪方向，发布保持电流
                        new_torque[i] = cfg_GRIP_HOLDING_CURRENT;
                        // 设置全局保持电流状态，供control_thread持续发送
                        is_holding_object_global[i] = true;
                        holding_current_values[i] = cfg_GRIP_HOLDING_CURRENT;
                        static bool holding_current_printed = false;
                        if (!holding_current_printed) {
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 正在输出保持电流 " << cfg_GRIP_HOLDING_CURRENT << "A 维持夹持力（关爪方向）" << std::endl;
                            holding_current_printed = true;
                        }
                    } else {
                        // 夹到东西但在开爪方向，清零电流
                        new_torque[i] = 0.0f;
                        // 清除全局保持电流状态
                        is_holding_object_global[i] = false;
                        holding_current_values[i] = 0.0f;
                        static bool opening_direction_printed = false;
                        if (!opening_direction_printed) {
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 开爪方向，清零电流" << std::endl;
                            opening_direction_printed = true;
                        }
                    }
                    // 夹到东西时，输出保持电流后跳过后续的PD控制
                    continue;
                } else {
                    // 2. 稳定状态（未夹到东西）：清零电流，允许退出move_paw
                    new_torque[i] = 0.0f;
                    // 清除全局保持电流状态
                    is_holding_object_global[i] = false;
                    holding_current_values[i] = 0.0f;
                    continue;
                }
            }
            // VR 模式下即使稳定也继续 PD 控制，不跳过

            // 正常PD控制逻辑
            if (!in_impact_mode[i]) {
                // 计算当前位置百分比
                float total_range = std::abs(joint_end_positions[i] - joint_start_positions[i]);
                float current_percent = 0.0f;
                if (total_range > 0.0f) {
                    current_percent = ((current_pos - joint_start_positions[i]) / total_range) * 100.0f;
                    current_percent = std::max(0.0f, std::min(100.0f, current_percent));
                }
                
                // 根据运动方向确定限位刹车范围
                // 使用1.2.2的渐变刹车机制：计算刹车速度系数
                // brake_factor 同时应用于 KP 和 KD，保持阻尼比不变，确保左右夹爪速度一致
                float brake_factor = calculate_brake_speed_factor(
                    current_pos, target_pos, joint_start_positions[i], joint_end_positions[i]);

                // 应用刹车速度系数到PD控制参数（KP和KD同时乘以相同系数）
                float adjusted_kp = cfg_KP * brake_factor;
                float adjusted_kd = cfg_KD * brake_factor;

                float velocity_error = target_velocity - current_velocity;  // 速度误差
                float pd_output = adjusted_kp * error + adjusted_kd * velocity_error;
                float torque_value = std::max(-cfg_MAX_CURRENT, std::min(cfg_MAX_CURRENT, pd_output));
                new_torque[i] = lowPassFilter(torque_value, prev_torque[i], cfg_ALPHA);
                prev_torque[i] = new_torque[i];
            }

            // VR模式下的卡死检测逻辑，基于多次周期检测
            if (is_vr_control_mode && !stuck_detected[i] && !in_impact_mode[i]) {
                // 首先检查目标位置与当前位置的差值
                float target_position_error = std::abs(target_positions[i] - current_pos);
                
                // 只有当目标位置与当前位置差值大于阈值时，才进行卡死检测
                if (target_position_error > cfg_VR_TARGET_POSITION_THRESHOLD) {
                    float position_change = std::abs(current_pos - vr_stuck_check_positions[i]);
                    
                    // 如果位置变化小于阈值，增加卡死计数
                    if (position_change < cfg_VR_STUCK_POSITION_THRESHOLD) {
                        vr_stuck_cycle_count[i]++;
                    } else {
                        // 位置有变化，重置计数和检测位置
                        vr_stuck_cycle_count[i] = 0;
                        vr_stuck_check_positions[i] = current_pos;
                    }
                    
                    // 如果连续多个周期位置变化都很小，判定为卡死
                    if (vr_stuck_cycle_count[i] >= cfg_VR_STUCK_DETECTION_CYCLES) {
                        // VR模式下，只有在LIMIT_RANGE_PERCENT范围内才允许反冲
                        if (in_impact_range) {
                            stuck_detected[i] = true;
                            in_impact_mode[i] = true;
                            impact_start_times[i] = std::chrono::steady_clock::now();
                            impact_active[i] = true;
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " VR模式检测到卡死(" << (is_closing_direction ? "关闭方向" : "打开方向") << ")！目标位置差值:" << target_position_error << " rad，在限位范围内，开始" << (is_closing_direction ? "+" : "-") << cfg_IMPACT_CURRENT << "A冲击模式" << std::endl;
                        } else {
                            // 在限位范围外，不进行反冲，重置卡死检测状态
                            vr_stuck_cycle_count[i] = 0;
                            vr_stuck_check_positions[i] = current_pos;
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " VR模式检测到卡死(" << (is_closing_direction ? "关闭方向" : "打开方向") << ")！目标位置差值:" << target_position_error << " rad，但不在限位范围内，不进行反冲" << std::endl;
                        }
                    }
                } else {
                    // 目标位置与当前位置差值很小，重置卡死检测状态
                    vr_stuck_cycle_count[i] = 0;
                    vr_stuck_check_positions[i] = current_pos;
                }
            }
            
            // 卡死检测逻辑（只在对应方向的限位范围内才允许卡死检测和反冲）
            if (in_impact_range && !is_vr_control_mode) {
                auto detection_delay_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - stuck_detection_start_times[i]).count();
                if (detection_delay_elapsed >= cfg_STUCK_DETECTION_DELAY_MS && !stuck_detection_enabled[i]) {
                    stuck_detection_enabled[i] = true;
                    stuck_check_positions[i] = current_pos;
                    stuck_start_times[i] = std::chrono::steady_clock::now();
                }
                if (stuck_detection_enabled[i] && !stuck_detected[i] && !in_impact_mode[i]) {
                    float position_change = std::abs(current_pos - stuck_check_positions[i]);
                    auto stuck_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - stuck_start_times[i]).count();
                    // 只有在位置变化很小且时间足够长时才判定为卡死
                    if (position_change < cfg_STUCK_POSITION_THRESHOLD && stuck_elapsed > cfg_STUCK_DETECTION_TIME_MS) {
                        stuck_detected[i] = true;
                        in_impact_mode[i] = true;
                        impact_start_times[i] = std::chrono::steady_clock::now();
                        impact_active[i] = true;
                        std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 检测到卡死(" << (is_closing_direction ? "关闭方向" : "打开方向") << ")！开始" << (is_closing_direction ? "+" : "-") << cfg_IMPACT_CURRENT << "A冲击模式" << std::endl;
                    } else if (position_change >= cfg_STUCK_POSITION_THRESHOLD) {
                        // 位置有变化，重置卡死检测
                        stuck_check_positions[i] = current_pos;
                        stuck_start_times[i] = std::chrono::steady_clock::now();
                    }
                }
            }

            // 冲击模式逻辑（只在对应方向的限位范围内才允许反冲）
            if (in_impact_mode[i] && in_impact_range) {
                auto impact_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - impact_start_times[i]).count();
                if (impact_active[i]) {
                    if (impact_elapsed < cfg_IMPACT_DURATION_MS) {
                        // 根据运动方向确定冲击电流方向
                        if (is_closing_direction) {
                            // 关闭方向：在开限位附近，使用+冲击电流
                            new_torque[i] = cfg_IMPACT_CURRENT;
                        } else {
                            // 打开方向：在关限位附近，使用-冲击电流
                            new_torque[i] = -cfg_IMPACT_CURRENT;
                        }
                        if (impact_elapsed == 0) {
                            three_amp_current_usage_count++;
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 进入" << (is_closing_direction ? "+" : "-") << cfg_IMPACT_CURRENT << "A冲击模式 - 第" << three_amp_current_usage_count << "次" << std::endl;
                        }
                    } else {
                        impact_active[i] = false;
                        new_torque[i] = 0.0f;
                        impact_start_times[i] = std::chrono::steady_clock::now();
                    }
                } else {
                    if (impact_elapsed < cfg_IMPACT_INTERVAL_MS) {
                        new_torque[i] = 0.0f;
                    } else {
                        // 检查位置是否有变化，如果有变化则退出冲击模式
                        float position_change = std::abs(current_pos - stuck_check_positions[i]);
                        if (position_change > cfg_STUCK_POSITION_THRESHOLD) {
                            // 位置有变化，退出冲击模式，回到正常控制
                            in_impact_mode[i] = false;
                            stuck_detected[i] = false;
                            stuck_detection_enabled[i] = false;
                            vr_stuck_cycle_count[i] = 0;  // 重置VR卡死检测计数
                            vr_stuck_check_positions[i] = current_pos;  // 重置VR卡死检测位置
                            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i] << " 冲击成功，退出冲击模式" << std::endl;
                        } else {
                            // 位置无变化，继续冲击
                            impact_active[i] = true;
                            impact_start_times[i] = std::chrono::steady_clock::now();
                        }
                    }
                }
            }
        }

        // 检查所有夹爪是否都稳定（到达目标位置或稳定状态）
        all_reached = true;
        // [回退1.2.2] VR模式用位置误差判断，非VR模式用稳定检测
        for (size_t i = 0; i < Joint_address_list.size(); ++i) {
            if (Joint_online_list[i]) {
                // VR控制模式下，使用位置精度判断
                if (is_vr_control_mode) {
                    float current_pos = state[i][1];
                    float target_pos = mapped_positions[i];
                    float pos_error = target_pos - current_pos;
                    // 计算误差阈值
                    float total_range = std::abs(joint_end_positions[i] - joint_start_positions[i]);
                    float min_error = total_range * (cfg_MIN_ERROR_PERCENT / 100.0f);
                    if (std::abs(pos_error) > min_error) {
                        all_reached = false;
                        break;
                    }
                } else {
                    // 非VR控制模式下，需要稳定检测
                    if (!is_stable[i]) {
                        all_reached = false;
                        break;
                    }
                }
            }
        }

        // 发送电流
        {
            std::lock_guard<std::mutex> lock(sendtor_lock);
            // 只更新夹爪电机电流
            for (size_t i = 0; i < Joint_address_list.size(); ++i)
            {
                if (Joint_online_list[i])
                {
                    target_torque[i] = new_torque[i];
                }
                else
                {
                    target_torque[i] = 0.0f;
                }
            }
            target_update = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(cfg_DT * 1000)));
    }

    // 最终停止：如需保持电流则设置全局状态，否则直接清零
    // VR 模式下不清零电流，保持当前控制状态（VR 是持续控制）
    bool need_hold_current = false;
    std::vector<float> final_torque(target_torque.size(), 0.0f);

    // VR 模式：不清零电流，直接返回，让下一次 move_paw 继续控制
    if (is_vr_control_mode) {
        // 记录本次目标位置
        for (size_t i = 0; i < Joint_address_list.size(); ++i) {
            if (Joint_online_list[i]) {
                last_move_paw_target_positions[i] = current_target_positions[i];
            }
        }
        // VR 模式不清零，直接返回
        return PawMoveState::LEFT_REACHED_RIGHT_REACHED;
    }

    // 非 VR 模式：正常处理退出逻辑
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (!Joint_online_list[i])
        {
            continue;
        }

        // 记录本次目标位置
        last_move_paw_target_positions[i] = current_target_positions[i];

        // 非VR模式：如果启用了"100位置则维持电流"功能，优先检查目标位置是否为100（允许0.1%误差）
        if (cfg_ENABLE_100_POSITION_HOLDING_CURRENT && current_target_positions[i] >= 99.9)
        {
            // 目标位置是100，直接设置保持夹持电流
            final_torque[i] = cfg_GRIP_HOLDING_CURRENT;
            is_holding_object_global[i] = true;
            holding_current_values[i] = cfg_GRIP_HOLDING_CURRENT;  // 设置维持电流值，供control_thread使用
            last_exit_with_grip_current[i] = true;  // 记录退出时处于维持夹持电流状态
            need_hold_current = true;
            std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i]
                      << " 退出move_paw（目标位置100%），设置维持夹持电流 " << cfg_GRIP_HOLDING_CURRENT << "A" << std::endl;
        }
        // 检查是否检测到夹取（电流>1A）
        else if (is_stable[i] && is_holding_object[i])
        {
            float current_pos = state[i][1];
            float target_pos = mapped_positions[i];
            float error = target_pos - current_pos;
            if (error > 0)
            {
                // 检测到夹取（关爪方向），设置维持夹持电流
                final_torque[i] = cfg_GRIP_HOLDING_CURRENT;
                is_holding_object_global[i] = true;
                holding_current_values[i] = cfg_GRIP_HOLDING_CURRENT;  // 设置维持电流值，供control_thread使用
                last_exit_with_grip_current[i] = true;  // 记录退出时处于维持夹持电流状态
                need_hold_current = true;
                std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i]
                          << " 退出move_paw（检测到夹取），设置维持夹持电流 " << cfg_GRIP_HOLDING_CURRENT << "A，目标位置: " << current_target_positions[i] << "%" << std::endl;
            }
            else
            {
                // 开爪方向，清除保持电流状态
                final_torque[i] = 0.0f;
                is_holding_object_global[i] = false;
                holding_current_values[i] = 0.0f;  // 清除维持电流值
                last_exit_with_grip_current[i] = false;  // 记录退出时不是维持夹持电流状态
                std::cout << "[LEJU claw] 夹爪 " << Joint_address_list[i]
                          << " 退出move_paw（开爪方向，清除维持夹持状态），清零电流，目标位置: " << current_target_positions[i] << "%" << std::endl;
            }
        }
        else
        {
            // 未夹到东西，清除保持电流状态
            final_torque[i] = 0.0f;
            is_holding_object_global[i] = false;
            holding_current_values[i] = 0.0f;  // 清除维持电流值
            last_exit_with_grip_current[i] = false;  // 记录退出时不是维持夹持电流状态
        }
    }

    if (!need_hold_current)
    {
        clear_all_torque();
        
        // 多次发布0电流，确保control_thread发布0电流
        for (int i = 0; i < 5; ++i)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            {
                std::lock_guard<std::mutex> lock(sendtor_lock);
                // 确保所有电流都是0
                for (size_t j = 0; j < Joint_address_list.size(); ++j)
                {
                    if (Joint_online_list[j])
                    {
                        target_torque[j] = 0.0f;
                        is_holding_object_global[j] = false;
                        holding_current_values[j] = 0.0f;
                    }
                }
                target_update = true;
            }
        }
        
        // 等待control_thread处理完清零指令，确认3次，确保电流真正被清零
        int confirm_count = 0;
        const int required_confirm_count = 3;  // 需要确认3次
        const int max_wait_count = 50;  // 每次确认最多等待50ms
        
        while (confirm_count < required_confirm_count)
        {
            // 设置target_update，等待control_thread处理
            {
                std::lock_guard<std::mutex> lock(sendtor_lock);
                // 确保所有电流都是0
                for (size_t j = 0; j < Joint_address_list.size(); ++j)
                {
                    if (Joint_online_list[j])
                    {
                        target_torque[j] = 0.0f;
                        is_holding_object_global[j] = false;
                        holding_current_values[j] = 0.0f;
                    }
                }
                target_update = true;
            }
            
            // 等待target_update被处理（每次确认最多等待50ms）
            int wait_count = 0;
            bool update_processed = false;
            while (wait_count < max_wait_count)
            {
                {
                    std::lock_guard<std::mutex> lock(sendpos_lock);
                    if (!target_update)
                    {
                        // target_update已被处理，说明control_thread已经接收到清零指令
                        update_processed = true;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                wait_count++;
            }
            
            if (update_processed)
            {
                confirm_count++;
            }
            else
            {
                // 本次确认超时，但仍然继续尝试下一次确认
                std::cout << "[LEJU claw] 警告：第 " << (confirm_count + 1) << " 次确认超时（等待 " << wait_count << "ms）" << std::endl;
                confirm_count++;  // 仍然计数，但记录警告
            }
        }
        
        if (confirm_count < required_confirm_count)
        {
            std::cout << "[LEJU claw] 警告：等待control_thread处理清零指令超时（仅确认 " << confirm_count << " 次，需要 " << required_confirm_count << " 次）" << std::endl;
        }
        else
        {
            // std::cout << "[LEJU claw] 已确认control_thread接收到清零指令（确认 " << confirm_count << " 次）" << std::endl;
        }
    }
    else
    {
        // 即使设置了全局状态，也需要触发一次发送
        {
            std::lock_guard<std::mutex> lock(sendtor_lock);
            target_torque = final_torque;
            target_update = true;
        }
        
        // 等待control_thread处理完发送指令，确认3次，确保维持夹持电流被发送
        int confirm_count = 0;
        const int required_confirm_count = 3;  // 需要确认3次
        const int max_wait_count = 50;  // 每次确认最多等待50ms
        
        while (confirm_count < required_confirm_count)
        {
            // 设置target_update，等待control_thread处理
            {
                std::lock_guard<std::mutex> lock(sendtor_lock);
                // 确保维持夹持电流被设置
                target_torque = final_torque;
                target_update = true;
            }
            
            // 等待target_update被处理（每次确认最多等待50ms）
            int wait_count = 0;
            bool update_processed = false;
            while (wait_count < max_wait_count)
            {
                {
                    std::lock_guard<std::mutex> lock(sendpos_lock);
                    if (!target_update)
                    {
                        // target_update已被处理，说明control_thread已经接收到发送指令
                        update_processed = true;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                wait_count++;
            }
            
            if (update_processed)
            {
                confirm_count++;
            }
            else
            {
                // 本次确认超时，但仍然继续尝试下一次确认
                std::cout << "[LEJU claw] 警告：第 " << (confirm_count + 1) << " 次确认超时（等待 " << wait_count << "ms）" << std::endl;
                confirm_count++;  // 仍然计数，但记录警告
            }
        }
        
        if (confirm_count < required_confirm_count)
        {
            std::cout << "[LEJU claw] 警告：等待control_thread处理发送指令超时（仅确认 " << confirm_count << " 次，需要 " << required_confirm_count << " 次）" << std::endl;
        }
        else
        {
            // std::cout << "[LEJU claw] 已确认control_thread接收到发送指令（确认 " << confirm_count << " 次）" << std::endl;
        }
    }

    // 所有夹爪都到达目标位置
    return PawMoveState::LEFT_REACHED_RIGHT_REACHED;
}

float LeJuClaw::lowPassFilter(float input, float prevOutput, float alpha)
{
    return alpha * input + (1 - alpha) * prevOutput;
}

std::vector<float> LeJuClaw::create_zero_vector(size_t size)
{
    return std::vector<float>(size, 0.0f);
}

void LeJuClaw::send_torque_with_lock(const std::vector<float>& torques)
{
    std::lock_guard<std::mutex> lock(sendtor_lock);
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            target_torque[i] = torques[i];
        }
        else
        {
            target_torque[i] = 0.0f;
        }
    }
    target_update = true;
}

void LeJuClaw::send_torque_direct(const std::vector<float>& torques)
{
    if (!thread_running)
    {
        return;
    }
    
    // 直接使用run_torque_mode发送电流指令
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            // 检查是否夹爪电机
            bool is_claw_joint = false;
            for (size_t j = 0; j < Claw_joint_address.size(); ++j) {
                if (Joint_address_list[i] == Claw_joint_address[j]) {
                    is_claw_joint = true;
                    break;
                }
            }
            
            if (is_claw_joint) {
                float state_list[6];
                int errorcode = run_torque_mode(&ruiwo, Joint_address_list[i], torques[i], state_list);
                if (errorcode == 0)
                {
                    // 更新关节状态
                    std::vector<float> state;
                    state.assign(state_list, state_list + 6);
                    set_joint_state(i, state);
                }
            }
        }
    }
}

void LeJuClaw::send_velocity_direct(const std::vector<float>& velocities)
{
    if (!thread_running)
    {
        return;
    }
    
    // 直接使用run_vel_mode发送速度指令
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            // 检查是否夹爪电机
            bool is_claw_joint = false;
            for (size_t j = 0; j < Claw_joint_address.size(); ++j) {
                if (Joint_address_list[i] == Claw_joint_address[j]) {
                    is_claw_joint = true;
                    break;
                }
            }
            
            if (is_claw_joint) {
                float state_list[6];
                // 使用配置文件的Kp、Kd、Ki参数
                int errorcode = run_vel_mode(&ruiwo, Joint_address_list[i], velocities[i], 
                                           target_vel_kp[i], target_vel_kd[i], target_vel_ki[i], state_list);
                if (errorcode == 0)
                {
                    // 更新关节状态
                    std::vector<float> state;
                    state.assign(state_list, state_list + 6);
                    set_joint_state(i, state);
                }
            }
        }
    }
}

void LeJuClaw::set_torque(const std::vector<uint8_t> &index, const std::vector<double> &torque)
{
    if (!thread_running)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sendtor_lock);
        auto new_torque = target_torque;

        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_address_list[i] == DISABLE_ADDRESS)
            {
                continue;
            }

            for (size_t j = 0; j < index.size(); ++j)
            {
                if (Joint_address_list[i] - 1 == static_cast<int>(index[j]))
                {
                    if (std::find(Negtive_joint_address_list.begin(), Negtive_joint_address_list.end(), Joint_address_list[i]) != Negtive_joint_address_list.end())
                    {
                        new_torque[i] = -torque[j];
                    }
                    else
                    {
                        new_torque[i] = torque[j];
                    }
                    break;
                }
            }
        }

        target_torque = new_torque;
    }

    {
        std::lock_guard<std::mutex> lock(update_lock);
        target_update = true;
    }
}

void LeJuClaw::set_velocity(const std::vector<uint8_t> &index, const std::vector<double> &velocity)
{
    if (!thread_running)
    {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(sendvel_lock);
        auto new_velocity = target_velocity;

        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            if (Joint_address_list[i] == DISABLE_ADDRESS)
            {
                continue;
            }
            
            for (size_t j = 0; j < index.size(); ++j)
            {
                if (Joint_address_list[i] - 1 == static_cast<int>(index[j]))
                {
                    if (std::find(Negtive_joint_address_list.begin(), Negtive_joint_address_list.end(), Joint_address_list[i]) != Negtive_joint_address_list.end())
                    {
                        new_velocity[i] = -velocity[j];
                    }
                    else
                    {
                        new_velocity[i] = velocity[j];
                    }
                    break;
                }
            }
        }

        target_velocity = new_velocity;
    }

    {
        std::lock_guard<std::mutex> lock(update_lock);
        target_update = true;
    }
}

std::vector<double> LeJuClaw::get_positions()
{
    if (!thread_running)
    {
        return {};
    }

    std::vector<double> pose_double;
    int size = current_positions.size();
    pose_double.resize(size);
    std::lock_guard<std::mutex> lock(recvpos_lock);

    for (size_t i = 0; i < size; i++)
    {
        float joint_start_position = joint_start_positions[i];
        float joint_end_position = joint_end_positions[i];

        if (joint_end_position == joint_start_position)
        {
            // 如果关节范围为零，直接设置为默认值
            pose_double[i] = 0.0;
        }
        else
        {
            // 将实际位置映射为 0~100
            float actual_position = current_positions[i];
            double mapped_position = static_cast<double>(
                (actual_position - joint_start_position) * 100.0 /
                (joint_end_position - joint_start_position));

            // 确保映射值在 0~100 的范围内
            mapped_position = std::max(0.0, std::min(100.0, mapped_position));

            pose_double[i] = mapped_position;
        }
    }

    return pose_double;
}


std::vector<double> LeJuClaw::get_torque()
{
    if (!thread_running)
    {
        return {};
    }
    std::vector<double> torque_double;
    int size = current_torque.size();
    torque_double.resize(size);
    std::lock_guard<std::mutex> lock(recvtor_lock);
    for (size_t i = 0; i < size; i++)
    {
        torque_double[i] = static_cast<double>(current_torque[i]);
    }
    return torque_double;
}

std::vector<double> LeJuClaw::get_target_torque()
{
    if (!thread_running)
    {
        return {};
    }
    std::vector<double> target_torque_double;
    int size = target_torque.size();
    target_torque_double.resize(size);
    std::lock_guard<std::mutex> lock(sendtor_lock);
    for (size_t i = 0; i < size; i++)
    {
        target_torque_double[i] = static_cast<double>(target_torque[i]);
    }
    return target_torque_double;
}

std::vector<double> LeJuClaw::get_velocity()
{
    if (!thread_running)
    {
        return {};
    }
    std::vector<double> velocity_double;
    int size = current_velocity.size();
    velocity_double.resize(size);
    std::lock_guard<std::mutex> lock(recvvel_lock);
    for (size_t i = 0; i < size; i++)
    {
        velocity_double[i] = static_cast<double>(current_velocity[i]);
    }
    return velocity_double;
}

void LeJuClaw::set_joint_state(int index, const std::vector<float> &state)
{
    if (!thread_running)
    {
        return;
    }

    auto new_state = state;
    if (std::find(Negtive_joint_address_list.begin(), Negtive_joint_address_list.end(), new_state[0]) != Negtive_joint_address_list.end())
    {
        new_state[1] = -new_state[1];
        new_state[2] = -new_state[2];
        new_state[3] = -new_state[3];
    }

    {
        std::lock_guard<std::mutex> lock(state_lock);
        joint_status[index] = new_state;
    }
}

std::vector<std::vector<float>> LeJuClaw::get_joint_state()
{
    if (!thread_running)
    {
        return {};
    }

    std::lock_guard<std::mutex> lock(state_lock);
    return joint_status;
}

void LeJuClaw::close()
{
    thread_running = false;
    auto time0 = std::chrono::system_clock::now();
    auto time0_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time0.time_since_epoch()).count();
    while (!thread_end)
    {
        auto time1 = std::chrono::system_clock::now();
        auto time1_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(time1.time_since_epoch()).count();
        auto time_diff = time1_milliseconds - time0_milliseconds;
        if (time_diff > 1000)
        {
            std::cout << "[LEJU claw]:Threadend Timeout" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    disable();
    auto errorcode = close_canbus(&ruiwo);
    if (errorcode == 0)
    {
        std::cout << "[LEJU claw]:Canbus status: [Close]" << std::endl;
    }
    else
    {
        std::cout << "[LEJU claw]:Close canbus failed, Errorcode: " << "[" << errorcode << "]" << std::endl;
    }
}

LeJuClaw::MotorStateDataVec LeJuClaw::get_motor_state()
{
    MotorStateDataVec motor_states;
    
    auto  all_motor_addrs = Claw_joint_address;
    
    for(int i = 0 ; i < all_motor_addrs.size(); i++) {
        MotorStateData state;
        state.id = static_cast<uint8_t>(all_motor_addrs[i]);
        auto iter = std::find(Joint_address_list.begin(), Joint_address_list.end(), all_motor_addrs[i]);
        if(iter != Joint_address_list.end()) {
            state.state = State::Enabled;
        }   
        else {
            state.state = State::Disabled;
        }

        motor_states.push_back(state); 
    }
    return motor_states;
}

void LeJuClaw::set_claw_kpkd(int kp, int kd)
{
    bool need_update = false;
    
    // 遍历所有电机
    for (size_t i = 0; i < Joint_address_list.size(); ++i) {
        bool is_claw_joint = false;
        for (size_t j = 0; j < Claw_joint_address.size(); ++j) {
            if (Joint_address_list[i] == Claw_joint_address[j]) {
                is_claw_joint = true;
                break;
            }
        }
        
        if (is_claw_joint) {
            if (target_pos_kp[i] != kp || target_pos_kd[i] != kd) {
                need_update = true;
                break;
            }
        }
    }
    
    if (need_update) {
        for (size_t i = 0; i < Joint_address_list.size(); ++i) {
            // 检查是否夹爪电机
            bool is_claw_joint = false;
            for (size_t j = 0; j < Claw_joint_address.size(); ++j) {
                if (Joint_address_list[i] == Claw_joint_address[j]) {
                    is_claw_joint = true;
                    break;
                }
            }
            
            // 只对夹爪电机设置Kp和Kd
            if (is_claw_joint) {
                target_pos_kp[i] = kp;
                target_pos_kd[i] = kd;
                std::cout << "[LEJU claw]:设置夹爪电机 ID:" << Joint_address_list[i] << " kp=" << kp << " kd=" << kd << std::endl;
            }
        }
    }
}

void LeJuClaw::clear_all_torque()
{
    if (!thread_running)
    {
        return;
    }
    
    // 清零所有电流并清除全局保持电流状态
    {
        std::lock_guard<std::mutex> lock(sendtor_lock);
        for (size_t i = 0; i < Joint_address_list.size(); ++i)
        {
            target_torque[i] = 0.0f;
            is_holding_object_global[i] = false;
            holding_current_values[i] = 0.0f;
        }
        target_update = true;
    }
    
    std::cout << "[LEJU claw]:所有电流已清零，全局保持电流状态已清除" << std::endl;
}

void LeJuClaw::send_claw_torque_only(const std::vector<float>& torques)
{
    std::lock_guard<std::mutex> lock(sendtor_lock);
    
    // 只更新夹爪电机的电流，完全独立控制
    for (size_t i = 0; i < Joint_address_list.size(); ++i)
    {
        if (Joint_online_list[i])
        {
            // 检查是否夹爪电机
            bool is_claw_joint = false;
            for (size_t j = 0; j < Claw_joint_address.size(); ++j) {
                if (Joint_address_list[i] == Claw_joint_address[j]) {
                    is_claw_joint = true;
                    break;
                }
            }
            
            if (is_claw_joint) {
                // 只更新夹爪电机的电流
                target_torque[i] = torques[i];
            }
        }
        else
        {
            target_torque[i] = 0.0f;
        }
    }
    target_update = true;
}

// 计算刹车速度系数（恢复自1.2.2版本）
float LeJuClaw::calculate_brake_speed_factor(float current_position_percent, float target_position_percent,
                                             float start_position, float end_position)
{
    // 计算当前位置在行程中的百分比
    float total_range = std::abs(end_position - start_position);
    if (total_range == 0.0f) {
        return 1.0f; // 如果行程为0，返回正常速度
    }

    // 将当前位置转换为百分比 (0-100%)
    float current_percent = ((current_position_percent - start_position) / total_range) * 100.0f;
    current_percent = std::max(0.0f, std::min(100.0f, current_percent));

    // 计算目标位置百分比
    float target_percent = ((target_position_percent - start_position) / total_range) * 100.0f;
    target_percent = std::max(0.0f, std::min(100.0f, target_percent));

    float speed_factor = 1.0f; // 默认正常速度

    // 刹车逻辑：基于目标位置决定刹车策略
    // 检查目标位置是否在两端各cfg_BRAKE_RANGE_PERCENT的范围内
    bool target_in_start_range = (target_percent <= cfg_BRAKE_RANGE_PERCENT);
    bool target_in_end_range = (target_percent >= (100.0f - cfg_BRAKE_RANGE_PERCENT));

    if (target_in_start_range || target_in_end_range) {
        // 目标位置在两端刹车范围内，使用正常的限位刹车逻辑
        bool moving_towards_start = (target_percent < current_percent);
        bool moving_towards_end = (target_percent > current_percent);

        if (moving_towards_start) {
            // 向开爪方向运动 (0%方向)
            if (current_percent <= cfg_BRAKE_RANGE_PERCENT) {
                // 在开爪端刹车范围内，进行减速
                float distance_to_start = current_percent;
                float normalized_distance = distance_to_start / cfg_BRAKE_RANGE_PERCENT; // 0-1
                // 使用指数曲线进行减速，距离越近速度越慢
                speed_factor = cfg_BRAKE_MIN_SPEED_FACTOR +
                              (1.0f - cfg_BRAKE_MIN_SPEED_FACTOR) *
                              std::pow(normalized_distance, cfg_BRAKE_CURVE_EXPONENT);
            }
        } else if (moving_towards_end) {
            // 向关爪方向运动 (100%方向)
            if (current_percent >= (100.0f - cfg_BRAKE_RANGE_PERCENT)) {
                // 在关爪端刹车范围内，进行减速
                float distance_to_end = 100.0f - current_percent;
                float normalized_distance = distance_to_end / cfg_BRAKE_RANGE_PERCENT; // 0-1
                // 使用指数曲线进行减速，距离越近速度越慢
                speed_factor = cfg_BRAKE_MIN_SPEED_FACTOR +
                              (1.0f - cfg_BRAKE_MIN_SPEED_FACTOR) *
                              std::pow(normalized_distance, cfg_BRAKE_CURVE_EXPONENT);
            }
        }
    } else {
        // 目标位置在中间范围内，使用目标位置刹车逻辑
        float distance_to_target = std::abs(current_percent - target_percent);
        if (distance_to_target <= cfg_TARGET_PROXIMITY_BRAKE_PERCENT) {
            // 在目标位置附近，使用目标位置刹车逻辑
            float normalized_distance = distance_to_target / cfg_TARGET_PROXIMITY_BRAKE_PERCENT; // 0-1
            speed_factor = cfg_TARGET_PROXIMITY_MIN_FACTOR +
                          (1.0f - cfg_TARGET_PROXIMITY_MIN_FACTOR) *
                          std::pow(normalized_distance, cfg_BRAKE_CURVE_EXPONENT);
        }
    }

    // 确保速度系数在合理范围内
    speed_factor = std::max(cfg_BRAKE_MIN_SPEED_FACTOR, std::min(1.0f, speed_factor));

    return speed_factor;
}
