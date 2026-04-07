#include <algorithm>
#include "touch_hand_controller.h"
#include "dexhand_def.h"
#include "touch_dexhand.h"
#include "stark_dexhand.h"
#include "revo1_hand_can_customed.h"
#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include <iostream>
#include <fstream>
#include "json.hpp"

namespace eef_controller {
using namespace dexhand;

// open dual hand positions
const UnsignedDualHandsArray kDualHandOpenPositions = {kOpenFingerPositions, kOpenFingerPositions};
// close dual hand positions
const UnsignedDualHandsArray kDualHandClosePositions = {kCloseFingerPositions, kCloseFingerPositions};     



bool DexhandController::ParseActionSequenceFile(const std::string & gesture_file_path, std::vector<GestureInfoPtr> &gesture_infos)
{
    nlohmann::json data;
    try {
        std::ifstream file(gesture_file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to open gesture config file: " << gesture_file_path << std::endl;
            return false;
        }

        file >> data;

        for (auto &item : data) {
            GestureInfoPtr gest_info = std::make_shared<GestureInfo>();
            gest_info->gesture_name = item["gesture_name"];
            gest_info->description = item["description"];

            // action_sequences
            const auto &action_sequences = item["action_sequences"];
            for (auto &as : action_sequences) {
                eef_controller::ActionSequenceData as_data;
                as_data.duration_ms = as["duration_ms"];
                if (as["positions"].is_array() && as["positions"].size() == 6) {
                    for (int i = 0; i < 6; i++) {
                        as_data.positions[i] = as["positions"][i];
                    }
                }
                if (as["speeds"].is_array() && as["speeds"].size() == 6) {
                    for (int i = 0; i < 6; i++) {
                        as_data.speeds[i] = as["speeds"][i];
                    }
                }
                if (as["forces"].is_array() && as["forces"].size() == 6) {
                    for (int i = 0; i < 6; i++) {
                        as_data.forces[i] = as["forces"][i];
                    }
                }
                gest_info->action_sequences.emplace_back(as_data);
            }

            // alias
            const auto &alias = item["alias"];
            for (auto &alias_item : alias) {
                gest_info->alias.emplace_back(alias_item);
            }

            gesture_infos.emplace_back(gest_info);
        }
    }
    catch (std::exception &e) {
        std::cout << "parse gesture file failed, e.what(): " << e.what() << "\n";
        return false;
    }

    return true;
}

DexhandController::DexhandController(const std::string &action_sequences_path, bool is_touch_dexhand, bool is_can_protocol): 
action_sequences_path_(action_sequences_path), is_touch_dexhand_(is_touch_dexhand), is_can_protocol_(is_can_protocol) {
    finger_status_ = {
        FingerStatus(),
        FingerStatus()
    };
    finger_touch_status_ = {
        TouchSensorStatusArray(),
        TouchSensorStatusArray()
    };
}

DexhandController::~DexhandController(){
    this->close();
}

bool DexhandController::init_touch_dexhand()
{
    // init modbus 
    std::array<bool, 2> successes = {true, true};
    const std::string kLeftHandPort = "/dev/stark_serial_touch_L";
    const std::string kRightHandPort = "/dev/stark_serial_touch_R";
    right_dexhand_ = TouchDexhand::Connect(kRightHandPort, 1, 115200);
    if(right_dexhand_ == nullptr) {
        std::cerr << "\033[33m[DexhandController] Failed to connect to right dexhand at `" << kRightHandPort << "` with baudrate 115200\033[0m" << std::endl;
        successes[1] = false;
    }
    else {
        int count = 3;
        while(count > 0) {
        DeviceInfo_t dev_info;
        if (right_dexhand_->getDeviceInfo(dev_info)) {
                std::cout <<"------------------------------------------------------------------\n";
                std::cout << "[DexhandController] Connected to right dexhand:\n" << dev_info;
                std::cout <<"------------------------------------------------------------------\n";
                if(auto hand = dynamic_cast<dexhand::TouchDexhand*>(right_dexhand_.get())) {
                    hand->enableTouchSensor(0xFF);
                }
                break;
            }
            count --;
        }
        if(count <= 0) {
            std::cerr << "\033[33m[DexhandController] Failed to get right hand device info, timeout. \033[0m" << std::endl;
        }
    }
    
    left_dexhand_ = TouchDexhand::Connect(kLeftHandPort, 1, 115200);
    if(left_dexhand_ == nullptr) {
        std::cerr << "\033[33m[DexhandController] Failed to connect to left dexhand at `"<< kLeftHandPort << "`with baudrate 115200\033[0m" << std::endl;
        successes[0] = false;
    }
    else {
        DeviceInfo_t dev_info;
        int count = 3;
        while(count > 0) {
            DeviceInfo_t dev_info;
            if (left_dexhand_->getDeviceInfo(dev_info)) {
                std::cout <<"------------------------------------------------------------------\n";
                std::cout << "[DexhandController] Connected to left dexhand:\n" << dev_info;
                std::cout <<"------------------------------------------------------------------\n";
                if(auto hand = dynamic_cast<dexhand::TouchDexhand*>(left_dexhand_.get())) {
                    hand->enableTouchSensor(0xFF);
                }
                break;
            }
            count --;
        }
        if(count <= 0) {
            std::cerr << "\033[33m[DexhandController] Failed to get left hand device info, timeout. \033[0m" << std::endl;
        }
    }

    // all success are false, return false
    if (std::none_of(successes.begin(), successes.end(), [](bool v) { return v; })) {
        return false;
    }

    return true;
}

bool DexhandController::init_normal_dexhand()
{
   std::array<bool, 2> successes = {true, true};
    const std::string kLeftHandPort = "/dev/stark_serial_L";
    const std::string kRightHandPort = "/dev/stark_serial_R";
    right_dexhand_ = StarkDexhand::Connect(kRightHandPort);
    if(right_dexhand_ == nullptr) {
        std::cerr << "\033[33m[DexhandController] Failed to connect to right dexhand at `" << kRightHandPort << "` with baudrate 115200\033[0m" << std::endl;
        successes[1] = false;
    }
    else {
        int count = 3;
        while(count > 0) {
        DeviceInfo_t dev_info;
            if (right_dexhand_->getDeviceInfo(dev_info)) {
                std::cout <<" ------------------------------------------------------------------\n";
                std::cout << "[DexhandController] Connected to right dexhand:\n" << dev_info;
                std::cout <<" ------------------------------------------------------------------\n";
                break;
            }
            count --;
        }
        if(count <= 0) {
            std::cerr << "\033[33m[DexhandController] Failed to get right hand device info, timeout. \033[0m" << std::endl;
        }
    }
    
    left_dexhand_ = StarkDexhand::Connect(kLeftHandPort);
    if(left_dexhand_ == nullptr) {
        std::cerr << "\033[33m[DexhandController] Failed to connect to left dexhand at `"<< kLeftHandPort << "`with baudrate 115200\033[0m" << std::endl;
        successes[0] = false;
    }
    else {
        DeviceInfo_t dev_info;
        int count = 3;
        while(count > 0) {
            DeviceInfo_t dev_info;
            if (left_dexhand_->getDeviceInfo(dev_info)) {
                std::cout <<" ------------------------------------------------------------------\n";
                std::cout << "[DexhandController] Connected to left dexhand:\n" << dev_info;
                std::cout <<" ------------------------------------------------------------------\n";
                break;
            }
            count --;
        }
        if(count <= 0) {
            std::cerr << "\033[33m[DexhandController] Failed to get left hand device info, timeout. \033[0m" << std::endl;
        }
    }

    // all success are false, return false
    if (std::none_of(successes.begin(), successes.end(), [](bool v) { return v; })) {
        return false;
    }

    return true;
}

bool DexhandController::init_revo1_normal_can_customed()
{
    using namespace canbus_sdk;
    auto config_file = ConfigParser::getDefaultConfigFilePath();

    /** 配置文件读取与解析 */
    canbus_sdk::ConfigParser parser;
    if (!parser.parseFromFile(config_file)) { // 会抛异常(无所谓), 配置文件都解析失败,程序就没法玩了
        printf("\033[31m[Revo1HandController] ERROR: Failed to parse config file: %s\033[0m\n", config_file.c_str());
        return false;
    }

    /** 获取CAN总线配置, 没有就返回失败 */
    auto canbus_configs = parser.getCanBusConfigs();
    if (canbus_configs.empty()) {
        printf("\033[31m[Revo1HandController] ERROR: No CAN bus configurations found in config file\033[0m\n");
        return false;
    }

    canbus_sdk::CanBusController::getInstance().init(); // 重复初始化无所谓
    std::vector<DeviceConfig> revo1_devices;
    for (const auto &canbus_config : canbus_configs) { 
        auto devices = parser.getDevices(canbus_config.name, canbus_sdk::DeviceType::REVO1_HAND);
        if(devices.empty()) {
            continue;
        }

        // 找到REVO1 DEXHAND配置在CAN总线上，那么就可以后续的初始化操作
        auto result = canbus_sdk::CanBusController::getInstance().openCanBus(
            canbus_config.name.c_str(), canbus_config.type, canbus_config.bitrate);
        if (result.has_value()) {
            // 总线打开成功，准备把这些设备后续注册到总线
            for (auto &d : devices) {
                revo1_devices.push_back(d);
            }
        }
        else {
            printf("\033[31m[Revo1HandController] ERROR: 打开CAN总线 %s 失败: %s\033[0m\n", canbus_config.name.c_str(),canbus_sdk::errorToString(result.error()));
        }
    }

    if(revo1_devices.empty()) {
        printf("\033[31m[Revo1HandController] ERROR: 没有找到REVO1 DEXHAND设备\033[0m\n");
        return false;
    }

    // 初始化设备!
    // 定义设备初始化函数
    auto init_device = [&revo1_devices](const std::string& device_name, const char* hand_desc) -> std::unique_ptr<Revo1CanDexhand> {
        auto it = std::find_if(revo1_devices.begin(), revo1_devices.end(),
            [&](const canbus_sdk::DeviceConfig& device) { return device.name == device_name; });

        if (it != revo1_devices.end()) {
            // 检查设备是否被忽略
            if (it->ignore) {
                printf("\033[33m[Revo1HandController] WARN: %s Revo1CanDexhand 设备被忽略，跳过初始化\033[0m\n", hand_desc);
                return nullptr;
            }

            auto hand = Revo1CanDexhand::Connect(*it);
            printf("%s[Revo1HandController] %s%s Revo1CanDexhand 设备%s\033[0m\n",
                   hand ? "\033[32m" : "\033[31m", hand ? "成功初始化" : "初始化", hand_desc, hand ? "" : "失败");
            if(hand) {
                DeviceInfo_t dev_info;
                if (hand->getDeviceInfo(dev_info)) {
                    std::cout << " ------------------------------------------------------------------\n";
                    std::cout << "[Revo1HandController] Connected to " << device_name << ":\n" << dev_info;
                    std::cout << " ------------------------------------------------------------------\n";
                }       
            }
            return hand;
        }
        return nullptr;
    };

    // 初始化左右手设备
    right_dexhand_ = init_device("Rhand_revo1_hand", "右手");
    left_dexhand_ = init_device("Lhand_revo1_hand", "左手");

    if (!left_dexhand_ && !right_dexhand_) {
        printf("\033[31m[Revo1HandController] ERROR: 未找到任何可用的REVO1灵巧手设备\033[0m\n");
        return false;
    }

    return true;
}

bool DexhandController::init()
{
    /* read gesture file */
    std::vector<GestureInfoPtr> gesture_infos;
    if (!this->ParseActionSequenceFile(action_sequences_path_, gesture_infos)) {
        std::cerr << "[DexhandController] Failed to parse action sequence file: " << action_sequences_path_ << std::endl;
    }

    /* add gesture to map */
    for(const auto &gesture_info : gesture_infos) {
        gesture_map_[gesture_info->gesture_name] = gesture_info;
        for (const auto &alias_name : gesture_info->alias) {
            gesture_map_[alias_name] = gesture_info;
        }
    }

    if(is_can_protocol_) {
        // CAN协议，使用CAN自定义协议初始化
        if (!init_revo1_normal_can_customed()) {
            return false;
        }
    }
    else {
        // 485协议，根据是否为触觉手选择初始化方式
        if (is_touch_dexhand_ && !init_touch_dexhand()) {
            return false;
        }
        else if (!is_touch_dexhand_ && !init_normal_dexhand()) {
            return false;
        }
    }

    // start control thread
    running_ = true;
    control_thread_ = std::thread(&DexhandController::control_thread_func, this);
    gesture_thread_ = std::thread(&DexhandController::gesture_thread_func, this);

    /* close && open*/
    // Wait for control thread to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    this->send_position(kDualHandClosePositions);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    this->send_position(kDualHandOpenPositions);

    return true;
}

bool DexhandController::close(){
    if (running_) {
        running_ = false;
        task_cv_.notify_all();
        std::cout << "[DexhandController] Control thread joining." << std::endl;
        if (control_thread_.joinable()) {
            control_thread_.join();
        }
        std::cout << "[DexhandController] Work thread joining." << std::endl;
        task_cv_.notify_all();
        if (gesture_thread_.joinable()) {
            gesture_thread_.join();
        }
        std::cout << "[DexhandController] all thread joined." << std::endl;
    }

    left_dexhand_ = nullptr;
    right_dexhand_ = nullptr;

    return true;
}

void DexhandController::control_thread_func()
{
    while (running_) {
        // control right hand : positon mode
        if (r_position_updated_ && right_dexhand_ != nullptr) {
            right_dexhand_->setFingerPositions(right_position_);
            r_position_updated_ = false;
        }
        // control left hand : positon mode
        if (l_position_updated_ && left_dexhand_ != nullptr) {
            left_dexhand_->setFingerPositions(left_position_); 
            l_position_updated_ = false;
        }
        
        // control right hand : speed mode
        if (r_speed_updated_ && right_dexhand_ != nullptr) {
            right_dexhand_->setFingerSpeeds(right_speed_);
            r_speed_updated_ = false;
        }
        // control left hand : speed mode
        if (l_speed_updated_ && left_dexhand_ != nullptr) {
            left_dexhand_->setFingerSpeeds(left_speed_);
            l_speed_updated_ = false;
        }
       
        // read finger status & touch status
        {
            if(left_dexhand_) {
                dexhand::FingerStatus left_temp_status;
                if (left_dexhand_->getFingerStatus(left_temp_status)) {
                    std::lock_guard<std::mutex> finger_lock(finger_status_mutex_);
                    finger_status_[0] = left_temp_status;
                }
            }
            if (right_dexhand_) {
                dexhand::FingerStatus right_temp_status;
                if (right_dexhand_->getFingerStatus(right_temp_status)) {
                    std::lock_guard<std::mutex> finger_lock(finger_status_mutex_);
                    finger_status_[1] = right_temp_status;
                }
            }
        }
        
        {
            if(left_dexhand_) {
                // 只有触觉手才有此接口!
                if (left_dexhand_->getDexHandFwType() == DexHandFwType::V1_TOUCH) {
                    if(auto hand = dynamic_cast<dexhand::TouchDexhand*>(left_dexhand_.get())) {
                        std::lock_guard<std::mutex> touch_lock(touch_status_mutex_);
                        finger_touch_status_[0] = hand->getTouchStatus();
                    }
                }
            }
            
            if (right_dexhand_) {
                // 只有触觉手才有此接口!
                if (right_dexhand_->getDexHandFwType() == DexHandFwType::V1_TOUCH) {
                    if(auto hand = dynamic_cast<dexhand::TouchDexhand*>(right_dexhand_.get())) {
                        std::lock_guard<std::mutex> touch_lock(touch_status_mutex_);
                        finger_touch_status_[1] = hand->getTouchStatus();
                    }
                }
            }
        }
        
        // Control at specified frequency
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/control_frequency_));
    }

    std::cout << "[DexhandController] Control loop exited." << std::endl;
}


void DexhandController::gesture_thread_func()
{
    while (running_) {
        TaskFunc task;
        {
            std::unique_lock<std::mutex> lock(task_mutex_);
            task_cv_.wait(lock, [this] { return current_task_.has_value() || !running_; });
            if (!running_ && !current_task_.has_value()) {
                return;
            }

            // 取出当前任务并清空任务变量
            if (current_task_.has_value()) {
                task = std::move(*current_task_);
                current_task_.reset();
            }
        }

        //
        if (task) {
            task();
        }
    }
}

void DexhandController::send_left_position(const UnsignedFingerArray & position)
{
    if (!left_dexhand_) {
        // std::cerr << "[DexhandController] Left dexhand is not connected." << std::endl;
    }

    // update right position
    left_position_ = position;
    l_position_updated_ = true;
}

void DexhandController::send_right_position(const UnsignedFingerArray & position)
{
    if (!right_dexhand_) {
        // std::cerr << "[DexhandController] Right dexhand is not connected." << std::endl;
    }

    // update right position
    right_position_ = position;
    r_position_updated_ = true;
}

void DexhandController::send_position(const UnsignedDualHandsArray & finger_positions)
{
    if (left_dexhand_) {
        left_position_ = finger_positions[0];
        l_position_updated_ = true;
    } else {
        // std::cerr << "[DexhandController] Left dexhand is not connected." << std::endl;
    }

    if (right_dexhand_) {
        right_position_ = finger_positions[1]; 
        r_position_updated_ = true;
    } else {
        // std::cerr << "[DexhandController] Right dexhand is not connected." << std::endl;
    }
}

void DexhandController::send_left_speed(const FingerArray & speeds)
{
    if (!left_dexhand_) {
        // std::cerr << "[DexhandController] Left dexhand is not connected." << std::endl;
    }

    // update left speed
    left_speed_ = speeds;
    l_speed_updated_ = true;
}

void DexhandController::send_right_speed(const FingerArray & speeds)
{
    if (!right_dexhand_) {
        // std::cerr << "[DexhandController] Right dexhand is not connected." << std::endl;
    }

    // update right speed
    right_speed_ = speeds;
    r_speed_updated_ = true;
}

void DexhandController::send_speed(const DualHandsArray & speeds)
{
    if (left_dexhand_) {
        left_speed_ = speeds[0];
        l_speed_updated_ = true;
    } else {
        // std::cerr << "[DexhandController] Left dexhand is not connected." << std::endl;
    }

    if (right_dexhand_) {
        right_speed_ = speeds[1];
        r_speed_updated_ = true;
    } else {
        // std::cerr << "[DexhandController] Right dexhand is not connected." << std::endl;
    }
}

FingerStatusArray DexhandController::get_finger_status()
{
    std::lock_guard<std::mutex> lock(finger_status_mutex_);
    return finger_status_;
}   

FingerTouchStatusArray DexhandController::get_touch_status()
{
    std::lock_guard<std::mutex> lock(touch_status_mutex_);
    return finger_touch_status_;
}

bool DexhandController::set_hand_force_level(GripForce level)
{
    return set_right_hand_force_level(level) || set_left_hand_force_level(level);
}

bool DexhandController::set_right_hand_force_level(GripForce level)
{   
    bool ret = false;

    if (right_dexhand_) {
        // TODO: 处理 modbus连接失败阻塞的情况!
        right_dexhand_->setGripForce(level);
        ret = true;
    }

    return ret;
}

bool DexhandController::set_left_hand_force_level(GripForce level)
{
    bool ret = false;
    if (left_dexhand_) {
        // TODO: 处理 modbus连接失败阻塞的情况!
        left_dexhand_->setGripForce(level);
        ret = true;
    }

    return ret;
}

bool DexhandController::enable_left_hand_touch_sensor(uint8_t mask)
{
    bool ret = false;
    if (left_dexhand_) {
        if (left_dexhand_->getDexHandFwType() == DexHandFwType::V1_TOUCH) {
            if(auto hand = dynamic_cast<dexhand::TouchDexhand*>(left_dexhand_.get())) {
                hand->enableTouchSensor(mask);
                ret = true;
            }
            else {
                std::cerr << "[DexhandController] Left dexhand is not a touch dexhand." << std::endl;
                return false;
            }
        }
        else {
            std::cerr << "[DexhandController] Left dexhand is not a touch dexhand." << std::endl;
            return false;
        }
    }

    return ret;
}   

bool DexhandController::enable_right_hand_touch_sensor(uint8_t mask)
{
    bool ret = false;
    if (right_dexhand_) {
        // 只有触觉手才可以执行
        if (right_dexhand_->getDexHandFwType() == DexHandFwType::V1_TOUCH) {
            if(auto hand = dynamic_cast<dexhand::TouchDexhand*>(right_dexhand_.get())) {
                hand->enableTouchSensor(mask);
                ret = true;
            }
            else {
                std::cerr << "[DexhandController] Right dexhand is not a touch dexhand." << std::endl;
                return false;
            }
        }
        else {
            std::cerr << "[DexhandController] Right dexhand is not a touch dexhand." << std::endl;
            return false;
        }
    }

    return ret;
}   

std::vector<GestureInfoPtr> DexhandController::list_gestures()
{
    std::vector<GestureInfoPtr> gestures;
    for (const auto &gesture : gesture_map_) {
        if(std::find(gestures.begin(), gestures.end(), gesture.second) == gestures.end()) {
            gestures.push_back(gesture.second);
        }
    }
    return gestures;
}


bool DexhandController::execute_gestures(const GestureExecuteInfoVec& gesture_tasks, std::string &err_msg)
{
    bool no_gesture = false;
    err_msg = "Not found gesture:";

    BatchGestureTask batch_task;
    for(const auto& task: gesture_tasks) {
        if (gesture_map_.find(task.gesture_name) == gesture_map_.end()) {
            no_gesture = true;
            err_msg += "'" + task.gesture_name + "', ";
            continue;
        }
        /* add to batch! */
        batch_task.emplace_back(GestureExecuteTask{task.hand_side, gesture_map_[task.gesture_name]});        
    }

    if(batch_task.size() == 0) {
        // std::cout << "BrainCoController execute_gestures: no gesture to execute.\n";
        err_msg = "no gesture to execute.";
        return false;
    }

    std::unique_lock<std::mutex> lock(task_mutex_);
    current_task_ = [this, batch_task]() {
        gesture_executing_ = true;          /* gesture executing flag. */
        abort_running_task_flag_ = false;   /* reset abort flag. */

        // 手势执行
        bool batch_task_aborted = false;
        for(auto &task : batch_task) {
            if(!running_) return;    /* thread exit! */
            // std::cout << "gesture_thread_func execute: " << task.gesture->gesture_name << "\n";
            for (auto &as : task.gesture->action_sequences) {
                if(!running_) return; /* thread exit! */
                if(abort_running_task_flag_) {
                    // std::cout << "gesture_thread_func: previous task abort:" << task.gesture->gesture_name << "\n";;
                    batch_task_aborted = true;
                    break;  /* abort! 被新的手势请求打断! */
                }
                
                // std::cout << "gesture_thread_func: send_position:";
                // for(int i = 0; i < 6; i++) {
                //     std::cout << " " << static_cast<int>(as.positions[i]);
                // }
                // std::cout << "\n";

                if(task.hand_side == HandSide::LEFT) {
                    UnsignedFingerArray left_positions;
                    for(int i = 0; i < 6; i++) {
                        left_positions[i] = as.positions[i];
                    }
                    this->send_left_position(left_positions);
                }
                else if(task.hand_side == HandSide::RIGHT){
                    UnsignedFingerArray right_positions;
                    for(int i = 0; i < 6; i++) {
                        right_positions[i] = as.positions[i];
                    }
                    this->send_right_position(right_positions);
                }
                else {
                    UnsignedFingerArray left_positions;
                    UnsignedFingerArray right_positions;
                    for(int i = 0; i < 6; i++) {
                        left_positions[i] = as.positions[i];
                        right_positions[i] = as.positions[i];
                    }
                    this->send_left_position(left_positions);
                    this->send_right_position(right_positions);
                }

                /* sleep duration */
                if(!this->sleep_for_100ms(std::max(1, as.duration_ms/100))) {
                    batch_task_aborted = true;
                    break;
                }
            } // range-for action_sequences of task.

            if(batch_task_aborted) {
                std::cout << "gesture_thread_func: previous task abort:" << task.gesture->gesture_name << "\n";
                break;
            }
        } // range-for batch task.

        gesture_executing_ = false;          /* gesture executing flag. */
    };
    abort_running_task_flag_ = true;        /* abort previous gesture */
    task_cv_.notify_all();

    // std::unique_lock<std::mutex> lock(task_mutex_);
    // current_task_ = [this, batch_task]() {
    //     gesture_executing_ = true;
    //     ActionSeqDataTypeVec r_action_seq_data;
    //     ActionSeqDataTypeVec l_action_seq_data;

    //     for (const auto& task : batch_task) {
    //         for(const auto& action_seq : task.gesture->action_sequences) {
    //             ActionSeqDataType as;
    //             as.duration_ms = action_seq.duration_ms + 500;
    //             for(int i = 0; i < 6; i++) {
    //                 as.positions[i] = action_seq.positions[i];
    //                 as.speeds[i] = action_seq.speeds[i];
    //                 as.forces[i] = action_seq.forces[i];
    //             }
    //             if (task.hand_side == HandSide::LEFT) {
    //                 l_action_seq_data.push_back(as);
    //             }
    //             else if (task.hand_side == HandSide::RIGHT) {
    //                 r_action_seq_data.push_back(as);
    //             }
    //             else {
    //                 l_action_seq_data.push_back(as);
    //                 r_action_seq_data.push_back(as);
    //             }
    //         }   
    //     }

    //     if(l_action_seq_data.size() > 0) {
    //         if(left_dexhand_) {
    //             left_dexhand_->setActionSequence(ActionSequenceId_t::ACTION_SEQUENCE_ID_CUSTOM_GESTURE1, l_action_seq_data);
    //             left_dexhand_->runActionSequence(ActionSequenceId_t::ACTION_SEQUENCE_ID_CUSTOM_GESTURE1);
    //         }
    //     }
    //     if(r_action_seq_data.size() > 0) {
    //         if(right_dexhand_) {
    //             right_dexhand_->setActionSequence(ActionSequenceId_t::ACTION_SEQUENCE_ID_CUSTOM_GESTURE1, r_action_seq_data);
    //             right_dexhand_->runActionSequence(ActionSequenceId_t::ACTION_SEQUENCE_ID_CUSTOM_GESTURE1);
    //         }
    //     }
    //     gesture_executing_ = false;
    // };
    // task_cv_.notify_all();

    return true;
}

bool DexhandController::is_gesture_executing()
{
    return gesture_executing_;
}

bool DexhandController::sleep_for_100ms(int ms_count)
{
    int i = 0;
    while(i < ms_count) {
        if(abort_running_task_flag_) {
            std::cout << "sleep_for_100ms: abort!\n";
            return false;
        }

        /* thread sleep for 100ms loop. */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // std::cout << "sleep_for 100ms ...\n";
        i++;
    }

    return true;
}
} // namespace eef_controller
