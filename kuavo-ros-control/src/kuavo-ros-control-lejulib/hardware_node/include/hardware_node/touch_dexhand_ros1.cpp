#include "touch_dexhand_ros1.h"
#include "sensor_msgs/JointState.h"
#include "kuavo_msgs/dexhandTouchState.h"
#include <tuple>

namespace eef_controller {

kuavo_msgs::touchSensorStatus convertTouchSensorStatus(const dexhand::TouchSensorStatus_t& status) {
    kuavo_msgs::touchSensorStatus msg;
    msg.normal_force1 = status.normal_force1;
    msg.normal_force2 = status.normal_force2;
    msg.normal_force3 = status.normal_force3;
    msg.tangential_force1 = status.tangential_force1;
    msg.tangential_force2 = status.tangential_force2;
    msg.tangential_force3 = status.tangential_force3;
    msg.tangential_direction1 = status.tangential_direction1;
    msg.tangential_direction2 = status.tangential_direction2;
    msg.tangential_direction3 = status.tangential_direction3;
    msg.self_proximity1 = status.self_proximity1;
    msg.self_proximity2 = status.self_proximity2;
    msg.mutual_proximity = status.mutual_proximity;
    msg.status = status.status;
    return msg;
}

DexhandRosNode::DexhandRosNode() {
    finger_count_ = std::tuple_size<dexhand::UnsignedFingerArray>::value;
    hand_count_ = std::tuple_size<eef_controller::UnsignedDualHandsArray>::value;
    frequency_ = 200;
}

DexhandRosNode::~DexhandRosNode() {
    stop();
}

bool DexhandRosNode::init(ros::NodeHandle& nh, 
const std::string &action_sequences_path, 
double frequency, 
std::unique_ptr<DexhandController> &controller,
bool is_touch_dexhand,
bool is_can_protocol) {

    nh_ = nh;
    frequency_ = frequency;
    is_touch_dexhand_ = is_touch_dexhand;
    is_can_protocol_ = is_can_protocol;
    
    finger_count_ = std::tuple_size<dexhand::UnsignedFingerArray>::value;
    hand_count_ = std::tuple_size<eef_controller::UnsignedDualHandsArray>::value;

    /* ROS */
    auto right_hand_callback = [this](const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
        this->controlSingleHand(HandSide::RIGHT, msg);
    };

    auto left_hand_callback = [this](const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
        this->controlSingleHand(HandSide::LEFT, msg);
    };

    command_sub_ = nh_.subscribe("dexhand/command", 10, &DexhandRosNode::dualHandCommandCallback, this);
    r_hand_command_sub_ = nh_.subscribe<kuavo_msgs::dexhandCommand>("dexhand/right/command", 10, right_hand_callback);
    l_hand_command_sub_ = nh_.subscribe<kuavo_msgs::dexhandCommand>("dexhand/left/command", 10, left_hand_callback);
    status_pub_ = nh_.advertise<sensor_msgs::JointState>("dexhand/state", 10);
    change_force_level_srv_ = nh_.advertiseService("dexhand/change_force_level", &DexhandRosNode::changeForceLevelCallback, this);

    // 触觉手接口
    if (is_touch_dexhand_) {
        touch_status_pub_ = nh_.advertise<kuavo_msgs::dexhandTouchState>("dexhand/touch_state", 10);
        enable_left_hand_touch_sensor_srv_ = nh_.advertiseService("dexhand/left/enable_touch_sensor", 
            &DexhandRosNode::enableLeftHandTouchSensorCallback, this);
        enable_right_hand_touch_sensor_srv_ = nh_.advertiseService("dexhand/right/enable_touch_sensor", 
            &DexhandRosNode::enableRightHandTouchSensorCallback, this);  
    }
    
    // 兼容原来的接口
    hand_sub_ = nh_.subscribe("control_robot_hand_position", 10, &DexhandRosNode::controlHandCallback, this);
    gesture_execute_srv_ = nh_.advertiseService("gesture/execute", &DexhandRosNode::gestureExecuteCallback, this);
    gesture_list_srv_ = nh_.advertiseService("gesture/list", &DexhandRosNode::gestureListCallback, this);
    gesture_exec_state_srv_ = nh_.advertiseService("gesture/execute_state", &DexhandRosNode::gestureExecuteStateCallback, this);
    
    /* Initialize controller */
    if (controller) {
        controller_ = std::move(controller);
        controller = nullptr;
    }
    else {
        controller_ = eef_controller::DexhandController::Create(action_sequences_path, is_touch_dexhand_, is_can_protocol_);
        if (!controller_->init()) {
            std::cerr << "[DexhandRosNode] Failed to initialize." << std::endl;
            return false;
        }
    }


    /* create publish thread */
    running_ = true;
    publish_thread_ = std::thread(&DexhandRosNode::publish_loop, this);
    
    return true;
}

void DexhandRosNode::stop() {
    
    running_ = false;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }

    // Close controller - 添加空指针检查
    if (controller_) {
        controller_->close();
        controller_.reset();  // 显式重置指针
    }
}

void DexhandRosNode::publish_loop()
{
    const auto period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / frequency_));

    while (running_)  {
        auto start_time = std::chrono::steady_clock::now();

        // Get finger status
        auto finger_status = controller_->get_finger_status();

        // Publish finger status
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name = {"l_thumb", "l_thumb_aux", "l_index", "l_middle", "l_ring", "l_pinky",
                           "r_thumb", "r_thumb_aux", "r_index", "r_middle", "r_ring", "r_pinky"};

        // Add joint names and positions for both hands
        for (int hand = 0; hand < 2; hand++) {
            for (int finger = 0; finger < 6; finger++) {
                joint_state.position.push_back(finger_status[hand].positions[finger]);
                joint_state.velocity.push_back(finger_status[hand].speeds[finger]); 
                joint_state.effort.push_back(finger_status[hand].currents[finger]);
            }
        }

        // Publish touch sensor status
        if (is_touch_dexhand_) {
            auto finger_touch_status = controller_->get_touch_status();
            kuavo_msgs::dexhandTouchState touch_state;
            touch_state.header.stamp = ros::Time::now();

            for(int i = 0; i < 5; i++) {
                touch_state.left_hand[i] = convertTouchSensorStatus(finger_touch_status[0][i]);
                touch_state.right_hand[i] = convertTouchSensorStatus(finger_touch_status[1][i]);
            }
            touch_status_pub_.publish(touch_state);
        }

        status_pub_.publish(joint_state);

        // Calculate execution time and sleep for remaining time
        auto execution_time = std::chrono::steady_clock::now() - start_time;
        auto remaining_time = period - std::chrono::duration_cast<std::chrono::microseconds>(execution_time);

        if (remaining_time.count() > 0) {
            std::this_thread::sleep_for(remaining_time);
        }
    }
    
}

int DexhandRosNode::get_hand_joints_num() {
    return finger_count_ * hand_count_;
}

void DexhandRosNode::dualHandCommandCallback(const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    /* check control mode*/
    if(msg->control_mode != kuavo_msgs::dexhandCommand::POSITION_CONTROL 
        && msg->control_mode != kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
            std::cerr << "[DexhandRosNode] Unsupported control mode:" << static_cast<int>(msg->control_mode) << std::endl;
            return ;
    }

    constexpr int finger_count = std::tuple_size<dexhand::UnsignedFingerArray>::value;
    constexpr int hand_count = std::tuple_size<eef_controller::UnsignedDualHandsArray>::value;

    if (msg->data.size() != finger_count*hand_count) {
        std::cerr << "[DexhandRosNode] Invalid data size:" << msg->data.size() << "!= " << finger_count*hand_count << std::endl;
    }

    if(msg->control_mode == kuavo_msgs::dexhandCommand::POSITION_CONTROL) {
        eef_controller::UnsignedDualHandsArray positions;
        // Assign values to left and right hands
        for(int i = 0; i < finger_count; i++) {
            positions[0][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->data[i]));
            positions[1][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->data[i + finger_count]));
        }
        controller_->send_position(positions);
    } else if(msg->control_mode == kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
        eef_controller::DualHandsArray vels;
        // Assign values to left and right hands
        for(int i = 0; i < finger_count; i++) {
            vels[0][i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i]));
            vels[1][i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i + finger_count]));
        }
        controller_->send_speed(vels);
    }
}

void DexhandRosNode::controlSingleHand(HandSide side, const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    /* check control mode*/
    if(msg->control_mode != kuavo_msgs::dexhandCommand::POSITION_CONTROL 
        && msg->control_mode != kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
            std::cerr << "[DexhandRosNode] Unsupported control mode:" << static_cast<int>(msg->control_mode) << std::endl;
            return ;
    }

    constexpr int finger_count = std::tuple_size<dexhand::UnsignedFingerArray>::value; // 获取大小
    if (msg->data.size() != finger_count) {
        std::cerr << "[DexhandRosNode] Invalid data size:" << msg->data.size() << "!= " << finger_count << std::endl;
    }

    if(msg->control_mode == kuavo_msgs::dexhandCommand::POSITION_CONTROL) {
        dexhand::UnsignedFingerArray positions;
        for(int i = 0; i < finger_count; i++) {
            positions[i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->data[i]));
        }
        
        if (side == HandSide::LEFT) {
            controller_->send_left_position(positions);
        }
        else if (side == HandSide::RIGHT) {
            controller_->send_right_position(positions);
        }
    } else if(msg->control_mode == kuavo_msgs::dexhandCommand::VELOCITY_CONTROL) {
        dexhand::FingerArray vels;
        for(int i = 0; i < finger_count; i++) {
            vels[i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i]));
        }
        if (side == HandSide::LEFT) {
            controller_->send_left_speed(vels);
        }
        else if (side == HandSide::RIGHT) {
            controller_->send_right_speed(vels);
        }
    }
}

bool DexhandRosNode::changeForceLevelCallback(kuavo_msgs::handForceLevelRequest &req,
                        kuavo_msgs::handForceLevelResponse &res)
                        {
    auto level = req.force_level;
    if (level < 0 || level > 3) {
        res.success = false;
        res.message = "Invalid force level";
        return true;
    }

    if(req.hand_side == kuavo_msgs::handForceLevelRequest::LEFT_HAND) {
        res.success = controller_->set_left_hand_force_level(static_cast<dexhand::GripForce>(level));
    }
    else if (req.hand_side == kuavo_msgs::handForceLevelRequest::RIGHT_HAND) {
        res.success = controller_->set_right_hand_force_level(static_cast<dexhand::GripForce>(level));
    }
    else if (req.hand_side == kuavo_msgs::handForceLevelRequest::BOTH_HANDS) {
        res.success = controller_->set_hand_force_level(static_cast<dexhand::GripForce>(level));
    }

    if(!res.success) {
        res.message = "Failed to change force level";
    }
    else {
        res.message = "Success";
    }
    return true;
}

bool DexhandRosNode::enableLeftHandTouchSensorCallback(kuavo_msgs::enableHandTouchSensorRequest &req,
                        kuavo_msgs::enableHandTouchSensorResponse &res) {
    res.success = true;
    res.message = "Success";

    if(!controller_->enable_left_hand_touch_sensor(req.mask)) {
        res.success = false;
        res.message = "Failed to enable left hand touch sensor";
    }

    return true;
}

bool DexhandRosNode::enableRightHandTouchSensorCallback(kuavo_msgs::enableHandTouchSensorRequest &req,
                          kuavo_msgs::enableHandTouchSensorResponse &res) {
    res.success = true;
    res.message = "Success";

    if(!controller_->enable_right_hand_touch_sensor(req.mask)) {
        res.success = false;
        res.message = "Failed to enable right hand touch sensor";
    }

    return true;
}

void DexhandRosNode::controlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    constexpr int finger_count = std::tuple_size<dexhand::UnsignedFingerArray>::value;
    constexpr int hand_count = std::tuple_size<eef_controller::UnsignedDualHandsArray>::value;

    if (msg->left_hand_position.size() != finger_count || msg->right_hand_position.size() != finger_count) 
    {
        ROS_WARN("Received desired positions vector of incorrect size");
        return;
    }

    eef_controller::UnsignedDualHandsArray positions;
    // Assign values to left and right hands
    for(int i = 0; i < finger_count; i++) {
        positions[0][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->left_hand_position[i]));
        positions[1][i] = std::min<uint16_t>(100, std::max<uint16_t>(0, msg->right_hand_position[i]));
    }
    controller_->send_position(positions);
}

/* gesture execute service. */
bool DexhandRosNode::gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
                        kuavo_msgs::gestureExecuteResponse &res)
{
    if (!controller_) {
        res.success = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    std::string err_msg;
    GestureExecuteInfoVec gesture_tasks;
    for(const auto & gs : req.gestures) {
        gesture_tasks.push_back(GestureExecuteInfo{static_cast<HandSide>(gs.hand_side), gs.gesture_name});
    }
    
    if(gesture_tasks.size() == 0) {
        res.success = false;
        res.message = "No gesture to execute.";
        return false;
    }

    /* execute gestures */
    bool exec_ret = controller_->execute_gestures(gesture_tasks, err_msg);
    if(exec_ret) {
        res.message = "success.";
        res.success = true;
    }    
    else {
        res.message = err_msg;
        res.success = false;
    }
    return true;
}

/* gesture list service. */
bool DexhandRosNode::gestureListCallback(kuavo_msgs::gestureListRequest &req,
                        kuavo_msgs::gestureListResponse &res)
{
    if (!controller_) {
        res.gesture_infos.clear();
        res.gesture_count  = 0;
        res.success = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    /* convert gesture info to kuavo_msgs::gestureInfo */
    for(const auto &gesture_info : controller_->list_gestures()) {
        kuavo_msgs::gestureInfo msg;
        msg.gesture_name = gesture_info->gesture_name;
        msg.alias = gesture_info->alias;
        msg.description = gesture_info->description;
        res.gesture_infos.push_back(msg);
    }

    res.success = true;
    res.message = "success";
    res.gesture_count = res.gesture_infos.size();
    return true;
}

/* gesture execute state service. */
bool DexhandRosNode::gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
                        kuavo_msgs::gestureExecuteStateResponse &res)
{
    if (!controller_) {
        res.is_executing = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    res.is_executing = controller_->is_gesture_executing();        
    res.message = "success";
    
    return true;
}

} // namespace eef_controller
