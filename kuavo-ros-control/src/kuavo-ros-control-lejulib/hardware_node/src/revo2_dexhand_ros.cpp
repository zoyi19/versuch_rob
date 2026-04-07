#include "revo2_dexhand_ros.h"
#include "sensor_msgs/JointState.h"
#include <tuple>

namespace eef_controller {


Revo2DexhandRosNode::Revo2DexhandRosNode() {
    finger_count_ = std::tuple_size<dexhand::UnsignedFingerArray>::value;
    hand_count_ = std::tuple_size<eef_controller::UnsignedDualHandsArray>::value;
    frequency_ = 200;
}

Revo2DexhandRosNode::~Revo2DexhandRosNode() {
    stop();
}

bool Revo2DexhandRosNode::init(ros::NodeHandle& nh,
const std::string &action_sequences_path,
double frequency,
std::unique_ptr<Revo2HandController> &controller,
bool is_can_protocol) {

    nh_ = nh;
    frequency_ = frequency;
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

    command_sub_ = nh_.subscribe("dexhand/command", 10, &Revo2DexhandRosNode::dualHandCommandCallback, this);
    r_hand_command_sub_ = nh_.subscribe<kuavo_msgs::dexhandCommand>("dexhand/right/command", 10, right_hand_callback);
    l_hand_command_sub_ = nh_.subscribe<kuavo_msgs::dexhandCommand>("dexhand/left/command", 10, left_hand_callback);
    status_pub_ = nh_.advertise<sensor_msgs::JointState>("dexhand/state", 10);
    change_force_level_srv_ = nh_.advertiseService("dexhand/change_force_level", &Revo2DexhandRosNode::changeForceLevelCallback, this);


    // 兼容原来的接口
    hand_sub_ = nh_.subscribe("control_robot_hand_position", 10, &Revo2DexhandRosNode::controlHandCallback, this);
    gesture_execute_srv_ = nh_.advertiseService("gesture/execute", &Revo2DexhandRosNode::gestureExecuteCallback, this);
    gesture_list_srv_ = nh_.advertiseService("gesture/list", &Revo2DexhandRosNode::gestureListCallback, this);
    gesture_exec_state_srv_ = nh_.advertiseService("gesture/execute_state", &Revo2DexhandRosNode::gestureExecuteStateCallback, this);
    
    /* Initialize controller */
    if (controller) {
        controller_ = std::move(controller);
        controller = nullptr;
    }
    else {
        controller_ = eef_controller::Revo2HandController::Create(action_sequences_path, is_can_protocol_);
        if (!controller_->init()) {
            std::cerr << "[DexhandRosNode] Failed to initialize." << std::endl;
            return false;
        }
    }


    /* create publish thread */
    running_ = true;
    publish_thread_ = std::thread(&Revo2DexhandRosNode::publish_loop, this);
    
    return true;
}

void Revo2DexhandRosNode::stop() {
    
    running_ = false;
    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }

    // Close controller
    controller_->close();
}

void Revo2DexhandRosNode::publish_loop()
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


        status_pub_.publish(joint_state);

        // Calculate execution time and sleep for remaining time
        auto execution_time = std::chrono::steady_clock::now() - start_time;
        auto remaining_time = period - std::chrono::duration_cast<std::chrono::microseconds>(execution_time);

        if (remaining_time.count() > 0) {
            std::this_thread::sleep_for(remaining_time);
        }
    }
    
}

int Revo2DexhandRosNode::get_hand_joints_num() {
    return finger_count_ * hand_count_;
}

void Revo2DexhandRosNode::dualHandCommandCallback(const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
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
            // 二代手默认为千分比模式，相较于一代手，需要乘以10
            vels[0][i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i])) * 10;
            vels[1][i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i + finger_count])) * 10;
        }
        controller_->send_speed(vels);
    }
}

void Revo2DexhandRosNode::controlSingleHand(HandSide side, const kuavo_msgs::dexhandCommand::ConstPtr& msg) {
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
            // 二代手默认为千分比模式，相较于一代手，需要乘以10
            vels[i] = std::min<int16_t>(100, std::max<int16_t>(-100, msg->data[i])) * 10;
        }
        if (side == HandSide::LEFT) {
            controller_->send_left_speed(vels);
        }
        else if (side == HandSide::RIGHT) {
            controller_->send_right_speed(vels);
        }
    }
}

bool Revo2DexhandRosNode::changeForceLevelCallback(kuavo_msgs::handForceLevelRequest &req,
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


void Revo2DexhandRosNode::controlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg) {
    if(!running_) {
        return ;
    }

    constexpr int finger_count = std::tuple_size<dexhand::UnsignedFingerArray>::value;
    // constexpr int hand_count = std::tuple_size<eef_controller::UnsignedDualHandsArray>::value; // fix cpplint 

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
bool Revo2DexhandRosNode::gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
                        kuavo_msgs::gestureExecuteResponse &res)
{
    if (!controller_) {
        res.success = false;
        res.message = "hand controller not initialized.";
        return false;
    }

    std::string err_msg;
    GestureExecuteInfoVec gesture_tasks;
    // Reserve enough space in gesture_tasks to avoid multiple memory reallocations
    gesture_tasks.reserve(gesture_tasks.size() + req.gestures.size());

    // Use std::transform to convert and insert 
    std::transform(req.gestures.begin(), req.gestures.end(), std::back_inserter(gesture_tasks), [](const auto& gs) {
        return GestureExecuteInfo{static_cast<HandSide>(gs.hand_side), gs.gesture_name};
    });
    
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
bool Revo2DexhandRosNode::gestureListCallback(kuavo_msgs::gestureListRequest &req,
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
bool Revo2DexhandRosNode::gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
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
