#ifndef REVO2_DEXHAND_ROS_H
#define REVO2_DEXHAND_ROS_H
#include <ros/ros.h>
#include <thread>
#include <string>

#include "dexhand_def.h"
#include "revo2_hand_controller.h"
#include "kuavo_msgs/dexhandCommand.h"
#include "kuavo_msgs/handForceLevel.h"
#include "kuavo_msgs/robotHandPosition.h"
#include "kuavo_msgs/gestureExecute.h"
#include "kuavo_msgs/gestureList.h"
#include "kuavo_msgs/gestureExecuteState.h"

namespace eef_controller {

class Revo2DexhandRosNode {
public:
    Revo2DexhandRosNode();
    ~Revo2DexhandRosNode();

    /**
     * @brief Initialize the TouchDexHand node
     * @return true if initialization is successful, false otherwise
     */
    bool init(ros::NodeHandle& nh, 
        const std::string &action_sequences_path, 
        double frequency, 
        std::unique_ptr<Revo2HandController> &controller,
        bool is_can_protocol);

    /**
     * @brief Stop the TouchDexHand node and cleanup resources
     */
    void stop();

    /**
     * @brief Get the hand joints num
     * @return the hand joints num
     */
    int get_hand_joints_num();

private:
    void dualHandCommandCallback(const kuavo_msgs::dexhandCommand::ConstPtr& msg);
    void controlSingleHand(HandSide side, const kuavo_msgs::dexhandCommand::ConstPtr& msg);
    // 兼容原来的 control_robot_hand_position 接口
    void controlHandCallback(const kuavo_msgs::robotHandPosition::ConstPtr& msg);
    bool changeForceLevelCallback(kuavo_msgs::handForceLevelRequest &req,
                          kuavo_msgs::handForceLevelResponse &res);
    /* gesture execute service. */
    bool gestureExecuteCallback(kuavo_msgs::gestureExecuteRequest &req,
                          kuavo_msgs::gestureExecuteResponse &res);
    /* gesture list service. */
    bool gestureListCallback(kuavo_msgs::gestureListRequest &req,
                          kuavo_msgs::gestureListResponse &res);
    /* gesture execute state service. */
    bool gestureExecuteStateCallback(kuavo_msgs::gestureExecuteStateRequest &req,
                          kuavo_msgs::gestureExecuteStateResponse &res);
    void publish_loop();
    
    /* ROS */
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    ros::Subscriber r_hand_command_sub_;
    ros::Subscriber l_hand_command_sub_;
    ros::Publisher status_pub_;
    ros::ServiceServer change_force_level_srv_;
    // 兼容原来的 control_robot_hand_position 接口
    ros::Subscriber hand_sub_;  
    // 兼容原来的手势接口
    ros::ServiceServer gesture_execute_srv_;
    ros::ServiceServer gesture_list_srv_;
    ros::ServiceServer gesture_exec_state_srv_;

    std::unique_ptr<eef_controller::Revo2HandController> controller_;
    std::thread publish_thread_;
    bool running_{false};
    bool is_can_protocol_{false};
    int finger_count_;
    int hand_count_;
    double frequency_;
};

} // namespace eef_controller
#endif
