#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>

#include "ruiwo_actuator_base.h"
#include "ruiwo_actuator.h"
#include "motorevo/motorevo_actuator.h"
#include "canbus_sdk/config_parser.h"
#include "revo2_dexhand_ros.h"
#include "kuavo_common/common/common.h"
#include "kuavo_common/kuavo_common.h"
#include "kuavo_assets/include/package_path.h"

std::string GetHWNodePackagePath(const std::string &path)
{
    std::string package_path = ros::package::getPath("hardware_node");
    std::string abs_path = path;
    if (path[0] != '/')
    {
        abs_path = (package_path + "/../hardware_plant/" + path);
    }
    return abs_path;
}

class RuiwoControllerTest
{
public:
  RuiwoControllerTest() = default;

  ~RuiwoControllerTest()
  {
      close();
  }

  void close() {
      // Shutdown ROS if it's still running
      if (ros::isInitialized() && !ros::isShuttingDown()) {
        ros::shutdown();
      }

      // Stop the state publishing thread
      running_ = false;
      if (state_thread_.joinable()) {
        state_thread_.join();
      }

      if(ruiwo_actuator_) {
        ruiwo_actuator_->close();
        ruiwo_actuator_ = nullptr;
      }

      ROS_INFO("RuiwoControllerTest shutting down");
  }

  bool init(bool cali_arm, bool old_version=true) {
    // Set joint names for roban
    joint_names_ = {
      "zarm_11_joint", "zarm_12_joint", "zarm_l3_joint", "zarm_l4_joint",  // Left arm joints
      "zarm_r1_joint","zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",   // Right arm joints
      "zhead_1_joint", "zhead_2_joint"  // Head joints
    };

    // Initialize the RuiWoActuator
    if(old_version) {
      ruiwo_actuator_ = new RuiWoActuator(GetHWNodePackagePath("lib/ruiwo_controller"), cali_arm);
    }
    else {
      auto config_file = canbus_sdk::ConfigParser::getDefaultConfigFilePath();
      ruiwo_actuator_ = new motorevo::MotorevoActuator(config_file, cali_arm);
    }

    if(ruiwo_actuator_->initialize() != 0) {
      std::cout << "RuiWoActuator initialization failed" << std::endl;
      return false;
    }

    // Initialize ROS node handle
    ros::NodeHandle nh;

    // Subscribe to the /ruiwo_motor/command topic
    motor_cmd_sub_ = nh.subscribe("/ruiwo_motor/command", 10,
                                    &RuiwoControllerTest::jointCmdCallback, this);
    motor_state_pub_ = nh.advertise<sensor_msgs::JointState>("/ruiwo_motor/state", 10);

    // Start the state publishing thread
    running_ = true;
    state_thread_ = std::thread(&RuiwoControllerTest::statePublishLoop, this);

    ROS_INFO("RuiwoControllerTest initialized, subscribing to /ruiwo_motor/command");
    return true;
  }
private:
  // Callback function for the joint state subscriber
  // 单位是弧度
  void jointCmdCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // Check if dimensions match
    if (msg->name.size() != joint_names_.size()) {
      ROS_WARN_STREAM("Dimension mismatch: received " << msg->name.size()
                      << " joints, but joint_names_ has " << joint_names_.size() << " joints");
      return;
    }

    // Check if position array length matches joint names length
    if (msg->position.size() != msg->name.size()) {
      ROS_WARN_STREAM("Position array length (" << msg->position.size()
                      << ") does not match joint names length (" << msg->name.size() << ")");
      return;
    }

    for (int i = 0; i < msg->position.size(); i++) {
      if (msg->position[i] > M_PI || msg->position[i] < -M_PI) {
        ROS_WARN_STREAM("Invalid joint position: " << msg->position[i] << " for joint " << msg->name[i]);
        return;
      }
    }
    /**************************************************/
    // 调用 ruiwo_actuator
    /**************************************************/
    std::vector<uint8_t> indexs;
    std::vector<double> positions, velocity, torque;
    for(int i = 0; i < msg->name.size(); i++) {
      indexs.push_back(static_cast<uint8_t>(i));
      positions.push_back(msg->position[i] * 180.0 / M_PI);  // 单位角度
      velocity.push_back(0.0);
      torque.push_back(0.5);
    }
    ruiwo_actuator_->set_positions(indexs, positions, torque, velocity);
  }

  // Thread function to publish state at 200Hz
  void statePublishLoop()
  {
    ros::Rate rate(200); // 200Hz

    while (running_ && ros::ok()) {
        sensor_msgs::JointState state_msg;
        state_msg.header.stamp = ros::Time::now();
        state_msg.name = joint_names_;
        /******************************************************************/
        state_msg.position = std::vector<double>(joint_names_.size(), 0.0);
        auto positions = ruiwo_actuator_->get_positions();
        if(positions.size() != joint_names_.size()) {
          std::cout << "positions.size() != " << positions.size() << std::endl;
          continue;
        }
        for (size_t i = 0; i < positions.size(); ++i) {
            state_msg.position[i] = positions[i]; // 单位弧度
        }
        /******************************************************************/
        motor_state_pub_.publish(state_msg);
        rate.sleep();
    }
  }

  // ROS subscriber and publisher
  ros::Subscriber motor_cmd_sub_;
  ros::Publisher motor_state_pub_;
  
  // Thread for state publishing
  std::thread state_thread_;
  bool running_ {false};

  // Joint names constants
  std::vector<std::string> joint_names_;

  RuiwoActuatorBase* ruiwo_actuator_=nullptr;
};

RuiwoControllerTest* g_arm_controller = nullptr;
std::shared_ptr<eef_controller::Revo2DexhandRosNode> g_hand_node = nullptr;

// Main function
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "roban_arm_with_revo2_test");

  bool cali_arm = false;

  if(ros::param::has("cali_arm")) {
    ros::param::get("cali_arm", cali_arm);
  }

  // 判断CAN协议类型 - 参考dexhand_controller_test.cc中的实现
  const std::string kuavo_assets_path = ocs2::kuavo_assets::getPath();
  RobotVersion rb_version_ = RobotVersion(4, 0);
  auto kuavo_common_ptr = HighlyDynamic::KuavoCommon::getInstancePtr(rb_version_, kuavo_assets_path);
  auto kuavo_settings = kuavo_common_ptr->getKuavoSettings();
  HighlyDynamic::CanbusWiringType canbus_mode = kuavo_settings.hardware_settings.getCanbusWiringType(rb_version_);
  bool is_can_protocol = canbus_mode == HighlyDynamic::CanbusWiringType::DUAL_BUS;

  // Create and initialize arm controller
  g_arm_controller = new RuiwoControllerTest();
  if (!g_arm_controller->init(cali_arm, !is_can_protocol)) {
    ROS_ERROR("Failed to initialize RuiwoControllerTest");
    return 0;
  }

  // Initialize Revo2 hand ROS node in main function
  ros::NodeHandle nh;
  std::string config_path = kuavo_assets_path + "/config/gesture/preset_gestures.json";

  g_hand_node = std::make_shared<eef_controller::Revo2DexhandRosNode>();
  std::unique_ptr<eef_controller::Revo2HandController> controller = nullptr;


  if (!g_hand_node->init(nh, config_path, 1000, controller, is_can_protocol)) {
      std::cout << "[ERROR] Failed to init dexhand node.\n";
      return 0;
  }

  std::cout << "\n\n----- Roban2 Arm with Revo2 Hand ROS Node ----- \n";
  std::cout << "Topics:\n";
  std::cout << " - /control_robot_hand_position \n";
  std::cout << " - /dexhand/command \n";
  std::cout << " - /dexhand/right/command \n";
  std::cout << " - /dexhand/left/command \n";
  std::cout << " - /dexhand/state \n";
  std::cout << " - /ruiwo_motor/command \n";
  std::cout << " - /ruiwo_motor/state \n";
  std::cout << "Services:\n";
  std::cout << "/gesture/execute \n";
  std::cout << "/gesture/execute_state \n";
  std::cout << "/gesture/list \n";
  std::cout << "/dexhand/change_force_level \n";

  // Spin to receive callbacks
  ros::spin();

  // Cleanup
  g_arm_controller->close();
  if(g_hand_node) {
    g_hand_node->stop();
    g_hand_node.reset();
  }

  return 0;
}