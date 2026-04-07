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

  bool init(bool cali_arm, bool old_version=true, const std::string& robot_type="kuavo") {
    // Set joint names based on robot type
    if (robot_type == "roban") {
      joint_names_ = {
        "zarm_11_joint", "zarm_12_joint", "zarm_l3_joint", "zarm_l4_joint",  // Left arm joints
        "zarm_r1_joint","zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",   // Right arm joints
        "zhead_1_joint", "zhead_2_joint"  // Head joints
      };
    } else if (robot_type == "kuavo") {
      joint_names_ = {
        "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",  // Left arm joints
        "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",   // Right arm joints
        "zhead_1_joint", "zhead_2_joint"  // Head joints
      };
    } else {
      ROS_ERROR("Unknown robot type: %s. Defaulting to kuavo.", robot_type.c_str());
      joint_names_ = {
        "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",  // Left arm joints
        "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",   // Right arm joints
        "zhead_1_joint", "zhead_2_joint"  // Head joints
      };
    }

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
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10); 

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

    // 转发到 /joint_states 话题测试
    if(!real_) {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.stamp = ros::Time::now();
        
        // Add other joints that aren't controlled by this node but need to be in joint_states
        std::vector<std::string> other_joint_names = {
            "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
            "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
            "zarm_l1_joint", "zarm_r1_joint"
        };
        
        // Combine joint names and positions
        joint_state_msg.name = other_joint_names;
        joint_state_msg.name.insert(joint_state_msg.name.end(), msg->name.begin(), msg->name.end());
        
        // Set zero positions for other joints
        std::vector<double> other_positions(other_joint_names.size(), 0.0);
        joint_state_msg.position = other_positions;
        joint_state_msg.position.insert(joint_state_msg.position.end(), msg->position.begin(), msg->position.end());
        
        // Publish to joint_states topic
        joint_state_pub_.publish(joint_state_msg);
    }
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
  ros::Publisher joint_state_pub_;
  
  // Thread for state publishing
  std::thread state_thread_;
  bool running_ {false};
  bool real_ {false};

  // Joint names constants
  std::vector<std::string> joint_names_;

  RuiwoActuatorBase* ruiwo_actuator_=nullptr;
};

RuiwoControllerTest* g_controller = nullptr;

// Main function
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ruiwo_motor_controller_test");

  bool cali_arm = false;
  bool use_old_version = true;  // Default value
  std::string robot_type = "kuavo";  // Default value

  if(ros::param::has("cali_arm")) {
    ros::param::get("cali_arm", cali_arm);
  }

  if(ros::param::has("use_old_version")) {
    ros::param::get("use_old_version", use_old_version);
  }

  if(ros::param::has("robot_type")) {
    ros::param::get("robot_type", robot_type);
  }

  // Create an instance of the RuiwoControllerTest class
  g_controller = new RuiwoControllerTest();

  if (!g_controller->init(cali_arm, use_old_version, robot_type)) {
    ROS_ERROR("Failed to initialize RuiwoControllerTest");
    return 0;
  }

  // Spin to receive callbacks
  ros::spin();
  g_controller->close();

  return 0;
}
