#include "humanoid_interface/common/TopicLogger.h"
namespace ocs2
{
  namespace humanoid
  {
    TopicLogger::TopicLogger(ros::NodeHandle& nh) : nh_(nh)
    {
      predefinedTopics();
    }
    TopicLogger::TopicLogger()
    {
      nh_ = ros::NodeHandle();
      predefinedTopics();
    }
    void TopicLogger::predefinedTopics()
    {
      // 预定义一些常用的log记录topic列表, 加快初始发布速度
      // tips: 运行程序，通过以下命令查找筛选需要发布的topic(仅支持Float64MultiArray)：
      // `rostopic list | while read topic; do if [ "$(rostopic type $topic)" == "std_msgs/Float64MultiArray" ]; then echo "\"$topic\""; fi; done`

      const std::vector<std::string> predefinedTopics_ = {
          "/humanoid_controller/com/r",
          "/humanoid_controller/com/r_des",
          "/humanoid_controller/com/rd",
          "/humanoid_controller/com/rd_des",
          "/humanoid_controller/estimated_contact_force_",
          "/humanoid_controller/measuredRbdState_",
          "/humanoid_controller/measuredRbdState_/angle_zyx",
          "/humanoid_controller/measuredRbdState_/angular_vel_zyx",
          "/humanoid_controller/measuredRbdState_/joint_pos",
          "/humanoid_controller/measuredRbdState_/joint_vel",
          "/humanoid_controller/measuredRbdState_/linear_vel",
          "/humanoid_controller/measuredRbdState_/pos_xyz",
          "/humanoid_controller/optimizedInput_mrt_",
          "/humanoid_controller/optimizedState_mrt_",
          "/humanoid_controller/optimizedState_mrt_origin",
          "/humanoid_controller/optimizedInput_mrt_origin",
          "/humanoid_controller/optimizedState_mrt_/angular_vel_xyz",
          "/humanoid_controller/optimizedState_mrt_/angular_zyx",
          "/humanoid_controller/optimizedState_mrt_/joint_pos",
          "/humanoid_controller/optimizedState_mrt_/linear_vel_xyz",
          "/humanoid_controller/optimizedState_mrt_/pos_xyz",
          "/humanoid_controller/swing_leg/acc_desired_0",
          "/humanoid_controller/swing_leg/acc_desired_1",
          "/humanoid_controller/swing_leg/acc_desired_2",
          "/humanoid_controller/swing_leg/acc_desired_3",
          "/humanoid_controller/swing_leg/acc_desired_4",
          "/humanoid_controller/swing_leg/acc_desired_5",
          "/humanoid_controller/swing_leg/acc_desired_6",
          "/humanoid_controller/swing_leg/acc_desired_7",
          "/humanoid_controller/swing_leg/pos_desired_0",
          "/humanoid_controller/swing_leg/pos_desired_1",
          "/humanoid_controller/swing_leg/pos_desired_2",
          "/humanoid_controller/swing_leg/pos_desired_3",
          "/humanoid_controller/swing_leg/pos_desired_4",
          "/humanoid_controller/swing_leg/pos_desired_5",
          "/humanoid_controller/swing_leg/pos_desired_6",
          "/humanoid_controller/swing_leg/pos_desired_7",
          "/humanoid_controller/swing_leg/pos_measured_0",
          "/humanoid_controller/swing_leg/pos_measured_1",
          "/humanoid_controller/swing_leg/pos_measured_2",
          "/humanoid_controller/swing_leg/pos_measured_3",
          "/humanoid_controller/swing_leg/pos_measured_4",
          "/humanoid_controller/swing_leg/pos_measured_5",
          "/humanoid_controller/swing_leg/pos_measured_6",
          "/humanoid_controller/swing_leg/pos_measured_7",
          "/humanoid_controller/swing_leg/vel_desired_0",
          "/humanoid_controller/swing_leg/vel_desired_1",
          "/humanoid_controller/swing_leg/vel_desired_2",
          "/humanoid_controller/swing_leg/vel_desired_3",
          "/humanoid_controller/swing_leg/vel_desired_4",
          "/humanoid_controller/swing_leg/vel_desired_5",
          "/humanoid_controller/swing_leg/vel_desired_6",
          "/humanoid_controller/swing_leg/vel_desired_7",
          "/humanoid_controller/swing_leg/vel_measured_0",
          "/humanoid_controller/swing_leg/vel_measured_1",
          "/humanoid_controller/swing_leg/vel_measured_2",
          "/humanoid_controller/swing_leg/vel_measured_3",
          "/humanoid_controller/swing_leg/vel_measured_4",
          "/humanoid_controller/swing_leg/vel_measured_5",
          "/humanoid_controller/swing_leg/vel_measured_6",
          "/humanoid_controller/swing_leg/vel_measured_7",
          "/humanoid_controller/torque",
          "/humanoid_controller/wbc_planned_body_acc/angular",
          "/humanoid_controller/wbc_planned_body_acc/linear",
          "/humanoid_controller/wbc_planned_contact_force/left_foot",
          "/humanoid_controller/wbc_planned_contact_force/right_foot",
          "/humanoid_controller/wbc_planned_joint_acc",
          "/inekf_state/est/acc",
          "/inekf_state/est/angular_velocity",
          "/inekf_state/est/contact/state",
          "/inekf_state/est/contact/l_point",
          "/inekf_state/est/contact/r_point",
          "/inekf_state/est/orientation",
          "/inekf_state/est/pose",
          "/inekf_state/est/velocity",
          "/inekf_state/imu_quat",
          "/inekf_state/lf_in_base",
          "/inekf_state/rf_in_base",
          "/state_estimate/change_contact_time_sum_",
          "/state_estimate/dsigma_l_filter",
          "/state_estimate/dsigma_r_filter",
          "/humanoid_controller/rf_fly",
          "/humanoid/mpc/arm_control_mode",
          "/rl_controller/actions",
          "/rl_controller/InputData/actions",
          "/rl_controller/InputData/motion_command",
          "/rl_controller/InputData/motion_target_pos",
          "/rl_controller/InputData/motion_anchor_ori_b",
          "/rl_controller/InputData/projected_gravity",
          "/rl_controller/InputData/base_ang_vel",
          "/rl_controller/InputData/joint_pos",
          "/rl_controller/InputData/joint_vel",
          "/rl_controller/torque",


      };

      for (const auto &topic : predefinedTopics_)
      {
        ros::Publisher newPublisher = nh_.advertise<std_msgs::Float64MultiArray>(topic, 10);
        publishedTopics_[topic] = newPublisher;
      }
    }
    void TopicLogger::publishVector(const std::string &topic_name, const ocs2::vector_t &data, const std::string &label)
    {
      std::vector<double> stdVector(data.data(), data.data() + data.size());
      publishVector(topic_name, stdVector, label);
    }
    void TopicLogger::publishVector(const std::string &topic_name, const std::vector<double> &data, const std::string &label)
    {
      if (publishedTopics_.count(topic_name) == 0)
      {
        // 如果该topic尚未发布过，则创建新的发布者
        ros::Publisher newPublisher = nh_.advertise<std_msgs::Float64MultiArray>(topic_name, 10);
        publishedTopics_[topic_name] = newPublisher;
      }

      // 发布数据到对应的topic
      std_msgs::Float64MultiArray arrayMsg;
      arrayMsg.data = data;
      // arrayMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
      // arrayMsg.layout.dim[0].size = arrayMsg.data.size();
      // arrayMsg.layout.dim[0].stride = 1;
      // arrayMsg.layout.dim[0].label = label;

      publishedTopics_[topic_name].publish(arrayMsg);
    }
    void TopicLogger::publishValue(const std::string &topic_name, const double &data)
    {
      if (publishedValueTopics_.count(topic_name) == 0)
      {
        // 如果该topic尚未发布过，则创建新的发布者
        ros::Publisher newPublisher = nh_.advertise<std_msgs::Float64>(topic_name, 10);
        publishedValueTopics_[topic_name] = newPublisher;
      }

      // 发布数据到对应的topic
      std_msgs::Float64 arrayMsg;
      arrayMsg.data = data;
      publishedValueTopics_[topic_name].publish(arrayMsg);
    }
  }
}
