#ifndef HARDWARE_NODE_H
#define HARDWARE_NODE_H
#pragma once

#include <memory>
#include <queue>
#include <set>
#include <atomic>
#include <claw_types.h>
#include <gesture_types.h>
#include <lejuclaw_controller.h>
#include "hardware_plant.h"
#if defined(USE_DDS) || defined(USE_LEJU_DDS)
#include "hardware_dds_plant.h"
#endif
#include "humanoid_interface/common/TopicLogger.h"

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include "touch_dexhand_ros1.h"
#include "revo2_dexhand_ros.h"

#include "kuavo_msgs/sensorsData.h"
#include "kuavo_msgs/jointData.h"
#include "kuavo_msgs/jointCmd.h"
#include "kuavo_msgs/jointMoveTo.h"
#include "kuavo_msgs/jointMoveToRequest.h"
#include "kuavo_msgs/jointMoveToResponse.h"
#include "kuavo_msgs/setHwIntialState.h"
#include "kuavo_msgs/setMotorEncoderRoundService.h"
#include "kuavo_msgs/robotHandPosition.h"
#include "kuavo_msgs/robotHeadMotionData.h"
#include "kuavo_msgs/gestureExecute.h"
#include "kuavo_msgs/gestureList.h"
#include "kuavo_msgs/gestureExecuteState.h"
#include "kuavo_msgs/gestureInfo.h"
#include "kuavo_msgs/controlLejuClaw.h"
#include "kuavo_msgs/lejuClawState.h"
#include "kuavo_msgs/lejuClawCommand.h"
#include "kuavo_msgs/changeMotorParam.h"
#include "kuavo_msgs/getMotorParam.h"
#include "kuavo_msgs/adjustZeroPoint.h"
#include "kuavo_msgs/getMotorZeroPoints.h"
#include "kuavo_msgs/goToZeroPoint.h"
#include "kuavo_msgs/robotSwitchPose.h"

namespace leju_utils {
class ActuatorDynamicsCompensator;
}

namespace HighlyDynamic
{
  class HardwareNode
  {

  public:
    enum PROTECT_ACTIVE_MODE : int
    {
      ENUM_ACTIVE_DECE = 0,
      ENUM_ACTIVE_DISABLE_MPC = 1,
      ENUM_ACTIVE_POS_LOCK = 2,
      ENUM_ACTIVE_BOT_RELEASE = 3,

      ENUM_ACTIVE_INIT = 99
    };

    enum PROTEST_MODE : int
    {
      ENUM_PEAR_PROTECTION = 0,
      ENUM_LOCKED_ROTOR_PROTECTION = 1,
      ENUM_SPEED_PROTECTION = 2
    };

  private:
      std::unique_ptr<HighlyDynamic::HardwarePlant> hardware_plant_;

      ros::NodeHandle nh_;
      std::thread sensor_thread_;
      std::atomic<bool> sensor_thread_running_{false};
      ros::Publisher sensor_data_pub_;
      ros::Subscriber joint_sub_;
      ros::Subscriber hand_sub_;
      ros::Subscriber head_sub_;
      ros::Subscriber stop_sub_;
      ros::Publisher stop_pub_;
      ros::Publisher enable_wbc_pub_;
      ros::Publisher enable_mpc_pub_;
      ros::Subscriber re_start_sub_;
      ros::Subscriber bot_stand_up_sub_;
      ros::Publisher leju_claw_state_pub_;
      ros::Subscriber leju_claw_command_sub_;
      ros::Timer     leju_claw_state_timer_;

      ros::ServiceServer joint_move_to_srv_;
      ros::ServiceServer set_intial_state_srv_;
      ros::ServiceServer modify_motor_encoder_offset_srv_;
      ros::ServiceServer deintial_srv_;
      ros::ServiceServer leju_claw_ctrl_srv_;

      ros::ServiceServer change_motor_param_srv_;
      ros::ServiceServer get_motor_param_srv_;

      ros::ServiceServer adjust_zero_point_srv_;
      ros::ServiceServer get_hardware_ready_srv_;
      ros::ServiceServer get_motor_zero_points_srv_;
      ros::ServiceServer switch_robot_state_srv_;

      /**
       * for kuavo-humanoid-websocket-sdk: 获取启动状态服务 
       * initing: 初始化中
       * calibrate: 准备下蹲缩腿 --> cali 全身绷直状态
       * ready_stance: 缩腿准备站立
       * launched: 启动完成, 站立 OK
       */
      ros::ServiceServer real_launch_status_srv_;

      ocs2::humanoid::TopicLogger *logger_ptr_;

      int8_t bot_stand_up_complete_{0};    //0:初始化  -1：站立失败   1：站立成功
      HardwareParam hardware_param_;
      std::shared_ptr<eef_controller::DexhandRosNode> dexhand_ros_actuator_;
      std::shared_ptr<eef_controller::Revo2DexhandRosNode> revo2_dexhand_ros_actuator_;

    bool joint_protect_enable_{false};
    bool cmd_truncation_enable_{false};
    bool leg_disable_mode_{false};  // 腿部失能模式：false=单个关节失能, true=腿部全失能
    bool cali_zero_point_mode{false};
    bool zero_point_status_swith{false};
    std::mutex enable_mpc_mtx_;
    std::mutex protect_mode_mtx_;
    Eigen::VectorXd his_cmd_;
    Eigen::VectorXd cmd_out_prev_;

    std::vector<double> curVel_;
    std::vector<double> curPos_;
    std::vector<int> armJointIndices_;
    std::unique_ptr<leju_utils::ActuatorDynamicsCompensator> actuatorDynamicsCompensatorPtr_;

    double c2t_coeff_[28];
    std::mutex disable_motor_mtx_;
    std::set<int> disableMotor_;
    std::mutex motor_status_map_mtx_;
    std::map<int, bool> motorStatusMap_; 
    std::vector<ros::Time> pre_vel_over_time_;
    std::vector<ros::Time> pre_cur_over_time_;
    std::vector<ros::Time> pre_peak_over_time_;
    ros::Time set_mpc_disable_time_;
    PROTECT_ACTIVE_MODE cur_protect_mode_{ENUM_ACTIVE_INIT};
    char stand_up_cmd_ = '\0';
    double dt_;
    int num_joint;
    double jointMoveSpeed_{60.0};
    

    // Dexterous hand state publish frequency in Hz
    static constexpr double kDexhandStatePublishFrequency = 200.0;

  public:
      HardwareNode(ros::NodeHandle &nh, double dt);
      ~HardwareNode();
      void init();
      void jointCmdCallback(const kuavo_msgs::jointCmd::ConstPtr& msg);
      void handCmdCallback(const kuavo_msgs::robotHandPosition::ConstPtr &msg);
      bool jointMoveToServiceCallback(kuavo_msgs::jointMoveToRequest &req, kuavo_msgs::jointMoveToResponse &res);
      bool setHwIntialStateCallback(kuavo_msgs::setHwIntialStateRequest &req, kuavo_msgs::setHwIntialStateResponse &res);
      bool setMotorEncoderOffsetCallback(kuavo_msgs::setMotorEncoderRoundServiceRequest &req, kuavo_msgs::setMotorEncoderRoundServiceResponse &res);
      bool deintialCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      void stopCallback(const std_msgs::Bool::ConstPtr &msg);
      bool controlLejuClawCallback(kuavo_msgs::controlLejuClawRequest &req, kuavo_msgs::controlLejuClawResponse &res);
      void lejuClawCommandCallback(const kuavo_msgs::lejuClawCommand::ConstPtr &msg);
      void pubLejuClawStateTimerCallback(const ros::TimerEvent& event);
      bool changeMotorParamCallback(kuavo_msgs::changeMotorParamRequest &req, kuavo_msgs::changeMotorParamResponse &res);
      bool getMotorParamCallback(kuavo_msgs::getMotorParamRequest &req, kuavo_msgs::getMotorParamResponse &res);
      void reStartCallback(const std_msgs::Bool::ConstPtr &msg);
      void getBotStandUpCompleteCallback(const std_msgs::Int8::ConstPtr &msg);
      void StandUpLoop();
      void real_init_wait();
      void readyToRecvCmd();
      bool realLaunchStatusCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

      // 新增的三个服务回调函数
      bool adjustZeroPointCallback(kuavo_msgs::adjustZeroPoint::Request &req, kuavo_msgs::adjustZeroPoint::Response &res);
      bool getHardwareReadyCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
      bool getMotorZeroPointsCallback(kuavo_msgs::getMotorZeroPoints::Request &req, kuavo_msgs::getMotorZeroPoints::Response &res);
      bool goToMotorZeroPointsCallback(kuavo_msgs::goToZeroPoint::Request &req,kuavo_msgs::goToZeroPoint::Response &res);
      bool switchRobotStateCallback(kuavo_msgs::robotSwitchPose::Request &req, kuavo_msgs::robotSwitchPose::Response &res);  // 新增的服务回调函数


      void sensorThreadFunc(ros::Publisher &sensor_data_pub);
      // void signalHandler(int signal);

      bool realIntialStartCallback(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
      {
        std::cout << "realIntialStartCallback" << std::endl;
        response.success = true;
        hardware_plant_->initial_input_cmd_ = 'o';
        stand_up_cmd_ = hardware_plant_->initial_input_cmd_; 
        return true;
      }

      inline size_t get_num_actuated() const {return hardware_plant_->get_num_actuated();}
      inline HardwareSettings get_motor_info() const {return hardware_plant_->get_motor_info();}
      inline void jointMoveTo(std::vector<double> goal_pos, double speed, double dt = 1e-3, double current_limit=-1){
        hardware_plant_->jointMoveTo(goal_pos, speed, dt, current_limit);
        
      }

      void cmdTruncation(Eigen::VectorXd &cmd_out);

      inline int getDisableMotorId()
      {
        std::lock_guard<std::mutex> lk(disable_motor_mtx_);
        if (disableMotor_.empty())
        {
          return -1;
        }
        auto first_id = *disableMotor_.begin();
        disableMotor_.erase(first_id);
        return first_id;
      }

      void JointProtectionAction(Eigen::VectorXd &cmd_out, std::vector<int> &control_mode);
      
      int JointProtectionTrigger(PROTEST_MODE mode, std::vector<double> &values);  

      void zeroTorqueCalibrateLegs();
      void applyArmActuatorDynamicsCompensation(const kuavo_msgs::jointCmd::ConstPtr& msg, Eigen::VectorXd& cmd);

      void publishDisableWbcFlag()
      {
        std_msgs::Bool enable_wbc_msg;
        enable_wbc_msg.data = false;
        enable_wbc_pub_.publish(enable_wbc_msg);
      }

      void publishDisableMpcFlag()
      {
          std_msgs::Bool enable_mpc_msg;
          enable_mpc_msg.data = false;
          enable_mpc_pub_.publish(enable_mpc_msg);
      }

      void publishEnableMpcFlag()
      {
          std_msgs::Bool enable_mpc_msg;
          enable_mpc_msg.data = true;
          enable_mpc_pub_.publish(enable_mpc_msg);
      }
  };
}
#endif
