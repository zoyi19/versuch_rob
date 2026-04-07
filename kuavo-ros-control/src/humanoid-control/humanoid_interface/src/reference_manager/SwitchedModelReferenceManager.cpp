/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/
#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include "humanoid_interface/reference_manager/SwitchedModelReferenceManager.h"
#include "humanoid_interface/common/Types.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include "kuavo_msgs/footPoseTargetTrajectories.h"
#include "kuavo_msgs/footPose6DTargetTrajectories.h"
#include "kuavo_msgs/fullBodyTargetTrajectories.h"
#include "kuavo_msgs/gaitTimeName.h"
#include "kuavo_msgs/qv.h"
#include "kuavo_msgs/robotWaistControl.h"

namespace ocs2 {
namespace humanoid {
  Eigen::VectorXd vectorToEigen(const std::vector<double>& vec) {
    return Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
}

  template <typename T>
  T square(T a)
  {
    return a * a;
  }
  template <typename SCALAR_T>
  Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
  {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) =
        std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) =
        std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
  }
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const CentroidalModelInfo &info,
                                                             const ModelSettings& modelSettings,
                                                             RobotVersion rbVersion)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      pinocchioInterface_(pinocchioInterface), info_(info),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      armTargetTrajectories_(std::move(TargetTrajectories())),
      armFullDofTargetTrajectories_(std::move(TargetTrajectories())),
      poseTargetTrajectories_(std::move(TargetTrajectories())),
      armWrenchBuffer_(std::move(vector_t::Zero(12))),
      rbdConversions_(pinocchioInterface, info),
      contactNames3DoF_(modelSettings.contactNames3DoF),
      rbVersion_(rbVersion)
{
  gait_Q_ = matrix_t::Identity(info_.stateDim, info_.stateDim);
  gait_R_ = matrix_t::Identity(info_.inputDim, info_.inputDim);
  // Q_ = matrix_t::Identity(24, 24);

  ros::NodeHandle nh;
  ros::param::get("/mpc/mpcWaistDof", waistNums_);

  waistNums_ = modelSettings.mpcWaistDof;
  // Inverse Kinematic
  inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(pinocchioInterface_),
                              std::make_shared<CentroidalModelInfo>(info_));

  // Target Velocity
  cmdVel_ = vector_t::Zero(6);
  cmdPose_ = vector_t::Zero(6);
  tempCmdPose_ = vector_t::Zero(6);
  cmdPoseWorld_ = vector_t::Zero(6);

  // velocity_scale: x, z, pitch, yaw用0.35，y用0.15
  torso_velocity_scale_ = (vector6_t() << 0.35, 0.15, 0.35, 0.35, 0.35, 0.35).finished();

  armJointNums_ = info_.stateDim - 12 - feetJointNums_ - waistNums_;   //减去waist的自由度

 
  if (nodeHandle_.hasParam("/armRealDof"))
  {
    nodeHandle_.getParam("/armRealDof", armRealDof_);
    std::cout << "[SwitchedModelReferenceManager]: Get armRealDof: " << armRealDof_ << std::endl;
  }
  armRealDof_ = modelSettings.modelDof - modelSettings.mpcWaistDof - modelSettings.mpcLegsDof; // 减去waist和腿的自由度
  swingTrajectoryPlannerConfig_ = swingTrajectoryPtr_->getConfig();

  if (nodeHandle_.hasParam("/dynamicQrFile"))
  {
    nodeHandle_.getParam("/dynamicQrFile", dynamic_qr_file_);
    if(swingTrajectoryPlannerConfig_.enable_dynamic_q == true)
      loadBaseTrackingQ(dynamic_qr_file_);
    if(swingTrajectoryPlannerConfig_.enable_dynamic_r == true)
      loadBaseTrackingR(dynamic_qr_file_);
    // 加载所有Q_dynamic_<gait_name>和R_dynamic_<gait_name>到map中
    loadDynamicQRMap(dynamic_qr_file_);
    dynamic_qr_flag_ = true;
    std::cout << "dynamic_qr_file: " << dynamic_qr_file_ << std::endl;
  }

  swingTrajectoryPtr_->setArmDof(armJointNums_);
  swingTrajectoryPtr_->setInverseKinematics(std::make_shared<InverseKinematics>(inverseKinematics_));
  swingTrajectoryPlannerConfig_ = swingTrajectoryPtr_->getConfig();

  if (nodeHandle_.hasParam("/arm_move_spd"))
  {
    nodeHandle_.getParam("/arm_move_spd", arm_move_spd_);
  }
  if (nodeHandle_.hasParam("/waist_move_spd"))
  {
    nodeHandle_.getParam("/waist_move_spd", waist_move_spd_);
  }

  if (nodeHandle_.hasParam("/only_half_up_body"))
  {
    nodeHandle_.getParam("/only_half_up_body", only_half_up_body_);
  }

  headArrayPublisher_ = nodeHandle_.advertise<std_msgs::Float64MultiArray>("/robot_head_motion_array", 10);
  
  
  // Initialize VR waist control from ROS parameter
  if (nodeHandle_.hasParam("/vr_waist_control_enabled"))
  {
    nodeHandle_.getParam("/vr_waist_control_enabled", vrWaistControlEnabled_);
    std::cout << "[SwitchedModelReferenceManager]: VR waist control enabled: " << vrWaistControlEnabled_ << std::endl;
  }
}

void SwitchedModelReferenceManager::setupSubscriptions(std::string nodeHandleName)
{
  if (nodeHandle_.hasParam("/armRealDof"))
  {
    nodeHandle_.getParam("/armRealDof", armRealDof_);
    std::cout << "[SwitchedModelReferenceManager]: Get armRealDof: " << armRealDof_ << std::endl;
  }
  TargetTrajectories originalTargetTrajectories = {{0.0}, {vector_t::Zero(armRealDof_)}, {vector_t::Zero(info_.inputDim)}};
  armFullDofTargetTrajectories_.setBuffer(std::move(originalTargetTrajectories));

  auto targetVelocityCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
  {
    // if (velCmdUpdated_ == false)
    // {
    cmdvel_mtx_.lock();
    cmdVel_[0] = msg->linear.x;
    cmdVel_[1] = msg->linear.y;
    cmdHeight_ = msg->linear.z;
    cmdVel_[3] = msg->angular.z;
    velCmdUpdated_ = true;
    cmdvel_mtx_.unlock();
    checkSingleStepControlAndStop();
    // }
  };
  targetVelocitySubscriber_ =
      nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, targetVelocityCallback);


  auto targetPoseCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
  {
    // if (velCmdUpdated_ == false)
    // {
    // std::cout << "[CMD POSE]: GET MSG" << std::endl;

    cmdPose_mtx_.lock();
    // std::cout << "[CMD POSE]: GET MSG and LOCK" << std::endl;
    cmdPose_.setZero();
    cmdPose_[0] = msg->linear.x;
    cmdPose_[1] = msg->linear.y;
    cmdHeight_ = msg->linear.z;
    cmdPose_[3] = msg->angular.z;
    cmdPose_[5] = msg->angular.x;
    cmdPitch_ = msg->angular.y;
    PoseCmdUpdated_ = true;
    PoseWorldCmdUpdated_ = false;
    isCmdPoseCached = false;

    cmdPose_mtx_.unlock();
    checkSingleStepControlAndStop();
    // }
  };
  targetPoseSubscriber_ =
      nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose", 1, targetPoseCallback);

  auto targetPoseWorldCallback = [this](const geometry_msgs::Twist::ConstPtr &msg)
  {
    cmdPoseWorld_mtx_.lock();
    cmdPoseWorld_.setZero();
    cmdPoseWorld_[0] = msg->linear.x;
    cmdPoseWorld_[1] = msg->linear.y;
    cmdHeight_ = msg->linear.z;
    cmdPoseWorld_[3] = msg->angular.z;
    cmdPitch_ = msg->angular.y;
    PoseWorldCmdUpdated_ = true;
    PoseCmdUpdated_ = false;
    isCmdPoseCached = false;

    cmdPoseWorld_mtx_.unlock();
    // checkSingleStepControlAndStop();
    // std::cout << "cmdPose_ : " << cmdPose_.transpose() << std::endl;
    // }
  };
  targetPoseWorldSubscriber_ =
      nodeHandle_.subscribe<geometry_msgs::Twist>("/cmd_pose_world", 1, targetPoseWorldCallback);

  // Arm TargetTrajectories
  auto armTargetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg)
  {
    if (newArmControlMode_ != ArmControlMode::EXTERN_CONTROL && !only_half_up_body_)
    {
      ros::Time currentTime = ros::Time::now();
      if ((currentTime - lastArmControlModeWarnTime_).toSec() >= 1.0) // 检查是否过了1秒
      {
        ROS_WARN_STREAM("[ReferenceManager]: received arm targetTrajectories, but current armControlMode_ " << std::to_string(currentArmControlMode_) << " != EXTERN_CONTROL");
        lastArmControlModeWarnTime_ = currentTime; 
      }
      return;
    }
    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
   
    if (targetTrajectories.stateTrajectory[0].size() != armJointNums_)
    {
      if (armRealDof_ == -1)// 还没获取到实物关节数
      {
        if (nodeHandle_.hasParam("/armRealDof"))
        {
          nodeHandle_.getParam("/armRealDof", armRealDof_);
          std::cout << "[ReferenceManager]:Get armRealDof: " << armRealDof_ << std::endl;
        }
        else
        {
          ROS_WARN_STREAM("[ReferenceManager]: arm targetTrajectories size : " << std::to_string(targetTrajectories.stateTrajectory[0].size()) << " is not equal to " << std::to_string(armJointNums_) << " and armRealDof_ is not set");
          return;
        }
      }
      if (targetTrajectories.stateTrajectory[0].size() != armRealDof_)
      {
        ROS_WARN_STREAM("[ReferenceManager]: arm targetTrajectories size : "
                        << std::to_string(targetTrajectories.stateTrajectory[0].size()) << " is not equal to RealArmDof: "
                        << std::to_string(armRealDof_) << " or MPCArmDof: " << std::to_string(armJointNums_));
        return;
      }
      else // 收到了和实物一致维度的轨迹，处理成和MPC一致的轨迹
      {
                TargetTrajectories originalTargetTrajectories = targetTrajectories;
        armFullDofTargetTrajectories_.setBuffer(std::move(originalTargetTrajectories));

        int armDofMPC_ = armJointNums_ / 2;
        int armDofReal_ = armRealDof_ / 2;
        int armDofDiff_ = armDofReal_ - armDofMPC_;

        TargetTrajectories newTargetTrajectories;
        int trajSize = targetTrajectories.timeTrajectory.size();
        for (int i = 0; i < trajSize; i++)
        {
          newTargetTrajectories.timeTrajectory.push_back(targetTrajectories.timeTrajectory[i]);
          vector_t newState = vector_t::Zero(armJointNums_);
          newState << targetTrajectories.stateTrajectory[i].head(armDofMPC_), targetTrajectories.stateTrajectory[i].segment(armDofReal_, armDofMPC_);
          newTargetTrajectories.stateTrajectory.push_back(newState);
          newTargetTrajectories.inputTrajectory.push_back(vector_t::Zero(info_.inputDim));// no use
        }
        targetTrajectories = newTargetTrajectories;
      }
    }
    else{
    } // 收到了MPC一致维度的轨迹
    if (targetTrajectories.stateTrajectory[0].size() != armJointNums_)
    {
      ROS_WARN_STREAM("[ReferenceManager]: arm targetTrajectories size : " << std::to_string(targetTrajectories.stateTrajectory[0].size()) << " is not equal to " << std::to_string(armJointNums_));
      return;
    }
    armTargetTrajectories_.setBuffer(std::move(targetTrajectories));
    armTargetUpdated_ = true;
    if (!isArmControlModeChanged_ && currentArmControlMode_ == ArmControlMode::EXTERN_CONTROL) // 只有当前没有在修改mode，并且在EXTERN_CONTROL模式下才更新armTargetCommanded_
    {
      std::lock_guard<std::mutex> lock(armTargetCommanded_mtx_);
      armTargetCommandedPublisher_.publish(*msg);
    }
    else if(only_half_up_body_) 
    //&& currentArmControlMode_ == ArmControlMode::EXTERN_CONTROL)
    {
      std::lock_guard<std::mutex> lock(armTargetCommanded_mtx_);
      armTargetCommandedPublisher_.publish(*msg);
    }

    
  };
  armTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<ocs2_msgs::mpc_target_trajectories>(nodeHandleName+"_mpc_target_arm", 3, armTargetTrajectoriesCallback);
  armTargetCommandedPublisher_ = nodeHandle_.advertise<ocs2_msgs::mpc_target_trajectories>(nodeHandleName + "_mpc_arm_commanded", 1);// 发布当前使用的手臂目标轨迹

// Waist TargetTrajectories
  auto waistTargetTrajectoriesCallback = [this](const kuavo_msgs::robotWaistControl::ConstPtr &msg)
  {
    joyWaist_ = Eigen::Map<const Eigen::VectorXd>(msg->data.data.data(), msg->data.data.size()) * 3.1415926 / 180.0;
    waistTargetUpdated_ = true;
  };
  waistTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<kuavo_msgs::robotWaistControl>("/robot_waist_motion_data", 3, waistTargetTrajectoriesCallback);


  // pose TargetTrajectories
  auto poseTargetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg)
  {

    auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
    if (targetTrajectories.stateTrajectory[0].size() != 6)
    {
      ROS_WARN_STREAM("pose targetTrajectories size : " << std::to_string(targetTrajectories.stateTrajectory[0].size()) << " is not a 6dof pose");
      return;
    }
    poseTargetTrajectories_.setBuffer(std::move(targetTrajectories));
    poseTargetUpdated_ = true;
    checkSingleStepControlAndStop();
  };
  poseTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<ocs2_msgs::mpc_target_trajectories>(nodeHandleName+"_mpc_target_pose", 3, poseTargetTrajectoriesCallback);
  // 6D足部姿态目标轨迹回调函数
  auto footPose6DTargetTrajectoriesCallback = [this](const kuavo_msgs::footPose6DTargetTrajectories::ConstPtr &msg)
  {
    this->processFootPose6DTargetTrajectories(msg, FrameType::Local);
  };

  // 6D世界坐标系足部姿态目标轨迹回调函数
  auto footPose6DWorldTargetTrajectoriesCallback = [this](const kuavo_msgs::footPose6DTargetTrajectories::ConstPtr &msg)
  {
    insert_time  = msg->insertTime;
    this->processFootPose6DTargetTrajectories(msg, FrameType::World);
  };

  // 4D足部姿态目标轨迹回调函数 - 通过调用6D回调函数实现
  auto footPoseTargetTrajectoriesCallback = [this](const kuavo_msgs::footPoseTargetTrajectories::ConstPtr &msg)
  {
    // 将4D消息转换为6D消息
    kuavo_msgs::footPose6DTargetTrajectories::Ptr msg6d(new kuavo_msgs::footPose6DTargetTrajectories);
    
    // 复制基本字段
    msg6d->timeTrajectory = msg->timeTrajectory;
    msg6d->footIndexTrajectory = msg->footIndexTrajectory;
    msg6d->swingHeightTrajectory = msg->swingHeightTrajectory;
    
    // 转换足部姿态从4D到6D
    msg6d->footPoseTrajectory.resize(msg->footPoseTrajectory.size());
    for(size_t i = 0; i < msg->footPoseTrajectory.size(); i++)
    {
      // 复制4D数据到6D的前4个元素，后2个元素设为0
      for(int j = 0; j < 4; j++)
      {
        msg6d->footPoseTrajectory[i].footPose6D[j] = msg->footPoseTrajectory[i].footPose[j];
      }
      msg6d->footPoseTrajectory[i].footPose6D[4] = 0.0; // pitch
      msg6d->footPoseTrajectory[i].footPose6D[5] = 0.0; // roll
      
      // 复制躯干姿态，从4D扩展到6D
      for(int j = 0; j < 4; j++)
      {
        msg6d->footPoseTrajectory[i].torsoPose6D[j] = msg->footPoseTrajectory[i].torsoPose[j];
      }
      msg6d->footPoseTrajectory[i].torsoPose6D[4] = 0.0; // pitch
      msg6d->footPoseTrajectory[i].torsoPose6D[5] = 0.0; // roll
    }
    
    // 转换额外足部姿态
    msg6d->additionalFootPoseTrajectory.resize(msg->additionalFootPoseTrajectory.size());
    for(size_t i = 0; i < msg->additionalFootPoseTrajectory.size(); i++)
    {
      msg6d->additionalFootPoseTrajectory[i].data.resize(msg->additionalFootPoseTrajectory[i].data.size());
      for(size_t j = 0; j < msg->additionalFootPoseTrajectory[i].data.size(); j++)
      {
        for(int k = 0; k < 4; k++)
        {
          msg6d->additionalFootPoseTrajectory[i].data[j].footPose6D[k] = msg->additionalFootPoseTrajectory[i].data[j].footPose[k];
        }
        msg6d->additionalFootPoseTrajectory[i].data[j].footPose6D[4] = 0.0; // pitch
        msg6d->additionalFootPoseTrajectory[i].data[j].footPose6D[5] = 0.0; // roll
      }
    }
    
    // 调用公用的6D处理函数
    this->processFootPose6DTargetTrajectories(msg6d, FrameType::Local);
  };

  // 4D世界坐标系足部姿态目标轨迹回调函数 - 通过调用6D回调函数实现
  auto footPoseWorldTargetTrajectoriesCallback = [this](const kuavo_msgs::footPoseTargetTrajectories::ConstPtr &msg)
  {
    // 将4D消息转换为6D消息
    kuavo_msgs::footPose6DTargetTrajectories::Ptr msg6d(new kuavo_msgs::footPose6DTargetTrajectories);
    
    // 复制基本字段
    msg6d->timeTrajectory = msg->timeTrajectory;
    msg6d->footIndexTrajectory = msg->footIndexTrajectory;
    msg6d->swingHeightTrajectory = msg->swingHeightTrajectory;
    
    // 转换足部姿态从4D到6D
    msg6d->footPoseTrajectory.resize(msg->footPoseTrajectory.size());
    for(size_t i = 0; i < msg->footPoseTrajectory.size(); i++)
    {
      // 复制4D数据到6D的前4个元素，后2个元素设为0
      for(int j = 0; j < 4; j++)
      {
        msg6d->footPoseTrajectory[i].footPose6D[j] = msg->footPoseTrajectory[i].footPose[j];
      }
      msg6d->footPoseTrajectory[i].footPose6D[4] = 0.0; // pitch
      msg6d->footPoseTrajectory[i].footPose6D[5] = 0.0; // roll
      
      // 复制躯干姿态，从4D扩展到6D
      for(int j = 0; j < 4; j++)
      {
        msg6d->footPoseTrajectory[i].torsoPose6D[j] = msg->footPoseTrajectory[i].torsoPose[j];
      }
      msg6d->footPoseTrajectory[i].torsoPose6D[4] = 0.0; // pitch
      msg6d->footPoseTrajectory[i].torsoPose6D[5] = 0.0; // roll
    }
    
    // 转换额外足部姿态
    msg6d->additionalFootPoseTrajectory.resize(msg->additionalFootPoseTrajectory.size());
    for(size_t i = 0; i < msg->additionalFootPoseTrajectory.size(); i++)
    {
      msg6d->additionalFootPoseTrajectory[i].data.resize(msg->additionalFootPoseTrajectory[i].data.size());
      for(size_t j = 0; j < msg->additionalFootPoseTrajectory[i].data.size(); j++)
      {
        for(int k = 0; k < 4; k++)
        {
          msg6d->additionalFootPoseTrajectory[i].data[j].footPose6D[k] = msg->additionalFootPoseTrajectory[i].data[j].footPose[k];
        }
        msg6d->additionalFootPoseTrajectory[i].data[j].footPose6D[4] = 0.0; // pitch
        msg6d->additionalFootPoseTrajectory[i].data[j].footPose6D[5] = 0.0; // roll
      }
    }
    
    // 调用公用的6D处理函数
    this->processFootPose6DTargetTrajectories(msg6d, FrameType::World);
  };

  auto fullBodyTargetTrajectoriesCallback = [this](const kuavo_msgs::fullBodyTargetTrajectories::ConstPtr &msg)
  {
    if (!update_full_body_trajecory_)
    {
      const int size = msg->timeTrajectory.size();
      if (size ==0 ||size != msg->footIndexTrajectory.size() || size != msg->fullBodyQTrajectory.size())
      {
        ROS_WARN_STREAM("fullBodyTargetTrajectories size is not equal to timeTrajectory, footIndexTrajectory, and footPoseTrajectory");
        return;
      }
      ROS_INFO_STREAM("[ReferenceManager]: receiving fullBodyTargetTrajectories, size: " << size);
      
      auto qvToEigen = [](const kuavo_msgs::qv &qv_msg) -> Eigen::VectorXd
      {
        if (qv_msg.value.empty())
        {
          throw std::runtime_error("Empty qv message value");
        }
        return Eigen::Map<const Eigen::VectorXd>(qv_msg.value.data(), qv_msg.value.size());
      };
      bool has_head_trajectory = false;
      {
        if (qvToEigen(msg->fullBodyQTrajectory[0]).size() == 7 + feetJointNums_ + armRealDof_ + 2)
        {
          has_head_trajectory = true;
        }
        ROS_INFO_STREAM("[ReferenceManager]: has_head_trajectory: " << has_head_trajectory);
      }

      auto getSimplifiedActuatedPos = [&](const vector_t &drake_q) -> vector_t
      {
        vector_t simplified_actuated_pos(info_.actuatedDofNum);
        simplified_actuated_pos << drake_q.segment(7, feetJointNums_), drake_q.segment(7 + feetJointNums_, armJointNums_ / 2), drake_q.segment(7 + feetJointNums_ + armRealDof_ / 2, armJointNums_ / 2);
        return simplified_actuated_pos;
      };

      auto getSimplifiedActuatedVel = [&](const vector_t &drake_v) -> vector_t
      {
        vector_t simplified_actuated_vel(info_.actuatedDofNum);
        simplified_actuated_vel << drake_v.segment(6, feetJointNums_), drake_v.segment(6 + feetJointNums_, armJointNums_ / 2), drake_v.segment(6 + feetJointNums_ + armRealDof_ / 2, armJointNums_ / 2);
        return simplified_actuated_vel;
      };

      auto drake_qv_to_state = [&](const vector_t &drake_q, const vector_t &drake_v) -> vector_t
      {
        size_t nq = drake_q.size();
        size_t nv = drake_v.size();

        vector3_t body_pos = drake_q.segment<3>(4);
        vector_t body_quat = drake_q.segment<4>(0);
        vector3_t body_vel = drake_v.segment<3>(3);
        vector3_t body_angular_vel = drake_v.segment<3>(0);
        Eigen::Quaternion<scalar_t> body_quat_eigen(body_quat[0], body_quat[1], body_quat[2], body_quat[3]);
        vector3_t body_zyx = quatToZyx(body_quat_eigen);
        vector_t rbdState(info_.generalizedCoordinatesNum * 2);
        rbdState.segment<3>(0) = body_zyx;
        rbdState.segment<3>(3) = body_pos;
        vector_t simplified_actuated_pos = getSimplifiedActuatedPos(drake_q);
        vector_t simplified_actuated_vel = getSimplifiedActuatedVel(drake_v);
        rbdState.segment(6, info_.actuatedDofNum) = simplified_actuated_pos;
        rbdState.segment<3>(info_.generalizedCoordinatesNum) = body_angular_vel;
        rbdState.segment<3>(info_.generalizedCoordinatesNum + 3) = body_vel;
        rbdState.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum) = simplified_actuated_vel;
        return rbdConversions_.computeCentroidalStateFromRbdModel(rbdState);
      };
      // std::vector<scalar_t> eventTimes;
      // std::vector<size_t> modeIndices;
      { // gen mode schedule
        fullBodySchedule_.clear();
        
        // 从ROS参数获取调度控制参数
        bool schedule_enable = false;
        double schedule_start_time = -1.0;
        
        if (nodeHandle_.getParam("/mpc/schedule/enable", schedule_enable) && 
            nodeHandle_.getParam("/mpc/schedule/start_time", schedule_start_time) &&
            schedule_enable) {
            fullBodySchedule_.startTime = schedule_start_time;
            ROS_INFO_STREAM("[ReferenceManager]: Schedule enabled, start time: " << schedule_start_time);
            
            // 使用后立即重置enable参数
            nodeHandle_.setParam("/mpc/schedule/enable", false);
            ROS_INFO("[ReferenceManager]: Reset schedule enable to false");
        } else {
            fullBodySchedule_.startTime = -1.0;
            ROS_INFO("[ReferenceManager]: Schedule disabled or params not found, using default start time: -1");
        }
        
        auto temp_mode = static_cast<FootIdx>(msg->footIndexTrajectory[0]);
        auto temp_time = msg->timeTrajectory[0];
        fullBodySchedule_.footIndices.push_back(temp_mode);
        for (int i = 0; i < size; i++)
        {
          auto current_mode = static_cast<FootIdx>(msg->footIndexTrajectory[i]);
          // mode
          if (current_mode != temp_mode || (temp_mode == FootIdx::Stance && msg->timeTrajectory[i] - temp_time > 0.5))
          {// 0.5划分SS接触相,方便终止防止最后执行手臂动作时间很长
            fullBodySchedule_.footIndices.push_back(current_mode);
            fullBodySchedule_.eventTimes.push_back(msg->timeTrajectory[i]);
            temp_mode = current_mode;
            temp_time = msg->timeTrajectory[i];
            std::cout << "fullBodySchedule_.eventTimes.push_back(" << msg->timeTrajectory[i] << ") : " << static_cast<int>(current_mode) << ")" << std::endl;
          }

          // stateTrajectory
          vector_t fullBodyQ = qvToEigen(msg->fullBodyQTrajectory[i]);// 注意可能包含头部！
          vector_t fullBodyV = qvToEigen(msg->fullBodyVTrajectory[i]);
          // fullBodySchedule_.fullBodyQTrajectory.push_back(fullBodyQ);
          // fullBodySchedule_.fullBodyVTrajectory.push_back(fullBodyV);
          
          vector_t state = drake_qv_to_state(fullBodyQ, fullBodyV);
          if (fullBodySchedule_.targetTrajectories.size() < fullBodySchedule_.footIndices.size() )
          {
            TargetTrajectories targetTrajectories;
            targetTrajectories.timeTrajectory.push_back(msg->timeTrajectory[i]);
            targetTrajectories.stateTrajectory.push_back(state);
            targetTrajectories.inputTrajectory.push_back(vector_t::Zero(info_.inputDim));
            fullBodySchedule_.targetTrajectories.push_back(targetTrajectories);
          }
          else
          {
            fullBodySchedule_.targetTrajectories[fullBodySchedule_.eventTimes.size()].timeTrajectory.push_back(msg->timeTrajectory[i]);
            fullBodySchedule_.targetTrajectories[fullBodySchedule_.eventTimes.size()].stateTrajectory.push_back(state);
          }

          // armTargetTrajectories
          fullBodySchedule_.armTargetTrajectories.timeTrajectory.push_back(msg->timeTrajectory[i]);
          fullBodySchedule_.armTargetTrajectories.stateTrajectory.push_back(fullBodyQ.segment(7+feetJointNums_, armRealDof_));
          fullBodySchedule_.armTargetTrajectories.inputTrajectory.push_back(vector_t::Zero(armRealDof_));
          if (has_head_trajectory)
          {
            fullBodySchedule_.headTargetTrajectories.timeTrajectory.push_back(msg->timeTrajectory[i]);
            fullBodySchedule_.headTargetTrajectories.stateTrajectory.push_back(fullBodyQ.tail(2));
            fullBodySchedule_.headTargetTrajectories.inputTrajectory.push_back(vector_t::Zero(2));
          }
        }// 遍历完所有轨迹点

        {// last mode
          fullBodySchedule_.footIndices.push_back(FootIdx::Stance);
          fullBodySchedule_.eventTimes.push_back(msg->timeTrajectory[size-1]);
        }
      }
      ROS_INFO_STREAM("[ReferenceManager]: received fullBodyTargetTrajectories, process size: " << size 
                      << ", startTime: " << fullBodySchedule_.startTime);
      update_full_body_trajecory_ = true;
    }
  };

  auto estContactStateCallback = [this](const std_msgs::Float64::ConstPtr &msg) {
        estContactState_ = static_cast<int>(msg->data);
        isContactStateUpdated_ = true;
      };

  estContactStateSubscriber_ = nodeHandle_.subscribe<std_msgs::Float64>("/state_estimate/Contact_Detection/mode", 1, estContactStateCallback);

  footPoseTargetTrajectoriesService_ = nodeHandle_.advertiseService(nodeHandleName + "_mpc_foot_pose_target_trajectories_srv", 
      &SwitchedModelReferenceManager::footPoseTargetTrajectoriesSrvCallback, this);
  footPose6DTargetTrajectoriesService_ = nodeHandle_.advertiseService(nodeHandleName + "_mpc_foot_pose_6d_target_trajectories_srv", 
      &SwitchedModelReferenceManager::footPose6DTargetTrajectoriesSrvCallback, this);

  fullBodyTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<kuavo_msgs::fullBodyTargetTrajectories>(nodeHandleName+"_mpc_fullbody_target_trajectories", 3, fullBodyTargetTrajectoriesCallback);
      
  // 4D足部姿态目标轨迹订阅者
  footPoseTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<kuavo_msgs::footPoseTargetTrajectories>(nodeHandleName+"_mpc_foot_pose_target_trajectories", 3, footPoseTargetTrajectoriesCallback);
  // 4D世界坐标系足部姿态目标轨迹订阅者
  footPoseWorldTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<kuavo_msgs::footPoseTargetTrajectories>(nodeHandleName+"_mpc_foot_pose_world_target_trajectories", 3, footPoseWorldTargetTrajectoriesCallback);

  // 6D世界坐标系足部姿态目标轨迹订阅者
  footPose6DWorldTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<kuavo_msgs::footPose6DTargetTrajectories>(nodeHandleName+"_mpc_foot_pose_6d_world_target_trajectories", 3, footPose6DWorldTargetTrajectoriesCallback);

  // 6D足部姿态目标轨迹订阅者
  footPose6DTargetTrajectoriesSubscriber_ =
      nodeHandle_.subscribe<kuavo_msgs::footPose6DTargetTrajectories>(nodeHandleName+"_mpc_foot_pose_6d_target_trajectories", 3, footPose6DTargetTrajectoriesCallback);
  current_mode_service_ = nodeHandle_.advertiseService(nodeHandleName + "_get_current_gait", &SwitchedModelReferenceManager::getCurrentGaitCallback, this);
  change_arm_control_service_ = nodeHandle_.advertiseService(nodeHandleName + "_change_arm_ctrl_mode", &SwitchedModelReferenceManager::armControlModeSrvCallback, this);
  change_torso_control_service_ = nodeHandle_.advertiseService(nodeHandleName + "_change_torso_ctrl_mode", &SwitchedModelReferenceManager::torsoControlModeSrvCallback, this);
  get_arm_control_mode_service_ = nodeHandle_.advertiseService(nodeHandleName + "_get_arm_ctrl_mode", &SwitchedModelReferenceManager::getArmControlModeCallback, this);
  // auto_gait_mode_service_ = nodeHandle_.advertiseService(nodeHandleName + "_auto_gait",&SwitchedModelReferenceManager::autoGaitModeSrvCallback, this);
  footContactPointPublisher_ = nodeHandle_.advertise<std_msgs::Float32MultiArray>(nodeHandleName + "_foot_contact_point", 10);
  footDesiredPointPublisher_ = nodeHandle_.advertise<std_msgs::Float32MultiArray>(nodeHandleName + "_foot_desired_point", 10);
  singleStepControlService_ = nodeHandle_.advertiseService(nodeHandleName + "_single_step_control", &SwitchedModelReferenceManager::singleStepControlCallback, this);
  stopSingleStepControlService_ = nodeHandle_.advertiseService(nodeHandleName + "_stop_single_step_control", &SwitchedModelReferenceManager::stopSingleStepControlCallback, this);

  armTargetPublisher_ = nodeHandle_.advertise<ocs2_msgs::mpc_target_trajectories>(nodeHandleName+"_mpc_target_arm", 10);
  gaitTimeNamePublisher_ = nodeHandle_.advertise<kuavo_msgs::gaitTimeName>(nodeHandleName + "_mpc_gait_time_name", 10);
  ros_logger_ = new TopicLogger(nodeHandle_);

  isCustomGaitPublisher_ = nodeHandle_.advertise<std_msgs::Bool>("/monitor/is_custom_gait", 10);
  singleStepModePublisher_ = nodeHandle_.advertise<std_msgs::Bool>(nodeHandleName + "/single_step_mode", 10);
  currentFootPosesPublisher_ = nodeHandle_.advertise<std_msgs::Float32MultiArray>(nodeHandleName + "/current_foot_poses", 10);
  currentFootCenterPosePublisher_ = nodeHandle_.advertise<std_msgs::Float32MultiArray>(nodeHandleName + "/current_foot_center_pose", 10);
  isArmExecutingPublisher_ = nodeHandle_.advertise<std_msgs::Bool>(nodeHandleName + "/mpc/is_arm_executing", 1);

  eef_wrench_sub_ = nodeHandle_.subscribe<std_msgs::Float64MultiArray>(
    "/hand_wrench_cmd", 
    10, 
    [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if(msg->data.size() == 12)
        {
          vector_t wrench = vector_t::Zero(12);
          wrench << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11];
          armWrenchBuffer_.setBuffer(wrench);
         
        }
        else
        {
          std::cout << "[ReferenceManager] eef_wrench_sub_ size is not 12" << std::endl;
        }
    }
  );
  slope_planning_sub_ = nodeHandle_.subscribe<std_msgs::Bool>(
    "/humanoid/mpc/enable_slope_planning", 
    10, 
    [this](const std_msgs::Bool::ConstPtr& msg) {
      enable_slope_planning_ = msg->data;
      std::cout << "[ReferenceManager] slope planning: " << enable_slope_planning_ << std::endl;
    }
  );
  is_rl_controller_sub_ = nodeHandle_.subscribe<std_msgs::Float64>(
    "/humanoid_controller/is_rl_controller_", 
    10, 
    [this](const std_msgs::Float64::ConstPtr& msg) {
      prev_is_rl_controller_ = is_rl_controller_;
      is_rl_controller_ = msg->data;
      // 检测从RL切换到MPC（RL: >0.5, MPC: <=0.5）
      if (prev_is_rl_controller_ > 0.5 && is_rl_controller_ <= 0.5)
      {
        // 从RL切换到MPC时，直接重置joyWaist_为0，避免执行RL的腰部指令
        joyWaist_.setZero();
        ROS_INFO("[SwitchedModelReferenceManager] [RL->MPC] Reset joyWaist_ to zero");
      }
    }
  );
  enable_pitch_limit_service_ = nodeHandle_.advertiseService("/humanoid/mpc/enable_base_pitch_limit", &SwitchedModelReferenceManager::enablePitchLimitCallback, this);
  pitch_limit_status_service_ = nodeHandle_.advertiseService("/humanoid/mpc/pitch_limit_status", &SwitchedModelReferenceManager::pitchLimitStatusCallback, this);
  vr_waist_control_service_ = nodeHandle_.advertiseService("/humanoid/mpc/vr_waist_control", &SwitchedModelReferenceManager::vrWaistControlCallback, this);
  load_dynamic_qr_service_ = nodeHandle_.advertiseService("/humanoid/mpc/load_dynamic_qr", &SwitchedModelReferenceManager::loadDynamicQRCallback, this);
  modeSchedulePublisher_ = nodeHandle_.advertise<kuavo_msgs::kuavoModeSchedule>("/modeSchedule", 10);

  // 获取/robot_version参数，判断是否为roban版本
  if (nodeHandle_.hasParam("/robot_version")) {
    int robot_version = 0;
    nodeHandle_.getParam("/robot_version", robot_version);
    if (robot_version < 30) {
      is_roban_version_ = true;
      std::cout << "[SwitchedModelReferenceManager]: Detected roban version, robot_version=" << robot_version << std::endl;
    }
  }
}

bool SwitchedModelReferenceManager::enablePitchLimitCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  enable_pitch_limit_ = req.data;
  res.success = true;
  std::cout << "[ReferenceManager] enable_pitch_limit callback: " << enable_pitch_limit_ << std::endl;
  return true;
}

bool SwitchedModelReferenceManager::pitchLimitStatusCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  res.success = true;
  res.message = (enable_pitch_limit_ ? "enabled" : "disabled");
  return true;
}

bool SwitchedModelReferenceManager::vrWaistControlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  bool wasEnabled = vrWaistControlEnabled_;
  vrWaistControlEnabled_ = req.data;
  // 检测从VR腰部控制切换到非VR腰部控制
  if (wasEnabled && !vrWaistControlEnabled_) {
    // 计算高度跳变
    scalar_t currentHeight = cmdHeight_;
    scalar_t targetHeight = 0.0;  // 退出VR模式后的默认高度
    scalar_t heightJump = std::abs(currentHeight - targetHeight);

    // 计算pitch跳变
    scalar_t currentPitch = cmdPitch_;
    scalar_t targetPitch = initTargetState_[10];  // 退出VR模式后使用初始目标pitch
    scalar_t pitchJump = std::abs(currentPitch - targetPitch);

    // 检查是否需要触发平滑过渡（高度或pitch任一超过阈值）
    bool needHeightTransition = (heightJump > heightJumpThreshold_);
    bool needPitchTransition = (pitchJump > pitchJumpThreshold_);

    if (needHeightTransition || needPitchTransition) {
      // 根据高度跳变和最大速度计算所需时间
      scalar_t heightDuration = 0.0;
      if (needHeightTransition) {
        heightDuration = heightJump / heightTransitionMaxSpeed_;
      }

      // 根据pitch跳变和最大角速度计算所需时间
      scalar_t pitchDuration = 0.0;
      if (needPitchTransition) {
        pitchDuration = pitchJump / pitchTransitionMaxSpeed_;
      }

      // 取两者中的最大值作为过渡时间
      scalar_t calculatedDuration = std::max(heightDuration, pitchDuration);

      // 限制在最小和最大时间范围内
      heightTransitionDuration_ = std::max(heightTransitionMinDuration_,
                                          std::min(heightTransitionMaxDuration_, calculatedDuration));

      // 计算实际速度
      scalar_t actualHeightSpeed = (heightJump > 0) ? (heightJump / heightTransitionDuration_) : 0.0;
      scalar_t actualPitchSpeed = (pitchJump > 0) ? (pitchJump / heightTransitionDuration_) : 0.0;

      // 设置过渡参数
      heightSmoothTransitionActive_ = true;
      heightTransitionStartTime_ = ros::Time::now().toSec();
      heightBeforeTransition_ = currentHeight;
      heightAfterTransition_ = targetHeight;
      pitchBeforeTransition_ = currentPitch;
      pitchAfterTransition_ = targetPitch;

      // 将cmdHeight_和cmdPitch_设置为目标值
      cmdHeight_ = targetHeight;
      cmdPitch_ = targetPitch;

      // 确保在平滑过渡期间持续生成轨迹
      PoseCmdUpdated_ = true;

      std::cout << "[ReferenceManager] 🔄 Smooth transition triggered:" << std::endl;
      if (needHeightTransition) {
        std::cout << "[ReferenceManager]    Height: " << heightJump << " m > "
                  << heightJumpThreshold_ << " m (threshold)" << std::endl;
        std::cout << "[ReferenceManager]    Height duration: " << heightDuration << " s" << std::endl;
      }
      if (needPitchTransition) {
        std::cout << "[ReferenceManager]    Pitch: " << (pitchJump * 180.0 / M_PI) << " deg > "
                  << (pitchJumpThreshold_ * 180.0 / M_PI) << " deg (threshold)" << std::endl;
        std::cout << "[ReferenceManager]    Pitch duration: " << pitchDuration << " s" << std::endl;
      }
      std::cout << "[ReferenceManager]    Final transition time: " << heightTransitionDuration_
                << " s (calculated: " << calculatedDuration << " s)" << std::endl;
      std::cout << "[ReferenceManager]    Actual height speed: " << actualHeightSpeed
                << " m/s (max: " << heightTransitionMaxSpeed_ << " m/s)" << std::endl;
      std::cout << "[ReferenceManager]    Actual pitch speed: " << (actualPitchSpeed * 180.0 / M_PI)
                << " deg/s (max: " << (pitchTransitionMaxSpeed_ * 180.0 / M_PI) << " deg/s)" << std::endl;

      res.message = "VR waist control disabled with smooth height and pitch transition";
    }
    else {
      heightSmoothTransitionActive_ = false;
      std::cout << "[ReferenceManager] Height jump is small (" << heightJump
                << " m <= " << heightJumpThreshold_
                << " m). No smooth transition needed." << std::endl;
      res.message = "VR waist control disabled";
    }
  }
  else {
    // 启用VR控制或保持当前状态，不需要平滑过渡
    heightSmoothTransitionActive_ = false;
    res.message = vrWaistControlEnabled_ ? "VR waist control enabled" : "VR waist control disabled";
  }

  res.success = true;
  std::cout << "[ReferenceManager] VR waist control callback: " << vrWaistControlEnabled_ << std::endl;
  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::setModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

vector_t SwitchedModelReferenceManager::getLocalPlannerVel(double initTime)
{
  const TargetTrajectories targetTraj = getTargetTrajectories();
  return getLocalPlannerVel(initTime, targetTraj);
}

vector_t SwitchedModelReferenceManager::getLocalPlannerVel(double initTime, const TargetTrajectories& targetTraj)
{
  // cal planner_v
  auto state0 = targetTraj.getDesiredState(initTime);
  auto state1 = targetTraj.getDesiredState(initTime + 0.015);
  vector_t planner_v = (state1.segment<3>(6) - state0.segment<3>(6)) / 0.015;
  planner_v[2] = 0.0;
  const Eigen::Matrix<scalar_t, 3, 1> planner_zyx(-state0(9), 0, 0);
  Eigen::Matrix<double, 3, 3> R_WB = getRotationMatrixFromZyxEulerAngles(planner_zyx);
  planner_v = R_WB * planner_v;
  // std::cout << "[GaitRecevier] planner_v: " << planner_v << std::endl;

  return planner_v;
}

double SwitchedModelReferenceManager::calTerrainHeight(const contact_flag_t& contact_flags, const feet_array_t<vector3_t>& feet_pos)
{
  double terrainHeight = 0.0;
  int num_feet_in_contact = 0;
   for (auto i = 0; i < feet_pos.size(); i++)
   {
      if (!contact_flags[i])
        continue;
      terrainHeight += feet_pos[i][2];
      num_feet_in_contact++;
   }
   if (num_feet_in_contact == 0)
      return 0.0;
   terrainHeight /= num_feet_in_contact;
   return terrainHeight;
}

// *********设置手臂轨迹***************
void SwitchedModelReferenceManager::setArmTrajectory(scalar_t current_time, scalar_t startTime, FullBodySchedule& fullBodySchedule)
{

  // 直接切换外部控制模式
  isArmControlModeChanged_ = true;
  isArmControlModeChangedTrigger_ = true;
  isCalcArmControlModeChangedTime_ = true;
  newArmControlMode_ = ArmControlMode::EXTERN_CONTROL;
  // 创建新的目标轨迹
  TargetTrajectories armTargetTrajectories = fullBodySchedule.armTargetTrajectories;
  
  for (int i = 0; i < armTargetTrajectories.timeTrajectory.size(); i++)
  {
    armTargetTrajectories.timeTrajectory[i] = armTargetTrajectories.timeTrajectory[i] + startTime;
  }
  fullBodyArmTargetTrajectories_ = armTargetTrajectories;
  // 将轨迹转换为ROS消息
  armTargetTrajectoriesMsg_ = ros_msg_conversions::createTargetTrajectoriesMsg(armTargetTrajectories.segmentTargetTrajectories(startTime - 1, startTime + 2, false));
  
  // 发布消息
  armTargetPublisher_.publish(armTargetTrajectoriesMsg_);
  
  ROS_INFO("[ReferenceManager]: Published arm trajectory with %zu points", armTargetTrajectories.timeTrajectory.size());

  // 处理头部轨迹
  if (!fullBodySchedule.headTargetTrajectories.timeTrajectory.empty()) {
    TargetTrajectories headTargetTrajectories = fullBodySchedule.headTargetTrajectories;
    
    // 时间校准
    for (int i = 0; i < headTargetTrajectories.timeTrajectory.size(); i++) {
      headTargetTrajectories.timeTrajectory[i] = headTargetTrajectories.timeTrajectory[i] + startTime;
    }
    fullBodyHeadTargetTrajectories_ = headTargetTrajectories;
  }
}

// ***************校准广义位置轨迹的起点xy和yaw***************
void SwitchedModelReferenceManager::processFullBodySchedule(const vector_t& initState, FullBodySchedule& fullBodySchedule)
{
  lastFootCalibrationDiffXY_ = Eigen::Vector2d::Zero();
  auto first_state = fullBodySchedule.targetTrajectories[0].stateTrajectory[0];
  // Eigen::Vector2d pos_diff = first_state.segment<2>(6) - initState.segment<2>(6);
  double yaw_diff = first_state(9) - initState(9);  // yaw在第9个位置
  // 创建yaw旋转矩阵
  Eigen::Matrix2d R_init;
  R_init << std::cos(initState(9)), -std::sin(initState(9)),
            std::sin(initState(9)), std::cos(initState(9));
            
  Eigen::Matrix2d R_first;
  R_first << std::cos(first_state(9)), -std::sin(first_state(9)),
            std::sin(first_state(9)), std::cos(first_state(9));
            
  // 计算从first_state到initState的旋转矩阵
  Eigen::Matrix2d R_diff = R_init * R_first.transpose();
  
  // 遍历所有轨迹点进行调整
  for (int i = 0; i < fullBodySchedule.targetTrajectories.size(); i++)
  {
    auto& trajs = fullBodySchedule.targetTrajectories[i];
    for (int j = 0; j < trajs.timeTrajectory.size(); j++)
    {
      auto& state = trajs.stateTrajectory[j];
      // 调整位置(xy)
      Eigen::Vector2d pos = state.segment<2>(6);
      state.segment<2>(6) = R_diff * (pos - first_state.segment<2>(6)) + initState.segment<2>(6);
      // state[8] -= 0.02;
      // 调整yaw角
      state(9) = state(9) - yaw_diff;
      
      // 调整世界系线速度(vx, vy) - 状态向量前3个元素
      Eigen::Vector2d world_vel = state.segment<2>(0);
      state.segment<2>(0) = R_diff * world_vel;
      
      // 调整世界系角速度(wx, wy, wz) - 状态向量3-5元素
      Eigen::Vector2d world_angular_vel_xy = state.segment<2>(3);  // 取出角速度的xy分量
      state.segment<2>(3) = R_diff * world_angular_vel_xy;  // 对xy分量进行旋转变换
      // z轴角速度不需要变换，保持不变
    }
  }
  
  ROS_INFO("轨迹已调整对齐到初始状态");
}
TargetTrajectories SwitchedModelReferenceManager::generateTargetwithfullBodySchedule(scalar_t initTime, scalar_t scheduleStartTime, scalar_t scheduleEndTime, const vector_t& initState, 
                                                                     const TargetTrajectories& targetTrajectories, const FullBodySchedule& fullBodySchedule, ModeSchedule& modeSchedule)
{
  TargetTrajectories new_target_trajectories;
  new_target_trajectories.stateTrajectory.push_back(targetTrajectories.getDesiredState(initTime));
  new_target_trajectories.inputTrajectory.push_back(targetTrajectories.getDesiredInput(initTime));
  new_target_trajectories.timeTrajectory.push_back(initTime);
  for (int i = 0; i < fullBodySchedule.targetTrajectories.size(); i++)
  {
    auto trajs = fullBodySchedule.targetTrajectories[i];
    for (int j = 0; j < trajs.timeTrajectory.size(); j++)
    {
      new_target_trajectories.timeTrajectory.push_back(scheduleStartTime + trajs.timeTrajectory[j]);
      new_target_trajectories.stateTrajectory.push_back(trajs.stateTrajectory[j]);
      new_target_trajectories.inputTrajectory.push_back(vector_t::Zero(info_.inputDim));
    }
  }
  return new_target_trajectories;
} 
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  feet_array_t<vector3_t> feet_pos = inverseKinematics_.computeFootPos(initState);
  auto current_mode = modeSchedule.modeAtTime(initTime);
  contact_flag_t contact_flags = modeNumber2StanceLeg(current_mode);
  if (isContactStateUpdated_)// 接触估计更新了
  {
    bool has_swing_leg = std::any_of(contact_flags.begin(), contact_flags.end(), [](bool flag) { return !flag; });
    if(has_swing_leg && estContactState_ == ModeNumber::SS) // 当前是腾空相并且接触估计回到双足支撑了
    {
      swingTrajectoryPtr_->interruptSwing();
    }
    isContactStateUpdated_ = false;
  }

  // slope planning
  swingTrajectoryPtr_->setSlopePlanning(enable_slope_planning_);

  double newTerrainHeight_ = this->calTerrainHeight(contact_flags, feet_pos);
  double alpha = 0.1;
  terrainHeight_ = (1-alpha)*terrainHeight_ + alpha*newTerrainHeight_;
  
  if (ros_logger_ != nullptr)
  {
    ros_logger_->publishValue("/humanoid/mpc/terrainHeight", terrainHeight_);
  }
  if (isFirstRun_)
  {
    initTargetState_ = initState;
    std::cout << "target_init:  " << initState.transpose() << std::endl;
    isFirstRun_ = false;
    vector_array_t stateTrajectory{initState.tail(armJointNums_)};
    vector_array_t inputTrajectory{vector_t::Zero(info_.inputDim)};
    vector_array_t waist_stateTrajectory{initState.segment(12+feetJointNums_, waistNums_)};
    vector_array_t waist_inputTrajectory{vector_t::Zero(waistNums_)};
    currentArmTargetTrajectories_ = {{0.0}, stateTrajectory, inputTrajectory};
    currentWaistTargetTrajectories_ = {{0.0}, waist_stateTrajectory, waist_inputTrajectory};
    currentArmTargetTrajectoriesWithAllJoints_ = {{0.0}, stateTrajectory, inputTrajectory};
  }
  std_msgs::Bool msg;
  msg.data = gaitSchedulePtr_->getModeSchedule().existValidFootPose(initTime);
  singleStepModePublisher_.publish(msg);

  bool autoGaitEnabled_ = gaitSchedulePtr_->isAutoGaitEnabled();
  // pose target

  if (velCmdUpdated_) // using vel cmd
  {
    cmdvel_mtx_.lock();
    currentCmdVel_ = cmdVel_;
    velCmdUpdated_ = false;
    cmdvel_mtx_.unlock();
    bool updated = true;
    // if (autoGaitEnabled_)
    // {
     updated = checkAndApplyCommandLine(initTime, finalTime, initState, currentCmdVel_);
    //  std::cout << "[mpc]updated: " << updated << std::endl;
    //  std::cout << "[mpc]cmdVel: " << cmdVel_.transpose() << std::endl;
    //  std::cout << "[mpc]cmdHeight: " << cmdHeight_ << std::endl;
    // }
    if (updated)
      targetTrajectories = generateTargetwithVelcmd(initTime, finalTime, initState, targetTrajectories, modeSchedule, currentCmdVel_);
  }
  else if (PoseCmdUpdated_ || heightSmoothTransitionActive_)  // 添加高度平滑过渡的检查
  {
    std::lock_guard<std::mutex> lock(cmdPose_mtx_);
    currentCmdPose_ = cmdPose_;
    cmdPose_mtx_.unlock();

    // 如果正在进行高度平滑过渡，即使PoseCmdUpdated_为false也要生成轨迹
    bool needUpdate = false;
    if (heightSmoothTransitionActive_) {
      needUpdate = true;
    } else {
      needUpdate = checkCmdPoseAndApplyCommandLine(initTime, finalTime, initState, currentCmdPose_);
    }

    if (needUpdate) {
      targetTrajectories = generateTargetwithPoscmd(initTime, initState, targetTrajectories, modeSchedule, currentCmdPose_);
    }

    if(!isCmdPoseCached && !heightSmoothTransitionActive_) {
      PoseCmdUpdated_ = false; // cmdPose没有缓存的数值时候，且不在高度过渡期间
    }

  } else if (PoseWorldCmdUpdated_)
  {
    auto handleSingleStepPhase = [&](scalar_t initTime, 
                                                      TargetTrajectories& targetTrajectories,
                                                      ModeSchedule& modeSchedule,
                                                      const feet_array_t<vector3_t>& feet_pos,
                                                      const vector_t& foot_center_pose,
                                                      bool verbose = false)
    {
      // 发布自定义步态消息
      kuavo_msgs::gaitTimeName gaitTimeNameMsg;
      gaitTimeNameMsg.start_time = initTime;
      gaitTimeNameMsg.gait_name = "custom_gait";
      gaitTimeNamePublisher_.publish(gaitTimeNameMsg);
      // 获取当前躯干姿态
      vector_t torso_pose = targetTrajectories.getDesiredState(initTime).segment<4>(6);
      // auto com_pos = getComPos(initState);
      // torso_pose.head(2) = com_pos.head(2);
      torso_pose.head(2) = foot_center_pose.head(2);
      // 获取足部轨迹计划
      FootPoseSchedule footPoseSchedule = swingTrajectoryPtr_->getfootPoseSchedule();
      ROS_DEBUG_STREAM("Foot pose schedule:\n" << footPoseSchedule);
      // 更新模式调度
      gaitSchedulePtr_->modifyModePoseSchedules(initTime, torso_pose, footPoseSchedule, feet_pos, -1, TargetTrajectories(), terrainHeight_);
      modeSchedule = gaitSchedulePtr_->getFullModeSchedule(); // 获取全程的modeSchedule
      // 生成目标轨迹
      vector_t torso_modified_state = initState;
      // torso_modified_state.segment<2>(6) = torso_pose.head(2);
      torso_modified_state.segment<2>(6) = foot_center_pose.head(2);
      targetTrajectories = generateTargetwithModeSchedule(initTime, finalTime, torso_modified_state, targetTrajectories, modeSchedule);
      // 结束单步控制阶段
      swingTrajectoryPtr_->setSingleStepPhase(false);
      // 调试信息输出
      if(verbose){
        ROS_DEBUG_STREAM("Target Trajectories in single step:");
        for (int i = 0; i < targetTrajectories.timeTrajectory.size(); i++) {
            ROS_DEBUG_STREAM("time[" << i << "]: " << targetTrajectories.timeTrajectory[i] 
                            << ", state: " << targetTrajectories.stateTrajectory[i].segment<4>(6).transpose());
        }
      }
    };
    const Eigen::Vector3d delta_pose = (Eigen::Vector3d() 
        << (cmdPoseWorld_.head(2) - initState.segment<2>(6)), 0).finished();
    
    // 坐标转换
    const Eigen::Vector3d current_zyx(initState(9), 0, 0);
    const Eigen::Matrix3d R_WBr = getRotationMatrixFromZyxEulerAngles(current_zyx);
    currentCmdPose_.head(2) = R_WBr.transpose() * delta_pose.head(2);
    currentCmdPose_(3) = normalizedYaw(normalizedYaw(cmdPoseWorld_[3]) - normalizedYaw(initState(9)));

    // 缓存世界坐标系下的命令
    if(swingTrajectoryPtr_->isSingleStepPhase()) {
      const Eigen::Matrix<scalar_t, 3, 1> targetZyx = {initState[9], 0, 0};
      currentCmdPose_.head(3) = getRotationMatrixFromZyxEulerAngles(targetZyx) * currentCmdPose_.head(3);
      cachedCmdPoseInWorldFrame_ = initState.segment<6>(6) + currentCmdPose_;
      cachedCmdPoseInWorldFrame_(2) = 0.0;
      cachedCmdPoseInWorldFrame_.tail(2).setZero();
      ismdPoseInWorldFrameCached_ = true;
    }

    auto feet_poses = getFeetPoses(initState);
    vector_t foot_avg_pose = (feet_poses[0] + feet_poses[1]) / 2.0;
    Eigen::Vector2d feet_torso_dis_err = initState.segment<2>(6) - foot_avg_pose.head(2);
    ros_logger_->publishValue("/humanoid/feet_torso_dis_err", feet_torso_dis_err.norm());
    // 处理单步控制
    if(swingTrajectoryPtr_->isSingleStepPhase()) {
      double start_time = swingTrajectoryPtr_->getFinalYawSingleStepTime(); 
      // if(feet_torso_dis_err.norm() > 0.05)
      if(0)
      {
        ROS_WARN_STREAM("Foot and torso x-y distance error is too large: " << feet_torso_dis_err.norm() << ", will NOT apply handleSingleStepPhase().");
      }
      else
        handleSingleStepPhase(start_time, targetTrajectories, modeSchedule, feet_pos, foot_avg_pose, true);
    }

    // 更新世界坐标系缓存
    if(ismdPoseInWorldFrameCached_) {
      // 局部系保留旧逻辑，在target这一名义状态上做变换
      const auto& init_target_state = targetTrajectories.getDesiredState(initTime);
      const auto current_torso_target = (vector_t(6) 
          << init_target_state.segment<2>(6), 0, init_target_state(9), 0, 0).finished();
      const Eigen::Matrix<scalar_t, 3, 1> targetZyx = {init_target_state[9], 0, 0};
      currentCmdPose_ = (cachedCmdPoseInWorldFrame_ - current_torso_target);
      currentCmdPose_.head(3) = getRotationMatrixFromZyxEulerAngles(targetZyx).transpose() * currentCmdPose_.head(3);
      ismdPoseInWorldFrameCached_ = false;
    }
    if(swingTrajectoryPtr_->getFinalYawSingleStepMode())
    {
      double target_yaw = swingTrajectoryPtr_->getTargetYaw();
      currentCmdPose_(3) = normalizedYaw(normalizedYaw(target_yaw) - normalizedYaw(initState(9)));
    }
    // 检查并应用命令
    checkCmdPoseAndApplyCommandLine(initTime, finalTime, initState, currentCmdPose_);
    bool sgl_mode = gaitSchedulePtr_->getFullModeSchedule().existValidFootPose(initTime);
    // 生成目标轨迹

    // ros_logger_->publishVector("/humanoid/mpc/currentCmdPose_", currentCmdPose_);
    if (!sgl_mode) 
    {
      targetTrajectories = generateTargetwithPoscmdInCurrentPose(initTime, initState, targetTrajectories, modeSchedule, currentCmdPose_);
    }
    
    // 更新标志位
    if(!isCmdPoseCached && (!sgl_mode && !swingTrajectoryPtr_->isSingleStepPhase() && !swingTrajectoryPtr_->getFinalYawSingleStepMode())) {
      PoseWorldCmdUpdated_ = false;
    }
  }
  if (poseTargetUpdated_) // using pose target
  {
    poseTargetTrajectories_.updateFromBuffer();
    switch (torsoControlMode_)
    {
    case TorsoControlMode::SIX_DOF:
      targetTrajectories.fillTargetTrajectories(poseTargetTrajectories_.get(), 6);
      break;
    case TorsoControlMode::ZYP:
      targetTrajectories.mergeTargetTrajectories(poseTargetTrajectories_.get(), 6+2, 2, 3);
      break;
    case TorsoControlMode::ZP:
      targetTrajectories.mergeTargetTrajectories(poseTargetTrajectories_.get(), 6+2, 2, 1);//height
      targetTrajectories.mergeTargetTrajectories(poseTargetTrajectories_.get(), 6+4, 4, 1);//pitch
      break;
    default:
      break;
    }
    poseTargetUpdated_ = false;
  }
  
 

  const auto timeHorizon = finalTime - initTime;
  const auto endTrajTime = finalTime + timeHorizon;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon, true);
  kuavo_msgs::kuavoModeSchedule modeScheduleMsg;
  modeScheduleMsg = createModeScheduleMsg(modeSchedule, initTime);
  modeSchedulePublisher_.publish(modeScheduleMsg);
  if(gaitSchedulePtr_->getGaitName(initTime) == "stance" && begin_step_gait == true)
  {
    if(initTime > customGait_start_time && initTime < customGait_end_time) // 切换stance时间必须在步态时间范围内
    {
      // 进入stance，更新状态
      std::cout << "stop cusom gait" << std::endl;
      targetTrajectories.cutTargetTrajectoriesAfter(initTime);
      begin_step_gait = false;
    }
    else if(initTime > customGait_end_time)
      {begin_step_gait = false;}
  }
  if(update_foot_trajectory_)
  {
    {
      begin_step_gait = true;
      const auto& torso_pose = targetTrajectories.getDesiredState(initTime).segment<4>(6);
      modeSchedule = gaitSchedulePtr_->modifyModePoseSchedules(initTime, torso_pose, footPoseSchedule_, feet_pos, footPoseSchedule_.startTime, targetTrajectories, terrainHeight_);
      targetTrajectories = generateTargetwithModeSchedule(initTime, finalTime, initState, targetTrajectories, modeSchedule);

      customGait_start_time = gaitSchedulePtr_->getCustomGaitStartTime();
      customGait_end_time = gaitSchedulePtr_->getCustomGaitEndTime();
      kuavo_msgs::gaitTimeName gaitTimeNameMsg;
      gaitTimeNameMsg.start_time = customGait_start_time;
      std::cout << "custom gait start time: " << gaitTimeNameMsg.start_time << std::endl;
      gaitTimeNameMsg.gait_name = "custom_gait";
      gaitTimeNamePublisher_.publish(gaitTimeNameMsg);//TODO: 临时添加，用于适配站立时的状态估计

      kuavo_msgs::gaitTimeName gaitTimeNameMsgEnd;

      gaitTimeNameMsgEnd.start_time = customGait_end_time;
      std::cout << "custom gait end time: " << gaitTimeNameMsgEnd.start_time << std::endl;
      gaitTimeNameMsgEnd.gait_name = "stance";
      gaitTimeNamePublisher_.publish(gaitTimeNameMsgEnd);//TODO: 临时添加，用于适配站立时的状态估计
      
      // 添加单步步态
      gaitSchedulePtr_->addGait(customGait_start_time, "custom_gait");
      gaitSchedulePtr_->addGait(customGait_end_time, "stance");
    }
    update_foot_trajectory_ = false;
  }

  if(update_foot_world_trajectory_)
  {
    const auto &torso_pose = targetTrajectories.getDesiredState(initTime).segment<4>(6);
    modeSchedule = gaitSchedulePtr_->modifyModeWorldPoseSchedules(initTime, torso_pose, footPoseWorldSchedule_, feet_pos, footPoseWorldSchedule_.startTime, targetTrajectories, insert_time);
    targetTrajectories = generateTargetwithModeScheduleWorld(initTime, finalTime, initState, targetTrajectories, modeSchedule);

    kuavo_msgs::gaitTimeName gaitTimeNameMsg;
    gaitTimeNameMsg.start_time = gaitSchedulePtr_->getCustomGaitStartTime();
    std::cout << "custom gait start time: " << gaitTimeNameMsg.start_time << std::endl;
    gaitTimeNameMsg.gait_name = "custom_gait";
    gaitTimeNamePublisher_.publish(gaitTimeNameMsg); // TODO: 临时添加，用于适配站立时的状态估计

    kuavo_msgs::gaitTimeName gaitTimeNameMsgEnd;

    gaitTimeNameMsgEnd.start_time = gaitSchedulePtr_->getCustomGaitEndTime();
    gaitTimeNameMsgEnd.gait_name = "stance";
    gaitTimeNamePublisher_.publish(gaitTimeNameMsgEnd); // TODO: 临时添加，用于适配站立时的状态估计

    update_foot_world_trajectory_ = false;
    insert_time = 0.0; // 重置插入时间
  }

  if(update_stop_single_step_)
  {
    gaitSchedulePtr_->disableFoot(initTime);
    targetTrajectories = generateTargetAsCurrent(initTime, finalTime, targetTrajectories.getDesiredState(initTime));
    update_stop_single_step_ = false;
    std::cout << "[SwitchedModelReferenceManager] Single step control stopped." << std::endl;
  }

  if (update_full_body_trajecory_)
  {
    std::cout << "update_full_body_trajecory_" << std::endl;
    const auto &torso_pose = targetTrajectories.getDesiredState(initTime).segment<4>(6);
    // feet_array_t<vector3_t> foot_pos = inverseKinematics_.computeFootPos(initState);
    processFullBodySchedule(initState, fullBodySchedule_);
    modeSchedule = gaitSchedulePtr_->modifyModeFullBodySchedules(initTime, torso_pose, fullBodySchedule_, feet_pos, fullBodySchedule_.startTime);
    std::cout << "update modeSchedule" << std::endl;
    fullbodyScheduleStartTime_ = gaitSchedulePtr_->getFullBodyGaitStartTime();
    std::cout << "fullbodyScheduleStartTime_: " << fullbodyScheduleStartTime_ << std::endl;
    fullbodyScheduleEndTime_ = gaitSchedulePtr_->getFullBodyGaitEndTime();
    std::cout << "fullbodyScheduleEndTime_: " << fullbodyScheduleEndTime_ << std::endl;
    if (fullbodyScheduleStartTime_ > 0 && fullbodyScheduleEndTime_ > 0)
    {
      fullBodyTargetTrajectories_ = generateTargetwithfullBodySchedule(initTime, fullbodyScheduleStartTime_, fullbodyScheduleEndTime_, initState, targetTrajectories, fullBodySchedule_, modeSchedule);
      targetTrajectories = fullBodyTargetTrajectories_.segmentTargetTrajectories(initTime - timeHorizon, finalTime + timeHorizon, false);
      kuavo_msgs::gaitTimeName gaitTimeNameMsg;
      gaitTimeNameMsg.start_time = fullbodyScheduleStartTime_;
      std::cout << "custom gait start time: " << gaitTimeNameMsg.start_time << std::endl;
      gaitTimeNameMsg.gait_name = "custom_gait";
      gaitTimeNamePublisher_.publish(gaitTimeNameMsg);
      setArmTrajectory(initTime, fullbodyScheduleStartTime_, fullBodySchedule_);
      kuavo_msgs::gaitTimeName gaitTimeNameMsgEnd;

      gaitTimeNameMsgEnd.start_time = fullbodyScheduleEndTime_;
      gaitTimeNameMsgEnd.gait_name = "stance";
      gaitTimeNamePublisher_.publish(gaitTimeNameMsgEnd);
    } //跳过执行全身控制
    update_full_body_trajecory_ = false;
  }

  gaitSchedulePtr_->countSteps(initTime);
  // std::cout << "currentCmdVel_" << currentCmdVel_  << std::endl;
  auto planner_vel = this->getLocalPlannerVel(initTime, targetTrajectories);
  ros_logger_->publishVector("/humanoid/mpc/planner_vel", planner_vel);
  swingTrajectoryPtr_->setBodyVelCmd(planner_vel);

  armWrenchBuffer_.updateFromBuffer();  
  vector_t armWrench = armWrenchBuffer_.get();
  swingTrajectoryPtr_->setArmEeWrenchConstraint(armWrench);

  swingTrajectoryPtr_->setCurrentState(initState);
  swingTrajectoryPtr_->setCurrentBodyState(initState.segment(0, 12));
  swingTrajectoryPtr_->setCurrentFeetPosition(feet_pos);
  publishFootContactPoint();
  publishFootDesiredPoint(initTime);
  // scalar_t terrainHeight_ = 0.0;
  // if(modeSchedule.modeAtTime(initTime) == ModeNumber::SS)
  // if(!modeSchedule.existValidFootPose())
  // {
  //     auto footPosition = swingTrajectoryPtr_->getNextFootPositions();
  //     terrainHeight_ = (footPosition[0].z() + footPosition[3].z()) / 2.0;
  //     // std::cout << "[mpc] terrain height: " << terrainHeight_ << std::endl;
  // };
  auto gait_time_name = gaitSchedulePtr_->getGgaitTimeName();
  // TODO: 短期内不启动动态QR，待mpc维度适配完毕后再添加（为了提高站立时的躯干高度控制精度，启用动态QR）
  // if(is_roban_version_ && last_gait_name_ != gait_time_name.second)
  // std::cout << "✅ gait changed from " << last_gait_name_ << " to " << gait_time_name.second << std::endl;
  // vrWaistControlEnabled_切换时也需要启动动态Q
  if(swingTrajectoryPlannerConfig_.enable_dynamic_q && ((last_gait_name_ != gait_time_name.second) || (vrWaistControlEnabled_!=vrWaistControlEnabledPrev_)))// 通过task.info的enable_dynamic_qr启用动态QR
  {
    setMatrixQByGaitPair(gait_time_name.second, gait_time_name.first);
  }
  // 增加判断是否存在customgait，如果从stance切换到customgait也会触发
  // 获取当前的 modeSchedule 并检查是否所有模式都是 SS
  //const auto& modeSchedule = gaitSchedulePtr_->getModeSchedule();
  bool all_stance = std::all_of(modeSchedule.modeSequence.begin(),
                                modeSchedule.modeSequence.end(),
                                [](size_t mode) { return mode == ModeNumber::SS; });
  // std::cout << "✅[setMatrixRByGaitPair] Gait name: " << gait_time_name.second << ", All modes SS: " << (all_stance ? "Yes" : "No") << std::endl;

  if(swingTrajectoryPlannerConfig_.enable_dynamic_r && ((last_gait_name_ != gait_time_name.second || all_stance != last_all_stance_)||(vrWaistControlEnabled_!=vrWaistControlEnabledPrev_)))// 通过task.info的enable_dynamic_qr启用动态QR
  {
    setMatrixRByGaitPair(gait_time_name.second, gait_time_name.first, all_stance);
  }
  last_all_stance_ = all_stance;
  last_gait_name_ = gait_time_name.second;
  vrWaistControlEnabledPrev_ = vrWaistControlEnabled_;
  std::string current_gait = gaitSchedulePtr_->getGaitName(initTime);
  static bool is_fullbody_trajectory_processed = false;
  if (initTime > fullbodyScheduleStartTime_ && initTime < fullbodyScheduleEndTime_) {
    processFullBodyTrajectories(initTime, finalTime, timeHorizon, targetTrajectories, initState, feet_pos);
    // 处理完全身轨迹后，再去更新足部轨迹
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight_, targetTrajectories, initTime);
    fullbodyScheduleEndTime_ = gaitSchedulePtr_->getFullBodyGaitEndTime();// 更新结束时间，用于判断是否手动清空了mode序列
    is_fullbody_trajectory_processed = true;
  }
  else
  {
    if (is_fullbody_trajectory_processed)// 从fullbody轨迹切换时，需要重置
    {
      std::cout << "[SwitchedModelReferenceManager] fullbody trajectory processed end, reset target" << std::endl;
      targetTrajectories = targetTrajectories.segmentTargetTrajectories(initTime - timeHorizon, initTime + 0.1);
      auto armTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(
      fullBodyArmTargetTrajectories_.segmentTargetTrajectories(initTime - timeHorizon, initTime + 0.1));
      armTargetPublisher_.publish(armTargetTrajectoriesMsg);
      is_fullbody_trajectory_processed = false;
    }
    swingTrajectoryPtr_->update(modeSchedule, terrainHeight_, targetTrajectories, initTime);
    // TODO: roban分支需要确认禁用关节参考
    // if (current_gait != "walk")
    if (!is_roban_version_ && current_gait != "walk")
    {
      calculateJointRef(initTime, endTrajTime, initState, targetTrajectories, modeSchedule);
    }
    else
    {
      // std::cout << "[SwitchedModelReferenceManager] Gait changed from " << current_gait << " to " << gaitSchedulePtr_->getLastGaitName()
      //           << ", resetting norminal state reference." << std::endl;
      for (auto &state_traj : targetTrajectories.stateTrajectory)
      {
        state_traj.tail(info_.actuatedDofNum) = info_.qPinocchioNominal.tail(info_.actuatedDofNum);
      }
    }
  }

  auto feet_poses = getFeetPoses(initState);
  vector_t foot_avg_pose = (feet_poses[0] + feet_poses[1]) / 2.0;
  // publish
  {
    std_msgs::Float32MultiArray foot_center_pose_msg;
    for(int j = 0; j < foot_avg_pose.size(); j++)
      foot_center_pose_msg.data.push_back(foot_avg_pose(j));
    currentFootCenterPosePublisher_.publish(foot_center_pose_msg);

    std_msgs::Float32MultiArray msg;
    for(int i = 0; i < feet_poses.size(); i++)
    {
      for(int j = 0; j < feet_poses[i].size(); j++)
        msg.data.push_back(feet_poses[i](j));
    }
    // [xyz-yaw_pitch_roll, xyz-yaw_pitch_roll,...]
    currentFootPosesPublisher_.publish(msg);
  }
  if(current_gait == "walk" && current_gait != gaitSchedulePtr_->getLastGaitName())
  {
    ROS_INFO_STREAM("Gait changed from " << current_gait << " to " << gaitSchedulePtr_->getLastGaitName() << ", resetting torso position.");
    // 覆盖躯干target(x,y,yaw)
    for(auto &state_traj: targetTrajectories.stateTrajectory)
    {
      state_traj.segment<2>(6) = foot_avg_pose.head(2);
      // state_traj(3) = foot_avg_pose(3);//暂时不覆盖yaw
    }
  }
  // arm target
  auto getDesiredArmTargetTrajectories = [&](TargetTrajectories& newArmTargetTrajectories, ArmControlMode mode)
  {
    bool updated = false;
    if (mode == ArmControlMode::AUTO_SWING)
    {
      updated = true;
      newArmTargetTrajectories = swingTrajectoryPtr_->getSwingArmTargetTrajectories(initTime);
      return updated;
      // newArmTargetTrajectories = swingTrajectoryPtr_->getSwingArmTargetTrajectories();
    }
    else if (mode == ArmControlMode::EXTERN_CONTROL)
    {
      if (armTargetUpdated_)
      {
        armTargetTrajectories_.updateFromBuffer();
        newArmTargetTrajectories = armTargetTrajectories_.get();
        int size = newArmTargetTrajectories.timeTrajectory.size();
        updated = (newArmTargetTrajectories.timeTrajectory[size - 1] < currentArmTargetTrajectories_.timeTrajectory[0]) ? false : true;
        armTargetUpdated_ = false;
        return updated;
      }
    }else
    {
      newArmTargetTrajectories = {{initTime}, {currentArmTargetTrajectories_.getDesiredState(initTime)}, {vector_t::Zero(info_.inputDim)}};
      updated = true;
    }
    return updated; 
  };

  if (armJointNums_ > 0)
  {
    if (isArmControlModeChanged_) // arm control mode changed, interpolate arm target
    {
      TargetTrajectories newArmTargetTrajectories;
      if (getDesiredArmTargetTrajectories(newArmTargetTrajectories, newArmControlMode_)) // 只有target更新了才会进行插值
      {
        auto current_arm_state = currentArmTargetTrajectories_.getDesiredState(initTime); // current arm state
        auto new_desire_arm_state = newArmTargetTrajectories.getDesiredState(initTime);

        auto err = (new_desire_arm_state - current_arm_state).norm();

        currentArmTargetTrajectories_ = interpolateArmTarget(initTime, current_arm_state, new_desire_arm_state, arm_move_spd_);

        { // 更新简化关节部分的target

          armFullDofTargetTrajectories_.updateFromBuffer();
          auto lastArmFullDofTargetTrajectories = armFullDofTargetTrajectories_.get();
            
          int armDofMPC_ = armJointNums_ / 2;
          int armDofReal_ = armRealDof_ / 2;
          auto generateFullState = [&](const vector_t &simplifiedState, const vector_t &defaultState) -> vector_t
          {
            vector_t newState = defaultState;
            newState.head(armDofMPC_) = simplifiedState.head(armDofMPC_);
            newState.segment(armDofReal_, armDofMPC_) = simplifiedState.segment(armDofMPC_, armDofMPC_);
            return newState;
          };

          vector_t st_state = vector_t::Zero(armRealDof_);
          vector_t end_state = (newArmControlMode_ != ArmControlMode::EXTERN_CONTROL && newArmControlMode_ != ArmControlMode::KEEP) ? vector_t::Zero(armRealDof_) : lastArmFullDofTargetTrajectories.getDesiredState(initTime);
          if (isArmControlModeChangedTrigger_) // 只有第一个周期修改一次插值起点
          {
            // 记录模式切换开始时间
            arm_mode_change_start_time_ = initTime;
            switch (currentArmControlMode_)
            {
              case ArmControlMode::EXTERN_CONTROL: // 当前是EXTERN_CONTROL，获取起点为当前的arm target
                st_state = lastArmFullDofTargetTrajectories.getDesiredState(initTime);
                // std::cout << "EXTERN_CONTROL st_state: " << st_state.transpose() << std::endl;
                break;
              case ArmControlMode::KEEP: // 当前是KEEP，获取起点为当前的全自由度的arm target
                st_state = currentArmTargetTrajectoriesWithAllJoints_.getDesiredState(initTime);
                // std::cout << "KEEP st_state: " << st_state.transpose() << std::endl;
                break;
              default: // 当前是AUTO_SWING，获取起点为0
                st_state = vector_t::Zero(armRealDof_);
                // std::cout << "AUTO_SWING st_state: " << st_state.transpose() << std::endl;
                break;
            }
            isArmControlModeChangedTrigger_ = false;
          }
          else
          {
            st_state = currentArmTargetTrajectoriesWithAllJoints_.getDesiredState(initTime); // 使用此前插值轨迹中的点进行迭代
          }

          auto current_arm_state_full = generateFullState(current_arm_state, st_state);
          auto new_arm_state_full = generateFullState(new_desire_arm_state, end_state);
          err = (new_arm_state_full - current_arm_state_full).norm();
          if(isCalcArmControlModeChangedTime_)
          {
            min_arm_mode_change_time_ = err / arm_move_spd_;
            isCalcArmControlModeChangedTime_ = false;
            std::cout << "[mpc] arm control mode changed_time_ calculated: " << min_arm_mode_change_time_ << " s for err: " << err << std::endl;
          }
          currentArmTargetTrajectoriesWithAllJoints_ = interpolateArmTarget(initTime, current_arm_state_full, new_arm_state_full, arm_move_spd_);
          currentArmTargetTrajectories_.timeTrajectory = currentArmTargetTrajectoriesWithAllJoints_.timeTrajectory;
          armFullDofTargetTrajectories_.setBuffer(currentArmTargetTrajectoriesWithAllJoints_);
          auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(currentArmTargetTrajectoriesWithAllJoints_);
          {
            std::lock_guard<std::mutex> lock(armTargetCommanded_mtx_);
            armTargetCommandedPublisher_.publish(mpcTargetTrajectoriesMsg);
          }
        }
        std::cout << "[mpc] arm control mode changing from "<< currentArmControlMode_ << " to " << newArmControlMode_ << ", diff: " << err << std::endl;
        
        bool min_time_met = (arm_mode_change_start_time_ >= 0.0 && 
                             (initTime - arm_mode_change_start_time_) >= min_arm_mode_change_time_);
        // 检查是否满足模式切换完成条件：误差阈值和最小切换时间
        if (err < 2.20 && min_time_met || err < 0.2) 
        {
          currentArmTargetTrajectories_ = newArmTargetTrajectories;
          isArmControlModeChanged_ = false;
          currentArmControlMode_ = newArmControlMode_;
          arm_mode_change_start_time_ = -1.0;  // 重置时间戳
        }
        else if (err < 2.20 && !min_time_met)
        {
          std::cout << "[mpc] arm control mode error threshold met (err=" << err 
                    << "), but waiting for minimum transition time (elapsed: " 
                    << (arm_mode_change_start_time_ >= 0.0 ? (initTime - arm_mode_change_start_time_) : 0.0) 
                    << "s, required: " << min_arm_mode_change_time_ << "s)" << std::endl;
        }
        
      }
      else
      {
        std::cout << "[mpc] arm control mode changed, but target not updated, requested newArmControlMode_:" << newArmControlMode_ << std::endl;
      }
    }
    else
    {
      getDesiredArmTargetTrajectories(currentArmTargetTrajectories_, currentArmControlMode_);
    }

    targetTrajectories.fillTargetTrajectories(currentArmTargetTrajectories_, 12 + feetJointNums_ + waistNums_);
  }
  vector_t arm_control_mode_vec(2);
  arm_control_mode_vec << currentArmControlMode_, newArmControlMode_;
  ros_logger_->publishVector("/humanoid/mpc/arm_control_mode", arm_control_mode_vec);
  // ros_logger_->publishValue("/humanoid/mpc/armControlMode", currentArmControlMode_);
  // ros_logger_->publishValue("/humanoid/mpc/newArmControlMode", newArmControlMode_);
  //waist trajectory
  if(waistNums_ > 0)
  {
    if (waistTargetUpdated_)
    {
      auto waist_target_state = joyWaist_;
      TargetTrajectories waist_traj;
      auto waist_init_state = initState.segment(12 + feetJointNums_, waistNums_);
      waist_traj.timeTrajectory.push_back(initTime); // 腰部插值起点
      waist_traj.stateTrajectory.push_back(waist_init_state);
      waist_traj.inputTrajectory.push_back(vector_t::Zero(waistNums_));

      double dul_time = std::max(0.01, (waist_target_state - waist_init_state).norm() / waist_move_spd_);
      waist_traj.timeTrajectory.push_back(initTime + dul_time); // 腰部插值终点
      waist_traj.stateTrajectory.push_back(waist_target_state);
      waist_traj.inputTrajectory.push_back(vector_t::Zero(waistNums_));

      currentWaistTargetTrajectories_ = waist_traj;
      waistTargetUpdated_ = false;
    }
    targetTrajectories.fillTargetTrajectories(currentWaistTargetTrajectories_, 12 + feetJointNums_);
  }

  // Update isArmExecuting status
  isArmExecuting_ = isArmControlModeChanged_ || (currentArmControlMode_ == ArmControlMode::EXTERN_CONTROL && !currentArmTargetTrajectories_.empty() && initTime < currentArmTargetTrajectories_.timeTrajectory.back());
  std_msgs::Bool is_arm_executing_msg;
  is_arm_executing_msg.data = isArmExecuting_;
  isArmExecutingPublisher_.publish(is_arm_executing_msg);

  // update terrain height
  // auto desire_state_init = targetTrajectories.getDesiredState(initTime);
  // feet_array_t<vector3_t> feet_pos_desire_init = inverseKinematics_.computeFootPos(desire_state_init);
  // double terrainHeightDesire = this->calTerrainHeight(contact_flags, feet_pos_desire_init);
  // double heightDiff = terrainHeightDesire - terrainHeight_;
  // bool isMultiTraj = (targetTrajectories.timeTrajectory.size() > 1);
  // auto is_walking_on_plane = [&feet_pos_desire_init]()
  // {
  //   double z_diff = 0.0;
  //   int num_feet = feet_pos_desire_init.size()/2;
  //   for (int i = 0; i < num_feet; i++)
  //   {
  //     z_diff += feet_pos_desire_init[i][2] - feet_pos_desire_init[i+num_feet][2];
  //   }
  //   z_diff /= num_feet;
  //   return std::abs(z_diff) < 0.01;
  // };
    // std::cout << "heightDiff:" << heightDiff << std::endl;
    // std::cout << "terrainHeight_:"<< terrainHeight_ << std::endl;
    // std::cout << "terrainHeightDesire:"<< terrainHeightDesire << std::endl;
  // if (current_mode!= ModeNumber::SS || is_walking_on_plane())//只有单足或者双足支撑是在平面上的情况才进行高度补偿
  bool cali_target_height = false;
  if (enable_slope_planning_)
  {
    cali_target_height = true;
  }

  // if (modeSchedule.existValidFootPose(initTime))
  // {
  //   // if (!is_walking_on_plane() || current_mode != ModeNumber::SS)
      // cali_target_height = false;
  //     terrainHeightPrev_ = terrainHeight_;

  // }
  if (cali_target_height)
  {
    double terrainHeightDiff = terrainHeight_ - terrainHeightPrev_;
    double adjust_limit = 0.005;
    if (std::abs(terrainHeightDiff) > adjust_limit)
    {
      std::cout << "terrainHeightDiff too large: " << terrainHeightDiff << std::endl;
      terrainHeightDiff = std::min(std::max(terrainHeightDiff, -adjust_limit), adjust_limit);
    }
    terrainHeightPrev_ +=  terrainHeightDiff;
    if (ros_logger_ != nullptr)
    {
      ros_logger_->publishValue("/humanoid/mpc/heightDiff", terrainHeightDiff);
    }
    for (int i = 0; i < targetTrajectories.timeTrajectory.size(); i++)
    {
      // if (isMultiTraj && i == 0)
      //   continue;
      targetTrajectories.stateTrajectory[i][8] += terrainHeightDiff;
      // targetTrajectories.stateTrajectory[i][8] -= heightDiff;

    }
  }else
  {
    terrainHeightPrev_ = terrainHeight_;
  }

  targetTrajectories.trimTargetTrajectories(initTime - timeHorizon);


  // for (int i = 0; i < targetTrajectories.stateTrajectory.size(); i++)
  // {
  //   auto& traj_input = targetTrajectories.inputTrajectory[i];
  //   traj_input.segment(info_.numThreeDofContacts*3, 12) = armWrench;
  // }
  // std::cout << "Target Trajectories:\n";
  // for (int i = 0; i < targetTrajectories.timeTrajectory.size(); i++)
  // {
  //   std::cout << "time[" << i << "]: " << targetTrajectories.timeTrajectory[i] << ", state: " << targetTrajectories.stateTrajectory[i].segment<4>(6).transpose() << std::endl;
  // }
  ros_logger_->publishValue("/humanoid/mpc/isCmdPoseCached", isCmdPoseCached);
  ros_logger_->publishValue("/humanoid/mpc/PoseCmdUpdate", PoseCmdUpdated_);
  ros_logger_->publishValue("/humanoid/mpc/PoseWorldCmdUpdated", PoseWorldCmdUpdated_);
  ros_logger_->publishValue("/humanoid/mpc/ismdPoseInWorldFrameCached", ismdPoseInWorldFrameCached_);
  ros_logger_->publishValue("/humanoid/mpc/velCmdUpdate", velCmdUpdated_);
  ros_logger_->publishVector("/humanoid/mpc/targetState", targetTrajectories.getDesiredState(initTime));
}

TargetTrajectories SwitchedModelReferenceManager::interpolateArmTarget(scalar_t startTime, const vector_t& currentArmState, const vector_t& newDesiredArmState, scalar_t maxSpeed)
{
     TargetTrajectories plannedTrajectories;

    // 计算状态差
    vector_t deltaState = newDesiredArmState - currentArmState;
    scalar_t totalDistance = deltaState.norm();

    scalar_t requiredTime = totalDistance / maxSpeed;

    scalar_t endTime = startTime + requiredTime;

    plannedTrajectories.timeTrajectory.push_back(startTime);
    plannedTrajectories.stateTrajectory.push_back(currentArmState);

    plannedTrajectories.timeTrajectory.push_back(endTime);
    plannedTrajectories.stateTrajectory.push_back(newDesiredArmState);
    
    plannedTrajectories.inputTrajectory = vector_array_t(2, vector_t::Zero(info_.inputDim));

    return plannedTrajectories;
}

bool SwitchedModelReferenceManager::checkAndApplyCommandLine(scalar_t initTime, scalar_t finalTime, const vector_t& initState, vector_t& cmdVel)
{
  vector_t cmd_vec = vector_t::Zero(3);
  cmd_vec << cmdVel[0], cmdVel[1], cmdVel[3];

  if(is_roban_version_ && cmdVel[3] != 0.0)
  {
    if(std::fabs(cmdVel[0]) >= std::fabs(cmdVel[1])) cmdVel[1] = 0.0;
    if(std::fabs(cmdVel[1]) > std::fabs(cmdVel[0]))  cmdVel[0] = 0.0;
  }
  auto current_gait = gaitSchedulePtr_->getGaitName(initTime);
  auto next_gait = gaitSchedulePtr_->getLastGaitName();
  if (current_gait != "stance" && next_gait != "stance")
  {
    cmdHeight_ = 0.0;
    return true;
  }
  cmdVel.setZero();
  return true;

}

bool SwitchedModelReferenceManager::checkCmdPoseAndApplyCommandLine(scalar_t initTime, scalar_t finalTime, const vector_t& initState, vector_t& cmdPose)
{
  auto current_gait = gaitSchedulePtr_->getGaitName(initTime);
  auto next_gait = gaitSchedulePtr_->getLastGaitName();
  if (current_gait != "stance" && next_gait != "stance")
  {
    cmdHeight_ = 0.0;
    cmdPitch_ = initTargetState_[10];
    std::cout << "[CMD POS] : not stance , check get cmdPose" << std::endl;
    if (isCmdPoseCached) 
    {
      cmdPose = tempCmdPose_;
      tempCmdPose_.setZero();
      isCmdPoseCached = false;
      std::cout << "[CMD POS] : not stance , Using tmp cmdPose" << std::endl;

    }
    return true;
  }
  else if (cmdPose.norm() <= cmd_threshold && !vrWaistControlEnabled_) // cmdPose，机器人不移动的情况，站立即可执行，不更新tempCmdPose_
  {
    isCmdPoseCached = false;
    cmdPose.setZero();
    // std::cout << "[CMD POS] : stance, cmdPose is samll , tmp not used" << std::endl;
    return true;

  }
  else if(vrWaistControlEnabled_)
  {
    // std::cout << "[CMD POS] : stance, vrWaistControlEnabled_, Save cmdPose as tmpCmdPose" << std::endl;
    tempCmdPose_ = cmdPose;
    isCmdPoseCached = false;
    return true;
  }
  else
  {
    // std::cout << "[CMD POS] : stance, Save cmdPose as tmpCmdPose" << std::endl;

    tempCmdPose_ = cmdPose;
    isCmdPoseCached = true;
    cmdPose.setZero();
    return true;
  } 
}

TargetTrajectories SwitchedModelReferenceManager::generateTargetwithVelcmd(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                                           TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule, const vector_t &cmdVel)
{

  const auto timeHorizon = finalTime - initTime;
  const auto endTrajTime = finalTime + timeHorizon;
  // generate time sequence for the trajectory with velocity

  auto init_target_state = targetTrajectories.getDesiredState(initTime);

  vector_t cmdVelRot = cmdVel;
  const Eigen::Matrix<scalar_t, 3, 1> targetZyx = {init_target_state[9], 0, 0};
  cmdVelRot.head(3) = getRotationMatrixFromZyxEulerAngles(targetZyx) * cmdVel.head(3);

  // slope planning
  if (enable_slope_planning_)
  {
    auto feet_normal_vectors = swingTrajectoryPtr_->getFeetNormalVectors();
    vector3_t lf_normal_vector = feet_normal_vectors.first;
    vector3_t rf_normal_vector = feet_normal_vectors.second;
    vector3_t slope_vector = lf_normal_vector + rf_normal_vector;
    
    vector3_t z_axis = slope_vector.normalized();
    // 构建xy平面的投影矩阵 P = I - nn^T，其中n是法向量
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - z_axis * z_axis.transpose();
    
    // 只投影xy平面的速度，保持z方向不变
    vector3_t projected_vel = P * cmdVelRot.head(3);
    cmdVelRot[0] = projected_vel[0];
    cmdVelRot[1] = projected_vel[1];

    ros_logger_->publishVector("/humanoid/mpc/projected_vel", projected_vel);

  }
  // slope planning

  // 计算当前目标高度
  scalar_t currentTargetHeight = terrainHeight_ + initTargetState_[8] + cmdHeight_;
  // 计算相对高度位移：当前目标高度与当前状态高度的差值
  scalar_t heightDisplacement = currentTargetHeight - init_target_state[8];
  
  vector6_t torsoDisplacement = (vector6_t() << 0, 0, heightDisplacement, 0, 0, 0).finished();
  vector_t final_target_state_torso;
  double torso_max_time;
  // velocity_scale: 所有维度使用0.3（仅高度变化的情况）
  vector6_t velocity_scale = (vector6_t() << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3).finished();
  generateTargetwithTorsoMove(initTime, initTargetState_, torsoDisplacement, targetTrajectories, final_target_state_torso, torso_max_time, velocity_scale);
  
  // 更新上一次目标高度
  if(torso_max_time > timeHorizon)
  {
    std::cout << "torso_max_time is: " << torso_max_time << ", timeHorizon is: " << timeHorizon << std::endl;
    ROS_WARN("torso_max_time is greater than timeHorizon, set torso_max_time to timeHorizon");
    torso_max_time = timeHorizon;
  }
  scalar_array_t timeTrajectoryWithVel{initTime, initTime + torso_max_time, finalTime, endTrajTime};

  auto final_target_state = targetTrajectories.getDesiredState(finalTime);
  final_target_state.segment(6, 6) = init_target_state.segment(6, 6) + cmdVelRot.head(6) * timeHorizon * 1.0;
  final_target_state[8] = terrainHeight_ + initTargetState_[8] + cmdHeight_;

  auto end_traj_target_state = final_target_state;
  end_traj_target_state.segment(6, 6) = init_target_state.segment(6, 6) + cmdVelRot.head(6) * timeHorizon * 2.0 * 1.0;
  end_traj_target_state[8] = terrainHeight_ + initTargetState_[8] + cmdHeight_;

  vector_array_t stateTrajectoryWithVel{init_target_state, final_target_state_torso, final_target_state, end_traj_target_state};
  vector_array_t inputTrajectoryWithVel(timeTrajectoryWithVel.size(), vector_t::Zero(info_.inputDim));

  return {timeTrajectoryWithVel, stateTrajectoryWithVel, inputTrajectoryWithVel};
}


TargetTrajectories SwitchedModelReferenceManager::generateTargetwithPoscmd(scalar_t initTime, const vector_t &initState,TargetTrajectories &targetTrajectories, 
                                                                           ModeSchedule &modeSchedule, const vector_t &cmdPos)
{

  // generate time sequence for the trajectory with target position

  auto init_target_state = targetTrajectories.getDesiredState(initTime);

  // std::cout << "TEST generateTargetwithPoscmd  cmdPos : " << cmdPos << std::endl;

  vector_t cmdPosRot = cmdPos;
  const Eigen::Matrix<scalar_t, 3, 1> targetZyx = {init_target_state[9], 0, 0};
  cmdPosRot.head(3) = getRotationMatrixFromZyxEulerAngles(targetZyx) * cmdPos.head(3);


  // 应用高度和pitch平滑过渡逻辑
  scalar_t effectiveCmdHeight = cmdHeight_;
  scalar_t effectiveCmdPitch = cmdPitch_;

  if (heightSmoothTransitionActive_)
  {
    scalar_t currentTime = ros::Time::now().toSec();
    scalar_t elapsedTime = currentTime - heightTransitionStartTime_;

    if (elapsedTime >= heightTransitionDuration_) {
      // 过渡完成
      heightSmoothTransitionActive_ = false;
      effectiveCmdHeight = heightAfterTransition_;
      effectiveCmdPitch = pitchAfterTransition_;
    } else {
      // 使用三次多项式插值进行平滑过渡 (S曲线)
      scalar_t t = elapsedTime / heightTransitionDuration_;  // 归一化时间 [0, 1]
      // 使用 smoothstep 函数: 3t² - 2t³
      scalar_t alpha = 3.0 * t * t - 2.0 * t * t * t;

      // 对高度和pitch同时进行插值
      effectiveCmdHeight = heightBeforeTransition_ + alpha * (heightAfterTransition_ - heightBeforeTransition_);
      effectiveCmdPitch = pitchBeforeTransition_ + alpha * (pitchAfterTransition_ - pitchBeforeTransition_);

    }
  }

  // 计算当前目标高度和 pitch
  scalar_t currentTargetHeight = terrainHeight_ + initTargetState_[8] + effectiveCmdHeight;
  scalar_t currentTargetPitch = effectiveCmdPitch;
  
  // 计算相对位移：目标值与当前状态的差值
  scalar_t xDisplacement = cmdPosRot[0];
  scalar_t yDisplacement = cmdPosRot[1];
  scalar_t heightDisplacement = currentTargetHeight - init_target_state[8];
  scalar_t pitchDisplacement = currentTargetPitch - init_target_state[10];
  scalar_t yawDisplacement = cmdPosRot[3];
  
  vector6_t torsoDisplacement = (vector6_t() << xDisplacement, yDisplacement, heightDisplacement, 0, pitchDisplacement, yawDisplacement).finished();
  vector_t final_target_state_torso;
  double torso_max_time;

  generateTargetwithTorsoMove(initTime, initTargetState_, torsoDisplacement, targetTrajectories, final_target_state_torso, torso_max_time, torso_velocity_scale_);

  // calculate the final time 
  auto xMoveDesiredTime = abs(cmdPos[0]) / c_relative_base_limit_[0];
  auto yMoveDesiredTime = abs(cmdPos[1]) / c_relative_base_limit_[1];
  auto heaightMoveDesiredTime = abs(heightDisplacement) / c_relative_base_limit_[2];

  auto yawMoveDesiredTime = abs(cmdPos[3]) / c_relative_base_limit_[3];

  auto pitchMoveDesiredTime = abs(pitchDisplacement) / c_relative_base_limit_[4];

  auto timeHorizon = std::max({xMoveDesiredTime, yMoveDesiredTime, heaightMoveDesiredTime, yawMoveDesiredTime, pitchMoveDesiredTime});

  scalar_t finalTime = initTime + timeHorizon;
  if(torso_max_time > finalTime - initTime)
  {
    std::cout << "torso_max_time is: " << torso_max_time << ", (finalTime - initTime) is: " << finalTime - initTime << std::endl;
    ROS_WARN("torso_max_time is greater than (finalTime - initTime), set torso_max_time to (finalTime - initTime)");
    torso_max_time = finalTime - initTime;
  }
  scalar_array_t timeTrajectoryWithPos{initTime, initTime + torso_max_time, finalTime};

  auto final_target_state = targetTrajectories.getDesiredState(finalTime);
  final_target_state.segment(6, 6) = init_target_state.segment(6, 6) + cmdPosRot.head(6);
  final_target_state[8] = terrainHeight_ + initTargetState_[8] + effectiveCmdHeight;
  final_target_state[10] = std::max(0.0, std::min(60*M_PI/180.0, effectiveCmdPitch));
  //腰部控制开启时，更新腰部的yaw，根据cmdPos[3]和当前实际腰部进行更新
  if(vrWaistControlEnabled_)
  {
    final_target_state[9] = currentTorsoYaw_ + cmdPosRot[3]; 
    final_target_state[11] = currentTorsoRoll_ + cmdPosRot[5];
  }
  
  vector_array_t stateTrajectoryWithPos{init_target_state, final_target_state_torso, final_target_state};
  vector_array_t inputTrajectoryWithPos(timeTrajectoryWithPos.size(), vector_t::Zero(info_.inputDim));

  return {timeTrajectoryWithPos, stateTrajectoryWithPos, inputTrajectoryWithPos};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories SwitchedModelReferenceManager::generateTargetwithPoscmdInCurrentPose(scalar_t initTime, const vector_t &initState, TargetTrajectories &targetTrajectories, ModeSchedule &modeSchedule, const vector_t &cmdPos)
{
  auto init_target_state = targetTrajectories.getDesiredState(initTime);

  vector_t cmdPosRot = cmdPos;
  const Eigen::Matrix<scalar_t, 3, 1> targetZyx = {initState[9], 0, 0};
  cmdPosRot.head(3) = getRotationMatrixFromZyxEulerAngles(targetZyx) * cmdPos.head(3);

  // calculate the final time 
  auto xMoveDesiredTime = abs(cmdPos[0]) / c_relative_base_limit_[0];
  auto yMoveDesiredTime = abs(cmdPos[1]) / c_relative_base_limit_[1];
  auto heaightMoveDesiredTime = abs(cmdHeight_) / c_relative_base_limit_[2];

  auto yawMoveDesiredTime = abs(cmdPos[3]) / c_relative_base_limit_[3];

  auto pitchMoveDesiredTime = abs(cmdPitch_-initState[10]) / c_relative_base_limit_[4];

  auto timeHorizon = std::max({xMoveDesiredTime, yMoveDesiredTime, heaightMoveDesiredTime, yawMoveDesiredTime, pitchMoveDesiredTime});

  scalar_t finalTime = initTime + timeHorizon;

  scalar_array_t timeTrajectoryWithPos{initTime, finalTime};

  auto final_target_state = targetTrajectories.getDesiredState(finalTime);
  final_target_state.segment(6, 6) = initState.segment(6, 6) + cmdPosRot.head(6);
  final_target_state[8] = terrainHeight_ + initTargetState_[8] + cmdHeight_;
  final_target_state[10] = std::max(0.0, std::min(60*M_PI/180.0, cmdPitch_));
  final_target_state[11] = info_.qPinocchioNominal[5];

  
  vector_array_t stateTrajectoryWithPos{init_target_state, final_target_state};
  vector_array_t inputTrajectoryWithPos(timeTrajectoryWithPos.size(), vector_t::Zero(info_.inputDim));

  return {timeTrajectoryWithPos, stateTrajectoryWithPos, inputTrajectoryWithPos};
}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TargetTrajectories SwitchedModelReferenceManager::generateTargetwithModeSchedule(scalar_t initTime, scalar_t finalTime,
                                                    const vector_t &initState, 
                                                    const TargetTrajectories &targetTrajectories,
                                                    const ModeSchedule &modeSchedule)
  {
    const auto timeHorizon = finalTime - initTime;
    const auto endTrajTime = finalTime + timeHorizon;

    const auto &eventTimes = modeSchedule.eventTimes;
    const auto &modeSequence = modeSchedule.modeSequence;
    const auto &enableFootSequence = modeSchedule.enableFootSequence;
    const auto &isLastCommandSequence = modeSchedule.isLastCommandSequence;
    const auto &torsoPoseSequence = modeSchedule.torsoPoseSequence;

    
    const size_t index_start = std::lower_bound(eventTimes.begin(), eventTimes.end(), initTime) - eventTimes.begin();
    size_t index_target = 0;

    scalar_t start_time = initTime;
    for(int i=index_start; i<eventTimes.size(); i++)
    {
      if(enableFootSequence[i] && !isLastCommandSequence[i])
      {
        start_time = eventTimes[i-1];
        break;
      }
    }
    scalar_array_t timeTrajectoryWithModeSchedule{start_time};
    const auto &init_target_state = targetTrajectories.getDesiredState(start_time);
    vector_array_t stateTrajectoryWithModeSchedule{init_target_state};
    const Eigen::Matrix<scalar_t, 3, 1> currentZyx = {init_target_state(9), 0, 0};
    const auto Rot = getRotationMatrixFromZyxEulerAngles(currentZyx);
    auto generateTargetState = [&](size_t index_target) -> vector_t
    {
      // 从6D向量中提取前4个元素(x,y,z,yaw)，保持兼容性
      Eigen::Vector4d torso_pose;
      torso_pose.head<4>() = torsoPoseSequence[index_target].head<4>();
      auto final_target_state = targetTrajectories.getDesiredState(start_time);
      final_target_state.segment<3>(6) = init_target_state.segment<3>(6) + Rot*torso_pose.head<3>();
      final_target_state(9) = init_target_state(9) + torso_pose(3);
      final_target_state(10) = init_target_state(10) + torsoPoseSequence[index_target](4);
      final_target_state(11) = init_target_state(11) + torsoPoseSequence[index_target](5);
      return std::move(final_target_state);
    };
    auto generateTargetState_lasttime = [&](size_t index_target) -> vector_t
    {
      Eigen::Vector4d torso_pose = torsoPoseSequence[index_target].head<4>();
      auto final_target_state = last_init_target_state;
      final_target_state.segment<3>(6) +=  Rot*torso_pose.head<3>();
      final_target_state(9) += torso_pose(3);
      return std::move(final_target_state);
    };
    for(int i=index_start; i<eventTimes.size(); i++)
    {
      if(enableFootSequence[i])
      {
        if(isLastCommandSequence[i]){
          index_target = i;
          double delta_time = eventTimes[index_target] - start_time;
          timeTrajectoryWithModeSchedule.push_back(start_time + delta_time);
          auto final_target_state = generateTargetState_lasttime(index_target);
          stateTrajectoryWithModeSchedule.push_back(final_target_state);
          // std::cout << "delta time: " << delta_time << std::endl;
          std::cout << "torso target_state: " << final_target_state.segment<4>(6).transpose() << std::endl;
        }else{
          index_target = i;
          double delta_time = eventTimes[index_target] - start_time;
          timeTrajectoryWithModeSchedule.push_back(start_time + delta_time);
          auto final_target_state = generateTargetState(index_target);
          stateTrajectoryWithModeSchedule.push_back(final_target_state);
          // std::cout << "delta time: " << delta_time << std::endl;
          std::cout << "torso target_state: " << final_target_state.segment<4>(6).transpose() << std::endl;
        }
      }
    }
    if (index_target == 0)
    {
      std::cout << "generateTargetwithModeSchedule: No target found." << std::endl;
      return targetTrajectories;
    }
    vector_array_t inputTrajectoryWithModeSchedule(timeTrajectoryWithModeSchedule.size(), vector_t::Zero(info_.inputDim));
    last_init_target_state = init_target_state;

    return {timeTrajectoryWithModeSchedule, stateTrajectoryWithModeSchedule, inputTrajectoryWithModeSchedule};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TargetTrajectories SwitchedModelReferenceManager::generateTargetwithModeScheduleWorld(scalar_t initTime, scalar_t finalTime,
                                                                                        const vector_t &initState,
                                                                                        const TargetTrajectories &targetTrajectories,
                                                                                        const ModeSchedule &modeSchedule)
  {
    const auto timeHorizon = finalTime - initTime;
    const auto endTrajTime = finalTime + timeHorizon;

    const auto &eventTimes = modeSchedule.eventTimes;
    const auto &modeSequence = modeSchedule.modeSequence;
    const auto &enableFootSequence = modeSchedule.enableFootSequence;
    const auto &isLastCommandSequence = modeSchedule.isLastCommandSequence;
    const auto &torsoPoseSequence = modeSchedule.torsoPoseSequence;

    const size_t index_start = std::lower_bound(eventTimes.begin(), eventTimes.end(), initTime) - eventTimes.begin();
    size_t index_target = 0;

    scalar_t start_time = initTime;

    for (int i = index_start; i < eventTimes.size(); i++)
    {
      if (enableFootSequence[i])
      {
        start_time = eventTimes[i - 2];
        break;
      }
    }
    scalar_array_t timeTrajectoryWithModeSchedule{start_time};
    const auto &init_target_state = targetTrajectories.getDesiredState(start_time);
    vector_array_t stateTrajectoryWithModeSchedule{init_target_state};
    const Eigen::Matrix<scalar_t, 3, 1> currentZyx = {init_target_state(9), 0, 0};
    const auto Rot = getRotationMatrixFromZyxEulerAngles(currentZyx);
    auto generateTargetState = [&](size_t index_target) -> vector_t
    {
      // 从6D向量中提取前4个元素(x,y,z,yaw)，保持兼容性
      Eigen::Vector4d torso_pose;
      torso_pose.head<4>() = torsoPoseSequence[index_target].head<4>();
      auto final_target_state = targetTrajectories.getDesiredState(start_time);
      final_target_state.segment<3>(6) = torso_pose.head<3>();
      final_target_state(8) += initTargetState_[8]; // 加上初始站立时的躯干高度
      final_target_state(9) = torso_pose(3);
      final_target_state(10) = torsoPoseSequence[index_target](4);
      final_target_state(11) = torsoPoseSequence[index_target](5);
      return std::move(final_target_state);
    };
    auto generateTargetState_lasttime = [&](size_t index_target) -> vector_t
    {
      Eigen::Vector4d torso_pose = torsoPoseSequence[index_target].head<4>();
      auto final_target_state = last_init_target_state;
      final_target_state.segment<3>(6) = torso_pose.head<3>();
      final_target_state(8) += initTargetState_[8]; // 加上初始站立时的躯干高度
      final_target_state(9) = torso_pose(3);
      final_target_state(10) = torsoPoseSequence[index_target](4);
      final_target_state(11) = torsoPoseSequence[index_target](5);
      return std::move(final_target_state);
    };
    for (int i = index_start; i < eventTimes.size(); i++)
    {
      if (enableFootSequence[i])
      {
        if (isLastCommandSequence[i])
        {
          index_target = i;
          double delta_time = eventTimes[index_target] - start_time;
          timeTrajectoryWithModeSchedule.push_back(start_time + delta_time);
          auto final_target_state = generateTargetState_lasttime(index_target);
          stateTrajectoryWithModeSchedule.push_back(final_target_state);
          // std::cout << "delta time: " << delta_time << std::endl;
          std::cout << "torso target_state: " << final_target_state.segment<4>(6).transpose() << std::endl;
        }
        else
        {
          index_target = i;
          double delta_time = eventTimes[index_target] - start_time;
          timeTrajectoryWithModeSchedule.push_back(start_time + delta_time);
          auto final_target_state = generateTargetState(index_target);
          stateTrajectoryWithModeSchedule.push_back(final_target_state);
          // std::cout << "delta time: " << delta_time << std::endl;
          std::cout << "torso target_state: " << final_target_state.segment<4>(6).transpose() << std::endl;
        }
      }
    }
    if (index_target == 0)
    {
      std::cout << "generateTargetwithModeScheduleWorld: No target found." << std::endl;
      return targetTrajectories;
    }
    vector_array_t inputTrajectoryWithModeSchedule(timeTrajectoryWithModeSchedule.size(), vector_t::Zero(info_.inputDim));
    last_init_target_state = init_target_state;

    return {timeTrajectoryWithModeSchedule, stateTrajectoryWithModeSchedule, inputTrajectoryWithModeSchedule};
  }

  TargetTrajectories SwitchedModelReferenceManager::generateTargetAsCurrent(scalar_t initTime, scalar_t finalTime, const vector_t &initState)
  {
    scalar_array_t timeTrajectoryWithModeSchedule{initTime, finalTime};
    vector_array_t stateTrajectoryWithModeSchedule{initState, initState};
    vector_array_t inputTrajectoryWithModeSchedule(timeTrajectoryWithModeSchedule.size(), vector_t::Zero(info_.inputDim));

    return {timeTrajectoryWithModeSchedule, stateTrajectoryWithModeSchedule, inputTrajectoryWithModeSchedule};
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::setSwingHeight(scalar_t toe_height, scalar_t heel_height)
  {
    swingTrajectoryPlannerConfig_ = swingTrajectoryPtr_->getConfig();
    swingTrajectoryPlannerConfig_.toeSwingHeight = toe_height;
    swingTrajectoryPlannerConfig_.heelSwingHeight = heel_height;
    swingTrajectoryPtr_->updateConfig(swingTrajectoryPlannerConfig_);
  }

  /***********/
  // trigger callback function in every observation callback

  void SwitchedModelReferenceManager::observationStateCallback(const vector_t &state)
  {
    vel_norm_ = state.segment(0, 3).norm();
    currentState_ = state;  // 更新当前状态，用于获取torso yaw角
  }

  bool SwitchedModelReferenceManager::stopSingleStepControlCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    update_stop_single_step_ = true;
    return true;
  }

  bool SwitchedModelReferenceManager::loadDynamicQRCallback(kuavo_msgs::ExecuteArmAction::Request &req, kuavo_msgs::ExecuteArmAction::Response &res)
  {
    if (!dynamic_qr_flag_)
    {
      res.success = false;
      res.message = "Dynamic QR is not enabled. Please ensure enable_dynamic_q and enable_dynamic_r are set to true in task.info";
      return true;
    }

    std::string gait_name = req.action_name;

    // Get current time
    scalar_t current_time = ros::Time::now().toSec();

    // Check if all modes are SS for stance gait
    const auto& modeSchedule = gaitSchedulePtr_->getModeSchedule();
    bool all_stance = std::all_of(modeSchedule.modeSequence.begin(),
                            modeSchedule.modeSequence.end(),
                            [](size_t mode) { return mode == ModeNumber::SS; });

    if(gait_name == "stance")
    {
      dynamic_r_set_ = false; 
      setMatrixQByGaitPair(gait_name, current_time);
      setMatrixRByGaitPair(gait_name, current_time, all_stance);
    }
    else
    {
      auto it = dynamic_qr_map_.find(gait_name);
      if (it == dynamic_qr_map_.end())
      {
        res.success = false;
        res.message = "Q and R matrices for gait '" + gait_name + "' not found in dynamic_qr_map. Please ensure Q_dynamic_" + gait_name + " and R_dynamic_" + gait_name + " exist in dynamic_qr.info.";
        return true;
      }
      
      // 从map中获取QR矩阵
      dynamic_Q = it->second.Q;
      dynamic_R = it->second.R;

      dynamic_r_set_ = true;
      setMatrixQByGaitPair("dynamic_qr", current_time);
      setMatrixRByGaitPair("dynamic_qr", current_time, all_stance);
    }
    res.success = true;
    res.message = "Successfully loaded Q and R matrices for gait: " + gait_name;
    return true;
  }

  bool SwitchedModelReferenceManager::singleStepControlCallback(kuavo_msgs::singleStepControl::Request &req, kuavo_msgs::singleStepControl::Response &res)
  {
    const auto& trajectories = req.foot_pose_target_trajectories;
    const int size = trajectories.timeTrajectory.size();
    
    if(size != trajectories.footPoseTrajectory.size())
    {
        res.success = false;
        res.message = "Inconsistent sizes in request data";
        ROS_WARN_STREAM(res.message);
        return true;
    }

    footPoseSchedule_.eventTimes.clear();
    footPoseSchedule_.footIndices.clear();
    footPoseSchedule_.footPoseSequence.clear();
    footPoseSchedule_.torsoPoseSequence.clear();

    footPoseSchedule_.eventTimes.reserve(size * 2);
    footPoseSchedule_.footIndices.reserve(size * 2);
    footPoseSchedule_.footPoseSequence.reserve(size * 2);
    footPoseSchedule_.torsoPoseSequence.reserve(size * 2);

    double prevTime = 0.0;
    boost::array<double, 4> prevTorsoPose = {0.0, 0.0, 0.0, 0.0};

    // Define validation function
    auto validateTorsoPoseChange = [](const boost::array<double, 4>& prevPose, const boost::array<double, 4>& currentPose) -> std::string {
        double dx = std::abs(currentPose[0] - prevPose[0]);
        double dy = std::abs(currentPose[1] - prevPose[1]);
        double dz = std::abs(currentPose[2] - prevPose[2]);
        double dyaw = std::abs(currentPose[3] - prevPose[3]);

        if (dx > X_MAX_SINGLE_STEP_SIZE) {
            return "X change " + std::to_string(dx) + " exceeds maximum limit of " + std::to_string(X_MAX_SINGLE_STEP_SIZE);
        }
        if (dy > Y_MAX_SINGLE_STEP_SIZE) {
            return "Y change " + std::to_string(dy) + " exceeds maximum limit of " + std::to_string(Y_MAX_SINGLE_STEP_SIZE);
        }
        if (dz > Z_MAX_SINGLE_STEP_SIZE) {
            return "Z change " + std::to_string(dz) + " exceeds maximum limit of " + std::to_string(Z_MAX_SINGLE_STEP_SIZE);
        }
        if (dyaw > YAW_MAX_SINGLE_STEP_SIZE) {
            return "Yaw change " + std::to_string(dyaw) + " exceeds maximum limit of " + std::to_string(YAW_MAX_SINGLE_STEP_SIZE);
        }
        return "";  // Empty string means validation passed
    };

    for(int i = 0; i < size; i++)
    {
        const auto& torso_pose = trajectories.footPoseTrajectory[i].torsoPose;
        
        if (i > 0)  // Skip validation for the first pose
        {
            std::string validationResult = validateTorsoPoseChange(prevTorsoPose, torso_pose);
            if(!validationResult.empty())
            {
                res.success = false;
                res.message = "Invalid torso pose change at index " + std::to_string(i) + ": " + validationResult;
                ROS_ERROR_STREAM(res.message);
                return true;
            }
        }

        Eigen::Vector3d torso_pos(torso_pose[0], torso_pose[1], torso_pose[2]);
        double torso_yaw = torso_pose[3];

        Eigen::Vector3d l_foot, r_foot;
        std::tie(l_foot, r_foot) = generate_steps(torso_pos, torso_yaw);

        double current_time = trajectories.timeTrajectory[i];
        double mid_time = (prevTime + current_time) / 2.0;

        // Add mid-point (Left foot)
        footPoseSchedule_.eventTimes.push_back(mid_time);
        footPoseSchedule_.footIndices.push_back(FootIdx::Left);
        vector6_t left_foot_pose;
        left_foot_pose << l_foot.x(), l_foot.y(), l_foot.z(), torso_yaw, 0.0, 0.0;
        footPoseSchedule_.footPoseSequence.emplace_back(left_foot_pose);
        vector6_t left_torso_pose;
        left_torso_pose << torso_pos.x(), torso_pos.y(), torso_pos.z(), torso_yaw, 0.0, 0.0;
        footPoseSchedule_.torsoPoseSequence.emplace_back(left_torso_pose);

        // Add end-point (Right foot)
        footPoseSchedule_.eventTimes.push_back(current_time);
        footPoseSchedule_.footIndices.push_back(FootIdx::Right);
        vector6_t right_foot_pose;
        right_foot_pose << r_foot.x(), r_foot.y(), r_foot.z(), torso_yaw, 0.0, 0.0;
        footPoseSchedule_.footPoseSequence.emplace_back(right_foot_pose);
        vector6_t right_torso_pose;
        right_torso_pose << torso_pos.x(), torso_pos.y(), torso_pos.z(), torso_yaw, 0.0, 0.0;
        footPoseSchedule_.torsoPoseSequence.emplace_back(right_torso_pose);

        prevTime = current_time;
        prevTorsoPose = torso_pose;
    }

    for(size_t i = 0; i < footPoseSchedule_.eventTimes.size(); i++)
    {
      ROS_INFO_STREAM((footPoseSchedule_.footIndices[i]==FootIdx::Left?"L":"R")
                  << ", time: " << footPoseSchedule_.eventTimes[i] << ", footPose: "
                  << footPoseSchedule_.footPoseSequence[i].transpose() << ", torsoPose: "
                  << footPoseSchedule_.torsoPoseSequence[i].transpose());
    }

    update_foot_trajectory_ = true;
    res.success = true;
    res.message = "Foot pose target trajectories set successfully";
    return true;
  }

  // Helper function to generate foot steps (should be implemented in the class)
  std::pair<Eigen::Vector3d, Eigen::Vector3d> SwitchedModelReferenceManager::generate_steps(const Eigen::Vector3d& torso_pos, const double torso_yaw, const double foot_bias)
  {
    Eigen::Vector3d l_foot_bias(0, foot_bias, -torso_pos.z());
    Eigen::Vector3d r_foot_bias(0, -foot_bias, -torso_pos.z());
    
    Eigen::Matrix3d R_z;
    R_z << cos(torso_yaw), -sin(torso_yaw), 0,
          sin(torso_yaw), cos(torso_yaw), 0,
          0, 0, 1;
    
    Eigen::Vector3d l_foot = torso_pos + R_z * l_foot_bias;
    Eigen::Vector3d r_foot = torso_pos + R_z * r_foot_bias;

    return {l_foot, r_foot};
  }

  bool SwitchedModelReferenceManager::footPoseTargetTrajectoriesSrvCallback(kuavo_msgs::footPoseTargetTrajectoriesSrv::Request &req, kuavo_msgs::footPoseTargetTrajectoriesSrv::Response &res)
  {
    // 将4D请求转换为6D请求
    kuavo_msgs::footPose6DTargetTrajectoriesSrv::Request req6d;
    
    // 复制基本字段
    req6d.foot_pose_target_trajectories.timeTrajectory = req.foot_pose_target_trajectories.timeTrajectory;
    req6d.foot_pose_target_trajectories.footIndexTrajectory = req.foot_pose_target_trajectories.footIndexTrajectory;
    req6d.foot_pose_target_trajectories.swingHeightTrajectory = req.foot_pose_target_trajectories.swingHeightTrajectory;
    
    // 转换足部姿态从4D到6D
    req6d.foot_pose_target_trajectories.footPoseTrajectory.resize(req.foot_pose_target_trajectories.footPoseTrajectory.size());
    for(size_t i = 0; i < req.foot_pose_target_trajectories.footPoseTrajectory.size(); i++)
    {
      // 复制4D数据到6D的前4个元素，后2个元素设为0
      for(int j = 0; j < 4; j++)
      {
        req6d.foot_pose_target_trajectories.footPoseTrajectory[i].footPose6D[j] = req.foot_pose_target_trajectories.footPoseTrajectory[i].footPose[j];
      }
      req6d.foot_pose_target_trajectories.footPoseTrajectory[i].footPose6D[4] = 0.0; // pitch
      req6d.foot_pose_target_trajectories.footPoseTrajectory[i].footPose6D[5] = 0.0; // roll
      
      // 复制躯干姿态，从4D扩展到6D
      for(int j = 0; j < 4; j++)
      {
        req6d.foot_pose_target_trajectories.footPoseTrajectory[i].torsoPose6D[j] = req.foot_pose_target_trajectories.footPoseTrajectory[i].torsoPose[j];
      }
      req6d.foot_pose_target_trajectories.footPoseTrajectory[i].torsoPose6D[4] = 0.0; // pitch
      req6d.foot_pose_target_trajectories.footPoseTrajectory[i].torsoPose6D[5] = 0.0; // roll
    }
    
    // 转换额外足部姿态
    req6d.foot_pose_target_trajectories.additionalFootPoseTrajectory.resize(req.foot_pose_target_trajectories.additionalFootPoseTrajectory.size());
    for(size_t i = 0; i < req.foot_pose_target_trajectories.additionalFootPoseTrajectory.size(); i++)
    {
      req6d.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data.resize(req.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data.size());
      for(size_t j = 0; j < req.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data.size(); j++)
      {
        for(int k = 0; k < 4; k++)
        {
          req6d.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data[j].footPose6D[k] = req.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data[j].footPose[k];
        }
        req6d.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data[j].footPose6D[4] = 0.0; // pitch
        req6d.foot_pose_target_trajectories.additionalFootPoseTrajectory[i].data[j].footPose6D[5] = 0.0; // roll
      }
    }
    
    // 调用6D服务回调函数
    kuavo_msgs::footPose6DTargetTrajectoriesSrv::Response res6d;
    bool result = footPose6DTargetTrajectoriesSrvCallback(req6d, res6d);
    
    // 复制响应结果
    res.success = res6d.success;
    return result;
  }

  bool SwitchedModelReferenceManager::footPose6DTargetTrajectoriesSrvCallback(kuavo_msgs::footPose6DTargetTrajectoriesSrv::Request &req, kuavo_msgs::footPose6DTargetTrajectoriesSrv::Response &res)
  {
    bool exit_non_stance_mode = false;
    const auto &mode_schedule = gaitSchedulePtr_->getFullModeSchedule().modeSequence;
    for(auto m: mode_schedule)
      if(m!= ModeNumber::SS)
      {
        exit_non_stance_mode = true;
        break;
      }
    if(exit_non_stance_mode)
    {
      std::cout << "\033[33m[WARNING] Current mode is NOT SS mode, could NOT update foot pose target!!!\033[0m" << std::endl;
      res.success = false;
      return true;
    }
    else if(vel_norm_ > 0.03)
    {
      std::cout << "\033[33m[WARNING] Current speed[" << vel_norm_ << " m/s] is too high, could NOT update foot pose target!!!\033[0m" << std::endl;
      res.success = false;
      return true;
    }
    auto foot_traj = req.foot_pose_target_trajectories;
    if(!update_foot_trajectory_)
    {
      const int size = foot_traj.timeTrajectory.size();
      if(size!= foot_traj.footIndexTrajectory.size() || size!= foot_traj.footPoseTrajectory.size())
      {
        ROS_WARN_STREAM("footPose6DTargetTrajectories size is not equal to timeTrajectory, footIndexTrajectory, and footPoseTrajectory");
        res.success = false;
        return true;
      }
      footPoseSchedule_.eventTimes.resize(size);
      footPoseSchedule_.footIndices.resize(size);
      footPoseSchedule_.footPoseSequence.resize(size);
      footPoseSchedule_.torsoPoseSequence.resize(size);
      for(int i=0; i<foot_traj.timeTrajectory.size(); i++)
      {
        footPoseSchedule_.eventTimes[i] = foot_traj.timeTrajectory[i];
        footPoseSchedule_.footIndices[i] = static_cast<FootIdx>(foot_traj.footIndexTrajectory[i]);
        for(int j=0; j<foot_traj.footPoseTrajectory[i].footPose6D.size(); j++)
        {
          footPoseSchedule_.footPoseSequence[i](j) = foot_traj.footPoseTrajectory[i].footPose6D[j];
        }
        for(int j=0; j<foot_traj.footPoseTrajectory[i].torsoPose6D.size(); j++)
        {
          footPoseSchedule_.torsoPoseSequence[i](j) = foot_traj.footPoseTrajectory[i].torsoPose6D[j];
        }
        std::cout << (footPoseSchedule_.footIndices[i]==FootIdx::Left?"L":"R")
                  << ", time: " << footPoseSchedule_.eventTimes[i] << ", footPose6D: "
                  << footPoseSchedule_.footPoseSequence[i].transpose() << ", torsoPose6D: "
                  << footPoseSchedule_.torsoPoseSequence[i].transpose() << std::endl;
      }
      update_foot_trajectory_ = true;
    }
    res.success = true;
    return true;
  }


bool SwitchedModelReferenceManager::armControlModeSrvCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
{

  res.result = true;
  switch (req.control_mode)
  {
  case 0:
    res.message = "Arm control mode 0: keep current control position";
    break;
  case 1:
    res.message = "Arm control mode 1: auto swing arm";
    break;
  case 2:
    res.message = "Arm control mode 2: using external controller";
    break;
  default:
    res.result = false;
    res.message = "Invalid control mode :" + std::to_string(req.control_mode);
    break;
  }
  if (res.result)
  {
    isArmControlModeChanged_ = true;
    isArmControlModeChangedTrigger_ = true;
    isCalcArmControlModeChangedTime_ = true;
    newArmControlMode_ = static_cast<ArmControlMode>(req.control_mode);
    
    // Only half up body, MPC doesn't run. change arm ctrl mode!
    if(only_half_up_body_) {
      currentArmControlMode_ = newArmControlMode_;
    }
    std::cout << "currentArmControlMode_:"<< currentArmControlMode_ << std::endl;
    res.mode = newArmControlMode_;
    ROS_INFO_STREAM(res.message);
    vector_t arm_control_mode_vec(2);
    arm_control_mode_vec << currentArmControlMode_, newArmControlMode_;
    ros_logger_->publishVector("/humanoid/mpc/arm_control_mode", arm_control_mode_vec);
  }
  else
  {
    ROS_ERROR_STREAM(res.message);
  }

  return true;
}

  bool SwitchedModelReferenceManager::torsoControlModeSrvCallback(kuavo_msgs::changeTorsoCtrlMode::Request &req, kuavo_msgs::changeTorsoCtrlMode::Response &res)
  {

    res.result = true;
    switch (req.control_mode)
    {
    case 0:
      res.message = "Torso control mode 0: Control torso 6-dof";
      break;
    case 1:
      res.message = "Torso control mode 1: Control torso height+yaw+pitch";
      break;
    case 2:
      res.message = "Torso control mode 2: Control torso height+pitch";
      break;
    default:
      res.result = false;
      res.message = "Invalid control mode :" + std::to_string(req.control_mode);
      break;
    }
    if (res.result)
    {
      torsoControlMode_ = static_cast<TorsoControlMode>(req.control_mode);
      std::cout << "Torso Control Mode:"<< torsoControlMode_ << std::endl;
      res.mode = torsoControlMode_;
      ROS_INFO_STREAM(res.message);
    }
    else
    {
      ROS_ERROR_STREAM(res.message);
    }

    return true;
  }

  struct Pose
  {
    Eigen::Vector3d position; // 位置
    Eigen::Matrix3d rotation; // 旋转矩阵
  };

  Pose calculateFeetPose(
      const std::vector<Eigen::Vector3d> &local_points, // 局部坐标系下的点位置
      const std::vector<Eigen::Vector3d> &world_points) // 世界坐标系下的点位置
  {
    if (local_points.size() != 3 || world_points.size() != 3)
    {
      throw std::runtime_error("Need exactly 3 points!");
    }

    // 1. 计算两组点的质心
    Eigen::Vector3d local_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d world_centroid = Eigen::Vector3d::Zero();

    for (int i = 0; i < 3; i++)
    {
      local_centroid += local_points[i];
      world_centroid += world_points[i];
    }
    local_centroid /= 3.0;
    world_centroid /= 3.0;

    // 2. 将点集去质心
    Eigen::MatrixXd local_centered(3, 3);
    Eigen::MatrixXd world_centered(3, 3);

    for (int i = 0; i < 3; i++)
    {
      local_centered.col(i) = local_points[i] - local_centroid;
      world_centered.col(i) = world_points[i] - world_centroid;
    }

    // 3. 计算协方差矩阵
    Eigen::Matrix3d H = local_centered * world_centered.transpose();

    // 4. SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 5. 计算旋转矩阵
    Eigen::Matrix3d R = V * U.transpose();

    // 检查是否是正交矩阵（行列式为1）
    if (R.determinant() < 0)
    {
      V.col(2) *= -1;
      R = V * U.transpose();
    }

    // 6. 计算平移向量
    Eigen::Vector3d t = world_centroid - R * local_centroid;

    // 返回姿态结果
    Pose result;
    result.position = t;
    result.rotation = R;
    return result;
  }

  void SwitchedModelReferenceManager::checkSingleStepControlAndStop()
  {
    if (gaitSchedulePtr_->getFullModeSchedule().existValidFootPose()) update_stop_single_step_ = true;
  }

  void SwitchedModelReferenceManager::calculateJointRef(scalar_t initTime, scalar_t finalTime, const vector_t &initState,
                                                        TargetTrajectories &targetTrajectories, const ModeSchedule& modeSchedule)
  {
    bool verbose = false;
    vector3_t euler_zyx = initState.segment<3>(6 + 3);
    matrix3_t world2body_rotation = getRotationMatrixFromZyxEulerAngles(euler_zyx);
    matrix3_t body2foot_rotation = getRotationMatrixFromZyxEulerAngles(vector3_t(0, 0, 0));
    matrix3_t R_des = world2body_rotation * body2foot_rotation;
    if (targetTrajectories.size() <= 1)
      return;
    auto q_ref = vector_t(info_.generalizedCoordinatesNum);

    const scalar_t step = 0.10;
    int sample_size = floor((finalTime - initTime) / step) + 1;

    if (sample_size <= 2)
      return;
    vector_t TsVec = vector_t::LinSpaced(sample_size, initTime, finalTime);
    const auto old_target_trajectories = targetTrajectories;
    std::vector<double> selected_time_trajectory = std::vector<double>(TsVec.data(), TsVec.data() + TsVec.size());
    for (int i = 0; i < modeSchedule.eventTimes.size(); i++)
    {
      if (modeSchedule.eventTimes[i] > initTime && modeSchedule.eventTimes[i] < finalTime)
      {
        sample_size ++ ;
        selected_time_trajectory.push_back(modeSchedule.eventTimes[i]);
      }
    }
    std::sort(selected_time_trajectory.begin(), selected_time_trajectory.end());
    vector_t Ts(sample_size);
    for (int i = 0; i < sample_size; i++)
    {
      Ts[i] = selected_time_trajectory[i];
    }

    targetTrajectories.timeTrajectory.resize(sample_size);
    targetTrajectories.stateTrajectory.resize(sample_size);
    targetTrajectories.inputTrajectory.resize(sample_size);

    for (int i = 0; i < sample_size; i++)
    {
      targetTrajectories.timeTrajectory[i] = Ts[i];
      targetTrajectories.stateTrajectory[i] = old_target_trajectories.getDesiredState(Ts[i]);
      targetTrajectories.inputTrajectory[i] = old_target_trajectories.getDesiredInput(Ts[i]);
    }
    const auto &joint_num = info_.actuatedDofNum;
    targetTrajectories.stateTrajectory[0].segment(6 + 6, joint_num) = initTargetState_.segment(6 + 6, joint_num);
    auto init_target_state = targetTrajectories.stateTrajectory[0] / sample_size;

    auto modifi_x_state = [&](vector_t &state, int i, int leg_index, const vector_t &new_state)
    {
      auto mode_schedule =  gaitSchedulePtr_->getModeSchedule(initTime, finalTime, false);
      
      std::vector<int> feet_index = {2, 3, 4};
      // TODO: 临时解决方案
      std_msgs::Bool is_custom_gait_msg;
      bool is_custom_gait = mode_schedule.existValidFootPose(initTime);// 从当前时刻开始判断是否还有单步指令
      if(is_custom_gait)
         feet_index = {0, 1, 2, 3, 4};
      is_custom_gait_msg.data = is_custom_gait;
      isCustomGaitPublisher_.publish(is_custom_gait_msg);
      for (auto &index : feet_index)
        state.segment<6>(12 + leg_index)[index] = new_state[index];
      if (!is_custom_gait)
        {
          state.segment<6>(12 + leg_index)[0] -= state.segment<6>(12 + leg_index)[0] * i/sample_size;
          state.segment<6>(12 + leg_index)[1] -= state.segment<6>(12 + leg_index)[1] * i/sample_size;
        }
    };
    if (verbose)
    {
      std::cout << "\n\n";
      std::cout << "inittime: " << initTime << std::endl;

    }
    auto getFootRotation = [&](int idx_a, int idx_b, scalar_t time) -> matrix3_t 
    {
      Eigen::Vector2d vec_foot;
      Eigen::Vector2d pos_foot_a;
      Eigen::Vector2d pos_foot_b;
      pos_foot_a.x() = swingTrajectoryPtr_->getXpositionConstraint(idx_a, time);
      pos_foot_a.y() = swingTrajectoryPtr_->getYpositionConstraint(idx_a, time);
      pos_foot_b.x() = swingTrajectoryPtr_->getXpositionConstraint(idx_b, time);
      pos_foot_b.y() = swingTrajectoryPtr_->getYpositionConstraint(idx_b, time);
      // swingTrajectoryPtr_->getFootPositionConstraint(time);
      
      vec_foot = pos_foot_b - pos_foot_a;
      double foot_yaw = atan2(vec_foot.y(), vec_foot.x());
      if (verbose)
      {
        // std::cout << "pos_foot_a: " << pos_foot_a.transpose() << std::endl;
        // std::cout << "pos_foot_b: " << pos_foot_b.transpose() << std::endl;
        // std::cout << "vec_foot: " << vec_foot.transpose() << std::endl;
        std::cout << "foot_yaw: " << foot_yaw * 180 / M_PI << "deg." << std::endl;
      }
      matrix3_t R_wf = getRotationMatrixFromZyxEulerAngles(vector3_t(foot_yaw, 0, 0));
      return std::move(R_wf); 
    };
    const int feet_num = 2;
    for (int i = 0; i < sample_size; i++)
    {
      q_ref.head<6>() = targetTrajectories.stateTrajectory[i].segment<6>(6);
      q_ref.segment(6, joint_num) = targetTrajectories.stateTrajectory[i].segment(6 + 6, joint_num);
      vector3_t des_foot_p;

      for (int leg = 0; leg < feet_num; leg++)
      {
        int index = InverseKinematics::leg2index(leg);
        int select_pos = InverseKinematics::leg2point(leg);
        des_foot_p.x() = swingTrajectoryPtr_->getXpositionConstraint(select_pos, Ts[i]);
        des_foot_p.y() = swingTrajectoryPtr_->getYpositionConstraint(select_pos, Ts[i]);
        des_foot_p.z() = swingTrajectoryPtr_->getZpositionConstraint(select_pos, Ts[i]);
        // if(leg == 0)
        //   R_des = getFootRotation(2, 0, Ts[i]);
        // else
        //   R_des = getFootRotation(6, 4, Ts[i]);

        std::vector<vector3_t> local_foot_points = {// 0,1,2个接触点
            {0.16902, 0.04773, -0.0593},
            {0.16902, -0.04773, -0.0593},
            {-0.07316, 0.04773, -0.0593},
        };

        feet_array_t<vector3_t> foot_positions = swingTrajectoryPtr_->getFootPositionConstraint(Ts[i]);

        std::vector<vector3_t> world_foot_points  = (leg == 0)?std::vector<vector3_t>{
          foot_positions[0],
          foot_positions[1],
          foot_positions[2],
        } : 
        std::vector<vector3_t>{
          foot_positions[4],
          foot_positions[5],
          foot_positions[6],
        };
          

        auto foot_pose = calculateFeetPose(local_foot_points, world_foot_points);
        R_des = foot_pose.rotation;
        // std::cout << "foot_pose.position: " << foot_pose.position.transpose() << std::endl;
        // std::cout << "foot_pose.rotation:\n" << foot_pose.rotation.eulerAngles(2, 1, 0).transpose() << std::endl;

        // ikTimer_.startTimer();
        modifi_x_state(targetTrajectories.stateTrajectory[i], i, index, inverseKinematics_.computeIK(q_ref, leg, des_foot_p, R_des));
        

        if (verbose)
        {
          std::cout << "select_pos:" << select_pos ;

          std::cout << " leg: " << leg << "\nTs[i]: " << Ts[i] << "\n"
                    << "body:" << q_ref.head<3>().transpose() << "\ndes_foot_p: " << des_foot_p.transpose() << std::endl;
          auto foot_pos = inverseKinematics_.computeFootPos(targetTrajectories.stateTrajectory[i]);
          std::cout << "foot_pos: " << foot_pos[select_pos].transpose() << std::endl;
        } // auto foot_pos = inverseKinematics_.computeFootPos(targetTrajectories.stateTrajectory[i]);
        // std::cout << "foot_pos: " << foot_pos[select_pos].transpose() << std::endl;
        // ikTimer_.endTimer();
      }
    }

    // add longger targetTrajectories
    for (int i = 0; i < old_target_trajectories.timeTrajectory.size(); i++)
    {
      if (old_target_trajectories.timeTrajectory[i] > finalTime)
      {
        targetTrajectories.timeTrajectory.push_back(old_target_trajectories.timeTrajectory[i]);
        targetTrajectories.stateTrajectory.push_back(old_target_trajectories.stateTrajectory[i]);
        targetTrajectories.inputTrajectory.push_back(old_target_trajectories.inputTrajectory[i]);
      }
    }
  }
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::setMatrixQByGaitPair(const std::string &gait_name, const scalar_t &time)
  {
    // auto gait_name = gaitSchedulePtr_->getGgaitTimeName();
    // TO-DO: load file in constructor
    // loadBaseTrackingQ(dynamic_qr_file);
    // printf("Getting Q and R for %s gait\n", gait_name.c_str());
    // matrix_t Q(6, 6);
    if (dynamic_qr_flag_)
    {
      if (gait_name == "trot" || gait_name == "walk")
        gait_Q_ = baseTrackingQ_.Walk;
      if (gait_name == "stance") {
        // Check if VR waist control is enabled
        if (vrWaistControlEnabled_) {
          gait_Q_ = baseTrackingQ_.StanceVRwaist;
          //记录当前torsoyaw角
          currentTorsoYaw_ = currentState_[9];
          //currentTorsoRoll_ = currentState_[11];
          setEnablePitchLimit(false); // Disable pitch limit when using VR waist control
          std::cout << "✅Using VR waist control Q matrix for stance" << std::endl;
        } else {
          setEnablePitchLimit(true); // Enable pitch limit when not using VR waist control
          gait_Q_ = baseTrackingQ_.Stance;
          std::cout << "✅Using Stance Q matrix for stance" << std::endl;
        }
      }
      if (gait_name == "jump")
        gait_Q_ = baseTrackingQ_.Jump;
      if (gait_name == "dynamic_qr" )
      {
        setEnablePitchLimit(false);
        gait_Q_ = dynamic_Q;
        std::cout << "✅Using Taiji Q matrix for dynamic_Q gait" << std::endl;
      }
    }
    // std::cout << "Q:\n" << Q_ << std::endl;
    // setMatrixQ(Q_);报错
    // gait_Q_ = std::move(Q_);
    setChangeQTime(time);
    setUpdatedQ(true);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  matrix_t SwitchedModelReferenceManager::initializeStateCostWeightDynamic(const std::string &taskFile, const std::string &fieldName)
  {
    // 使用已有的成员变量，避免依赖ROS参数服务器的时序问题
    int mpcArmsDof = armJointNums_;  // 使用已有的成员变量
    //int mpcLegsDof = feetJointNums_;
    //int mpcWaistDof = waistNums_;
    
    // std::cout << "✅mpcArmsDof: " << mpcArmsDof << ", mpcLegsDof: " << mpcLegsDof << ", mpcWaistDof: " << mpcWaistDof << std::endl;
    // 计算原始手臂自由度和MPC手臂自由度的差异
    int originArmDof = armRealDof_; // 使用实际手臂自由度
    int armDofDiff = originArmDof - mpcArmsDof;
    int halfArmDof = mpcArmsDof / 2;
    int halfOriginArmDof = originArmDof / 2;
    // std::cout << "✅originArmDof: " << originArmDof << ", armDofDiff: " << armDofDiff << std::endl;
    // 加载原始Q矩阵（包含完整手臂自由度）
    matrix_t Q_origin(info_.stateDim + armDofDiff, info_.stateDim + armDofDiff);
    matrix_t Q(info_.stateDim, info_.stateDim);
    Q.setZero();
    loadData::loadEigenMatrix(taskFile, fieldName, Q_origin);
    
    // 简化Q矩阵：将完整手臂自由度转换为MPC所需的简化自由度
    int DofWithoutArms = info_.stateDim - mpcArmsDof;
    Q.block(0, 0, DofWithoutArms, DofWithoutArms) = Q_origin.block(0, 0, DofWithoutArms, DofWithoutArms);
    
    // 处理左右手臂的权重矩阵
    for (int i = 0; i < 2; i++)
    {
      Q.block(DofWithoutArms + halfArmDof * i, DofWithoutArms + halfArmDof * i, halfArmDof, halfArmDof) =
          Q_origin.block(DofWithoutArms + halfOriginArmDof * i, DofWithoutArms + halfOriginArmDof * i, halfArmDof, halfArmDof);
    }
    //std::cout << "✅ Loaded state cost weight Q matrix dimension: " << Q.rows() << "x" << Q.cols() << std::endl;
    return Q;
  }

  matrix_t SwitchedModelReferenceManager::initializeInputCostWeightDynamic(const std::string &taskFile, const std::string &fieldName)
  {
    // 使用与HumanoidInterface完全一致的变量名和值
    int joint_Num = feetJointNums_;  // 对应HumanoidInterface的joint_Num_
    const size_t totalThreeDofContactDim = 3 * info_.numThreeDofContacts;
    const size_t totalSixDofContactDim = 6 * info_.numSixDofContacts;
    const size_t totalContactDim = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
    int originArmDof = armRealDof_;
    int dualArmDof = info_.actuatedDofNum - feetJointNums_ - waistNums_;
    int halfOriginArmDof = originArmDof / 2;
    int halfDualArmDof = dualArmDof / 2;

    // 计算雅可比矩阵用于腿部关节速度权重（完全按照HumanoidInterface实现）
    vector_t initialState(info_.stateDim);
    initialState.setZero();
    
    // 使用HumanoidInterfaceDrake获取正确的初始状态（与HumanoidInterface完全一致）
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rbVersion_, true, 2e-3);
    // 注意：getInitialState()的结构是 [zeros(6), base_vec(6), joint_pos(12)]
    // HumanoidInterface使用: head(6)=base_vec, tail(12)=joint_pos
    initialState.head(12) = drake_interface_->getInitialState().head(12);
    initialState.segment(12, joint_Num) = drake_interface_->getInitialState().segment(12, joint_Num);
    //std::cout << "✅ initialState:\n" << initialState.transpose() << std::endl;
    const auto &model = pinocchioInterface_.getModel();
    auto &data = pinocchioInterface_.getData();
    const auto q = centroidal_model::getGeneralizedCoordinates(initialState, info_);
    //std::cout << "✅ q:\n" << q.transpose() << std::endl;
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    matrix_t baseToFeetJacobians(totalThreeDofContactDim, joint_Num);

    //std::cout << "✅ baseToFeetJacobiansdim: " << baseToFeetJacobians.rows() << "x" << baseToFeetJacobians.cols() << std::endl;
    // 使用接触点名称计算雅可比矩阵（与HumanoidInterface完全一致）
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, model.getBodyId(contactNames3DoF_[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                      jacobianWorldToContactPointInWorldFrame);
      //std::cout << "✅ jacobianWorldToContactPointInWorldFrame for " << contactNames3DoF_[i] << ":\n"
                //<< jacobianWorldToContactPointInWorldFrame << std::endl;
      baseToFeetJacobians.block(3 * i, 0, 3, joint_Num) =
              jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, joint_Num);
    }
    //std::cout << "✅ baseToFeetJacobians:\n" << baseToFeetJacobians << std::endl;

    // 加载原始R矩阵
    matrix_t R_modelSpace(totalThreeDofContactDim * 2 + totalSixDofContactDim + originArmDof + waistNums_,
                          totalThreeDofContactDim * 2 + totalSixDofContactDim + originArmDof + waistNums_);
    loadData::loadEigenMatrix(taskFile, fieldName, R_modelSpace);
    
    // 创建任务空间R矩阵
    matrix_t R_taskspace(totalThreeDofContactDim * 2 + totalSixDofContactDim + dualArmDof + waistNums_,
                         totalThreeDofContactDim * 2 + totalSixDofContactDim + dualArmDof + waistNums_);
    R_taskspace.setZero();
    
    // 简化QR矩阵
    // R_modelSpace的顺序：接触力 → 脚速度 → 腰部 → 手臂速度
    int DofWithoutArms = totalThreeDofContactDim * 2 + totalSixDofContactDim + waistNums_;
    //std::cout << "✅ switch_DofWithoutArms:" << DofWithoutArms << std::endl;
    // 复制接触力部分
    R_taskspace.block(0, 0, totalContactDim, totalContactDim) =
        R_modelSpace.block(0, 0, totalContactDim, totalContactDim);

    // 复制脚速度部分（用于后面的雅可比变换）
    R_taskspace.block(totalContactDim, totalContactDim, totalThreeDofContactDim, totalThreeDofContactDim) =
        R_modelSpace.block(totalContactDim, totalContactDim, totalThreeDofContactDim, totalThreeDofContactDim);
    
    // 复制腰部部分（注意在R_modelSpace中腰部在脚速度之后）
    int waistPosInModelSpace = totalContactDim + totalThreeDofContactDim;
    R_taskspace.block(totalContactDim + totalThreeDofContactDim, totalContactDim + totalThreeDofContactDim, waistNums_, waistNums_) =
        R_modelSpace.block(waistPosInModelSpace, waistPosInModelSpace, waistNums_, waistNums_);

    // 复制手臂速度部分
    for (int i = 0; i < 2; i++)
    {
      R_taskspace.block(DofWithoutArms + halfDualArmDof * i, DofWithoutArms + halfDualArmDof * i, halfDualArmDof, halfDualArmDof) =
          R_modelSpace.block(DofWithoutArms + halfOriginArmDof * i, DofWithoutArms + halfOriginArmDof * i, halfDualArmDof, halfDualArmDof);
    }

    // 构建最终的R矩阵
    // 最终R矩阵的顺序：接触力 → 腿部关节速度 → 腰部关节速度 → 手臂关节速度
    matrix_t R = matrix_t::Zero(info_.inputDim, info_.inputDim);

    // 1. 接触力
    R.topLeftCorner(totalThreeDofContactDim, totalThreeDofContactDim) = 
        R_taskspace.topLeftCorner(totalThreeDofContactDim, totalThreeDofContactDim);
    
    if (totalSixDofContactDim != 0)
    {
      R.block(totalThreeDofContactDim, totalThreeDofContactDim, totalSixDofContactDim, totalSixDofContactDim) =
          R_taskspace.block(totalThreeDofContactDim, totalThreeDofContactDim, totalSixDofContactDim, totalSixDofContactDim);
    }
    
    // 2. 腿部关节速度（通过雅可比矩阵从脚速度转换）
    R.block(totalContactDim, totalContactDim, feetJointNums_, feetJointNums_) =
        baseToFeetJacobians.transpose() * 
        R_taskspace.block(totalContactDim, totalContactDim, totalThreeDofContactDim, totalThreeDofContactDim) *
        baseToFeetJacobians;
    
    // 3. 腰部（在R中接着腿部关节速度）
    R.block(totalContactDim + feetJointNums_, totalContactDim + feetJointNums_, waistNums_, waistNums_) =
        R_taskspace.block(totalContactDim + totalThreeDofContactDim, totalContactDim + totalThreeDofContactDim, waistNums_, waistNums_);

    // 4. 手臂关节速度
    if (dualArmDof != 0)
    {
      R.bottomRightCorner(dualArmDof, dualArmDof) = R_taskspace.bottomRightCorner(dualArmDof, dualArmDof);
    }

    return R;
  }

  void SwitchedModelReferenceManager::loadBaseTrackingQ(const std::string &dynamic_qr_file)
  {
    baseTrackingQ_.Stance = initializeStateCostWeightDynamic(dynamic_qr_file, "Q_base_stance");
    baseTrackingQ_.StanceVRwaist = initializeStateCostWeightDynamic(dynamic_qr_file, "Q_base_stance_VRwaist");
    baseTrackingQ_.Walk = initializeStateCostWeightDynamic(dynamic_qr_file, "Q_base_walk");
    baseTrackingQ_.Jump = initializeStateCostWeightDynamic(dynamic_qr_file, "Q_base_jump");
    
    std::cout << "Loaded dynamic Q matrices with dimension adjustments - Stance: " << baseTrackingQ_.Stance.rows() << "x" << baseTrackingQ_.Stance.cols() 
              << ", StanceVRwaist: " << baseTrackingQ_.StanceVRwaist.rows() << "x" << baseTrackingQ_.StanceVRwaist.cols()
              << ", Walk: " << baseTrackingQ_.Walk.rows() << "x" << baseTrackingQ_.Walk.cols() 
              << ", Jump: " << baseTrackingQ_.Jump.rows() << "x" << baseTrackingQ_.Jump.cols() << std::endl;
    // std::cout << "stanceQ_:\n" << baseTrackingQ_.Stance << std::endl;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::loadBaseTrackingR(const std::string &dynamic_qr_file)
  {
    baseTrackingR_.Stance = initializeInputCostWeightDynamic(dynamic_qr_file, "R_base_stance");
    baseTrackingR_.Walk = initializeInputCostWeightDynamic(dynamic_qr_file, "R_base_walk");
    std::cout << "Loaded dynamic R matrices with dimension adjustments - Stance: " << baseTrackingR_.Stance.rows() << "x" << baseTrackingR_.Stance.cols() 
              << ", Walk: " << baseTrackingR_.Walk.rows() << "x" << baseTrackingR_.Walk.cols()  << std::endl;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::loadDynamicQRMap(const std::string &dynamic_qr_file)
  {
    dynamic_qr_map_.clear();
    
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(dynamic_qr_file, pt);
    
    // 扫描所有键，找到Q_dynamic_<gait_name>格式的键
    for (const auto& pair : pt) {
      const std::string& key = pair.first;
      if (key.find("Q_dynamic_") == 0) {
        // 提取gait_name
        std::string gait_name = key.substr(10);  // 去掉"Q_dynamic_"前缀
        std::string q_field_name = "Q_dynamic_" + gait_name;
        std::string r_field_name = "R_dynamic_" + gait_name;
        
        // 检查对应的Q和R矩阵是否存在
        if (pt.find(q_field_name) != pt.not_found() && pt.find(r_field_name) != pt.not_found()) {
          DynamicQRPair qr_pair;
          // 直接调用提取函数
          qr_pair.Q = initializeStateCostWeightDynamic(dynamic_qr_file, q_field_name);
          qr_pair.R = initializeInputCostWeightDynamic(dynamic_qr_file, r_field_name);
          dynamic_qr_map_[gait_name] = qr_pair;
          std::cout << "[loadDynamicQRMap] Loaded QR matrices for gait: " << gait_name << std::endl;
        }
      }
    }
    
    std::cout << "[loadDynamicQRMap] Total loaded " << dynamic_qr_map_.size() << " dynamic QR pairs." << std::endl;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::setMatrixRByGaitPair(const std::string &gait_name, const scalar_t &time, bool all_stance)
  {
    if (dynamic_qr_flag_)
    {
      if (gait_name == "trot" || gait_name == "walk")
      {
        gait_R_ = baseTrackingR_.Walk;
        std::cout << "[setMatrixRByGaitPair] Setting Walk R matrix" << std::endl;
      }
      // 判断modeschedule
      else if (gait_name == "stance" && vrWaistControlEnabled_)
      { 
        gait_R_ = baseTrackingR_.Stance;
        std::cout << "[setMatrixRByGaitPair] ✅ Using Stance R matrix for gait: " << gait_name << std::endl;
      }
      // 其他步态默认使用walk模式的R矩阵
      else
      {
        gait_R_ = baseTrackingR_.Walk;
        std::cout << "[setMatrixRByGaitPair] ✅ Using Walk R matrix for gait: " << gait_name << std::endl;
      }

      if (all_stance)
      {
        if(dynamic_r_set_)
        {
          gait_R_ = dynamic_R;
          std::cout << "[setMatrixRByGaitPair] Using Taiji R matrix for gait: dynamic_R" << std::endl;
        }
        else
        {
          gait_R_ = baseTrackingR_.Stance;
          std::cout << "[setMatrixRByGaitPair] Setting Stance R matrix (all modes are SS)" << std::endl;
        }
      }
      else
      {
        gait_R_ = baseTrackingR_.Walk;
        std::cout << "[setMatrixRByGaitPair] Not all modes are SS, using Walk R matrix instead" << std::endl;
      }
      
      setChangeRTime(time);
      setUpdatedR(true);
    }
  }

  // /******************************************************************************************************/
  // /******************************************************************************************************/
  // /******************************************************************************************************/
  // void SwitchedModelReferenceManager::loadBaseTrackingR(const std::string &dynamic_qr_file)
  // {
  //   baseTrackingR_.Stance = initializeInputCostWeightDynamic(dynamic_qr_file, "R_base_stance");
  //   baseTrackingR_.Walk = initializeInputCostWeightDynamic(dynamic_qr_file, "R_base_walk");
    
  //   std::cout << "Loaded dynamic R matrices with dimension adjustments - Stance: " << baseTrackingR_.Stance.rows() << "x" << baseTrackingR_.Stance.cols() 
  //             << ", Walk: " << baseTrackingR_.Walk.rows() << "x" << baseTrackingR_.Walk.cols() << std::endl;
  // }

  // /******************************************************************************************************/
  // /******************************************************************************************************/
  // /******************************************************************************************************/
  // void SwitchedModelReferenceManager::setMatrixRByGaitPair(const std::string &gait_name, const scalar_t &time)
  // {
  //   if (dynamic_qr_flag_)
  //   {
  //     if (gait_name == "trot" || gait_name == "walk")
  //     {
  //       gait_R_ = baseTrackingR_.Walk;
  //       std::cout << "[setMatrixRByGaitPair] Setting Walk R matrix" << std::endl;
  //     }
  //     else if (gait_name == "stance")
  //     {
  //       gait_R_ = baseTrackingR_.Stance;
  //       std::cout << "[setMatrixRByGaitPair] Setting Stance R matrix" << std::endl;
  //     }
  //     // 其他步态默认使用stance模式的R矩阵
  //     else
  //     {
  //       gait_R_ = baseTrackingR_.Stance;
  //       std::cout << "[setMatrixRByGaitPair] Using Stance R matrix for gait: " << gait_name << std::endl;
  //     }
      
  //     setChangeRTime(time);
  //     setUpdatedR(true);
  //   }
  // }

  void SwitchedModelReferenceManager::publishFootContactPoint()
  {
    feet_array_t<vector3_t> foot_positions = swingTrajectoryPtr_->getNextFootPositions();
    std_msgs::Float32MultiArray foot_contact_point_msg;
    foot_contact_point_msg.data.resize(foot_positions.size() * 3);
    for (int i = 0; i < foot_positions.size(); i++)
    {
      foot_contact_point_msg.data[i * 3 + 0] = foot_positions[i].x();
      foot_contact_point_msg.data[i * 3 + 1] = foot_positions[i].y();
      foot_contact_point_msg.data[i * 3 + 2] = foot_positions[i].z();
    }
    footContactPointPublisher_.publish(foot_contact_point_msg);
  }

  void SwitchedModelReferenceManager::publishFootDesiredPoint(scalar_t time)
  {
    feet_array_t<vector3_t> foot_positions = swingTrajectoryPtr_->getFootPositionConstraint(time);
    std_msgs::Float32MultiArray foot_contact_point_msg;

    foot_contact_point_msg.data.resize(foot_positions.size() * 3);
    for (int i = 0; i < foot_positions.size(); i++)
    {
      foot_contact_point_msg.data[i * 3 + 0] = foot_positions[i].x();
      foot_contact_point_msg.data[i * 3 + 1] = foot_positions[i].y();
      foot_contact_point_msg.data[i * 3 + 2] = foot_positions[i].z();
    }
    footDesiredPointPublisher_.publish(foot_contact_point_msg);
  }

  /** brief: get all feet poses [position1, rotation1, position2, rotation2, ...]
   * @return: vector_t of feet center point poses [position1, rotation1, position2, rotation2, ...]
  */ 
  std::vector<vector_t> SwitchedModelReferenceManager::getFeetPoses(const vector_t& initState)
  {
    if(endEffectorKinematicsPtr_ == nullptr)
    {
      ROS_WARN_STREAM("endEffectorKinematicsPtr_ is nullptr, can not update foot position.");
      assert(false);// 当endEffectorKinematicsPtr_为空时，不应该调用此函数
    }
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData(); // 确保data是非const引用

    // 将Block类型显式转换为VectorXd
    const auto& q = centroidal_model::getGeneralizedCoordinates(initState, info_).eval();
    const vector_t v = vector_t::Zero(model.nv); // model.nv 是速度变量数

    // 调用forwardKinematics
    pinocchio::forwardKinematics(model, data, q); // 注意参数顺序

    pinocchio::updateFramePlacements(model, data);

    const auto feetPositions = endEffectorKinematicsPtr_->getPosition(initState);
    //
    const int sgl_foot_contact_num = 4;
    std::vector<vector_t> feet_poses;
    // TODO: 暂时只考虑脚的姿态只存在yaw方向的自由度，roll pitch不动
    for(int i = 0; i < info_.numThreeDofContacts/sgl_foot_contact_num; i++)
    {
      int idx = i * sgl_foot_contact_num;
      vector3_t foot_center_pos = (feetPositions[idx+0] + feetPositions[idx+1] + feetPositions[idx+2] + feetPositions[idx+3]) / 4.0;
      vector3_t foot_vec = feetPositions[idx+0] - feetPositions[idx+2];//左上-左下
      double foot_yaw = atan2(foot_vec.x(), foot_vec.y());
      auto foot_pose = (vector_t(6) << foot_center_pos, foot_yaw, 0.0, 0.0).finished();
      feet_poses.push_back(foot_pose);
    }
    return getFeetPoses(feetPositions);
  }

  /** brief: get all feet poses [position1, rotation1, position2, rotation2, ...]
   * @param[in] feetPositions: foot contact point positions in world frame
   * @return: vector_t of feet center point poses [position1, rotation1, position2, rotation2, ...]
  */ 
  std::vector<vector_t> SwitchedModelReferenceManager::getFeetPoses(const std::vector<vector3_t> &feetPositions)
  {
    const int sgl_foot_contact_num = 4;
    std::vector<vector_t> feet_poses;
    // TODO: 暂时只考虑脚的姿态只存在yaw方向的自由度，roll pitch不动
    for(int i = 0; i < info_.numThreeDofContacts/sgl_foot_contact_num; i++)
    {
      int idx = i * sgl_foot_contact_num;
      vector3_t foot_center_pos = (feetPositions[idx+0] + feetPositions[idx+1] + feetPositions[idx+2] + feetPositions[idx+3]) / 4.0;
      vector3_t foot_vec = feetPositions[idx+0] - feetPositions[idx+2];//左上-左下
      double foot_yaw = atan2(foot_vec.x(), foot_vec.y());
      auto foot_pose = (vector_t(6) << foot_center_pos, foot_yaw, 0.0, 0.0).finished();
      feet_poses.push_back(foot_pose);
    }
    return std::move(feet_poses);
  }

  vector3_t SwitchedModelReferenceManager::getComPos(const vector_t& state)
  {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    const vector_t v = vector_t::Zero(model.nv); // model.nv 是速度变量数

    pinocchio::centerOfMass(model, data, centroidal_model::getGeneralizedCoordinates(state, info_), v);
    return data.com[0];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::processFullBodyTrajectories(scalar_t initTime, scalar_t finalTime, scalar_t timeHorizon, 
                                                                  TargetTrajectories& targetTrajectories, const vector_t& initState,
                                                                  const feet_array_t<vector3_t>& feet_pos) {
    // 提取原始轨迹
    TargetTrajectories extractedTrajectories = fullBodyTargetTrajectories_.segmentTargetTrajectories(initTime - timeHorizon, finalTime + timeHorizon);
    
    // 在线校准轨迹，处理由于执行误差导致躯干漂移的情况
    targetTrajectories = extractedTrajectories;
    calibrateTrajectoryOnline(initTime, finalTime, initState, targetTrajectories, feet_pos);
    
    // 处理手臂轨迹
    auto armTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(
      fullBodyArmTargetTrajectories_.segmentTargetTrajectories(initTime - timeHorizon, initTime + timeHorizon));
    armTargetPublisher_.publish(armTargetTrajectoriesMsg);

    // 添加头部轨迹处理
    if (!fullBodyHeadTargetTrajectories_.timeTrajectory.empty()) {
      std_msgs::Float64MultiArray headMsg;
      auto currentHeadState = fullBodyHeadTargetTrajectories_.getDesiredState(initTime);
      headMsg.data = {currentHeadState[0], currentHeadState[1]};  // 假设头部关节角度在状态向量的前两个位置
      headArrayPublisher_.publish(headMsg);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::calibrateTrajectoryOnline(scalar_t initTime, scalar_t finalTime, 
                                                            const vector_t& currentState, 
                                                            TargetTrajectories& targetTrajectories,
                                                            const feet_array_t<vector3_t>& currentFeet) {
    // 获取当前支撑足信息
    contact_flag_t contactFlags = getContactFlags(initTime);
    
    // 检查是否有支撑足
    bool left_foot_contact = std::any_of(contactFlags.begin(), contactFlags.begin() + 4, [](bool flag) { return flag; });
    bool right_foot_contact = std::any_of(contactFlags.begin() + 4, contactFlags.end(), [](bool flag) { return flag; });
    
    // 获取理论规划的状态
    vector_t plannedState = fullBodyTargetTrajectories_.getDesiredState(initTime);
    feet_array_t<vector3_t> plannedFeet = inverseKinematics_.computeFootPos(plannedState);
    
    // 计算最后一次落地的脚的xy平面位置差异
    Eigen::Vector2d positionDiffXY;
    if (left_foot_contact && right_foot_contact)
    {
        positionDiffXY  = Eigen::Vector2d(
          currentFeet[0].x() + currentFeet[5].x() - plannedFeet[0].x() - plannedFeet[5].x(),
          currentFeet[0].y() + currentFeet[5].y() - plannedFeet[0].y() - plannedFeet[5].y()
      ) * 0.5;  // 取平均值
    }
    else
    {
      auto index = left_foot_contact ? 0 : 5;
      positionDiffXY = Eigen::Vector2d(
        currentFeet[index].x() - plannedFeet[index].x(),
        currentFeet[index].y() - plannedFeet[index].y()
      );
    }

    double alpha = 0.1;
    positionDiffXY = lastFootCalibrationDiffXY_ * (1 - alpha) + positionDiffXY * alpha;
    lastFootCalibrationDiffXY_ = positionDiffXY;
    
    // 应用差异到轨迹中的每个状态
    for (size_t i = 0; i < targetTrajectories.timeTrajectory.size(); ++i) {
        vector_t& state = targetTrajectories.stateTrajectory[i];
        
        // 只更新躯干xy位置
        state[6] += positionDiffXY.x();  // x position
        state[7] += positionDiffXY.y();  // y position
    }

    if (ros_logger_ != nullptr) {
        ros_logger_->publishVector("/humanoid/mpc/calibration/positionDiffXY", positionDiffXY);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void SwitchedModelReferenceManager::processFootPose6DTargetTrajectories(const kuavo_msgs::footPose6DTargetTrajectories::ConstPtr &msg, FrameType frameType)
  {
    if(frameType == FrameType::Local){
      if(!update_foot_trajectory_)
      {
        const int size = msg->timeTrajectory.size();
        if(size!= msg->footIndexTrajectory.size() || size!= msg->footPoseTrajectory.size())
        {
          ROS_WARN_STREAM("footPose6DTargetTrajectories size is not equal to timeTrajectory, footIndexTrajectory, and footPoseTrajectory");
          return;
        }

        // 获取调度控制参数
        bool schedule_enable = false;
        double schedule_start_time = -1.0;
        
        if (nodeHandle_.getParam("/mpc/schedule/enable", schedule_enable) && 
            nodeHandle_.getParam("/mpc/schedule/start_time", schedule_start_time) &&
            schedule_enable) {
            footPoseSchedule_.startTime = schedule_start_time;
            ROS_INFO_STREAM("[ReferenceManager]: Schedule enabled, start time: " << schedule_start_time);
            
            // 使用后立即重置enable参数
            nodeHandle_.setParam("/mpc/schedule/enable", false);
            ROS_INFO("[ReferenceManager]: Reset schedule enable to false");
        } else {
            footPoseSchedule_.startTime = -1.0;
            ROS_INFO("[ReferenceManager]: Schedule disabled or params not found, using default start time: -1");
        }

        footPoseSchedule_.eventTimes.resize(size);
        footPoseSchedule_.footIndices.resize(size);
        footPoseSchedule_.footPoseSequence.resize(size);
        footPoseSchedule_.torsoPoseSequence.resize(size);
        footPoseSchedule_.additionalFootPoseSequence.clear();
        if (msg->additionalFootPoseTrajectory.size() > 0)// 有额外足迹轨迹
          footPoseSchedule_.additionalFootPoseSequence.resize(size);
        footPoseSchedule_.swingHeightSequence.resize(size);
        std::fill(footPoseSchedule_.swingHeightSequence.begin(), footPoseSchedule_.swingHeightSequence.end(), 0.06);
        for(int i=0; i<msg->timeTrajectory.size(); i++)
        {
          footPoseSchedule_.eventTimes[i] = msg->timeTrajectory[i];
          if (i > 0 && msg->timeTrajectory[i] <= msg->timeTrajectory[i-1])// 检查时间不是递增的
          {
            ROS_WARN_STREAM("footPose6DTargetTrajectories time trajectory is not sorted");
            return;
          }
          footPoseSchedule_.footIndices[i] = static_cast<FootIdx>(msg->footIndexTrajectory[i]);
          for(int j=0; j<msg->footPoseTrajectory[i].footPose6D.size(); j++)
          {
            footPoseSchedule_.footPoseSequence[i](j) = msg->footPoseTrajectory[i].footPose6D[j];
          }
          // 处理躯干姿态
          footPoseSchedule_.torsoPoseSequence[i].setZero();
          for(int j=0; j<msg->footPoseTrajectory[i].torsoPose6D.size(); j++)
          {
            footPoseSchedule_.torsoPoseSequence[i](j) = msg->footPoseTrajectory[i].torsoPose6D[j];
          }
          if (msg->additionalFootPoseTrajectory.size() > 0)// 有额外足迹轨迹
          {
            std::vector<vector6_t> additionalPoses;
            for (int j = 0; j < msg->additionalFootPoseTrajectory[i].data.size(); j++)
            {
              Eigen::Map<const vector6_t> pose(msg->additionalFootPoseTrajectory[i].data[j].footPose6D.data());
              additionalPoses.push_back(pose);
            }
            std::cout << "step: " << i << " additionalPoses.size(): " << additionalPoses.size() << std::endl;
            footPoseSchedule_.additionalFootPoseSequence[i] = additionalPoses;
          }
          if (msg->swingHeightTrajectory.size() == footPoseSchedule_.footIndices.size())
          {
            footPoseSchedule_.swingHeightSequence[i] = msg->swingHeightTrajectory[i];
          }
          else
          {
            ROS_WARN_STREAM("swingHeightTrajectory size is not equal to footPoseTrajectory, use default swingHeight: 0.06");
          }
          std::cout << (footPoseSchedule_.footIndices[i]==FootIdx::Left?"L":"R")
                    << ", time: " << footPoseSchedule_.eventTimes[i] << ", footPose: "
                    << footPoseSchedule_.footPoseSequence[i].transpose() << ", torsoPose: "
                    << footPoseSchedule_.torsoPoseSequence[i].transpose() << ", swingHeight: "
                    << footPoseSchedule_.swingHeightSequence[i] << std::endl;
        }
        update_foot_trajectory_ = true;
      }
    }else{
      if(!update_foot_world_trajectory_)
      {
        const int size = msg->timeTrajectory.size();
        if(size!= msg->footIndexTrajectory.size() || size!= msg->footPoseTrajectory.size())
        {
          ROS_WARN_STREAM("footPose6DTargetTrajectories size is not equal to timeTrajectory, footIndexTrajectory, and footPoseTrajectory");
          return;
        }

        // 获取调度控制参数
        bool schedule_enable = false;
        double schedule_start_time = -1.0;
        
        if (nodeHandle_.getParam("/mpc/schedule/enable", schedule_enable) && 
            nodeHandle_.getParam("/mpc/schedule/start_time", schedule_start_time) &&
            schedule_enable) {
            footPoseWorldSchedule_.startTime = schedule_start_time;
            ROS_INFO_STREAM("[ReferenceManager]: Schedule enabled, start time: " << schedule_start_time);
            
            // 使用后立即重置enable参数
            nodeHandle_.setParam("/mpc/schedule/enable", false);
            ROS_INFO("[ReferenceManager]: Reset schedule enable to false");
        } else {
            footPoseWorldSchedule_.startTime = -1.0;
            ROS_INFO("[ReferenceManager]: Schedule disabled or params not found, using default start time: -1");
        }

        footPoseWorldSchedule_.eventTimes.resize(size);
        footPoseWorldSchedule_.footIndices.resize(size);
        footPoseWorldSchedule_.footPoseSequence.resize(size);
        footPoseWorldSchedule_.torsoPoseSequence.resize(size);
        footPoseWorldSchedule_.additionalFootPoseSequence.clear();
        if (msg->additionalFootPoseTrajectory.size() > 0)// 有额外足迹轨迹
          footPoseWorldSchedule_.additionalFootPoseSequence.resize(size);
        footPoseWorldSchedule_.swingHeightSequence.resize(size, 0.06);
        for(int i=0; i<msg->timeTrajectory.size(); i++)
        {
          footPoseWorldSchedule_.eventTimes[i] = msg->timeTrajectory[i];
          if (i > 0 && msg->timeTrajectory[i] <= msg->timeTrajectory[i-1])// 检查时间不是递增的
          {
            ROS_WARN_STREAM("footPose6DTargetTrajectories time trajectory is not sorted");
            return;
          }
          footPoseWorldSchedule_.footIndices[i] = static_cast<FootIdx>(msg->footIndexTrajectory[i]);
          for(int j=0; j<msg->footPoseTrajectory[i].footPose6D.size(); j++)
          {
            footPoseWorldSchedule_.footPoseSequence[i](j) = msg->footPoseTrajectory[i].footPose6D[j];
          }
          // 处理躯干姿态
          footPoseWorldSchedule_.torsoPoseSequence[i].setZero();
          for(int j=0; j<msg->footPoseTrajectory[i].torsoPose6D.size(); j++)
          {
            footPoseWorldSchedule_.torsoPoseSequence[i](j) = msg->footPoseTrajectory[i].torsoPose6D[j];
          }
          if (msg->additionalFootPoseTrajectory.size() > 0)// 有额外足迹轨迹
          {
            std::vector<vector6_t> additionalPoses;
            for (int j = 0; j < msg->additionalFootPoseTrajectory[i].data.size(); j++)
            {
              Eigen::Map<const vector6_t> pose(msg->additionalFootPoseTrajectory[i].data[j].footPose6D.data());
              additionalPoses.push_back(pose);
            }
            std::cout << "step: " << i << " additionalPoses.size(): " << additionalPoses.size() << std::endl;
            footPoseWorldSchedule_.additionalFootPoseSequence[i] = additionalPoses;
          }
          if (msg->swingHeightTrajectory.size() == footPoseWorldSchedule_.footIndices.size())
          {
            footPoseWorldSchedule_.swingHeightSequence[i] = msg->swingHeightTrajectory[i];
          }
          else
          {
            ROS_WARN_STREAM("swingHeightTrajectory size is not equal to footPoseTrajectory, use default swingHeight: 0.06");
          }
          std::cout << (footPoseWorldSchedule_.footIndices[i]==FootIdx::Left?"WL":"WR")
                    << ", time: " << footPoseWorldSchedule_.eventTimes[i] << ", footPose: "
                    << footPoseWorldSchedule_.footPoseSequence[i].transpose() << ", torsoPose: "
                    << footPoseWorldSchedule_.torsoPoseSequence[i].transpose() << ", swingHeight: "
                    << footPoseWorldSchedule_.swingHeightSequence[i] << std::endl;
        }
        update_foot_world_trajectory_ = true;
      }
    }
  }
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  kuavo_msgs::kuavoModeSchedule SwitchedModelReferenceManager::createModeScheduleMsg(const ModeSchedule &modeSchedule, scalar_t initTime)
  {
    kuavo_msgs::kuavoModeSchedule modeScheduleMsg;
    // event times
    modeScheduleMsg.eventTimes.clear();
    modeScheduleMsg.eventTimes.reserve(modeSchedule.eventTimes.size());
    for (const auto &ti : modeSchedule.eventTimes)
    {
      modeScheduleMsg.eventTimes.push_back(ti);
    }
    // mode sequence
    modeScheduleMsg.modeSequence.clear();
    modeScheduleMsg.modeSequence.reserve(modeSchedule.modeSequence.size());
    for (const auto &si : modeSchedule.modeSequence)
    {
      modeScheduleMsg.modeSequence.push_back(si);
    }
    modeScheduleMsg.currentTime = initTime;
    modeScheduleMsg.footPoseSequence.clear();
    modeScheduleMsg.footPoseSequence.reserve(modeSchedule.footPoseSequence.size());
    for (const auto &pose : modeSchedule.footPoseSequence)
    {
      kuavo_msgs::footPose6D pose6D;
      for (int i = 0; i < 6; ++i)
      {
        pose6D.footPose6D[i] = pose[i];  // 逐个元素赋值
        pose6D.torsoPose6D[i] = pose[i]; // torso pose is the same as foot pose in this case
      }
      modeScheduleMsg.footPoseSequence.push_back(pose6D);
    }

    return modeScheduleMsg;
  }

  void SwitchedModelReferenceManager::generateTargetwithTorsoMove(scalar_t initTime, const vector_t &initState, const vector_t &torsoDisplacement,
                                                                  const TargetTrajectories &targetTrajectories, vector_t &finalState, double &torso_max_time, const vector6_t &velocity_scale)
  {
    const vector6_t torso_velocity = velocity_scale;
    vector6_t torso_time = torsoDisplacement.cwiseQuotient(torso_velocity).cwiseAbs();
    // std::cout << "torso move time: " << torso_time.transpose() << std::endl;
    torso_max_time = torso_time.maxCoeff();
    // std::cout << "torso max time: " << torso_max_time << std::endl;
    finalState = targetTrajectories.getDesiredState(initTime + torso_max_time);
    // 使用当前轨迹在initTime时刻的状态 + 相对位移
    auto currentState = targetTrajectories.getDesiredState(initTime);
    finalState(6) = currentState[6] + torsoDisplacement[0];  // x
    finalState(7) = currentState[7] + torsoDisplacement[1];  // y
    finalState(8) = currentState[8] + torsoDisplacement[2];  // z (height)
    finalState(9) = currentState[9] + torsoDisplacement[5];  // yaw
    finalState(10) = currentState[10] + torsoDisplacement[4]; // pitch
  }
} // namespace humanoid
}  // namespace ocs2
