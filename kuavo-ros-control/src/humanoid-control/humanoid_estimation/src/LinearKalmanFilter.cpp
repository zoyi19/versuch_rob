/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_estimation/LinearKalmanFilter.h"

#include <humanoid_interface/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <std_msgs/Float32MultiArray.h>

namespace ocs2
{
  namespace humanoid
  {
    KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics &eeKinematics)
        : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics), tfListener_(tfBuffer_), topicUpdated_(false)
    {
      xHat_.setZero();
      base_pos_.setZero();
      last_base_ang_.setZero();
      last_base_pos_.setZero();
      last_foot_pos_.setZero();
      last_contact_point_index_ = 0;
      ps_.setZero();
      vs_.setZero();
      a_.setZero();
      a_.block(0, 0, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
      a_.block(3, 3, 3, 3) = Eigen::Matrix<scalar_t, 3, 3>::Identity();
      a_.block(6, 6, num_contact_dof, num_contact_dof) = Eigen::Matrix<scalar_t, num_contact_dof, num_contact_dof>::Identity();
      b_.setZero();

      Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c1(3, 6);
      c1 << Eigen::Matrix<scalar_t, 3, 3>::Identity(), Eigen::Matrix<scalar_t, 3, 3>::Zero();
      Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> c2(3, 6);
      c2 << Eigen::Matrix<scalar_t, 3, 3>::Zero(), Eigen::Matrix<scalar_t, 3, 3>::Identity();
      c_.setZero();
      c_.block(0, 6, num_contact_dof, num_contact_dof) = -Eigen::Matrix<scalar_t, num_contact_dof, num_contact_dof>::Identity();
      for (size_t i = 0; i < info_.numThreeDofContacts; i++)
      {
        c_.block(i * 3, 0, 3, 6) = c1;
        c_.block(i * 3 + num_contact_dof, 0, 3, 6) = c2;
        c_(num_contact_dof * 2 + i, 8 + i * 3) = 1.0;
      }
      ros_logger_ = new TopicLogger();
      // c_.block(0, 0, 3, 6) = c1;
      // c_.block(3, 0, 3, 6) = c1;
      // c_.block(6, 0, 3, 6) = c1;
      // c_.block(9, 0, 3, 6) = c1;
      // c_.block(0, 6, num_contact_dof, num_contact_dof) = -Eigen::Matrix<scalar_t, num_contact_dof, num_contact_dof>::Identity();
      // c_.block(num_contact_dof, 0, 3, 6) = c2;
      // c_.block(15, 0, 3, 6) = c2;
      // c_.block(18, 0, 3, 6) = c2;
      // c_.block(21, 0, 3, 6) = c2;
      // c_(27, 17) = 1.0;
      // c_(26, 14) = 1.0;
      // c_(25, 11) = 1.0;
      // c_(24, 8) = 1.0;
      // 订阅足端接触点
      auto footContactPointCallback = [this](const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() >= info_.numThreeDofContacts * 3) {
          feet_array_t<vector3_t> foot_positions;
          for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
            foot_positions[i] = vector3_t(msg->data[i * 3], msg->data[i * 3 + 1], msg->data[i * 3 + 2]);
          }
          updateFootPosDesired(foot_positions);
        }
      };
      footContactPointSubscriber_ = ros::NodeHandle().subscribe<std_msgs::Float32MultiArray>(
          "/humanoid_foot_desired_point", 2, footContactPointCallback);

      p_.setIdentity();
      p_ = 100. * p_;
      q_.setIdentity();
      r_.setIdentity();
      feetHeights_.setZero(info_.numThreeDofContacts);
      eeKinematics_->setPinocchioInterface(pinocchioInterface_);

      world2odom_.setRotation(tf2::Quaternion::getIdentity());
      sub_ = ros::NodeHandle().subscribe<nav_msgs::Odometry>("/tracking_camera/odom/sample", 10,
                                                             &KalmanFilterEstimate::callback, this);
    }
    void KalmanFilterEstimate::set_intial_state(const vector_t &state)
    {
      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();
      vector_t qPino(info_.generalizedCoordinatesNum);
      qPino.head<3>() = state.segment<3>(6);
      qPino.segment<3>(3) = state.segment<3>(9);
      qPino.tail(info_.actuatedDofNum) = state.tail(info_.actuatedDofNum);
      pinocchio::forwardKinematics(model, data, qPino);
      pinocchio::updateFramePlacements(model, data);
      int leg_l6_link = model.getFrameId("leg_l6_link");
      int leg_r6_link = model.getFrameId("leg_r6_link");
      int base_link = model.getFrameId("base_link");
      for (size_t i = 0; i < 10; ++i)
      {
        base_quat_ = data.oMf[base_link].rotation();
        last_foot_quat_ = data.oMf[leg_l6_link].rotation();
        last_foot_pos_ = data.oMf[leg_l6_link].translation();
        base_pos_(2) = -last_foot_pos_(2);
        last_foot_ang_ << 0, 0, 0;
      }
      xHat_.segment<3>(0) = state.segment<3>(6);
      for (size_t i = 0; i < num_contact_points_est; ++i)
      {
        xHat_.segment<3>(6 + i * 3) = eeKinematics_->getPosition(vector_t())[i];
        // std::cout << "xHat_.segment<3>(6 + i * 3):"<<xHat_.segment<3>(6 + i * 3).transpose() << std::endl;
        // xHat_(6 + i * 3 + 2) = 0;
        // if (contactFlag_[i])
        {
          feetHeights_[i] = xHat_(6 + i * 3 + 2);
        }
      }
      vector_t jointPos = state.tail(info_.actuatedDofNum);
      vector_t jointVel = vector_t::Zero(info_.actuatedDofNum);
      Eigen::Vector3d base_velocity;
      base_velocity.setZero();
      this->updateJointStates(jointPos, jointVel);
      this->updateLinear(state.segment(6, 3), vector_t::Zero(3));
      this->updateAngular(state.segment(9, 3), vector_t::Zero(3));

      // std::cout << "initial xHat_:" << xHat_.transpose() << std::endl;
    };
    void KalmanFilterEstimate::reset()
    {
      std::cout << "[KalmanFilterEstimate] reset KalmanFilterEstimate" << std::endl;
      p_.setIdentity();
      p_ = 100. * p_;
      feetHeights_.setZero(info_.numThreeDofContacts);

    }
    nav_msgs::Odometry KalmanFilterEstimate::updateKinematics(const ros::Time &time, const Eigen::Quaterniond &imu_quat, const ros::Duration &period)
    {
      scalar_t dt = period.toSec();
      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();
      size_t actuatedDofNum = info_.actuatedDofNum;
      vector_t qPino(info_.generalizedCoordinatesNum);
      vector_t vPino(info_.generalizedCoordinatesNum);
      qPino.setZero();
      qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);
      vPino.setZero();
      vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
          qPino.segment<3>(3),
          rbdState_.segment<3>(info_.generalizedCoordinatesNum)); // Only set angular velocity, let linear velocity be zero
      vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);
      pinocchio::forwardKinematics(model, data, qPino, vPino);
      pinocchio::updateFramePlacements(model, data);
      int leg_l6_link = model.getFrameId("leg_l6_link");
      int leg_r6_link = model.getFrameId("leg_r6_link");
      int base_link = model.getFrameId("base_link");
      Eigen::Vector3d l_position = data.oMf[leg_l6_link].translation();
      Eigen::Vector3d r_position = data.oMf[leg_r6_link].translation();
      Eigen::Matrix3d l_rotation = data.oMf[leg_l6_link].rotation();
      Eigen::Matrix3d r_rotation = data.oMf[leg_r6_link].rotation();
      Eigen::Quaternion<scalar_t> l_quat(l_rotation);
      Eigen::Quaternion<scalar_t> r_quat(r_rotation);
      l_quat.normalize();
      r_quat.normalize();
      Eigen::Vector3d eulerAngles = quatToZyx(l_quat);
      Eigen::Vector3d l_eulerAngles;
      l_eulerAngles << eulerAngles(2), eulerAngles(1), eulerAngles(0);
      eulerAngles = quatToZyx(r_quat);
      Eigen::Vector3d r_eulerAngles;
      r_eulerAngles << eulerAngles(2), eulerAngles(1), eulerAngles(0);
      ros_logger_->publishVector("/state_estimate/kinematic/R2e/l_eulerAngles", l_eulerAngles.transpose());
      ros_logger_->publishVector("/state_estimate/kinematic/R2e/r_eulerAngles", r_eulerAngles.transpose());

      bool isLeftContact = contactFlag_[0] && contactFlag_[1] && contactFlag_[2] && contactFlag_[3];
      bool isRightContact = contactFlag_[4] && contactFlag_[5] && contactFlag_[6] && contactFlag_[7];

      if (isLeftContact && isRightContact) {
          if (last_contact_point_index_ == 1) {
              base_pos_ = last_foot_pos_ - r_position;
              base_quat_ = r_rotation.inverse() * last_foot_quat_;
              base_ang_ = last_foot_ang_ - r_eulerAngles;
          } else {
              base_pos_ = last_foot_pos_ - l_position;
              base_quat_ = l_rotation.inverse() * last_foot_quat_;
              base_ang_ = last_foot_ang_ - l_eulerAngles;
          }
      } 
      else if (isLeftContact) {
          if (last_contact_point_index_ == 0) {
              base_pos_ = last_foot_pos_ - l_position;
              base_quat_ = l_rotation.inverse() * last_foot_quat_;
              base_ang_ = last_foot_ang_ - l_eulerAngles;
              Eigen::Vector3d base_quaternion_;
              Eigen::Vector3d last_foot_quaternion_;
              base_quaternion_ = quatToZyx(base_quat_);
              last_foot_quaternion_ = quatToZyx(last_foot_quat_);
              ros_logger_->publishVector("/state_estimate/kinematic/base_quat_/left", base_quaternion_.transpose());
              ros_logger_->publishVector("/state_estimate/kinematic/last_foot_quat_/left", last_foot_quaternion_.transpose());
          } else {
              last_foot_pos_ = base_pos_ + l_position;
              last_foot_quat_ = l_rotation * base_quat_;
              last_foot_ang_ = base_ang_ + l_eulerAngles;
          }
          last_contact_point_index_ = 0;
      }
      else if (isRightContact) {
          if (last_contact_point_index_ == 1) {
              base_pos_ = last_foot_pos_ - r_position;
              base_quat_ = r_rotation.inverse() * last_foot_quat_;
              base_ang_ = last_foot_ang_ - r_eulerAngles;
              Eigen::Vector3d base_quaternion_;
              Eigen::Vector3d last_foot_quaternion_;
              base_quaternion_ = quatToZyx(base_quat_);
              last_foot_quaternion_ = quatToZyx(last_foot_quat_);
              ros_logger_->publishVector("/state_estimate/kinematic/base_quat_/right", base_quaternion_.transpose());
              ros_logger_->publishVector("/state_estimate/kinematic/last_foot_quat_/right", last_foot_quaternion_.transpose());
          } else {
              last_foot_pos_ = base_pos_ + r_position;
              last_foot_quat_ = r_rotation * base_quat_;
              last_foot_ang_ = base_ang_ + r_eulerAngles;
          }
          last_contact_point_index_ = 1;
      }
      ros_logger_->publishValue("/state_estimate/kinematic/last_contact_point_index_", last_contact_point_index_);
      Eigen::Vector3d base_vel_ = (base_pos_ - last_base_pos_) / dt;
      Eigen::Vector3d base_angl_vel_ = (base_ang_ - last_base_ang_) / dt;
      last_base_pos_ = base_pos_;
      last_base_ang_ = base_ang_;
      Eigen::Vector3d base_quaternion_;
      Eigen::Vector3d imu_quaternion_;
      Eigen::Vector3d update_eulerAngles;
      base_quaternion_ = quatToZyx(base_quat_);
      imu_quaternion_ = quatToZyx(imu_quat);
      update_eulerAngles << base_quaternion_(0), imu_quaternion_(1), imu_quaternion_(2);
      base_quat_ = Eigen::AngleAxisd(update_eulerAngles[0], Eigen::Vector3d::UnitZ())*
                   Eigen::AngleAxisd(update_eulerAngles[1], Eigen::Vector3d::UnitY())*
                   Eigen::AngleAxisd(update_eulerAngles[2], Eigen::Vector3d::UnitX());
      nav_msgs::Odometry kinematics_odom;
      kinematics_odom.pose.pose.position.x = base_pos_(0);
      kinematics_odom.pose.pose.position.y = base_pos_(1);
      kinematics_odom.pose.pose.position.z = base_pos_(2);
      kinematics_odom.pose.pose.orientation.w = base_quat_.w();
      kinematics_odom.pose.pose.orientation.x = base_quat_.x();
      kinematics_odom.pose.pose.orientation.y = base_quat_.y();
      kinematics_odom.pose.pose.orientation.z = base_quat_.z();
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          kinematics_odom.pose.covariance[i * 6 + j] = 0;
          kinematics_odom.pose.covariance[6 * (3 + i) + (3 + j)] = 0;
        }
      }
      kinematics_odom.pose.covariance[6 * (3 + 0) + (3 + 0)] = 0.02;
      kinematics_odom.pose.covariance[6 * (3 + 1) + (3 + 1)] = 0.02;
      kinematics_odom.pose.covariance[6 * (3 + 2) + (3 + 2)] = 0.02;
      //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "dummy_link"
      kinematics_odom.twist.twist.linear.x = base_vel_(0);
      kinematics_odom.twist.twist.linear.y = base_vel_(1);
      kinematics_odom.twist.twist.linear.z = base_vel_(2);
      kinematics_odom.twist.twist.angular.x = base_angl_vel_(0);
      kinematics_odom.twist.twist.angular.y = base_angl_vel_(1);
      kinematics_odom.twist.twist.angular.z = base_angl_vel_(2);
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          kinematics_odom.twist.covariance[i * 6 + j] = 0;
          kinematics_odom.twist.covariance[6 * (3 + i) + (3 + j)] = 0;
        }
      }
      kinematics_odom.header.stamp = time;
      kinematics_odom.header.frame_id = "odom";
      kinematics_odom.child_frame_id = "dummy_link";
      return kinematics_odom;
    }

    // not used
    bool KalmanFilterEstimate::updateKinematicsRL(const ros::Time &time, const ros::Duration &period)
    {
      scalar_t dt = period.toSec();
      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();
      size_t actuatedDofNum = info_.actuatedDofNum;
      vector_t qPino(info_.generalizedCoordinatesNum);
      vector_t vPino(info_.generalizedCoordinatesNum);
      qPino.setZero();
      qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);
      vPino.setZero();
      vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
          qPino.segment<3>(3),
          rbdState_.segment<3>(info_.generalizedCoordinatesNum)); // Only set angular velocity, let linear velocity be zero
      vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);
      pinocchio::forwardKinematics(model, data, qPino, vPino);
      pinocchio::updateFramePlacements(model, data);
      int leg_l6_link = model.getFrameId("leg_l6_link");
      int leg_r6_link = model.getFrameId("leg_r6_link");
      int base_link = model.getFrameId("base_link");
      Eigen::Vector3d l_position = data.oMf[leg_l6_link].translation();
      Eigen::Vector3d r_position = data.oMf[leg_r6_link].translation();
      bool isLeftContact = contactFlag_[0] && contactFlag_[1] && contactFlag_[2] && contactFlag_[3];
      bool isRightContact = contactFlag_[4] && contactFlag_[5] && contactFlag_[6] && contactFlag_[7];
      if (isLeftContact && isRightContact) {
          isRightContact = false;
      }
      if (isLeftContact) {
          if (last_contact_point_index_ == 0) {
              base_pos_ = last_foot_pos_ - l_position;
          } else {
              last_foot_pos_ = base_pos_ + l_position;
          }
          last_contact_point_index_ = 0;
      } 
      else if (isRightContact) {
          if (last_contact_point_index_ == 1) {
              base_pos_ = last_foot_pos_ - r_position;
          } else {
              last_foot_pos_ = base_pos_ + r_position;
          }
          last_contact_point_index_ = 1;
      }
      // ros_logger_->publishValue("/state_estimate/kinematic/last_contact_point_index_", last_contact_point_index_);
      last_base_pos_ = base_pos_;
      Eigen::VectorXd kinematicState_(9);
      kinematicState_ << base_pos_, l_position, r_position;
      double lfootX_rfootX_Diff;
      double lfootY_rfootY_Diff;
      double lfootY_rfootY_Bias;
      bool Trotgait_;
      lfootX_rfootX_Diff = abs(kinematicState_(3) - kinematicState_(6));
      lfootY_rfootY_Diff = abs(kinematicState_(4) - kinematicState_(7));
      lfootY_rfootY_Bias = (kinematicState_(4) + kinematicState_(7)) / 2;
      if(lfootY_rfootY_Bias < 0.04 && lfootX_rfootX_Diff < 0.02 && lfootY_rfootY_Diff > 0.17)
      {
        Trotgait_ = false;
      }
      else
      {
        Trotgait_ = true;
      }
      // ros_logger_->publishVector("/state_estimate/kinematicState/base_pos_xyz", kinematicState_.head(3));
      ros_logger_->publishVector("/state_estimate/kinematicState/l_foot_pos_xyz", kinematicState_.segment(3, 3));
      ros_logger_->publishVector("/state_estimate/kinematicState/r_foot_pos_xyz", kinematicState_.tail(3));
      ros_logger_->publishValue("/state_estimate/kinematicState/l_footY_r_footY_bias", lfootY_rfootY_Bias);
      ros_logger_->publishValue("/state_estimate/kinematicState/l_footY_r_footY_diff", lfootY_rfootY_Diff);
      ros_logger_->publishValue("/state_estimate/kinematicState/l_footX_r_footX_diff", lfootX_rfootX_Diff);
      ros_logger_->publishValue("/state_estimate/kinematicState/trot_gait", Trotgait_);
      return Trotgait_;
    }

    vector_t KalmanFilterEstimate::update(const ros::Time &time, const ros::Duration &period)
    {
      scalar_t dt = period.toSec();
      a_.block(0, 3, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
      b_.block(0, 0, 3, 3) = 0.5 * dt * dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
      b_.block(3, 0, 3, 3) = dt * Eigen::Matrix<scalar_t, 3, 3>::Identity();
      q_.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
      q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * Eigen::Matrix<scalar_t, 3, 3>::Identity();
      q_.block(6, 6, num_contact_dof, num_contact_dof) = dt * Eigen::Matrix<scalar_t, num_contact_dof, num_contact_dof>::Identity();

      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();
      size_t actuatedDofNum = info_.actuatedDofNum;

      vector_t qPino(info_.generalizedCoordinatesNum);
      vector_t vPino(info_.generalizedCoordinatesNum);
      qPino.setZero();
      qPino.segment<3>(3) = rbdState_.head<3>(); // Only set orientation, let position in origin.
      // qPino.head<3>() = rbdState_.segment<3>(3);
      qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

      vPino.setZero();
      vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
          qPino.segment<3>(3),
          rbdState_.segment<3>(info_.generalizedCoordinatesNum)); // Only set angular velocity, let linear velocity be zero
      vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

      pinocchio::forwardKinematics(model, data, qPino, vPino);
      pinocchio::updateFramePlacements(model, data);


      const auto eePos = eeKinematics_->getPosition(vector_t());
      const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());
      
      // 存储足端位置（8个接触点的xyz坐标拼接）
      // 将std::vector<Vector3d>转换为vector_t，并转换为世界系坐标
      Eigen::Vector3d basePosition = rbdState_.segment<3>(3);
      endEffectorPositions_.resize(eePos.size() * 3);
      for (size_t i = 0; i < eePos.size(); ++i)
      {
        // 将足端位置从base系转换到世界系
        endEffectorPositions_.segment<3>(i * 3) = eePos[i] + basePosition;
      }

      // the covariance of the process noise
      Eigen::Matrix<scalar_t, num_q_est, num_q_est> q = Eigen::Matrix<scalar_t, num_q_est, num_q_est>::Identity();
      if (gait_ == "stance")
      {
        q.block(0, 0, 2, 2) = q_.block(0, 0, 2, 2) * imuProcessNoisePosition_;
        q.block(2, 2, 1, 1) = q_.block(2, 2, 1, 1) * imuProcessNoiseZPosition_;
        q.block(3, 3, 2, 2) = q_.block(3, 3, 2, 2) * imuProcessNoiseVelocity_;
        q.block(5, 5, 1, 1) = q_.block(5, 5, 1, 1) * imuProcessNoiseZVelocity_;
      }
      else
      {
        q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imuProcessNoisePosition_;
        q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imuProcessNoiseVelocity_;
      }
      q.block(6, 6, num_contact_dof, num_contact_dof) = q_.block(6, 6, num_contact_dof, num_contact_dof) * footProcessNoisePosition_;

      // the covariance of the observation noise
      Eigen::Matrix<scalar_t, num_r_est, num_r_est> r = Eigen::Matrix<scalar_t, num_r_est, num_r_est>::Identity();
      r.block(0, 0, num_contact_dof, num_contact_dof) = r_.block(0, 0, num_contact_dof, num_contact_dof) * footSensorNoisePosition_;
      r.block(num_contact_dof, num_contact_dof, num_contact_dof, num_contact_dof) = r_.block(num_contact_dof, num_contact_dof, num_contact_dof, num_contact_dof) * footSensorNoiseVelocity_;
      const int fn = info_.numThreeDofContacts;
      r.block(num_contact_dof * 2, num_contact_dof * 2, fn, fn) = r_.block(num_contact_dof * 2, num_contact_dof * 2, fn, fn) * footHeightSensorNoise_;

      for (int i = 0; i < info_.numThreeDofContacts; i++)
      {
        int i1 = 3 * i;

        int qIndex = 6 + i1;
        int rIndex1 = i1;
        int rIndex2 = num_contact_dof + i1;
        int rIndex3 = num_contact_dof * 2 + i;
        bool isContact = contactFlag_[i];

        scalar_t high_suspect_number(1000);
        q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
        r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3); // position
        r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3); // velocity
        r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);                         // foot height

        ps_.segment(3 * i, 3) = -eePos[i];
        ps_.segment(3 * i, 3)[2] += footRadius_;
        vs_.segment(3 * i, 3) = -eeVel[i];

        double foot_height_desired = foot_pos_desired_[i](2);
        // std::cout << "foot_height_desired: " << foot_height_desired << std::endl;
        if(isFixHeight_)
        {
          feetHeights_[i] = 0;
        }
        else
        {
          if (resetGroundHeight_)
            feetHeights_[i] = (isContact) ? foot_height_desired : (eePos[i][2] + rbdState_.segment<3>(3)[2]);
          else
            feetHeights_[i] = (eePos[i][2] + rbdState_.segment<3>(3)[2]);


        }
        ros_logger_->publishVector("/state_estimate/end_effector/contact_point_" + std::to_string(i+1) + "/pos", ps_.segment(3 * i, 3));
        ros_logger_->publishVector("/state_estimate/end_effector/contact_point_" + std::to_string(i+1) + "/vel", vs_.segment(3 * i, 3));
        ros_logger_->publishValue("/state_estimate/end_effector/contact_point_" + std::to_string(i+1) + "/feet_heights", feetHeights_(i));
      }
      // std::cout << "feetHeights_"<<feetHeights_.transpose() << std::endl;

      vector3_t g(0, 0, -9.785);
      vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linearAccelLocal_ + g;
      // std::cout <<"\nxHat_:"<<xHat_.transpose()<<std::endl;
      // observation (or measurement)
      Eigen::Matrix<scalar_t, num_r_est, 1> y;
      y << ps_, vs_, feetHeights_;
      // ros_logger_->publishVector("/humanoid_controller/estimate/y", y);

      xHat_ = a_ * xHat_ + b_ * accel; // 先验估计，状态转移矩阵x上一个状态+控制矩阵x输入, 使用状态方程预测下一个时刻的状态
      Eigen::Matrix<scalar_t, num_q_est, num_q_est> at = a_.transpose();
      Eigen::Matrix<scalar_t, num_q_est, num_q_est> pm = a_ * p_ * at + q;
      Eigen::Matrix<scalar_t, num_q_est, num_r_est> cT = c_.transpose();
      Eigen::Matrix<scalar_t, num_r_est, 1> yModel = c_ * xHat_;
      Eigen::Matrix<scalar_t, num_r_est, 1> ey = y - yModel;
      Eigen::Matrix<scalar_t, num_r_est, num_r_est> s = c_ * pm * cT + r;

      Eigen::Matrix<scalar_t, num_r_est, 1> sEy = s.lu().solve(ey);// sEy = s^-1 * ey
      xHat_ += pm * cT * sEy;// P * C^T * s^(-1) * ey，等效于 K * (y - C * x)，K = P * C^T * s^(-1)

      Eigen::Matrix<scalar_t, num_r_est, num_q_est> sC = s.lu().solve(c_);
      p_ = (Eigen::Matrix<scalar_t, num_q_est, num_q_est>::Identity() - pm * cT * sC) *
           pm;

      Eigen::Matrix<scalar_t, num_q_est, num_q_est> pt = p_.transpose();
      p_ = (p_ + pt) / 2.0;//确保对称性：消除数值误差产生的非对称性
      double reset_threshold = 0.000001;

      bool is_stance = gait_ == "stance";
      ros_logger_->publishValue("/state_estimate/estimate/is_stance", is_stance);
      ros_logger_->publishValue("/state_estimate/estimate/determinant", p_.block(0, 0, 2, 2).determinant());
      if (p_.block(0, 0, 2, 2).determinant() > reset_threshold || is_stance)// 行列式大于n, 协方差过大，重置p_
      {
        
        // TODO: 临时解决imu站立飘移问题，stance 时，不重置足端估计相关维度
        // if (is_stance)
        {
          p_.block(0, 2, 2, 4).setZero();
          p_.block(2, 0, 4, 2).setZero();
        }
        // if (p_.block(0, 0, 2, 2).determinant() > 150)
        // {
        //   std::cout << "[KalmanFilterEstimate] reset p_" << std::endl;
        //   p_.block(0, 2, 2, num_q_est-2).setZero();
        //   p_.block(2, 0, num_q_est-2, 2).setZero();
        // }
        // p_.block(0, 0, 2, 2) /= 10.0;
        p_.block(0, 0, 2, 2).setZero();

      }

      if (topicUpdated_)
      {
        std::cout << "topic updated" << std::endl;
        updateFromTopic();
        topicUpdated_ = false;
      }

      updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

      auto odom = getOdomMsg();
      odom.header.stamp = time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      publishMsgs(odom);

      return rbdState_;
    }

    void KalmanFilterEstimate::updateFromTopic()
    {
      auto *msg = buffer_.readFromRT();

      tf2::Transform world2sensor;
      world2sensor.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
      world2sensor.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                               msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));

      tf2::Transform base2sensor;
      try
      {
        geometry_msgs::TransformStamped tf_msg =
            tfBuffer_.lookupTransform("dummy_link", msg->child_frame_id, msg->header.stamp);
        tf2::fromMsg(tf_msg.transform, base2sensor);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        return;
      }
      tf2::Transform odom2base = world2odom_.inverse() * world2sensor * base2sensor.inverse();
      vector3_t newPos(odom2base.getOrigin().x(), odom2base.getOrigin().y(), odom2base.getOrigin().z());

      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();

      vector_t qPino(info_.generalizedCoordinatesNum);
      qPino.head<3>() = newPos;
      qPino.segment<3>(3) = rbdState_.head<3>();
      qPino.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
      pinocchio::forwardKinematics(model, data, qPino);
      pinocchio::updateFramePlacements(model, data);
      xHat_.segment<3>(0) = newPos;
      for (size_t i = 0; i < num_contact_points_est; ++i)
      {
        xHat_.segment<3>(6 + i * 3) = eeKinematics_->getPosition(vector_t())[i];
        xHat_(6 + i * 3 + 2) -= footRadius_;
        if (contactFlag_[i])
        {
          feetHeights_[i] = xHat_(6 + i * 3 + 2);
        }
      }

      auto odom = getOdomMsg();
      odom.header = msg->header;
      odom.child_frame_id = "base_link";
      publishMsgs(odom);
    }

    void KalmanFilterEstimate::callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
      buffer_.writeFromNonRT(*msg);
      topicUpdated_ = true;
    }

    nav_msgs::Odometry KalmanFilterEstimate::getOdomMsg()
    {
      nav_msgs::Odometry odom;
      odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
      odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
      odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
      odom.pose.pose.orientation.x = quat_.x();
      odom.pose.pose.orientation.y = quat_.y();
      odom.pose.pose.orientation.z = quat_.z();
      odom.pose.pose.orientation.w = quat_.w();
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          odom.pose.covariance[i * 6 + j] = p_(i, j);
          odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
        }
      }
      //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "dummy_link"
      vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
      odom.twist.twist.linear.x = twist.x();
      odom.twist.twist.linear.y = twist.y();
      odom.twist.twist.linear.z = twist.z();
      odom.twist.twist.angular.x = angularVelLocal_.x();
      odom.twist.twist.angular.y = angularVelLocal_.y();
      odom.twist.twist.angular.z = angularVelLocal_.z();
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
          odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
        }
      }
      return odom;
    }

    void KalmanFilterEstimate::loadSettings(const std::string &taskFile, bool verbose, const std::string &referenceFile)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "kalmanFilter.";
      if (verbose)
      {
        std::cerr << "\n #### Kalman Filter Noise:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", verbose);
      loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", verbose);
      loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", verbose);
      loadData::loadPtreeValue(pt, imuProcessNoiseZPosition_, prefix + "imuProcessNoiseZPosition", verbose);
      loadData::loadPtreeValue(pt, imuProcessNoiseZVelocity_, prefix + "imuProcessNoiseZVelocity", verbose);
      loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
      loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
      loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
      loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
      loadData::loadPtreeValue(pt, resetGroundHeight_, prefix + "resetGroundHeight", verbose);

      StateEstimateBase::loadSettings(taskFile, verbose, referenceFile);
    }

    vector3_t KalmanFilterEstimate::getFeetCenterPosition() const
    {
      if (endEffectorPositions_.size() < 24) // 8个接触点 * 3维坐标 = 24
      {
        return vector3_t::Zero();
      }
      
      vector3_t center = vector3_t::Zero();
      int numPoints = endEffectorPositions_.size() / 3;
      
      for (int i = 0; i < numPoints; ++i)
      {
        center += endEffectorPositions_.segment<3>(i * 3);
      }
      
      center /= numPoints;
      return center;
    }


  } // namespace humanoid
} // namespace ocs2
