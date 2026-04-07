//
// Created by qiayuan on 2022/7/1.
//

// some ref: https://github.com/skywoodsz/qm_control

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "humanoid_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <pinocchio/algorithm/centroidal.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <utility>


namespace ocs2
{
  namespace humanoid
  {
    WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics &eeKinematics)
        : pinocchioInterfaceMeasured_(pinocchioInterface), pinocchioInterfaceDesired_(pinocchioInterface), info_(std::move(info)), mapping_(info_), inputLast_(vector_t::Zero(info_.inputDim)), eeKinematics_(eeKinematics.clone()), rbdConversions_(pinocchioInterface, info_)
    {
      Eigen::setNbThreads(1);  // No multithreading within Eigen.
      Eigen::initParallel();
      
      // linear force, plus angular force
      contact_force_size_ = 3 * info_.numThreeDofContacts;
      // 决策变量, 6*body_acc + 12*joint_acc + 3x4*contact_force + 12*torque = 42
      // generalizedCoordinatesNum: pinocchioInterface.getGeneralizedCoordinatesNum(), 广义坐标数
      numDecisionVars_ = info_.generalizedCoordinatesNum + contact_force_size_ + info_.actuatedDofNum; // 27+3*8+21=72
      qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
      vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
      // vd_measured_ = vector_t(info_.generalizedCoordinatesNum);
      // vd_measured_.setZero();
      cmd_body_pos_.resize(6);
      cmd_body_pos_.setZero();
      cmd_body_vel_.resize(6);
      cmd_body_vel_.setZero();
      earlyLatecontact_[0].fill(false);
      earlyLatecontact_[1].fill(false);
      topic_logger_ = new TopicLogger;
      inputDesired_prev_ = vector_t::Zero(info_.inputDim);
      jointAccelerations_ = vector_t::Zero(info_.actuatedDofNum);
      velDesired_prev_.resize(info_.numThreeDofContacts);
      eeAccDesired_.resize(info_.numThreeDofContacts);
    }

    vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                             size_t mode, scalar_t period, bool mpc_update)
    {
      period_ = period;
      mpc_updated_ = mpc_update;
      if (mode != mode_)
      {
        after_change_index_ = 0;
        ROS_INFO_STREAM("mode changed to " << modeNumber2String(mode));
        mode_ = mode;
      }
      if (after_change_index_++ < 5)
      {
        mpc_updated_ = true;
      }
      contactFlag_ = modeNumber2StanceLeg(mode);
      // std::cout << "contactFlag_:" << contactFlag_ << std::endl;
      numContacts_ = 0;
      for (bool flag : contactFlag_)
      {
        if (flag)
        {
          numContacts_++;
        }
      }
      if (!mpc_updated_)
      {
        // jointAccelerations_ = (inputDesired.tail(info_.actuatedDofNum) - inputDesired_prev_.tail(info_.actuatedDofNum)) / period;
      }
      inputDesired_prev_ = inputDesired;

      // topic_logger_->publishVector("/humanoid_controller/jointAccelerations_", jointAccelerations_);

      updateMeasured(rbdStateMeasured);
      updateDesired(stateDesired, inputDesired);
      vd_measured_ = (vMeasured_ - last_v_measured_) / period;
      vd_des_fd_ = (qd_des_ - last_v_des_) / period;
      last_v_des_ = qd_des_;
      last_v_measured_ = vMeasured_;

      // precomputate ee
      preComputation();

      return {};
    }

    void WbcBase::updateMeasured(const vector_t &rbdStateMeasured)
    {
      qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3); // xyz linaer pos
      qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
      qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
      vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
      vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
          qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
      vMeasured_.tail(info_.actuatedDofNum) =
          rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

      const auto &model = pinocchioInterfaceMeasured_.getModel();
      auto &data = pinocchioInterfaceMeasured_.getData();

      // For floating base EoM task
      pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
      pinocchio::computeJointJacobians(model, data);
      pinocchio::updateFramePlacements(model, data);
      pinocchio::crba(model, data, qMeasured_);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
      pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
      j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
      }
      

      pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
      dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                                 pinocchio::LOCAL_WORLD_ALIGNED, jac);
        dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
      }
      if (info_.numSixDofContacts > 0)
      {
        j_hand_ = matrix_t(6 * info_.numSixDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numSixDofContacts; ++i)
        {
          Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
          jac.setZero(6, info_.generalizedCoordinatesNum);
          pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[info_.numThreeDofContacts + i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
          j_hand_.block(6 * i, 0, 6, info_.generalizedCoordinatesNum) = jac;
        }

        dj_hand_ = matrix_t(6 * info_.numSixDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numSixDofContacts; ++i)
        {
          Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
          jac.setZero(6, info_.generalizedCoordinatesNum);
          pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[info_.numThreeDofContacts + i],
                                                   pinocchio::LOCAL_WORLD_ALIGNED, jac);
          dj_hand_.block(6 * i, 0, 6, info_.generalizedCoordinatesNum) = jac;
        }
      }

      // For base motion tracking task
      base_j_.setZero(6, info_.generalizedCoordinatesNum);
      base_dj_.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, model.getBodyId("dummy_link"), pinocchio::LOCAL_WORLD_ALIGNED, base_j_);
      pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("dummy_link"), pinocchio::LOCAL_WORLD_ALIGNED,
                                               base_dj_);
      // For center of mass task
      pinocchio::centerOfMass(model, data, qMeasured_, vMeasured_);
      r = data.com[0];
      rd = data.vcom[0];
      rdd = data.acom[0];
      topic_logger_->publishVector("/humanoid_controller/com/r", r);
      topic_logger_->publishVector("/humanoid_controller/com/rd", rd);
      Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> dJs_dt;
      // pinocchio::jacobianCenterOfMassTimeVariation(model, data, qMeasured_, vMeasured_, dJs_dt);
      // Compute centroidal quantities
      Js_v_Ccm_ = pinocchio::jacobianCenterOfMass(model, data, qMeasured_);
      updateCentroidalDynamics(pinocchioInterfaceMeasured_, info_, qMeasured_);
      A = pinocchio::ccrba(model, data, qMeasured_, vMeasured_);
      ADot = pinocchio::dccrba(model, data, qMeasured_, vMeasured_);
      //  std::cout << "qMeasured_:\n" << std::fixed << std::setprecision(5)<< qMeasured_.transpose() << std::endl;
      //  std::cout << "vMeasured_:\n" << std::fixed << std::setprecision(5)<< vMeasured_.transpose() << std::endl;
    }

    void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
    {
      const auto &model = pinocchioInterfaceDesired_.getModel();
      auto &data = pinocchioInterfaceDesired_.getData();

      mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
      const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
      pinocchio::forwardKinematics(model, data, qDesired);
      pinocchio::computeJointJacobians(model, data, qDesired);
      pinocchio::updateFramePlacements(model, data);
      updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
      const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
      pinocchio::forwardKinematics(model, data, qDesired, vDesired);

      rbdConversions_.computeBaseKinematicsFromCentroidalModel(stateDesired, inputDesired, jointAccelerations_, basePoseDes_,
                                                               baseVelocityDes_, baseAccelerationDes_);
      // For center of mass task
      pinocchio::centerOfMass(model, data, qDesired, vDesired);
      r_des = data.com[0];
      rd_des = data.vcom[0];
      q_des_ = qDesired;
      qd_des_ = vDesired;

      topic_logger_->publishVector("/humanoid_controller/com/r_des", r_des);
      topic_logger_->publishVector("/humanoid_controller/com/rd_des", rd_des);
      // std::cout << "qDesired:\n"<< std::fixed << std::setprecision(5)<<qDesired.transpose()<<std::endl;
      // std::cout << "vDesired:\n"<< std::fixed << std::setprecision(5)<<vDesired.transpose()<<std::endl;

      {
        // pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
        pinocchio::forwardKinematics(model, data, qDesired, vDesired);
        pinocchio::computeJointJacobians(model, data);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::crba(model, data, qDesired);
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        pinocchio::nonLinearEffects(model, data, qDesired, vDesired);
        j_des_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
          Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
          jac.setZero(6, info_.generalizedCoordinatesNum);
          pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
          j_des_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }

        pinocchio::computeJointJacobiansTimeVariation(model, data, qDesired, vDesired);
        dj_des_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
          Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
          jac.setZero(6, info_.generalizedCoordinatesNum);
          pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                                   pinocchio::LOCAL_WORLD_ALIGNED, jac);
          dj_des_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }
      }
    }

    Task WbcBase::formulateFloatingBaseEomTask(const vector_t &inputDesired)
    {
      auto &data = pinocchioInterfaceMeasured_.getData();
      matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
      s.block(0, 0, info_.actuatedDofNum, 6).setZero();
      s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
      matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose())
                       .finished();
      vector_t b = -data.nle;
      if (info_.numSixDofContacts > 0)
        b += j_hand_.transpose() * inputDesired.segment(info_.numThreeDofContacts * 3, info_.numSixDofContacts * 6);

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorqueLimitsTask()
    {
      matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
      d.setZero();
      matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
      d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
              info_.actuatedDofNum) = i;
      d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
              info_.actuatedDofNum) = -i;
      vector_t f(2 * info_.actuatedDofNum);
      vector_t all_joint_limits(info_.actuatedDofNum);
      all_joint_limits << torqueLimits_.head(6), torqueLimits_.head(6), torqueLimits_.segment(6, waist_nums_), torqueLimits_.tail(arm_nums_/2), torqueLimits_.tail(arm_nums_/2);
      f << all_joint_limits, all_joint_limits;
      // const int dofPerLeg = info_.actuatedDofNum / 2;
      // for (size_t l = 0; l < 2 * info_.actuatedDofNum / dofPerLeg; ++l)
      // { 
      //   f.segment(dofPerLeg * l, dofPerLeg) = torqueLimits_;
      // }
      return {matrix_t(), vector_t(), d, f};
    }

    Task WbcBase::formulateNoContactMotionTask()
    {
      matrix_t a(3 * numContacts_, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();
      size_t j = 0;
      for (size_t i = 0; i < info_.numThreeDofContacts; i++)
      {
        if (contactFlag_[i])
        {

          // vector3_t accel = swingKp_ * (eePosDesired_[i] - eePosMeasured_[i]) + swingKd_ * (eeVelDesired_[i] - eeVelMeasured_[i]);
          // a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
          // b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
          // j++;
          // if (stance_mode_)
          vector3_t accel = stanceKp_ * (eePosDesired_[i] - eePosMeasured_[i]) + stanceKd_ * (eeVelDesired_[i] - eeVelMeasured_[i]);
          a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
          b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
          j++;
        }
      }

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateFrictionConeTask()
    {
      matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
      a.setZero();
      size_t j = 0;
      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        if (!contactFlag_[i])
        {
          a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
      }
      vector_t b(a.rows());
      b.setZero();

      matrix_t frictionPyramic(5, 3); // clang-format off
      frictionPyramic << 0,  0, -1,
                         1,  0, -frictionCoeff_,
                         -1, 0, -frictionCoeff_,
                         0,  1, -frictionCoeff_,
                         0, -1, -frictionCoeff_; // clang-format on

      matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
      d.setZero();
      j = 0;
      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        if (contactFlag_[i])
        {
          d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
        }
      }
      vector_t f = Eigen::VectorXd::Zero(d.rows());

      return {a, b, d, f};
    }

    // Tracking base xy linear motion task
    Task WbcBase::formulateBaseXYLinearAccelTask()
    {
      // 构造最小二乘问题 min 0.5 * ||ax - b||^2, 优化变量为base xy加速度
      matrix_t a(2, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();

      a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);
      b = baseAccelerationDes_.segment<2>(0);

      return {a, b, matrix_t(), vector_t()};
    }

    // Tracking base height motion task
    Task WbcBase::formulateBaseHeightMotionTask()
    {
      // 构造最小二乘问题 min 0.5 * ||ax - b||^2, 优化变量为base的高度
      matrix_t a(1, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();
      a.block(0, 2, 1, 1).setIdentity(); // 只控制决策变量的高度项

      b[0] = baseAccelerationDes_[2] + baseHeightKp_ * (basePoseDes_[2] - qMeasured_[2]) +
             baseHeightKd_ * (baseVelocityDes_[2] - vMeasured_[2]); // 线性项由pd反馈得到

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateCenterOfMassTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
      matrix_t a(3, numDecisionVars_);
      vector_t b(a.rows());
      a.setZero();
      b.setZero();
      matrix_t rotationYawMeasuredToBase = rotationYawBaseMeasuredToWorld_.inverse();
      Eigen::VectorXd x_d(6);
      x_d << rotationYawMeasuredToBase * r_des, rotationYawMeasuredToBase * rd_des;
      Eigen::VectorXd x(6);
      x << rotationYawMeasuredToBase * r, rotationYawMeasuredToBase * rd;
      vector_t rdd_d = Wbc_rdd_K_ * (x_d - x);
      rdd_d = rotationYawBaseMeasuredToWorld_ * rdd_d;

      Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
      Eigen::Vector3d centroidalMomentumRate_lin = centroidalMomentumRate.segment(0, 3);
      a.block(0, 0, 3, info_.generalizedCoordinatesNum) = A.block(0, 0, 3, info_.generalizedCoordinatesNum);
      b = centroidalMomentumRate_lin + info_.robotMass * rdd_d - ADot.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      return {a, b, matrix_t(), vector_t()};
    }
    // Tracking base angular motion task
    Task WbcBase::formulateBaseAngularMotionTask()
    {
      // 构造最小二乘问题 min 0.5 * ||ax - b||^2, 优化变量为base的角速度
      matrix_t a(3, numDecisionVars_);
      vector_t b(a.rows());

      a.setZero();
      b.setZero();

      a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

      vector3_t eulerAngles = qMeasured_.segment<3>(3);

      // from derivative euler to angular
      vector3_t vMeasuredGlobal =
          getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
      vector3_t vDesiredGlobal = baseVelocityDes_.tail<3>();

      // from euler to rotation
      vector3_t eulerAnglesDesired = basePoseDes_.tail<3>();
      matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
      matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

      vector3_t p_err_world = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);
      vector3_t v_err_world = vDesiredGlobal - vMeasuredGlobal;

      matrix_t rotationYawMeasuredToBase = rotationYawBaseMeasuredToWorld_.inverse();
      vector3_t p_err = rotationYawMeasuredToBase * p_err_world;
      vector3_t v_err = rotationYawMeasuredToBase * v_err_world;

      // desired acc
      vector3_t accDesired = baseAccelerationDes_.tail<3>();

      b = accDesired + rotationBaseMeasuredToWorld * (baseAngular3dKp_.cwiseProduct(p_err)) + rotationBaseMeasuredToWorld * (baseAngular3dKd_.cwiseProduct(v_err)) -
          base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      // std::cout << "accDesired: \n" << accDesired.transpose() << std::endl;
      // std::cout << "p_err: \n" << p_err.transpose() << std::endl;
      // std::cout << "(vDesiredGlobal - vMeasuredGlobal): \n" << (vDesiredGlobal - vMeasuredGlobal).transpose() << std::endl;
      // std::cout << "baseAngularTask: \n" << b.transpose() << std::endl;
      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
      return formulateBaseXYLinearAccelTask() + formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask();
    }
    void WbcBase::preComputation()
    {
      vector3_t euler4Transform = qMeasured_.segment<3>(3);
      euler4Transform.tail<2>().setZero();
      // std::cout << "euler4Transform: \n"
      //           << euler4Transform.transpose() << std::endl;
      rotationYawBaseMeasuredToWorld_ = getRotationMatrixFromZyxEulerAngles<scalar_t>(euler4Transform);

            
      auto angular_zyx = vector_t(3);
      angular_zyx << -qMeasured_.segment<3>(3)[0],0, 0;
      auto R = getRotationMatrixFromZyxEulerAngles<scalar_t>(angular_zyx);
      vector_t rd_base = R * rd;
      topic_logger_->publishVector("/humanoid_controller/com/rd_base", rd_base);
      

      // vector3_t tet;
      // tet << M_PI / 4, 0, 0;
      // auto testma = getRotationMatrixFromZyxEulerAngles<scalar_t>(tet);
      // vector3_t vec;
      // vec << 1, 0, 0.22;
      // std::cout << "testma * vec: " << testma * vec << std::endl;
      // std::cout << "testmmmm:" << testma.inverse() * vec << std::endl;
      eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
      eePosMeasured_ = eeKinematics_->getPosition(vector_t());
      eeVelMeasured_ = eeKinematics_->getVelocity(vector_t(), vector_t());
      eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
      eePosDesired_ = eeKinematics_->getPosition(vector_t());
      eeVelDesired_ = eeKinematics_->getVelocity(vector_t(), vector_t());
      auto eeData2Vector = [&](const std::vector<vector3_t> &data)
      {
        vector_t result(data.size() * data[0].rows());
        for (size_t i = 0; i < data.size(); i++)
        {
          result.segment(i * data[0].rows(), data[0].rows()) = data[i];
        }
        return result;
      };
      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        if (!mpc_updated_)
          eeAccDesired_[i] = (eeVelDesired_[i] - velDesired_prev_[i]) / period_;
        eeAccDesired_[i][2] = 0;
      }

      auto avg_feet = [&](const std::vector<vector3_t> &data)
      {
          vector_t result(3 * 2); // 左右脚的平均值
          {
              vector3_t leftFootAvg = (data[0] + data[1] + data[2] + data[3]) / 4.0; // 
              vector3_t rightFootAvg = (data[4] + data[5] + data[6] + data[7]) / 4.0; // 
              
              result.segment<3>(0) = leftFootAvg; // 
              result.segment<3>(3) = rightFootAvg; // 
          }
          return result;
      };
      auto avg_feet_pos = avg_feet(eePosMeasured_);
      // auto avg_feet_vel = avg_feet(eeVelMeasured_);
      auto avg_feet_pos_des = avg_feet(eePosDesired_);
      vector_t diff_r_lf = r - avg_feet_pos.segment<3>(0);
      vector_t diff_r_rf = r - avg_feet_pos.segment<3>(3);
      vector_t diff_r_lf_des = r - avg_feet_pos_des.segment<3>(0);
      vector_t diff_r_rf_des = r - avg_feet_pos_des.segment<3>(3);

      topic_logger_->publishVector("/humanoid_controller/com/com_lf_diff", R*diff_r_lf);
      topic_logger_->publishVector("/humanoid_controller/com/com_rf_diff", R*diff_r_rf);
      topic_logger_->publishVector("/humanoid_controller/com/com_lf_diff_des", R*diff_r_lf_des);
      topic_logger_->publishVector("/humanoid_controller/com/com_rf_diff_des", R*diff_r_rf_des);
      
          
      topic_logger_->publishVector("/humanoid_controller/swing_leg/pos_measured", eeData2Vector(eePosMeasured_));
      topic_logger_->publishVector("/humanoid_controller/swing_leg/vel_measured", eeData2Vector(eeVelMeasured_));
      topic_logger_->publishVector("/humanoid_controller/swing_leg/pos_desired", eeData2Vector(eePosDesired_));
      topic_logger_->publishVector("/humanoid_controller/swing_leg/vel_desired", eeData2Vector(eeVelDesired_));
      topic_logger_->publishVector("/humanoid_controller/swing_leg/acc_desired", eeData2Vector(eeAccDesired_));
      velDesired_prev_ = eeVelDesired_;
    }

    Task WbcBase::formulateStandUpJointAccelTask(const vector_t &stateDesied, const vector_t &inputDesired, scalar_t period)
    {
      matrix_t a(info_.actuatedDofNum, numDecisionVars_);
      vector_t b(a.rows());
      int leg_num = info_.actuatedDofNum - arm_nums_ - waist_nums_;
      // 先写为加速度控制任务  ddq = kp_arm * (q - qd) + kd_arm * (dq - dqd);
      a.setZero();
      b.setZero();
      a.block(0, info_.generalizedCoordinatesNum-info_.actuatedDofNum, info_.actuatedDofNum, info_.actuatedDofNum) = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
      b.head(leg_num) = standUp_legKp_ * (stateDesied.tail(info_.actuatedDofNum).head(leg_num) - qMeasured_.tail(info_.actuatedDofNum).head(leg_num)) + 
                        standUp_legKd_ * (inputDesired.tail(info_.actuatedDofNum).head(leg_num) - vMeasured_.tail(info_.actuatedDofNum).head(leg_num));
      b.segment(leg_num, waist_nums_) = standUp_waistKp_ * (stateDesied.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_) - qMeasured_.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_)) + 
                                        standUp_waistKd_ * (inputDesired.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_) - vMeasured_.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_));
      b.tail(arm_nums_) = standUp_armKp_ * (stateDesied.tail(arm_nums_) - qMeasured_.tail(arm_nums_)) + 
                          standUp_armKd_ * (inputDesired.tail(arm_nums_) - vMeasured_.tail(arm_nums_));

      // qMeasured_; vMeasured_; info_.generalizedCoordinatesNum; 
      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateJointAccelTask(const vector_t &stateDesied, const vector_t &inputDesired, scalar_t period)
    {
      int leg_num = info_.actuatedDofNum - arm_nums_ - waist_nums_;
      matrix_t a(leg_num, numDecisionVars_);
      vector_t b(a.rows());
      // 先写为加速度控制任务  ddq = kp_arm * (q - qd) + kd_arm * (dq - dqd);
      a.setZero();
      b.setZero();
      a.block(0, info_.generalizedCoordinatesNum-info_.actuatedDofNum, leg_num, leg_num) = matrix_t::Identity(leg_num, leg_num);
      b.head(leg_num) = jointAcc_Kp_ * (stateDesied.tail(info_.actuatedDofNum).head(leg_num) - qMeasured_.tail(info_.actuatedDofNum).head(leg_num)) + 
                        jointAcc_Kd_ * (inputDesired.tail(info_.actuatedDofNum).head(leg_num) - vMeasured_.tail(info_.actuatedDofNum).head(leg_num));

      // qMeasured_; vMeasured_; info_.generalizedCoordinatesNum; 
      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateWaistJointAccelTask(const vector_t &stateDesied, const vector_t &inputDesired, scalar_t period)
    {
      matrix_t a(waist_nums_, numDecisionVars_);
      vector_t b(a.rows());
      int leg_num = info_.actuatedDofNum - arm_nums_ - waist_nums_;
      // 先写为加速度控制任务  ddq = kp_arm * (q - qd) + kd_arm * (dq - dqd);
      a.setZero();
      b.setZero();
      a.block(0, info_.generalizedCoordinatesNum-info_.actuatedDofNum+leg_num, waist_nums_, waist_nums_) = matrix_t::Identity(waist_nums_, waist_nums_);
      b = waistJointKp_ * (stateDesied.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_) - qMeasured_.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_)) + 
          waistJointKd_ * (inputDesired.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_) - vMeasured_.tail(info_.actuatedDofNum).segment(leg_num, waist_nums_));

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateArmJointAccelTask(const vector_t &stateDesied, const vector_t &inputDesired, scalar_t period)
    {
      matrix_t a(arm_nums_, numDecisionVars_);
      vector_t b(a.rows());
      // 先写为加速度控制任务  ddq = kp_arm * (q - qd) + kd_arm * (dq - dqd);
      a.setZero();
      b.setZero();
      a.block(0, info_.generalizedCoordinatesNum-arm_nums_, arm_nums_, arm_nums_) = matrix_t::Identity(arm_nums_, arm_nums_);
      b = armJointKp_.cwiseProduct(stateDesied.tail(arm_nums_) - qMeasured_.tail(arm_nums_)) + 
          armJointKd_.cwiseProduct(inputDesired.tail(arm_nums_) - vMeasured_.tail(arm_nums_));

      // qMeasured_; vMeasured_; info_.generalizedCoordinatesNum; 
      return {a, b, matrix_t(), vector_t()};
    }
    
   

    Task WbcBase::formulateSwingLegTask()
    {

      matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
      vector_t b(a.rows());
      a.setZero();
      b.setZero();
      size_t j = 0;
      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        if (!contactFlag_[i])
        {
          vector3_t accel = swingKp3d_.cwiseProduct(eePosDesired_[i] - eePosMeasured_[i]) + swingKd3d_.cwiseProduct(eeVelDesired_[i] - eeVelMeasured_[i]);
          a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
          b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
          j++;
        }
      }

      return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateContactForceTask(const vector_t &inputDesired) const
    {
      matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
      vector_t b(a.rows());
      a.setZero();

      for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
      {
        a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      }
      b = inputDesired.head(a.rows());

      return {a, b, matrix_t(), vector_t()};
    }

    void WbcBase::compensateFriction(vector_t &x)
    {
      vector_t coulomb_friction(info_.actuatedDofNum);
      vector_t joint_v = vMeasured_.tail(info_.actuatedDofNum);
      for (int i = 0; i < info_.actuatedDofNum; i++)
      {
        const int sgn = (joint_v[i] > 0) - (joint_v[i] < 0);
        coulomb_friction[i] = (abs(joint_v[i]) > 0.001) ? (sgn * 0.2) : 0;
      }
      x.tail(12) = x.tail(12) + coulomb_friction;
    }

    void WbcBase::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      // Load task file
      torqueLimits_ = vector_t((info_.actuatedDofNum - waist_nums_) / 2 + waist_nums_);
      loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
      if (verbose)
      {
        std::cerr << "\n #### Torque Limits Task:";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "\n #### motor1, motor2, motor3, motor4, motor5, motor6: " << torqueLimits_.transpose() << "\n";
        std::cerr << " #### =============================================================================\n";
      }

      Wbc_rdd_K_ = matrix_t(3, 6);
      Wbc_rdd_K_stance_ = matrix_t(3, 6);
      Wbc_rdd_K_walk_ = matrix_t(3, 6);
      loadData::loadEigenMatrix(taskFile, "Wbc_rdd_K_.stance", Wbc_rdd_K_stance_);
      Wbc_rdd_K_ = Wbc_rdd_K_stance_;
      loadData::loadEigenMatrix(taskFile, "Wbc_rdd_K_.walk", Wbc_rdd_K_walk_);

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "frictionConeTask.";
      if (verbose)
      {
        std::cerr << "\n #### Friction Cone Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
      if (verbose)
      {
        std::cerr << " #### =============================================================================\n";
      }
      prefix = "swingLegTask.";
      if (verbose)
      {
        std::cerr << "\n #### Swing Leg Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      if (is_real)
      {
        prefix += "real.";
      }
      else
      {
        prefix += "sim.";
      }
      double kp_xy, kd_xy;
      loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
      loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
      loadData::loadPtreeValue(pt, kp_xy, prefix + "kp_xy", verbose);
      loadData::loadPtreeValue(pt, kd_xy, prefix + "kd_xy", verbose);
      
      swingKp3d_ << kp_xy, kp_xy, swingKp_;
      swingKd3d_ << kd_xy, kd_xy, swingKd_;
      
      prefix = "stanceLegTask." + std::string(is_real ? "real." : "sim.");
      loadData::loadPtreeValue(pt, stanceKp_, prefix + "kp", verbose);
      loadData::loadPtreeValue(pt, stanceKd_, prefix + "kd", verbose);

      prefix = "baseAccelTask.";
      if (verbose)
      {
        std::cerr << "\n #### Base Accel(Tracking) Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, com_kp_, prefix + "kp", verbose);
      loadData::loadPtreeValue(pt, com_kd_, prefix + "kd", verbose);

      prefix = "baseHeightTask.";
      if (verbose)
      {
        std::cerr << "\n #### Base Height Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, baseHeightKp_, prefix + "kp", verbose);
      loadData::loadPtreeValue(pt, baseHeightKd_, prefix + "kd", verbose);
      prefix = "baseAngularTask.";
      if (verbose)
      {
        std::cerr << "\n #### Base Angular Task:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadEigenMatrix(taskFile, prefix + "kp", baseAngular3dKp_);
      loadData::loadEigenMatrix(taskFile, prefix + "kd", baseAngular3dKd_);

      if(arm_nums_ != 0){
        prefix = "armAccelTask.";
        if(verbose)
        {
          std::cerr << "\n #### arm Accel Task:";
          std::cerr << "\n #### =============================================================================\n";
        }
        armJointKp_.resize(arm_nums_);
        armJointKd_.resize(arm_nums_);
        loadData::loadEigenMatrix(taskFile, prefix + "kp", armJointKp_);
        loadData::loadEigenMatrix(taskFile, prefix + "kd", armJointKd_);
      }

      if(waist_nums_ != 0){
        prefix = "waistAccelTask.";
        if(verbose)
        {
          std::cerr << "\n #### waist Accel Task:";
          std::cerr << "\n #### =============================================================================\n";
        }
        waistJointKp_.resize(waist_nums_);
        waistJointKd_.resize(waist_nums_);
        loadData::loadEigenMatrix(taskFile, prefix + "kp", waistJointKp_);
        loadData::loadEigenMatrix(taskFile, prefix + "kd", waistJointKd_);
      }

      prefix = "standUpJointAccelTask.";
      if(verbose)
      {
        std::cerr << "\n #### Stand Up Joint Accel Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      if (is_real)
      {
        loadData::loadPtreeValue(pt, standUp_legKp_, prefix + "leg.kp", verbose);
        loadData::loadPtreeValue(pt, standUp_legKd_, prefix + "leg.kd", verbose);
      }
      else
      {
        loadData::loadPtreeValue(pt, standUp_legKp_, prefix + "leg.kp_sim", verbose);
        loadData::loadPtreeValue(pt, standUp_legKd_, prefix + "leg.kd_sim", verbose);
      }
      loadData::loadPtreeValue(pt, standUp_armKp_, prefix + "arm.kp", verbose);
      loadData::loadPtreeValue(pt, standUp_armKd_, prefix + "arm.kd", verbose);
      loadData::loadPtreeValue(pt, standUp_waistKp_, prefix + "waist.kp", verbose);
      loadData::loadPtreeValue(pt, standUp_waistKd_, prefix + "waist.kd", verbose);

      prefix = "JointAccelTask.";
      loadData::loadPtreeValue(pt, jointAcc_Kp_, prefix + "kp", verbose);
      loadData::loadPtreeValue(pt, jointAcc_Kd_, prefix + "kd", verbose);
    }

    void WbcBase::loadSwitchParamsSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);

      std::string prefix = "switchJointAccelTask.";
      if(verbose)
      {
        std::cerr << "\n #### Switch Joint Accel Task:";
        std::cerr << "\n #### =============================================================================\n";
      }
      if (is_real)
      {
        loadData::loadPtreeValue(pt, standUp_legKp_, prefix + "leg.kp", verbose);
        loadData::loadPtreeValue(pt, standUp_legKd_, prefix + "leg.kd", verbose);
      }
      else
      {
        loadData::loadPtreeValue(pt, standUp_legKp_, prefix + "leg.kp_sim", verbose);
        loadData::loadPtreeValue(pt, standUp_legKd_, prefix + "leg.kd_sim", verbose);
      }
      loadData::loadPtreeValue(pt, standUp_armKp_, prefix + "arm.kp", verbose);
      loadData::loadPtreeValue(pt, standUp_armKd_, prefix + "arm.kd", verbose);
      loadData::loadPtreeValue(pt, standUp_waistKp_, prefix + "waist.kp", verbose);
      loadData::loadPtreeValue(pt, standUp_waistKd_, prefix + "waist.kd", verbose);
    }

  } // namespace humanoid
} // namespace ocs2
