/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "humanoid_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <humanoid_interface/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <numeric>

namespace ocs2
{
  namespace humanoid
  {

    /** DLS（阻尼最小二乘 / L2正则）求解：
     *      min_x ||A x - b||^2 + λ^2 ||x||^2
     *    闭式解：
     *      x = (AᵀA + λ²I)⁻¹ Aᵀ b
     *
     *  作用：
     *  - 近奇异时不会出现“伪逆数值爆炸”（小奇异值方向增益被抑制）
     *  - 不引入相位滞后（不使用历史数据，不做时间滤波，仅对当前帧做正则化）
     */
    static vector6_t dlsSolve6(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, double lambda) {
      // A: (m x 6), b: (m)
      matrix66_t AtA = A.transpose() * A;
      const scalar_t l2 = static_cast<scalar_t>(lambda * lambda);
      AtA.diagonal().array() += l2;
      vector6_t Atb = A.transpose() * b;
      // Use LDLT for symmetric positive definite / semidefinite + λ²I
      return AtA.ldlt().solve(Atb);
    }

    /** 基于SVD的阻尼伪逆（DLS形式）：
     *      A^+ = V * diag( s / (s^2 + λ^2) ) * U^T
     *  说明：与上面的 DLS 直接解等价，都是为了解决近奇异时的数值放大问题。
     */
    static Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& A, double lambda) {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const auto& s = svd.singularValues();
      Eigen::VectorXd s_damped = Eigen::VectorXd::Zero(s.size());
      const double lambda2 = lambda * lambda;
      for (int i = 0; i < s.size(); ++i) {
        const double si = s(i);
        s_damped(i) = (si > 1e-12) ? (si / (si * si + lambda2)) : 0.0;
      }
      return svd.matrixV() * s_damped.asDiagonal() * svd.matrixU().transpose();
    }

    Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &A, double epsilon = 1e-6) {
        // 进行 SVD 分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs()(0);
        
        // 计算 Sigma^+
        Eigen::VectorXd singularValues_inv = svd.singularValues();
        for (long i = 0; i < singularValues_inv.size(); ++i) {
            if (singularValues_inv(i) > tolerance)
                singularValues_inv(i) = 1.0 / singularValues_inv(i);
            else
                singularValues_inv(i) = 0;
        }

        // 计算 A^+ = V * Sigma^+ * U^T
        Eigen::MatrixXd Sigma_pinv = singularValues_inv.asDiagonal();
        return svd.matrixV() * Sigma_pinv * svd.matrixU().transpose();
    }
    bool verifyPseudoinverse(const Eigen::MatrixXd& A, const Eigen::MatrixXd& A_plus, double tol = 1e-6) {
        // 条件一：A * A_plus * A == A
        Eigen::MatrixXd condition1 = A * A_plus * A;
        bool cond1 = condition1.isApprox(A, tol);
        if (!cond1) {
            std::cerr << "条件一不满足：A * A+ * A != A\n";
        }

        // 条件二：A_plus * A * A_plus == A_plus
        Eigen::MatrixXd condition2 = A_plus * A * A_plus;
        bool cond2 = condition2.isApprox(A_plus, tol);
        if (!cond2) {
            std::cerr << "条件二不满足：A+ * A * A+ != A+\n";
        }

        // 条件三： (A * A_plus)^T == A * A_plus
        Eigen::MatrixXd AAplus = A * A_plus;
        Eigen::MatrixXd condition3 = AAplus.transpose();
        bool cond3 = condition3.isApprox(AAplus, tol);
        if (!cond3) {
            std::cerr << "条件三不满足：(A * A+)^T != A * A+\n";
        }

        // 条件四： (A_plus * A)^T == A_plus * A
        Eigen::MatrixXd AplusA = A_plus * A;
        Eigen::MatrixXd condition4 = AplusA.transpose();
        bool cond4 = condition4.isApprox(AplusA, tol);
        if (!cond4) {
            std::cerr << "条件四不满足：(A+ * A)^T != A+ * A\n";
        }

        // 返回所有条件是否都满足
        return cond1 && cond2 && cond3 && cond4;
    }

    // Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &A, double epsilon = 1e-6) {
    //     // 计算 A^T * A
    //     Eigen::MatrixXd AtA = A.transpose() * A;

    //     // 检查 A^T * A 是否可逆
    //     if (AtA.determinant() == 0) {
    //         std::cerr << "矩阵 A^T * A 是奇异的，无法通过直接法计算伪逆。\n";
    //         return Eigen::MatrixXd();
    //     }

    //     // 计算 (A^T * A)^{-1}
    //     Eigen::MatrixXd AtA_inv = AtA.inverse();

    //     // 计算 A^+ = (A^T * A)^{-1} * A^T
    //     Eigen::MatrixXd A_pinv = AtA_inv * A.transpose();

    //     return A_pinv;
    // }

    feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t> &phaseIDsStock)
    {
      const size_t numPhases = phaseIDsStock.size();

      feet_array_t<std::vector<bool>> contactFlagStock;
      std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

      for (size_t i = 0; i < numPhases; i++)
      {
        const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
        for (size_t j = 0; j < contactFlag.size(); j++)
        {
          contactFlagStock[j][i] = contactFlag[j];
        }
      }
      return contactFlagStock;
    }

    Eigen::VectorXd StateEstimateBase::lowPassFilter(const Eigen::VectorXd& currentFrame, Eigen::VectorXd& previousOutput, double alpha) {
    // 确保 previousOutput 的大小与 currentFrame 一致
    if (previousOutput.size() != currentFrame.size()) {
        previousOutput = Eigen::VectorXd::Zero(currentFrame.size());
    }

    // 应用低通滤波器公式: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    Eigen::VectorXd output = alpha * currentFrame + (1.0 - alpha) * previousOutput;

    // 更新 previousOutput 以供下一次调用使用
    previousOutput = output;

    return output;
}

    StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                         const PinocchioEndEffectorKinematics &eeKinematics)
        : pinocchioInterface_(std::move(pinocchioInterface)), info_(std::move(info)), eeKinematics_(eeKinematics.clone()), 
          rbdState_(vector_t ::Zero(2 * info_.generalizedCoordinatesNum)), latestStanceposition_{}
    {
      ros::NodeHandle nh;
      robotMass_ = info_.robotMass;
      ros::param::get("/mpc/mpcWaistDof", waistNum_);
      ros_logger_ = new TopicLogger(nh);
      
      // 初始化手臂接触力卡尔曼滤波器
      armForceKF_ = std::make_unique<ArmContactForceKalmanFilter>(12);
      odomPub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(nh, "odom", 10));

      posePub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>(nh, "pose", 10));

      pSCgZinvlast_.resize(info_.generalizedCoordinatesNum);
      pSCgZinvlast_.setZero();

      estContactforce_.resize(6 * 2 + 4); // 左右脚的6d接触力
      estContactforce_.fill(0);

      vMeasuredLast_.resize(info_.generalizedCoordinatesNum);
      vMeasuredLast_.fill(0);

      cmdTorque_.resize(info_.actuatedDofNum);
      cmdTorque_.setZero();
      jointPos_.resize(info_.actuatedDofNum);
      jointVel_.resize(info_.actuatedDofNum);
      jointVel_.setZero();
      earlyLateContactMsg_.data.resize(4, 0);
      foot_pos_desired_.fill(Eigen::Vector3d(0, 0, 0));
      
      // 初始化躯干稳定性检测相关变量
      torso_velocity_stable_start_time_ = ros::Time(0);
      torso_velocity_stable_tracking_ = false;
      is_torso_velocity_stable_ = false;

      // 初始化动力学项滑窗缓存
      pSCgArmSmoothWindow_.clear();
    }

    void StateEstimateBase::updateJointStates(const vector_t &jointPos, const vector_t &jointVel)
    {
      jointPos_ = jointPos;
      jointVel_ = jointVel;
      rbdState_.segment(6, info_.actuatedDofNum) = jointPos.head(info_.actuatedDofNum);
      rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel.head(info_.actuatedDofNum);
    }

    Eigen::Quaternion<scalar_t> StateEstimateBase::updateIntialEulerAngles(const Eigen::Quaternion<scalar_t> &quat_init)
    {
      vector3_t eulerAngles_init = quatToZyx(quat_init);
      std::cout << "eulerAngles_init: " << eulerAngles_init.transpose() << std::endl;
      angle_zyx_init_ = vector3_t(eulerAngles_init(0), 0, 0);
      Eigen::Quaternion<scalar_t> imu_yaw_offset_quat = getQuaternionFromEulerAnglesZyx(angle_zyx_init_);
      stance_angle_yaw_init_ = 0.0;
      yaw_offset_quat_ = getQuaternionFromEulerAnglesZyx(vector3_t(eulerAngles_init(0), 0, 0));

      vector3_t zyx = quatToZyx(quat_init) - angle_zyx_init_;
      Eigen::Quaternion<scalar_t> quat_adjusted = getQuaternionFromEulerAnglesZyx(zyx);
      prev_zyx_ = quatToZyx(quat_adjusted);// 
      return imu_yaw_offset_quat;
    }

    void StateEstimateBase::updateImu(const Eigen::Quaternion<scalar_t> &quat, const vector3_t &angularVelLocal,
                                      const vector3_t &linearAccelLocal, const matrix3_t &orientationCovariance,
                                      const matrix3_t &angularVelCovariance, const matrix3_t &linearAccelCovariance)
    {
      // std::cout << "before quatToZyx: " << quatToZyx(quat).transpose() << std::endl;
      // std::cout << "yaw_offset_quat_: " << quatToZyx(yaw_offset_quat_).transpose() << std::endl;
      // Eigen::Quaternion<scalar_t> quat_adjusted = quat * yaw_offset_quat_.inverse();
      // if (prev_gait_  != "stance" && gait_ == "stance")
      // {
      //   // 进入stance
      //   stance_angle_yaw_init_ = quatToZyx(quat_)[0];// TO-DO: 临时解决方案，后续需修改
      //   ROS_INFO_STREAM("stance_angle_yaw_init_: " << stance_angle_yaw_init_);
      // }
      // if (gait_ == "stance")
      // {
      //   auto diff_stance_angle_yaw = quatToZyx(quat)[0] - stance_angle_yaw_init_;
      //   angle_zyx_init_[0] = diff_stance_angle_yaw;
      //   ros_logger_->publishValue("/state_estimate/updateImu/angle_zyx_init_", angle_zyx_init_[0]);
      // }
      
      // vector3_t zyx = quatToZyx(quat) - angle_zyx_init_;// TO-DO: 临时解决方案，后续需修改
      vector3_t zyx = quatToZyx(quat);
      Eigen::Quaternion<scalar_t> quat_adjusted = getQuaternionFromEulerAnglesZyx(zyx);
      vector3_t zyx_adjusted = quatToZyx(quat_adjusted);
      // double angle_diff = (prev_zyx_ - zyx_adjusted).norm();
      // if (angle_diff > 1 * M_PI / 180.0 && angle_diff < M_PI) // TODO:IMU四元数存在小角度跳变bug
      // {
      //   double alpha = 0.01;
      //   zyx_adjusted = prev_zyx_ * (1 - alpha) + zyx_adjusted * alpha;
      //   std::cerr << "IMU angle jump detected, angle_diff: " << angle_diff << " corrected angle: " << zyx_adjusted.transpose() << std::endl;
      //   ros_logger_->publishValue("/state_estimate/updateImu/angle_diff", angle_diff);
      // }
      prev_zyx_ = zyx_adjusted;
      ros_logger_->publishVector("/state_estimate/updateImu/zyx_adjusted", zyx_adjusted);
      // std::cout << "after quatToZyx: " << quatToZyx(quat_adjusted).transpose()  << " offset: " << angle_zyx_init_.transpose() << std::endl;
      quat_ = quat_adjusted;
      angularVelLocal_ = angularVelLocal;
      linearAccelLocal_ = linearAccelLocal;
      orientationCovariance_ = orientationCovariance;
      angularVelCovariance_ = angularVelCovariance;
      linearAccelCovariance_ = linearAccelCovariance;

      // vector3_t zyx = quatToZyx(quat_adjusted) - zyxOffset_;
      vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
          zyx_adjusted, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(zyx_adjusted, angularVelLocal));
      angularVelWorld_ = angularVelGlobal;
      updateAngular(zyx_adjusted, angularVelGlobal);
    }

    void StateEstimateBase::updateAngular(const vector3_t &zyx, const vector_t &angularVel)
    {
      rbdState_.segment<3>(0) = zyx;
      rbdState_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const vector_t &pos, const vector_t &linearVel)
    {
      rbdState_.segment<3>(3) = pos;
      rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::Odometry &odom)
    {
      ros::Time time = odom.header.stamp;
      scalar_t publishRate = 200;
      if (lastPub_ + ros::Duration(1. / publishRate) < time)
      {
        lastPub_ = time;
        if (odomPub_->trylock())
        {
          odomPub_->msg_ = odom;
          odomPub_->unlockAndPublish();
        }
        if (posePub_->trylock())
        {
          posePub_->msg_.header = odom.header;
          posePub_->msg_.pose = odom.pose;
          posePub_->unlockAndPublish();
        }
      }
    }

    // ref: Contact Model Fusion for Event-Based Locomotion in Unstructured Terrains. Gerardo Bledt etc.
    void StateEstimateBase::estContactForce(const ros::Duration &period)
    {
      scalar_t dt = period.toSec();
      if (dt > 0.010 || dt < 1e-5)// 滤除掉十分异常的dt
        dt = 0.002;
      const scalar_t lamda = cutoffFrequency_;
      const scalar_t gama = exp(-lamda * dt);
      const scalar_t beta = (1 - gama) / (gama * dt);

      auto qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
      auto vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
      const auto &tauCmd = cmdTorque_;

      qMeasured_.head<3>() = rbdState_.segment<3>(3);
      qMeasured_.segment<3>(3) = rbdState_.head<3>();
      qMeasured_.tail(info_.actuatedDofNum) = rbdState_.segment(6, info_.actuatedDofNum);
      vMeasured_.head<3>() = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
      vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
          qMeasured_.segment<3>(3), rbdState_.segment<3>(info_.generalizedCoordinatesNum));
      vMeasured_.tail(info_.actuatedDofNum) = rbdState_.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

      const auto &model = pinocchioInterface_.getModel();
      auto &data = pinocchioInterface_.getData();

      matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
      s.block(0, 0, info_.actuatedDofNum, 6).setZero();
      s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

      pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);

      pinocchio::computeJointJacobians(model, data);
      pinocchio::updateFramePlacements(model, data);

      pinocchio::crba(model, data, qMeasured_);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

      pinocchio::getCoriolisMatrix(model, data);

      pinocchio::computeGeneralizedGravity(model, data, qMeasured_);

      vector_t p = data.M * vMeasured_;

      vector_t pSCg = beta * p + s.transpose() * tauCmd + data.C.transpose() * vMeasured_ - data.g;

      vector_t pSCg_z_inv = (1 - gama) * pSCg + gama * pSCgZinvlast_;
      pSCgZinvlast_ = pSCg_z_inv;

      // 由动力学方程： M(q) * v_dot + C(q, v) * v + g(q) = tau + J^T * F_c, 需要求外界的接触力 F_c
      // estDisturbancetorque_ = tau - M(q) * v_dot - C(q, v) * v - g(q)
      // 其中 estDisturbancetorque_ = J^T * F_c，再求F_c即可

      estDisturbancetorque_ = beta * p - pSCg_z_inv;

      auto Jac_i = matrix_t(6, info_.generalizedCoordinatesNum);
      auto S_li = matrix_t(6, info_.generalizedCoordinatesNum);
      std::vector<std::string> eeNames = {"leg_l6_link", "leg_r6_link"};
      for (size_t i = 0; i < 2; ++i)
      {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, model.getBodyId(eeNames[i]), pinocchio::LOCAL_WORLD_ALIGNED, jac);
        Jac_i = jac.template topRows<6>();
        S_li.setZero();
        int index = 0;
        if (i == 0)
          // index = 0 + waistNum_;
          index = 0;
        else if (i == 1)
          // index = 6 + waistNum_;
          index = 6;
        S_li.block<6, 6>(0, 6 + index) = Eigen::Matrix<scalar_t, 6, 6>::Identity();
        matrix66_t S_JT = S_li * Jac_i.transpose();
        vector6_t S_tau = S_li * estDisturbancetorque_;

        // 求解 S_JT * estContactforce = S_tau。
        estContactforce_.segment<6>(6 * i) = S_JT.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(S_tau);
      }
    }

    // 核心优化：7自由度手臂末端力估计（Pinocchio原生接口+自适应阻尼+三层平滑）
    void StateEstimateBase::estArmContactForce(const ros::Duration &period)
    {
      if (estArmContactforce_.size() == 0)
      {
        estArmContactforce_.resize(6 * 2);
        estArmContactforce_.fill(0);
        estArmContactforceLast_.resize(6 * 2);
        estArmContactforceLast_.fill(0);
      }

      scalar_t dt = period.toSec();
      if (dt > 0.010 || dt < 1e-5)
        dt = 0.002;
      
      // 动量微分法系数
      const scalar_t lamda = cutoffFrequency_;
      const scalar_t gama = exp(-lamda * dt);
      const scalar_t beta = (1 - gama) / (gama * dt);

      PinocchioInterface& pinInterface = use_full_arm_model_ ? *armForcePinocchioInterfacePtr_ : pinocchioInterface_;
      const CentroidalModelInfo& info = use_full_arm_model_ ? *armForceInfoPtr_ : info_;
      const vector_t& rbdState = use_full_arm_model_ ? armForceRbdState_ : rbdState_;
      const vector_t& tauCmd = use_full_arm_model_ ? armForceCmdTorque_ : cmdTorque_;

      const auto &model = pinInterface.getModel();
      auto &data = pinInterface.getData();

      // 维度检查（不匹配则跳过，避免崩溃/无效结果）
      if (static_cast<int>(model.nq) != info.generalizedCoordinatesNum ||
          static_cast<int>(model.nv) != info.generalizedCoordinatesNum ||
          rbdState.size() != 2 * info.generalizedCoordinatesNum ||
          tauCmd.size() != info.actuatedDofNum)
      {
        ROS_WARN_STREAM_THROTTLE(1.0,
                                 "[StateEstimateBase::estArmContactForce] 维度不匹配，跳过本次手臂接触力估计："
                                 << " model(nq,nv)=(" << model.nq << "," << model.nv << ")"
                                 << " info.generalizedCoordinatesNum=" << info.generalizedCoordinatesNum
                                 << " info.actuatedDofNum=" << info.actuatedDofNum
                                 << " rbdState.size=" << rbdState.size()
                                 << " tauCmd.size=" << tauCmd.size()
                                 << " use_full_arm_model_=" << use_full_arm_model_);
        return;
      }

      auto qMeasured = vector_t(info.generalizedCoordinatesNum);
      auto vMeasured = vector_t(info.generalizedCoordinatesNum);

      qMeasured.head<3>() = rbdState.segment<3>(3);
      qMeasured.segment<3>(3) = rbdState.head<3>();
      qMeasured.tail(info.actuatedDofNum) = rbdState.segment(6, info.actuatedDofNum);
      vMeasured.head<3>() = rbdState.segment<3>(info.generalizedCoordinatesNum + 3);
      vMeasured.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
          qMeasured.segment<3>(3), rbdState.segment<3>(info.generalizedCoordinatesNum));
      vMeasured.tail(info.actuatedDofNum) = rbdState.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

      // 计算手臂起始索引
      const size_t legDof = 12;
      const size_t totalArmDof = info.actuatedDofNum - waistNum_ - legDof;
      armDofPerSide_ = totalArmDof / 2;
      const size_t leftArmStartIdx = 6 + waistNum_ + legDof;
      const size_t rightArmStartIdx = leftArmStartIdx + armDofPerSide_;

      // 动力学计算（原有逻辑保留：正运动学+雅克比+CRBA+科氏力+重力）
      matrix_t s(info.actuatedDofNum, info.generalizedCoordinatesNum);
      s.block(0, 0, info.actuatedDofNum, 6).setZero();
      s.block(0, 6, info.actuatedDofNum, info.actuatedDofNum).setIdentity();
      
      pinocchio::forwardKinematics(model, data, qMeasured, vMeasured);
      pinocchio::computeJointJacobians(model, data);
      pinocchio::updateFramePlacements(model, data);

      pinocchio::crba(model, data, qMeasured);
      data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

      pinocchio::getCoriolisMatrix(model, data);
      pinocchio::computeGeneralizedGravity(model, data, qMeasured);

      // 动量微分法计算惯性项
      vector_t p = data.M * vMeasured;
      vector_t pSCg = beta * p + s.transpose() * tauCmd + data.C.transpose() * vMeasured - data.g;

      // 优化1：动力学项pSCg 3窗长滑窗平滑（抑制微小扰动被伪逆放大，无明显延迟）
      pSCgArmSmoothWindow_.push_back(pSCg);
      if (pSCgArmSmoothWindow_.size() > pSCgSmoothWindowSize_)
      {
        pSCgArmSmoothWindow_.pop_front();
      }
      Eigen::VectorXd pSCg_smooth = Eigen::VectorXd::Zero(pSCg.size());
      for (const auto& pSCg_val : pSCgArmSmoothWindow_)
      {
        pSCg_smooth += pSCg_val;
      }
      pSCg_smooth /= pSCgArmSmoothWindow_.size();

      // 低通滤波
      if (pSCgArmZinvlast_.size() != info.generalizedCoordinatesNum) {
        pSCgArmZinvlast_.resize(info.generalizedCoordinatesNum);
        pSCgArmZinvlast_.setZero();
      }
      vector_t pSCg_z_inv = (1 - gama) * pSCg_smooth + gama * pSCgArmZinvlast_;
      pSCgArmZinvlast_ = pSCg_z_inv;

      // 计算扰动力矩（包含惯性项）
      vector_t estDisturbancetorque = beta * p - pSCg_z_inv;

      // 调试输出：发布关键中间数据
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/qMeasured", qMeasured);
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/vMeasured", vMeasured);
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/tauCmd", tauCmd);
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/estDisturbancetorque", estDisturbancetorque);

      // 手臂末端力估计主循环（左右臂）
      std::vector<std::string> eeNames = {"zarm_l7_end_effector", "zarm_r7_end_effector"};
      std::vector<size_t> armStartIdxs = {leftArmStartIdx, rightArmStartIdx};
      std::vector<double> jacobian_conditions(2, 1.0);
      
      for (size_t i = 0; i < 2; ++i)
      {
        // 末端Frame合法性校验
        auto armFrameId = model.getFrameId(eeNames[i]);
        if (armFrameId < 0) continue;
        size_t armStartIdx = armStartIdxs[i];
        if (armStartIdx + armDofPerSide_ > info.generalizedCoordinatesNum) continue;

        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, armFrameId, pinocchio::LOCAL_WORLD_ALIGNED, jac);

        // 构造选择矩阵（只选择对应手臂的关节）
        matrix_t S_li_arm = matrix_t::Zero(armDofPerSide_, info.generalizedCoordinatesNum);
        S_li_arm.block(0, armStartIdx, armDofPerSide_, armDofPerSide_) = matrix_t::Identity(armDofPerSide_, armDofPerSide_);
        
        // S_JT = S_li * J^T
        Eigen::Matrix<scalar_t, Eigen::Dynamic, 6> S_JT = S_li_arm * jac.transpose();
        vector_t S_tau = S_li_arm * estDisturbancetorque;
        
        // 调试输出：发布每个手臂的计算数据
        std::string arm_side = (i == 0) ? "left" : "right";
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/armStartIdx", static_cast<double>(armStartIdx));
        ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/" + arm_side + "/S_tau", S_tau);

        // SVD奇异值/条件数：Eigen实现（避免依赖 pinocchio::jacobianSVD）
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_sjt(S_JT, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto s = svd_sjt.singularValues();
        const double maxS = (s.size() > 0) ? s(0) : 0.0;
        const double minS = (s.size() > 0) ? s(s.size() - 1) : 0.0;
        const double condNum = (minS > 1e-12) ? (maxS / minS) : 1e12;
        jacobian_conditions[i] = condNum;
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/jacobian_condition", condNum);
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/max_singular_value", maxS);
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/min_singular_value", minS);

        // 优化4：自适应阻尼计算+ Pinocchio原生带阻尼伪逆（替代手写，数值更稳定）
        const bool cond_trigger = (condNum > armJacCondThresh_);
        const bool minS_trigger = (minS < armJacMinSingularThresh_);
        const bool in_singular_damping = (cond_trigger || minS_trigger);

        // 调试输出：当前是否进入奇异阻尼分支，以及触发来源（cond / minS）
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/in_singular_damping",
                                  in_singular_damping ? 1.0 : 0.0);
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/singular_trigger_cond",
                                  cond_trigger ? 1.0 : 0.0);
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/singular_trigger_minS",
                                  minS_trigger ? 1.0 : 0.0);

        double lambda = 0.0; // 初始化：非奇异时为 0（表示未使用 DLS），仅用于调试输出
        if (in_singular_damping)
        {
          // 非线性阻尼增长，平滑无突变，避免过度增大
          double singularRatio = std::max(1e-3, minS / armJacMinSingularThresh_);
          lambda = armDampingLambda0_ * exp(armDampingGain_ * (1.0 / singularRatio - 1.0));
          lambda = std::min(lambda, 0.08); // 阻尼上限，保证估计精度
        }
        // 求解接触力：
        // 近奇异用DLS抑制数值放大；非奇异用普通最小二乘解（更贴近真实值）
        vector6_t F_estimated = in_singular_damping ? dlsSolve6(S_JT, S_tau, lambda) : svd_sjt.solve(S_tau);

        // 调试输出：发布求解后的F_estimated
        ros_logger_->publishValue("/state_estimate/arm_contact_force_debug/" + arm_side + "/damping_lambda", lambda);
        ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/" + arm_side + "/F_estimated_raw", F_estimated);

        // 优化5：末端力硬限幅（兜底防护1，根据物理特性，避免超量程跳变）
        // 兼容旧版Eigen：没有 cwiseClamp()，用 cwiseMax/cwiseMin 等价实现
        F_estimated.segment<3>(0) =
            F_estimated.segment<3>(0).cwiseMax(-armForceMax_).cwiseMin(armForceMax_);     // 力分量
        F_estimated.segment<3>(3) =
            F_estimated.segment<3>(3).cwiseMax(-armMomentMax_).cwiseMin(armMomentMax_);   // 力矩分量

        // 符号修正（原有逻辑保留，根据实物/仿真配置）
        // F_estimated = -F_estimated;  // 实物需要取反      
        estArmContactforce_.segment<6>(6 * i) = F_estimated;
      }

      // 优化6：末端力3窗长中值滤波（兜底防护2，剔除脉冲式尖峰，无信号失真）
      static std::deque<Eigen::VectorXd> armForceMedianWindow_;
      armForceMedianWindow_.push_back(estArmContactforce_);
      if (armForceMedianWindow_.size() > 3) armForceMedianWindow_.pop_front();
      if (armForceMedianWindow_.size() == 3)
      {
        Eigen::VectorXd F_median = Eigen::VectorXd::Zero(12);
        for (int j = 0; j < 12; ++j)
        {
          std::vector<double> vals = {armForceMedianWindow_[0](j), armForceMedianWindow_[1](j), armForceMedianWindow_[2](j)};
          std::sort(vals.begin(), vals.end());
          F_median(j) = vals[1]; // 3窗长取中值，最优尖峰抑制
        }
        estArmContactforce_ = F_median;
      }

      // 原有滤波逻辑保留：卡尔曼滤波/低通滤波（与中值滤波叠加，平滑效果更强）
      if (useArmForceKalmanFilter_) {
        // 使用卡尔曼滤波（带自适应R）
        armForceKF_->predict(dt);
        
        // 计算接触置信度（基于力的大小）
        double left_force_z = std::abs(estArmContactforce_(2));
        double right_force_z = std::abs(estArmContactforce_(8));
        double max_expected_force = 100.0;  // 最大预期力（N）
        double left_confidence = std::min(1.0, left_force_z / max_expected_force);
        double right_confidence = std::min(1.0, right_force_z / max_expected_force);
        
        // 取两侧的平均条件数和置信度
        double avg_condition = (jacobian_conditions[0] + jacobian_conditions[1]) / 2.0;
        double avg_confidence = (left_confidence + right_confidence) / 2.0;
        
        // 计算自适应R
        auto R_adaptive = armForceKF_->computeAdaptiveR(avg_condition, avg_confidence);
        
        // 更新（使用自适应R）
        armForceKF_->update(estArmContactforce_, R_adaptive);
        estArmContactforce_ = armForceKF_->getEstimate();
        
        // // 每2秒打印一次卡尔曼滤波状态
        // static int kf_debug_count = 0;
        // if (kf_debug_count++ % 2000 == 0) {
        //   ROS_INFO("[ArmForceKF] Filtered: L_Fz=%.2f, R_Fz=%.2f N | Cond=%.1f, Conf=%.2f, Uncertainty=%.2f", 
        //            estArmContactforce_(2), estArmContactforce_(8), 
        //            avg_condition, avg_confidence, armForceKF_->getEstimateUncertainty());
        // }
      } else {
        // 使用低通滤波
        const scalar_t cutoff = 5.0;
        const scalar_t alpha = dt / (dt + 1.0 / (2.0 * M_PI * cutoff));
        estArmContactforce_ = alpha * estArmContactforce_ + (1.0 - alpha) * estArmContactforceLast_;
      }
      estArmContactforceLast_ = estArmContactforce_;
      
      // 调试输出：发布最终的接触力估计结果
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/estArmContactforce_final", estArmContactforce_);
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/left_arm_force", estArmContactforce_.segment<6>(0));
      ros_logger_->publishVector("/state_estimate/arm_contact_force_debug/right_arm_force", estArmContactforce_.segment<6>(6));
    }

    bool StateEstimateBase::checkArmLoad(double force_threshold)
    {
      if (estArmContactforce_.size() < 12)
        return false;

      double leftArmForceZ = -estArmContactforce_[2];
      double rightArmForceZ = -estArmContactforce_[8];

      double totalArmForce = std::max(leftArmForceZ, 0.0) + std::max(rightArmForceZ, 0.0);

      armForceWindow_.push_back(totalArmForce);
      if (armForceWindow_.size() > armForceWindowSize_)
      {
        armForceWindow_.pop_front();
      }

      if (armForceWindow_.size() < armForceWindowSize_)
        return false;

      double avgForce = std::accumulate(armForceWindow_.begin(), armForceWindow_.end(), 0.0) / armForceWindow_.size();

      return (avgForce > force_threshold);
    }

    contact_flag_t StateEstimateBase::estContactState(const scalar_t &time)
    {
      contact_flag_t contact_state = cmdContactflag_;
      for (int i = 0; i < info_.numThreeDofContacts; i++)
      {
        const auto start_time = StartStopTime4Legs_[i].front();
        const auto stop_time = StartStopTime4Legs_[i].back();
        const auto period = stop_time - start_time;
        if (!cmdContactflag_[i] && (time - start_time > 0.75 * period)) // swing legs
        {
          contact_state[i] = (estContactforce_(6 * (i % 2) + 2) > contactThreshold_) ? true : false;
        }
        if (cmdContactflag_[i] && (time - start_time < 0.25 * period)) // stance legs
        {
          contact_state[i] = (estContactforce_(6 * (i % 2) + 2) > contactThreshold_) ? true : false;
        }
      }
      return std::move(contact_state);
    }

    void StateEstimateBase::earlyContactDetection(const ModeSchedule &modeSchedule, scalar_t current_time)
    {
      auto &modeSequence = modeSchedule.modeSequence;
      auto &eventTimes = modeSchedule.eventTimes;
      auto con_seq = extractContactFlags(modeSequence);

      // first, find where to insert by time sequence
      auto time_insert_it = std::lower_bound(eventTimes.begin(), eventTimes.end(), current_time);
      // "id" is the new element's index
      const size_t id = std::distance(eventTimes.begin(), time_insert_it);

      // reset early contact flag
      for (int leg = 0; leg < 4; leg++)
      {
        earlyLatecontact_[0][leg] = false;
      }

      bool insert_flag = false;
      size_array_t insert_legs;
      // check if there is early contact, and find each leg
      for (int leg = 0; leg < 4; leg++)
      {
        if (!con_seq[leg][id] && contactFlag_[leg])
        {
          insert_flag = true;
          insert_legs.push_back(leg);
        }
      }

      // mask the early contact feet's contact sequence
      if (insert_flag)
      {
        if (abs(eventTimes.at(id) - current_time) > 0.001 && abs(eventTimes.at(id - 1) - current_time) > 0.001)
        {
          size_array_t filtered_insert_legs;
          // check if leg just start swing, if do, remove that leg
          for (auto leg : insert_legs)
          {
            // find swing start time
            scalar_t start_time = current_time;
            for (size_t i = id; i > 0; i--)
            {
              if (!con_seq[leg][i])
                start_time = eventTimes[i - 1];
              else
                break;
            }
            // find swing stop time
            scalar_t stop_time = current_time;
            for (size_t i = id; i < eventTimes.size(); i++)
            {
              if (!con_seq[leg][i])
                stop_time = eventTimes[i];
              else
                break;
            }
            const scalar_t length = stop_time - start_time;
            // if just start swing, or very close to finish, pass that leg
            if (current_time - start_time > 0.75 * length && stop_time - current_time > 0.009)
            {
              earlyLatecontact_[0][leg] = true;
            }
          }
        }
      }
    }

    void StateEstimateBase::lateContactDetection(const ModeSchedule &modeSchedule, scalar_t current_time)
    {
      auto &modeSequence = modeSchedule.modeSequence;
      auto &eventTimes = modeSchedule.eventTimes;
      auto con_seq = extractContactFlags(modeSequence);

      estConHistory_.push_front(std::pair<scalar_t, contact_flag_t>{current_time, contactFlag_});
      if (estConHistory_.size() > 10)
        estConHistory_.pop_back();

      // # late contact handle
      // ## find close cmd stance foot
      // ### get swing start time
      auto time_insert_it = std::lower_bound(eventTimes.begin(), eventTimes.end(), current_time);
      // "id" is the new element's index
      const size_t id = std::distance(eventTimes.begin(), time_insert_it);

      // reset late contact flag
      for (int leg = 0; leg < 4; leg++)
      {
        earlyLatecontact_[1][leg] = false;
      }

      size_array_t stance_legs;
      size_array_t swing_legs;
      for (int leg = 0; leg < 4; leg++)
      {
        if (con_seq[leg][id]) // cmd stance leg
          stance_legs.push_back(leg);
        else // cmd swing leg
          swing_legs.push_back(leg);
      }
      // check stance leg first, and only delay once
      bool delayed_flag = false;
      // leg whether need to swing down
      feet_array_t<bool> leg_swing_down_flags{false, false};
      for (auto leg : stance_legs) // for cmd stance leg
      {
        // find stance start time
        scalar_t start_time = current_time;
        for (size_t i = id; i > 0; i--)
        {
          if (con_seq[leg][i])
            start_time = eventTimes[i - 1];
          else
            break;
        }
        // if just finish swing cmd and no contact detected
        if (current_time - start_time < 0.04 && !contactFlag_[leg])
        {
          bool back_check = true;
          for (auto histroy_con : estConHistory_)
          {
            if (histroy_con.first >= start_time)
            {
              if (histroy_con.second[leg])
              {
                back_check = false;
              }
            }
          }
          if (back_check)
          {
            earlyLatecontact_[1][leg] = true;
          }
        }
      }
    }

    void StateEstimateBase::loadSettings(const std::string &taskFile, bool verbose, const std::string &referenceFile)
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "contactForceEsimation.";
      if (verbose)
      {
        std::cerr << "\n #### contactForceEsimation:";
        std::cerr << "\n #### =============================================================================\n";
      }

      loadData::loadPtreeValue(pt, cutoffFrequency_, prefix + "cutoffFrequency", verbose);
      loadData::loadPtreeValue(pt, detectCutoffFrequency_, prefix + "detectCutoffFrequency", verbose);
      loadData::loadPtreeValue(pt, contactThreshold_, prefix + "contactThreshold", verbose);
      loadData::loadPtreeValue(pt, min_energy_threshold_, prefix + "minEnergyThreshold", verbose);
      loadData::loadPtreeValue(pt, max_energy_threshold_, prefix + "maxEnergyThreshold", verbose);
      loadData::loadPtreeValue(pt, min_energy_threshold2_, prefix + "minEnergyThreshold2", verbose);
      loadData::loadPtreeValue(pt, max_energy_threshold2_, prefix + "maxEnergyThreshold2", verbose);
      loadData::loadPtreeValue(pt, holdTime_, prefix + "timeTreshold", verbose);

      // 从 referenceFile 读取躯干速度稳定性检测参数
      if (!referenceFile.empty())
      {
        boost::property_tree::ptree ref_pt;
        try
        {
          boost::property_tree::read_info(referenceFile, ref_pt);
          loadData::loadPtreeValue(ref_pt, torsoVelocityThreshold_, "torsoVelocityThreshold", verbose);
          loadData::loadPtreeValue(ref_pt, torsoVelocityDuration_, "torsoVelocityDuration", verbose);
        }
        catch (const std::exception &e)
        {
          ROS_WARN("[StateEstimateBase] Failed to load torso velocity stability parameters from referenceFile: %s. Using default values.", e.what());
        }
      }
    }

    void StateEstimateBase::updateTorsoStability(const ros::Time &time, const ros::Duration &period)
    {
      // 确保时间有效
      if (!time.isValid() || time.toSec() <= 0)
      {
        return;
      }
      
      // 获取躯干状态
      vector_t torso_state = getTorsoState();
      
      // 提取线速度和角速度
      vector3_t linear_vel = torso_state.segment<3>(6);   // vx, vy, vz
      vector3_t angular_vel = torso_state.segment<3>(9);   // angularVx, angularVy, angularVz
      // linear_vel = vector3_t(0.0, 0.0, 0.0);
      // angular_vel = vector3_t(0.0, 0.0, 0.0);
      // 计算速度模长
      double linear_vel_magnitude = linear_vel.norm();
      double angular_vel_magnitude = angular_vel.norm();
      
      // 检查速度是否在阈值内
      bool velocity_stable = (linear_vel_magnitude < torsoVelocityThreshold_) && 
                             (angular_vel_magnitude < torsoVelocityThreshold_);
      if (velocity_stable)
      {
        // 如果速度稳定，检查是否已经开始跟踪
        if (!torso_velocity_stable_tracking_)
        {
          // 第一次检测到稳定，开始跟踪
          torso_velocity_stable_start_time_ = time;
          torso_velocity_stable_tracking_ = true;
          is_torso_velocity_stable_ = false;
        }
        else
        {
          // 已经在跟踪，检查持续时间
          double stable_duration = (time - torso_velocity_stable_start_time_).toSec();
          if (stable_duration >= torsoVelocityDuration_)
          {
            // 已经稳定足够长时间
            is_torso_velocity_stable_ = true;
          }
          else
          {
            // 还在等待稳定时间
            is_torso_velocity_stable_ = false;
          }
        }
      }
      else
      {
        // 速度不稳定，重置跟踪状态
        if (torso_velocity_stable_tracking_)
        {
          torso_velocity_stable_tracking_ = false;
        }
        is_torso_velocity_stable_ = false;
      }
      
    }
    void StateEstimateBase::updateContactProbabilities(double fzFilterLeft_, double fzFilterRight_, double robotMass, double dt) 
    {
      // 观测模型
      double left_cal = 0.0;
      double right_cal = 0.0;
      // 计算接触概率
      auto calculate_contact_probability = [this](double Fz_filter, double robotMass) {
        return 1.0 / (1.0 + std::exp(-(Fz_filter - (robotMass*9.81*0.5)) * 0.01));
      };
      left_cal = calculate_contact_probability(fzFilterLeft_,robotMass);
      right_cal = calculate_contact_probability(fzFilterRight_,robotMass);
      // 更新接触概率
      contactProbabilityLeft_ = alpha_ * left_cal + (1.0 - alpha_) * preContactProbabilityLeft_;
      contactProbabilityRight_ = alpha_ * right_cal + (1.0 - alpha_) * preContactProbabilityRight_;
      // contactProbabilityLeft_ = left_cal;
      // contactProbabilityRight_ = right_cal;
      preContactProbabilityLeft_ = contactProbabilityLeft_;
      preContactProbabilityRight_ = contactProbabilityRight_;
    }
    size_t StateEstimateBase::ContactDetection(const size_t nextMode_, const bool stanceMode_, const size_t plannedMode_, double robotMass, const double fzLeft, const double fzRight, double dt)
    {
      size_t contactState_ = preContactState_;
      dt = std::max(dt, 0.002);  // 确保 dt 大于等于 0.002
      contactHoldTime_ += dt;
      contactHoldTime_ = std::min(contactHoldTime_, holdTime_);
      double cutoffFreq = detectCutoffFrequency_;
      alpha_ = cutoffFreq * dt / (1 + cutoffFreq * dt);
      // 加权过滤
      double fzFilterLeft_ = alpha_ * fzLeft + (1.0 - alpha_) * preFzFilterLeft_;
      double fzFilterRight_ = alpha_ * fzRight + (1.0 - alpha_) * preFzFilterRight_;
      updateContactProbabilities(fzFilterLeft_, fzFilterRight_, robotMass, dt);
      ros_logger_->publishValue("/state_estimate/Contact_Detection/fzFilterLeft", fzFilterLeft_);
      ros_logger_->publishValue("/state_estimate/Contact_Detection/fzFilterRight", fzFilterRight_);
      if(contactProbabilityLeft_ + contactProbabilityRight_ > max_energy_threshold2_ || 
         contactProbabilityLeft_ + contactProbabilityRight_ < min_energy_threshold2_ || 
         cantactRight_ == 0 && cantactLeft_ == 0){
        unknewContact_ = true;
      }
      // ros_logger_->publishValue("/state_estimate/Contact_Detection/unknewContact_", (unknewContact_));
      if((contactState_ == ModeNumber::FS || contactState_ == ModeNumber::SF) && !unknewContact_ && contactHoldTime_ == holdTime_)
      {
        double avgValue = totalValue / sumCount_;
        // ros_logger_->publishValue("/state_estimate/Contact_Detection/avgValue", (avgValue));
        if(contactState_ == ModeNumber::FS){
          cantactRight_ = 1;
          cantactLeft_ = 0;
          if(contactProbabilityLeft_ >= max_energy_threshold_){
            leftCount_++;
            if(leftCount_ > 3){
              cantactLeft_ = 1;
            }
          }else{
            cantactLeft_ = 0;
          }
        }
        if(contactState_ == ModeNumber::SF){
          cantactLeft_ = 1;
          cantactRight_ = 0;
          if(contactProbabilityRight_ >= max_energy_threshold_){
            rightCount_++;
            if(rightCount_ > 3){
              cantactRight_ = 1;
            }
          }else{
            cantactRight_ = 0;
          }
        }
        // ros_logger_->publishValue("/state_estimate/Contact_Detection/cantactLeft", (cantactLeft_));
        // ros_logger_->publishValue("/state_estimate/Contact_Detection/cantactRight", (cantactRight_));
        // ros_logger_->publishValue("/state_estimate/Contact_Detection/leftCount_", (leftCount_));
        // ros_logger_->publishValue("/state_estimate/Contact_Detection/rightCount_", (rightCount_));
        if(cantactLeft_ == 1 && cantactRight_ == 1){
          contactState_ = ModeNumber::SS;
        } else if(cantactLeft_ == 1 && cantactRight_ == 0){
          contactState_ = ModeNumber::SF;
        } else if(cantactRight_ == 1 && cantactLeft_ == 0){
          contactState_ = ModeNumber::FS;
        }
      } else if(contactState_ == ModeNumber::FS && contactHoldTime_ != holdTime_){
        totalValue += contactProbabilityLeft_;
        sumCount_++;
      } else if(contactState_ == ModeNumber::SF && contactHoldTime_ != holdTime_){
        totalValue += contactProbabilityRight_;
        sumCount_++;
      }
      if(contactState_ != preContactState_){
        estModeCount_ ++;
      }
      if(plannedMode_ != prePlannedMode_){
        plannedModeCount_++;
      }
      bool delayContact_ = false;
      if(plannedModeCount_ > estModeCount_){
        delayContact_ = true;
      } else if(estModeCount_ > plannedModeCount_){
        contactState_ = nextMode_;
        estModeCount_ = 0;
        plannedModeCount_ = 0;
        unknewContact_ = false;
        totalValue = 0.0;
        sumCount_ = 0;
        leftCount_ = 0;
        rightCount_ = 0;
        leftCountHoldTime_ = 0.0;
        rightCountHoldTime_ = 0.0;
      }
      usePlannedMode_ = stanceMode_ || delayContact_;
      // ros_logger_->publishValue("/state_estimate/Contact_Detection/usePlannedMode", (usePlannedMode_));
      if(usePlannedMode_){
        contactState_ = plannedMode_;
        estModeCount_ = 0;
        plannedModeCount_ = 0;
        unknewContact_ = false;
        contactHoldTime_ = 0.0;
        totalValue = 0.0;
        sumCount_ = 0;
        leftCount_ = 0;
        rightCount_ = 0;
        leftCountHoldTime_ = 0.0;
        rightCountHoldTime_ = 0.0;
      }
      if(leftCount_ > 0 && leftCount_ < 3)
      {
        leftCountHoldTime_ += dt;
        if(leftCountHoldTime_ > 0.01){
          leftCount_ = 0;
          leftCountHoldTime_ = 0.0;
        }
      }
      if(rightCount_ > 0 && rightCount_ < 3)
      {
        rightCountHoldTime_ += dt;
        if(rightCountHoldTime_ > 0.01){
          rightCount_ = 0;
          rightCountHoldTime_ = 0.0;
        }
      }
      preContactState_ = contactState_;
      prePlannedMode_ = plannedMode_;
      preFzFilterLeft_ = fzFilterLeft_;
      preFzFilterRight_ = fzFilterRight_;
      ros_logger_->publishValue("/state_estimate/Contact_Detection/contactProbabilityLeft", contactProbabilityLeft_);
      ros_logger_->publishValue("/state_estimate/Contact_Detection/contactProbabilityRight", contactProbabilityRight_);
      // ros_logger_->publishVector("/state_estimate/Contact_Detection/stanceMode", {stanceMode_});
      // ros_logger_->publishVector("/state_estimate/Contact_Detection/contactHoldTime", {contactHoldTime_});
      return contactState_;
    }

  } // namespace humanoid
} // namespace ocs2
