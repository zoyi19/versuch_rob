//
// Created for arm contact force estimation with Kalman filtering
// Date: 2025-10-27
//

#include "humanoid_estimation/ArmContactForceKalmanFilter.h"
#include <cmath>

namespace ocs2 {
namespace humanoid {

ArmContactForceKalmanFilter::ArmContactForceKalmanFilter(int state_dim)
    : state_dim_(state_dim), innovation_norm_(0.0) {
  // 初始化向量和矩阵
  x_est_ = Eigen::VectorXd::Zero(state_dim_);
  x_pred_ = Eigen::VectorXd::Zero(state_dim_);
  P_est_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 100.0;  // 初始协方差较大
  P_pred_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  I_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  
  // 默认过程噪声Q：保持中等（平衡响应速度和稳定性）
  Q_base_ = Eigen::VectorXd(state_dim_);
  for (int i = 0; i < state_dim_; ++i) {
    if (i % 6 < 3) {  // 力分量 (Fx, Fy, Fz)
      Q_base_(i) = 1.0;  // 1.0N标准差（中等，避免过快响应导致振荡）
    } else {  // 力矩分量 (Tx, Ty, Tz)
      Q_base_(i) = 0.1;  // 0.1N·m标准差
    }
  }
  Q_ = Q_base_.cwiseProduct(Q_base_).asDiagonal();
  
  // 默认观测噪声R：基于动量微分法的经验值（中等信任度）
  R_base_ = Eigen::VectorXd(state_dim_);
  for (int i = 0; i < state_dim_; ++i) {
    if (i % 6 < 3) {  // 力分量
      R_base_(i) = 4.0;  // 4.0N标准差（中等信任，正常时R/Q≈4，卡尔曼增益适中）
    } else {  // 力矩分量
      R_base_(i) = 0.4;  // 0.4N·m标准差
    }
  }
  R_ = R_base_.cwiseProduct(R_base_).asDiagonal();
}

void ArmContactForceKalmanFilter::predict(double dt) {
  // 随机游走模型：x(k+1) = x(k)
  x_pred_ = x_est_;
  
  // 协方差预测：P(k|k-1) = P(k-1|k-1) + Q
  P_pred_ = P_est_ + Q_;
}

void ArmContactForceKalmanFilter::update(const Eigen::VectorXd& measurement) {
  update(measurement, R_);
}

void ArmContactForceKalmanFilter::update(const Eigen::VectorXd& measurement, 
                                          const Eigen::MatrixXd& R_custom) {
  // 新息（innovation）
  Eigen::VectorXd innovation = measurement - x_pred_;
  innovation_norm_ = innovation.norm();  // 记录新息范数
  
  // 新息协方差：S = P(k|k-1) + R
  Eigen::MatrixXd S = P_pred_ + R_custom;
  
  // 卡尔曼增益：K = P(k|k-1) * S^{-1}
  Eigen::MatrixXd K = P_pred_ * S.inverse();
  
  // 状态更新：x(k|k) = x(k|k-1) + K * innovation
  x_est_ = x_pred_ + K * innovation;
  
  // 协方差更新：P(k|k) = (I - K) * P(k|k-1)
  P_est_ = (I_ - K) * P_pred_;
  
  // 确保协方差矩阵的对称性
  P_est_ = 0.5 * (P_est_ + P_est_.transpose());
}

void ArmContactForceKalmanFilter::reset() {
  x_est_.setZero();
  x_pred_.setZero();
  P_est_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 100.0;
  P_pred_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  innovation_norm_ = 0.0;
}

Eigen::MatrixXd ArmContactForceKalmanFilter::computeAdaptiveR(
    double jacobian_condition, double contact_confidence) const {
  
  // 自适应因子计算（基于奇异性的分段策略）
  // 核心思想：奇异时保守，正常时积极但不过激
  
  // 1. 雅可比条件数因子：重点区分正常(<100)和奇异(>100)两种情况
  //    正常范围(1-100):   factor=1.0-1.5  （轻微调整，保持响应）
  //    中等奇异(100-1000): factor=1.5-2.5  （适度增大R，开始保守）
  //    严重奇异(>1000):   factor=2.5-4.0  （显著增大R，高度保守）
  double condition_factor = 1.0;
  if (jacobian_condition < 100.0) {
    // 正常区域：几乎不调整，保持快速响应
    // cond=1 -> 1.0, cond=100 -> 1.5
    condition_factor = 1.0 + (jacobian_condition - 1.0) / 99.0 * 0.5;
  } else if (jacobian_condition < 1000.0) {
    // 中等奇异：线性增长
    // cond=100 -> 1.5, cond=1000 -> 2.5
    condition_factor = 1.5 + (jacobian_condition - 100.0) / 900.0 * 1.0;
  } else {
    // 严重奇异：对数增长，最大4.0
    // cond=1000 -> 2.5, cond=1e6 -> 4.0
    condition_factor = std::min(4.0, 2.5 + std::log10(jacobian_condition / 1000.0) / 3.0 * 1.5);
  }
  
  // 2. 接触置信度因子：仅在低置信度(<0.2)时显著增大R
  //    高置信度(>0.5):   factor=1.0      （完全信任）
  //    中等置信度(0.2-0.5): factor=1.0-1.5 （轻微保守）
  //    低置信度(<0.2):   factor=1.5-2.5  （明显保守）
  double confidence_factor = 1.0;
  if (contact_confidence < 0.2) {
    // 低置信度：线性映射
    // confidence=0 -> 2.5, confidence=0.2 -> 1.5
    confidence_factor = 2.5 - contact_confidence / 0.2 * 1.0;
  } else if (contact_confidence < 0.5) {
    // 中等置信度：线性映射
    // confidence=0.2 -> 1.5, confidence=0.5 -> 1.0
    confidence_factor = 1.5 - (contact_confidence - 0.2) / 0.3 * 0.5;
  }
  // confidence >= 0.5: factor=1.0（不调整）
  
  // 综合自适应因子（标准差的放大倍数）
  double adaptive_factor = condition_factor * confidence_factor;
  
  // 应用到基准R（R_base_已经是标准差，需要先乘以adaptive_factor再平方）
  // R = (sigma * factor)^2 = sigma^2 * factor^2
  Eigen::VectorXd r_std_adapted = R_base_ * adaptive_factor;  // 自适应后的标准差
  Eigen::VectorXd r_var_adapted = r_std_adapted.cwiseProduct(r_std_adapted);  // 标准差平方得到方差
  return r_var_adapted.asDiagonal();
}

double ArmContactForceKalmanFilter::getEstimateUncertainty() const {
  // 返回协方差矩阵P的迹（所有对角元素之和）
  // 迹越大，估计的不确定性越大
  return P_est_.trace();
}

}  // namespace humanoid
}  // namespace ocs2

