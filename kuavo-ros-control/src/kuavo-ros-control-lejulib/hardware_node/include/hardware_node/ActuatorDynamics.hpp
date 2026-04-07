#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <iomanip>

// ============================================================================
// 执行器动力学补偿原理
// ============================================================================
// 标准刚体逆动力学：τ_rigid = M(q)q̈ + C(q, q̇)q̇ + G(q)
//
// 为对齐关节模组特性，需补偿执行器特性（armature、damping、frictionloss）：
// τ_total = τ_rigid + armature·q̈ + damping·q̇ + frictionloss·tanh(α·q̇)
//
// 其中：
//   - armature·q̈: 电机转子惯量补偿
//   - damping·q̇: 粘滞阻尼补偿（使用测量速度 dq_meas）
//   - frictionloss·tanh(α·q̇): 库伦摩擦补偿（tanh 平滑避免零速度抖动）
//
// 功能说明：
//   - 支持命令速度 (dq_cmd) 和测量速度 (dq_meas) 分别传入
//   - 摩擦损失计算函数 computeFrictionTerm(dq_cmd, dq_meas, params) 可独立修改
//   - 便于后续根据实际需求调整摩擦损失计算逻辑
// ============================================================================

namespace leju_utils {

struct ActuatorDynamicsParams {
  double armature_;
  double damping_;
  double frictionLoss_;
  double frictionThreshold_;
  double frictionAlpha_;
  bool isValid_;

  ActuatorDynamicsParams() : isValid_(false) {}

  ActuatorDynamicsParams(double inertia,
                         double damping,
                         double friction,
                         double frictionThreshold,
                         double frictionAlpha)
      : armature_(inertia),
        damping_(damping),
        frictionLoss_(friction),
        frictionThreshold_(frictionThreshold),
        frictionAlpha_(frictionAlpha),
        isValid_(true) {}
};

inline double computeEffectiveVelocity(double velocity, double threshold) {
  return (std::abs(velocity) < threshold) ? 0.0 : velocity;
}

inline double computeFrictionTerm(double dq_cmd, double dq_meas, const ActuatorDynamicsParams& params) {
  // dq_cmd 与 dq_meas符号相反时，摩擦力方向相反
  double q_switch = dq_cmd * dq_meas < 0.0 ? - dq_cmd : dq_cmd;
  double effectiveVelocity = computeEffectiveVelocity(q_switch, params.frictionThreshold_);
  return params.frictionLoss_ * std::tanh(params.frictionAlpha_ * effectiveVelocity);
}

// 使用命令速度和测量速度计算总力矩
inline double computeTotalTorque(double tauRBD, double ddq, double dq_cmd, double dq_meas, const ActuatorDynamicsParams& params) {
  double frictionTerm = computeFrictionTerm(dq_cmd, dq_meas, params);
  // 阻尼项使用测量速度
  return tauRBD + params.armature_ * ddq + params.damping_ * dq_meas + frictionTerm;
}

// 使用命令速度和测量速度的向量版本
inline std::vector<double> computeTotalTorque(const std::vector<double>& tauRBDs,
                                              const std::vector<double>& ddqs,
                                              const std::vector<double>& dqs_cmd,
                                              const std::vector<double>& dqs_meas,
                                              const std::vector<ActuatorDynamicsParams>& params) {
  std::vector<double> totalTorques(tauRBDs.size());
  for (size_t i = 0; i < totalTorques.size(); ++i) {
    totalTorques[i] = computeTotalTorque(tauRBDs[i], ddqs[i], dqs_cmd[i], dqs_meas[i], params[i]);
  }
  return totalTorques;
}

// 使用命令速度和测量速度的 Eigen 向量版本
inline Eigen::VectorXd computeTotalTorque(const Eigen::VectorXd& tauRBDs,
                                          const Eigen::VectorXd& ddqs,
                                          const Eigen::VectorXd& dqs_cmd,
                                          const Eigen::VectorXd& dqs_meas,
                                          const std::vector<ActuatorDynamicsParams>& params) {
  Eigen::VectorXd totalTorques = tauRBDs;
  for (Eigen::Index i = 0; i < totalTorques.size(); ++i) {
    totalTorques[i] = computeTotalTorque(tauRBDs[i], ddqs[i], dqs_cmd[i], dqs_meas[i], params[i]);
  }
  return totalTorques;
}

class ActuatorDynamicsCompensator {
 public:
  ActuatorDynamicsCompensator() {
    // 尝试从 YAML 文件加载参数，失败则使用默认值
    if (!loadParamsFromYaml()) {
      loadDefaultParams();
      printParamsTable();
    }
  }

 private:
  // 从 YAML 文件加载参数
  bool loadParamsFromYaml() {
    try {
      // 获取当前文件所在目录
      std::filesystem::path current_file(__FILE__);
      std::filesystem::path config_file = current_file.parent_path() / "actuator_dynamics_config.yaml";

      if (!std::filesystem::exists(config_file)) {
        std::cerr << "[ActuatorDynamicsCompensator] Config file not found: "
                  << config_file << ", using default parameters." << std::endl;
        return false;
      }

      YAML::Node config = YAML::LoadFile(config_file.string());

      if (!config["actuator_params"]) {
        std::cerr << "[ActuatorDynamicsCompensator] 'actuator_params' key not found in config file, "
                  << "using default parameters." << std::endl;
        return false;
      }

      const YAML::Node& params_node = config["actuator_params"];
      if (!params_node.IsSequence() || params_node.size() != 14) {
        std::cerr << "[ActuatorDynamicsCompensator] Invalid 'actuator_params' format or size (expected 14), "
                  << "using default parameters." << std::endl;
        return false;
      }

      for (size_t i = 0; i < 14 && i < params_node.size(); ++i) {
        const YAML::Node& param = params_node[i];
        double armature = param["armature"] ? param["armature"].as<double>() : 0.0;
        double damping = param["damping"] ? param["damping"].as<double>() : 0.0;
        double frictionLoss = param["frictionLoss"] ? param["frictionLoss"].as<double>() : 0.0;
        double frictionThreshold = param["frictionThreshold"] ? param["frictionThreshold"].as<double>() : 1e-3;
        double frictionAlpha = param["frictionAlpha"] ? param["frictionAlpha"].as<double>() : 100.0;

        params_[i] = ActuatorDynamicsParams(armature, damping, frictionLoss, frictionThreshold, frictionAlpha);
      }

      std::cout << "[ActuatorDynamicsCompensator] Successfully loaded parameters from: "
                << config_file << std::endl;
      printParamsTable();
      return true;
    } catch (const YAML::Exception& e) {
      std::cerr << "[ActuatorDynamicsCompensator] YAML parsing error: " << e.what()
                << ", using default parameters." << std::endl;
      return false;
    } catch (const std::exception& e) {
      std::cerr << "[ActuatorDynamicsCompensator] Error loading config: " << e.what()
                << ", using default parameters." << std::endl;
      return false;
    }
  }
  // 以表格格式打印参数
  void printParamsTable() {
    std::cout << "\n[ActuatorDynamicsCompensator] Parameters Table:" << std::endl;
    std::cout << std::string(120, '=') << std::endl;
    std::cout << std::left << std::setw(6) << "Joint"
              << std::setw(12) << "Armature"
              << std::setw(12) << "Damping"
              << std::setw(15) << "FrictionLoss"
              << std::setw(18) << "FrictionThreshold"
              << std::setw(15) << "FrictionAlpha"
              << std::endl;
    std::cout << std::string(120, '-') << std::endl;

    for (int i = 0; i < 14; ++i) {
      std::cout << std::left << std::setw(6) << (i + 1)
                << std::setw(12) << std::fixed << std::setprecision(6) << params_[i].armature_
                << std::setw(12) << std::fixed << std::setprecision(6) << params_[i].damping_
                << std::setw(15) << std::fixed << std::setprecision(6) << params_[i].frictionLoss_
                << std::setw(18) << std::scientific << std::setprecision(3) << params_[i].frictionThreshold_
                << std::setw(15) << std::fixed << std::setprecision(1) << params_[i].frictionAlpha_
                << std::endl;
    }
    std::cout << std::string(120, '=') << std::endl << std::endl;
  }

  // 加载默认参数
  void loadDefaultParams() {
    params_[0] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[1] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[2] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[3] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[4] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[5] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[6] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);

    params_[7] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[8] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[9] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[10] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[11] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[12] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
    params_[13] = ActuatorDynamicsParams(0.0, 0.0, 0.0, 1e-3, 100.0);
  }

 public:

  // 使用命令速度和测量速度
  Eigen::VectorXd compute(const Eigen::VectorXd& tauRBDs,
                          const Eigen::VectorXd& ddqs,
                          const Eigen::VectorXd& dqs_cmd,
                          const Eigen::VectorXd& dqs_meas) {
    if (tauRBDs.size() != 14 || ddqs.size() != 14 ||
        dqs_cmd.size() != 14 || dqs_meas.size() != 14) {
      return Eigen::VectorXd::Zero(0);
    }
    Eigen::VectorXd totalTorques(14);
    for (int i = 0; i < 14; ++i) {
      totalTorques[i] = computeTotalTorque(tauRBDs[i], ddqs[i], dqs_cmd[i], dqs_meas[i], params_[i]);
    }
    return totalTorques;
  }

 private:
  ActuatorDynamicsParams params_[14];
};

}  // namespace leju_utils
