#include <Eigen/Dense>
#include "plantIK.h"
#include "drake/solvers/snopt_solver.h"
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

namespace HighlyDynamic
{
  CoMIK::CoMIK(drake::multibody::MultibodyPlant<double> *plant,
               std::vector<std::string> frames_name)
      : plant_{plant}, frames_name_{std::move(frames_name)}
  {
    plant_context_ = plant_->CreateDefaultContext();
    nq_ = plant_->num_positions();
    prev_q_sol.resize(nq_);
    prev_q_sol.setZero();
  }

  drake::solvers::Binding<drake::solvers::Constraint>
  CoMIK::AddCoMPositionConstraint(drake::multibody::InverseKinematics &ik,
                                  const Eigen::Vector3d &r_des)
  {
    auto constraint = std::make_shared<drake::multibody::ComPositionConstraint>(
        (const drake::multibody::MultibodyPlant<double> *)plant_, std::nullopt,
        plant_->world_frame(), plant_context_.get());

    drake::solvers::VectorXDecisionVariable r = ik.get_mutable_prog()->NewContinuousVariables(3);

    drake::solvers::VectorXDecisionVariable vars(ik.q().size() + 3);
    vars << ik.q(), r;
    ik.get_mutable_prog()->AddConstraint(constraint, vars);

    drake::solvers::Binding<drake::solvers::Constraint> b = ik.get_mutable_prog()->AddBoundingBoxConstraint(r_des, r_des, r);

    ik.get_mutable_prog()->SetInitialGuess(r, r_des);
    return b;
  }

  bool CoMIK::solve(const FramePoseVec &pose, const Eigen::VectorXd &q0, Eigen::VectorXd &q_sol, IKParams params)
  {
    drake::multibody::InverseKinematics ik(*plant_);
    // ==================== 约束模式（位姿硬/软 + 三点模式） ====================
    // bit1: pos hard, bit0: ori hard, bit2: three-point position (replaces orientation constraints/costs)
    const uint8_t mode = MapExternalConstraintModeToInternal(static_cast<uint8_t>(params.constraint_mode));
    const bool use_pos_hard = (mode & 0x02) != 0;
    const bool use_ori_hard = (mode & 0x01) != 0;
    const bool use_three_point_pos = (mode & 0x04) != 0;
    const double pos_cost_weight = std::abs(params.pos_cost_weight);

    for (uint32_t i = 0; i < frames_name_.size(); i++)
    {
      const drake::multibody::Frame<double> &frame = plant_->GetFrameByName(frames_name_[i]);
      if (!use_three_point_pos && nq_ > 12 && (i == 1 || i == 2))// 只有手臂大于6自由度时才进行姿态约束
      {
        if (use_ori_hard)
        {
          ik.AddOrientationConstraint(
            plant_->world_frame(),
            drake::math::RotationMatrixd(Eigen::Quaterniond(pose[i].first)), 
            frame,
            drake::math::RotationMatrixd(drake::math::RollPitchYawd(0, 0, 0)), 
            params.oritation_constraint_tol);
        }
        else
        {
          // 使用软约束（成本函数）替代硬约束
          Eigen::Quaterniond normalized_quat = pose[i].first.normalized();
          drake::math::RotationMatrixd target_rotation(normalized_quat);
          ik.AddOrientationCost(
             plant_->world_frame(),          // 参考坐标系（世界）
             target_rotation,                // 期望的目标姿态（归一化后）
             frame,                          // 待约束的帧
             drake::math::RotationMatrixd::Identity(), // 帧的参考旋转（单位矩阵，语义正确）
             params.oritation_constraint_tol);     // 姿态 cost 权重（标量），作为硬约束的时候是容差，作为软约束的时候是权重
        }
      }

      if (i == 1 || i == 2)
      {
        if (use_three_point_pos)
        {
          // Three-point position constraints/costs to indirectly constrain orientation (no explicit orientation term).
          constexpr double kThumbX = 0.15;  // URDF: zarm_*7_virtual_thumb_joint_ee origin xyz="0.15 0 0"

          const Eigen::Matrix3d R_des = pose[i].first.normalized().toRotationMatrix();
          const Eigen::Vector3d p_des = pose[i].second;

          // Determine link7 frame for this hand.
          // i==1: left hand frame; i==2: right hand frame (matches existing code convention).
          const std::string link7_name = (i == 1) ? "zarm_l7_link" : "zarm_r7_link";
          const drake::multibody::Frame<double>& link7_frame = plant_->GetFrameByName(link7_name);

          // Compute the fixed transform X_link7_eef using the stored context.
          // (eef frame is typically a fixed frame attached to link7; the relative transform is constant.)
          plant_->SetPositions(plant_context_.get(), q0);
          const auto X_link7_eef = frame.CalcPose(*plant_context_.get(), link7_frame);
          const auto X_eef_link7 = X_link7_eef.inverse();

          const Eigen::Vector3d P_eef = Eigen::Vector3d::Zero();
          const Eigen::Vector3d P_l7 = X_eef_link7.translation();  // link7 origin expressed in eef frame
          const Eigen::Vector3d P_thumb(kThumbX, 0.0, 0.0);

          const Eigen::Vector3d P_local[3] = {
              Eigen::Vector3d::Zero(),           // eef origin
              P_l7,                       // link7 origin expressed in eef
              Eigen::Vector3d(kThumbX, 0, 0)};   // virtual thumb point

          const Eigen::Vector3d P_world[3] = {
            p_des,
            p_des + R_des * P_l7,
            p_des + R_des * P_thumb
          };

          if (use_pos_hard)
          {
            const Eigen::Vector3d tol = Eigen::Vector3d::Constant(params.pos_constraint_tol);
            for (int k = 0; k < 3; ++k)
            {
              if(0 == k)
              {
                ik.AddPositionConstraint(frame, P_local[k], plant_->world_frame(), P_world[k] - tol, P_world[k] + tol);
              }
              else
              {
                const Eigen::Matrix3d W = pos_cost_weight * Eigen::Matrix3d::Identity();
                ik.AddPositionCost(plant_->world_frame(), P_world[k], frame, P_local[k], W);
              }
            }
          }
          else
          {
            const Eigen::Matrix3d W = pos_cost_weight * Eigen::Matrix3d::Identity();
            for (int k = 0; k < 3; ++k)
              ik.AddPositionCost(plant_->world_frame(), P_world[k], frame, P_local[k], W);
          }
        }
        else if (use_pos_hard)
        {
          ik.AddPositionConstraint(frame, 
                                  Eigen::Vector3d::Zero(),
                                  plant_->world_frame(),
                                  pose[i].second - Eigen::Vector3d::Constant(params.pos_constraint_tol),
                                  pose[i].second + Eigen::Vector3d::Constant(params.pos_constraint_tol));
        }
        else
        {
          ik.AddPositionCost(
                      plant_->world_frame(),
                      pose[i].second,
                      frame,
                      Eigen::Vector3d::Zero(),
                      pos_cost_weight * Eigen::MatrixXd::Identity(3, 3));
        }
      }
      else if(i==3 || i==4) 
      {
        //elbow
        if(!pose[i].second.isZero())
        {
          std::cout << "add position cost for elbow" << std::endl;
          // elbow位置全为0时，肯定是无效位置，忽略它
          ik.AddPositionCost(
                      plant_->world_frame(),
                      pose[i].second,
                      frame,
                      Eigen::Vector3d::Zero(),
                      0.1 * pos_cost_weight * Eigen::MatrixXd::Identity(3, 3));
        }
        else
        {
          // std::cout << "pose[i].second is zero" << std::endl;
        }
      }
    }
    ik.get_mutable_prog()->SetInitialGuess(ik.q(), q0);
    ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", params.major_feasibility_tol);
    ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor feasibility tolerance", params.minor_feasibility_tol);
    ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(), "Major Optimality Tolerance", params.major_optimality_tol);
    ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(), "Major Iterations Limit", params.major_iterations_limit);

    // ik.get_mutable_prog()->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file", "tmp/snopt.out");
    drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(ik.prog());
    if (!result.is_success())
    {
      // std::cout << "Failed solution: " << result.GetSolution(ik.q()).transpose() << "\n";
      // std::cout << "Previous solution: " << prev_q_sol.transpose() << "\n";
      return false;
    }
    else
    {
      q_sol = result.GetSolution(ik.q());
      prev_q_sol = q_sol;
      return true;
    }
  }

  std::pair<Eigen::Vector3d, Eigen::Quaterniond> CoMIK::FK(const Eigen::VectorXd& q, HandSide side) {
    plant_->SetPositions(plant_context_.get(), q);
    std::string frame_name;
    if(side == HandSide::LEFT)
      frame_name = frames_name_[1];
    else if(side == HandSide::RIGHT)
      frame_name = frames_name_[2];
    auto pose = plant_->GetFrameByName(frame_name).CalcPose(*plant_context_.get(), plant_->GetFrameByName(frames_name_[0]));
    return std::make_pair(pose.translation(), pose.rotation().ToQuaternion());
  }

} // namespace drake
