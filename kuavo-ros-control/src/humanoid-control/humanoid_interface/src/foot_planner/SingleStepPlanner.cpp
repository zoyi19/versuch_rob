#include "humanoid_interface/foot_planner/SingleStepPlanner.h"

namespace ocs2 {
namespace humanoid {
  std::pair<Eigen::Vector3d, Eigen::Vector3d> generateSteps(const Eigen::Vector3d& torso_pos, const double torso_yaw, const double foot_bias)
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

  bool getMultipleStepsTrajectory(FootPoseSchedule &foot_pose_schedule, const std::vector<vector6_t>& body_poses, double dt, double foot_bias, bool collision_check)
  {
    foot_pose_schedule.clear();
    int num_steps = 2 * body_poses.size();
    auto &time_traj = foot_pose_schedule.eventTimes;
    auto &foot_idx_traj = foot_pose_schedule.footIndices;
    auto &foot_traj = foot_pose_schedule.footPoseSequence;
    auto &torso_traj = foot_pose_schedule.torsoPoseSequence;
    auto &swing_height_traj = foot_pose_schedule.swingHeightSequence;

    RotatingRectangle l_foot_rect_last(Eigen::Vector2d(0, 0.1), 0.24, 0.1, 0);
    RotatingRectangle r_foot_rect_last(Eigen::Vector2d(0, -0.1), 0.24, 0.1, 0);
    bool is_left_first = true;

    Vector4d torso_pose_last = Vector4d::Zero();
    for (int i = 0; i < num_steps; ++i) {
      time_traj.push_back(dt * (i + 1));
      Vector4d body_pose = body_poses[i / 2].head<4>();
      Eigen::Vector3d torso_pos(body_pose[0], body_pose[1], body_pose[2]);
      double torso_yaw = body_pose[3] * M_PI / 180.0;

      auto [l_foot, r_foot] = generateSteps(torso_pos, torso_yaw, foot_bias);
      Vector4d l_foot_vec(l_foot[0], l_foot[1], l_foot[2], torso_yaw);
      Vector4d r_foot_vec(r_foot[0], r_foot[1], r_foot[2], torso_yaw);

      if (i % 2 == 0) {
        is_left_first = (torso_yaw - torso_pose_last[3] >= 0.0);

        if (collision_check)
        {
          RotatingRectangle l_foot_rect_next(Eigen::Vector2d(l_foot[0], l_foot[1]), 0.24, 0.1, torso_yaw);
          RotatingRectangle r_foot_rect_next(Eigen::Vector2d(r_foot[0], r_foot[1]), 0.24, 0.1, torso_yaw);

          bool l_collision = l_foot_rect_next.is_collision(r_foot_rect_last);
          bool r_collision = r_foot_rect_next.is_collision(l_foot_rect_last);

          if (l_collision && r_collision) {
            std::cout << "[Error] Detect collision, Please adjust your body_poses input!!!" << std::endl;
            return false;
          }
          if (l_collision) {
            printf("[Info] Left foot[%d] is in collision, switch to right foot.\n", 1 + i / 2);
            is_left_first = false;
          }
          if (r_collision) {
            printf("[Info] Right foot[%d] is in collision, switch to left foot.\n", 1 + i / 2);
            is_left_first = true;
          }

          l_foot_rect_last = l_foot_rect_next;
          r_foot_rect_last = r_foot_rect_next;
        }
      }

      int foot_idx = is_left_first ? 0 : 1;
      auto next_body_pose = Vector4d(body_pose[0], body_pose[1], body_pose[2], torso_yaw);
      if (i % 2 == 0) {
        // 对于6D姿态，我们需要分别处理位置和姿态
        vector6_t mid_pose = vector6_t::Zero();
        mid_pose.head<4>() = (torso_pose_last + next_body_pose)/2.0;
       
        torso_traj.push_back(mid_pose);
        foot_idx_traj.push_back(static_cast<FootIdx>(foot_idx));
        vector6_t foot_pose = vector6_t::Zero();
        foot_pose.head<4>() = is_left_first ? l_foot_vec : r_foot_vec;
        foot_traj.push_back(foot_pose);
        swing_height_traj.push_back(0.06);
      } else {
        vector6_t mid_pose = vector6_t::Zero();
        mid_pose.head<4>() = next_body_pose;
        torso_traj.push_back(mid_pose);
        foot_idx = 1 - foot_idx;
        foot_idx_traj.push_back(static_cast<FootIdx>(foot_idx));
        vector6_t foot_pose = vector6_t::Zero();
        foot_pose.head<4>() = is_left_first ? r_foot_vec : l_foot_vec;
        foot_traj.push_back(foot_pose);
        swing_height_traj.push_back(0.06);
      }

      // torso_traj.push_back(Vector4d(body_pose[0], body_pose[1], body_pose[2], torso_yaw));
      torso_pose_last = torso_traj.back().head<4>();
    }
    return true;
  }
}  // namespace humanoid
}  // namespace ocs2
