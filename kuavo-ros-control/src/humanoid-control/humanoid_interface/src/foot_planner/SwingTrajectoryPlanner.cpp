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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "humanoid_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <humanoid_interface/gait/MotionPhaseDefinition.h>

namespace ocs2
{
    namespace humanoid
    {

        scalar_t computeCenterTime(const std::vector<scalar_t>& eventTimes, int startIndex, int endIndex) {
            return (eventTimes[startIndex] + eventTimes[endIndex]) / 2.0;
        }
        
        // ********已知脚掌上3点位置，求解脚掌的位姿*********
        /*
            例如已知三个足端点相对于末端link的位置为：
            std::vector<vector3_t> local_foot_points = {// 0,1,2个接触点
                {0.16902, 0.04773, -0.0593},
                {0.16902, -0.04773, -0.0593},
                {-0.07316, 0.04773, -0.0593},
                };
            
            传入对应三个足端点的世界坐标系下的位置，即可求解末端link的位姿
        */
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> calculateWorldPose(
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
            return std::make_pair(t, R);
        }
        
        // 已知平面上3个点，求解平面法向量
        inline Eigen::Vector3d getNormalVector(const std::vector<Eigen::Vector3d>& plane_points)
        {
            // 1. 计算平面法向量
            Eigen::Vector3d v1 = plane_points[1] - plane_points[0];
            Eigen::Vector3d v2 = plane_points[2] - plane_points[0];
            Eigen::Vector3d normal = v1.cross(v2).normalized();
            return normal;
        }
        
        // 计算足底平面与水平面的夹角（弧度）
        inline double calculateSlopeAngle(const Eigen::Vector3d& normal_vector)
        {
            // 水平面法向量 [0, 0, 1]
            Eigen::Vector3d horizontal_normal(0, 0, -1);
            
            // 计算两个法向量的夹角
            double cos_angle = normal_vector.dot(horizontal_normal);
            // 确保cos_angle在[-1, 1]范围内，避免数值误差
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            
            // 返回夹角（弧度）
            return std::acos(cos_angle);
        }
        
        // 计算足底平面与水平面的夹角（度数）
        inline double calculateSlopeAngleDegrees(const Eigen::Vector3d& normal_vector)
        {
            return calculateSlopeAngle(normal_vector) * 180.0 / M_PI;
        }
        
        // 计算足底平面的倾斜方向（坡度方向角，弧度）
        inline double calculateSlopeDirection(const Eigen::Vector3d& normal_vector)
        {
            // 计算法向量在水平面上的投影
            Eigen::Vector2d horizontal_projection(normal_vector(0), normal_vector(1));
            
            if (horizontal_projection.norm() < 1e-6) {
                return 0.0; // 如果投影很小，认为没有方向性
            }
            
            // 计算方向角（相对于x轴正方向）
            return std::atan2(horizontal_projection(1), horizontal_projection(0));
        }
        
        double calculateHeightDifference(const Eigen::Vector3d& normal_vector, const Eigen::Vector2d& point1, const Eigen::Vector2d& point2)
        {
            // 2. 平面方程: ax + by + cz + d = 0
            // 其中(a,b,c)是法向量，代入平面上一点可求d
            double a = normal_vector(0);
            double b = normal_vector(1);
            double c = normal_vector(2);
           
            // 3. 计算两点的z值差
            // z = -(ax + by + d)/c
            double z_diff = -(a * (point2.x() - point1.x()) + 
                            b * (point2.y() - point1.y())) / c;
            return z_diff;
        }
        // 已知平面上3个点，求解平面上某个两个已知xy的点的相对高度
        double calculateHeightDifference(
            const std::vector<Eigen::Vector3d>& plane_points,  // 平面上的3个点
            const Eigen::Vector2d& known_point,   // (x0,y0)
            const Eigen::Vector2d& query_point)  // (x1,y1)
        {
            Eigen::Vector3d normal = getNormalVector(plane_points);
            return calculateHeightDifference(normal, known_point, query_point);
        }


        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, 
                                                       const PinocchioInterface& interface, 
                                                       const CentroidalModelInfo& info,
                                                       size_t numFeet, size_t numHand) 
            : config_(std::move(config)), 
            numFeet_(numFeet), 
            numHand_(numHand) , 
            current_feet_position_buf_{std::move(feet_array_t<vector3_t>{})}, 
              current_body_state_buf_{std::move(vector_t{})}, 
              body_vel_cmd_buf_{std::move(vector_t{})},
              current_state_{std::move(vector_t{})},
              armTargetTrajectories_{std::move(TargetTrajectories())}
        {
            defaultConfig_ = config_;

            const auto& model = interface.getModel();
            auto data = interface.getData();
            // 计算正向运动学
            pinocchio::forwardKinematics(model, data, info.qPinocchioNominal);
            pinocchio::updateFramePlacements(model, data);  // 必须调用，否则 data.oMf 不会更新
            // 获取每个接触点的位置
            for(int i=0; i<info.numThreeDofContacts; i++)
            {
                const pinocchio::SE3& frame_pose = data.oMf[info.endEffectorFrameIndices[i]];
                // 提取位置和旋转
                Eigen::Vector3d position = frame_pose.translation();
                feetXYOffset_[i] << position[0], position[1], 0;
                // std::cout << "feetXYOffset_ [" << i << "]:  " << feetXYOffset_[i].transpose() << std::endl;  
            }
            // scalar_t x_abs = 0.12;
            // scalar_t y_outside = 0.15, y_inside = 0.05;

            // feetXYOffset_[0] << x_abs, y_outside, 0;
            // feetXYOffset_[1] << x_abs, y_inside, 0;
            // feetXYOffset_[2] << -x_abs, y_outside, 0;
            // feetXYOffset_[3] << -x_abs, y_inside, 0;
            // feetXYOffset_[4] << x_abs, -y_inside, 0;
            // feetXYOffset_[5] << x_abs, -y_outside, 0;
            // feetXYOffset_[6] << -x_abs, -y_inside, 0;
            // feetXYOffset_[7] << -x_abs, -y_outside, 0;
            // local frame
            feetXYOffsetLocalFrame_ = feetXYOffset_;
            for (size_t i = 0; i < 4; i++)
                feetXYOffsetLocalFrame_[i](1) += -0.1;
            for (size_t i = 4; i < 8; i++)
                feetXYOffsetLocalFrame_[i](1) += 0.1;
            // for(size_t i = 0; i < 8; i++)
            // {
            //     std::cout << "feetXYOffsetLocalFrame_[" << i << "]: " << feetXYOffsetLocalFrame_[i].transpose() << std::endl;
            // }

            armTargetTrajectories_.setBuffer(TargetTrajectories{{0.0}, {vector_t::Zero(num_arm_joints_)}, {vector_t::Zero(num_arm_joints_)}});

            last_calc_stance_position_.fill(vector3_t::Zero());
            latestStanceposition_.fill(vector3_t::Zero());
            latestStancepositionZ_.fill(0.0);
            lf_normal_vector_ = Eigen::Vector3d::Zero();
            lf_normal_vector_[2] = 1.0;
            rf_normal_vector_ = Eigen::Vector3d::Zero();
            rf_normal_vector_[2] = 1.0;

        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::getXvelocityConstraint(size_t leg, scalar_t time) const
        {
          const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
          return footXInterpolators_[leg][index].velocity(time);
        }
        
        void SwingTrajectoryPlanner::printFeetPositions(size_t leg) const
        {
            auto &feet_positions = feetXTrajectories_[leg];
            for (size_t i = 0; i < feet_positions.size(); i++)
            {
                auto &cubicSp = feet_positions[i];//CubicSpline
                auto startnode = cubicSp.start();
                auto endnode = cubicSp.end();
                std::cout << "leg:"<<leg<<" i:"<<i<<"\n";
                std::cout << "st: " << startnode.time << " et: " << endnode.time << std::endl;
                std::cout << "from: " << startnode.position << " to: " << endnode.position << std::endl;
            }
            std::cout << "\n\n";
            
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::getYvelocityConstraint(size_t leg, scalar_t time) const
        {
          const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
          return footYInterpolators_[leg][index].velocity(time);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const
        {
            const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
            return footZInterpolators_[leg][index].velocity(time);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::getXpositionConstraint(size_t leg, scalar_t time) const
        {
          const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
          return footXInterpolators_[leg][index].position(time);
        }
        
        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::getYpositionConstraint(size_t leg, scalar_t time) const
        {
          const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
          return footYInterpolators_[leg][index].position(time);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const
        {
            const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
            return footZInterpolators_[leg][index].position(time);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        feet_array_t<vector3_t> SwingTrajectoryPlanner::getFootPositionConstraint(scalar_t time) const
        {
            feet_array_t<vector3_t> footPosition;
            // std::cout << "time: " << time << std::endl;
            if(feetHeightTrajectoriesEvents_[0].size()==0)
            {
                std::cout << "feetHeightTrajectoriesEvents_[0].size() = 0" << std::endl;
                footPosition.fill(vector3_t::Zero());
                return std::move(footPosition);
            }
            // std::cout << "size z: " << feetHeightTrajectoriesEvents_.size() << std::endl;
            // std::cout << "size p0: " << feetHeightTrajectoriesEvents_[0].size() << std::endl;
            const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[0], time);
            // std::cout << "time: " << time << " index: " << index << std::endl;
            for (size_t i = 0; i < numFeet_; i++)
            {
                footPosition[i] = vector3_t::Zero();
                // std::cout << "x: " << feetXTrajectories_[i][index].position(time) << std::endl;
                footPosition[i](0) = footXInterpolators_[i][index].position(time);
                footPosition[i](1) = footYInterpolators_[i][index].position(time);
                footPosition[i](2) = footZInterpolators_[i][index].position(time);
                // std::cout << "footPosition[" << i << "]: " << footPosition[i].transpose() << std::endl;
            }
            return std::move(footPosition);
        }



        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        // scalar_t SwingTrajectoryPlanner::getArmJointVelocityConstraint(size_t hand, scalar_t time) const
        // {
        //     const auto index = lookup::findIndexInTimeArray(armJointTrajectoriesEvents_[hand],time);
        //     return armJointTrajectories_[hand][index].velocity(time);
        // }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void SwingTrajectoryPlanner::modifyHeightSequences(const ModeSchedule &modeSchedule,
                                                            feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                                            feet_array_t<scalar_array_t> &touchDownHeightSequence) 
        {
            const auto &eventTimes = modeSchedule.eventTimes;
            const auto &modeSequence = modeSchedule.modeSequence;
            const auto &footPoseSequence = modeSchedule.footPoseSequence;

            for (size_t j = 0; j < numFeet_; j++)
            {
                int last_mode_l = 1;
                int last_mode_r = 1;
                for (int p = 1; p < modeSequence.size()-1; ++p)
                {
                    // (p-1) is the previous mode, p is the current mode, (p+1) is the next mode
                    // left foot
                    if(modeSequence[p-1] == ModeNumber::SS && modeSequence[p] == ModeNumber::FS && modeSequence[p+1] == ModeNumber::SS)
                    {
                        if(j < numFeet_/2)
                        {
                            liftOffHeightSequence[j][p-1] = footPoseSequence[p-1](2);
                            liftOffHeightSequence[j][p+0] = footPoseSequence[p-1](2);
                            liftOffHeightSequence[j][p+1] = footPoseSequence[p+1](2);
                            touchDownHeightSequence[j][p-1] = footPoseSequence[p+1](2);
                            touchDownHeightSequence[j][p+0] = footPoseSequence[p+1](2);
                            touchDownHeightSequence[j][p+1] = footPoseSequence[p+1](2);
                        }
                        last_mode_l = p+1;
                    }
                    // right foot
                    else if(modeSequence[p-1] == ModeNumber::SS && modeSequence[p] == ModeNumber::SF && modeSequence[p+1] == ModeNumber::SS)
                    {
                        if(j >= numFeet_/2)
                        {
                            liftOffHeightSequence[j][p-1] = footPoseSequence[p-1](2);
                            liftOffHeightSequence[j][p+0] = footPoseSequence[p-1](2);
                            liftOffHeightSequence[j][p+1] = footPoseSequence[p+1](2);
                            touchDownHeightSequence[j][p-1] = footPoseSequence[p+1](2);
                            touchDownHeightSequence[j][p+0] = footPoseSequence[p+1](2);
                            touchDownHeightSequence[j][p+1] = footPoseSequence[p+1](2);
                        }
                        last_mode_r = p;
                    }
                }
                // fresh all the remaining modes to the last mode
                if(j < numFeet_/2)
                    for(int p=last_mode_l+1; p<liftOffHeightSequence.size()-1; ++p)
                    {
                        liftOffHeightSequence[j][p] = touchDownHeightSequence[j][last_mode_l];
                        touchDownHeightSequence[j][p] = touchDownHeightSequence[j][last_mode_l];
                    }
                if(j >= numFeet_/2)
                    for(int p=last_mode_r+1; p<liftOffHeightSequence.size()-1; ++p)
                    {
                        liftOffHeightSequence[j][p] = touchDownHeightSequence[j][last_mode_r];
                        touchDownHeightSequence[j][p] = touchDownHeightSequence[j][last_mode_r];
                    }
            }
            if(0)
            {
                std::cout << "p0 liftOffHeightSequence and touchDownHeightSequence:" << std::endl;
                for(int i=0;i < liftOffHeightSequence.size();i++)
                {
                    std::cout << modeNumber2String(modeSequence[i]) << "[" << i << "]: [";
                    std::cout << liftOffHeightSequence[0][i] << ", ";
                    std::cout << touchDownHeightSequence[0][i] << "] ";
                }
                std::cout << "\n\n";
            }
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, scalar_t terrainHeight, const TargetTrajectories& targetTrajectories, 
                                            const scalar_t& initTime)
        {
            const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
            feet_array_t<scalar_array_t> liftOffHeightSequence;
            liftOffHeightSequence.fill(terrainHeightSequence);
            feet_array_t<scalar_array_t> touchDownHeightSequence;
            touchDownHeightSequence.fill(terrainHeightSequence);
            if(modeSchedule.existValidFootPose())
                modifyHeightSequences(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
            update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, targetTrajectories, initTime);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                            const feet_array_t<scalar_array_t> &touchDownHeightSequence, const TargetTrajectories& targetTrajectories, 
                                            const scalar_t& initTime)
        {
            scalar_array_t heightSequence(modeSchedule.modeSequence.size());
            feet_array_t<scalar_array_t> maxHeightSequence;
            for (size_t j = 0; j < numFeet_; j++)
            {
                for (int p = 0; p < modeSchedule.modeSequence.size(); ++p)
                {
                    heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);
                }
                maxHeightSequence[j] = heightSequence;
            }
            update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence, targetTrajectories, initTime);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                            const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                                            const feet_array_t<scalar_array_t> &maxHeightSequence, 
                                            const TargetTrajectories& targetTrajectories, 
                                            const scalar_t& initTime)
        {
            const auto &modeSequence = modeSchedule.modeSequence;
            const auto &eventTimes = modeSchedule.eventTimes;
            // const auto &fullBodyStateSequence = modeSchedule.fullBodyStateSequence;
            // const auto &timeTrajectorySequence = modeSchedule.timeTrajectorySequence;
            const auto &enableFullBodySequence = modeSchedule.enableFullBodySequence;
            has_feet_trajectory_ = modeSchedule.existValidFootPose(initTime);
            std::vector<scalar_array_t> timeTrajectorySequence;
            std::vector<std::vector<feet_array_t<vector3_t>>> feet_pos_sequence;
            for(size_t i = 0; i < modeSequence.size(); i++)
            {
                if (!enableFullBodySequence[i] || modeSequence[i] == ModeNumber::SS)
                {
                    feet_pos_sequence.push_back(std::vector<feet_array_t<vector3_t>>());
                    timeTrajectorySequence.push_back(std::vector<scalar_t>());
                    continue;
                }
                double foot_traj_dt = 0.01;
                double start_time = eventTimes[i-1];
                double end_time = eventTimes[i];
                scalar_array_t time_traj;
                std::vector<feet_array_t<vector3_t>> feet_pos;
                for (double time = start_time + foot_traj_dt; time < end_time; time += foot_traj_dt)
                {
                    time_traj.push_back(time);
                    feet_pos.push_back(inverseKinematics_->computeFootPos(targetTrajectories.getDesiredState(time)));
                }

                timeTrajectorySequence.push_back(time_traj);
                feet_pos_sequence.push_back(feet_pos);
            }
            
            const auto eesContactFlagStocks = extractContactFlags(modeSequence);

            current_body_state_buf_.updateFromBuffer();
            const auto& currentBodyState = current_body_state_buf_.get();
            
            body_vel_cmd_buf_.updateFromBuffer();
            const auto& body_vel_cmd_ = body_vel_cmd_buf_.get();

            current_feet_position_buf_.updateFromBuffer();
            const auto& currentFeetPosition = current_feet_position_buf_.get();
            contact_flag_t cmd_state_leg = modeNumber2StanceLeg(modeSchedule.modeAtTime(initTime + 0.001));
            std::vector<vector3_t> left_foot_position;
            std::vector<vector3_t> right_foot_position;
            bool lf_contact = std::all_of(cmd_state_leg.begin(), cmd_state_leg.begin() + 4, [](int flag)
                                  { return flag; });
            bool rf_contact = std::all_of(cmd_state_leg.begin() + 4, cmd_state_leg.end(), [](int flag)
                                  { return flag; });

            for(size_t leg = 0; leg < numFeet_; leg++)
            {   
                double last_z = latestStanceposition_[leg].z();
                latestStanceposition_[leg] = cmd_state_leg[leg] ? currentFeetPosition[leg] : latestStanceposition_[leg];
                latestStancepositionZ_[leg] = cmd_state_leg[leg] ? currentFeetPosition[leg].z() : latestStancepositionZ_[leg];// 记录实际落足点Z坐标
                latestFullyStanceposition_[leg] = ((lf_contact && leg < numFeet_ / 2) || (rf_contact && leg >= numFeet_ / 2)) ? currentFeetPosition[leg] : latestFullyStanceposition_[leg];
                if (cmd_state_leg[leg] && feetHeightTrajectoriesEvents_[leg].size() > 0)
                {
                    last_z = this->getZpositionConstraint(leg, initTime);
                }
                if (!enable_slope_planning_)// 非斜坡规划时，使用规划值作为起点，避免漂移
                    latestStanceposition_[leg].z() = last_z;


                visualize_stance_position_flag_[leg] = false;
                if (leg < numFeet_/2)
                {
                    left_foot_position.push_back(latestFullyStanceposition_[leg]);
                }else
                {
                    right_foot_position.push_back(latestFullyStanceposition_[leg]);
                }
                // std::cout << "leg: " << leg << " latestFullyStanceposition_: " << latestFullyStanceposition_[leg].transpose() << std::endl;
            }

            if (!has_feet_trajectory_)
            {
                stancePositionBeforeStepControl_ = latestStanceposition_;
            }
            auto diff_pos = left_foot_position[0] - left_foot_position[2];
            auto diff_vel = diff_pos[2] / diff_pos[0];

            auto new_lf_normal_vector_ = getNormalVector(left_foot_position);
            auto new_rf_normal_vector_ = getNormalVector(right_foot_position);
            double alpha = 0.2;
            lf_normal_vector_ = lf_normal_vector_ * (1-alpha) + new_lf_normal_vector_ * alpha;
            rf_normal_vector_ = rf_normal_vector_ * (1-alpha) + new_rf_normal_vector_ * alpha;
            
            // 计算足底平面与水平面的夹角
            // double lf_slope_angle_rad = calculateSlopeAngle(lf_normal_vector_   );
            // double rf_slope_angle_rad = calculateSlopeAngle(rf_normal_vector_);
            double lf_slope_angle_deg = calculateSlopeAngleDegrees(lf_normal_vector_);
            double rf_slope_angle_deg = calculateSlopeAngleDegrees(rf_normal_vector_);
            
            // // 计算足底平面的倾斜方向
            // double lf_slope_direction = calculateSlopeDirection(lf_normal_vector_);
            // double rf_slope_direction = calculateSlopeDirection(rf_normal_vector_);

            // end_of_slope_planning threshold
            if (enable_slope_planning_ && ((lf_contact && lf_slope_angle_deg < config_.slope_planning_threshold) || (rf_contact && rf_slope_angle_deg < config_.slope_planning_threshold)))
            {
                enable_slope_planning_ = false;
                // std::cout << "slope_planning_threshold" << std::endl;
            }
            
           
            
            // std::cout << "lf_normal_vector_:" << lf_normal_vector_.transpose() << std::endl;
            // std::cout << "rf_normal_vector_:" << rf_normal_vector_.transpose() << std::endl;
            // std::cout << "diff_pos: " << diff_pos.transpose() << std::endl;
            // std::cout << "diff_vel: " << diff_vel << std::endl;
            feet_array_t<vector3_t> last_stance_position = latestStanceposition_;
            feet_array_t<vector3_t> next_stance_position = latestStanceposition_;
            feet_array_t<int> last_final_idx{};
            
            // bool is_walking_on_slope = false;
            // double is_slope_threshold = 0.08;
            // if (new_lf_normal_vector_.head(2).norm() > is_slope_threshold && new_rf_normal_vector_.head(2).norm() > is_slope_threshold)
            //     is_walking_on_slope = true;
            feet_array_t<std::vector<int>> startTimesIndices;
            feet_array_t<std::vector<int>> finalTimesIndices;
            for (size_t leg = 0; leg < numFeet_; leg++)
            {
                std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
            }

            for (size_t j = 0; j < numFeet_; j++)
            {
                footXInterpolators_[j].clear();
                footXInterpolators_[j].reserve(modeSequence.size());
                footYInterpolators_[j].clear();
                footYInterpolators_[j].reserve(modeSequence.size());
                footZInterpolators_[j].clear();
                footZInterpolators_[j].reserve(modeSequence.size());
                double current_height_offset = 0.0;
                bool verbose = false;

                if (verbose)
                {
                    std::cout << "\n\nj:" << j << std::endl;
                    std::cout << "modeSequence.size()" << modeSequence.size() << " initTime:" << initTime << std::endl;
                }
                bool calc_next_stance_position_success = false;
                std::vector<vector3_t> additional_stance_positions;
                for (int p = 0; p < modeSequence.size(); ++p)
                {
                    if (verbose)
                        std::cout << eventTimes[p] << " mode:" << modeSequence[p] << std::endl;
                    if (!eesContactFlagStocks[j][p])
                    { // for a swing leg
                        const int swingStartIndex = startTimesIndices[j][p];
                        const int swingFinalIndex = finalTimesIndices[j][p];
                        int fullySwingStartIndex = swingStartIndex;
                        int fullySwingFinalIndex = swingFinalIndex;
                        const scalar_t swingStartTime = eventTimes[swingStartIndex];
                        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
                        // std::cout << "swingStartTime: " << swingStartTime << " swingFinalTime: " << swingFinalTime << std::endl;
                        if (enableFullBodySequence[p])// using full body trajectory
                        {
                            std::vector<double> time_sequence;
                            std::vector<double> x_sequence;
                            std::vector<double> y_sequence;
                            std::vector<double> z_sequence;
                            std::vector<feet_array_t<vector3_t>> feet_pos_sequence_traj = feet_pos_sequence[p];
                            std::vector<double> time_sequence_traj = timeTrajectorySequence[p];
                            
                            // 记录原始序列
                            for (size_t feet_pos_idx = 0; feet_pos_idx < feet_pos_sequence_traj.size(); feet_pos_idx++)
                            {
                                time_sequence.push_back(time_sequence_traj[feet_pos_idx]);
                                // std::cout << "time_sequence_traj[feet_pos_idx]: " << time_sequence_traj[feet_pos_idx] << std::endl;
                                x_sequence.push_back(feet_pos_sequence_traj[feet_pos_idx][j][0]);
                                y_sequence.push_back(feet_pos_sequence_traj[feet_pos_idx][j][1]);
                                z_sequence.push_back(std::max(feet_pos_sequence_traj[feet_pos_idx][j][2], 0.0));
                            }
                            
                            // 处理z序列
                            if (z_sequence.size() > 12) {
                                const double time_margin = 0.05;  // 50ms的时间边界
                                const double distance_threshold = 0.01;  // 1cm的高度阈值
                                const double max_velocity = 0.1;  // 最大允许的z方向速度(m/s)
                                
                                std::vector<double> new_z_sequence;
                                std::vector<double> new_time_sequence;
                                
                                // 添加起始零点
                                new_time_sequence.push_back(swingStartTime);
                                new_z_sequence.push_back(0.0);
                                
                                // 检查起始阶段的速度
                                size_t start_idx = 0;
                                while (start_idx < z_sequence.size()/3) {  // 只检查前1/3的序列
                                    double dt = time_sequence[start_idx+1] - time_sequence[start_idx];
                                    double dz = std::abs(z_sequence[start_idx+1] - z_sequence[start_idx]);
                                    double vel = dz / dt;
                                    
                                    if (vel > max_velocity) {
                                        start_idx++;
                                    } else {
                                        break;
                                    }
                                }
                                
                                // 检查结束阶段的速度
                                size_t end_idx = z_sequence.size() - 1;
                                while (end_idx > z_sequence.size()*2/3) {  // 只检查后1/3的序列
                                    double dt = time_sequence[end_idx] - time_sequence[end_idx-1];
                                    double dz = std::abs(z_sequence[end_idx] - z_sequence[end_idx-1]);
                                    double vel = dz / dt;
                                    
                                    if (vel > max_velocity) {
                                        end_idx--;
                                    } else {
                                        break;
                                    }
                                }
                                
                                // 添加中间序列点
                                for (size_t i = start_idx; i <= end_idx; i++) {
                                    if (time_sequence[i] <= swingStartTime + time_margin || 
                                        time_sequence[i] >= swingFinalTime - time_margin) {
                                        continue;
                                    }
                                    new_time_sequence.push_back(time_sequence[i]);
                                    new_z_sequence.push_back(z_sequence[i]);
                                }
                                
                                // 添加结束零点
                                new_time_sequence.push_back(swingFinalTime);
                                new_z_sequence.push_back(0.0);
                                
                                // 调试输出
                                if (verbose) {
                                    std::cout << "起始段调整: 移除了 " << start_idx << " 个点" << std::endl;
                                    std::cout << "结束段调整: 移除了 " << (z_sequence.size()-1 - end_idx) << " 个点" << std::endl;
                                }
                                
                                footZInterpolators_[j].emplace_back(DrakeShapedInterpolator(new_time_sequence, new_z_sequence));
                                z_sequence = new_z_sequence;
                            }
                            else
                            {
                                footZInterpolators_[j].emplace_back(DrakeShapedInterpolator(time_sequence, z_sequence));
                            }

                            next_stance_position[j] = {x_sequence.back(), y_sequence.back(), z_sequence.back()};

                            if (!visualize_stance_position_flag_[j])// 显示第一个落点
                            {
                                calc_stance_position_[j] = next_stance_position[j];
                                visualize_stance_position_flag_[j] = true;
                            }

                            footXInterpolators_[j].emplace_back(DrakeShapedInterpolator(time_sequence, x_sequence));
                            footYInterpolators_[j].emplace_back(DrakeShapedInterpolator(time_sequence, y_sequence));
                            
                            continue;
                        }
                        auto findFullySwing = [&modeSequence, &swingStartIndex, &swingFinalIndex, &startTimesIndices, &finalTimesIndices,&j,&p,&eesContactFlagStocks](int& fullySwingStartIndex, int& fullySwingFinalIndex)
                        {
                            std::vector<int> swing_contact_indices;
                            for(int i = swingStartIndex+1; i<=swingFinalIndex; i++)// 遍历悬空时间
                            {
                                 switch(j){ 
                                    case 0:
                                    case 1:
                                        if(!eesContactFlagStocks[2][i]&&!eesContactFlagStocks[3][i])
                                            swing_contact_indices.push_back(i); 
                                        break;
                                    case 2:
                                    case 3:  
                                        if (!eesContactFlagStocks[0][i]&&!eesContactFlagStocks[1][i])
                                            swing_contact_indices.push_back(i); 
                                        break;
                                    case 4:
                                    case 5:
                                        if (!eesContactFlagStocks[6][i]&&!eesContactFlagStocks[7][i])
                                            swing_contact_indices.push_back(i); 
                                            break;
                                    case 6:
                                    case 7: 
                                        if (!eesContactFlagStocks[4][i]&&!eesContactFlagStocks[5][i])
                                            swing_contact_indices.push_back(i); 
                                        break;
                                }
                            }
                            // std::cout << "swing_contact_indices:"<< swing_contact_indices.size() << std::endl;
                            if (swing_contact_indices.size()>=1)
                            {
                                fullySwingStartIndex = swing_contact_indices[0]-1;//取出完全悬空点起点
                                fullySwingFinalIndex = swing_contact_indices[swing_contact_indices.size()-1];
                            }
                        
                        };
                        findFullySwing(fullySwingStartIndex, fullySwingFinalIndex);
                        // std::cout << "swingStartIndex:"<< swingStartIndex << " swingFinalIndex:"<< swingFinalIndex << std::endl;
                        // std::cout << "fullySwingStartIndex:"<< fullySwingStartIndex << " fullySwingFinalIndex:"<< fullySwingFinalIndex << std::endl;
                        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);
                        // std::cout << "current time:"<< eventTimes[p] << " index p:"<< p << std::endl;

                        const scalar_t fullySwingStartTime = eventTimes[fullySwingStartIndex];
                        const scalar_t fullySwingFinalTime = eventTimes[fullySwingFinalIndex];
                        const scalar_t currentTime = eventTimes[p];
                        

                        // std::cout << "swingStartTime:"<< swingStartTime << " swingFinalTime:"<< swingFinalTime << std::endl;
                        // std::cout << "fullySwingStartTime:"<< fullySwingStartTime << " fullySwingFinalTime:"<< fullySwingFinalTime << std::endl;
                        const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);
                        
                        scalar_t midHeight = next_stance_position[j][2] + scaling * config_.swingHeight;
                        auto handleSwing = [&eesContactFlagStocks](int& is_heel_toe_contact, double& new_height, const Config& config, int p, int j) {
                                if (eesContactFlagStocks[j][p])// 
                                {
                                    is_heel_toe_contact = 0;
                                    std::cout << "not contact at p:"<< p << std::endl;
                                    return;
                                }
                                switch(j){  // 判断是否为heel或toe着地，并且当前为抬起的接触点
                                    case 0:
                                    case 1: if (eesContactFlagStocks[2][p]) { is_heel_toe_contact = 1; new_height = config.toeSwingHeight;} break;
                                    case 2:
                                    case 3: if (eesContactFlagStocks[0][p]) { is_heel_toe_contact = 2; new_height = config.heelSwingHeight;} break;
                                    case 4:
                                    case 5: if (eesContactFlagStocks[6][p]) { is_heel_toe_contact = 1; new_height = config.toeSwingHeight;} break;
                                    case 6:
                                    case 7: if (eesContactFlagStocks[4][p]) { is_heel_toe_contact = 2; new_height = config.heelSwingHeight;} break;
                                }
                            };

                        // 计算落足点
                        double feet_height = scaling * config_.swingHeight;
                        if (initTime < swingFinalTime && swingFinalIndex > last_final_idx[j])
                        {
                            last_stance_position[j] = next_stance_position[j];
                            // if (verbose)
                                // std::cout << "last_stance_position: [" << j << "] " << last_stance_position[j].transpose() << std::endl;

                            scalar_t next_middle_time = 0;// 下一个半周期的中间时刻
                            scalar_t nextStanceFinalTime = 0;
                            if (swingFinalIndex < modeSequence.size() - 1)
                            {
                                const int nextStanceFinalIndex = finalTimesIndices[j][swingFinalIndex + 1];
                                nextStanceFinalTime = eventTimes[nextStanceFinalIndex];
                                next_middle_time = (swingFinalTime + nextStanceFinalTime) / 2;
                            }
                            else
                            {
                                next_middle_time = swingFinalTime;
                            }

                            vector_t next_middle_body_pos = targetTrajectories.getDesiredState(next_middle_time).segment<6>(6);
                            vector_t current_body_pos = current_state_.segment<6>(6);
                            vector_t current_body_vel = current_state_.segment<3>(0);
                            // next_stance_position[j] = calNextFootPos(j, swingStartTime, swingFinalTime, currentBodyState, targetTrajectories);
                            // std::cout << "modeSchedule.with_pos_sequence: " << modeSchedule.with_pos_sequence << std::endl;
                            if(has_feet_trajectory_)
                            {

                                auto result = findFootPosNext(j, p, swingStartIndex, swingFinalIndex, modeSchedule, next_stance_position[j], eesContactFlagStocks);
                                next_stance_position[j] = result.first;
                                additional_stance_positions = result.second;
                                feet_height = scaling * config_.climbStageSwingHeight;
                                if (!visualize_stance_position_flag_[j])// 显示第一个落点
                                {
                                    calc_stance_position_[j] = next_stance_position[j];
                                    visualize_stance_position_flag_[j] = true;
                                }
                            }
                            else // 正常踏步规划
                            {
                                next_stance_position[j] = calNextFootPos(j, swingStartTime, swingFinalTime, next_middle_time, next_middle_body_pos,
                                                                         current_body_pos, current_body_vel, targetTrajectories, next_stance_position[j], verbose);
                                feet_height = scaling * config_.swingHeight;
                                
                            }
                            if (config_.enable_interrupt_with_est_mode && interrupt_swing_ && initTime > swingStartTime)// 终止规划
                            {
                                std::cout << "interrupt_swing_ at time:" << initTime << ", leg:" << j  << ", height:" << next_stance_position[j][2] << " -> " << currentFeetPosition[j][2] << std::endl;
                                next_stance_position[j] = currentFeetPosition[j];
                            }
                            // **************** 计算斜面上的高度差 ******************
                            if (!has_feet_trajectory_ && enable_slope_planning_ && body_vel_cmd_[0] > 0)
                            {
                                // std::cout << "next_stance_position: [" << j << "] " << next_stance_position[j].transpose() << std::endl;
                                // double new_height_offset = calculateHeightDifference((j < numFeet_ / 2) ? left_foot_position : right_foot_position,
                                //                                                  Eigen::Vector2d(last_stance_position[j][0], last_stance_position[j][1]),
                                //                                                  Eigen::Vector2d(next_stance_position[j][0], next_stance_position[j][1]));
                                current_height_offset = calculateHeightDifference((j < numFeet_ / 2) ? lf_normal_vector_ : rf_normal_vector_,
                                                                                 Eigen::Vector2d(last_stance_position[j][0], last_stance_position[j][1]),
                                                                                 Eigen::Vector2d(next_stance_position[j][0], next_stance_position[j][1]));
                                // double alpha = 0.1;
                                // current_height_offset =  alpha * current_height_offset + (1 - alpha) * new_height_offset;
                                // std::cout << "current_height_offset: " << current_height_offset << std::endl;
                                // if (j < numFeet_ / 2)
                                // {

                                //     auto diff_ln = diff_vel * (next_stance_position[j][0] - last_stance_position[j][0]);
                                //     std::cout << "dx:" << next_stance_position[j][0] - last_stance_position[j][0] << std::endl;
                                //     std::cout << "current_height_offset ddd: " << diff_ln << std::endl;
                                // }

                                if (std::abs(current_height_offset) > 0.04)// TODO:需要调整阈值适配斜面
                                {
                                    next_stance_position[j][2] += current_height_offset;
                                    // {
                                    //     for (int i = 0; i < 3; ++i)
                                    //     {
                                    //         std::cout << "foot_pos: " <<( (j < numFeet_ / 2) ? left_foot_position[i] : right_foot_position[i]).transpose() << std::endl;
                                    //     }

                                    // }
                                    std::cout << "using current_height_offset: " << j << " " << current_height_offset << std::endl;
                                }
                                else
                                {
                                    current_height_offset = 0.0;   
                                }
                               
                            }
                            last_final_idx[j] = swingFinalIndex;
                            if (verbose)
                            {
                                std::cout << "current_body_pos: " << current_body_pos.transpose() << std::endl;
                                std::cout << "start body:" << targetTrajectories.getDesiredState(swingStartTime).segment<3>(6).transpose() << std::endl;
                                std::cout << "final body:" << targetTrajectories.getDesiredState(swingFinalTime).segment<3>(6).transpose() << std::endl;
                                std::cout << "next_stance_position: [" << j << "] " << next_stance_position[j].transpose() << std::endl;
                            }
                        }
                       
                        
                        // 生成规划轨迹
                        std::vector<double> z_time_sequence = {swingStartTime};
                        std::vector<double> z_pos_sequence = {last_stance_position[j][2]};
                        std::vector<double> xy_time_sequence = {swingStartTime};

                        std::vector<double> x_pos_sequence = {last_stance_position[j][0]};
                        std::vector<double> y_pos_sequence = {last_stance_position[j][1]};
                        if (additional_stance_positions.size() > 0)// 存在完全定义的足端轨迹
                        {
                            double time_interval = (fullySwingFinalTime - fullySwingStartTime) / (additional_stance_positions.size() + 1);
                            for (int i = 0; i < additional_stance_positions.size(); i++)
                            {
                                double time = fullySwingStartTime + time_interval * (i+1);
                                z_time_sequence.push_back(time);
                                xy_time_sequence.push_back(time);
                                z_pos_sequence.push_back(additional_stance_positions[i][2]);
                                x_pos_sequence.push_back(additional_stance_positions[i][0]);
                                y_pos_sequence.push_back(additional_stance_positions[i][1]);
                            }
                        }
                        else if (has_feet_trajectory_)// 在单步模式下，但是没有完全定义的足端轨迹，自动规划
                        {
                            for (int i = swingStartIndex+1; i <= swingFinalIndex; i++)
                            {
                                scalar_t new_height = scaling * config_.swingHeight;
                                int is_heel_toe_contact = 0; // 标识是否为heel 1或toe 2着地
                                handleSwing(is_heel_toe_contact, new_height, config_, i, j);

                                // 判断同一只脚的所有接触点是否都已经离地
                                size_t start_idx = (j < 4) ? 0 : 4;  // 左脚: 0-3, 右脚: 4-7
                                size_t end_idx = (j < 4) ? 4 : 8;
                                bool is_next_mode_swing = std::none_of(eesContactFlagStocks.begin() + start_idx, 
                                                                      eesContactFlagStocks.begin() + end_idx,
                                                                      [i](const std::vector<bool>& contact_flags) {
                                                                          return contact_flags[i+1];
                                                                      });
                                bool is_last_mode_swing = std::none_of(eesContactFlagStocks.begin() + start_idx, 
                                                                      eesContactFlagStocks.begin() + end_idx,
                                                                      [i](const std::vector<bool>& contact_flags) {
                                                                          return contact_flags[i-1];
                                                                      });
                                if (swingFinalIndex - swingStartIndex<=1)// mode 只有一个腾空点
                                {
                                    double top_height = std::max(last_stance_position[j][2], next_stance_position[j][2]) + feet_height;
                                    double mid_time = (swingStartTime + swingFinalTime) / 2;
                                    z_time_sequence.push_back(mid_time);
                                    z_pos_sequence.push_back(top_height);
                                    // std::cout << "z pushbackc: " << top_height << " time: " << mid_time << std::endl;
                                }
                                else if (is_heel_toe_contact != 0)//非完全腾空相
                                {
                                    double heel_toe_height = new_height + (is_next_mode_swing?last_stance_position[j][2]:next_stance_position[j][2]);
                                    double switch_time = (is_last_mode_swing)?eventTimes[i-1]:eventTimes[i];

                                    z_time_sequence.push_back(switch_time);
                                    z_pos_sequence.push_back(heel_toe_height);
                                    // std::cout << "z pushbacka: " << heel_toe_height << " time: " << switch_time << std::endl;
                                }
                                else// 完全腾空相
                                {
                                    double top_height = std::max(last_stance_position[j][2], next_stance_position[j][2]) + feet_height;
                                    double mid_time = (fullySwingStartTime + fullySwingFinalTime) / 2;
                                    z_time_sequence.push_back(mid_time);
                                    z_pos_sequence.push_back(top_height);
                                    // std::cout << "z pushbackb: " << top_height << " time: " << mid_time << std::endl;
                                }
                            }
                        }
                        else// 正常踏步规划
                        {
                            const scalar_t fullySwingMiddleTime=(fullySwingStartTime + fullySwingFinalTime) / 2;
                            for (int i = swingStartIndex+1; i <= swingFinalIndex; i++)
                            {
                                scalar_t new_height = scaling * config_.swingHeight;
                                int is_heel_toe_contact = 0; // 标识是否为heel 1或toe 2着地
                                handleSwing(is_heel_toe_contact, new_height, config_, i, j);
                                // std::cout << "eventTimes[" << i << "]: " << eventTimes[i] << " is_heel_toe_contact:"<< is_heel_toe_contact << std::endl;
                                // std::cout << "swing mid time:" << (fullySwingStartTime + fullySwingFinalTime) / 2 << std::endl;
                                if ((swingFinalIndex-swingStartIndex > 1) && is_heel_toe_contact != 0)
                                {
                                    if (std::abs(current_height_offset) > 0.04)// 正在进行斜面规划
                                        continue;
                                    double switch_time = eventTimes[i];
                                    if (switch_time>=fullySwingMiddleTime)
                                    {
                                        switch_time = eventTimes[i - 1];
                                        z_time_sequence.push_back(switch_time);
                                        z_pos_sequence.push_back(new_height + next_stance_position[j][2]);
                                    }
                                    else// 脚后跟离地
                                    {
                                        z_time_sequence.push_back(switch_time);
                                        z_pos_sequence.push_back(new_height + next_stance_position[j][2]);
                                    }
                                }
                                else // 完全悬空相，保证双脚腾空接触点同时最高
                                {
                                    double top_height = std::max(last_stance_position[j][2], next_stance_position[j][2]) + feet_height;
                                    if(std::abs(z_time_sequence.back()-fullySwingMiddleTime)>1e-9)
                                    {
                                        z_time_sequence.push_back(fullySwingMiddleTime);
                                        z_pos_sequence.push_back(top_height);
                                    }
                                }
                            }
                        }
                        xy_time_sequence.push_back(swingFinalTime);
                        x_pos_sequence.push_back(next_stance_position[j][0]);
                        y_pos_sequence.push_back(next_stance_position[j][1]);
                        footXInterpolators_[j].emplace_back(DrakeInterpolator(xy_time_sequence, x_pos_sequence));
                        footYInterpolators_[j].emplace_back(DrakeInterpolator(xy_time_sequence, y_pos_sequence));
                        if (verbose)
                        {
                            std::cout << "lastTargetPos:" << last_stance_position[j].transpose() << std::endl;
                            std::cout << "NextTargetPos: " << next_stance_position[j].transpose() << std::endl;
                        }

                        z_time_sequence.push_back(swingFinalTime);
                        z_pos_sequence.push_back(next_stance_position[j][2]);
                        // std::cout << "z nextpose: " << next_stance_position[j][2] << " time: " << swingFinalTime << std::endl;
                        // std::cout << "zpos:"<<std::endl;
                        // for (int xx=0;xx < z_pos_sequence.size();xx++)
                        // {
                        //     std::cout << " time[" << z_time_sequence[xx] << "] zpos[" << z_pos_sequence[xx] << "]" << std::endl;
                        // }
                        // std::cout  <<std::endl;
                        //   std::cout << "Swing phase:" << std::endl;
                        // std::cout << "lastTargetPos:" << last_calc_stance_position_[j].transpose() << std::endl;
                        // std::cout << "NextTargetPos: " << calc_stance_position_[j].transpose() << std::endl;

                        footZInterpolators_[j].emplace_back(DrakeInterpolator(z_time_sequence, z_pos_sequence, scaling * config_.liftOffVelocity, scaling * config_.touchDownVelocity));
                    }
                    else
                    { // for a stance leg
                        // Note: setting the time here arbitrarily to 0.0 -> 1.0 makes the assert in CubicSpline fail
                        // if (verbose)
                            // std::cout << "stanceleg:"<< next_stance_position[j].transpose() << std::endl;
                        std::vector<double> time_sequence = {0, 1};
                        
                        footXInterpolators_[j].emplace_back(DrakeInterpolator(time_sequence, {next_stance_position[j][0], next_stance_position[j][0]}));
                        footYInterpolators_[j].emplace_back(DrakeInterpolator(time_sequence, {next_stance_position[j][1], next_stance_position[j][1]}));

                        // std::cout << "Stance phase:" << std::endl;
                        // std::cout << "lastTargetPos:" << last_calc_stance_position_[j].transpose() << std::endl;
                        // std::cout << "NextTargetPos: " << calc_stance_position_[j].transpose() << std::endl;
                        double height = next_stance_position[j][2];
                        // std::cout << "height: " << height << std::endl;
                        // std::cout << "stance leg pos: "<< next_stance_position[j].transpose() << std::endl;
                        footZInterpolators_[j].emplace_back(DrakeInterpolator(time_sequence, {height, height}));
                    }
                }
                if(calc_next_stance_position_success)
                    last_calc_stance_position_[j] = calc_stance_position_[j];
                feetHeightTrajectoriesEvents_[j] = eventTimes;
            }
            if (interrupt_swing_)
                interrupt_swing_ = false;


            
            if (num_arm_joints_ == 0)// 完全没有手臂关节
            {
                return;
            }
            // planArmTrajectory
            scalar_array_t armTimeTrajectory;
            vector_array_t armStateTrajectory;
            auto generateSwingState = [&](int state_index)
            {   
                double zero_angle = config_.swing_shoulder_center;
                double zero_angle_roll = config_.swing_shoulder_roll_center;
                int single_arm_joint = num_arm_joints_/2;
                vector_t max_half_state =  vector_t::Zero(single_arm_joint);
                vector_t min_half_state = vector_t::Zero(single_arm_joint);
                max_half_state[0] = zero_angle - config_.swing_shoulder_scale;
                min_half_state[0] = zero_angle + config_.swing_shoulder_scale;
                if (single_arm_joint > 3)// 肘关节存在
                {
                    double zero_angle_elbow = config_.swing_elbow_center;
                    max_half_state[3] = zero_angle_elbow - config_.swing_elbow_scale;
                    min_half_state[3] = zero_angle_elbow ;
                }
                auto mid_state = (max_half_state + min_half_state) / 2;

                vector_t state = vector_t::Zero(num_arm_joints_);
                switch (state_index)
                {
                case -1:
                    state << max_half_state, min_half_state;
                    break;
                case 0:
                    state << mid_state, mid_state;
                    break;
                case 1:
                    state << min_half_state, max_half_state;
                    break;
                default:
                    break;
                }
                if (single_arm_joint > 1)// 肩roll关节存在
                {
                    state[1] = zero_angle_roll;     // 添加肩roll关节摆动期望
                    state[single_arm_joint + 1] = -zero_angle_roll;
                }
                return state;
            } ;
            // std::cout <<"initTime:"<<initTime<<std::endl;
            // for (int p = 0; p < eventTimes.size(); ++p)
            // {
            //     std::cout << "[" << p << "]: " << eventTimes[p] << " mode:"<<modeNumber2String(modeSequence[p]) << std::endl;
            //     // modeNumber2String
            // }
            // std::cout << std::endl;

            int startIndex = -1;
            std::string lastMode = "";

            for (size_t i = 1; i < modeSequence.size(); ++i) {
                std::string currentMode = modeNumber2String(modeSequence[i]);
                
                // std::cout << "lastMode:" << lastMode << " currentMode:" << currentMode << std::endl;
                if (lastMode != currentMode && lastMode != "")// 不同模式，结束一组
                {
                    if (lastMode[0] == 'F' || lastMode[1] == 'F' || lastMode == "SS")
                    {// 计算中心时间并存储
                        scalar_t centerTime = computeCenterTime(eventTimes, startIndex-1, i-1);
                        armTimeTrajectory.push_back(centerTime);
                        if (lastMode == "SS" || lastMode == "FF") {
                            armStateTrajectory.push_back(generateSwingState(0));
                        } else
                        {
                            armStateTrajectory.push_back(generateSwingState((lastMode[0] == 'F')?1:-1));
                        }
                        // std::cout << "centerTime:" << centerTime << " lastMode:" << lastMode<< " state:" << armStateTrajectory.back().transpose() << std::endl;
                        startIndex = i;  // 重置
                    }
                }

                // 判断模式是否包含 "F" 或 "SS"
                if (currentMode[0] == 'F' || currentMode[1] == 'F' || currentMode == "SS") {
                    // 开始记录一组模式
                    if (currentMode != lastMode)
                    {
                        startIndex = i;
                    }else
                    {
                        if (i - startIndex > 3 && ( lastMode == "SS" || lastMode == "FF"))
                        {
                            scalar_t centerTime = computeCenterTime(eventTimes, startIndex, startIndex+1);
                            armTimeTrajectory.push_back(centerTime);
                            armStateTrajectory.push_back(generateSwingState(0));
                            startIndex = i;  // 重置
                        }
                    }
    
                }
        
                lastMode = currentMode;

            }

            // 处理最后一组（如果模式序列以 "F" 或 "SS" 结尾）
            // if (startIndex != -1 && (lastMode.find('F') != std::string::npos || lastMode == "SS")) {
            //     scalar_t centerTime = computeCenterTime(eventTimes, startIndex, eventTimes.size() - 1);
            //     armTimeTrajectory.push_back(centerTime);
            // }

   
            vector_array_t armInputTrajectory(armTimeTrajectory.size(), vector_t::Zero(num_arm_joints_));

            armTargetTrajectories_.setBuffer({armTimeTrajectory, armStateTrajectory, armInputTrajectory});
        }
        

        // ref: Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control
        vector3_t SwingTrajectoryPlanner::calNextFootPos(int feet, scalar_t current_time, scalar_t stop_time,
                                                         scalar_t next_middle_time, const vector_t &next_middle_body_pos,
                                                         const vector_t &current_body_pos, const vector3_t &current_body_vel, 
                                                         const TargetTrajectories& targetTrajectories, vector3_t &last_stance_position, bool verbose)
        {
            vector3_t next_stance_position;
            vector3_t angular = next_middle_body_pos.tail(3);
            vector3_t roted_bias = getRotationMatrixFromZyxEulerAngles(angular) * feetXYOffset_[feet];
            vector3_t current_angular = current_body_pos.tail(3);
            auto current_rot = getRotationMatrixFromZyxEulerAngles(current_angular);
            
            const auto& body_vel_cmd_ = body_vel_cmd_buf_.get();
            const vector3_t &vel_cmd_linear = current_rot * body_vel_cmd_.head(3);
            const vector3_t &vel_cmd_angular = current_rot * body_vel_cmd_.tail(3);

            vector3_t current_body_vel_tmp = current_body_vel;
            current_body_vel_tmp(2) = 0;
            const vector3_t &vel_linear = current_body_vel_tmp;

            const scalar_t k = 0.0;
            const scalar_t desiredHeight = targetTrajectories.getDesiredState(current_time)[8];
            vector3_t p_shoulder = (stop_time - current_time) * (/* 0.5 * vel_linear + 0.5 *  */vel_cmd_linear) + roted_bias;
            vector3_t p_symmetry = (next_middle_time - stop_time) * vel_linear + k * (vel_linear - vel_cmd_linear);
            vector3_t p_centrifugal = 0.5 * sqrt(desiredHeight / 9.81) * vel_linear.cross(vel_cmd_angular);
            if (verbose)
            {
                std::cout << "current_time:" << current_time << " stop_time:" << stop_time << " next_middle_time:" << next_middle_time << std::endl;
                std::cout << "roted_bias:" << roted_bias.transpose() << std::endl;
                std::cout << "vel_linear:" << vel_linear.transpose() << std::endl;
                std::cout << "vel_cmd_linear:" << vel_cmd_linear.transpose() << std::endl;

                std::cout << "p_shoulder: " << p_shoulder.transpose() << std::endl;
                std::cout << "p_symmetry: " << p_symmetry.transpose() << std::endl;
                std::cout << "p_centrifugal: " << p_centrifugal.transpose() << std::endl;
            }
            // TODO:确认kuavo需要 next_stance_position = targetTrajectories.getDesiredState(current_time).segment<3>(6) + p_shoulder + p_symmetry + p_centrifugal;

            next_stance_position = targetTrajectories.getDesiredState(current_time).segment<3>(6) + p_shoulder /* + p_symmetry  */+ p_centrifugal;
            next_stance_position.z() = last_stance_position.z();
            return std::move(next_stance_position);
        }

        // ref: Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control
        vector3_t SwingTrajectoryPlanner::calNextFootPos(int feet, const scalar_t& swingInitTime, const scalar_t& swingFinalTime, 
                                                         const vector_t& currentBodyState, const TargetTrajectories& targetTrajectories)
        {
          vector3_t next_stance_position;
          const scalar_t swingMidTime = (swingInitTime + swingFinalTime) / 2;
          const vector_t swingMidTargetState = targetTrajectories.getDesiredState(swingMidTime);
          const vector3_t zyx = swingMidTargetState.segment(9, 3);
          const vector3_t rotationFeetOffset = getRotationMatrixFromZyxEulerAngles(zyx) * feetXYOffset_[feet];
          const vector3_t& velLinear = currentBodyState.head(3);
          const vector3_t& velAngular = currentBodyState.segment(3, 3);

          const auto& velCmd = body_vel_cmd_buf_.get();
          auto velcmdWorld = velCmd;
          velcmdWorld.head(3) = getRotationMatrixFromZyxEulerAngles(zyx) * velCmd.head(3);
          
          const vector3_t& velLinearCmd = velcmdWorld.head(3);
          const vector3_t& velAngularCmd = velcmdWorld.tail(3);
          const scalar_t k = 0.00;

          const vector_t swingFinalTargetState = targetTrajectories.getDesiredState(swingFinalTime);

          vector3_t p_shoulder = currentBodyState.segment(6, 3) + 0.5 * (velLinear + velLinearCmd) * (swingFinalTime - swingInitTime) + rotationFeetOffset;
          vector3_t p_symmetry = 0.5 * (swingFinalTime - swingInitTime) * velLinearCmd + k * (velLinear - velLinearCmd);
          vector3_t p_centrifugal = 0.5 * sqrt(currentBodyState[8] / 9.81) * velLinearCmd.cross(velAngularCmd);
          std::cout << "p_shoulder: " << p_shoulder.transpose() << std::endl;
          std::cout << "p_symmetry: " << p_symmetry.transpose() << std::endl;
          std::cout << "p_centrifugal: " << p_centrifugal.transpose() << std::endl;
          next_stance_position = p_shoulder + p_symmetry + p_centrifugal;
          next_stance_position.z() = latestStanceposition_[feet][2];
          return std::move(next_stance_position);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        std::pair<bool, vector3_t> SwingTrajectoryPlanner::calNextFootPos(int feet, const ModeSchedule& modeSchedule, const scalar_t& initTime)
        {
            vector3_t next_stance_position = vector3_t::Zero();
            int index_target=0;
            const auto &eventTimes = modeSchedule.eventTimes;
            const auto &modeSequence = modeSchedule.modeSequence;
            const auto &footPoseSequence = modeSchedule.footPoseSequence;

            // find the index on which the current time is located
            const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), initTime) - eventTimes.begin();
            // size_t mode = modeSchedule.modeAtTime(initTime + 0.001);
            size_t current_mode = modeSequence[index];
            // std::cout << "mode[0]: " << modeNumber2String(current_mode) << std::endl;
            if(current_mode == ModeNumber::SS)
            {
                // std::cout << "mode[-1]: " << modeNumber2String(modeSequence[index-1]) << std::endl;
                // std::cout << "mode[-2]: " << modeNumber2String(modeSequence[index-2]) << std::endl;
                // std::cout << "mode[1]: " << modeNumber2String(modeSequence[index+1]) << std::endl;
                // std::cout << "mode[2]: " << modeNumber2String(modeSequence[index+2]) << std::endl;
                // std::cout << "stance_position[-1]: " << footPoseSequence[index-1].head(3).transpose() << std::endl;
                // std::cout << "stance_position[0]: " << footPoseSequence[index].head(3).transpose() << std::endl;
                // std::cout << "stance_position[+1]: " << footPoseSequence[index+1].head(3).transpose() << std::endl;
                // std::cout << "stance_position[+2]: " << footPoseSequence[index+2].head(3).transpose() << std::endl;
                // left foot
                if(modeSequence[index+1] == ModeNumber::FS && modeSequence[index+2] == ModeNumber::SS)
                    index_target = index+2;
                // right foot
                if(modeSequence[index+1] == ModeNumber::SF && modeSequence[index+2] == ModeNumber::SS)
                    index_target = index+2;
                if(modeSequence[index-1] != ModeNumber::SS && modeSequence[index-2] == ModeNumber::SS)
                {
                    index_target = index;
                }
            }
            if(current_mode == ModeNumber::FS || current_mode == ModeNumber::SF)
            {
                // std::cout << "mode[-1]: " << modeNumber2String(modeSequence[index-1]) << std::endl;
                // std::cout << "mode[1]: " << modeNumber2String(modeSequence[index+1]) << std::endl;
                // std::cout << "stance_position[-1]: " << footPoseSequence[index-1].head(3).transpose() << std::endl;
                // std::cout << "stance_position[0]: " << footPoseSequence[index].head(3).transpose() << std::endl;
                // std::cout << "stance_position[+1]: " << footPoseSequence[index+1].head(3).transpose() << std::endl;
                // std::cout << "stance_position[+2]: " << footPoseSequence[index+2].head(3).transpose() << std::endl;
                if(modeSequence[index-1] == ModeNumber::SS && modeSequence[index+1] == ModeNumber::SS)
                    index_target = index+1;
            }
            const bool point_is_left_foot = (feet < latestStanceposition_.size() / 2);
            const bool target_is_left_foot = (modeSequence[index_target-1] == ModeNumber::FS);
            // bias
            bool match = (point_is_left_foot == target_is_left_foot);
            if(match)// same side
            {
                const Eigen::Vector3d initialZyx = {footPoseSequence[index_target](3), 0, 0};
                Eigen::Matrix3d R = getRotationMatrixFromZyxEulerAngles(initialZyx);
                // std::cout << "yaw: " << initialZyx(0) << ", R: " << R << std::endl;
                const Eigen::Vector3d delta_xyz = R * feetXYOffsetLocalFrame_[feet];
                // std::cout << "delta_xyz: " << delta_xyz.transpose() << std::endl;
                // std::cout << "footPose target: " << footPoseSequence[index_target].head(3).transpose() << std::endl;
                next_stance_position = footPoseSequence[index_target].head(3) + delta_xyz;
                // std::cout << "next_stance_position: " << next_stance_position.transpose() << std::endl;
            }
            return {match, std::move(next_stance_position)};
        }

        // /******************************************************************************************************/
        // /******************************************************************************************************/
        // /******************************************************************************************************/
        std::pair<vector3_t, std::vector<vector3_t>> SwingTrajectoryPlanner::findFootPosNext(int feet, int current_index, int swingStartIndex, int swingFinalIndex, const ModeSchedule& modeSchedule, const vector3_t &last_stance_position, const feet_array_t<std::vector<bool>> &eesContactFlagStocks)
        {
            vector3_t next_stance_position = last_stance_position;
            std::vector<vector3_t> additional_stance_positions;

            int index_target=-1;
            const auto &eventTimes = modeSchedule.eventTimes;
            const auto &modeSequence = modeSchedule.modeSequence;
            const auto &footPoseSequence = modeSchedule.footPoseSequence;
            const auto &enableFootPoseSequence = modeSchedule.enableFootSequence;
            const auto &additionalFootPoseSequence = modeSchedule.additionalFootPoseSequence;
            // std::cout << "additionalFootPoseSequence.size(): " << additionalFootPoseSequence.size() << std::endl;
            std::vector<vector6_t> currentAdditionalFootPoses;
            currentAdditionalFootPoses.clear();
            const auto &swingHeightSequence = modeSchedule.swingHeightSequence;
            // std::cout << "\nfeet: " << feet << " current_index: " << current_index << " swingStartIndex: " << swingStartIndex << " swingFinalIndex: " << swingFinalIndex << std::endl;
            // for (int i = swingStartIndex; i < modeSequence.size(); i++)
            // {
            //     std::cout << "mode[" << i << "]: " << modeNumber2String(modeSequence[i]) <<  " time: " << eventTimes[i] << std::endl;
            //     std::cout << "stance_position[" << i << "]: " << footPoseSequence[i].head(3).transpose() << std::endl;
            //     std::cout << "enableFootPose[" << i << "]: " << enableFootPoseSequence[i] << std::endl;
            //     std::cout << "additionalFootPoseSequence[" << i << "]: " << additionalFootPoseSequence[i].size() << std::endl;
            // }
            
            for (int i = swingStartIndex+1; i <= swingFinalIndex+1; i++)// 找到这个腾空相的落点
            {
                if (enableFootPoseSequence[i])// 使能了足端规划位置
                {

                    // 查找另外一只脚是否腾空
                    size_t start_idx = (feet < 4) ? 4 : 0;  // 左脚: 0-3, 右脚: 4-7
                    size_t end_idx = (feet < 4) ? 8 : 4;
                    bool is_another_feet_swing = std::any_of(eesContactFlagStocks.begin() + start_idx, 
                                                        eesContactFlagStocks.begin() + end_idx,
                                                        [i](const std::vector<bool>& contact_flags) {
                                                            return !contact_flags[i];
                                                        });
  
                    if (!is_another_feet_swing)// ！另外一只脚腾空了
                    {
                        index_target = i;

                        if (additionalFootPoseSequence[i].size() > 0)
                        {
                            currentAdditionalFootPoses = additionalFootPoseSequence[i];
                        }
                    }
                    if  (is_another_feet_swing || eesContactFlagStocks[feet][i])// 已经着地，或者另外一只脚腾空了
                    {
                        
                        break;
                    }
                }
            }
     
            if (index_target == -1)
            {
                std::cerr << "[SwingTrajectoryPlanner] findFootPosNext:" << " feet: " << feet << " foot pose trajectory not found, keep last stance position" << std::endl;
                return {next_stance_position, additional_stance_positions};
            }
            // std::cout << "swingFinalIndex: " << swingFinalIndex << " index_target: " << index_target << std::endl;
            // std::cout << "footPoseSequence[index_target]: " << footPoseSequence[index_target].head(4).transpose() << std::endl;
            // std::cout << "additionalFootPoseSequence[index_target]: " << additionalFootPoseSequence[index_target].size() << std::endl;
            config_.climbStageSwingHeight = swingHeightSequence[index_target-1];
            // 
            {
                auto func_get_stance_position = [&](int feet, vector6_t footPose)
                {
                    vector3_t stance_position;
                    const Eigen::Vector3d initialZyx = {footPose(3), footPose(4), footPose(5)};
                    Eigen::Matrix3d R = getRotationMatrixFromZyxEulerAngles(initialZyx);
                    const Eigen::Vector3d delta_xyz = R * feetXYOffsetLocalFrame_[feet];
                    stance_position = footPose.head(3) + delta_xyz;

                    return stance_position;
                };
                next_stance_position = func_get_stance_position(feet, footPoseSequence[index_target]);
                // next_stance_position[2] += stancePositionBeforeStepControl_[feet].z();
                // std::cout << "next_stance_position: " << next_stance_position.transpose() << std::endl;
                for(int i=0; i<currentAdditionalFootPoses.size(); i++)
                {
                    additional_stance_positions.push_back(func_get_stance_position(feet, currentAdditionalFootPoses[i]));
                }
            }
            return {next_stance_position, additional_stance_positions};
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool> &contactFlagStock)
        {
            const size_t numPhases = contactFlagStock.size();

            std::vector<int> startTimeIndexStock(numPhases, 0);
            std::vector<int> finalTimeIndexStock(numPhases, 0);

            // find the startTime and finalTime indices for swing feet
            for (size_t i = 0; i < numPhases; i++)
            {
                // if (!contactFlagStock[i])
                // {
                    std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
                // }
            }
            return {startTimeIndexStock, finalTimeIndexStock};
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        feet_array_t<std::vector<bool>> SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t> &phaseIDsStock) const
        {
            const size_t numPhases = phaseIDsStock.size();

            feet_array_t<std::vector<bool>> contactFlagStock;
            std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

            for (size_t i = 0; i < numPhases; i++)
            {
                const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
                for (size_t j = 0; j < numFeet_; j++)
                {
                    contactFlagStock[j][i] = contactFlag[j];
                }
            }
            return contactFlagStock;
        }


        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock)
        {
            const size_t numPhases = contactFlagStock.size();

            // find the starting time
            int startTimesIndex = 0;
            for (int ip = index - 1; ip >= 0; ip--)
            {
                if (contactFlagStock[ip] != contactFlagStock[index])
                {
                    startTimesIndex = ip;
                    break;
                }
            }

            // find the final time
            int finalTimesIndex = numPhases - 2;
            for (size_t ip = index + 1; ip < numPhases; ip++)
            {
                if (contactFlagStock[ip] != contactFlagStock[index])
                {
                finalTimesIndex = ip - 1;
                break;
                }
            }

            return { startTimesIndex, finalTimesIndex };
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                              const std::vector<size_t> &phaseIDsStock)
        {
            const size_t numSubsystems = phaseIDsStock.size();
            if (startIndex < 0)
            {
                std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
                for (size_t i = 0; i < numSubsystems; i++)
                {
                    std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
                }
                std::cerr << std::endl;

                throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
            }
            if (finalIndex >= numSubsystems - 1)
            {
                std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
                for (size_t i = 0; i < numSubsystems; i++)
                {
                    std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
                }
                std::cerr << std::endl;

                throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
            }
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale)
        {
            return std::min(1.0, (finalTime - startTime) / swingTimeScale);
        }

        /******************************************************************************************************/
        /******************************************************************************************************/
        /******************************************************************************************************/
        SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName, const std::string &fieldName, bool verbose)
        {
            boost::property_tree::ptree pt;
            boost::property_tree::read_info(fileName, pt);

            if (verbose)
            {
                std::cerr << "\n #### Swing Trajectory Config:";
                std::cerr << "\n #### =============================================================================\n";
            }

            SwingTrajectoryPlanner::Config config;
            const std::string prefix = fieldName + ".";

            loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
            loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
            loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
            loadData::loadPtreeValue(pt, config.climbStageSwingHeight, prefix + "climbStageSwingHeight", verbose);
            loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);
            loadData::loadPtreeValue(pt, config.toeSwingHeight, prefix + "toeSwingHeight", verbose);
            loadData::loadPtreeValue(pt, config.heelSwingHeight, prefix + "heelSwingHeight", verbose);
            loadData::loadPtreeValue(pt, config.deadBandVelocity, prefix + "deadBandVelocity", verbose);
            loadData::loadPtreeValue(pt, config.heelToeMaxHeightVelocity, prefix + "heelToeMaxHeightVelocity", verbose);
            loadData::loadPtreeValue(pt, config.swing_elbow_center, prefix + "swing_elbow_center", verbose);
            loadData::loadPtreeValue(pt, config.swing_elbow_scale, prefix + "swing_elbow_scale", verbose);
            loadData::loadPtreeValue(pt, config.swing_shoulder_scale, prefix + "swing_shoulder_scale", verbose);
            loadData::loadPtreeValue(pt, config.swing_shoulder_center, prefix + "swing_shoulder_center", verbose);
            loadData::loadPtreeValue(pt, config.swing_shoulder_roll_center, prefix + "swing_shoulder_roll_center", verbose);
            loadData::loadPtreeValue(pt, config.enable_interrupt_with_est_mode, prefix + "enable_interrupt_with_est_mode", verbose);
            loadData::loadPtreeValue(pt, config.enable_slope_planner, prefix + "enable_slope_planner", verbose);
            loadData::loadPtreeValue(pt, config.enable_dynamic_q, prefix + "enable_dynamic_q", verbose);
            loadData::loadPtreeValue(pt, config.enable_dynamic_r, prefix + "enable_dynamic_r", verbose);
            loadData::loadPtreeValue(pt, config.slope_planning_threshold, prefix + "slope_planning_threshold", verbose);
            if (verbose)
            {
                std::cerr << " #### =============================================================================" << std::endl;
            }

            return config;
        }


    bool SwingTrajectoryPlanner::planSingleStep(scalar_t initTime, const vector_t& cmd_pose, Eigen::Vector3d xyYawTh)
    {
        final_yaw_single_step_time_ = initTime;
        // const double yaw_threashold = 1.0;// 1rad
        // const auto xy_yaw_th = Eigen::Vector3d(0.3, 0.15, 1.0);
        const double dis_norm_th = xyYawTh.head<2>().norm();
        Eigen::Vector2d cmd_pose_xy = cmd_pose.head<2>();
        // 校验xyYawTh(2)不得接近0，否则会产生数值问题
        if(std::abs(xyYawTh(2)) < 1e-6)
        {
            std::cout << "xyYawTh(2) is too small, set it to 1.0." << std::endl;
            xyYawTh(2) = 1.0;
        }
        // if(abs(cmd_pose(2)) > xyYawTh(2))
        {
            std::cout << "cmd_pose: " << cmd_pose.transpose() << std::endl;
            std::cout << "cmd_pose(3) > yaw_threashold, start single step control" << std::endl;
            int num_steps = static_cast<int>(std::abs(cmd_pose(2) / xyYawTh(2)));
            const int sign = (cmd_pose(2) > 0) ? 1 : -1;
            if(abs(cmd_pose(2) - sign * num_steps * xyYawTh(2)) > xyYawTh(2)/3.0) // 
                ++num_steps;
            std::cout << "num_steps: " << num_steps << std::endl;

            // cmd_pose_xy(0) = std::min(num_steps*xyYawTh(0), std::max(-num_steps*xyYawTh(0), cmd_pose(0)));
            // cmd_pose_xy(1) = std::min(num_steps*xyYawTh(1), std::max(-num_steps*xyYawTh(1), cmd_pose(1)));
            if(cmd_pose_xy.norm() > dis_norm_th)            
                cmd_pose_xy = dis_norm_th * cmd_pose_xy / cmd_pose_xy.norm();
            // const auto& torso_pose = targetTrajectories.getDesiredState(initTime).segment<4>(6);
            std::vector<vector6_t> body_poses;
            for(int i = 0; i < num_steps; i++)
            {
                double yaw_step = cmd_pose(2) / num_steps * (i+1);
                auto xy_step = cmd_pose_xy / num_steps * (i+1);
                vector6_t pose_step;
                pose_step << xy_step, 0, yaw_step * 180.0 / M_PI, 0, 0, 0;  // x, y, z, yaw, pitch, roll
                body_poses.push_back(pose_step);
                std::cout << "pose_step[" << i << "]: " << pose_step.transpose() << std::endl;
            }
            const double foot_bias = 0.15;
            bool success = getMultipleStepsTrajectory(footPoseSchedule_, body_poses, 0.4, foot_bias, true);
            return success;
        }
        return false;
    }

    } // namespace humanoid
} // namespace ocs2
