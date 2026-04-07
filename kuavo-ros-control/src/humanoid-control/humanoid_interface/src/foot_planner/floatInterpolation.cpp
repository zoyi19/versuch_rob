
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include "humanoid_interface/foot_planner/InverseKinematics.h"
#include "humanoid_interface/foot_planner/floatInterpolation.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2
{
namespace humanoid
{

std::pair<vector3_t, matrix3_t> FloatInterpolation::getSingleFootPose(const vector_t& init_q, int legIndex)
{
    const auto& model = pinocchio_interface_.getModel();
    auto& data = pinocchio_interface_.getData();
    vector_t q(model.nq);
    q.setZero();
    q = init_q;

    pinocchio::framesForwardKinematics(model, data, q);
    vector3_t footPosition = data.oMf[footFrameIds_[legIndex]].translation();
    matrix3_t footRotation = data.oMf[footFrameIds_[legIndex]].rotation();

    return std::make_pair(footPosition, footRotation);
}

std::pair<vector3_t, matrix3_t> FloatInterpolation::getBasePose(const vector_t& init_q)
{
    const auto& model = pinocchio_interface_.getModel();
    auto& data = pinocchio_interface_.getData();
    vector_t q(model.nq);
    q.setZero();
    q = init_q;

    pinocchio::framesForwardKinematics(model, data, q);
    auto frameId = model.getFrameId(baseFrameNames_);
    vector3_t footPosition = data.oMf[frameId].translation();
    matrix3_t footRotation = data.oMf[frameId].rotation();

    return std::make_pair(footPosition, footRotation);
}

// 假设足端位置不变(从 init_q 更新)，调整躯干位置获取逆解
vector_t FloatInterpolation::getlegJointAngles(const vector_t& observationState, const vector6_t& desiredBasePose, 
                                               const vector_t& desired_leg_q)
{
    vector_t init_q = vector_t::Zero(pinocchio_interface_.getModel().nq);
    init_q.head<6>() = observationState.segment(6, 6);          // 躯干位姿
    init_q.segment(6, 12) = observationState.segment(12, 12); // 双腿关节角度

    // 1. 获取当前状态
    const auto leftFootPoseWorld = getSingleFootPose(init_q, 0);    // 左脚世界坐标
    const auto rightFootPoseWorld = getSingleFootPose(init_q, 1);   // 右脚世界坐标
    const auto currentBasePoseWorld = getBasePose(init_q);          // 当前躯干位姿

    const vector3_t& p_base_cur = currentBasePoseWorld.first;      // 当前躯干位置
    const matrix3_t& R_base_cur = currentBasePoseWorld.second;     // 当前躯干旋转矩阵

    // 2. 期望的躯干位姿
    const vector3_t p_base_des = desiredBasePose.head<3>();  // 期望位置
    const matrix3_t R_base_des = getRotationMatrixFromZyxEulerAngles(vector3_t(desiredBasePose(5), 
                                                                               desiredBasePose(4), 
                                                                               desiredBasePose(3)));

    // 3. 计算相对旋转矩阵变化
    const matrix3_t R_des_to_base_cur = R_base_cur * R_base_des.transpose();

    // 左脚的期望位姿（相对于期望躯干）
    std::pair<vector3_t, matrix3_t> leftFootPoseDesired;
    leftFootPoseDesired.first = p_base_cur + R_des_to_base_cur * (leftFootPoseWorld.first - p_base_des);
    leftFootPoseDesired.second = R_des_to_base_cur * leftFootPoseWorld.second;

    // 右脚的期望位姿（相对于期望躯干）
    std::pair<vector3_t, matrix3_t> rightFootPoseDesired;
    rightFootPoseDesired.first = p_base_cur + R_des_to_base_cur * (rightFootPoseWorld.first - p_base_des);
    rightFootPoseDesired.second = R_des_to_base_cur * rightFootPoseWorld.second;

    // 修改 init_q 的下肢关节角度为 desired_leg_q，以便 IK 计算不会陷入奇异
    init_q.segment(6, 12) = desired_leg_q;

    vector6_t leftLegJoint = ikSolver_.computeIkWithDesiredFrame(init_q, 0, leftFootPoseDesired.first, leftFootPoseDesired.second, footFrameIds_[0]);
    vector6_t rightLegJoint = ikSolver_.computeIkWithDesiredFrame(init_q, 1, rightFootPoseDesired.first, rightFootPoseDesired.second, footFrameIds_[1]);

    vector_t legJointAngles = vector_t::Zero(12);
    legJointAngles.segment<6>(0) = leftLegJoint;
    legJointAngles.segment<6>(6) = rightLegJoint;

    return legJointAngles;
}

}  // namespace humanoid
}  // namespace ocs2
