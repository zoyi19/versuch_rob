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

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // Must be included first

#include "humanoid_interface/constraint/CenterOfMassConstraint.h"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "humanoid_interface/HumanoidPreComputation.h"

namespace ocs2
{
namespace humanoid
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CenterOfMassConstraint::CenterOfMassConstraint(const PinocchioInterface& pinocchioInterface,
                                               const CentroidalModelInfo& info,
                                               const SwitchedModelReferenceManager& referenceManager,
                                               const std::vector<std::string>& contactFrameNames)
  : StateConstraint(ConstraintOrder::Linear)
  , pinocchioInterfacePtr_(&pinocchioInterface)
  , info_(info)
  , referenceManagerPtr_(&referenceManager)
  , contactFrameNames_(contactFrameNames)
  , rosInitialized_(false)
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CenterOfMassConstraint::CenterOfMassConstraint(const CenterOfMassConstraint& rhs)
  : StateConstraint(rhs)
  , pinocchioInterfacePtr_(rhs.pinocchioInterfacePtr_)
  , info_(rhs.info_)
  , referenceManagerPtr_(rhs.referenceManagerPtr_)
  , contactFrameNames_(rhs.contactFrameNames_)
  , rosInitialized_(false)
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CenterOfMassConstraint::isActive(scalar_t time) const
{
  // 1. 获取当前步态类型
  std::string currentGait = "unknown";
  try {
    currentGait = referenceManagerPtr_->getGaitSchedule()->getGaitName(time);
  } catch (const std::exception& e) {
    // 如果获取失败，默认为stance模式（保守策略）
    currentGait = "stance";
  }
  
  // 2. 步态类型判断：只在stance步态且腰部控制启用时激活约束
  const bool isWaistControlEnabled = referenceManagerPtr_->isVRWaistControlEnabled();
  // std::cout << "✅[CenterOfMassConstraint] Current gait: " << currentGait 
  //           << ", VR waist control enabled: " << (isWaistControlEnabled ? "Yes" : "No") << std::endl;
  bool isStanceGait = (currentGait == "stance" && isWaistControlEnabled);  // 仅当步态为stance且腰部控制启用时认为是stance步态
  
  // 如果不是stance步态或腰部控制未启用，直接返回false
  if (!isStanceGait) {
    return false;
  }
  
  // 3. 获取接触状态
  auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  
  // 4. 检查所有足部接触点是否都在接触状态
  // 8个接触点对应的框架名称：
  // 0: ll_foot_toe (左脚掌左前), 1: lr_foot_toe (左脚掌右前), 2: ll_foot_heel (左脚掌左后), 3: lr_foot_heel (左脚掌右后)
  // 4: rl_foot_toe (右脚掌左前), 5: rr_foot_toe (右脚掌右前), 6: rl_foot_heel (右脚掌左后), 7: rr_foot_heel (右脚掌右后)
  bool allLeftFootContactsActive = true;
  bool allRightFootContactsActive = true;
  
  // Check left foot contacts (indices 0-3)
  for (size_t i = 0; i < 4 && i < contactFlags.size(); ++i) {
    if (!contactFlags[i]) {
      allLeftFootContactsActive = false;
      break;
    }
  }
  
  // Check right foot contacts (indices 4-7) 
  for (size_t i = 4; i < 8 && i < contactFlags.size(); ++i) {
    if (!contactFlags[i]) {
      allRightFootContactsActive = false;
      break;
    }
  }
  
  // 5. 综合判断：只有在stance步态且双足完全接触时才激活约束
  bool contactCondition = allLeftFootContactsActive && allRightFootContactsActive;
  
  // 调试输出（可选）
  // std::cout << "[CenterOfMassConstraint] time: " << time << ", gait: " << currentGait 
  //           << ", isStance: " << isStanceGait << ", contactOK: " << contactCondition 
  //           << ", active: " << (isStanceGait && contactCondition) << std::endl;
  
  return isStanceGait && contactCondition;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CenterOfMassConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const
{
  // Use the pre-computed PinocchioInterface from PreComputation
  const auto& preCompHumanoid = cast<HumanoidPreComputation>(preComp);
  const auto& model = preCompHumanoid.getPinocchioInterface().getModel();
  const auto& data = preCompHumanoid.getPinocchioInterface().getData();

  // Get center of mass position
  const vector3_t comPosition = data.com[0];
  
  // Use precomputed support polygon data
  const auto& comConstraintData = preCompHumanoid.getCenterOfMassConstraintData();
  
  // Check if precomputed data is valid
  if (!comConstraintData.isValid) {
    // Fallback: return zero constraint if no valid precomputed data
    vector_t constraint(4);
    constraint.setZero();
    return constraint;
  }
  
  // Get precomputed boundaries
  const scalar_t minX = comConstraintData.minX;
  const scalar_t maxX = comConstraintData.maxX;
  const scalar_t minY = comConstraintData.minY;
  const scalar_t maxY = comConstraintData.maxY;
  
  // Define constraints: COM should stay within support polygon
  // For relaxed barrier penalty, we need h(x) >= 0 formulation
  // So we compute the distance from COM to each boundary
  vector_t constraint(4);  // 4 constraints: minX, maxX, minY, maxY
  
  // X constraints: distance from boundaries (should be >= 0)
  constraint[0] = comPosition[0] - minX;  // Distance from left boundary
  constraint[1] = maxX - comPosition[0];  // Distance from right boundary
  
  // Y constraints: distance from boundaries (should be >= 0)
  constraint[2] = comPosition[1] - minY;  // Distance from front boundary
  constraint[3] = maxY - comPosition[1];  // Distance from back boundary
  
  // Optional: Publish visualization data (using precomputed support points)
  // publishSupportPolygonVisualization(comConstraintData.supportPoints, comPosition);
  // publishSupportPolygonBounds(minX, maxX, minY, maxY, comPosition);
  // publishConstraintValue(constraint);

  return constraint;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation CenterOfMassConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComp) const
{
  // Use the pre-computed PinocchioInterface from PreComputation
  const auto& preCompHumanoid = cast<HumanoidPreComputation>(preComp);
  const auto& model = preCompHumanoid.getPinocchioInterface().getModel();
  const auto& data = preCompHumanoid.getPinocchioInterface().getData();
  
  // Get generalized coordinates from state
  const vector_t q = centroidal_model::getGeneralizedCoordinates(state, info_);
  
  // Compute COM Jacobian (following WbcBase.cpp pattern)
  // First compute center of mass to ensure data is up to date
  auto& mutableData = const_cast<pinocchio::Data&>(data); 
  pinocchio::centerOfMass(model, mutableData, q);
  // Now compute COM Jacobian
  const matrix_t comJacobian = pinocchio::jacobianCenterOfMass(model, mutableData, q);
  
  // Get constraint value using precomputed data
  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = getValue(time, state, preComp);
  
  // Initialize jacobians
  linearApproximation.dfdx = matrix_t::Zero(4, state.size());
  
  // Use precomputed support polygon data
  const auto& comConstraintData = preCompHumanoid.getCenterOfMassConstraintData();
  
  // Check if precomputed data is valid
  if (!comConstraintData.isValid) {
    // Return zero jacobian if no valid precomputed data
    return linearApproximation;
  }
  
  // Map COM Jacobian to state jacobian
  // The generalized coordinates start at index 6 in the state vector (after base pose)
  const size_t qOffset = 6;  // Offset for generalized coordinates in state
  const size_t qSize = q.size();
  
  if (qOffset + qSize <= state.size()) {
    // For constraint[0] = comPosition[0] - minX
    linearApproximation.dfdx.block(0, qOffset, 1, qSize) = comJacobian.row(0);
    
    // For constraint[1] = maxX - comPosition[0]
    linearApproximation.dfdx.block(1, qOffset, 1, qSize) = -comJacobian.row(0);
    
    // For constraint[2] = comPosition[1] - minY
    linearApproximation.dfdx.block(2, qOffset, 1, qSize) = comJacobian.row(1);
    
    // For constraint[3] = maxY - comPosition[1]
    linearApproximation.dfdx.block(3, qOffset, 1, qSize) = -comJacobian.row(1);
  }
  
  return linearApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CenterOfMassConstraint::publishSupportPolygonVisualization(const std::vector<vector3_t>& supportPoints, 
                                                                const vector3_t& comPosition) const
{
  // 初始化 ROS 发布器（只在第一次调用时）
  if (!rosInitialized_) {
    ros::NodeHandle nh;
    supportPolygonPub_ = nh.advertise<visualization_msgs::MarkerArray>("/humanoid_controller/support_polygon", 1);
    comPositionPub_ = nh.advertise<visualization_msgs::MarkerArray>("/humanoid_controller/com_position", 1);
    boundaryPub_ = nh.advertise<visualization_msgs::MarkerArray>("/humanoid_controller/support_boundary", 1);
    constraintValuePub_ = nh.advertise<std_msgs::Float64>("/humanoid_controller/com_constraint_value", 1);
    rosInitialized_ = true;
  }
  
  // 创建支撑点标记
  visualization_msgs::MarkerArray supportMarkers;
  
  // 支撑点标记
  for (size_t i = 0; i < supportPoints.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "support_points";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = supportPoints[i][0];
    marker.pose.position.y = supportPoints[i][1];
    marker.pose.position.z = supportPoints[i][2];
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.05;  // 5cm 直径
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    marker.color.r = 0.0;
    marker.color.g = 1.0;  // 绿色
    marker.color.b = 0.0;
    marker.color.a = 0.8;
    
    supportMarkers.markers.push_back(marker);
  }
  
  // 支撑多边形边界线
  if (supportPoints.size() >= 4) {
    visualization_msgs::Marker polygonMarker;
    polygonMarker.header.frame_id = "odom";
    polygonMarker.header.stamp = ros::Time::now();
    polygonMarker.ns = "support_polygon";
    polygonMarker.id = 100;
    polygonMarker.type = visualization_msgs::Marker::LINE_STRIP;
    polygonMarker.action = visualization_msgs::Marker::ADD;
    
    polygonMarker.scale.x = 0.02;  // 线宽
    polygonMarker.color.r = 0.0;
    polygonMarker.color.g = 0.8;
    polygonMarker.color.b = 1.0;  // 青色
    polygonMarker.color.a = 0.8;
    
    // 添加多边形顶点（按顺序连接）
    for (const auto& point : supportPoints) {
      geometry_msgs::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      polygonMarker.points.push_back(p);
    }
    // 闭合多边形
    if (!supportPoints.empty()) {
      geometry_msgs::Point p;
      p.x = supportPoints[0][0];
      p.y = supportPoints[0][1];
      p.z = supportPoints[0][2];
      polygonMarker.points.push_back(p);
    }
    
    supportMarkers.markers.push_back(polygonMarker);
  }
  
  // 发布支撑多边形
  supportPolygonPub_.publish(supportMarkers);
  
  // 创建质心位置标记
  visualization_msgs::MarkerArray comMarkers;
  visualization_msgs::Marker comMarker;
  comMarker.header.frame_id = "odom";
  comMarker.header.stamp = ros::Time::now();
  comMarker.ns = "center_of_mass";
  comMarker.id = 0;
  comMarker.type = visualization_msgs::Marker::SPHERE;
  comMarker.action = visualization_msgs::Marker::ADD;
  
  comMarker.pose.position.x = comPosition[0];
  comMarker.pose.position.y = comPosition[1];
  comMarker.pose.position.z = comPosition[2];
  comMarker.pose.orientation.w = 1.0;
  
  comMarker.scale.x = 0.08;  // 8cm 直径，比支撑点大
  comMarker.scale.y = 0.08;
  comMarker.scale.z = 0.08;
  
  comMarker.color.r = 1.0;  // 红色
  comMarker.color.g = 0.0;
  comMarker.color.b = 0.0;
  comMarker.color.a = 0.9;
  
  comMarkers.markers.push_back(comMarker);
  
  // 发布质心位置
  comPositionPub_.publish(comMarkers);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CenterOfMassConstraint::publishSupportPolygonBounds(scalar_t minX, scalar_t maxX, scalar_t minY, scalar_t maxY, 
                                                         const vector3_t& comPosition) const
{
  // 初始化 ROS 发布器（如果还没有初始化）
  if (!rosInitialized_) {
    ros::NodeHandle nh;
    boundaryPub_ = nh.advertise<visualization_msgs::MarkerArray>("/humanoid_controller/support_boundary", 1);
    const_cast<bool&>(rosInitialized_) = true;
  }
  
  visualization_msgs::MarkerArray boundaryMarkers;
  
  // 创建边界矩形的4个角点
  std::vector<vector3_t> boundaryCorners = {
    vector3_t(minX, minY, comPosition[2]),  // 左前角
    vector3_t(maxX, minY, comPosition[2]),  // 右前角
    vector3_t(maxX, maxY, comPosition[2]),  // 右后角
    vector3_t(minX, maxY, comPosition[2])   // 左后角
  };
  
  // 创建边界矩形线框
  visualization_msgs::Marker boundaryMarker;
  boundaryMarker.header.frame_id = "odom";
  boundaryMarker.header.stamp = ros::Time::now();
  boundaryMarker.ns = "support_boundary";
  boundaryMarker.id = 0;
  boundaryMarker.type = visualization_msgs::Marker::LINE_STRIP;
  boundaryMarker.action = visualization_msgs::Marker::ADD;
  
  boundaryMarker.scale.x = 0.01;  // 线宽
  boundaryMarker.color.r = 1.0;
  boundaryMarker.color.g = 0.5;
  boundaryMarker.color.b = 0.0;  // 橙色
  boundaryMarker.color.a = 1.0;
  
  // 添加矩形边界点（按顺序连接）
  for (const auto& corner : boundaryCorners) {
    geometry_msgs::Point p;
    p.x = corner[0];
    p.y = corner[1];
    p.z = corner[2];
    boundaryMarker.points.push_back(p);
  }
  // 闭合矩形
  geometry_msgs::Point p;
  p.x = boundaryCorners[0][0];
  p.y = boundaryCorners[0][1];
  p.z = boundaryCorners[0][2];
  boundaryMarker.points.push_back(p);
  
  boundaryMarkers.markers.push_back(boundaryMarker);
  
  // 发布边界标记
  boundaryPub_.publish(boundaryMarkers);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CenterOfMassConstraint::publishConstraintValue(const vector_t& constraint) const
{
  // 初始化 ROS 发布器（如果还没有初始化）
  if (!rosInitialized_) {
    ros::NodeHandle nh;
    constraintValuePub_ = nh.advertise<std_msgs::Float64>("/humanoid_controller/com_constraint_value", 1);
    const_cast<bool&>(rosInitialized_) = true;
  }
  
  // 计算约束值总和
  // constraint[0]: 左边界距离, constraint[1]: 右边界距离
  // constraint[2]: 前边界距离, constraint[3]: 后边界距离
  scalar_t totalConstraintValue = 0.0;
  for (int i = 0; i < constraint.size(); ++i) {
    totalConstraintValue += constraint[i];
  }
  
  // 发布约束值总和
  std_msgs::Float64 constraintMsg;
  constraintMsg.data = totalConstraintValue;
  constraintValuePub_.publish(constraintMsg);
  
  // 同时打印约束值信息（用于调试）
  std::cout << "COM Constraint Values: [" 
            << constraint[0] << ", " << constraint[1] << ", " 
            << constraint[2] << ", " << constraint[3] << "] "
            << "Sum: " << totalConstraintValue << std::endl;
}

}  // namespace humanoid
}  // namespace ocs2
