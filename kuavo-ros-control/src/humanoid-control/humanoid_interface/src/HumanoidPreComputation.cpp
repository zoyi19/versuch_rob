/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Numerics.h>

#include <humanoid_interface/HumanoidPreComputation.h>
#include <limits>
#include <algorithm>

namespace ocs2 {
namespace humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation::HumanoidPreComputation(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                                     const SwingTrajectoryPlanner& swingTrajectoryPlanner, ModelSettings settings)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      info_(std::move(info)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      mappingPtr_(new CentroidalModelPinocchioMapping(info_)),
      settings_(std::move(settings)),
      feetKinematicsPtr_(new PinocchioEndEffectorKinematics(pinocchioInterface_, *mappingPtr_, settings_.contactNames3DoF)) {
  eeNormalVelConConfigs_.resize(info_.numThreeDofContacts);
  eeZeroVelConConfigs_.resize(info_.numThreeDofContacts);
  armSwingPosConConfigs_.resize(settings_.info.eeFrame.size());
  feetKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
  armJointPos_.setZero(2);
  armJointVel_.setZero(2);
  eeXYRefConConfigs_.resize(info_.numThreeDofContacts);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation* HumanoidPreComputation::clone() const {
  return new HumanoidPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation::HumanoidPreComputation(const HumanoidPreComputation& rhs)
        : pinocchioInterface_(rhs.pinocchioInterface_),
          info_(rhs.info_),
          swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
          mappingPtr_(rhs.mappingPtr_->clone()),
          settings_(rhs.settings_),
          feetKinematicsPtr_(rhs.feetKinematicsPtr_->clone()) {
    eeNormalVelConConfigs_.resize(rhs.eeNormalVelConConfigs_.size());
    eeZeroVelConConfigs_.resize(rhs.eeZeroVelConConfigs_.size());
    eeXYRefConConfigs_.resize(rhs.eeXYRefConConfigs_.size());
    armSwingPosConConfigs_.resize(rhs.armSwingPosConConfigs_.size());
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);
    feetKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);
    armJointPos_.setZero(2);
    armJointVel_.setZero(2);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void HumanoidPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  vector_t q = mappingPtr_->getPinocchioJointPosition(x);
  if(request.contains(Request::Approximation)) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    pinocchio::computeJointJacobians(model, data);

    updateCentroidalDynamics(pinocchioInterface_, info_, q);
    vector_t v = mappingPtr_->getPinocchioJointVelocity(x, u);
    updateCentroidalDynamicsDerivatives(pinocchioInterface_, info_, q, v);
  } else {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
  }

  // for(int i=0; i<2; i++){
  //   armJointPos_[i] = swingTrajectoryPlannerPtr_->getArmJointPositionConstraint(i, t);
  //   armJointVel_[i] = x(0) * swingTrajectoryPlannerPtr_->getArmJointVelocityConstraint(i, t);
  // }
  // lambda to set config for normal velocity constraints

  scalar_t posGainZ = settings_.positionErrorGain;
  if (swingTrajectoryPlannerPtr_->hasFeetTrajectory())
  {
    posGainZ = settings_.positionErrorGainZStepControl;
  }
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    if (!numerics::almost_eq(posGainZ, 0.0)) {
      config.b(0) -= posGainZ * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      // std::cout << "swingTrajectoryPlannerPtr_->getZpositionConstraint:"<< swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t) << std::endl;
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, posGainZ).finished();
    }
    return config;
  };

  // std::cout << "xyTraj_:  " << swingTrajectoryPlannerPtr_->getXvelocityConstraint(0, t) << "  " << swingTrajectoryPlannerPtr_->getYvelocityConstraint(0, t) << std::endl;
  // std::cout << "xyTraj_:  " << swingTrajectoryPlannerPtr_->getXpositionConstraint(0, t) << "  " << swingTrajectoryPlannerPtr_->getYpositionConstraint(0, t) << std::endl;
  scalar_t velGainXY = settings_.velocityErrorGain_xy;
  scalar_t posGainXY = settings_.positionErrorGain_xy;
  if (swingTrajectoryPlannerPtr_->hasFeetTrajectory())
  {
    velGainXY = settings_.velocityErrorGainXYStepControl;
    posGainXY = settings_.positionErrorGainXYStepControl;
  }
  // lambda to set config for xy velocity constraints
  auto eeXYRefConConfig = [&](size_t footIndex) {
    EndEffectorLinearConstraint::Config config;
    config.b = (vector_t(2) << - velGainXY * swingTrajectoryPlannerPtr_->getXvelocityConstraint(footIndex, t), 
                               - velGainXY * swingTrajectoryPlannerPtr_->getYvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(2, 3) << velGainXY, 0.0, 0.0, 0.0, velGainXY, 0.0).finished();
    // if (!numerics::almost_eq(settings_.positionErrorGain, 0.0)) {
    config.b(0) -= posGainXY * swingTrajectoryPlannerPtr_->getXpositionConstraint(footIndex, t);
    config.b(1) -= posGainXY * swingTrajectoryPlannerPtr_->getYpositionConstraint(footIndex, t);
    config.Ax = (matrix_t(2, 3) << posGainXY, 0.0, 0.0, 0.0, posGainXY, 0.0 /* settings_.positionErrorGain_xy */).finished();
    // }
    return config;
  };

  // lambda to set config for zero velocity constraints
  auto eeZeroVelConConfig = [&](size_t footIndex)
  {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);// 速度项，xyz
    config.Ax.setZero(3, 3); // Ax, 位置项，xyz
    if (!numerics::almost_eq(settings_.positionErrorGain_zero, 0.0))
    {
      config.b(2) -= settings_.positionErrorGain_zero * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t); // 只包含z的高度偏差
      config.Ax(2, 2) = settings_.positionErrorGain_zero;// 只考虑z的高度偏差项
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      eeXYRefConConfigs_[i] = eeXYRefConConfig(i);
      eeZeroVelConConfigs_[i] = eeZeroVelConConfig(i);
    }
  }

  armWrench_ = swingTrajectoryPlannerPtr_->getArmEeWrenchConstraint();
  
  // Precompute center of mass constraint data
  if (request.contains(Request::Constraint)) {
    computeCenterOfMassConstraintData();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void HumanoidPreComputation::computeCenterOfMassConstraintData() {
  const auto& model = pinocchioInterface_.getModel();
  const auto& data = pinocchioInterface_.getData();
  
  // Reset data
  comConstraintData_.isValid = false;
  comConstraintData_.supportPoints.clear();
  
  // Define support polygon frame names (consistent with CenterOfMassConstraint)
  const std::vector<std::string> supportPolygonFrameNames = {
    "ll_foot_toe",   // 左脚掌左前
    "ll_foot_heel",  // 左脚掌左后  
    "rr_foot_toe",   // 右脚掌右前
    "rr_foot_heel"   // 右脚掌右后
  };
  
  // Get support points positions
  for (const std::string& frameName : supportPolygonFrameNames) {
    try {
      const auto frameId = model.getFrameId(frameName);
      const vector3_t contactPosition = data.oMf[frameId].translation();
      comConstraintData_.supportPoints.push_back(contactPosition);
    } catch (const std::exception& e) {
      // If frame name doesn't exist, skip this point
      continue;
    }
  }
  
  // Calculate support polygon boundaries if we have enough points
  if (comConstraintData_.supportPoints.size() >= 4) {
    scalar_t minX = std::numeric_limits<scalar_t>::max();
    scalar_t maxX = std::numeric_limits<scalar_t>::lowest();
    scalar_t minY = std::numeric_limits<scalar_t>::max();
    scalar_t maxY = std::numeric_limits<scalar_t>::lowest();
    
    for (const auto& pos : comConstraintData_.supportPoints) {
      minX = std::min(minX, pos[0]);
      maxX = std::max(maxX, pos[0]);
      minY = std::min(minY, pos[1]);
      maxY = std::max(maxY, pos[1]);
    }
    
    // Add safety margin (consistent with CenterOfMassConstraint)
    const scalar_t safetyMargin = 0.06;
    comConstraintData_.minX = minX + safetyMargin;
    comConstraintData_.maxX = maxX - safetyMargin;
    comConstraintData_.minY = minY + safetyMargin;
    comConstraintData_.maxY = maxY - safetyMargin;
    
    comConstraintData_.isValid = true;
  }
}

}  // namespace humanoid
}  // namespace ocs2
