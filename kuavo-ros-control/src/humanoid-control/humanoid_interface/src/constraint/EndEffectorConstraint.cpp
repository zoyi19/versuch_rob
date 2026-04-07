#include "humanoid_interface/constraint/EndEffectorConstraint.h"
#include "humanoid_interface/HumanoidPreComputation.h"

#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
namespace humanoid {

int EndEffectorConstraint::eeCount = 0;
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorConstraint::EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                             const ReferenceManager& referenceManager, const int eeIndex)
        : StateConstraint(ConstraintOrder::Linear),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          referenceManagerPtr_(&referenceManager), 
          eeIndex_(eeIndex) {
    eeCount = std::max(eeCount, eeIndex_+1);
    if (endEffectorKinematics.getIds().size() != 1) {
        throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
    }
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t EndEffectorConstraint::getNumConstraints(scalar_t time) const {
    return 6;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool EndEffectorConstraint::isActive(scalar_t time) const {
    // bool active = numerics::almost_eq(referenceManagerPtr_->getTargetTrajectories().stateTrajectory[0](38), 1.0);
    // if (active)
    // {
    //     std::cout << "[EndEffectorConstraint] is active" << std::endl;
    // }
    // TODO： 末端约束的激活条件
    return false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
    // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<HumanoidPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }

    const auto desiredPositionOrientation = interpolateEndEffectorPose(time, state);

    vector_t constraint(6);
    constraint.head<3>() = endEffectorKinematicsPtr_->getPosition(state).front() - desiredPositionOrientation.first;
    constraint.tail<3>() = endEffectorKinematicsPtr_->getOrientationError(state, {desiredPositionOrientation.second}).front();
    return constraint;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                const PreComputation& preComputation) const {
    // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface.
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preCompMM = cast<HumanoidPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
    }

    const auto& stateTrajectory = referenceManagerPtr_->getTargetTrajectories().stateTrajectory;

    auto approximation = VectorFunctionLinearApproximation(6, state.rows());

    if(stateTrajectory[0].size() > 39) {
        const auto desiredPositionOrientation = interpolateEndEffectorPose(time, state);
        const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
        approximation.f.head<3>() = eePosition.f - desiredPositionOrientation.first;
        approximation.dfdx.topRows<3>() = eePosition.dfdx;

        const auto eeOrientationError =
                endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(state, {desiredPositionOrientation.second}).front();
        approximation.f.tail<3>() = eeOrientationError.f;
        approximation.dfdx.bottomRows<3>() = eeOrientationError.dfdx;
    } else {
        approximation.setZero(6, state.rows());
    }

    return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto EndEffectorConstraint::interpolateEndEffectorPose(scalar_t time, const vector_t& state) const -> std::pair<vector_t, quaternion_t> {
    const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
    const auto& timeTrajectory = targetTrajectories.timeTrajectory;
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;

    vector_t position;
    quaternion_t orientation;

    if (stateTrajectory.size() > 1) {
        // Normal interpolation case
        int index;
        scalar_t alpha;
        std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

        // const auto& lhs = stateTrajectory[index].tail<7>();
        const auto& lhs = stateTrajectory[index].tail(eeCount*7).segment<7>(eeIndex_*7);
        const auto& rhs = stateTrajectory[index + 1].tail(eeCount*7).segment<7>(eeIndex_*7);
        const quaternion_t q_lhs(lhs.tail<4>());
        const quaternion_t q_rhs(rhs.tail<4>());

        position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
        orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
    } else {  // stateTrajectory.size() == 1
        position = endEffectorKinematicsPtr_->getPosition(state).front();
        orientation = quaternion_t(1, 0, 0, 0).coeffs();
    }

    return {position, orientation};
}

}  // namespace humanoid
}  // namespace ocs2
