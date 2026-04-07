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

#include "humanoid_interface/constraint/ZeroSixDofForceConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroSixDofForceConstraint::ZeroSixDofForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t sixDofContactPointIndex,
                                         CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      sixDofContactPointIndex_(sixDofContactPointIndex), 
      info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroSixDofForceConstraint::isActive(scalar_t time) const {
  return true;    // TODO:  通过 referenceManager 设置触发的 Trigger
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroSixDofForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
  return centroidal_model::getContactForces(input, info_.numThreeDofContacts + sixDofContactPointIndex_, info_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroSixDofForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const PreComputation& preComp) const {
  VectorFunctionLinearApproximation approx;
  approx.f = getValue(time, state, input, preComp);
  approx.dfdx = matrix_t::Zero(3, state.size());
  approx.dfdu = matrix_t::Zero(3, input.size());
  approx.dfdu.middleCols<3>(3 * info_.numThreeDofContacts + 6 * sixDofContactPointIndex_).diagonal() = vector_t::Ones(3);
  return approx;
}

}  // namespace humanoid
}  // namespace ocs2
