//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_wbc/StandUpWbc.h"

#include <qpOASES.hpp>
#include <fstream>

namespace ocs2
{
  namespace humanoid
  {
    using Duration = std::chrono::duration<double>;
    using Clock = std::chrono::high_resolution_clock;

    vector_t StandUpWbc::update(const vector_t &stateDesired, const vector_t &inputDesired,
                                 const vector_t &rbdStateMeasured, size_t mode, scalar_t period, bool mpc_update)
    {
      WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, mpc_update);
      // Constraints
      Task constraints = formulateConstraints(inputDesired);
      size_t numConstraints = constraints.b_.size() + constraints.f_.size();

      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
      vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
      A << constraints.a_,
          constraints.d_;

      lbA << constraints.b_,
            -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
      ubA << constraints.b_,
            constraints.f_; // clang-format on

      // Cost
      Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
          weighedTask.a_.transpose() * weighedTask.a_;
      vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

      // Solve
      auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_LOW;
      options.enableEqualities = qpOASES::BT_TRUE;
      qpProblem.setOptions(options);
      int nWsr = 200;
      qpOASES::real_t cpu_time = 0.01;
      qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr, &cpu_time);
      vector_t qpSol(getNumDecisionVars());

      int ret = qpProblem.getPrimalSolution(qpSol.data());

      if (ret != qpOASES::SUCCESSFUL_RETURN)
      {
        std::cout << "ERROR: WeightWBC Not Solved!!!" << std::endl;
        if (last_qpSol.size() > 0)
          qpSol = last_qpSol;
        else
          qpSol.setZero();
      }
      last_qpSol = qpSol;
      return qpSol;
    }

    Task StandUpWbc::formulateConstraints(const vector_t &inputDesired)
    {
      return formulateFloatingBaseEomTask(inputDesired) + 
             formulateTorqueLimitsTask() + 
             formulateFrictionConeTask();
    }

    Task StandUpWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
      return formulateCenterOfMassTask(stateDesired,inputDesired,period) * weightBaseLinear_ + 
             formulateBaseAngularMotionTask() * weightBaseAngular_ + 
             formulateStandUpJointAccelTask(stateDesired,inputDesired,period) * weightJointAccel_ +
             formulateNoContactMotionTask() * 1000 +
             formulateContactForceTask(inputDesired) * weightContactForce_;
    }

    void StandUpWbc::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      WbcBase::loadTasksSetting(taskFile, verbose, is_real);

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "standUpWeight.";
      if (verbose)
      {
        std::cerr << "\n #### standUpWbc Weight:";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, weightBaseLinear_, prefix + "Linear", verbose);
      loadData::loadPtreeValue(pt, weightBaseAngular_, prefix + "Angular", verbose);
      loadData::loadPtreeValue(pt, weightJointAccel_, prefix + "JointAcc", verbose);
      loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
    }

    void StandUpWbc::loadSwitchParamsSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      WbcBase::loadSwitchParamsSetting(taskFile, verbose, is_real);

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "switchWeight.";
      if (verbose)
      {
        std::cerr << "\n #### switchWbc Weight:";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, weightBaseLinear_, prefix + "Linear", verbose);
      loadData::loadPtreeValue(pt, weightBaseAngular_, prefix + "Angular", verbose);
      loadData::loadPtreeValue(pt, weightJointAccel_, prefix + "JointAcc", verbose);
      loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
    }

  } // namespace humanoid
} // namespace ocs2
