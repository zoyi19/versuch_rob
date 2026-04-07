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

#include <iostream>
#include <string>
#include <future>
#include <dlfcn.h>

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "humanoid_interface/HumanoidInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "humanoid_interface/HumanoidPreComputation.h"
#include "humanoid_interface/constraint/FrictionConeConstraint.h"
#include "humanoid_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "humanoid_interface/constraint/XYReferenceConstraintCppAd.h"
#include "humanoid_interface/constraint/ZeroForceConstraint.h"
#include "humanoid_interface/constraint/ZeroSixDofForceConstraint.h"
#include "humanoid_interface/constraint/ZeroSixDofTorqueConstraint.h"
#include "humanoid_interface/constraint/ZeroVelocityConstraintCppAd.h"
#include "humanoid_interface/constraint/FootRollConstraint.h"
#include "humanoid_interface/constraint/LeggedSelfCollisionConstraint.h"
#include "humanoid_interface/constraint/EndEffectorConstraint.h"
#include "humanoid_interface/constraint/StateInputSoftErrBoxConstraint.h"
#include "humanoid_interface/constraint/CenterOfMassConstraint.h"

#include "humanoid_interface/cost/HumanoidQuadraticTrackingCost.h"
#include "humanoid_interface/dynamics/HumanoidDynamicsAD.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "ocs2_core/thread_support/SetThreadPriority.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2
{
  namespace humanoid
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    HumanoidInterface::HumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitFile,
                                         RobotVersion rb_version, bool useHardFrictionConeConstraint)
        : useHardFrictionConeConstraint_(useHardFrictionConeConstraint), rb_version_(rb_version)
    {
      // check that task file exists
      boost::filesystem::path taskFilePath(taskFile);
      if (boost::filesystem::exists(taskFilePath))
      {
        std::cerr << "[HumanoidInterface] Loading task file: " << taskFilePath << std::endl;
      }
      else
      {
        throw std::invalid_argument("[HumanoidInterface] Task file not found: " + taskFilePath.string());
      }
      // check that urdf file exists
      boost::filesystem::path urdfFilePath(urdfFile);
      if (boost::filesystem::exists(urdfFilePath))
      {
        std::cerr << "[HumanoidInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
      }
      else
      {
        throw std::invalid_argument("[HumanoidInterface] URDF file not found: " + urdfFilePath.string());
      }
      // check that targetCommand file exists
      boost::filesystem::path referenceFilePath(referenceFile);
      if (boost::filesystem::exists(referenceFilePath))
      {
        std::cerr << "[HumanoidInterface] Loading target command settings from: " << referenceFilePath << std::endl;
      }
      else
      {
        throw std::invalid_argument("[HumanoidInterface] targetCommand file not found: " + referenceFilePath.string());
      }

      bool verbose;
      loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);

      // load setting from loading file
      modelSettings_ = loadModelSettings(taskFile, urdfFile, "model_settings", verbose);
      ros::NodeHandle nh;
      ros::param::set("/mpc/mpcArmsDof", modelSettings_.mpcArmsDof);
      ros::param::set("/mpc/mpcLegsDof", modelSettings_.mpcLegsDof);
      ros::param::set("/mpc/mpcWaistDof", modelSettings_.mpcWaistDof);
      mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
      ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
      sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
      ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
      rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
      waist_Num_ = modelSettings_.mpcWaistDof;
      // OptimalConrolProblem

      setupOptimalControlProblem(taskFile, urdfFile, referenceFile, gaitFile, verbose);

      // initial state
      initialState_.setZero(centroidalModelInfo_.stateDim);
      auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version_, true, 2e-3);
      initialState_.head(12 + joint_Num_) = drake_interface_->getInitialState();
    }
    void HumanoidInterface::setupCPUconfig()
    {
      auto isolate_core = getIsolatedCpus();
      std::cout << "setup CPUconfig, get isolated core size: "<< isolate_core.size() << "\n";
      

      if (isolate_core.size() >= 4)
      {
        std::cout << "isolated core: ";
        for (auto i : isolate_core)
        {
          std::cout <<  i << ", ";
        }
        std::cout << std::endl;
        auto it = isolate_core.end() - 1;
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(*it, &cpuset); 
        CPU_SET(*(it - 1), &cpuset); // 倒数一二个
        std::cout << "set thread affinity to core: " << *it << ", " << *(it - 1) << std::endl;
        setThreadAffinity(cpuset,pthread_self());
        setThisThreadPriority(80);
      }else
      {
        std::cout << "not enough isolated core, use all cores.\n";
      }
      
    }
      
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void HumanoidInterface::setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile,
                                                       const std::string &referenceFile, const std::string &gaitFile , bool verbose)
    {
      ros::NodeHandle nh;
      if (nh.hasParam("build_cppad_state"))
      {
        nh.getParam("build_cppad_state", build_cppad_status_);

        if (build_cppad_status_ == 0)
        {
          nh.setParam("build_cppad_state", 1);
          std::cout << "cppad building start in this node. please wait some minutes...\n";
          build_cppad_status_ = 1;
        }
        else if (build_cppad_status_ == 1)
        {
          std::cout << "waiting for anothor node to finish building cppad .... \n";
          while (build_cppad_status_ == 1)
          {
            nh.getParam("build_cppad_state", build_cppad_status_);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }
      }
      else
      {
        std::cerr << "build_cppad_state not found in parameter server, set to 0 by default. \n";
      }
      // PinocchioInterface
      pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));


      vector_t defaultJointState(pinocchioInterfacePtr_->getModel().nq);
      defaultJointState.setZero();
      auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version_, true, 2e-3);
      defaultJointState.head(6 + joint_Num_) = drake_interface_->getInitialState().tail(6 + joint_Num_);

      // CentroidalModelInfo
      centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
          *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile), defaultJointState, modelSettings_.contactNames3DoF,
          modelSettings_.contactNames6DoF);
      // Swing trajectory planner
      auto swingTrajectoryPlanner =
          std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 
                                                   *pinocchioInterfacePtr_, centroidalModelInfo_, 8);

      // Mode schedule manager
      referenceManagerPtr_ =
          std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, gaitFile, verbose), std::move(swingTrajectoryPlanner), 
                                                         *pinocchioInterfacePtr_,  centroidalModelInfo_, 
                                                         modelSettings_, rb_version_);

      // Optimal control problem
      problemPtr_.reset(new OptimalControlProblem);

      // Dynamics
      bool useAnalyticalGradientsDynamics = false;
      loadData::loadCppDataType(taskFile, "humanoid_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
      // std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
      if (useAnalyticalGradientsDynamics)
      {
        throw std::runtime_error("[HumanoidInterface::setupOptimalControlProblem] The analytical dynamics class is not yet implemented!");
      }

      // 检测已有的 CppAD .so 文件是否损坏，如果损坏则删除并重建
      checkAndCleanCorruptedCppAdLibs();
      
      // HumanoidDynamicsAD
      std::future<std::unique_ptr<HumanoidDynamicsAD>> dynamicsFuture = std::async(std::launch::async, [this]()
                                                                                   {
        const std::string modelName = "dynamics";
        return std::make_unique<HumanoidDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_); });

      // PinocchioEndEffectorKinematicsCppAd
      std::vector<std::future<std::unique_ptr<PinocchioEndEffectorKinematicsCppAd>>> footEeKinematicsFutures;
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        const std::string &footName = modelSettings_.contactNames3DoF[i];

        footEeKinematicsFutures.push_back(std::async(std::launch::async, [this, &footName]()
                                                     {
            const auto infoCppAd = centroidalModelInfo_.toCppAd();
            const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
            auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
                const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
                updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
            };
            return std::make_unique<PinocchioEndEffectorKinematicsCppAd>(
                *pinocchioInterfacePtr_, pinocchioMappingCppAd, 
                std::vector<std::string>{footName}, centroidalModelInfo_.stateDim, 
                centroidalModelInfo_.inputDim, velocityUpdateCallback, 
                footName, modelSettings_.modelFolderCppAd, 
                modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd); }));
      }

      // arm
      std::vector<std::future<std::unique_ptr<EndEffectorKinematics<scalar_t>>>> armsEeKinematicsFutures;
      for (size_t i = 0; i < modelSettings_.info.eeFrame.size(); i++)
      {
        const std::string &eeName = modelSettings_.info.eeFrame[i];

        armsEeKinematicsFutures.push_back(std::async(std::launch::async, [this, &eeName]()
                                                     { return getEeKinematicsPtr({eeName}, eeName); }));
      }

      // 等待所有任务完成并获取结果
      auto dynamicsPtr = dynamicsFuture.get();
      std::vector<std::unique_ptr<PinocchioEndEffectorKinematicsCppAd>> eeKinematicsPtrs;
      std::vector<std::unique_ptr<PinocchioEndEffectorKinematicsCppAd>> armKinematicsPtrs;

      for (auto &future : footEeKinematicsFutures)
      {
        eeKinematicsPtrs.push_back(future.get());
      }
      for (auto &future : armsEeKinematicsFutures)
      {
        future.get();
        // armKinematicsPtrs.push_back(future.get());
      }

      std::cout << "build cppad model complete..." << std::endl;
      

      nh.setParam("build_cppad_state", 2); // done

      problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

      // Cost terms
      problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, false));
      bool enableBaseTrackingTerminalCost = false;
      loadData::loadCppDataType(taskFile, "enableBaseTrackingTerminalCost", enableBaseTrackingTerminalCost);
      if (enableBaseTrackingTerminalCost)
      {
        std::cout << "enableBaseTrackingTerminalCost" << std::endl;
        problemPtr_->finalCostPtr->add("baseTrackingTerminalCost", getBaseTrackingTerminalCost(taskFile, centroidalModelInfo_, false));
      }

      bool enableBasePitchLimitsCost = false;
      loadData::loadCppDataType(taskFile, "basePitchLimits.enable", enableBasePitchLimitsCost);
      referenceManagerPtr_->setEnablePitchLimit(enableBasePitchLimitsCost);// set initial value
      if (enableBasePitchLimitsCost)
        problemPtr_->softConstraintPtr->add("pitchLimits", getBasePitchSoftConstraint(taskFile, false));

      // Constraint terms
      // friction cone settings
      scalar_t frictionCoefficient = 0.7;
      RelaxedBarrierPenalty::Config barrierPenaltyConfig;
      std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);
      scalar_t zero_velocity_scale = 0.2;
      bool use_zero_velocity_soft_constraints = true;
      std::tie(zero_velocity_scale, use_zero_velocity_soft_constraints) = loadZeroVelocityConstraintScale(taskFile, verbose);
      bool useAnalyticalGradientsConstraints = false;
      loadData::loadCppDataType(taskFile, "humanoid_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        const std::string &footName = modelSettings_.contactNames3DoF[i];

        // std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
        if (useAnalyticalGradientsConstraints)
        {
          throw std::runtime_error(
              "[HumanoidInterface::setupOptimalControlProblem] The analytical end-effector linear constraint is not implemented!");
        }
        else
        {
          const auto infoCppAd = centroidalModelInfo_.toCppAd();
          const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
          auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t &state, PinocchioInterfaceCppAd &pinocchioInterfaceAd)
          {
            const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
            updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
          };
          // eeKinematicsPtr.reset(eeKinematicsPtrs[i]);
        }

        if (useHardFrictionConeConstraint_)
        {
          problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone", getFrictionConeConstraint(i, frictionCoefficient));
        }
        else
        {
          problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                              getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));
        }
        problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));
        if (!use_zero_velocity_soft_constraints)
        {
          // if (i == 0 || i == 4) //如果使用ddp+硬约束，则应只约束一个点
          problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                                  getZeroVelocityConstraint(*eeKinematicsPtrs[i], i, useAnalyticalGradientsConstraints));
        }
        else
          problemPtr_->softConstraintPtr->add(footName + "_zeroVelocity",
                                              getZeroVelocitySoftConstraint(*eeKinematicsPtrs[i], i, useAnalyticalGradientsConstraints, zero_velocity_scale));
        problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity",
                                                getNormalVelocityConstraint(*eeKinematicsPtrs[i], i, useAnalyticalGradientsConstraints));
        problemPtr_->softConstraintPtr->add(footName + "_xySwingSoft",
                                        getSoftSwingTrajConstraint(*eeKinematicsPtrs[i], i, taskFile, verbose));
        // 由于一只脚上有4个虚拟接触点，只需要给其中一个接触点添加滚转角约束
        // if (i == 0 || i == 4) // ll_toe + rl_toe
        // if (i < 2)
        // {
        //   problemPtr_->equalityConstraintPtr->add(footName + "_footRoll", getFootRollConstraint(i));
        // }
      }
      bool enableZeroSixDofForceConstraint = false;
      loadData::loadCppDataType(taskFile, "enableZeroSixDofForceConstraint", enableZeroSixDofForceConstraint);
      if (enableZeroSixDofForceConstraint)
      {
        for (size_t i = 0; i < centroidalModelInfo_.numSixDofContacts; i++)
        {
          const std::string &sixContactPointName = modelSettings_.contactNames6DoF[i];
          problemPtr_->equalityConstraintPtr->add(sixContactPointName + "_zeroSixDofForce", getZeroSixDofForceConstraint(i));
          problemPtr_->equalityConstraintPtr->add(sixContactPointName + "_zeroSixDofTorque", getZeroSixDofTorqueConstraint(i));
        }
      }

      // ee Cost
      for (size_t i = 0; i < modelSettings_.info.eeFrame.size(); i++)
      {
        const std::string &eeName = modelSettings_.info.eeFrame[i];
        problemPtr_->stateSoftConstraintPtr->add(eeName + "_endEffector",
                                                 getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "endEffector", eeName, i, verbose));
        problemPtr_->finalSoftConstraintPtr->add(eeName + "_finalEndEffector",
                                                 getEndEffectorConstraint(*pinocchioInterfacePtr_, taskFile, "finalEndEffector", eeName, i, verbose));
      }

      // Self-collision avoidance constraint
      problemPtr_->stateSoftConstraintPtr->add("selfCollision",
                                               getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));
      
      // Center of Mass constraint
      auto centerOfMassConstraint = getCenterOfMassConstraint(*pinocchioInterfacePtr_, taskFile, verbose);
      if (centerOfMassConstraint != nullptr) {
        problemPtr_->stateSoftConstraintPtr->add("centerOfMass", std::move(centerOfMassConstraint));
      }

      // Pre-computation
      problemPtr_->preComputationPtr.reset(new HumanoidPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                      *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_));

      // Rollout
      rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

      // Initialization
      constexpr bool extendNormalizedMomentum = true;
      initializerPtr_.reset(new HumanoidInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateCost> HumanoidInterface::getEndEffectorConstraint(const ocs2::PinocchioInterface &pinocchioInterface,
                                                                           const std::string &taskFile, const std::string &prefix,
                                                                           const std::string &eeName, const int eeIndex, bool verbose)
    {
      scalar_t muPosition = 1.0;
      scalar_t muOrientation = 1.0;

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);

      loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", verbose);
      loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", verbose);

      if (referenceManagerPtr_ == nullptr)
      {
        throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr should be set first!");
      }

      std::unique_ptr<StateConstraint> constraint;
      std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({eeName}, eeName);
      constraint.reset(new EndEffectorConstraint(*eeKinematicsPtr, *referenceManagerPtr_, eeIndex));
      std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
      std::generate_n(penaltyArray.begin(), 3, [&]
                      { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
      std::generate_n(penaltyArray.begin() + 3, 3, [&]
                      { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

      return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputCost> HumanoidInterface::getBasePitchSoftConstraint(const std::string& taskFile, bool verbose) {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);

        const int pitchAngleIdx = 6 + 4;
        const int pitchAngleVelIdx = 4;

        // Load Pitch position limits
        std::vector<StateInputSoftErrBoxConstraint::BoxConstraint> stateLimits;
        stateLimits.reserve(2);
        {
            scalar_t muPitchLimits = 1e-2;
            scalar_t deltaPitchLimits = 1e-3;

            scalar_t lowerBoundPitch = 0;
            scalar_t upperBoundPitch = 0;

            loadData::loadPtreeValue(pt, muPitchLimits, "basePitchLimits.pos.mu", verbose);
            loadData::loadPtreeValue(pt, deltaPitchLimits, "basePitchLimits.pos.delta", verbose);

            loadData::loadPtreeValue(pt, lowerBoundPitch, "basePitchLimits.pos.lowerBound", verbose);
            loadData::loadPtreeValue(pt, upperBoundPitch, "basePitchLimits.pos.upperBound", verbose);

            StateInputSoftErrBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = pitchAngleIdx;
            boxConstraint.lowerBound = lowerBoundPitch;
            boxConstraint.upperBound = upperBoundPitch;
            boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPitchLimits, deltaPitchLimits}));
            stateLimits.push_back(std::move(boxConstraint));
        }

        {
            scalar_t muPitchVelLimits = 1e-2;
            scalar_t deltaPitchVelLimits = 1e-3;

            scalar_t lowerBoundPitchVel = 0;
            scalar_t upperBoundPitchVel = 0;

            loadData::loadPtreeValue(pt, muPitchVelLimits, "basePitchLimits.vel.mu", verbose);
            loadData::loadPtreeValue(pt, deltaPitchVelLimits, "basePitchLimits.vel.delta", verbose);

            loadData::loadPtreeValue(pt, lowerBoundPitchVel, "basePitchLimits.vel.lowerBound", verbose);
            loadData::loadPtreeValue(pt, upperBoundPitchVel, "basePitchLimits.vel.upperBound", verbose);

            StateInputSoftErrBoxConstraint::BoxConstraint boxConstraint;
            boxConstraint.index = pitchAngleVelIdx;
            boxConstraint.lowerBound = lowerBoundPitchVel;
            boxConstraint.upperBound = upperBoundPitchVel;
            boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty({muPitchVelLimits, deltaPitchVelLimits}));
            stateLimits.push_back(std::move(boxConstraint));
        }

        // load arm velocity limits
        std::vector<StateInputSoftErrBoxConstraint::BoxConstraint> inputLimits;

        auto boxConstraints = std::unique_ptr<StateInputSoftErrBoxConstraint>(new StateInputSoftErrBoxConstraint(*referenceManagerPtr_, stateLimits, inputLimits));
        boxConstraints->initializeOffset(0.0, vector_t::Zero(centroidalModelInfo_.stateDim), vector_t::Zero(centroidalModelInfo_.inputDim));
        return boxConstraints;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::shared_ptr<GaitSchedule> HumanoidInterface::loadGaitSchedule(const std::string &file, const std::string & gaitfile, bool verbose) const
    {
      const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
      const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

      const auto defaultGait = [&]
      {
        Gait gait{};
        gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
        // Events: from time -> phase
        std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                      [&](double eventTime)
                      { gait.eventPhases.push_back(eventTime / gait.duration); });
        // Modes:
        gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
        return gait;
      }();

      // display
      if (verbose)
      {
        std::cerr << "\n#### Modes Schedule: ";
        std::cerr << "\n#### =============================================================================\n";
        std::cerr << "Initial Modes Schedule: \n"
                  << initModeSchedule;
        std::cerr << "Default Modes Sequence Template: \n"
                  << defaultModeSequenceTemplate;
        std::cerr << "#### =============================================================================\n";
      }

      return std::make_shared<GaitSchedule>(file, gaitfile, modelSettings_.phaseTransitionStanceTime);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    matrix_t HumanoidInterface::initializeInputCostWeight(const std::string &taskFile, const CentroidalModelInfo &info)
    {
      const size_t totalThreeDofContactDim = 3 * info.numThreeDofContacts;
      const size_t totalSixDofContactDim = 6 * info.numSixDofContacts;
      const size_t totalContactDim = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
      int originArmDof = modelSettings_.modelDof - modelSettings_.mpcLegsDof - modelSettings_.mpcWaistDof;
      int dualArmDof = info.actuatedDofNum - joint_Num_ - waist_Num_;  //减waist的自由度
      int halfOriginArmDof = originArmDof / 2;
      int halfDualArmDof = dualArmDof / 2;
      int armDofDiff = originArmDof - dualArmDof;

      vector_t initialState(centroidalModelInfo_.stateDim);
      initialState.setZero();
      auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(rb_version_, true, 2e-3);
      initialState.head(12 + joint_Num_) = drake_interface_->getInitialState();

      const auto &model = pinocchioInterfacePtr_->getModel();
      auto &data = pinocchioInterfacePtr_->getData();
      const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
      //std::cout << "q:\n" << q.transpose() << std::endl;
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      matrix_t baseToFeetJacobians(totalThreeDofContactDim, joint_Num_);
      for (size_t i = 0; i < info.numThreeDofContacts; i++)
      {
        matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                    jacobianWorldToContactPointInWorldFrame);
        baseToFeetJacobians.block(3 * i, 0, 3, joint_Num_) =
            jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, joint_Num_);
      }
      //std::cout << "baseToFeetJacobians:\n" << baseToFeetJacobians << std::endl;

      matrix_t R_modelSpace(totalThreeDofContactDim * 2 + totalSixDofContactDim + originArmDof + waist_Num_, totalThreeDofContactDim * 2 + totalSixDofContactDim + originArmDof + waist_Num_);
      loadData::loadEigenMatrix(taskFile, "R", R_modelSpace);      
      matrix_t R_taskspace(totalThreeDofContactDim * 2 + totalSixDofContactDim + dualArmDof + waist_Num_, totalThreeDofContactDim * 2 + totalSixDofContactDim + dualArmDof + waist_Num_);
      R_taskspace.setZero();
      { // 简化QR
        
        int DofWithoutArms = totalThreeDofContactDim * 2 + totalSixDofContactDim + waist_Num_;

        R_taskspace.block(0, 0, DofWithoutArms, DofWithoutArms) =
            R_modelSpace.block(0, 0, DofWithoutArms, DofWithoutArms);
        for (int i = 0; i < 2; i++)
        {
          R_taskspace.block(DofWithoutArms + halfDualArmDof * i, DofWithoutArms + halfDualArmDof * i, halfDualArmDof, halfDualArmDof) =
              R_modelSpace.block(DofWithoutArms + halfOriginArmDof * i, DofWithoutArms + halfOriginArmDof * i, halfDualArmDof, halfDualArmDof);
        }
      }

      matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);
      // Contact Forces
      R.topLeftCorner(totalThreeDofContactDim, totalThreeDofContactDim) = R_taskspace.topLeftCorner(totalThreeDofContactDim, totalThreeDofContactDim);
      if (totalSixDofContactDim != 0)
      {
        R.block(totalThreeDofContactDim, totalThreeDofContactDim, totalSixDofContactDim, totalSixDofContactDim) =
            R_taskspace.block(totalThreeDofContactDim, totalThreeDofContactDim, totalSixDofContactDim, totalSixDofContactDim);
      }

      // Leg Joint velocities
      R.block(totalContactDim, totalContactDim, joint_Num_, joint_Num_) =
          baseToFeetJacobians.transpose() * R_taskspace.block(totalContactDim, totalContactDim, totalThreeDofContactDim, totalThreeDofContactDim) * baseToFeetJacobians;
      
      // Waist Joint velocities
      R.block(totalContactDim + joint_Num_, totalContactDim + joint_Num_, waist_Num_, waist_Num_) = 
          R_taskspace.block(totalContactDim + totalThreeDofContactDim, totalContactDim + totalThreeDofContactDim, waist_Num_, waist_Num_);

      // Arm Joint velocities
      if (dualArmDof != 0)
      {
        R.bottomRightCorner(dualArmDof, dualArmDof) = R_taskspace.bottomRightCorner(dualArmDof, dualArmDof);
      }

      return R;
    }
    matrix_t HumanoidInterface::initializeStateCostWeight(const std::string &taskFile, const CentroidalModelInfo &info, std::string feildName)
    {
      int originArmDof = modelSettings_.modelDof - modelSettings_.mpcLegsDof - modelSettings_.mpcWaistDof;
      int mpcArmsDof = modelSettings_.mpcArmsDof;
      int armDofDiff = originArmDof - mpcArmsDof;
      int halfArmDof = mpcArmsDof / 2;
      int halfOriginArmDof = originArmDof / 2;
      matrix_t Q_origin(info.stateDim + armDofDiff, info.stateDim + armDofDiff);
      matrix_t Q(info.stateDim, info.stateDim);
      Q.setZero();
      loadData::loadEigenMatrix(taskFile, feildName, Q_origin);
      { // 简化Q
        int DofWithoutArms = info.stateDim - mpcArmsDof;
        Q.block(0, 0, DofWithoutArms, DofWithoutArms) = Q_origin.block(0, 0, DofWithoutArms, DofWithoutArms);
        for (int i = 0; i < 2; i++)
        {
          Q.block(DofWithoutArms + halfArmDof * i, DofWithoutArms + halfArmDof * i, halfArmDof, halfArmDof) =
              Q_origin.block(DofWithoutArms + halfOriginArmDof * i, DofWithoutArms + halfOriginArmDof * i, halfArmDof, halfArmDof);
        }
      }
      //std::cout << "Q:\n" << Q << std::endl;
      return Q;
    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputCost> HumanoidInterface::getBaseTrackingCost(const std::string &taskFile, const CentroidalModelInfo &info,
                                                                           bool verbose)
    {
      matrix_t Q = initializeStateCostWeight(taskFile, info, "Q");
      matrix_t R = initializeInputCostWeight(taskFile, info);
      if (verbose)
      {
        std::cerr << "\n #### Base Tracking Cost Coefficients: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "Q:\n"
                  << Q << "\n";
        std::cerr << "R:\n"
                  << R << "\n";
        std::cerr << " #### =============================================================================\n";
      }

      return std::make_unique<HumanoidStateInputQuadraticCost>(std::move(Q), std::move(R), info, *referenceManagerPtr_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateCost> HumanoidInterface::getBaseTrackingTerminalCost(const std::string &taskFile, const CentroidalModelInfo &info,
                                                                              bool verbose)
    {
      matrix_t P = initializeStateCostWeight(taskFile, info, "P");

      if (verbose)
      {
        std::cerr << "\n #### Base Tracking Cost Terminal Coefficients: ";
        std::cerr << "\n #### =============================================================================\n";
        std::cerr << "P:\n"
                  << P << "\n";
        std::cerr << " #### =============================================================================\n";
      }

      return std::make_unique<HumanoidStateQuadraticCost>(std::move(P), info, *referenceManagerPtr_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::pair<scalar_t, RelaxedBarrierPenalty::Config> HumanoidInterface::loadFrictionConeSettings(const std::string &taskFile,
                                                                                                   bool verbose) const
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      const std::string prefix = "frictionConeSoftConstraint.";

      scalar_t frictionCoefficient = 1.0;
      RelaxedBarrierPenalty::Config barrierPenaltyConfig;
      if (verbose)
      {
        std::cerr << "\n #### Friction Cone Settings: ";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
      loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
      loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
      if (verbose)
      {
        std::cerr << " #### =============================================================================\n";
      }

      return {frictionCoefficient, std::move(barrierPenaltyConfig)};
    }

    std::pair<scalar_t, bool> HumanoidInterface::loadZeroVelocityConstraintScale(const std::string &taskFile, bool verbose) const
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      const std::string prefix = "zeroVelocityConstraint.";

      scalar_t zero_velocity_constraint_scale = 1.0;
      bool use_zero_velocity_soft_constraints = true;
      if (verbose)
      {
        std::cerr << "\n #### Zero Velocity Constraint Settings: ";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, use_zero_velocity_soft_constraints, prefix + "enable", verbose);
      loadData::loadPtreeValue(pt, zero_velocity_constraint_scale, prefix + "scale", verbose);
      if (verbose)
      {
        std::cerr << " #### =============================================================================\n";
      }

      return {zero_velocity_constraint_scale, use_zero_velocity_soft_constraints};
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getFrictionConeConstraint(size_t contactPointIndex,
                                                                                       scalar_t frictionCoefficient)
    {
      FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
      return std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex,
                                                      centroidalModelInfo_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputCost> HumanoidInterface::getFrictionConeSoftConstraint(
        size_t contactPointIndex, scalar_t frictionCoefficient, const RelaxedBarrierPenalty::Config &barrierPenaltyConfig)
    {
      return std::make_unique<StateInputSoftConstraint>(getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
                                                        std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getZeroForceConstraint(size_t contactPointIndex)
    {
      return std::make_unique<ZeroForceConstraint>(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getZeroSixDofForceConstraint(size_t sixDofContactPointIndex)
    {
      return std::make_unique<ZeroSixDofForceConstraint>(*referenceManagerPtr_, sixDofContactPointIndex, centroidalModelInfo_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getZeroSixDofTorqueConstraint(size_t sixDofContactPointIndex)
    {
      return std::make_unique<ZeroSixDofTorqueConstraint>(*referenceManagerPtr_, sixDofContactPointIndex, centroidalModelInfo_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                                       size_t contactPointIndex,
                                                                                       bool useAnalyticalGradients)
    {
      auto eeZeroVelConConfig = [](scalar_t positionErrorGain)
      {
        EndEffectorLinearConstraint::Config config;
        config.b.setZero(3);
        config.Av.setIdentity(3, 3);
        if (!numerics::almost_eq(positionErrorGain, 0.0))
        {
          config.Ax.setZero(3, 3);
          config.Ax(2, 2) = positionErrorGain;
        }
        return config;
      };

      if (useAnalyticalGradients)
      {
        throw std::runtime_error(
            "[HumanoidInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
      }
      else
      {
        
        return std::make_unique<ZeroVelocityConstraintCppAd>(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                             eeZeroVelConConfig(modelSettings_.positionErrorGain));
      }
    }

    std::unique_ptr<StateInputCost> HumanoidInterface::getZeroVelocitySoftConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                                     size_t contactPointIndex, bool useAnalyticalGradients,
                                                                                     const scalar_t &scale)
    {
      return std::make_unique<StateInputSoftConstraint>(getZeroVelocityConstraint(eeKinematics, contactPointIndex, useAnalyticalGradients),
                                                        std::make_unique<QuadraticPenalty>(scale));
    }
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                                         size_t contactPointIndex,
                                                                                         bool useAnalyticalGradients)
    {
      if (useAnalyticalGradients)
      {
        throw std::runtime_error(
            "[HumanoidInterface::getNormalVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
      }
      else
      {
        return std::make_unique<NormalVelocityConstraintCppAd>(*referenceManagerPtr_, eeKinematics, contactPointIndex);
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputCost> HumanoidInterface::getSoftSwingTrajConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                size_t contactPointIndex, const std::string& taskFile, bool verbose)
    {
      scalar_t weight = 10;

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      if (verbose)
      {
        std::cerr << "\n #### SoftSwingTraj Settings: ";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, weight, "softSwingTrajConstraint.weight", verbose);
      return std::make_unique<StateInputSoftConstraint>(
          std::make_unique<XYReferenceConstraintCppAd>(*referenceManagerPtr_, eeKinematics, contactPointIndex),
          std::make_unique<QuadraticPenalty>(weight));
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint> HumanoidInterface::getFootRollConstraint(size_t contactPointIndex)
    {
      return std::make_unique<FootRollConstraint>(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateCost> HumanoidInterface::getSelfCollisionConstraint(const PinocchioInterface &pinocchioInterface,
                                                                             const std::string &taskFile, const std::string &prefix,
                                                                             bool verbose)
    {
      std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
      std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
      scalar_t mu = 1e-2;
      scalar_t delta = 1e-3;
      scalar_t minimumDistance = 0.0;

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      if (verbose)
      {
        std::cerr << "\n #### SelfCollision Settings: ";
        std::cerr << "\n #### =============================================================================\n";
      }
      loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
      loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
      loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
      loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
      loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

      geometryInterfacePtr_ = std::make_unique<PinocchioGeometryInterface>(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
      if (verbose)
      {
        std::cerr << " #### =============================================================================\n";
        const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
        std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
      }

      std::unique_ptr<StateConstraint> constraint = std::make_unique<LeggedSelfCollisionConstraint>(
          CentroidalModelPinocchioMapping(centroidalModelInfo_), *geometryInterfacePtr_, minimumDistance);

      auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{mu, delta});

      return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<StateCost> HumanoidInterface::getCenterOfMassConstraint(const PinocchioInterface &pinocchioInterface, const std::string& taskFile, bool verbose)
    {
      // Load constraint parameters from config file
      bool enableCenterOfMassConstraint = false;
      bool useRelaxedBarrier = true;  // Default to use relaxed barrier
      scalar_t muCenterOfMass = 10.0;  // Default penalty weight for quadratic penalty
      RelaxedBarrierPenalty::Config barrierConfig;  // For relaxed barrier penalty
      
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      
      loadData::loadPtreeValue(pt, enableCenterOfMassConstraint, "centerOfMass.enable", verbose);
      loadData::loadPtreeValue(pt, useRelaxedBarrier, "centerOfMass.useRelaxedBarrier", verbose);
      
      if (!enableCenterOfMassConstraint) {
        if (verbose) {
          std::cerr << "[HumanoidInterface] Center of Mass constraint is disabled." << std::endl;
        }
        return nullptr;
      }
      
      // Load penalty parameters based on type
      if (useRelaxedBarrier) {
        loadData::loadPtreeValue(pt, barrierConfig.mu, "centerOfMass.mu", verbose);
        loadData::loadPtreeValue(pt, barrierConfig.delta, "centerOfMass.delta", verbose);
        if (verbose) {
          std::cerr << "[HumanoidInterface] Center of Mass constraint is enabled with RelaxedBarrierPenalty:" << std::endl;
          std::cerr << "  mu = " << barrierConfig.mu << ", delta = " << barrierConfig.delta << std::endl;
        }
      } else {
        loadData::loadPtreeValue(pt, muCenterOfMass, "centerOfMass.weight", verbose);
        if (verbose) {
          std::cerr << "[HumanoidInterface] Center of Mass constraint is enabled with QuadraticPenalty:" << std::endl;
          std::cerr << "  weight = " << muCenterOfMass << std::endl;
        }
      }
      
      
      // Check if we have enough contact points
      if (modelSettings_.contactNames3DoF.size() < 8) {
        if (verbose) {
          std::cerr << "[HumanoidInterface] Warning: Not enough contact points for center of mass constraint" << std::endl;
          std::cerr << "  Available: " << modelSettings_.contactNames3DoF.size() << ", Required: 8" << std::endl;
        }
        return nullptr;
      }
      
      // Create the constraint with contact indices and frame names
      std::unique_ptr<StateConstraint> constraint = std::make_unique<CenterOfMassConstraint>(
          pinocchioInterface, centroidalModelInfo_, *referenceManagerPtr_,
          modelSettings_.contactNames3DoF);
      
      std::cout << "[HumanoidInterface] Center of Mass constraint created with contact points: ";
      for (const auto& name : modelSettings_.contactNames3DoF) {
        std::cout << name << " ";
      }
      std::cout << std::endl;
      
      // Create penalty based on type
      if (useRelaxedBarrier) {
        // For relaxed barrier penalty, use a single penalty for all 4 boundary constraints
        auto penalty = std::make_unique<RelaxedBarrierPenalty>(barrierConfig);
        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
      } else {
        // For quadratic penalty, use a single penalty for all 4 boundary constraints
        auto penalty = std::make_unique<QuadraticPenalty>(muCenterOfMass);
        return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
      }
    }
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::unique_ptr<EndEffectorKinematics<scalar_t>> HumanoidInterface::getEeKinematicsPtr(const std::vector<std::string> &endEffectorIds, const std::string &modelName)
    {
      std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

      const auto infoCppAd = centroidalModelInfo_.toCppAd();
      const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
      auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t &state, PinocchioInterfaceCppAd &pinocchioInterfaceAd)
      {
        const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
        updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
      };
      eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, endEffectorIds,
                                                                    centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                    velocityUpdateCallback, modelName, modelSettings_.modelFolderCppAd,
                                                                    modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

      return eeKinematicsPtr;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    // 检查 cppadDir 目录下所有 .so 动态库文件是否可以正常加载，如果存在损坏的库文件，重新生成
    void HumanoidInterface::checkAndCleanCorruptedCppAdLibs()
    {
      std::string cppadDir = modelSettings_.modelFolderCppAd;
      
      std::string findCmd = "find " + cppadDir + " -name '*.so'";
      
      // popen() 创建一个子进程执行命令，并返回一个可读取命令输出的文件指针
      FILE* pipe = popen(findCmd.c_str(), "r");
      if (!pipe) return;  // 如果管道创建失败，直接返回
      
      char buffer[512];   // 用于存储每行 find 命令输出的缓冲区
      bool allValid = true;  // 标记所有库文件是否都有效
      
      // 逐行读取 find 命令的输出（每行是一个 .so 文件的完整路径）
      while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string soPath(buffer);
        soPath.erase(soPath.find_last_not_of(" \n\r\t") + 1);  // 去除路径字符串末尾的空白字符
        
        // 尝试动态加载 .so 库文件
        void* handle = dlopen(soPath.c_str(), RTLD_LAZY);
        
        if (handle == nullptr) {
          std::cerr << "[CppAD] 检测到损坏的库文件: " << soPath << std::endl;
          std::cerr << "[CppAD] 错误: " << dlerror() << std::endl;         // dlopen 失败说明库文件损坏或依赖缺失
          allValid = false;
          break;  // 发现损坏文件后立即停止检查
        }
        dlclose(handle);        // 验证通过后关闭库句柄，释放资源
      }

      pclose(pipe);      // 关闭管道，回收子进程资源
      
      if (!allValid) {
        std::cerr << "[CppAD] 正在删除并重建: " << cppadDir << std::endl;
        (void)system(("rm -rf " + cppadDir).c_str());
        modelSettings_.recompileLibrariesCppAd = true;
      }
    }

  } // namespace humanoid
} // namespace ocs2
