//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "humanoid_wbc/WeightedWbc.h"

#include <qpOASES.hpp>
#include <cstdio>
#include <unistd.h>

namespace ocs2
{

  void saveBinaryData(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &H,
                      const vector_t &g,
                      const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &A,
                      const vector_t &lbA,
                      const vector_t &ubA,
                      int nWsr,
                      qpOASES::real_t cpu_time,
                      int qpRet,
                      const vector_t &qpSol,
                      const std::string &filename)
  {
    FILE* fp = fopen(filename.c_str(), "wb");
    if (fp != nullptr)
    {
      const size_t nVar = g.size();
      const size_t nC = lbA.size();
      // 写入维度与标量，便于读回时解析
      fwrite(&nVar, sizeof(size_t), 1, fp);
      fwrite(&nC, sizeof(size_t), 1, fp);
      fwrite(&nWsr, sizeof(int), 1, fp);
      fwrite(&cpu_time, sizeof(qpOASES::real_t), 1, fp);
      fwrite(&qpRet, sizeof(int), 1, fp);
      // 写入 QP 输入: H, g, A, lbA, ubA
      fwrite(H.data(), sizeof(double), H.size(), fp);
      fwrite(g.data(), sizeof(double), g.size(), fp);
      fwrite(A.data(), sizeof(double), A.size(), fp);
      fwrite(lbA.data(), sizeof(double), lbA.size(), fp);
      fwrite(ubA.data(), sizeof(double), ubA.size(), fp);
      // 写入 QP 输出: 解向量
      fwrite(qpSol.data(), sizeof(double), qpSol.size(), fp);

      fflush(fp);
      fsync(fileno(fp));
      fclose(fp);
    }
    else
    {
      std::cerr << "Unable to open file for writing";
    }
  }
  namespace humanoid
  {
    using Duration = std::chrono::duration<double>;
    using Clock = std::chrono::high_resolution_clock;

    vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired,
                                 const vector_t &rbdStateMeasured, size_t mode, scalar_t period, bool mpc_update)
    {
      const auto t1 = Clock::now();
      WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, mpc_update);
      const auto t2 = Clock::now();
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

      const auto t3 = Clock::now();
      // Solve
      auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
      qpOASES::Options options;
      options.setToMPC();
      options.printLevel = qpOASES::PL_LOW;
      options.enableEqualities = qpOASES::BT_TRUE;
      qpProblem.setOptions(options);
      int nWsr = 200;
      qpOASES::real_t cpu_time = 0.002;
      const auto t4 = Clock::now();
      // minimize 0.5 * x^T * H * x + g^T * x
      // subject to lbA <= A * x <= ubA
      qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr, &cpu_time);
      vector_t qpSol(getNumDecisionVars());

       int ret = qpProblem.getPrimalSolution(qpSol.data());

      // if (!qpProblem.isSolved())
      if (ret != qpOASES::SUCCESSFUL_RETURN)
      {
        ROS_ERROR_STREAM("WeightWBC Not Solved!!!");
        if (last_qpSol.size() > 0)
          qpSol = last_qpSol;
        else
          qpSol.setZero();
      }

      const auto t5 = Clock::now();
      last_qpSol = qpSol;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t1).count() > 1000)
      {
        std::cout << "at1-t2: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
        std::cout << "at2-t3: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms" << std::endl;
        std::cout << "at3-t4: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() << " ms" << std::endl;
        std::cout << "at4-t5: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << " ms" << std::endl;
        saveBinaryData(H, g, A, lbA, ubA, nWsr, cpu_time, ret, qpSol, "wbc_data.bin");
        std::cout << "save wbc_data.bin, exit..." << std::endl;
        // std::cout << "H: " << H << std::endl;
        // std::cout << "g: " << g.transpose() << std::endl;
        // std::cout << "A: " << A << std::endl;
        // std::cout << "lbA: " << lbA.transpose() << std::endl;
        // std::cout << "ubA: " << ubA.transpose() << std::endl;
        exit(0);
      }
      // std::cout << "qpSol: " << qpSol.transpose() << std::endl;
      return qpSol;
    }

    Task WeightedWbc::formulateConstraints(const vector_t &inputDesired)
    {
      return formulateFloatingBaseEomTask(inputDesired)+ formulateTorqueLimitsTask() + formulateFrictionConeTask();
    }

    Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
    {
     

      Wbc_rdd_K_ = (stance_mode_)? Wbc_rdd_K_stance_ : Wbc_rdd_K_walk_;
      Task totalTask;
      if (half_body_mode_) {
        return formulateArmJointAccelTask(stateDesired, inputDesired, period) * wbc_weight_stance_.weightArmAccel_;
      }
      else if (stance_mode_)
      {
        if (pull_up_state_)// 被拉起
        {
          totalTask = formulateStandUpJointAccelTask(stateDesired,inputDesired,period) * 20 ;
        }
        else // 没有被拉起
        {
          totalTask = formulateCenterOfMassTask(stateDesired, inputDesired, period) * wbc_weight_stance_.weightComPos_ +
                      //  formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
                      formulateBaseXYLinearAccelTask() * wbc_weight_stance_.weightBaseAccelXY_ +
                      formulateBaseHeightMotionTask() * wbc_weight_stance_.weightBaseAccelHeight_ +
                      formulateBaseAngularMotionTask() * wbc_weight_stance_.weightBaseAccelAngular_ + //;// +
                      formulateContactForceTask(inputDesired) * wbc_weight_stance_.weightContactForce_ +
                      formulateNoContactMotionTask() * wbc_weight_stance_.weightStanceLeg_;
        }
        if (roban_mode_)
        {
          totalTask = totalTask + formulateJointAccelTask(stateDesired,inputDesired,period) * 0.1;
        }
      }
      else
      {
        totalTask = formulateSwingLegTask() * wbc_weight_walk_.weightSwingLeg_ +
                    //  formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
                    formulateBaseXYLinearAccelTask() * wbc_weight_walk_.weightBaseAccelXY_ +
                    formulateBaseHeightMotionTask() * wbc_weight_walk_.weightBaseAccelHeight_ +
                    formulateBaseAngularMotionTask() * wbc_weight_walk_.weightBaseAccelAngular_ +
                    formulateContactForceTask(inputDesired) * wbc_weight_walk_.weightContactForce_ +
                    formulateCenterOfMassTask(stateDesired, inputDesired, period) * wbc_weight_walk_.weightComPos_ +
                    formulateNoContactMotionTask() * wbc_weight_walk_.weightStanceLeg_;
        if (roban_mode_)
        {
          totalTask = totalTask + formulateJointAccelTask(stateDesired,inputDesired,period) * 0.1;
        }
      }
      if (arm_nums_ != 0)
      {
        totalTask = totalTask + 
                    formulateArmJointAccelTask(stateDesired, inputDesired, period) * wbc_weight_stance_.weightArmAccel_;
      }
      if (waist_nums_ != 0)
      {
        totalTask = totalTask + 
                    formulateWaistJointAccelTask(stateDesired, inputDesired, period) * wbc_weight_stance_.weightArmAccel_;
      }
     
      return totalTask;
    }

    Task WeightedWbc::formulateStanceBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired,
                                                   scalar_t period)
    {
      // 构造最小二乘问题 min 0.5 * ||ax - b||^2, 优化变量为base acceleration
      matrix_t a(6, numDecisionVars_);
      a.setZero();
      a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

      vector6_t b;
      b.setZero();

      return {a, b, matrix_t(), vector_t()};
    }

    void WeightedWbc::loadTasksSetting(const std::string &taskFile, bool verbose, bool is_real)
    {
      WbcBase::loadTasksSetting(taskFile, verbose, is_real);

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      std::string prefix = "weight.";
      if (verbose)
      {
        std::cerr << "\n #### WBC weight:";
        std::cerr << "\n #### =============================================================================\n";
      }
      // loadData::loadPtreeValue(pt, wbc_weight_stance_.weightSwingLeg_, prefix + "stance.swingLeg", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccel_, prefix + "stance.baseAccel", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightComPos_, prefix + "stance.comPos", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightContactForce_, prefix + "stance.contactForce", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightStanceLeg_, prefix + "stance.stanceLeg", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccelXY_, prefix + "stance.accXY", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccelHeight_, prefix + "stance.height", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightBaseAccelAngular_, prefix + "stance.angular", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightArmAccel_, prefix + "stance.accArm", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_stance_.weightFeetAccel_, prefix + "stance.accFeet", verbose);

      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightSwingLeg_, prefix + "walk.swingLeg", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccel_, prefix + "walk.baseAccel", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightComPos_, prefix + "walk.comPos", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightContactForce_, prefix + "walk.contactForce", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightStanceLeg_, prefix + "walk.stanceLeg", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccelXY_, prefix + "walk.accXY", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccelHeight_, prefix + "walk.height", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightBaseAccelAngular_, prefix + "walk.angular", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightArmAccel_, prefix + "walk.accArm", verbose);
      loadData::loadPtreeValue(pt, wbc_weight_walk_.weightFeetAccel_, prefix + "walk.accFeet", verbose);
    }

  } // namespace humanoid
} // namespace ocs2
