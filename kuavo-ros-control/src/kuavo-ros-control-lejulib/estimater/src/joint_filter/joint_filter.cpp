#include "kuavo_estimation/joint_filter/joint_filter.h"
namespace HighlyDynamic
{
    using namespace drake;
    Eigen::MatrixXd solveDARE(const Eigen::MatrixXd &A, const Eigen::MatrixXd &H, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R, int maxIterations, double tolerance)
    {
        Eigen::MatrixXd P = Q;
        Eigen::MatrixXd P_prev;
        int i = 0;
        for (i = 0; i < maxIterations; ++i)
        {
            P_prev = P;
            Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
            P = A.transpose() * P * A - A.transpose() * P * H.transpose() * K + Q;

            // 检查收敛性
            if ((P - P_prev).norm() < tolerance)
            {
                break;
            }
        }
        std::cout << "DARE solver converged in " << i << " iterations." << std::endl;
        if (i == maxIterations - 1)
        {
            std::cerr << "Warning: DARE solver did not converge within " << maxIterations << " iterations." << std::endl;
        }
        return P;
    }
    Eigen::MatrixXd JointFilter::solvePosKk()
    {
        Eigen::MatrixXd Q(12, 12);
        Q.setIdentity();
        for (int i = 0; i < 12; i++)
        {
            Q(i, i) *= joint_pos_filter_Q_[i];
        }
        Eigen::MatrixXd R(12, 12);
        R.setIdentity();
        for (int i = 0; i < 12; i++)
        {
            R(i, i) *= joint_pos_filter_R_[i];
        }
        Eigen::Matrix<double, 12, 12> F = Eigen::Matrix<double, 12, 12>::Identity();
        Eigen::Matrix<double, 12, 12> H = Eigen::Matrix<double, 12, 12>::Identity();
        Eigen::Matrix<double, 12, 12> joint_pos_filter_P = solveDARE(F, H, Q, R);
        Eigen::MatrixXd joint_pos_filter_K = joint_pos_filter_P * H.transpose() * (H * joint_pos_filter_P * H.transpose() + R).inverse();
        return joint_pos_filter_K;
    }
    void JointFilter::update(const Eigen::VectorXd &rbdState, Eigen::VectorXd &joint_pos, Eigen::VectorXd &joint_vel, const Eigen::VectorXd &joint_current, const Eigen::VectorXd &tau_input, const size_t &mode)
    {

        Eigen::Vector2d defor_k;
        defor_k << l_joint_defor_K, r_joint_defor_K;
        for (size_t i = 0; i < joint_pos_hip_offset_.size(); i++)
        {
            joint_pos_hip_offset_[i] = (joint_current[i * 6] / defor_k[i]) * M_PI / 180;
        }
        size_t generalizedCoordinatesNum = rbdState.size() / 2;
        angular_velocity_hip_ = -(velocity_base_pre[1] / pose_base_pre[2] + angular_velocity_base_pre[0]);

        auto velocity_base = rbdState.segment<3>(generalizedCoordinatesNum + 3);
        auto pose_base = rbdState.segment<3>(3);
        auto angular_velocity_base = rbdState.segment<3>(generalizedCoordinatesNum);
        velocity_base_pre = velocity_base;
        pose_base_pre = pose_base;
        angular_velocity_base_pre = angular_velocity_base;
        int touch_down_state = mode2state(mode);
        switch (touch_down_state)
        {
        case 0:
        case 1:
            joint_vel[0] = angular_velocity_hip_;
            break;
        case 2:
            joint_vel[6] = angular_velocity_hip_;
            break;
        default:
            break;
        }
        logPublishVector("joint_filter/joint_vel", joint_vel);
        logPublishVector("joint_filter/joint_pos_hip_offset_", joint_pos_hip_offset_);
        this->jointPositionFilter(rbdState, joint_pos, joint_vel, tau_input, mode);
        // this->jointVelocityFilter(rbdState, joint_vel, tau_input, mode);
    }

    void JointFilter::jointVelocityFilter(const Eigen::VectorXd &rbdState, Eigen::VectorXd &joint_vel, const Eigen::VectorXd &tau_input, const size_t &mode)
    {
        Eigen::VectorXd qv = rbdState2drakeQV(rbdState);
        plant_->SetPositionsAndVelocities(plant_context_.get(), qv);
        Eigen::VectorXd qd = qv.tail(nv_);
        int touch_down_state = mode2state(mode);
        bool is_changing_state = (touch_down_state != touch_down_state_prev_) ? true : false;
        touch_down_state_prev_ = touch_down_state;
        double detal_x = 1e-3; // 无穷小量
        Eigen::VectorXd tau = tau_input;
        // tau.setZero();
        // std::cout << "touch_down_state: " << touch_down_state << std::endl;
        // std::cout << "qv: " << qv.transpose() << std::endl;
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Kalmen_test

        struct timespec t0_kfsol, t1_kfsol;
        clock_gettime(CLOCK_MONOTONIC, &t0_kfsol);

        Eigen::MatrixXd mass_matrix(18, 18);
        Eigen::MatrixXd Js_V_wxjr(6, 18);
        Eigen::MatrixXd Js_V_wxjl(6, 18);
        plant_->CalcMassMatrix(*plant_context_, &mass_matrix);

        plant_->CalcJacobianSpatialVelocity(*plant_context_, multibody::JacobianWrtVariable::kV,
                                            plant_->GetFrameByName(end_frames_name_[2]), Eigen::Vector3d::Zero(),
                                            plant_->world_frame(), plant_->world_frame(), &Js_V_wxjr);
        plant_->CalcJacobianSpatialVelocity(*plant_context_, multibody::JacobianWrtVariable::kV,
                                            plant_->GetFrameByName(end_frames_name_[1]), Eigen::Vector3d::Zero(),
                                            plant_->world_frame(), plant_->world_frame(), &Js_V_wxjl);

        Eigen::MatrixXd S(18, 18);
        S.setIdentity();
        S.topLeftCorner(6, 6).setZero();
        /// S构建完毕
        Eigen::MatrixXd tau_kfliter(18, 1);
        tau_kfliter.setZero();
        tau_kfliter.bottomRows(12) = tau;

        ////Tau构建完毕
        Eigen::VectorXd cv_kf(nv_);
        plant_->CalcBiasTerm(*plant_context_, &cv_kf);

        Eigen::VectorXd g_kf(nv_);
        g_kf = plant_->CalcGravityGeneralizedForces(*plant_context_);

        Eigen::VectorXd hqqdot = cv_kf - g_kf;
        ////hqqdot构建完毕
        Eigen::VectorXd b_up = S * tau_kfliter - hqqdot;
        // b_up构建完毕

        Eigen::VectorXd qd_detal_x = qd;
        qd_detal_x.array() += detal_x;
        Eigen::VectorXd qv_detal_x = qv;
        qv_detal_x.tail(nv_) = qd_detal_x;
        plant_->SetPositionsAndVelocities(plant_context_.get(), qv_detal_x);

        Eigen::VectorXd cv_kf_detal(nv_);
        plant_->CalcBiasTerm(*plant_context_, &cv_kf_detal);

        Eigen::VectorXd g_kf_detal(nv_);
        g_kf_detal = plant_->CalcGravityGeneralizedForces(*plant_context_);

        Eigen::VectorXd hqqdot_detal = cv_kf_detal - g_kf_detal;
        ////hqqdot构建完毕
        Eigen::VectorXd b_up_detal_x = S * tau_kfliter - hqqdot_detal;
        // b_up构建完毕

        Eigen::MatrixXd qddkf_pd_final;
        Eigen::VectorXd qddkf;
        if (touch_down_state == 0)
        {
            Eigen::Matrix<double, 12, 18> Js_V_wxj_;
            Eigen::MatrixXd Js_V_wxj(18, 12);
            Eigen::MatrixXd A1_0(30, 18);
            A1_0.setZero();
            Js_V_wxj_ << Js_V_wxjl, Js_V_wxjr;
            Js_V_wxj = -Js_V_wxj_.transpose();

            A1_0.block<18, 6>(0, 0) = mass_matrix.block<18, 6>(0, 0);
            A1_0.block<18, 12>(0, 6) = Js_V_wxj.block<18, 12>(0, 0);
            A1_0.block<12, 6>(18, 0) = Js_V_wxj_.block<12, 6>(0, 0);

            Eigen::HouseholderQR<Eigen::Matrix<double, 30, 18>> qrwxj;

            qrwxj.compute(A1_0);

            Eigen::MatrixXd RSUM = qrwxj.matrixQR().triangularView<Eigen::Upper>();

            Eigen::MatrixXd QSUM = qrwxj.householderQ();

            Eigen::MatrixXd Q2 = QSUM.block<30, 12>(0, 18);

            // Q2赋值完成
            Eigen::MatrixXd A2(30, 12);
            A2.setZero();

            A2.topRows(18) = mass_matrix.block<18, 12>(0, 6);
            A2.bottomRows(12) = Js_V_wxj_.block<12, 12>(0, 6);

            // A2赋值完成
            // 构建B///////////

            //  std::cout<<"Js_V_wxj_ \n"<<Js_V_wxj_<<std::endl;
            //  std::cout<<"J_old_ \n"<<J_old<<std::endl;
            if (is_changing_state)
                J_old = Js_V_wxj_;
            Eigen::MatrixXd J_d = (Js_V_wxj_ - J_old) / dt_;
            /// jcd确定
            Eigen::VectorXd b_low = -J_d * qd; // 12x18 18x1

            // b_low构建完毕

            Eigen::VectorXd bkf(30);
            bkf << b_up, b_low; // 18nv+12

            J_old << Js_V_wxj_;

            qddkf = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * bkf;

            for (int i = 0; i < 12; i++)
            {
                if (abs(qddkf(i)) <= 0.05)
                {
                    qddkf(i) = 0;
                }
            }
            /////////////////////////求FFFFF
            Eigen::MatrixXd qddkf_pd = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * dt_;
            // std::cout <<"wxj \n"<<qddkf_pd<< '\n';
            Eigen::VectorXd delta_bkf = bkf - bkf_old;
            bkf_old = bkf;
            Eigen::VectorXd delta_joint_v = q_est_kf_zy - joint_v_old;
            joint_v_old = q_est_kf_zy;
            Eigen::Matrix<double, 30, 12> db_djointdot;

            // f(x+deltax)
            Eigen::VectorXd b_low_detal_x = -J_d * qd_detal_x;
            Eigen::VectorXd bkf_detal_x(30);
            bkf_detal_x << b_up_detal_x, b_low_detal_x; // 18nv+12
            for (int i = 0; i < 30; i++)
            {
                for (int j = 0; j < 12; j++)
                {
                    db_djointdot(i, j) = (bkf_detal_x[i] - bkf[i]) / detal_x;
                }
            }

            qddkf_pd_final = qddkf_pd * db_djointdot; // 12x30 30x12
        }
        else if (touch_down_state == 1) // 左脚触地/////////////////////////////////////////////////////////////////////////////////////////////////////
        {
            Eigen::MatrixXd Js_V_wxjl_t = Js_V_wxjl.transpose();
            Eigen::MatrixXd A1_0(24, 12);
            A1_0.setZero();
            A1_0.block<18, 6>(0, 0) = mass_matrix.block<18, 6>(0, 0);
            A1_0.block<18, 6>(0, 6) = -Js_V_wxjl_t.block<18, 6>(0, 0);
            A1_0.block<6, 6>(18, 0) = Js_V_wxjl.block<6, 6>(0, 0);

            Eigen::HouseholderQR<Eigen::Matrix<double, 24, 12>> qrwxj;

            qrwxj.compute(A1_0);

            Eigen::MatrixXd RSUM = qrwxj.matrixQR().triangularView<Eigen::Upper>();

            Eigen::MatrixXd QSUM = qrwxj.householderQ();

            Eigen::MatrixXd Q2 = QSUM.block<24, 12>(0, 12);

            // Q2赋值完成
            Eigen::MatrixXd A2(24, 12);
            A2.setZero();
            A2.topRows(18) = mass_matrix.block<18, 12>(0, 6);
            A2.bottomRows(6) = Js_V_wxjl.block<6, 12>(0, 6);

            // A2赋值完成
            // 构建B///////////
            if (is_changing_state)
                J_old_1 = Js_V_wxjl;
            Eigen::MatrixXd J_d = (Js_V_wxjl - J_old_1) / dt_;
            /// jcd确定
            // Eigen::VectorXd qd = qv.tail(nv_);
            Eigen::VectorXd b_low = -J_d * qd;
            // b_low构建完毕

            Eigen::VectorXd bkf(24);
            bkf << b_up, b_low;

            J_old_1 << Js_V_wxjl;

            qddkf = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * bkf;

            for (int i = 0; i < 12; i++)
            {
                if (abs(qddkf(i)) <= 0.05)
                {
                    qddkf(i) = 0;
                }
            }

            /////////////////////////求FFFFF
            Eigen::MatrixXd qddkf_pd = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * dt_;

            Eigen::VectorXd delta_bkf = bkf - bkf_old_1;
            bkf_old_1 = bkf;
            Eigen::VectorXd delta_joint_v = q_est_kf_zy - joint_v_old;
            joint_v_old = q_est_kf_zy;

            Eigen::VectorXd b_low_detal_x = -J_d * qd_detal_x;
            Eigen::VectorXd bkf_detal_x(24);
            bkf_detal_x << b_up_detal_x, b_low_detal_x;

            Eigen::Matrix<double, 24, 12> db_djointdot;
            for (int i = 0; i < 24; i++)
            {
                for (int j = 0; j < 12; j++)
                {
                    db_djointdot(i, j) = (bkf_detal_x[i] - bkf[i]) / detal_x;
                }
            }

            qddkf_pd_final = qddkf_pd * db_djointdot;
        }
        else if (touch_down_state == 2) // 右脚触地/////////////////////////////////////////////////////////////////////////////////////////////////
        {
            Eigen::MatrixXd Js_V_wxjr_t = Js_V_wxjr.transpose();
            Eigen::MatrixXd A1_0(24, 12);
            A1_0.setZero();

            A1_0.block<18, 6>(0, 0) = mass_matrix.block<18, 6>(0, 0);
            A1_0.block<18, 6>(0, 6) = -Js_V_wxjr_t.block<18, 6>(0, 0);
            A1_0.block<6, 6>(18, 0) = Js_V_wxjr.block<6, 6>(0, 0);

            Eigen::HouseholderQR<Eigen::Matrix<double, 24, 12>> qrwxj;

            qrwxj.compute(A1_0);

            Eigen::MatrixXd RSUM = qrwxj.matrixQR().triangularView<Eigen::Upper>();

            Eigen::MatrixXd QSUM = qrwxj.householderQ();

            Eigen::MatrixXd Q2 = QSUM.block<24, 12>(0, 12);

            // Q2赋值完成
            Eigen::MatrixXd A2(24, 12);
            A2.setZero();
            A2.block<18, 12>(0, 0) = mass_matrix.block<18, 12>(0, 6);
            A2.block<6, 12>(18, 0) = Js_V_wxjr.block<6, 12>(0, 6);

            // A2赋值完成
            // 构建B///////////
            if (is_changing_state)
                J_old_1 = Js_V_wxjr;
            Eigen::MatrixXd J_d = (Js_V_wxjr - J_old_1) / dt_;
            /// jcd确定
            // Eigen::VectorXd qd = qv.tail(nv_);
            Eigen::VectorXd b_low = -J_d * qd;
            // b_low构建完毕

            Eigen::VectorXd bkf(24);
            bkf << b_up, b_low;

            J_old_1 << Js_V_wxjr;

            qddkf = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * bkf;

            for (int i = 0; i < 12; i++)
            {
                if (abs(qddkf(i)) <= 0.05)
                {
                    qddkf(i) = 0;
                }
            }

            /////////////////////////求FFFFF
            Eigen::MatrixXd qddkf_pd = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * dt_;

            Eigen::VectorXd delta_bkf = bkf - bkf_old_1;
            bkf_old_1 = bkf;
            Eigen::VectorXd delta_joint_v = q_est_kf_zy - joint_v_old;
            joint_v_old = q_est_kf_zy;

            Eigen::VectorXd b_low_detal_x = -J_d * qd_detal_x;
            Eigen::VectorXd bkf_detal_x(24);
            bkf_detal_x << b_up_detal_x, b_low_detal_x;
            Eigen::Matrix<double, 24, 12> db_djointdot;
            for (int i = 0; i < 24; i++)
            {
                for (int j = 0; j < 12; j++)
                {
                    db_djointdot(i, j) = (bkf_detal_x[i] - bkf[i]) / detal_x;
                }
            }

            qddkf_pd_final = qddkf_pd * db_djointdot;
        }
        else if (touch_down_state == 3) // 腾空///////////////////////////////////////////////////////////////////////////////////////////////////////
        {

            Eigen::MatrixXd A1_0(18, 6);
            A1_0.setZero();
            A1_0.block<18, 6>(0, 0) = mass_matrix.block<18, 6>(0, 0);

            Eigen::HouseholderQR<Eigen::Matrix<double, 18, 6>> qrwxj;

            qrwxj.compute(A1_0);

            Eigen::MatrixXd RSUM = qrwxj.matrixQR().triangularView<Eigen::Upper>();

            Eigen::MatrixXd QSUM = qrwxj.householderQ();

            Eigen::MatrixXd Q2(18, 12);

            Q2 = QSUM.block(0, 6, 18, 12);

            // Q2赋值完成
            Eigen::MatrixXd A2(18, 12);

            A2 = mass_matrix.block(0, 6, 18, 12);

            // A2赋值完成
            // 构建B///////////

            Eigen::VectorXd bkf(18);
            bkf << b_up;

            qddkf = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * bkf;

            for (int i = 0; i < 12; i++)
            {
                if (abs(qddkf(i)) <= 0.05)
                {
                    qddkf(i) = 0;
                }
            }

            /////////////////////////求FFFFF
            Eigen::MatrixXd qddkf_pd = (((Q2.transpose() * A2).inverse()) * Q2.transpose()) * dt_;

            Eigen::VectorXd delta_bkf = bkf - bkf_old_2;
            bkf_old_2 = bkf;
            Eigen::VectorXd delta_joint_v = q_est_kf_zy - joint_v_old;
            joint_v_old = q_est_kf_zy;

            Eigen::VectorXd bkf_detal_x(18);
            bkf_detal_x << b_up_detal_x;
            Eigen::Matrix<double, 18, 12> db_djointdot;
            for (int i = 0; i < 18; i++)
            {
                for (int j = 0; j < 12; j++)
                {
                    db_djointdot(i, j) = (bkf_detal_x[i] - bkf[i]) / detal_x;
                }
            }

            qddkf_pd_final = qddkf_pd * db_djointdot;
        }
        Eigen::MatrixXd Iin_F(12, 12);
        Iin_F.setIdentity();

        Eigen::MatrixXd F_FINAL = Iin_F + qddkf_pd_final;
        // std::cout<<"tau: \n"<<F_FINAL<<std::endl;
        // /////////////////////////////////////F处理完毕
        Eigen::VectorXd q_est_kf = q_est_kf_zy + qddkf * dt_;

        Eigen::MatrixXd Q(12, 12);
        Q.setIdentity();
        // Q.diagonal() *= joint_filter_Q_;
        for (int i = 0; i < 12; i++)
        {
            Q(i, i) *= joint_filter_Q_[i];
        }
        // Prediction step
        P = F_FINAL * P_old * (F_FINAL.transpose()) + Q;
        Eigen::MatrixXd R(12, 12);
        R.setIdentity();
        // R.diagonal() *= joint_filter_R_;
        for (int i = 0; i < 12; i++)
        {
            R(i, i) *= joint_filter_R_[i];
        }
        Eigen::MatrixXd Kk = P * ((P + R).inverse());

        q_est_kf_zy = q_est_kf + Kk * (joint_vel - q_est_kf);
        logPublishVector("joint_filter/predict_v", q_est_kf);

        Eigen::MatrixXd III(12, 12);
        III.setIdentity();
        P_old = (III - Kk) * P;

        logPublishVector("joint_filter/_v", q_est_kf_zy);
        logPublishVector("joint_filter/_v_noise", qd.segment(6, 12));
        Eigen::VectorXd tv(1);
        tv << touch_down_state;
        logPublishVector("joint_filter/touch_down_state", tv);

        size_t generalizedCoordinatesNum = rbdState.size() / 2;
        joint_vel = q_est_kf_zy;
        clock_gettime(CLOCK_MONOTONIC, &t1_kfsol);
    }
    void JointFilter::jointPositionFilter(const Eigen::VectorXd &rbdState, Eigen::VectorXd &joint_pos, const Eigen::VectorXd &joint_vel, const Eigen::VectorXd &tau_input, const size_t &mode)
    {
        size_t generalizedCoordinatesNum = rbdState.size() / 2;
        Eigen::VectorXd joint_q = joint_pos;
        // Eigen::VectorXd joint_qd = rbdState.segment(generalizedCoordinatesNum + 6, na_);
        // update step
        q_est_pos_be_ = q_est_pos_af_ + joint_vel * dt_;
        q_est_pos_af_ = q_est_pos_be_ + joint_pos_filter_Kk_ * (joint_q - q_est_pos_be_); // use steady state Kalman Gains

        joint_pos = q_est_pos_af_;

        logPublishVector("joint_filter/_pos", q_est_pos_af_);
        logPublishVector("joint_filter/_pos_noise", joint_q);
    }

}; //
