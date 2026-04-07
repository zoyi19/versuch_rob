#include "humanoid_interface_drake/ankle_solver/arm_tendon_controller.h"

#include <iostream>

namespace lower_leg
{

    using drake::MatrixX;
    using drake::Vector3;
    using drake::VectorX;
    using drake::multibody::JacobianWrtVariable;
    using drake::multibody::MultibodyPlant;

    // const std::string end_link = "l_hand_pitch";
    // const std::string r_link = "l_r_arm_tendon";
    // const std::string l_link = "l_l_arm_tendon";

    // const std::string roll_joint = "l_hand_roll"; // first
    // const std::string pitch_joint = "l_hand_pitch";
    // const std::string r_motor_joint = "l_r_arm_bar";
    // const std::string l_motor_joint = "l_l_arm_bar";

    // const std::string r_motor_actuator = "l_r_arm_bar_motor";
    // const std::string l_motor_actuator = "l_l_arm_bar_motor";

    // const std::string r_frame = "l_r_arm_tendon_socket";
    // const std::string l_frame = "l_l_arm_tendon_socket";
    // const std::string r_end_frame = "l_r_hand_socket";
    // const std::string l_end_frame = "l_l_hand_socket";
    // const std::string r_motor_frame = "l_r_arm_bar_frame"; // 考虑了偏移量
    // const std::string l_motor_frame = "l_l_arm_bar_frame";

    ArmTendonController::ArmTendonController(MultibodyPlant<double> &plant, const std::vector<std::string> &cfg_str)
        : plant_(plant), cfg_(cfg_str)
    {
        plant_context_ = plant_.CreateDefaultContext();
        nv_ = plant_.num_velocities();
        nq_ = plant_.num_positions();
        na_ = plant_.num_actuated_dofs();
        // in_roll_ = plant_.GetJointActuatorByName("a-roll").index();
        // in_pitch_ = plant_.GetJointActuatorByName("a-pitch").index();
        in_l_rocker_arm_ = plant_.GetJointActuatorByName(cfg_.l_motor_actuator).index();
        in_r_rocker_arm_ = plant_.GetJointActuatorByName(cfg_.r_motor_actuator).index();
        const auto &j_l_arm = plant_.GetJointByName(cfg_.l_motor_joint);
        // j_l_arm.Lock(plant_context_.get());
        i_j_l_arm_ = j_l_arm.position_start();
        const auto &j_r_arm = plant_.GetJointByName(cfg_.r_motor_joint);
        // j_r_arm.Lock(plant_context_.get());
        i_j_r_arm_ = j_r_arm.position_start();
        const auto &j_foot_x = plant_.GetJointByName(cfg_.roll_joint);
        i_roll_ = j_foot_x.position_start();
        const auto &j_foot_y = plant_.GetJointByName(cfg_.pitch_joint);
        i_pitch_ = j_foot_y.position_start();
        l_Lkleft_ = plant_.CalcRelativeTransform(*plant_context_,
                                                 plant_.GetBodyByName(cfg_.l_link).body_frame(),
                                                 plant_.GetFrameByName(cfg_.l_frame))
                        .translation()
                        .norm();
        l_Lkright_ = plant_.CalcRelativeTransform(*plant_context_,
                                                  plant_.GetBodyByName(cfg_.r_link).body_frame(),
                                                  plant_.GetFrameByName(cfg_.r_frame))
                         .translation()
                         .norm();
    }

    /**
     * @brief 从roll,pitch 的力Tau到 两电机力的换算
     *
     * @param ankle_q position:[roll, pitch, l_motor, r_motor]
     * @param lower_tau roll,pitch 力矩
     * @param ankle_motor_tau l_motor, r_motor 力矩
     */
    void ArmTendonController::CalcTau(Eigen::Vector4d &ankle_q, Eigen::Vector2d &lower_tau, Eigen::Vector2d &ankle_motor_tau) const
    {
        // std::cout.setf(std::ios::fixed);
        // std::cout.precision(3);
        // Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ",", "\n", " ", ";", "[", "];\n");
        // Eigen::IOFormat VecFmt(Eigen::StreamPrecision, 0, ",", "", " ", ";", "[", "];");
        Eigen::VectorXd q_input(nq_);
        Eigen::VectorXd tau_input(2);
        Eigen::VectorXd tau_out(2);
        q_input << ankle_q.head(3), 0, 0, ankle_q.tail(1), 0, 0;
        tau_input = lower_tau;

        plant_.SetPositions(plant_context_.get(), q_input);
        const auto &world_frame = plant_.world_frame();
        const auto &body_Lkleft = plant_.GetBodyByName(cfg_.l_link);
        const auto &body_Lkright = plant_.GetBodyByName(cfg_.r_link);
        const auto &frame_Flkleft = plant_.GetFrameByName(cfg_.l_end_frame);
        const auto &frame_Flkright = plant_.GetFrameByName(cfg_.r_end_frame);
        const auto &X_LkFlkleft = plant_.CalcRelativeTransform(*plant_context_,
                                                               body_Lkleft.body_frame(), frame_Flkleft);
        const auto &X_LkFlkright = plant_.CalcRelativeTransform(*plant_context_,
                                                                body_Lkright.body_frame(), frame_Flkright);
        const auto &p_LkFlkleft = X_LkFlkleft.translation();
        const auto &p_LkFlkright = X_LkFlkright.translation();

        const auto &X_WFlkleft = frame_Flkleft.CalcPoseInWorld(*plant_context_);
        const auto &X_WFlkright = frame_Flkright.CalcPoseInWorld(*plant_context_);
        const auto &p_WFlkleft = X_WFlkleft.translation();
        const auto &p_WFlkright = X_WFlkright.translation();
        const auto &X_WLkleft = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                           body_Lkleft);
        const auto &X_WLkright = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                            body_Lkright);
        const auto &p_WLkleft = X_WLkleft.translation();
        const auto &p_WLkright = X_WLkright.translation();

        const auto &p_FlkLkleft_W = p_WLkleft - p_WFlkleft; // l_link起点到foot-l_link的位置差
        const auto &p_FlkLkright_W = p_WLkright - p_WFlkright;
        MatrixXd Jv_v_WFlkleft(3, nv_);
        // Jv_v_WFlkleft 表示 foot-l_link 的平移速度与各关节速度之间的关系
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 frame_Flkleft,
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WFlkleft);
        MatrixXd Jv_v_WLkleft(3, nv_);
        // Jv_v_WLkleft 表示 l_link 的平移速度与各关节速度之间的关系
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 body_Lkleft.body_frame(),
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WLkleft);
        MatrixXd Jv_v_WFlkright(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 frame_Flkright,
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WFlkright);
        MatrixXd Jv_v_WLkright(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 body_Lkright.body_frame(),
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WLkright);
        // std::cout << "Jv_v_WFlkleft =...\n" << Jv_v_WFlkleft << "\n";
        // std::cout << "Jv_v_WFlkright =...\n" << Jv_v_WFlkright << "\n";
        // std::cout << "Jv_v_WLkleft =...\n" << Jv_v_WLkleft << "\n";
        // std::cout << "Jv_v_WLkright =...\n" << Jv_v_WLkright << "\n";
        Vector3d phat_LkFlkleft_W = (-p_FlkLkleft_W / l_Lkleft_); // 从l_link起点到foot-l_link的单位向量
        Vector3d phat_LkFlkright_W = (-p_FlkLkright_W / l_Lkright_);
        MatrixXd JakDt_lDt_Lk(2, 2);
        // Jv_v_WFlkleft.col(i_roll_)表示取出雅可比矩阵中 foot-l_link 的平移速度与foot-x的关系的列
        JakDt_lDt_Lk(0, 0) = phat_LkFlkleft_W.dot(Jv_v_WFlkleft.col(i_roll_));  // phat_LkFlkleft_W 点乘Jv_v_WFlkleft的i_roll列,Jv_v_WFlkleft的i_roll列: foot-x -> foot-l_link
        JakDt_lDt_Lk(0, 1) = phat_LkFlkleft_W.dot(Jv_v_WFlkleft.col(i_pitch_)); // foot-y -> foot-l_link 在杠上的投影即关节所需要承受的力矩
        JakDt_lDt_Lk(1, 0) = phat_LkFlkright_W.dot(Jv_v_WFlkright.col(i_roll_));
        JakDt_lDt_Lk(1, 1) = phat_LkFlkright_W.dot(Jv_v_WFlkright.col(i_pitch_));
        MatrixXd JmtDt_lDt_Lk(2, 2);
        JmtDt_lDt_Lk(0, 0) = phat_LkFlkleft_W.dot(Jv_v_WLkleft.col(i_j_l_arm_)); // j-l_rocker_arm -> l_link
        JmtDt_lDt_Lk(0, 1) = phat_LkFlkleft_W.dot(Jv_v_WLkleft.col(i_j_r_arm_));
        JmtDt_lDt_Lk(1, 0) = phat_LkFlkright_W.dot(Jv_v_WLkright.col(i_j_l_arm_));
        JmtDt_lDt_Lk(1, 1) = phat_LkFlkright_W.dot(Jv_v_WLkright.col(i_j_r_arm_));
        // std::cout << "JakDt_lDt_Lk =...\n" << JakDt_lDt_Lk.format(CleanFmt) << "\n";
        // std::cout << "JmtDt_lDt_Lk =...\n" << JmtDt_lDt_Lk.format(CleanFmt) << "\n";
        //   MatrixXd JvMt_vAk = JakDt_lDt_Lk.inverse() * JmtDt_lDt_Lk;
        //   MatrixXd JvMt_vAk = JmtDt_lDt_Lk * JakDt_lDt_Lk.inverse();
        VectorXd tau_ak(2);
        tau_ak << tau_input(i_roll_), tau_input(i_pitch_); // 按roll pitch的顺序取
        // tau_ak = tau_input;
        // tau = J^T * F, 由末端执行器的力计算关节力
        Vector2d tau_mt = JmtDt_lDt_Lk.transpose() * JakDt_lDt_Lk.transpose().inverse() * tau_ak;
        // std::cout << "tau_mt = " << tau_mt.transpose() << std::endl;
        ankle_motor_tau = tau_mt;
        // std::cout << "tau_mt = " << tau_mt.format(VecFmt) << "\n";
        // tau_out(in_roll_) = tau_ak(0);
        // tau_out(in_pitch_) = tau_ak(1);
        // tau_out(in_l_rocker_arm_) = -tau_mt(0);
        // tau_out(in_r_rocker_arm_) = -tau_mt(1);
    }

    void ArmTendonController::CalcLambda(Eigen::VectorXd &ankle_q, Eigen::Vector2d &lower_tau) const
    {
        // std::cout.setf(std::ios::fixed);
        // std::cout.precision(3);
        // Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ",", "\n", " ", ";", "[", "];\n");
        // Eigen::IOFormat VecFmt(Eigen::StreamPrecision, 0, ",", "", " ", ";", "[", "];");

        Eigen::VectorXd q_input(nq_);
        Eigen::VectorXd tau_input(2);
        Eigen::VectorXd lambda(2);
        q_input = ankle_q;
        tau_input = lower_tau;

        plant_.SetPositions(plant_context_.get(), q_input);
        const auto &world_frame = plant_.world_frame();
        const auto &body_Lkleft = plant_.GetBodyByName(cfg_.l_link);
        const auto &body_Lkright = plant_.GetBodyByName(cfg_.r_link);
        const auto &frame_Flkleft = plant_.GetFrameByName(cfg_.l_end_frame);
        const auto &frame_Flkright = plant_.GetFrameByName(cfg_.r_end_frame);
        const auto &X_LkFlkleft = plant_.CalcRelativeTransform(*plant_context_,
                                                               body_Lkleft.body_frame(), frame_Flkleft);
        const auto &X_LkFlkright = plant_.CalcRelativeTransform(*plant_context_,
                                                                body_Lkright.body_frame(), frame_Flkright);
        const auto &p_LkFlkleft = X_LkFlkleft.translation();
        const auto &p_LkFlkright = X_LkFlkright.translation();
        MatrixXd Jv_v_WFlkleft(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 frame_Flkleft,
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WFlkleft);
        MatrixXd Jv_v_WLkleft(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 body_Lkleft.body_frame(),
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WLkleft);
        MatrixXd Jv_v_WFlkright(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 frame_Flkright,
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WFlkright);
        MatrixXd Jv_v_WLkright(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                 JacobianWrtVariable::kV,
                                                 body_Lkright.body_frame(),
                                                 Vector3d::Zero(),
                                                 world_frame,
                                                 world_frame,
                                                 &Jv_v_WLkright);
        // std::cout << "Jv_v_WFlkleft =...\n" << Jv_v_WFlkleft << "\n";
        // std::cout << "Jv_v_WFlkright =...\n" << Jv_v_WFlkright << "\n";
        // std::cout << "Jv_v_WLkleft =...\n" << Jv_v_WLkleft << "\n";
        // std::cout << "Jv_v_WLkright =...\n" << Jv_v_WLkright << "\n";
        Vector3d phat_LkFlkleft = (p_LkFlkleft / l_Lkleft_);
        Vector3d phat_LkFlkright = (p_LkFlkright / l_Lkright_);
        MatrixXd JakDt_lDt_Lk(2, 2);
        JakDt_lDt_Lk(0, 0) = phat_LkFlkleft.dot(Jv_v_WFlkleft.col(i_roll_));
        JakDt_lDt_Lk(0, 1) = phat_LkFlkleft.dot(Jv_v_WFlkleft.col(i_pitch_));
        JakDt_lDt_Lk(1, 0) = phat_LkFlkright.dot(Jv_v_WFlkright.col(i_roll_));
        JakDt_lDt_Lk(1, 1) = phat_LkFlkright.dot(Jv_v_WFlkright.col(i_pitch_));
        MatrixXd JmtDt_lDt_Lk(2, 2);
        JmtDt_lDt_Lk(0, 0) = phat_LkFlkleft.dot(Jv_v_WLkleft.col(i_j_l_arm_));
        JmtDt_lDt_Lk(0, 1) = phat_LkFlkleft.dot(Jv_v_WLkleft.col(i_j_r_arm_));
        JmtDt_lDt_Lk(1, 0) = phat_LkFlkright.dot(Jv_v_WLkright.col(i_j_l_arm_));
        JmtDt_lDt_Lk(1, 1) = phat_LkFlkright.dot(Jv_v_WLkright.col(i_j_r_arm_));
        // std::cout << "JakDt_lDt_Lk =...\n" << JakDt_lDt_Lk.format(CleanFmt) << "\n";
        // std::cout << "JmtDt_lDt_Lk =...\n" << JmtDt_lDt_Lk.format(CleanFmt) << "\n";
        //   MatrixXd JvMt_vAk = JakDt_lDt_Lk.inverse() * JmtDt_lDt_Lk;
        //   MatrixXd JvMt_vAk = JmtDt_lDt_Lk * JakDt_lDt_Lk.inverse();
        VectorXd tau_ak(2);
        tau_ak << tau_input;
        lambda = JakDt_lDt_Lk.transpose().inverse() * tau_ak;
        std::cout << "lambda = " << lambda.transpose() << std::endl;
    }

    std::pair<double, double> ArmTendonController::frc_joint2actuator_left(double pitch, double roll, double llbar, double lrbar, double tau_pitch, double tau_roll)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        double c3 = cos(llbar);
        double s3 = sin(llbar);
        double c4 = cos(lrbar);
        double s4 = sin(lrbar);

        Matrix<double, 6, 2> JAk_p_WEq;
        JAk_p_WEq << 
            -x_lleq * s1 + y_lleq * s2 * c1 + z_lleq * c1 * c2, y_lleq * s1 * c2 - z_lleq * s1 * s2,
            0, -y_lleq * s2 - z_lleq * c2,
            -x_lleq * c1 - y_lleq * s1 * s2 - z_lleq * s1 * c2, y_lleq * c1 * c2 - z_lleq * s2 * c1,
            -x_lreq * s1 + y_lreq * s2 * c1 + z_lreq * c1 * c2, y_lreq * s1 * c2 - z_lreq * s1 * s2,
            0, -y_lreq * s2 - z_lreq * c2,
            -x_lreq * c1 - y_lreq * s1 * s2 - z_lreq * s1 * c2, y_lreq * c1 * c2 - z_lreq * s2 * c1;

        Matrix<double, 6, 2> JAct_p_WTd;
        JAct_p_WTd << 
            0, 0,
            -y_lltd * s3 - z_lltd * c3, 0,
            y_lltd * c3 - z_lltd * s3, 0,
            0, 0,
            0, -y_lrtd * s4 - z_lrtd * c4,
            0, y_lrtd * c4 - z_lrtd * s4;

        Matrix<double, 2, 6> Jxx_l_Td;
        Jxx_l_Td << 
            x_lleq * c1 - x_lltd + y_lleq * s1 * s2 + z_lleq * s1 * c2, 
            y_lleq * c2 - y_lltd * c3 - z_lleq * s2 + z_lltd * s3, 
            -x_lleq * s1 + y_lleq * s2 * c1 - y_lltd * s3 - z_llbar + z_lleq * c1 * c2 - z_lltd * c3 + z_pitch,
            0, 0, 0,
            0, 0, 0,
            x_lreq * c1 - x_lrtd + y_lreq * s1 * s2 + z_lreq * s1 * c2, 
            y_lreq * c2 - y_lrtd * c4 - z_lreq * s2 + z_lrtd * s4, 
            -x_lreq * s1 + y_lreq * s2 * c1 - y_lrtd * s4 - z_lrbar + z_lreq * c1 * c2 - z_lrtd * c4 + z_pitch;

        Matrix<double, 2, 2> JAct_Ak = (Jxx_l_Td * JAk_p_WEq).inverse() * (Jxx_l_Td * JAct_p_WTd);
        Vector2d tau_Ak(tau_pitch, tau_roll);

        // Calculate actuator torques
        Vector2d tau_Act = JAct_Ak.transpose() * tau_Ak;
        double tau_llbar = tau_Act(0);
        double tau_lrbar = tau_Act(1);

        return {tau_llbar, tau_lrbar};
    }
    std::pair<double, double> ArmTendonController::frc_actuator2joint_left(double pitch, double roll, double llbar, double lrbar, double tau_llbar, double tau_lrbar)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        double c3 = cos(llbar);
        double s3 = sin(llbar);
        double c4 = cos(lrbar);
        double s4 = sin(lrbar);

        Matrix<double, 6, 2> JAk_p_WEq;
        JAk_p_WEq << 
            -x_lleq * s1 + y_lleq * s2 * c1 + z_lleq * c1 * c2, y_lleq * s1 * c2 - z_lleq * s1 * s2,
            0, -y_lleq * s2 - z_lleq * c2,
            -x_lleq * c1 - y_lleq * s1 * s2 - z_lleq * s1 * c2, y_lleq * c1 * c2 - z_lleq * s2 * c1,
            -x_lreq * s1 + y_lreq * s2 * c1 + z_lreq * c1 * c2, y_lreq * s1 * c2 - z_lreq * s1 * s2,
            0, -y_lreq * s2 - z_lreq * c2,
            -x_lreq * c1 - y_lreq * s1 * s2 - z_lreq * s1 * c2, y_lreq * c1 * c2 - z_lreq * s2 * c1;

        Matrix<double, 6, 2> JAct_p_WTd;
        JAct_p_WTd << 
            0, 0,
            -y_lltd * s3 - z_lltd * c3, 0,
            y_lltd * c3 - z_lltd * s3, 0,
            0, 0,
            0, -y_lrtd * s4 - z_lrtd * c4,
            0, y_lrtd * c4 - z_lrtd * s4;

        Matrix<double, 2, 6> Jxx_l_Td;
        Jxx_l_Td << 
            x_lleq * c1 - x_lltd + y_lleq * s1 * s2 + z_lleq * s1 * c2, 
            y_lleq * c2 - y_lltd * c3 - z_lleq * s2 + z_lltd * s3, 
            -x_lleq * s1 + y_lleq * s2 * c1 - y_lltd * s3 - z_llbar + z_lleq * c1 * c2 - z_lltd * c3 + z_pitch,
            0, 0, 0,
            0, 0, 0,
            x_lreq * c1 - x_lrtd + y_lreq * s1 * s2 + z_lreq * s1 * c2, 
            y_lreq * c2 - y_lrtd * c4 - z_lreq * s2 + z_lrtd * s4, 
            -x_lreq * s1 + y_lreq * s2 * c1 - y_lrtd * s4 - z_lrbar + z_lreq * c1 * c2 - z_lrtd * c4 + z_pitch;

        Matrix<double, 2, 2> JAct_Ak = (Jxx_l_Td * JAct_p_WTd).inverse() * (Jxx_l_Td * JAk_p_WEq);

        Vector2d tau_Act(tau_llbar, tau_lrbar);

        // Calculate joint torques
        Vector2d tau_Ak = JAct_Ak.transpose() * tau_Act;
        double tau_pitch = tau_Ak(0);
        double tau_roll = tau_Ak(1);

        return {tau_pitch, tau_roll};
    }

    std::pair<double, double> ArmTendonController::frc_joint2actuator_right(double pitch, double roll, double llbar, double lrbar, double tau_pitch, double tau_roll)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        double c3 = cos(llbar);
        double s3 = sin(llbar);
        double c4 = cos(lrbar);
        double s4 = sin(lrbar);

        Matrix<double, 6, 2> JAk_p_WEq;
        JAk_p_WEq << 
            -x_rleq * s1 + y_rleq * s2 * c1 + z_rleq * c1 * c2, y_rleq * s1 * c2 - z_rleq * s1 * s2,
            0, -y_rleq * s2 - z_rleq * c2,
            -x_rleq * c1 - y_rleq * s1 * s2 - z_rleq * s1 * c2, y_rleq * c1 * c2 - z_rleq * s2 * c1,
            -x_rreq * s1 + y_rreq * s2 * c1 + z_rreq * c1 * c2, y_rreq * s1 * c2 - z_rreq * s1 * s2,
            0, -y_rreq * s2 - z_rreq * c2,
            -x_rreq * c1 - y_rreq * s1 * s2 - z_rreq * s1 * c2, y_rreq * c1 * c2 - z_rreq * s2 * c1;

        Matrix<double, 6, 2> JAct_p_WTd;
        JAct_p_WTd << 
            0, 0,
            -y_rltd * s3 - z_rltd * c3, 0,
            y_rltd * c3 - z_rltd * s3, 0,
            0, 0,
            0, -y_rrtd * s4 - z_rrtd * c4,
            0, y_rrtd * c4 - z_rrtd * s4;

        Matrix<double, 2, 6> Jxx_l_Td;
        Jxx_l_Td << 
            x_rleq * c1 - x_rltd + y_rleq * s1 * s2 + z_rleq * s1 * c2, 
            y_rleq * c2 - y_rltd * c3 - z_rleq * s2 + z_rltd * s3, 
            -x_rleq * s1 + y_rleq * s2 * c1 - y_rltd * s3 - z_rlbar + z_rleq * c1 * c2 - z_rltd * c3 + z_pitch,
            0, 0, 0,
            0, 0, 0,
            x_rreq * c1 - x_rrtd + y_rreq * s1 * s2 + z_rreq * s1 * c2, 
            y_rreq * c2 - y_rrtd * c4 - z_rreq * s2 + z_rrtd * s4, 
            -x_rreq * s1 + y_rreq * s2 * c1 - y_rrtd * s4 - z_rrbar + z_rreq * c1 * c2 - z_rrtd * c4 + z_pitch;

        Matrix<double, 2, 2> JAct_Ak = (Jxx_l_Td * JAk_p_WEq).inverse() * (Jxx_l_Td * JAct_p_WTd);
        Vector2d tau_Ak(tau_pitch, tau_roll);

        // Calculate actuator torques
        Vector2d tau_Act = JAct_Ak.transpose() * tau_Ak;
        double tau_llbar = tau_Act(0);
        double tau_lrbar = tau_Act(1);

        return {tau_llbar, tau_lrbar};
    }

    std::pair<double, double> ArmTendonController::frc_actuator2joint_right(double pitch, double roll, double llbar, double lrbar, double tau_llbar, double tau_lrbar)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        double c3 = cos(llbar);
        double s3 = sin(llbar);
        double c4 = cos(lrbar);
        double s4 = sin(lrbar);

        Matrix<double, 6, 2> JAk_p_WEq;
        JAk_p_WEq << 
            -x_rleq * s1 + y_rleq * s2 * c1 + z_rleq * c1 * c2, y_rleq * s1 * c2 - z_rleq * s1 * s2,
            0, -y_rleq * s2 - z_rleq * c2,
            -x_rleq * c1 - y_rleq * s1 * s2 - z_rleq * s1 * c2, y_rleq * c1 * c2 - z_rleq * s2 * c1,
            -x_rreq * s1 + y_rreq * s2 * c1 + z_rreq * c1 * c2, y_rreq * s1 * c2 - z_rreq * s1 * s2,
            0, -y_rreq * s2 - z_rreq * c2,
            -x_rreq * c1 - y_rreq * s1 * s2 - z_rreq * s1 * c2, y_rreq * c1 * c2 - z_rreq * s2 * c1;

        Matrix<double, 6, 2> JAct_p_WTd;
        JAct_p_WTd << 
            0, 0,
            -y_rltd * s3 - z_rltd * c3, 0,
            y_rltd * c3 - z_rltd * s3, 0,
            0, 0,
            0, -y_rrtd * s4 - z_rrtd * c4,
            0, y_rrtd * c4 - z_rrtd * s4;

        Matrix<double, 2, 6> Jxx_l_Td;
        Jxx_l_Td << 
            x_rleq * c1 - x_rltd + y_rleq * s1 * s2 + z_rleq * s1 * c2, 
            y_rleq * c2 - y_rltd * c3 - z_rleq * s2 + z_rltd * s3, 
            -x_rleq * s1 + y_rleq * s2 * c1 - y_rltd * s3 - z_rlbar + z_rleq * c1 * c2 - z_rltd * c3 + z_pitch,
            0, 0, 0,
            0, 0, 0,
            x_rreq * c1 - x_rrtd + y_rreq * s1 * s2 + z_rreq * s1 * c2, 
            y_rreq * c2 - y_rrtd * c4 - z_rreq * s2 + z_rrtd * s4, 
            -x_rreq * s1 + y_rreq * s2 * c1 - y_rrtd * s4 - z_rrbar + z_rreq * c1 * c2 - z_rrtd * c4 + z_pitch;

        Matrix<double, 2, 2> JAct_Ak = (Jxx_l_Td * JAct_p_WTd).inverse() * (Jxx_l_Td * JAk_p_WEq);

        Vector2d tau_Act(tau_llbar, tau_lrbar);

        // Calculate joint torques
        Vector2d tau_Ak = JAct_Ak.transpose() * tau_Act;
        double tau_pitch = tau_Ak(0);
        double tau_roll = tau_Ak(1);

        return {tau_pitch, tau_roll};
    }

    double ArmTendonController::frc_joint2actuator_knee(double knee, double bar, double tau_knee)
    {
        // 使用向量法，避开使用 acos，使用点 B 作为原点
        double x_B = 0, y_B = 0;
        double x_A = x_B - AB * sin(bar + offsetBar);
        double y_A = y_B + AB * cos(bar + offsetBar);
        double x_C = 0, y_C = -BC;
        double x_D = x_C + CD * sin(knee + offsetKnee);
        double y_D = y_C + CD * cos(knee + offsetKnee);

        // 使用 Eigen 库定义向量
        Eigen::Vector2d vec_BA(x_A - x_B, y_A - y_B);
        Eigen::Vector2d vec_BA_cross(-vec_BA.y(), vec_BA.x());  // BA 的垂线方向
        Eigen::Vector2d vec_CD(x_D - x_C, y_D - y_C);
        Eigen::Vector2d vec_CD_cross(vec_CD.y(), -vec_CD.x());  // CD 的垂线方向
        Eigen::Vector2d vec_AD(x_D - x_A, y_D - y_A);

        // 计算模长
        double norm_BA_cross = vec_BA_cross.norm();
        double norm_AD = vec_AD.norm();
        double norm_CD_cross = vec_CD_cross.norm();

        // 力分量推导
        double F_knee = tau_knee / CD;
        double F_knee_AD = F_knee / (vec_AD.dot(vec_CD_cross) / (norm_AD * norm_CD_cross));
        double F_bar = F_knee_AD * (vec_BA_cross.dot(vec_AD) / (norm_BA_cross * norm_AD));
        double tau_bar = F_bar * AB;

        return tau_bar;
    }

} // namespace lower_leg
