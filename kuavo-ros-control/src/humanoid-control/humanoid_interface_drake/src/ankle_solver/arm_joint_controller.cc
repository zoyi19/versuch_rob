#include "humanoid_interface_drake/ankle_solver/arm_joint_controller.h"

#include <iostream>
#include <drake/math/wrap_to.h>

namespace lower_leg
{
    using drake::MatrixX;
    using drake::Vector3;
    using drake::VectorX;
    using drake::math::wrap_to;
    using drake::multibody::JacobianWrtVariable;
    using drake::multibody::MultibodyPlant;
    using drake::systems::BasicVector;

    // const std::string end_link = "l_hand_pitch";
    // const std::string l_link = "l_l_arm_tendon";
    // const std::string r_link = "l_r_arm_tendon";
    // const std::string l_bar_link = "l_l_arm_bar";
    // const std::string r_bar_link = "l_r_arm_bar";

    // const std::string roll_joint = "l_hand_roll"; // first
    // const std::string pitch_joint = "l_hand_pitch";
    // const std::string l_link_joint = "l_l_arm_tendon";
    // const std::string r_link_joint = "l_r_arm_tendon";
    // const std::string l_motor_joint = "l_l_arm_bar";
    // const std::string r_motor_joint = "l_r_arm_bar";

    // const std::string r_end_frame = "l_r_hand_socket";
    // const std::string l_end_frame = "l_l_hand_socket";
    // const std::string r_frame = "l_r_arm_tendon_socket";
    // const std::string l_frame = "l_l_arm_tendon_socket";
    // const std::string r_motor_frame = "l_r_arm_bar_frame"; // 考虑了偏移量
    // const std::string l_motor_frame = "l_l_arm_bar_frame";
    double ArmJointController::calProjectAngleXZ(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) const
    {
        Eigen::Vector2d vector1_xz(a.x() - b.x(), a.z() - b.z());
        Eigen::Vector2d vector2_xz(c.x() - b.x(), c.z() - b.z());
        double norm = vector1_xz.norm() * vector2_xz.norm();
        if (norm < 1e-6)
            return 0;
        double theta_xz = std::acos(vector1_xz.dot(vector2_xz) / norm);
        return theta_xz;
    }
    // double calProjectAngleYZ(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
    // {
    //     Eigen::Vector2d vector1_yz(a.y() - b.y(), a.z() - b.z());
    //     Eigen::Vector2d vector2_yz(c.y() - b.y(), c.z() - b.z());
    //     double norm = vector1_yz.norm() * vector2_yz.norm();
    //     if (norm < 1e-6)
    //         return 0;
    //     double theta_yz = std::acos(vector1_yz.dot(vector2_yz) / norm);
    //     return theta_yz;
    // }
    ArmJointController::ArmJointController(multibody::MultibodyPlant<double> &plant, const std::vector<std::string> &cfg_str)
        : plant_(plant), cfg_(cfg_str)
    {
        plant_context_ = plant_.CreateDefaultContext();
        nq_ = plant_.num_positions();
        nv_ = plant_.num_velocities();

        const auto &j_l_arm = plant_.GetJointByName(cfg_.l_motor_joint);
        i_j_l_arm_ = j_l_arm.position_start();
        const auto &j_r_arm = plant_.GetJointByName(cfg_.r_motor_joint);
        i_j_r_arm_ = j_r_arm.position_start();
        const auto &j_foot_x = plant_.GetJointByName(cfg_.roll_joint); // TODO: check
        i_roll_ = j_foot_x.position_start();
        const auto &j_foot_y = plant_.GetJointByName(cfg_.pitch_joint);
        i_pitch_ = j_foot_y.position_start();
        const auto &j_l_link = plant_.GetJointByName(cfg_.l_link_joint);
        i_j_l_link = j_l_link.position_start();
        const auto &j_r_link = plant_.GetJointByName(cfg_.r_link_joint);
        i_j_r_link = j_r_link.position_start();
        l_Lkleft_ = plant_.CalcRelativeTransform(*plant_context_,
                                                 plant_.GetBodyByName(cfg_.l_link).body_frame(),
                                                 plant_.GetFrameByName(cfg_.l_frame))
                        .translation()
                        .norm(); // link长度
        l_Lkright_ = plant_.CalcRelativeTransform(*plant_context_,
                                                  plant_.GetBodyByName(cfg_.r_link).body_frame(),
                                                  plant_.GetFrameByName(cfg_.r_frame))
                         .translation()
                         .norm();
        const auto &p_ArmLkleft = plant_.CalcRelativeTransform(*plant_context_,
                                                               plant_.GetFrameByName(cfg_.l_motor_frame),
                                                               plant_.GetBodyByName(cfg_.l_link).body_frame())
                                      .translation();
        l_Armleft_ = p_ArmLkleft.norm();
        qO_Armleft_ = atan2(-p_ArmLkleft(2), p_ArmLkleft(0)); // 电机输出杆的初始位置
        const auto &p_ArmLkright = plant_.CalcRelativeTransform(*plant_context_,
                                                                plant_.GetFrameByName(cfg_.r_motor_frame),
                                                                plant_.GetBodyByName(cfg_.r_link).body_frame())
                                       .translation();
        l_Armright_ = p_ArmLkright.norm();
        qO_Armright_ = atan2(-p_ArmLkright(2), p_ArmLkright(0));
        const auto &p_WLkleft = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                           plant_.GetBodyByName(cfg_.l_link))
                                    .translation();
        // const auto &p_WFlkleft = plant_.GetFrameByName("foot-l_link").CalcPoseInWorld(*plant_context_).translation();
        // const auto &p_WArmleft = plant_.GetFrameByName("f-l_rocker_arm").CalcPoseInWorld(*plant_context_).translation();
        Eigen::Vector3d p_WFlkleft = plant_.CalcRelativeTransform(*plant_context_, plant_.world_frame(), plant_.GetFrameByName(cfg_.l_end_frame)).translation();
        Eigen::Vector3d p_WArmleft = plant_.CalcRelativeTransform(*plant_context_, plant_.world_frame(), plant_.GetFrameByName(cfg_.l_motor_frame)).translation();
        const auto pos = plant_.GetPositions(*plant_context_);
        init_l_link_xz = calProjectAngleXZ(p_WFlkleft, p_WLkleft, p_WArmleft);
        const auto &p_WLkright = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                            plant_.GetBodyByName(cfg_.r_link))
                                     .translation();
        // const auto &p_WFlkright = plant_.GetFrameByName("foot-r_link").CalcPoseInWorld(*plant_context_).translation();
        // const auto &p_WArmright = plant_.GetFrameByName("f-r_rocker_arm").CalcPoseInWorld(*plant_context_).translation();
        Eigen::Vector3d p_WFlkright = plant_.CalcRelativeTransform(*plant_context_, plant_.world_frame(), plant_.GetFrameByName(cfg_.r_end_frame)).translation();
        Eigen::Vector3d p_WArmright = plant_.CalcRelativeTransform(*plant_context_, plant_.world_frame(), plant_.GetFrameByName(cfg_.r_motor_frame)).translation();

        init_r_link_xz = calProjectAngleXZ(p_WFlkright, p_WLkright, p_WArmright);

        // Eigen::Vector3d p_llink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
        //                                                               plant_.GetFrameByName("f-l_rocker_arm"),
        //                                                               plant_.GetBodyByName("l_link").body_frame())
        //                                      .translation();
        Eigen::Vector3d p_f_llink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
                                                                        plant_.GetFrameByName(cfg_.l_motor_frame),
                                                                        plant_.GetFrameByName(cfg_.l_end_frame))
                                               .translation();
        init_l_link_yz = atan2(p_f_llink_in_arm(1), p_f_llink_in_arm(2));

        // Eigen::Vector3d p_rlink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
        //                                                               plant_.GetFrameByName("f-r_rocker_arm"),
        //                                                               j_r_link.frame_on_parent())
        //                                      .translation();
        Eigen::Vector3d p_f_rlink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
                                                                        plant_.GetFrameByName(cfg_.r_motor_frame),
                                                                        plant_.GetFrameByName(cfg_.r_end_frame))
                                               .translation();
        init_r_link_yz = atan2(p_f_rlink_in_arm(1), p_f_rlink_in_arm(2));

        // for (drake::multibody::JointIndex i{0}; i < plant_.num_joints(); i++)
        // {
        //     const auto &joint = plant_.get_joint(i);
        //     std::cout << "cont joint(" << i << "):" << joint.name() << ", ";
        //     std::cout << "q start: " << joint.position_start() << ", ";
        //     std::cout << "v start: " << joint.velocity_start() << ";";
        //     std::cout << "type: " << joint.type_name() << "\n";
        //     std::cout << std::endl;
        // }

        /********************** 大迭代版本初始化数据 *******************************/
        l_llbar = sqrt(y_lltd*y_lltd + z_lltd*z_lltd);
        l_lrbar = sqrt(y_lrtd*y_lrtd + z_lrtd*z_lrtd);
        l_rlbar = sqrt(y_rltd*y_rltd + z_rltd*z_rltd);
        l_rrbar = sqrt(y_rrtd*y_rrtd + z_rrtd*z_rrtd);
    }

    /**
     * @brief foot_qv => ankle_motor_qv(all)
     *
     * @param lower_qv foot_qv foot_roll,foot_pitch
     * @param ankle_motor_qv ankle_motor_qv qv
     */
    void ArmJointController::CalcArmStateQv(Eigen::Vector4d &lower_qv, Eigen::VectorXd &ankle_motor_qv) const
    {
        Eigen::VectorXd q_out(2);
        Eigen::VectorXd v_out(2);
        Eigen::VectorXd q_input(nq_);
        Eigen::VectorXd v_input(nv_);
        q_input << lower_qv.head(2), 0, 0, 0, 0, 0, 0;
        v_input << lower_qv.tail(2), 0, 0, 0, 0, 0, 0;
        q_input(i_j_l_arm_) = 0.;
        q_input(i_j_r_arm_) = 0.;

        plant_.SetPositions(plant_context_.get(), q_input);
        const auto &world_frame = plant_.world_frame();
        const auto &frame_Armleft = plant_.GetFrameByName(cfg_.l_motor_frame);  // 左侧电机frame
        const auto &frame_Armright = plant_.GetFrameByName(cfg_.r_motor_frame); // 右侧电机frame
        const auto &frame_Flkleft = plant_.GetFrameByName(cfg_.l_end_frame);    // 在foot上的左连杆和脚连接点的frame
        const auto &frame_Flkright = plant_.GetFrameByName(cfg_.r_end_frame);

        const auto &X_WArmleft = frame_Armleft.CalcPoseInWorld(*plant_context_);
        const auto &X_WArmright = frame_Armright.CalcPoseInWorld(*plant_context_);
        const auto &p_WArmleft = X_WArmleft.translation();
        const auto &p_WArmright = X_WArmright.translation();
        const auto &X_WFlkleft = frame_Flkleft.CalcPoseInWorld(*plant_context_);
        const auto &X_WFlkright = frame_Flkright.CalcPoseInWorld(*plant_context_);
        const auto &p_WFlkleft = X_WFlkleft.translation();
        const auto &p_WFlkright = X_WFlkright.translation();

        const auto &p_FlkArmleft_W = p_WArmleft - p_WFlkleft;
        const auto &p_FlkArmright_W = p_WArmright - p_WFlkright;

        // 计算电机端关节角度
        double qleft_avg = atan2(p_FlkArmleft_W(2), -p_FlkArmleft_W(0));
        double qright_avg = atan2(p_FlkArmright_W(2), -p_FlkArmright_W(0));

        double a, b, c2;

        a = l_Armleft_; // 电机输出杆长
        b = sqrt(p_FlkArmleft_W(0) * p_FlkArmleft_W(0) + p_FlkArmleft_W(2) * p_FlkArmleft_W(2));
        c2 = l_Lkleft_ * l_Lkleft_ - p_FlkArmleft_W(1) * p_FlkArmleft_W(1);
        double qleft_delta = acos((a * a + b * b - c2) / (2 * a * b)); // 得到的是在电机平面上l_Armleft_ 和 电机与脚部球关节连线投影 的夹角
        double tmp = qleft_avg + qleft_delta - qO_Armleft_;
        q_out(0) = wrap_to(tmp, -M_PI, M_PI);
        q_input(i_j_l_arm_) = q_out(0);

        a = l_Armright_;
        b = sqrt(p_FlkArmright_W(0) * p_FlkArmright_W(0) + p_FlkArmright_W(2) * p_FlkArmright_W(2));
        c2 = l_Lkright_ * l_Lkright_ - p_FlkArmright_W(1) * p_FlkArmright_W(1);
        // std::cout << "(a * a + b * b - c2) / (2 * a * b):"<<(a * a + b * b - c2) / (2 * a * b)<<std::endl;
        double qright_delta = acos((a * a + b * b - c2) / (2 * a * b));
        tmp = qright_avg + qright_delta - qO_Armright_;
        q_out(1) = wrap_to(tmp, -M_PI, M_PI);
        q_input(i_j_r_arm_) = q_out(1);
        // std::cout << "q_input:"<<q_input<<std::endl;

        plant_.SetPositions(plant_context_.get(), q_input);

        const auto &body_Lkleft = plant_.GetBodyByName(cfg_.l_link);
        const auto &body_Lkright = plant_.GetBodyByName(cfg_.r_link);

        const auto &X_WLkleft = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                           body_Lkleft);
        const auto &X_WLkright = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                            body_Lkright);
        const auto &p_WLkleft = X_WLkleft.translation();
        const auto &p_WLkright = X_WLkright.translation();

        // 求球型关节的夹角
        // Eigen::Vector3d p_llink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
        //                                                               frame_Armleft,
        //                                                               plant_.GetBodyByName("l_link").body_frame())
        //                                      .translation();
        Eigen::Vector3d p_f_llink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
                                                                        frame_Armleft,
                                                                        frame_Flkleft)
                                               .translation();
        Eigen::Vector3d p_f_rlink_in_arm = plant_.CalcRelativeTransform(*plant_context_,
                                                                        frame_Armright,
                                                                        frame_Flkright)
                                               .translation();

        double left_theta_xz = calProjectAngleXZ(p_WFlkleft, p_WLkleft, p_WArmleft);

        // double left_theta_yz = calProjectAngleYZ(p_WFlkleft, p_WLkleft, p_WArmleft);
        double left_theta_yz = atan2(p_f_llink_in_arm(1), p_f_llink_in_arm(2));
        left_theta_xz = wrap_to(left_theta_xz - init_l_link_xz, -M_PI, M_PI);
        left_theta_yz = wrap_to(left_theta_yz - init_l_link_yz, -M_PI, M_PI);

        double right_theta_xz = calProjectAngleXZ(p_WFlkright, p_WLkright, p_WArmright);
        // double right_theta_yz = calProjectAngleYZ(p_WFlkright, p_WLkright, p_WArmright);
        double right_theta_yz = atan2(p_f_rlink_in_arm(1), p_f_rlink_in_arm(2));
        right_theta_xz = wrap_to(right_theta_xz - init_r_link_xz, -M_PI, M_PI);
        right_theta_yz = wrap_to(right_theta_yz - init_r_link_yz, -M_PI, M_PI);
        q_input.segment(i_j_l_link, 2) << left_theta_yz, left_theta_xz;   // 3\4
        q_input.segment(i_j_r_link, 2) << right_theta_yz, right_theta_xz; // 6\7 绕着x,y旋转的方向
        plant_.SetPositions(plant_context_.get(), q_input);

        const auto &p_FlkLkleft_W = p_WLkleft - p_WFlkleft; // l_link起点到foot-l_link的位置差
        const auto &p_FlkLkright_W = p_WLkright - p_WFlkright;
        Eigen::MatrixXd Jv_v_WFlkleft(3, nv_);
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
        Vector3d phat_LkFlkleft_W = (-p_FlkLkleft_W / l_Lkleft_);
        Vector3d phat_LkFlkright_W = (-p_FlkLkright_W / l_Lkright_);
        MatrixXd JakDt_lDt_Lk(2, 2);
        JakDt_lDt_Lk(0, 0) = phat_LkFlkleft_W.dot(Jv_v_WFlkleft.col(i_roll_));
        JakDt_lDt_Lk(0, 1) = phat_LkFlkleft_W.dot(Jv_v_WFlkleft.col(i_pitch_));
        JakDt_lDt_Lk(1, 0) = phat_LkFlkright_W.dot(Jv_v_WFlkright.col(i_roll_));
        JakDt_lDt_Lk(1, 1) = phat_LkFlkright_W.dot(Jv_v_WFlkright.col(i_pitch_));
        MatrixXd JmtDt_lDt_Lk(2, 2);
        JmtDt_lDt_Lk(0, 0) = phat_LkFlkleft_W.dot(Jv_v_WLkleft.col(i_j_l_arm_));
        JmtDt_lDt_Lk(0, 1) = phat_LkFlkleft_W.dot(Jv_v_WLkleft.col(i_j_r_arm_));
        JmtDt_lDt_Lk(1, 0) = phat_LkFlkright_W.dot(Jv_v_WLkright.col(i_j_l_arm_));
        JmtDt_lDt_Lk(1, 1) = phat_LkFlkright_W.dot(Jv_v_WLkright.col(i_j_r_arm_));

        VectorXd v_ak(2);
        v_ak << v_input(i_roll_), v_input(i_pitch_);
        VectorXd v_mt(2);
        // JmtDt_lDt_Lk 应该是对角矩阵
        v_mt = JmtDt_lDt_Lk.inverse() * JakDt_lDt_Lk * v_ak;

        Eigen::VectorXd output_q(nq_), output_v(nv_);
        output_q = plant_.GetPositions(*plant_context_);
        output_v = plant_.GetVelocities(*plant_context_);
        // std::cout << "output_q:"<<output_q.transpose()<<std::endl;
        // std::cout << "output_v:"<<output_v.transpose()<<std::endl;

        output_v(i_j_l_arm_) = v_mt(0);
        output_v(i_j_r_arm_) = v_mt(1);
        plant_.SetVelocities(plant_context_.get(), output_v);

        ankle_motor_qv << output_q, output_v;
    }
    /**
     * @brief foot_qv => ankle_motor_qv
     *
     * @param lower_qv foot_qv foot_roll,foot_pitch
     * @param ankle_motor_qv ankle_motor_qv l_motor,r_motor
     */
    void ArmJointController::CalcArmState(Eigen::Vector4d &lower_qv, Vector4d &ankle_motor_qv) const
    {
        Eigen::VectorXd motor_qv(nq_ + nv_);
        CalcArmStateQv(lower_qv, motor_qv);
        ankle_motor_qv << motor_qv(i_j_l_arm_), motor_qv(i_j_r_arm_), motor_qv(nq_ + i_j_l_arm_), motor_qv(nq_ + i_j_r_arm_);
        // output_q(i_j_l_arm_), output_q(i_j_r_arm_), output_v(i_j_l_arm_), output_v(i_j_r_arm_);
    }

    std::pair<double, double> ArmJointController::pos_joint2actuator_left(double pitch, double roll)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        
        double y_LlbarEq_W = y_lleq * c2 - z_lleq * s2;
        double z_LlbarEq_W = -x_lleq * s1 + y_lleq * s2 * c1 - z_llbar + z_lleq * c1 * c2 + z_pitch;
        double y_LrbarEq_W = y_lreq * c2 - z_lreq * s2;
        double z_LrbarEq_W = -x_lreq * s1 + y_lreq * s2 * c1 - z_lrbar + z_lreq * c1 * c2 + z_pitch;
        double x_LltdEq_W = x_lleq * c1 - x_lltd + y_lleq * s1 * s2 + z_lleq * s1 * c2;
        double x_LrtdEq_W = x_lreq * c1 - x_lrtd + y_lreq * s1 * s2 + z_lreq * s1 * c2;
        
        double b_ll = sqrt(y_LlbarEq_W * y_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
        double a_ll = l_llbar;
        double c_ll = sqrt(l_lltd * l_lltd - x_LltdEq_W * x_LltdEq_W);
        
        double theta_llbar = atan2(z_LlbarEq_W, y_LlbarEq_W) + safe_acos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2 * a_ll * b_ll)) - atan2(z_lltd, y_lltd);
        
        double b_lr = sqrt(y_LrbarEq_W * y_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
        double a_lr = l_lrbar;
        double c_lr = sqrt(l_lrtd * l_lrtd - x_LrtdEq_W * x_LrtdEq_W);
        
        double theta_lrbar = atan2(z_LrbarEq_W, y_LrbarEq_W) - safe_acos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2 * a_lr * b_lr)) - atan2(z_lrtd, y_lrtd);
        
        return {theta_llbar, theta_lrbar};
    }
    std::pair<double, double> ArmJointController::vel_joint2actuator_left(double pitch, double roll, double llbar, double lrbar, double pitchDt, double rollDt)
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
            -x_lleq * s1 + y_lleq * s2 * c1 + z_lleq * c1 * c2,  y_lleq * s1 * c2 - z_lleq * s1 * s2,
            0,  -y_lleq * s2 - z_lleq * c2,
            -x_lleq * c1 - y_lleq * s1 * s2 - z_lleq * s1 * c2,  y_lleq * c1 * c2 - z_lleq * s2 * c1,
            -x_lreq * s1 + y_lreq * s2 * c1 + z_lreq * c1 * c2,  y_lreq * s1 * c2 - z_lreq * s1 * s2,
            0,  -y_lreq * s2 - z_lreq * c2,
            -x_lreq * c1 - y_lreq * s1 * s2 - z_lreq * s1 * c2,  y_lreq * c1 * c2 - z_lreq * s2 * c1;

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

        Vector2d AkDt(pitchDt, rollDt);
        
        // Calculate actuator velocities
        Vector2d ActDt = (Jxx_l_Td * JAct_p_WTd).ldlt().solve(Jxx_l_Td * JAk_p_WEq * AkDt);
        double theta_llbarDt = ActDt(0);
        double theta_lrbarDt = ActDt(1);

        return {theta_llbarDt, theta_lrbarDt};
    }

    std::pair<double, double> ArmJointController::pos_joint2actuator_right(double pitch, double roll)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        
        double y_LlbarEq_W = y_rleq * c2 - z_rleq * s2;
        double z_LlbarEq_W = -x_rleq * s1 + y_rleq * s2 * c1 - z_rlbar + z_rleq * c1 * c2 + z_pitch;
        double y_LrbarEq_W = y_rreq * c2 - z_rreq * s2;
        double z_LrbarEq_W = -x_rreq * s1 + y_rreq * s2 * c1 - z_rrbar + z_rreq * c1 * c2 + z_pitch;
        double x_LltdEq_W = x_rleq * c1 - x_rltd + y_rleq * s1 * s2 + z_rleq * s1 * c2;
        double x_LrtdEq_W = x_rreq * c1 - x_rrtd + y_rreq * s1 * s2 + z_rreq * s1 * c2;
        
        double b_ll = sqrt(y_LlbarEq_W * y_LlbarEq_W + z_LlbarEq_W * z_LlbarEq_W);
        double a_ll = l_rlbar;
        double c_ll = sqrt(l_rltd * l_rltd - x_LltdEq_W * x_LltdEq_W);
        
        double theta_llbar = atan2(z_LlbarEq_W, y_LlbarEq_W) + safe_acos((a_ll * a_ll + b_ll * b_ll - c_ll * c_ll) / (2 * a_ll * b_ll)) - atan2(z_rltd, y_rltd);
        
        double b_lr = sqrt(y_LrbarEq_W * y_LrbarEq_W + z_LrbarEq_W * z_LrbarEq_W);
        double a_lr = l_rrbar;
        double c_lr = sqrt(l_rrtd * l_rrtd - x_LrtdEq_W * x_LrtdEq_W);
        
        double theta_lrbar = atan2(z_LrbarEq_W, y_LrbarEq_W) - safe_acos((a_lr * a_lr + b_lr * b_lr - c_lr * c_lr) / (2 * a_lr * b_lr)) - atan2(z_rrtd, y_rrtd);
        
        return {theta_llbar, theta_lrbar};
    }
    std::pair<double, double> ArmJointController::vel_joint2actuator_right(double pitch, double roll, double llbar, double lrbar, double pitchDt, double rollDt)
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
            -x_rleq * s1 + y_rleq * s2 * c1 + z_rleq * c1 * c2,  y_rleq * s1 * c2 - z_rleq * s1 * s2,
            0,  -y_rleq * s2 - z_rleq * c2,
            -x_rleq * c1 - y_rleq * s1 * s2 - z_rleq * s1 * c2,  y_rleq * c1 * c2 - z_rleq * s2 * c1,
            -x_rreq * s1 + y_rreq * s2 * c1 + z_rreq * c1 * c2,  y_rreq * s1 * c2 - z_rreq * s1 * s2,
            0,  -y_rreq * s2 - z_rreq * c2,
            -x_rreq * c1 - y_rreq * s1 * s2 - z_rreq * s1 * c2,  y_rreq * c1 * c2 - z_rreq * s2 * c1;

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

        Vector2d AkDt(pitchDt, rollDt);
        
        // Calculate actuator velocities
        Vector2d ActDt = (Jxx_l_Td * JAct_p_WTd).ldlt().solve(Jxx_l_Td * JAk_p_WEq * AkDt);
        double theta_llbarDt = ActDt(0);
        double theta_lrbarDt = ActDt(1);

        return {theta_llbarDt, theta_lrbarDt};
    }

    double ArmJointController::pos_joint2actuator_knee(double knee)
    {
        double angle_BCD = knee + offsetKnee;
        double BD = std::sqrt(BC * BC + CD * CD - 2 * BC * CD * std::cos(angle_BCD));
        double angle_CBD = std::acos((BC * BC + BD * BD - CD * CD) / (2 * BC * BD));
        double angle_ABD = std::acos((AB * AB + BD * BD - AD * AD) / (2 * AB * BD));  
        double bar = M_PI - (angle_ABD - angle_CBD) - offsetBar;  
  
        return bar;  
    }

    double ArmJointController::vel_joint2actuator_knee(double knee, double bar, double kneeDt)
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
        Eigen::Vector2d vec_BA_cross(-vec_BA.y(), vec_BA.x());
        Eigen::Vector2d vec_CD(x_D - x_C, y_D - y_C);
        Eigen::Vector2d vec_CD_cross(vec_CD.y(), -vec_CD.x());
        Eigen::Vector2d vec_AD(x_D - x_A, y_D - y_A);
        double norm_AD = vec_AD.norm();

        // 计算点 D 的线速度
        double V_D = kneeDt * CD;

        // 计算点 A 的线速度在 AD 方向上的分量
        double norm_CD_cross = vec_CD_cross.norm();
        double V_A_AD = V_D * (vec_AD.dot(vec_CD_cross) / (norm_AD * norm_CD_cross));

        // 计算点 D 绕点 C 旋转的切线线速度
        double norm_BA_cross = vec_BA_cross.norm();
        double V_A = V_A_AD / ((vec_BA_cross.dot(vec_AD)) / (norm_BA_cross * norm_AD));

        // 计算膝关节的角速度
        double barDt = V_A / AB;

        return barDt;
    }

} // namespace lower_leg
