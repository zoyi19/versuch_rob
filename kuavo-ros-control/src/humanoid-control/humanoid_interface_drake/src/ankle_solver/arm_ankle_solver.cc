#include "humanoid_interface_drake/ankle_solver/arm_ankle_solver.h"

#include <iostream>
#include <string>

#include <Eigen/LU>
#include <drake/common/eigen_types.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/tree/linear_bushing_roll_pitch_yaw.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/snopt_solver.h>

namespace lower_leg
{

    using drake::MatrixX;
    using drake::Vector3;
    using drake::VectorX;
    using drake::multibody::JacobianWrtVariable;
    using drake::multibody::LinearBushingRollPitchYaw;
    using drake::multibody::MultibodyPlant;

    ArmAnkleSolverSystem::ArmAnkleSolverSystem(const MultibodyPlant<double> &plant, const std::vector<std::string> &cfg_str)
        : plant_(plant), cfg_(cfg_str)
    {
        nq_ = plant_.num_positions();
        nv_ = plant_.num_velocities();
        plant_context_ = plant_.CreateDefaultContext();

        const auto &j_l_arm = plant_.GetJointByName(cfg_.l_motor_joint);
        j_l_arm.Lock(plant_context_.get());
        i_j_l_arm_ = j_l_arm.position_start();
        const auto &j_r_arm = plant_.GetJointByName(cfg_.r_motor_joint);
        j_r_arm.Lock(plant_context_.get());
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
        /********************** 大迭代版本初始化数据 *******************************/
        l_llbar = sqrt(y_lltd*y_lltd + z_lltd*z_lltd);
        l_lrbar = sqrt(y_lrtd*y_lrtd + z_lrtd*z_lrtd);
        l_rlbar = sqrt(y_rltd*y_rltd + z_rltd*z_rltd);
        l_rrbar = sqrt(y_rrtd*y_rrtd + z_rrtd*z_rrtd);
    }

    void ArmAnkleSolverSystem::CalcQ(VectorXd &q) const
    {
        Eigen::VectorXd q_input(nq_);
        Eigen::VectorXd v_input(nv_);
        Eigen::VectorXd q_out(nq_);
        Eigen::VectorXd v_out(nv_);
        q_input = q.head(nq_);

        Eigen::VectorXd old_q = plant_.GetPositions(*plant_context_);
        old_q(i_j_l_arm_) = q_input(i_j_l_arm_);
        old_q(i_j_r_arm_) = q_input(i_j_r_arm_);

        const auto start = std::chrono::steady_clock::now();
        drake::multibody::InverseKinematics ik(plant_, plant_context_.get());
        for (drake::multibody::ForceElementIndex bushing_index(1);
             bushing_index < plant_.num_force_elements(); ++bushing_index)
        {
            const auto &bushing =
                plant_.template GetForceElement<LinearBushingRollPitchYaw>(bushing_index);

            ik.AddPositionConstraint(bushing.frameA(), Eigen::Vector3d::Zero(),
                                     bushing.frameC(),
                                     -1e-4 * Eigen::Vector3d::Ones(),
                                     1e-4 * Eigen::Vector3d::Ones());
        }
        const auto before_solve = std::chrono::steady_clock::now();
        const auto &res = drake::solvers::Solve(ik.prog(), old_q);
        const auto end = std::chrono::steady_clock::now();
        const std::chrono::duration calc_elapsed_ns = end - start;
        using namespace std::chrono_literals;
        double calc_elapsed = calc_elapsed_ns / 1us;
        // std::cout << "calc elapsed: " << calc_elapsed << "\n";
        double solve_elapsed = (end - before_solve) / 1us;
        std::cout << "solve elapsed: " << solve_elapsed << "\n";
        q_out = plant_.GetPositions(*plant_context_);
        // const auto& Xleft = plant_.CalcRelativeTransform(*plant_context_,
        //     plant_.GetFrameByName("l_link-foot"), plant_.GetFrameByName("foot-l_link"));
        // std::cout << "left distance: " << Xleft.translation().norm() << "\n";
    }

    /**
     * @brief motor_qv => foot_qv
     *
     * @param qv_ankle_arm l_motor, r_motor, l_motor_v, r_motor_v
     * @param qv output qv foot_roll, foot_pitch, long_motor, long_link(2), short_motor, short_link(2)
     */
    void ArmAnkleSolverSystem::CalcState(Vector4d qv_ankle_arm, VectorXd &ankle_qv)
    {
        const auto start = std::chrono::steady_clock::now();
        Eigen::VectorXd q_out(nq_);
        Eigen::VectorXd v_out(nv_);

        q_out.setZero();
        v_out.setZero();

        // 计算ankle关节位置
        // q_out = plant_.GetPositions(*plant_context_);
        q_out(i_j_l_arm_) = qv_ankle_arm(0);
        q_out(i_j_r_arm_) = qv_ankle_arm(1);
        v_out(i_j_l_arm_) = qv_ankle_arm(2);
        v_out(i_j_r_arm_) = qv_ankle_arm(3);

        const auto &world_frame = plant_.world_frame();
        const auto &body_foot = plant_.GetBodyByName(cfg_.end_link);
        const auto &body_Lkleft = plant_.GetBodyByName(cfg_.l_link);
        const auto &body_Lkright = plant_.GetBodyByName(cfg_.r_link);
        const auto &frame_Flkleft = plant_.GetFrameByName(cfg_.l_end_frame);
        const auto &frame_Flkright = plant_.GetFrameByName(cfg_.r_end_frame);

        // 迭代求解ankle关节角度
        // 迭代变量 roll, pitch
        for (int i = 0; i < 5; i++)
        {
            plant_.SetPositions(plant_context_.get(), q_out);
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

            const auto &p_FlkLkleft_W = p_WLkleft - p_WFlkleft;
            const auto &p_FlkLkright_W = p_WLkright - p_WFlkright;

            // 用上次迭代的pitch和roll代替真值，用勾股定理得到棍的z值。原理是假设棍子接近竖直。
            double z_FlkLkleft_W = sqrt(l_Lkleft_ * l_Lkleft_ - p_FlkLkleft_W(0) * p_FlkLkleft_W(0) - p_FlkLkleft_W(1) * p_FlkLkleft_W(1));
            double z_FlkLkright_W = sqrt(l_Lkright_ * l_Lkright_ - p_FlkLkright_W(0) * p_FlkLkright_W(0) - p_FlkLkright_W(1) * p_FlkLkright_W(1));

            // roll[k+1] = arcsin(Δz_Lkf(roll[k], pitch[k]) / Δy)
            double z_LkfrightLkfleft_W = (p_WLkleft(2) - z_FlkLkleft_W) - (p_WLkright(2) - z_FlkLkright_W);
            double y_FlkrightFlkleft = plant_.CalcRelativeTransform(
                                                 *plant_context_, frame_Flkright, frame_Flkleft)
                                           .translation()(1);
            q_out(i_roll_) = asin(z_LkfrightLkfleft_W / y_FlkrightFlkleft);

            // pitch[k+1] = arcsin(z_avg / (x_avg*cos(roll[k+1]))), avg表示加权平均，
            // 就是x值对齐ankle joint的点. z_avg是对于Lkf的。
            double y_FootFlkleft =
                frame_Flkleft.CalcPoseInBodyFrame(*plant_context_).translation()(1);
            double y_FootFlkright =
                frame_Flkright.CalcPoseInBodyFrame(*plant_context_).translation()(1);
            double avg_z = (y_FootFlkleft * (p_WLkright(2) - z_FlkLkright_W) + -y_FootFlkright * (p_WLkleft(2) - z_FlkLkleft_W)) / (y_FootFlkleft - y_FootFlkright);
            // left_x=right_x
            double avg_x =
                -frame_Flkleft.CalcPoseInBodyFrame(*plant_context_).translation()(0);
            double z_WFt =
                plant_.EvalBodyPoseInWorld(*plant_context_, body_foot).translation()(2);
            q_out(i_pitch_) = asin((avg_z - z_WFt) / (avg_x * cos(q_out(i_roll_))));
            if (abs(p_LkFlkleft.norm() - l_Lkleft_) < 1e-6 && (p_LkFlkright.norm() - l_Lkright_) < 1e-6)
            {
                //   std::cout << i << "\n";
                break;
            }
        }
        // std::cout << "q pitch " << q_out(i_pitch_) << std::endl;
        // std::cout << "q roll " << q_out(i_roll_) << std::endl;

        // 计算关节速度
        plant_.SetPositions(plant_context_.get(), q_out); // 虽然在for中干了，再干一次以示尊敬
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

        const auto &p_FlkLkleft_W = p_WLkleft - p_WFlkleft;
        const auto &p_FlkLkright_W = p_WLkright - p_WFlkright;
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
        MatrixXd JakDt_lDt_Lk(2, 2);
        Vector3d phat_FlkLkleft_W = (p_FlkLkleft_W / l_Lkleft_);
        Vector3d phat_FlkLkright_W = (p_FlkLkright_W / l_Lkright_);
        JakDt_lDt_Lk(0, 0) = phat_FlkLkleft_W.dot(Jv_v_WFlkleft.col(i_roll_));
        JakDt_lDt_Lk(0, 1) = phat_FlkLkleft_W.dot(Jv_v_WFlkleft.col(i_pitch_));
        JakDt_lDt_Lk(1, 0) = phat_FlkLkright_W.dot(Jv_v_WFlkright.col(i_roll_));
        JakDt_lDt_Lk(1, 1) = phat_FlkLkright_W.dot(Jv_v_WFlkright.col(i_pitch_));
        VectorXd B(2);
        // 如果transpose是原地转置，那么会出问题
        // B(0) = (phat_FlkLkleft_W.transpose() * Jv_v_WLkleft * v_input)(0); // 可以更快，但也可以偷懒
        // B(1) = (phat_FlkLkright_W.transpose() * Jv_v_WLkright * v_input)(0);
        B(0) = (phat_FlkLkleft_W.transpose() * Jv_v_WLkleft * v_out)(0); // 可以更快，但也可以偷懒
        B(1) = (phat_FlkLkright_W.transpose() * Jv_v_WLkright * v_out)(0);
        VectorXd akDt = JakDt_lDt_Lk.lu().solve(B);
        v_out(i_roll_) = akDt(0);
        v_out(i_pitch_) = akDt(1);

        const auto end = std::chrono::steady_clock::now();
        const std::chrono::duration calc_elapsed_ns = end - start;
        using namespace std::chrono_literals;
        double calc_elapsed = calc_elapsed_ns / 1us;

        plant_.SetVelocities(plant_context_.get(), v_out);

        ankle_qv = plant_.GetPositionsAndVelocities(*plant_context_);
    }

    std::pair<double, double> ArmAnkleSolverSystem::pos_actuator2joint_left(double llbar, double lrbar)
    {
        double pitch = 0, roll = 0;
        for(int i=0; i<10; i++) {
            double pitch0 = 0, roll0 = 0;
            double c1 = cos(pitch0);
            double s1 = sin(pitch0);
            double c2 = cos(roll0);
            double s2 = sin(roll0);
            double c3 = cos(llbar);
            double s3 = sin(llbar);
            double c4 = cos(lrbar);
            double s4 = sin(lrbar);

            double x_LltendonEq_W = x_lleq*c1 - x_lltd + y_lleq*s1*s2 + z_lleq*s1*c2;
            double y_LltendonEq_W = y_lleq*c2 - y_lltd*c3 - z_lleq*s2 + z_lltd*s3;
            double z_LltendonEq_W = -sqrt(l_lltd*l_lltd - x_LltendonEq_W*x_LltendonEq_W - y_LltendonEq_W*y_LltendonEq_W);
            double z_WLltendon = y_lltd*s3 + z_llbar + z_lltd*c3;
            double z_PitchLleq_W = z_LltendonEq_W - (z_pitch - z_WLltendon);
            double x_LrtendonEq_W = x_lreq*c1 - x_lrtd + y_lreq*s1*s2 + z_lreq*s1*c2;
            double y_LrtendonEq_W = y_lreq*c2 - y_lrtd*c4 - z_lreq*s2 + z_lrtd*s4;
            double z_LrtendonEq_W = -sqrt(l_lrtd*l_lrtd - x_LrtendonEq_W*x_LrtendonEq_W - y_LrtendonEq_W*y_LrtendonEq_W);
            double z_WLrtendon = y_lrtd*s4 + z_lrbar + z_lrtd*c4;
            double z_PitchLreq_W = z_LrtendonEq_W - (z_pitch - z_WLrtendon);
            double z_PitchLtoe_W = (z_PitchLleq_W+z_PitchLreq_W)/2;

            double x = x_lleq;
            double y = y_lleq;
            double z = z_lleq;
            double avg = z_PitchLtoe_W;
            double dev = (z_PitchLleq_W - z_PitchLreq_W) / 2;
            double x2 = x*x;
            double y2 = y*y;
            double z2 = z*z;
            double avg2 = avg*z;
            double dev2 = dev*dev;
            double delta = avg2*x2 - (x2+z2)*(avg2+z2*dev2/y2-z2);
            double s_1 = (-avg*x-sqrt(delta))/(x2+z2);
            pitch = asin(s_1);
            double z_LrLleq_W = z_PitchLleq_W - z_PitchLreq_W;
            double z_LrLleq_Pitch = z_LrLleq_W / cos(pitch);
            roll = asin((z_LrLleq_Pitch)/(y_lleq-y_lreq));
            if((fabs(pitch-pitch0)+fabs(roll-roll0))<1e-6){
                break;
            }
        }
        return {pitch, roll};
    }

    std::pair<double, double> ArmAnkleSolverSystem::vel_actuator2joint_left(double pitch, double roll, double llbar, double lrbar, double llbarDt, double lrbarDt)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        double c3 = cos(llbar);
        double s3 = sin(llbar);
        double c4 = cos(lrbar);
        double s4 = sin(lrbar);

        MatrixXd JAk_p_WEq(6, 2);
        JAk_p_WEq << -x_lleq * s1 + y_lleq * s2 * c1 + z_lleq * c1 * c2, y_lleq * s1 * c2 - z_lleq * s1 * s2,
                      0, -y_lleq * s2 - z_lleq * c2,
                      -x_lleq * c1 - y_lleq * s1 * s2 - z_lleq * s1 * c2, y_lleq * c1 * c2 - z_lleq * s2 * c1,
                      -x_lreq * s1 + y_lreq * s2 * c1 + z_lreq * c1 * c2, y_lreq * s1 * c2 - z_lreq * s1 * s2,
                      0, -y_lreq * s2 - z_lreq * c2,
                      -x_lreq * c1 - y_lreq * s1 * s2 - z_lreq * s1 * c2, y_lreq * c1 * c2 - z_lreq * s2 * c1;
        // std::cout << "JAk_p_WEq.shape: " << JAk_p_WEq.rows() << ", " << JAk_p_WEq.cols() << std::endl;

        MatrixXd JAct_p_WTd(6, 2);
        JAct_p_WTd << 0, 0,
                      -y_lltd * s3 - z_lltd * c3, 0,
                      y_lltd * c3 - z_lltd * s3, 0,
                      0, 0,
                      0, -y_lrtd * s4 - z_lrtd * c4,
                      0, y_lrtd * c4 - z_lrtd * s4;
        // std::cout << "JAct_p_WTd.shape: " << JAct_p_WTd.rows() << ", " << JAct_p_WTd.cols() << std::endl;

        MatrixXd Jxx_l_Td(2, 6);
        Jxx_l_Td << x_lleq * c1 - x_lltd + y_lleq * s1 * s2 + z_lleq * s1 * c2,
                    y_lleq * c2 - y_lltd * c3 - z_lleq * s2 + z_lltd * s3,
                    -x_lleq * s1 + y_lleq * s2 * c1 - y_lltd * s3 - z_llbar + z_lleq * c1 * c2 - z_lltd * c3 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_lreq * c1 - x_lrtd + y_lreq * s1 * s2 + z_lreq * s1 * c2,
                    y_lreq * c2 - y_lrtd * c4 - z_lreq * s2 + z_lrtd * s4,
                    -x_lreq * s1 + y_lreq * s2 * c1 - y_lrtd * s4 - z_lrbar + z_lreq * c1 * c2 - z_lrtd * c4 + z_pitch;

        Vector2d ActDt(llbarDt, lrbarDt);
        Vector2d AkDt = (Jxx_l_Td * JAk_p_WEq).lu().solve(Jxx_l_Td * JAct_p_WTd * ActDt);
        double pitchDt = AkDt(0);
        double rollDt = AkDt(1);

        return {pitchDt, rollDt};
    }

    std::pair<double, double> ArmAnkleSolverSystem::pos_actuator2joint_right(double llbar, double lrbar)
    {
        double pitch = 0, roll = 0;
        for(int i=0; i<10; i++) {
            double pitch0 = 0, roll0 = 0;
            double c1 = cos(pitch0);
            double s1 = sin(pitch0);
            double c2 = cos(roll0);
            double s2 = sin(roll0);
            double c3 = cos(llbar);
            double s3 = sin(llbar);
            double c4 = cos(lrbar);
            double s4 = sin(lrbar);

            double x_LltendonEq_W = x_rleq*c1 - x_rltd + y_rleq*s1*s2 + z_rleq*s1*c2;
            double y_LltendonEq_W = y_rleq*c2 - y_rltd*c3 - z_rleq*s2 + z_rltd*s3;
            double z_LltendonEq_W = -sqrt(l_rltd*l_rltd - x_LltendonEq_W*x_LltendonEq_W - y_LltendonEq_W*y_LltendonEq_W);
            double z_WLltendon = y_rltd*s3 + z_rlbar + z_rltd*c3;
            double z_PitchLleq_W = z_LltendonEq_W - (z_pitch - z_WLltendon);
            double x_LrtendonEq_W = x_rreq*c1 - x_rrtd + y_rreq*s1*s2 + z_lreq*s1*c2;
            double y_LrtendonEq_W = y_rreq*c2 - y_rrtd*c4 - z_lreq*s2 + z_rrtd*s4;
            double z_LrtendonEq_W = -sqrt(l_rrtd*l_rrtd - x_LrtendonEq_W*x_LrtendonEq_W - y_LrtendonEq_W*y_LrtendonEq_W);
            double z_WLrtendon = y_rrtd*s4 + z_rrbar + z_rrtd*c4;
            double z_PitchLreq_W = z_LrtendonEq_W - (z_pitch - z_WLrtendon);
            double z_PitchLtoe_W = (z_PitchLleq_W+z_PitchLreq_W)/2;

            double x = x_rleq;
            double y = y_rleq;
            double z = z_rleq;
            double avg = z_PitchLtoe_W;
            double dev = (z_PitchLleq_W - z_PitchLreq_W) / 2;
            double x2 = x*x;
            double y2 = y*y;
            double z2 = z*z;
            double avg2 = avg*z;
            double dev2 = dev*dev;
            double delta = avg2*x2 - (x2+z2)*(avg2+z2*dev2/y2-z2);
            double s_1 = (-avg*x-sqrt(delta))/(x2+z2);
            pitch = asin(s_1);
            double z_LrLleq_W = z_PitchLleq_W - z_PitchLreq_W;
            double z_LrLleq_Pitch = z_LrLleq_W / cos(pitch);
            roll = asin((z_LrLleq_Pitch)/(y_rleq-y_rreq));
            if((fabs(pitch-pitch0)+fabs(roll-roll0))<1e-6){
                break;
            }
        }
        return {pitch, roll};
    }

    std::pair<double, double> ArmAnkleSolverSystem::vel_actuator2joint_right(double pitch, double roll, double llbar, double lrbar, double llbarDt, double lrbarDt)
    {
        double c1 = cos(pitch);
        double s1 = sin(pitch);
        double c2 = cos(roll);
        double s2 = sin(roll);
        double c3 = cos(llbar);
        double s3 = sin(llbar);
        double c4 = cos(lrbar);
        double s4 = sin(lrbar);

        MatrixXd JAk_p_WEq(6, 2);
        JAk_p_WEq << -x_rleq * s1 + y_rleq * s2 * c1 + z_rleq * c1 * c2, y_rleq * s1 * c2 - z_rleq * s1 * s2,
                      0, -y_rleq * s2 - z_rleq * c2,
                      -x_rleq * c1 - y_rleq * s1 * s2 - z_rleq * s1 * c2, y_rleq * c1 * c2 - z_rleq * s2 * c1,
                      -x_rreq * s1 + y_rreq * s2 * c1 + z_rreq * c1 * c2, y_rreq * s1 * c2 - z_rreq * s1 * s2,
                      0, -y_rreq * s2 - z_rreq * c2,
                      -x_rreq * c1 - y_rreq * s1 * s2 - z_rreq * s1 * c2, y_rreq * c1 * c2 - z_rreq * s2 * c1;
        // std::cout << "JAk_p_WEq.shape: " << JAk_p_WEq.rows() << ", " << JAk_p_WEq.cols() << std::endl;

        MatrixXd JAct_p_WTd(6, 2);
        JAct_p_WTd << 0, 0,
                      -y_rltd * s3 - z_rltd * c3, 0,
                      y_rltd * c3 - z_rltd * s3, 0,
                      0, 0,
                      0, -y_rrtd * s4 - z_rrtd * c4,
                      0, y_rrtd * c4 - z_rrtd * s4;
        // std::cout << "JAct_p_WTd.shape: " << JAct_p_WTd.rows() << ", " << JAct_p_WTd.cols() << std::endl;

        MatrixXd Jxx_l_Td(2, 6);
        Jxx_l_Td << x_rleq * c1 - x_rltd + y_rleq * s1 * s2 + z_rleq * s1 * c2,
                    y_rleq * c2 - y_rltd * c3 - z_rleq * s2 + z_rltd * s3,
                    -x_rleq * s1 + y_rleq * s2 * c1 - y_rltd * s3 - z_rlbar + z_rleq * c1 * c2 - z_rltd * c3 + z_pitch,
                    0, 0, 0,
                    0, 0, 0,
                    x_rreq * c1 - x_rrtd + y_rreq * s1 * s2 + z_rreq * s1 * c2,
                    y_rreq * c2 - y_rrtd * c4 - z_rreq * s2 + z_rrtd * s4,
                    -x_rreq * s1 + y_rreq * s2 * c1 - y_rrtd * s4 - z_rrbar + z_rreq * c1 * c2 - z_rrtd * c4 + z_pitch;

        Vector2d ActDt(llbarDt, lrbarDt);
        Vector2d AkDt = (Jxx_l_Td * JAk_p_WEq).lu().solve(Jxx_l_Td * JAct_p_WTd * ActDt);
        double pitchDt = AkDt(0);
        double rollDt = AkDt(1);

        return {pitchDt, rollDt};
    }

    double ArmAnkleSolverSystem::pos_actuator2joint_knee(double bar)
    {
        double angle_ABC = M_PI - (bar + offsetBar); // 使用M_PI表示π  
        double AC = sqrt(AB * AB + BC * BC - 2 * AB * BC * cos(angle_ABC));  
        double angle_ACB = acos((AC * AC + BC * BC - AB * AB) / (2 * AC * BC));  
        double angle_ACD = acos((AC * AC + CD * CD - AD * AD) / (2 * AC * CD));  
        double knee = (angle_ACD - angle_ACB) - offsetKnee;  
        return knee;
    }

    double ArmAnkleSolverSystem::vel_actuator2joint_knee(double knee, double bar, double barDt)
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

        double V_A = barDt * AB;

        // V_A 从垂直BA，获取沿AD的分量
        double norm_BA_cross = vec_BA_cross.norm();
        double V_A_AD = V_A * (vec_BA_cross.dot(vec_AD) / (norm_BA_cross * norm_AD));

        // V_A_AD 从沿着AD，获取沿DC垂直方向的分量
        double norm_CD_cross = vec_CD_cross.norm();
        double V_D = V_A_AD / (vec_AD.dot(vec_CD_cross) / (norm_AD * norm_CD_cross));

        double kneeDt = V_D / CD;

        return kneeDt;
    }

} // namespace lower_leg