#include "humanoid_interface_drake/humanoid_interface_drake.h"
#include "kuavo_common/kuavo_common.h"
#include "kuavo_assets/include/package_path.h"



namespace HighlyDynamic
{
    using namespace drake;
    std::mutex instance_mutex_;

    std::shared_ptr<HumanoidInterfaceDrake> HumanoidInterfaceDrake::instance = nullptr;
    HumanoidInterfaceDrake &HumanoidInterfaceDrake::getInstance(RobotVersion rb_version, bool real, double dt)
    {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        if (!instance)
        {
            std::cout << ">>>>>>>>>>>>>>>>>>>> Creating new instance of HumanoidInterfaceDrake <<<<<<<<<<\n";
            instance.reset(new HumanoidInterfaceDrake(rb_version, real, dt));
        }
        return *instance;
    }
    HumanoidInterfaceDrake *HumanoidInterfaceDrake::getInstancePtr(RobotVersion rb_version, bool real, double dt)
    {
        getInstance(rb_version, real, dt);
        return instance.get();
    }

    HumanoidInterfaceDrake::~HumanoidInterfaceDrake()
    {

        if (robot_config_) {
            delete robot_config_;
            robot_config_ = nullptr;
        }

        std::cout << "HumanoidInterfaceDrake is being destroyed." << std::endl;
    }

    HumanoidInterfaceDrake::HumanoidInterfaceDrake(RobotVersion rb_version, bool real, double dt) : dt_(dt), rb_version_(rb_version)
    {
        KuavoCommon& kuavo_common = KuavoCommon::getInstance(rb_version, ocs2::kuavo_assets::getPath());
        robot_config_ = kuavo_common.getRobotConfig();
        kuavo_settings_ = kuavo_common.getKuavoSettings();

        if(rb_version_.major() != 6)
        {
            buildMultibodyPlant(real);
            buildAnkleSolversMultibodyPlant(kuavo_settings_.model_settings.is_parallel_arm);
            buildAnkleSolvers(kuavo_settings_.model_settings.is_parallel_arm);
            calcSquatState(plant_ptr_.get(), plant_context_.get());
            calInitialState(plant_ptr_.get(), plant_context_.get());
            std::cout << ">>>>>>>>>>>>>>>>>>>> HumanoidInterfaceDrake INFO <<<<<<<<<<<<<<<<<<<<<<<<<\n";
            std::cout << "Robot version: " << rb_version << std::endl;
            std::cout << "Real: " << real << std::endl;
            std::cout << "Time step: " << dt << std::endl;
            std::cout << "Total mass: " << getPlantMass() << std::endl;
            std::cout << "Total mass (with arm): " << getPlantWithArmMass() << std::endl;
            // std::cout << "End frames name: ";
            // for (auto &n : end_frames_name_)
            //     std::cout << n << " ";
            // std::cout << std::endl;
            std::cout << ">>>>>>>>>>>>>>>>>>>> HumanoidInterfaceDrake INFO <<<<<<<<<<<<<<<<<<<<<<<<<\n";
        }
    }

    void HumanoidInterfaceDrake::buildMultibodyPlant(bool real)
    {
        std::string model_path = kuavo_settings_.model_settings.model_path;
        std::string model_with_arm_path = kuavo_settings_.model_settings.model_with_arm_path;
        // std::cout << "model_path: " << model_path << std::endl;
        // std::cout << "model_with_arm_path: " << model_with_arm_path << std::endl;

        plant_ptr_.reset(builder_.AddSystem<multibody::MultibodyPlant>(dt_));
        plant_with_arm_ptr_.reset(builder_.AddSystem<multibody::MultibodyPlant>(dt_));

        if (!real)
        {
            lcm_ptr_.reset(builder_.AddSystem<systems::lcm::LcmInterfaceSystem>());
            scene_graph_ptr_.reset(builder_.AddSystem<geometry::SceneGraph>());
            plant_with_arm_ptr_->RegisterAsSourceForSceneGraph(scene_graph_ptr_.get());
            builder_.Connect(plant_with_arm_ptr_->get_geometry_poses_output_port(), scene_graph_ptr_->get_source_pose_port(plant_with_arm_ptr_->get_source_id().value()));
            builder_.Connect(scene_graph_ptr_->get_query_output_port(), plant_with_arm_ptr_->get_geometry_query_input_port());
            geometry::DrakeVisualizerd::AddToBuilder(&builder_, *scene_graph_ptr_, lcm_ptr_.get());

            // std::cout << "model_with_arm_path: " << model_with_arm_path << std::endl;
            const multibody::ModelInstanceIndex model_instance = multibody::Parser(plant_with_arm_ptr_.get(), scene_graph_ptr_.get()).AddModelFromFile(model_with_arm_path);
            // Add model of the ground.
            const double static_friction = 1.0;
            const drake::Vector4<double> color(0.9, 0.9, 0.9, 1.0);
            plant_with_arm_ptr_->RegisterVisualGeometry(plant_with_arm_ptr_->world_body(),
                                                        math::RigidTransformd(), geometry::HalfSpace(),
                                                        "GroundVisualGeometry",
                                                        color);
            // For a time-stepping model only static friction is used.
            const multibody::CoulombFriction<double> ground_friction(static_friction, static_friction);
            plant_with_arm_ptr_->RegisterCollisionGeometry(plant_with_arm_ptr_->world_body(),
                                                           math::RigidTransformd(),
                                                           geometry::HalfSpace(),
                                                           "GroundCollisionGeometry",
                                                           ground_friction);
        }
        else
        {
            multibody::Parser(plant_with_arm_ptr_.get()).AddModelFromFile(model_with_arm_path);
        }
        multibody::Parser(plant_ptr_.get()).AddModelFromFile(model_path);
        plant_with_arm_ptr_->set_discrete_contact_solver(drake::multibody::DiscreteContactSolver::kSap);

        auto AddBallConstraintForAnkle = [&](const std::string &base_body,
                                             const std::string &tendon_l_body, const std::string &tendon_r_body,
                                             const std::string &base_l, const std::string &base_r,
                                             const std::string &tendon_l, const std::string &tendon_r)
        {
            const auto &p_base_l_socket = plant_with_arm_ptr_->GetFrameByName(base_l).GetFixedPoseInBodyFrame();
            const auto &p_tendon_l_socket = plant_with_arm_ptr_->GetFrameByName(tendon_l).GetFixedPoseInBodyFrame();
            const auto &p_base_r_socket = plant_with_arm_ptr_->GetFrameByName(base_r).GetFixedPoseInBodyFrame();
            const auto &p_tendon_r_socket = plant_with_arm_ptr_->GetFrameByName(tendon_r).GetFixedPoseInBodyFrame();
            plant_with_arm_ptr_->AddBallConstraint(plant_with_arm_ptr_->GetBodyByName(base_body),
                                                   p_base_l_socket.translation(),
                                                   plant_with_arm_ptr_->GetBodyByName(tendon_l_body),
                                                   p_tendon_l_socket.translation());
            plant_with_arm_ptr_->AddBallConstraint(plant_with_arm_ptr_->GetBodyByName(base_body),
                                                   p_base_r_socket.translation(),
                                                   plant_with_arm_ptr_->GetBodyByName(tendon_r_body),
                                                   p_tendon_r_socket.translation());
        };
        if (kuavo_settings_.model_settings.is_parallel_arm)
        { // arm
            std::cout << "The model has Parallel Arm" << std::endl;
            AddBallConstraintForAnkle("l_hand_pitch", "l_r_arm_tendon", "l_l_arm_tendon",
                                      "l_r_hand_socket", "l_l_hand_socket",
                                      "l_r_arm_tendon_socket", "l_l_arm_tendon_socket");
            AddBallConstraintForAnkle("r_hand_pitch", "r_r_arm_tendon", "r_l_arm_tendon",
                                      "r_r_hand_socket", "r_l_hand_socket",
                                      "r_r_arm_tendon_socket", "r_l_arm_tendon_socket");
        }
        // foot
        AddBallConstraintForAnkle("leg_l6_link", "l_l_tendon_y", "l_r_tendon_y",
                                  "l_l_foot_socket", "l_r_foot_socket", "l_l_tendon_socket", "l_r_tendon_socket");
        AddBallConstraintForAnkle("leg_r6_link", "r_l_tendon_y", "r_r_tendon_y",
                                  "r_l_foot_socket", "r_r_foot_socket", "r_l_tendon_socket", "r_r_tendon_socket");
        plant_ptr_->Finalize();
        plant_with_arm_ptr_->Finalize();

        if (!real)
        {
            // sim_sensors_ptr = new SimSensor(g_plant_with_arm, FLAGS_dt);
            // sim_sensors_ptr->AddImu(&builder, *g_plant_with_arm);
        }
        diagram_ = builder_.Build();
        diagram_context_ = diagram_->CreateDefaultContext();
        plant_context_.reset(&diagram_->GetMutableSubsystemContext(*plant_ptr_.get(), diagram_context_.get()));
        plant_with_arm_context_.reset(&diagram_->GetMutableSubsystemContext(*plant_with_arm_ptr_.get(), diagram_context_.get()));
    }

    void HumanoidInterfaceDrake::buildAnkleSolversMultibodyPlant(bool is_parallel_arm)
    {
        /* right ankle builder */
        static drake::systems::DiagramBuilder<double> right_ankle_builder;

        auto [right_ankle_plant, right_ankle_scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&right_ankle_builder, dt_);
        std::string relative_name = kuavo_settings_.model_settings.right_leg_urdf;
        drake::multibody::Parser(&right_ankle_plant).AddModelFromFile(relative_name);
        right_ankle_plant.WeldFrames(right_ankle_plant.world_frame(), right_ankle_plant.GetBodyByName("base_link").body_frame(),
                                     drake::math::RigidTransformd());
        right_ankle_plant.Finalize();
        right_foot_ankle_plant_ptr_.reset(&right_ankle_plant);

        /* left ankle builder */
        static drake::systems::DiagramBuilder<double> left_ankle_builder;

        auto [left_ankle_plant, left_ankle_scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&left_ankle_builder, dt_);
        std::string left_relative_name = kuavo_settings_.model_settings.left_leg_urdf;
        drake::multibody::Parser(&left_ankle_plant).AddModelFromFile(left_relative_name);
        left_ankle_plant.WeldFrames(left_ankle_plant.world_frame(), left_ankle_plant.GetBodyByName("base_link").body_frame(),
                                    drake::math::RigidTransformd());
        left_ankle_plant.Finalize();
        left_foot_ankle_plant_ptr_.reset(&left_ankle_plant);

        if (!is_parallel_arm)
            return;
        // arm
        /* left arm ankle builder */
        static drake::systems::DiagramBuilder<double> left_arm_ankle_builder;

        auto [left_arm_ankle_plant, left_arm_ankle_scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&left_arm_ankle_builder, dt_);
        std::string left_arm_relative_name = kuavo_settings_.model_settings.left_arm_urdf;
        drake::multibody::Parser(&left_arm_ankle_plant).AddModelFromFile(left_arm_relative_name);
        left_arm_ankle_plant.WeldFrames(left_arm_ankle_plant.world_frame(), left_arm_ankle_plant.GetBodyByName("base_link").body_frame(),
                                        drake::math::RigidTransformd());
        left_arm_ankle_plant.Finalize();
        left_arm_ankle_plant_ptr_.reset(&left_arm_ankle_plant);

        /* right arm ankle builder */
        static drake::systems::DiagramBuilder<double> right_arm_ankle_builder;

        auto [right_arm_ankle_plant, right_arm_ankle_scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&right_arm_ankle_builder, dt_);
        std::string right_arm_relative_name = kuavo_settings_.model_settings.right_arm_urdf;
        drake::multibody::Parser(&right_arm_ankle_plant).AddModelFromFile(right_arm_relative_name);
        right_arm_ankle_plant.WeldFrames(right_arm_ankle_plant.world_frame(), right_arm_ankle_plant.GetBodyByName("base_link").body_frame(),
                                         drake::math::RigidTransformd());
        right_arm_ankle_plant.Finalize();
        right_arm_ankle_plant_ptr_.reset(&right_arm_ankle_plant);
    }

    void HumanoidInterfaceDrake::buildAnkleSolvers(bool is_parallel_arm)
    {
        std::vector<std::string> l_foot_ankle = kuavo_settings_.model_settings.left_foot_ankle_link_joint_frames;
        std::vector<std::string> r_foot_ankle = kuavo_settings_.model_settings.right_foot_ankle_link_joint_frames;

        left_foot_ankle_solver_ptr_.reset(new lower_leg::ArmAnkleSolverSystem(*left_foot_ankle_plant_ptr_, l_foot_ankle));
        right_foot_ankle_solver_ptr_.reset(new lower_leg::ArmAnkleSolverSystem(*right_foot_ankle_plant_ptr_, r_foot_ankle));
        left_foot_ankle_jcont_ptr_.reset(new lower_leg::ArmJointController(*left_foot_ankle_plant_ptr_, l_foot_ankle));
        right_foot_ankle_jcont_ptr_.reset(new lower_leg::ArmJointController(*right_foot_ankle_plant_ptr_, r_foot_ankle));
        left_foot_ankle_tendon_ptr_.reset(new lower_leg::ArmTendonController(*left_foot_ankle_plant_ptr_, l_foot_ankle));
        right_foot_ankle_tendon_ptr_.reset(new lower_leg::ArmTendonController(*right_foot_ankle_plant_ptr_, r_foot_ankle));

        if (!is_parallel_arm)
            return;

        std::vector<std::string> l_arm_ankle = kuavo_settings_.model_settings.left_arm_ankle_link_joint_frames;
        std::vector<std::string> r_arm_ankle = kuavo_settings_.model_settings.right_arm_ankle_link_joint_frames;
        left_arm_ankle_solver_ptr_.reset(new lower_leg::ArmAnkleSolverSystem(*left_arm_ankle_plant_ptr_, l_arm_ankle));
        right_arm_ankle_solver_ptr_.reset(new lower_leg::ArmAnkleSolverSystem(*right_arm_ankle_plant_ptr_, r_arm_ankle));
        left_arm_ankle_jcont_ptr_.reset(new lower_leg::ArmJointController(*left_arm_ankle_plant_ptr_, l_arm_ankle));
        right_arm_ankle_jcont_ptr_.reset(new lower_leg::ArmJointController(*right_arm_ankle_plant_ptr_, r_arm_ankle));
        left_arm_ankle_tendon_ptr_.reset(new lower_leg::ArmTendonController(*left_arm_ankle_plant_ptr_, l_arm_ankle));
        right_arm_ankle_tendon_ptr_.reset(new lower_leg::ArmTendonController(*right_arm_ankle_plant_ptr_, r_arm_ankle));
    }

    Eigen::VectorXd HumanoidInterfaceDrake::calInitialState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context)
    {
        auto end_frames_name = kuavo_settings_.model_settings.end_frames_name;
        math::RigidTransformd foot_in_torso = plant->GetFrameByName(end_frames_name[1]).CalcPose(*plant_context, plant->GetFrameByName(end_frames_name[0]));
        math::RigidTransformd r_foot_in_torso = plant->GetFrameByName(end_frames_name[2]).CalcPose(*plant_context, plant->GetFrameByName(end_frames_name[0]));
        Eigen::Vector3d p0_foot(0, foot_in_torso.translation()[1], 0);
        Eigen::VectorXd q0 = plant->GetPositions(*plant_context);
        q0[6] = -foot_in_torso.translation()[2]; // 从动力学模型中测量出来的脚和躯干的高度（值为负），将躯提升到地面上这个高度
        q_initial_ = q0;
        plant->SetPositions(plant_context, q0);
        Eigen::VectorXd r0 = plant->CalcCenterOfMassPositionInWorld(*plant_context);

        std::cout << "foot width: " << foot_in_torso.translation()[1] - r_foot_in_torso.translation()[1] << std::endl;

        std::vector<std::string> initial_joint_name{"leg_l4_joint", "leg_r4_joint"};
        std::vector<double> initial_joint_pos{0.4, 0.4};
        for (uint32_t i = 0; i < initial_joint_name.size(); i++)
        {
            const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(initial_joint_name[i]);
            joint.set_angle(plant_context, initial_joint_pos[i]);
        }
        q0 = plant->GetPositions(*plant_context);

        CoMIK ik(plant, end_frames_name, 1e-6, 1e-6);
        JSONConfigReader *robot_config_ptr = getRobotConfig();
        auto torsoY = robot_config_ptr->getValue<double>("torsoY") * TO_RADIAN;
        std::vector<std::vector<Eigen::Vector3d>> pose_vec{
            {Eigen::Vector3d(0, robot_config_ptr->getValue<double>("torsoP") * TO_RADIAN, torsoY), Eigen::Vector3d(0, 0, robot_config_ptr->getValue<double>("com_z"))},
            {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(p0_foot[0], robot_config_ptr->getValue<double>("StepWith") / 2, p0_foot[2])},
            {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(p0_foot[0], -robot_config_ptr->getValue<double>("StepWith") / 2, p0_foot[2])}};
        Eigen::VectorXd q;
        bool result = ik.solve(pose_vec, q0, q);
        if (result == false)
        {
            std::cout << "torso: " << pose_vec[0][0].transpose() << ", " << pose_vec[0][1].transpose() << "\n";
            std::cout << "lfoot: " << pose_vec[1][0].transpose() << ", " << pose_vec[1][1].transpose() << "\n";
            std::cout << "rfoot: " << pose_vec[2][0].transpose() << ", " << pose_vec[2][1].transpose() << "\n";
            throw std::runtime_error("Failed to IK0!");
        }
        plant->SetPositions(plant_context, q);
        q_initial_ = q;

        std::cout << "q_initial_: \n";
        for (size_t i = 0; i < q_initial_.size(); i++)
        {
            std::cout << q_initial_[i] << ", ";
        }
        std::cout << std::endl;

        std::string base_str = R"__(
(6,0)  {1}     ; p_base_x
(7,0)  {2}     ; p_base_y
(8,0)  {3}   ; p_base_z
(9,0)  {4}     ; theta_base_z
(10,0) {5}     ; theta_base_y
(11,0) {6}     ; theta_base_x
)__";

        Eigen::VectorXd base_vec(6);
        base_vec << q_initial_.segment(4, 3), 0, robot_config_ptr->getValue<double>("torsoP") * TO_RADIAN, 0;
        Eigen::VectorXd joint_pos(12);
        joint_pos << q_initial_.segment(7, 12);
        std::string outputString = R"__(
(12,0) {1}   ; leg_l1_joint
(13,0) {2}   ; leg_l2_joint
(14,0) {3}   ; leg_l3_joint
(15,0) {4}  ; leg_l4_joint
(16,0) {5}   ; leg_l5_joint
(17,0) {6}  ; leg_l6_joint
(18,0)  {7}  ; leg_r1_joint
(19,0) {8}   ; leg_r2_joint
(20,0) {9}  ; leg_r3_joint
(21,0)  {10} ; leg_r4_joint
(22,0) {11}   ; leg_r5_joint
(23,0) {12}  ; leg_r6_joint
)__";

        std::string defaultJointState = R"__(
(0,0)   {1} ; leg_l1_joint
(1,0)   {2} ; leg_l2_joint
(2,0)   {3} ; leg_l3_joint
(3,0)   {4} ; leg_l4_joint
(4,0)   {5} ; leg_l5_joint
(5,0)   {6} ; leg_l6_joint
(6,0)   {7} ; leg_r1_joint
(7,0)   {8} ; leg_r2_joint
(8,0)   {9} ; leg_r3_joint
(9,0)   {10} ; leg_r4_joint
(10,0)  {11} ; leg_r5_joint
(11,0)  {12} ; leg_r6_joint
)__";

        for (int i = 0; i < joint_pos.size(); ++i)
        {
            std::string placeholder = "{" + std::to_string(i + 1) + "}";
            size_t pos = outputString.find(placeholder);
            if (pos != std::string::npos)
            {
                outputString.replace(pos, placeholder.length(), std::to_string(joint_pos(i)));
            }

            pos = defaultJointState.find(placeholder);
            if (pos != std::string::npos)
            {
                defaultJointState.replace(pos, placeholder.length(), std::to_string(joint_pos[i]));
            }

            pos = base_str.find(placeholder);
            if (pos != std::string::npos)
            {
                base_str.replace(pos, placeholder.length(), std::to_string(base_vec(i)));
            }
        }
        // std::cout << "base_str: \n"
        //           << base_str << std::endl;
        // std::cout << "initialstate: \n"
        //           << outputString << std::endl;
        // std::cout << "defaultJointState: \n"
        //           << defaultJointState << std::endl;
        initial_state_.resize(12 + 12);
        default_joint_state_.resize(12);
        initial_state_ << Eigen::VectorXd::Zero(6), base_vec, joint_pos;
        default_joint_state_ << joint_pos;
        return q_initial_;
    }

    void HumanoidInterfaceDrake::calcSquatState(multibody::MultibodyPlant<double> *plant, systems::Context<double> *plant_context)
    {
        auto end_frames_name = kuavo_settings_.model_settings.end_frames_name;
        math::RigidTransformd foot_in_torso = plant->GetFrameByName(end_frames_name[1]).CalcPose(*plant_context, plant->GetFrameByName(end_frames_name[0]));
        math::RigidTransformd r_foot_in_torso = plant->GetFrameByName(end_frames_name[2]).CalcPose(*plant_context, plant->GetFrameByName(end_frames_name[0]));
        Eigen::Vector3d p0_foot(0, foot_in_torso.translation()[1], 0);
        Eigen::VectorXd q0 = plant->GetPositions(*plant_context);
        q0[6] = -foot_in_torso.translation()[2]; // 从动力学模型中测量出来的脚和躯干的高度（值为负），将躯提升到地面上这个高度
        plant->SetPositions(plant_context, q0);
        Eigen::VectorXd r0 = plant->CalcCenterOfMassPositionInWorld(*plant_context);

        std::cout << "foot width: " << foot_in_torso.translation()[1] - r_foot_in_torso.translation()[1] << std::endl;

        std::vector<std::string> initial_joint_name{"leg_l4_joint", "leg_r4_joint"};
        std::vector<double> initial_joint_pos{0.4, 0.4};
        for (uint32_t i = 0; i < initial_joint_name.size(); i++)
        {
            const multibody::RevoluteJoint<double> &joint = plant->GetJointByName<multibody::RevoluteJoint>(initial_joint_name[i]);
            joint.set_angle(plant_context, initial_joint_pos[i]);
        }
        q0 = plant->GetPositions(*plant_context);

        CoMIK ik(plant, end_frames_name, 1e-6, 1e-6);
        JSONConfigReader *robot_config_ptr = getRobotConfig();
        auto torsoY = robot_config_ptr->getValue<double>("torsoY") * TO_RADIAN;
        auto defaultHeight = robot_config_ptr->getValue<double>("squat_z");
        Eigen::VectorXd q;
        while(1)
        {
            static double externHeight = 0;
            std::vector<std::vector<Eigen::Vector3d>> pose_vec{
                {Eigen::Vector3d(0, robot_config_ptr->getValue<double>("torsoP_squat") * TO_RADIAN, torsoY), Eigen::Vector3d(0, 0, externHeight + defaultHeight)},
                {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(p0_foot[0], robot_config_ptr->getValue<double>("StepWith") / 2, p0_foot[2])},
                {Eigen::Vector3d(0, 0, torsoY), Eigen::Vector3d(p0_foot[0], -robot_config_ptr->getValue<double>("StepWith") / 2, p0_foot[2])}};
            bool result = ik.solve(pose_vec, q0, q);
            if (result == false)
            {
                externHeight += 0.002;
                // std::cout << "squat joint state solve fail " << "\n";
                // std::cout << "now torso height is " << externHeight + defaultHeight << std::endl;
            }else{
                std::cout << "squat joint state solve successfull!!! " << "\n";
                std::cout << "squat state is " << q.transpose() << std::endl;
                std::cout << "squat_z should be bigger than " << externHeight + defaultHeight << std::endl;
                break;
            }
        }

        Eigen::VectorXd base_vec(6);
        base_vec << q.segment(4, 3), 0, robot_config_ptr->getValue<double>("torsoP_squat") * TO_RADIAN, 0;
        Eigen::VectorXd joint_pos(12);
        joint_pos << q.segment(7, 12);

        q_squat_initial_ = q;
        squat_initial_state_.resize(12 + 12);
        squat_initial_state_ << Eigen::VectorXd::Zero(6), base_vec, joint_pos;
        default_joint_state_.resize(12);
        default_joint_state_ << joint_pos;
    }
} // namespace HighlyDynamic
