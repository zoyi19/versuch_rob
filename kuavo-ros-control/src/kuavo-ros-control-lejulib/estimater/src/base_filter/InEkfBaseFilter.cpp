#include "kuavo_estimation/base_filter/InEkfBaseFilter.h"
namespace ocs2
{
  namespace humanoid
  {

    InEkfBaseFilter::InEkfBaseFilter(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                                     const PinocchioEndEffectorKinematics &eeKinematics,
                                     HighlyDynamic::HumanoidInterfaceDrake *drake_interface_ptr,
                                     double dt,
                                     ocs2::humanoid::TopicLogger *logger_ptr)
        : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics),
          drake_interface_ptr_(drake_interface_ptr),
          logger_ptr_(logger_ptr),
          dt_(dt)
    {
      auto [plant, context] = drake_interface_ptr_->getPlantAndContext();
      kuavo_settings_ = drake_interface_ptr_->getKuavoSettings();
      robot_config_ = drake_interface_ptr_->getRobotConfig();
      end_frames_name_ = kuavo_settings_.model_settings.end_frames_name;
      plant_ = &plant;
      plant_context_ = plant_->CreateDefaultContext();
      na_ = plant_->num_actuated_dofs();
      nq_ = plant_->num_positions();
      nv_ = plant_->num_velocities();
      contact_state_prev_des << 1, 1;
      contact_state_prev_est << 1, 1;
      use_contact_estimator = false;
    }
    void InEkfBaseFilter::set_intial_state(const vector_t &ocs2_state)
    {

      // this->updateJointStates();
      // this->updateLinear();
      // this->updateAngular();
      vector_t jointPos = ocs2_state.tail(info_.actuatedDofNum);
      vector_t jointVel = vector_t::Zero(info_.actuatedDofNum);
      this->updateJointStates(jointPos, jointVel);

      Eigen::Vector3d zyx = ocs2_state.segment(9, 3);
      Eigen::Quaterniond quaternion;
      quaternion = Eigen::AngleAxisd(zyx[2], Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(zyx[1], Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(zyx[0], Eigen::Vector3d::UnitX());
      Eigen::Vector4d quatd(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());

      inekf::RobotState initial_state;
      Eigen::VectorXd base_orientation(4);
      Eigen::Vector3d base_velocity, base_position, gyroscope_bias, accelerometer_bias;
      base_orientation << quatd;
      base_velocity.setZero();
      base_position << ocs2_state.segment(6, 3);
      this->updateLinear(base_position, base_velocity);
      this->updateAngular(zyx, vector_t::Zero(3));

      gyroscope_bias << robot_config_->getValue<Eigen::VectorXd>("gyroscope_bias");
      accelerometer_bias << robot_config_->getValue<Eigen::VectorXd>("accelerometer_bias");
      double base_orientation_std = robot_config_->getValue<double>("base_orientation_std");
      double base_velocity_std = robot_config_->getValue<double>("base_velocity_std");
      double base_position_std = robot_config_->getValue<double>("base_position_std");
      double gyroscope_bias_std = robot_config_->getValue<double>("gyroscope_bias_std");
      double accelerometer_bias_std = robot_config_->getValue<double>("accelerometer_bias_std");
      double ori_cov = base_orientation_std * base_orientation_std;
      double vel_cov = base_velocity_std * base_velocity_std;
      double pos_cov = base_position_std * base_position_std;
      double gyro_cov = gyroscope_bias_std * gyroscope_bias_std;
      double acc_cov = accelerometer_bias_std * accelerometer_bias_std;

      kinematics_noise.resize(6);
      stand_kinematics_noise.resize(6);
      Eigen::Vector3d gyroscope_noise, accelerometer_noise, gyroscope_bias_noise, accelerometer_bias_noise, contact_noise;
      gyroscope_noise = robot_config_->getValue<Eigen::VectorXd>("gyroscope_noise");
      accelerometer_noise = robot_config_->getValue<Eigen::VectorXd>("accelerometer_noise");
      gyroscope_bias_noise = robot_config_->getValue<Eigen::VectorXd>("gyroscope_bias_noise");
      accelerometer_bias_noise = robot_config_->getValue<Eigen::VectorXd>("accelerometer_bias_noise");
      contact_noise = robot_config_->getValue<Eigen::VectorXd>("contact_noise");
      kinematics_noise = robot_config_->getValue<Eigen::VectorXd>("kinematics_noise");
      stand_kinematics_noise = robot_config_->getValue<Eigen::VectorXd>("stand_kinematics_noise");
      contact_com_z_diff = robot_config_->getValue<Eigen::VectorXd>("contact_com_z_diff");
      use_contact_estimator = robot_config_->getValue<bool>("use_contact_estimator");
      noise_params.setGyroscopeNoise(gyroscope_noise);
      noise_params.setAccelerometerNoise(accelerometer_noise);
      noise_params.setGyroscopeBiasNoise(gyroscope_bias_noise);
      noise_params.setAccelerometerBiasNoise(accelerometer_bias_noise);
      noise_params.setContactNoise(contact_noise);
      inekf_filter.setNoiseParams(noise_params);
      Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
      Eigen::Matrix<double, 15, 15> P_init = Eigen::Matrix<double, 15, 15>::Zero();
      Eigen::Quaternion<double> q(base_orientation[0], base_orientation[1], base_orientation[2], base_orientation[3]);
      q.normalize();
      R_init = q.toRotationMatrix();
      P_init(0, 0) = ori_cov;
      P_init(1, 1) = ori_cov;
      P_init(2, 2) = ori_cov;
      P_init(3, 3) = ori_cov;
      P_init(4, 4) = ori_cov;
      P_init(5, 5) = vel_cov;
      P_init(6, 6) = pos_cov;
      P_init(7, 7) = pos_cov;
      P_init(8, 8) = pos_cov;
      P_init(9, 9) = gyro_cov;
      P_init(10, 10) = gyro_cov;
      P_init(11, 11) = gyro_cov;
      P_init(12, 12) = acc_cov;
      P_init(13, 13) = acc_cov;
      P_init(14, 14) = acc_cov;
      initial_state.setRotation(R_init);
      initial_state.setVelocity(base_velocity);
      initial_state.setPosition(base_position);
      initial_state.setGyroscopeBias(gyroscope_bias);
      initial_state.setAccelerometerBias(accelerometer_bias);
      initial_state.setP(P_init);
      inekf_filter.setState(initial_state);
      imu_measurement_prev = Eigen::Matrix<double, 6, 1>::Zero();
      std::cout << "Robot's state is initialized to: \n";
      std::cout << "R_init: " << R_init << std::endl;
      std::cout << "base_velocity: " << base_velocity.transpose() << std::endl;
      std::cout << "base_position: " << base_position.transpose() << std::endl;
      std::cout << inekf_filter.getState() << std::endl;
      std::cout << inekf_filter.getNoiseParams() << std::endl;
      updateMode(mode_);
      for (int i = 0; i < 6; i++)
      {
        this->UpdateQvInekf(rbdState_, mode_);
        std::cout << "update rbdState_:" << rbdState_.segment(0, 6).transpose() << std::endl;
      }
    }
    vector_t InEkfBaseFilter::update(const ros::Time &time, const ros::Duration &period)
    {
      this->UpdateQvInekf(rbdState_, mode_);
      return rbdState_;
    }

    void InEkfBaseFilter::UpdateContactState(bool contact_des, double diff_z, Eigen::Vector2d &contact_com_z_diff, bool &contact_est, int contact_id, std::vector<std::pair<int, bool>> &contacts)
    {
      if (contact_des) // 如果期望接触
      {
        contact_est = (diff_z < contact_com_z_diff[0] && diff_z >= 0);
      }
      else
      {
        contact_est = (diff_z >= contact_com_z_diff[1]);
      }
      contacts.push_back(std::make_pair(contact_id, contact_des));
    }

    void InEkfBaseFilter::UpdateQvInekf(Eigen::VectorXd &rbdState, const size_t &mode)
    {

      inekf::RobotState state = inekf_filter.getState();
      inekf::vectorKinematics measured_kinematics;
      Eigen::Quaternion<double> orientation;
      Eigen::Vector4d quat;
      Eigen::VectorXd qv(nq_ + nv_);
      Eigen::MatrixXd Jc(3, nv_);
      Eigen::MatrixXd Jc_lf(3, 6);
      Eigen::MatrixXd Jc_rf(3, 6);
      Eigen::Matrix<double, 6, 6> covariance;
      Eigen::Matrix<double, 6, 1> imu_measurement = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix4d kinematic_pose = Eigen::Matrix4d::Identity();
      Eigen::VectorXd kinematic_std(6);
      Eigen::Vector3d base_pose = state.getPosition();
      orientation = state.getRotation();
      orientation.normalize();
      quat << quat_.w(), quat_.x(), quat_.y(), quat_.z();
      logPublishVector("inekf_state/imu_quat", quat);
      qv << quat, state.getPosition(), jointPos_,
          angularVelWorld_, state.getVelocity(), jointVel_;
      kinematic_std = kinematics_noise;
      auto mContext = plant_context_.get();
      plant_->SetPositionsAndVelocities(mContext, qv);
      drake::math::RigidTransformd lf_in_base = plant_->GetFrameByName(end_frames_name_[END_FRAMES_L_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name_[END_FRAMES_TORSO]));
      drake::math::RigidTransformd rf_in_base = plant_->GetFrameByName(end_frames_name_[END_FRAMES_R_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name_[END_FRAMES_TORSO]));
      Eigen::Vector3d l_contact_pose = lf_in_base.translation();
      Eigen::Vector3d r_contact_pose = rf_in_base.translation();
      // imu
      imu_measurement << angularVelLocal_, linearAccelLocal_;
      // contact
      std::vector<std::pair<int, bool>> contacts;
      Eigen::Vector2d contact_state_des(1, 1);
      Eigen::Vector2d contact_state_est(1, 1);
      Eigen::Vector2d contact_z;
      Eigen::Vector2d diff_z;
      size_t num_contact = contactFlag_.size();
      bool L_contact_des = std::any_of(contactFlag_.begin(), contactFlag_.begin() + num_contact / 2, [](bool b)
                                       { return b; });
      bool R_contact_des = std::any_of(contactFlag_.begin() + num_contact / 2, contactFlag_.end(), [](bool b)
                                       { return b; });
      bool L_contact_est = false;
      bool R_contact_est = false;
      contact_state_des << L_contact_des, R_contact_des;
      contact_z << l_contact_pose[2], r_contact_pose[2];
      diff_z << base_pose[2] + l_contact_pose[2], base_pose[2] + r_contact_pose[2];
      UpdateContactState(contact_state_des[0], diff_z[0], contact_com_z_diff, L_contact_est, 0, contacts);
      UpdateContactState(contact_state_des[1], diff_z[1], contact_com_z_diff, R_contact_est, 1, contacts);
      // 更新接触状态估计
      contact_state_est = ((R_contact_est && L_contact_est) || (contact_state_des[0] && contact_state_des[1])) ? contact_state_des : contact_state_prev_est;
      if (use_contact_estimator)
      {
        for (size_t i = 0; i < 2; i++)
        {
          contacts.push_back(std::make_pair(i, contact_state_est(i)));
        }
      }
      contact_state_prev_des = contact_state_des;
      contact_state_prev_est = contact_state_est;
      // kinematics
      Eigen::Matrix<double, 6, 6> covM_66(kinematic_std.asDiagonal());
      // drake::math::RigidTransformd lf_in_base = plant_->GetFrameByName(end_frames_name_[END_FRAMES_L_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name_[END_FRAMES_TORSO]));
      const drake::math::RotationMatrix<double> &l_rotation_matrix = lf_in_base.rotation();
      const Eigen::Matrix3d &l_rotation_matrix_eigen = l_rotation_matrix.matrix();
      kinematic_pose.block<3, 3>(0, 0) = l_rotation_matrix_eigen;
      kinematic_pose.block<3, 1>(0, 3) = l_contact_pose;
      logPublishVector("inekf_state/lf_in_base", l_contact_pose);
      plant_->CalcJacobianTranslationalVelocity(*mContext, drake::multibody::JacobianWrtVariable::kV,
                                                plant_->GetFrameByName(end_frames_name_[END_FRAMES_L_FOOT_SOLE]), Eigen::Vector3d::Zero(),
                                                plant_->world_frame(), plant_->world_frame(), &Jc);
      Jc_lf = Jc.block<3, 6>(0, 6);
      covariance.block<3, 3>(3, 3) = Jc_lf * covM_66 * Jc_lf.transpose();
      inekf::Kinematics l_frame(0, kinematic_pose, covariance);
      measured_kinematics.push_back(l_frame);

      // drake::math::RigidTransformd rf_in_base = plant_->GetFrameByName(end_frames_name_[END_FRAMES_R_FOOT_SOLE]).CalcPose(*plant_context_.get(), plant_->GetFrameByName(end_frames_name_[END_FRAMES_TORSO]));
      const drake::math::RotationMatrix<double> &r_rotation_matrix = rf_in_base.rotation();
      const Eigen::Matrix3d &r_rotation_matrix_eigen = r_rotation_matrix.matrix();
      kinematic_pose.block<3, 3>(0, 0) = r_rotation_matrix_eigen;
      kinematic_pose.block<3, 1>(0, 3) = r_contact_pose;
      logPublishVector("inekf_state/rf_in_base", r_contact_pose);
      plant_->CalcJacobianTranslationalVelocity(*mContext, drake::multibody::JacobianWrtVariable::kV,
                                                plant_->GetFrameByName(end_frames_name_[END_FRAMES_R_FOOT_SOLE]), Eigen::Vector3d::Zero(),
                                                plant_->world_frame(), plant_->world_frame(), &Jc);
      Jc_rf = Jc.block<3, 6>(0, 12);
      covariance.block<3, 3>(3, 3) = Jc_rf * covM_66 * Jc_rf.transpose();
      inekf::Kinematics r_frame(1, kinematic_pose, covariance);
      measured_kinematics.push_back(r_frame);

      inekf_filter.Propagate(imu_measurement_prev, dt_);
      inekf_filter.setContacts(contacts);
      inekf_filter.CorrectKinematics(measured_kinematics);
      imu_measurement_prev = imu_measurement;

      state = inekf_filter.getState();
      qv << quat, state.getPosition(), jointPos_,
          angularVelWorld_, state.getVelocity(), jointVel_;
      Eigen::Vector3d l_contact_point;
      Eigen::Vector3d r_contact_point;
      Eigen::MatrixXd X = state.getX();
      std::map<int,int> estimated_contacts = inekf_filter.getEstimatedContactPositions();
      for (const auto& pair : estimated_contacts) {
        int id = pair.first;
        int index = pair.second;
        if (id == 0)
        {
          l_contact_point << X(0, index), X(1, index), X(2, index);
        }
        else if (id == 1)
        {
          r_contact_point << X(0, index), X(1, index), X(2, index);
        }
      }
    
      // state_est.q = qv.segment(0, nq_);
      // state_est.v = qv.segment(nq_, nv_);
      this->updateLinear(state.getPosition(), state.getVelocity());
      orientation = state.getRotation();
      orientation.normalize();
      quat << orientation.w(), orientation.x(), orientation.y(), orientation.z();
      logPublishVector("inekf_state/est/acc", imu_measurement.segment(3, 3));
      logPublishVector("inekf_state/est/pose", state.getPosition());
      logPublishVector("inekf_state/est/orientation", quat);
      logPublishVector("inekf_state/est/velocity", state.getVelocity());
      logPublishVector("inekf_state/est/angular_velocity", state.getAngularVelocity());
      logPublishVector("inekf_state/est/contact/contact_state_est", contact_state_est);
      logPublishVector("inekf_state/est/contact/contact_state_des", contact_state_des);
      logPublishVector("inekf_state/est/contact/l_point", l_contact_point);
      logPublishVector("inekf_state/est/contact/r_point", r_contact_point);
      logPublishVector("inekf_state/est/contact/diff_z", diff_z);
    }

  } // namespace humanoid
} // namespace ocs2
