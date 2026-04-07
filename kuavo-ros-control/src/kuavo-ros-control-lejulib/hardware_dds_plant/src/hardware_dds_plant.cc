#include "hardware_dds_plant.h"

using namespace HighlyDynamic;

// LowCmdListener template implementation
template<typename CmdType>
LowCmdListener<CmdType>::LowCmdListener(std::function<void(const CmdType&)> callback)
    : callback_(std::move(callback)) {}

template<typename CmdType>
void LowCmdListener<CmdType>::on_data_available(dds::sub::DataReader<CmdType>& reader) {
    auto samples = reader.read();
    for (const auto& sample : samples) {
        if (sample.info().valid()) {
            callback_(sample.data());
        }
    }
}

// HardwareDdsPlant template implementation
template<typename CmdType, typename StateType>
HardwareDdsPlant<CmdType, StateType>::HardwareDdsPlant(double dt,
                                                     HardwareParam hardware_param,
                                                     const std::string& hardware_abs_path,
                                                     uint8_t control_mode,
                                                     uint16_t num_actuated,
                                                     uint16_t nq_f,
                                                     uint16_t nv_f)
    : HardwarePlant(dt, hardware_param, hardware_abs_path, control_mode, num_actuated, nq_f, nv_f)
    , participant_(org::eclipse::cyclonedds::domain::default_id())
    , dds_cmd_topic_(participant_, DDS_CMD_TOPIC)
    , dds_state_topic_(participant_, DDS_STATE_TOPIC)
    , dds_cmd_reader_(dds::sub::Subscriber(participant_), dds_cmd_topic_)
    , dds_state_writer_(dds::pub::Publisher(participant_), dds_state_topic_)
    , dds_running_(false)
    , has_new_cmd_(false)
    , cmd_count_(0)
    , state_count_(0)
    , dds_ready_(false)
{
    // Print DDS type information
    #ifdef USE_DDS
    std::cout << "\033[33mInitializing HardwareDdsPlant with Unitree DDS communication\033[0m" << std::endl;
    #endif

    #ifdef USE_LEJU_DDS
    std::cout << "\033[33mInitializing HardwareDdsPlant with LEJU DDS communication\033[0m" << std::endl;
    #endif

    // Initialize and start DDS communication in constructor
    try {
        if (initializeDds()) {
            startDds();
            dds_ready_.store(true);

            #ifdef USE_DDS
            std::cout << "\033[33mHardwareDdsPlant created with Unitree DDS communication ready\033[0m" << std::endl;
            #endif

            #ifdef USE_LEJU_DDS
            std::cout << "\033[33mHardwareDdsPlant created with LEJU DDS communication ready\033[0m" << std::endl;
            #endif
        } else {
            std::cerr << "Warning: DDS initialization failed in constructor" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Warning: DDS setup failed in constructor: " << e.what() << std::endl;
    }
}

template<typename CmdType, typename StateType>
HardwareDdsPlant<CmdType, StateType>::~HardwareDdsPlant() {
    stopDds();
}

template<typename CmdType, typename StateType>
bool HardwareDdsPlant<CmdType, StateType>::initializeDds() {
    try {
        setupDdsEntities();
        setupCommandListener();
        std::cout << "DDS entities initialized successfully" << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "DDS initialization failed: " << e.what() << std::endl;
        return false;
    }
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::startDds() {
    if (dds_running_.exchange(true)) {
        std::cerr << "DDS threads already running" << std::endl;
        return;
    }

    // Start DDS command subscription thread
    command_subscribe_thread_ = std::thread(&HardwareDdsPlant<CmdType, StateType>::commandSubscribeThreadFunc, this);
    setThreadPriority(command_subscribe_thread_, 70);

    std::cout << "DDS command subscription started" << std::endl;
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::stopDds() {
    if (!dds_running_.exchange(false)) {
        return;
    }
    
    // Join command thread
    if (command_subscribe_thread_.joinable()) {
        command_subscribe_thread_.join();
    }
    
    std::cout << "DDS communication stopped" << std::endl;
    printDdsStatistics();
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::publishStateViaDDS() {
    // Only publish if DDS is ready
    if (!dds_ready_.load()) {
        return;
    }
    
    try {
        SensorData_t sensor_data_motor, sensor_data_joint;
        
        // Read current sensor data using parent class method
        if (!HardwarePlant::readSensor(sensor_data_motor)) {
            return;
        }
        
        // Convert to joint space using parent class method
        HardwarePlant::motor2joint(sensor_data_motor, sensor_data_joint);
        
        // Get current timestamp
        uint64_t timestamp = getCurrentTimestampNs();
        uint32_t timestamp_sec = timestamp / 1000000000;
        uint32_t timestamp_nsec = timestamp % 1000000000;
        
        // Publish via DDS
        publishStateViaDDS(sensor_data_joint, timestamp_sec, timestamp_nsec);
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading sensor data for DDS: " << e.what() << std::endl;
    }
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::publishStateViaDDS(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec) {
    // Only publish if DDS is ready
    if (!dds_ready_.load()) {
        return;
    }

    publishStateInternal(sensor_data, timestamp_sec, timestamp_nsec);
}


template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::processDdsCommand() {
    // Only process commands if we have new commands
    if (!has_new_cmd_.load(std::memory_order_acquire)) {
        return;
    }

    CmdType dds_cmd;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        dds_cmd = last_dds_cmd_;
    }

    try {
        if (!isValidLowCommand(dds_cmd)) {
            std::cerr << "Invalid low command received" << std::endl;
            return;
        }

        Eigen::VectorXd cmd_output;
        std::vector<int> control_modes;
        Eigen::VectorXd joint_kp, joint_kd;

        // Convert DDS command to hardware format
        convertToHardwareCommand(dds_cmd, cmd_output, control_modes, joint_kp, joint_kd);

        // Execute command using parent class method
        uint32_t num_joint = cmd_output.size() / 5;
        HardwarePlant::writeCommand(cmd_output, num_joint, control_modes, joint_kp, joint_kd);

        has_new_cmd_.store(false, std::memory_order_release);

    } catch (const std::exception& e) {
        std::cerr << "Error processing DDS command: " << e.what() << std::endl;
    }
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::setupDdsEntities() {
    // DDS entities are already created in constructor initializer list
    std::cout << "DDS entities set up on topics: " << DDS_CMD_TOPIC
              << ", " << DDS_STATE_TOPIC << std::endl;
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::setupCommandListener() {
    dds_cmd_listener_ = std::make_unique<LowCmdListener<CmdType>>(
        [this](const CmdType& cmd) {
            onLowCmdReceived(cmd);
        }
    );

    dds_cmd_reader_.listener(dds_cmd_listener_.get(),
                            dds::core::status::StatusMask::data_available());
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::commandSubscribeThreadFunc() {
    std::cout << "DDS command subscribe thread started" << std::endl;

    auto next_wake = std::chrono::steady_clock::now();

    while (dds_running_.load()) {
        // Process any new DDS commands
        processDdsCommand();

        // Maintain control frequency
        next_wake += CONTROL_LOOP_PERIOD;
        std::this_thread::sleep_until(next_wake);
    }

    std::cout << "DDS command subscribe thread stopped" << std::endl;
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::onLowCmdReceived(const CmdType& cmd) {
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_dds_cmd_ = cmd;
    }
    has_new_cmd_.store(true, std::memory_order_release);
    cmd_count_.fetch_add(1, std::memory_order_relaxed);
}

// Template specialization for Unitree DDS - isValidLowCommand
template<>
bool HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::isValidLowCommand(const unitree_hg::msg::dds_::LowCmd_& cmd) const {
    // Validate CRC
    uint32_t calculated_crc = Crc32Core((uint32_t*)&cmd,
                                       (sizeof(cmd) >> 2) - 1);
    if (cmd.crc() != calculated_crc) {
        std::cerr << "DDS command CRC validation failed" << std::endl;
        return false;
    }

    // Check motor command array size
    if (cmd.motor_cmd().size() != DDS_MOTOR_COUNT) {
        std::cerr << "Invalid DDS motor command array size" << std::endl;
        return false;
    }

    return true;
}

// Template specialization for Unitree DDS - convertToHardwareCommand
template<>
void HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::convertToHardwareCommand(const unitree_hg::msg::dds_::LowCmd_& dds_cmd,
                                                                                                                  Eigen::VectorXd& cmd_out,
                                                                                                                  std::vector<int>& control_modes,
                                                                                                                  Eigen::VectorXd& joint_kp,
                                                                                                                  Eigen::VectorXd& joint_kd) {
    try {
        const auto& motor_cmds = dds_cmd.motor_cmd();
        uint32_t num_joint = KUAVO_JOINT_COUNT;
        
        // Prepare output vectors
        Eigen::VectorXd cmd_input(num_joint * 5);
        cmd_out.resize(num_joint * 5);
        joint_kp.resize(num_joint);
        joint_kd.resize(num_joint);
        control_modes.clear();
        control_modes.reserve(num_joint);
        
        // Convert from DDS motor commands to joint commands
        // Focus on motors 0-27 (first 28 motors) from DDS IDL
        for (uint32_t kuavo_idx = 0; kuavo_idx < num_joint; ++kuavo_idx) {
            // Direct mapping: Kuavo joint i -> DDS motor i (0-27)
            if (kuavo_idx < motor_cmds.size()) {
                const auto& motor_cmd = motor_cmds[kuavo_idx];
                
                joint_kp(kuavo_idx) = static_cast<double>(motor_cmd.kp());
                joint_kd(kuavo_idx) = static_cast<double>(motor_cmd.kd());
                
                int control_mode = static_cast<int>(motor_cmd.mode());
                control_modes.push_back(control_mode);
                
                cmd_input[num_joint * 0 + kuavo_idx] = static_cast<double>(motor_cmd.q());     // position
                cmd_input[num_joint * 1 + kuavo_idx] = static_cast<double>(motor_cmd.dq());   // velocity
                cmd_input[num_joint * 2 + kuavo_idx] = static_cast<double>(motor_cmd.tau());  // torque
                cmd_input[num_joint * 3 + kuavo_idx] = 1000.0; // tau_max - default
                cmd_input[num_joint * 4 + kuavo_idx] = 1.0;    // tau_ratio - default
            } else {
                // Fallback for missing motor data - set to safe values
                joint_kp(kuavo_idx) = 0.0;
                joint_kd(kuavo_idx) = 0.0;
                control_modes.push_back(2); // Default control mode
                
                cmd_input[num_joint * 0 + kuavo_idx] = 0.0;
                cmd_input[num_joint * 1 + kuavo_idx] = 0.0;
                cmd_input[num_joint * 2 + kuavo_idx] = 0.0;
                cmd_input[num_joint * 3 + kuavo_idx] = 1000.0;
                cmd_input[num_joint * 4 + kuavo_idx] = 1.0;
            }
        }
        
        // Use HardwarePlant's command processing
        cmds2Cmdr(cmd_input, num_joint, cmd_out, num_joint);
        
        // Handle NaN values
        if (cmd_out.hasNaN()) {
            std::cerr << "Command output has NaN values, setting to zero" << std::endl;
            cmd_out.setZero();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error converting Unitree DDS to hardware command: " << e.what() << std::endl;
        cmd_out.setZero();
        control_modes.clear();
        control_modes.resize(KUAVO_JOINT_COUNT, 2);
        joint_kp.setZero();
        joint_kd.setZero();
    }
}

// Template specialization for Unitree DDS - convertSensorDataToState
template<>
void HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::convertSensorDataToState(const SensorData_t& sensor_data,
                                                                                                                  unitree_hg::msg::dds_::LowState_& dds_state,
                                                                                                                  uint32_t timestamp_sec, uint32_t timestamp_nsec) {
    // Set timestamp using reserve fields to avoid uint32_t overflow
    // Use reserve fields for timestamp: [0]=seconds, [1]=nanoseconds
    dds_state.reserve()[0] = timestamp_sec;
    dds_state.reserve()[1] = timestamp_nsec;
    
    // Keep tick as a simple counter or relative timestamp
    static uint32_t tick_counter = 0;
    dds_state.tick(++tick_counter);
    
    // Initialize ALL 35 motor states with zeros first
    for (size_t motor_idx = 0; motor_idx < DDS_MOTOR_COUNT; ++motor_idx) {
        auto& motor_state = dds_state.motor_state()[motor_idx];
        motor_state.mode(0);           // Disabled mode
        motor_state.q(0.0f);           // Zero position
        motor_state.dq(0.0f);          // Zero velocity
        motor_state.ddq(0.0f);         // Zero acceleration
        motor_state.tau_est(0.0f);     // Zero torque
        motor_state.temperature({{0, 0}}); // Zero temperature
        motor_state.vol(0.0f);         // Zero voltage
        motor_state.sensor({{0, 0}});  // Zero sensor
        motor_state.motorstate(0);     // Zero motor state
    }
    
    // Fill motors 0-27 with actual Kuavo joint data
    for (size_t kuavo_idx = 0; kuavo_idx < KUAVO_JOINT_COUNT; ++kuavo_idx) {
        // Direct mapping: Kuavo joint i -> DDS motor i (0-27)
        auto& motor_state = dds_state.motor_state()[kuavo_idx];
        
        motor_state.mode(1); // Enable mode
        motor_state.q(static_cast<float>(sensor_data.joint_q(kuavo_idx)));
        motor_state.dq(static_cast<float>(sensor_data.joint_v(kuavo_idx)));
        motor_state.ddq(static_cast<float>(sensor_data.joint_vd(kuavo_idx)));
        motor_state.tau_est(static_cast<float>(sensor_data.joint_current(kuavo_idx))); // Use actual torque from joint_current
        motor_state.temperature({{0, 0}}); // TODO: Add temperature data if available
        motor_state.vol(0.0f); // TODO: Add voltage data if available
        motor_state.sensor({{0, 0}}); // TODO: Add sensor data if available
        motor_state.motorstate(0); // TODO: Add motor state flags if available
    }
    
    // Set IMU data
    dds_state.imu_state().quaternion()[0] = static_cast<float>(sensor_data.quat.x());
    dds_state.imu_state().quaternion()[1] = static_cast<float>(sensor_data.quat.y());
    dds_state.imu_state().quaternion()[2] = static_cast<float>(sensor_data.quat.z());
    dds_state.imu_state().quaternion()[3] = static_cast<float>(sensor_data.quat.w());
    
    dds_state.imu_state().gyroscope()[0] = static_cast<float>(sensor_data.gyro.x());
    dds_state.imu_state().gyroscope()[1] = static_cast<float>(sensor_data.gyro.y());
    dds_state.imu_state().gyroscope()[2] = static_cast<float>(sensor_data.gyro.z());
    
    dds_state.imu_state().accelerometer()[0] = static_cast<float>(sensor_data.acc.x());
    dds_state.imu_state().accelerometer()[1] = static_cast<float>(sensor_data.acc.y());
    dds_state.imu_state().accelerometer()[2] = static_cast<float>(sensor_data.acc.z());
    
    // Calculate RPY from quaternion (using free_acc temporarily)
    dds_state.imu_state().rpy()[0] = static_cast<float>(sensor_data.free_acc.x());
    dds_state.imu_state().rpy()[1] = static_cast<float>(sensor_data.free_acc.y());
    dds_state.imu_state().rpy()[2] = static_cast<float>(sensor_data.free_acc.z());
    
    dds_state.imu_state().temperature(static_cast<int16_t>(0)); // TODO: Add IMU temperature if available
    
    // Initialize wireless remote
    std::fill(dds_state.wireless_remote().begin(), dds_state.wireless_remote().end(), 0);
    
    // Note: reserve[0] and reserve[1] are used for timestamp, rest can be initialized to 0
    for (size_t i = 2; i < dds_state.reserve().size(); ++i) {
        dds_state.reserve()[i] = 0;
    }
}

// Template specialization for Unitree DDS - publishStateInternal
template<>
void HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::publishStateInternal(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec) {
    try {
        unitree_hg::msg::dds_::LowState_ dds_state;
        convertSensorDataToState(sensor_data, dds_state, timestamp_sec, timestamp_nsec);

        // Calculate and set CRC (Unitree DDS requires CRC calculation)
        dds_state.crc(Crc32Core((uint32_t*)&dds_state,
                               (sizeof(dds_state) >> 2) - 1));

        dds_state_writer_.write(dds_state);
        state_count_.fetch_add(1, std::memory_order_relaxed);

    } catch (const std::exception& e) {
        std::cerr << "Error publishing Unitree DDS state: " << e.what() << std::endl;
    }
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::setThreadPriority(std::thread& thread, int priority) {
#ifdef __linux__
    try {
        pthread_t native_handle = thread.native_handle();
        struct sched_param param;
        param.sched_priority = priority;

        if (pthread_setschedparam(native_handle, SCHED_FIFO, &param) != 0) {
            std::cerr << "Warning: Could not set thread priority to " << priority << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Warning: Thread priority setting failed: " << e.what() << std::endl;
    }
#endif
}

template<typename CmdType, typename StateType>
uint64_t HardwareDdsPlant<CmdType, StateType>::getCurrentTimestampNs() const {
    const auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

template<typename CmdType, typename StateType>
void HardwareDdsPlant<CmdType, StateType>::printDdsStatistics() const {
    const uint64_t cmd_total = cmd_count_.load();
    const uint64_t state_total = state_count_.load();
    
      std::cout << "DDS Performance Statistics:" << std::endl;
    std::cout << "  DDS commands received: " << cmd_total << std::endl;
    std::cout << "  DDS state messages published: " << state_total << std::endl;
    std::cout << "  Average command rate: " << (cmd_total > 0 ? cmd_total / 60.0 : 0.0) << " Hz" << std::endl;
    std::cout << "  Average state rate: " << (state_total > 0 ? state_total / 60.0 : 0.0) << " Hz" << std::endl;
}

// Template specialization for Leju DDS - convertToHardwareCommand
template<>
void HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::convertToHardwareCommand(const leju::msgs::JointCmd& dds_cmd,
                                                                                        Eigen::VectorXd& cmd_out,
                                                                                        std::vector<int>& control_modes,
                                                                                        Eigen::VectorXd& joint_kp,
                                                                                        Eigen::VectorXd& joint_kd) {
    try {
        uint32_t num_joint = dds_cmd.joint_q().size();
        if (num_joint == 0) {
            std::cerr << "Leju JointCmd has empty joint data" << std::endl;
            return;
        }

        // Check dimension consistency - all vectors must have the same size
        if (dds_cmd.joint_v().size() != num_joint ||
            dds_cmd.tau().size() != num_joint ||
            dds_cmd.tau_max().size() != num_joint ||
            dds_cmd.tau_ratio().size() != num_joint ||
            dds_cmd.joint_kp().size() != num_joint ||
            dds_cmd.joint_kd().size() != num_joint ||
            dds_cmd.control_modes().size() != num_joint) {
            std::cerr << "Leju JointCmd vector dimensions are inconsistent" << std::endl;
            return;
        }

        // Prepare vectors only after dimension check passes
        Eigen::VectorXd cmd_input(num_joint * 5);
        cmd_out.resize(num_joint * 5);
        joint_kp.resize(num_joint);
        joint_kd.resize(num_joint);
        control_modes.resize(num_joint);

        // Convert from Leju JointCmd to hardware format - direct assignment without defaults
        for (uint32_t i = 0; i < num_joint; ++i) {
            joint_kp(i) = dds_cmd.joint_kp()[i];
            joint_kd(i) = dds_cmd.joint_kd()[i];
            control_modes[i] = dds_cmd.control_modes()[i];

            cmd_input[num_joint * 0 + i] = dds_cmd.joint_q()[i];      // position
            cmd_input[num_joint * 1 + i] = dds_cmd.joint_v()[i];      // velocity
            cmd_input[num_joint * 2 + i] = dds_cmd.tau()[i];          // torque
            cmd_input[num_joint * 3 + i] = dds_cmd.tau_max()[i];     // tau_max
            cmd_input[num_joint * 4 + i] = dds_cmd.tau_ratio()[i];   // tau_ratio
        }

        // Use HardwarePlant's command processing
        cmds2Cmdr(cmd_input, num_joint, cmd_out, num_joint);

        // Handle NaN values
        if (cmd_out.hasNaN()) {
            std::cerr << "Command output has NaN values, setting to zero" << std::endl;
            cmd_out.setZero();
        }

    } catch (const std::exception& e) {
        std::cerr << "Error converting Leju DDS to hardware command: " << e.what() << std::endl;
        cmd_out.setZero();
        control_modes.clear();
        joint_kp.setZero();
        joint_kd.setZero();
    }
}

// Template specialization for Leju DDS - convertSensorDataToState
template<>
void HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::convertSensorDataToState(const SensorData_t& sensor_data,
                                                                                        leju::msgs::SensorsData& dds_state,
                                                                                        uint32_t timestamp_sec, uint32_t timestamp_nsec) {
    // Set timestamp
    dds_state.header_sec(timestamp_sec);
    dds_state.header_nanosec(timestamp_nsec);
    dds_state.sensor_time(getCurrentTimestampNs());

    // Convert joint data
    auto& joint_data = dds_state.joint_data();

    // Check dimension consistency for joint data
    size_t num_joints = sensor_data.joint_q.size();
    if (num_joints == 0 ||
        sensor_data.joint_v.size() != num_joints ||
        sensor_data.joint_vd.size() != num_joints ||
        sensor_data.joint_current.size() != num_joints) {
        std::cerr << "Joint data dimensions are inconsistent in SensorData_t" << std::endl;
        return;
    }

    // Assign to Leju JointData
    joint_data.joint_q().resize(num_joints);
    joint_data.joint_v().resize(num_joints);
    joint_data.joint_vd().resize(num_joints);
    joint_data.joint_torque().resize(num_joints);

    for (size_t i = 0; i < num_joints; ++i) {
        joint_data.joint_q()[i] = sensor_data.joint_q(i);
        joint_data.joint_v()[i] = sensor_data.joint_v(i);
        joint_data.joint_vd()[i] = sensor_data.joint_vd(i);
        joint_data.joint_torque()[i] = sensor_data.joint_current(i);
    }

    // Convert IMU data
    auto& imu_data = dds_state.imu_data();

    // Convert gyroscope [x, y, z]
    std::array<double, 3> gyro_array = {
        sensor_data.gyro.x(),
        sensor_data.gyro.y(),
        sensor_data.gyro.z()
    };
    imu_data.gyro(std::move(gyro_array));

    // Convert accelerometer [x, y, z]
    std::array<double, 3> acc_array = {
        sensor_data.acc.x(),
        sensor_data.acc.y(),
        sensor_data.acc.z()
    };
    imu_data.acc(std::move(acc_array));

    // Convert free acceleration [x, y, z]
    std::array<double, 3> free_acc_array = {
        sensor_data.free_acc.x(),
        sensor_data.free_acc.y(),
        sensor_data.free_acc.z()
    };
    imu_data.free_acc(std::move(free_acc_array));

    // Convert quaternion [w, x, y, z] -> [x, y, z, w] for Leju format
    std::array<double, 4> quat_array = {
        sensor_data.quat.x(),  // x
        sensor_data.quat.y(),  // y
        sensor_data.quat.z(),  // z
        sensor_data.quat.w()   // w
    };
    imu_data.quat(std::move(quat_array));
}


// Template specialization for Leju DDS - publishStateInternal
template<>
void HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::publishStateInternal(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec) {
    try {
        leju::msgs::SensorsData dds_state;
        convertSensorDataToState(sensor_data, dds_state, timestamp_sec, timestamp_nsec);

        // Leju DDS does not require CRC calculation
        // Just write the state directly
        dds_state_writer_.write(dds_state);
        state_count_.fetch_add(1, std::memory_order_relaxed);

    } catch (const std::exception& e) {
        std::cerr << "Error publishing Leju DDS state: " << e.what() << std::endl;
    }
}

// Template specialization for Leju DDS - isValidLowCommand
template<>
bool HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::isValidLowCommand(const leju::msgs::JointCmd& cmd) const {
    // Validate timestamp (basic check)
    if (cmd.header_sec() < 0 || cmd.header_nanosec() >= 1000000000) {
        std::cerr << "Invalid timestamp in JointCmd" << std::endl;
        return false;
    }

    // Check array sizes directly
    size_t expected_joint_count = KUAVO_JOINT_COUNT;
    size_t expected_size = cmd.joint_q().size();

    if (cmd.joint_v().size() != expected_size ||
        cmd.tau().size() != expected_size ||
        cmd.tau_max().size() != expected_size ||
        cmd.tau_ratio().size() != expected_size ||
        cmd.joint_kp().size() != expected_size ||
        cmd.joint_kd().size() != expected_size ||
        cmd.control_modes().size() != expected_size) {
        std::cerr << "Inconsistent array dimensions in JointCmd" << std::endl;
        return false;
    }

    // Check if the consistent size matches expected joint count
    if (expected_size != expected_joint_count) {
        std::cerr << "Invalid joint command array size: expected "
                  << expected_joint_count << ", got " << expected_size << std::endl;
        return false;
    }

    return true;
}

// Explicit template instantiation
template class HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>;
template class HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>;