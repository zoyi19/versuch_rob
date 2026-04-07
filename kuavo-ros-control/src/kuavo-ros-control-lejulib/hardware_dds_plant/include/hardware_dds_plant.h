#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <memory>
#include <signal.h>
#include <cstdlib>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <array>

// DDS includes
#include <dds/dds.hpp>
#include <dds/sub/ddssub.hpp>
#include <dds/pub/ddspub.hpp>

// Hardware plant includes
#include "hardware_plant.h"

// Unitree SDK DDS types
#include "unitree/idl/hg/LowCmd_.hpp"
#include "unitree/idl/hg/LowState_.hpp"

// Leju DDS types
#include "leju/idl/JointCmd.hpp"
#include "leju/idl/SensorsData.hpp"

using namespace HighlyDynamic;
using namespace org::eclipse::cyclonedds;

// Constants
constexpr double DEFAULT_CONTROL_FREQUENCY = 500.0; // Hz
constexpr size_t DDS_MOTOR_COUNT = 35; // Based on LowCmd/LowState arrays
constexpr size_t KUAVO_JOINT_COUNT = 28; // Based on hardware requirements
constexpr uint32_t CRC_POLYNOMIAL = 0x04c11db7;
constexpr auto CONTROL_LOOP_PERIOD = std::chrono::microseconds(2000); // 1kHz

// Topics
static const std::string DDS_CMD_TOPIC = "rt/lowcmd";
static const std::string DDS_STATE_TOPIC = "rt/lowstate";

// CRC calculation utility (from G1 example)
inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = CRC_POLYNOMIAL;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// Low Command Listener
template<typename CmdType>
class LowCmdListener : public dds::sub::NoOpDataReaderListener<CmdType> {
public:
    explicit LowCmdListener(std::function<void(const CmdType&)> callback);
    void on_data_available(dds::sub::DataReader<CmdType>& reader) override;

private:
    std::function<void(const CmdType&)> callback_;
};

/**
 * HardwareDdsPlant - Hardware Plant with integrated DDS communication
 *
 * This class inherits from HardwarePlant and adds DDS communication capabilities.
 * Users can use all original HardwarePlant APIs while automatically getting:
 * - CmdType subscription
 * - StateType publishing
 * - Real-time communication threads
 *
 * Usage:
 *   HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_> plant(dt, hardware_param, hardware_path);
 *   plant.HWPlantInit();  // Original HardwarePlant method
 *   plant.startDds();     // Start DDS communication
 *
 *   // Use any HardwarePlant method:
 *   plant.readSensor(sensor_data);
 *   plant.writeCommand(cmd, na, modes, kp, kd);
 *   plant.jointMoveTo(goal_pos, speed);
 */
template<typename CmdType, typename StateType>
class HardwareDdsPlant : public HardwarePlant {
protected:
    // DDS entities
    dds::domain::DomainParticipant participant_;
    dds::topic::Topic<CmdType> dds_cmd_topic_;
    dds::topic::Topic<StateType> dds_state_topic_;
    dds::sub::DataReader<CmdType> dds_cmd_reader_;
    dds::pub::DataWriter<StateType> dds_state_writer_;

    // DDS command thread
    std::thread command_subscribe_thread_;
    std::atomic<bool> dds_running_;

    // Synchronization
    mutable std::mutex command_mutex_;

    // Command handling
    CmdType last_dds_cmd_;
    std::atomic<bool> has_new_cmd_;
    std::unique_ptr<LowCmdListener<CmdType>> dds_cmd_listener_;
    
    // Performance counters
    std::atomic<uint64_t> cmd_count_;
    std::atomic<uint64_t> state_count_;
    
    // DDS ready state
    std::atomic<bool> dds_ready_;

public:
    /**
     * Constructor - passes parameters to HardwarePlant base class
     */
    explicit HardwareDdsPlant(double dt = 1.0 / DEFAULT_CONTROL_FREQUENCY,
                             HardwareParam hardware_param = HardwareParam(),
                             const std::string& hardware_abs_path = "",
                             uint8_t control_mode = MOTOR_CONTROL_MODE_TORQUE,
                             uint16_t num_actuated = 0,
                             uint16_t nq_f = 7,
                             uint16_t nv_f = 6);
    
    virtual ~HardwareDdsPlant();
    
    // State publishing API (call this to publish state data via DDS)
    void publishStateViaDDS(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec);
    void publishStateViaDDS();
    
    // Check if DDS is ready
    bool isDdsReady() const { return dds_ready_.load(); }
    
    // DDS statistics and control
    void stopDds();
    void printDdsStatistics() const;
    uint64_t getCommandCount() const { return cmd_count_.load(); }
    uint64_t getStateCount() const { return state_count_.load(); }

protected:
    // Methods accessible to derived classes
    void onLowCmdReceived(const CmdType& cmd);
    void setThreadPriority(std::thread& thread, int priority);
    uint64_t getCurrentTimestampNs() const;

private:
    // DDS initialization and control (called automatically)
    bool initializeDds();
    void startDds();
    void setupDdsEntities();
    void setupCommandListener();
    
    // Thread functions
    void commandSubscribeThreadFunc();
    
    // Command processing
    bool isValidLowCommand(const CmdType& cmd) const;
    void processDdsCommand();
    void convertToHardwareCommand(const CmdType& dds_cmd,
                                 Eigen::VectorXd& cmd_out,
                                 std::vector<int>& control_modes,
                                 Eigen::VectorXd& joint_kp,
                                 Eigen::VectorXd& joint_kd);
    
    // State data publishing (internal implementation)
    void publishStateInternal(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec);
    void convertSensorDataToState(const SensorData_t& sensor_data,
                                 StateType& dds_state,
                                 uint32_t timestamp_sec, uint32_t timestamp_nsec);
};

// Template specialization for Leju DDS
// Template specialization for Unitree DDS
template<>
void HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::convertToHardwareCommand(const unitree_hg::msg::dds_::LowCmd_& dds_cmd,
                                                                                                                  Eigen::VectorXd& cmd_out,
                                                                                                                  std::vector<int>& control_modes,
                                                                                                                  Eigen::VectorXd& joint_kp,
                                                                                                                  Eigen::VectorXd& joint_kd);

template<>
void HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::convertSensorDataToState(const SensorData_t& sensor_data,
                                                                                                                  unitree_hg::msg::dds_::LowState_& dds_state,
                                                                                                                  uint32_t timestamp_sec, uint32_t timestamp_nsec);

template<>
bool HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::isValidLowCommand(const unitree_hg::msg::dds_::LowCmd_& cmd) const;

template<>
void HardwareDdsPlant<unitree_hg::msg::dds_::LowCmd_, unitree_hg::msg::dds_::LowState_>::publishStateInternal(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec);


// Template specialization for Leju DDS
template<>
void HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::convertToHardwareCommand(const leju::msgs::JointCmd& dds_cmd,
                                                                                        Eigen::VectorXd& cmd_out,
                                                                                        std::vector<int>& control_modes,
                                                                                        Eigen::VectorXd& joint_kp,
                                                                                        Eigen::VectorXd& joint_kd);

template<>
void HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::convertSensorDataToState(const SensorData_t& sensor_data,
                                                                                        leju::msgs::SensorsData& dds_state,
                                                                                        uint32_t timestamp_sec, uint32_t timestamp_nsec);


template<>
void HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::publishStateInternal(const SensorData_t& sensor_data, uint32_t timestamp_sec, uint32_t timestamp_nsec);

template<>
bool HardwareDdsPlant<leju::msgs::JointCmd, leju::msgs::SensorsData>::isValidLowCommand(const leju::msgs::JointCmd& cmd) const;
