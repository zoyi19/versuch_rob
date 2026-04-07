#include <ros/ros.h>
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <atomic>
#include <time.h>
#include <iomanip>
#include <cmath>
#include "utils.h"

#include "kuavo_solver/ankle_solver.h"
#include "imu_receiver.h"
#include "hipnuc_imu_receiver.h"
#include "touch_hand_controller.h"
#include "actuators_interface.h"
#include "lejuclaw_controller.h"
#include "ruiwo_actuator.h"
#include "kuavo_common/kuavo_common.h"

using namespace HighlyDynamic;

const int kEcMasterNilId = 254;

class hardwareSelfCheck
{
private:
struct RuiWoJointData
{
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> torque;
};
public:
    hardwareSelfCheck(double dt, ros::NodeHandle nh);
    ~hardwareSelfCheck();

    bool initMotor();
    bool initIMU();
    bool initEndEffector();

    inline std::vector<uint8_t> getJointIds()
    {
        return joint_ids_;
    }
    
    inline std::vector<double> getCurJointPos()
    {
        std::vector<double> curPos(num_joint_);
        for(int i = 0;i < num_joint_;i++)
        {
            curPos[i] = joint_data_[i].position;
        }
        return curPos;
    }

    void GetJointData(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);
    bool jointMoveTo(std::vector<double> goal_pos, double speed, double dt, double current_limit = -1);
    bool waitMoveDone(const std::vector<double> &tarPos);

private:
    void loadConfig();
    bool detectYawDrift();

    //move
    void SetJointPosition(const std::vector<uint8_t> &joint_ids, std::vector<JointParam_t> &joint_data);

    inline void convert_uint8_to_uint16(const uint8_t *input, uint16_t *output, size_t length)
    {
        for (size_t i = 0; i < length; i++)
        {
            // 将每个 uint8_t 类型的元素转换为 uint16_t 类型
            output[i] = static_cast<uint16_t>(input[i]);
        }
    }

    inline double calcCos(double start, double stop, double T, double t)
    {
        double A = (stop - start) / 2.0;
        return A * -cos(M_PI / T * t) + start + A;
    }

    void setEcmasterDriverType(const std::string &type);
   

public:
protected:
private:
    KuavoCommon *kuavo_common_ptr_;
    RuiWoActuator *ruiwo_actuator_{nullptr};
    eef_controller::LejuClawControllerPtr leju_claw_actuator_{nullptr};
    ActuatorsInterface_t actuators_;
    std::unique_ptr<eef_controller::DexhandController> dexhand_actuator_{nullptr};

    HighlyDynamic::KuavoSettings kuavo_settings_;
    HighlyDynamic::HardwareSettings motor_info;
    AnkleSolver ankleSolver_;

    std::string ecmaster_type_ = "elmo";
    std::string imu_type_str_ = "xsens";

    std::unordered_map<int, int> ec_index_map_; // 构建索引映射表

    bool is_can_protocol_{false};

    int na_foot_{12};
    int num_joint_{28};
    double dt_{0.0};
    ros::NodeHandle nh_;

    std::vector<JointParam_t> joint_data_;
    std::vector<uint8_t> joint_ids_;
    std::vector<JointParam_t> joint_cmd_;
};

