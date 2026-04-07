#include "hw_wrapped/hw_wrapped.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

int main() {
    std::cout << "=== Leju Hardware Interface Example ===" << std::endl;

    // 获取硬件接口单例
    leju::hw::HardwareInterface& hw = leju::hw::HardwareInterface::GetInstance();

    // 初始化硬件接口
    std::cout << "Initializing hardware interface..." << std::endl;
    if (hw.Initialize() == leju::hw::HwErrorType::Success) {
        std::cout << "Hardware interface initialized successfully!" << std::endl;
    } else {
        std::cerr << "Failed to initialize hardware interface!" << std::endl;
        return -1;
    }

    std::vector<double> squat_pos(hw.GetMotorNumber(), 0);
    squat_pos[0] = 0.027976495008415852 * 180.0 / M_PI;    // 1.603°
    squat_pos[1] = 0.0014651360392985808 * 180.0 / M_PI;   // 0.084°
    squat_pos[2] = -1.055823448348866 * 180.0 / M_PI;     // -60.520°
    squat_pos[3] = 1.8665743357740023 * 180.0 / M_PI;     // 106.945°
    squat_pos[4] = -0.8630915385720594 * 180.0 / M_PI;    // -49.452°
    squat_pos[5] = -0.028014584192649326 * 180.0 / M_PI;  // -1.605°
    squat_pos[6] = -0.02533468603942189 * 180.0 / M_PI;   // -1.452°
    squat_pos[7] = -0.0013270840552339982 * 180.0 / M_PI;  // -0.076°
    squat_pos[8] = -1.0565561530676477 * 180.0 / M_PI;     // -60.567°
    squat_pos[9] = 1.8674666837975518 * 180.0 / M_PI;      // 106.997°
    squat_pos[10] = -0.8632543448123566 * 180.0 / M_PI;    // -49.462°
    squat_pos[11] = 0.02536851832068221 * 180.0 / M_PI;    // 1.453°

    hw.JointMoveTo(squat_pos, 60.0);  // speed=60deg/s

    std::vector<double> zero_pos(hw.GetMotorNumber(), 0);
    hw.JointMoveTo(zero_pos, 60.0);  // speed=60deg/s

    // 运行一段时间
    std::cout << "Hardware interface is running..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    // 模拟运行
    for (int i = 0; i < 5; ++i) {
        std::cout << "Running... " << (5 - i) << " seconds remaining" << std::endl;
        leju::hw::JointData_t joint_state;
        leju::hw::ImuData_t imu_data;
        if(hw.GetSensorsState(joint_state, imu_data) == leju::hw::HwErrorType::Success) {
            std::cout << joint_state;
            std::cout << imu_data;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 关闭硬件接口
    std::cout << "Shutting down hardware interface..." << std::endl;
    hw.Shutdown();

    std::cout << "Example completed successfully!" << std::endl;
    return 0;
}