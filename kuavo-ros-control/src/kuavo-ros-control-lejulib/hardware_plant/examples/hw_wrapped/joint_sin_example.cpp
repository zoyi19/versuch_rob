// Copyright 2025 Lejurobot. All rights reserved.
//

#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <cmath>
#include <vector>
#include <atomic>
#include <mutex>
#include "hw_wrapped/hw_wrapped.h"

using namespace leju::hw;

static volatile bool running = true;

/**
 * @brief 机器人正弦波控制类
 *
 * 封装机器人的正弦波控制功能，包括初始化、控制循环和状态管理
 */
class RobotSinController {
public:
    /**
     * @brief 构造函数
     * @param amplitude 振幅 [rad]
     * @param frequency 频率 [Hz]
     * @param control_frequency 控制频率 [Hz]
     */
    RobotSinController(double amplitude = 0.1, double frequency = 0.5, double control_frequency = 100.0)
        : initialized_(false),
          motor_count_(0),
          amplitude_(amplitude),
          frequency_(frequency),
          control_frequency_(control_frequency) {}

    /**
     * @brief 析构函数
     */
    ~RobotSinController() {
        if (initialized_) {
            shutdown();
        }
    }

    /**
     * @brief 初始化机器人
     * @return 初始化是否成功
     */
    bool initialize() {
        std::cout << "--- Initializing Robot ---" << std::endl;

        // 获取硬件接口单例
        auto& hw = HardwareInterface::GetInstance();

        // 初始化硬件接口
        std::cout << "Initializing hardware interface..." << std::endl;
        HwErrorType ret = hw.Initialize();
        if (ret != HwErrorType::Success) {
            std::cerr << "Failed to initialize hardware interface, error code: "
                      << static_cast<int>(ret) << std::endl;
            return false;
        }

        // 获取电机数量
        motor_count_ = hw.GetMotorNumber();

        // 读取初始关节位置
        JointData_t joint_data; ImuData_t imu_data;
        ret = hw.GetSensorsState(joint_data, imu_data);
        if (ret != HwErrorType::Success) {
            std::cerr << "Failed to read sensors state!" << std::endl;
            return false;
        }

        // 全部关节回到 0.0 位置
        std::vector<double> zero_pos(hw.GetMotorNumber(), 0.0);
        hw.JointMoveTo(zero_pos, 60.0);  // speed=60deg/s

        // 记录所有关节初始位置作为正弦波中心
        init_positions_.resize(motor_count_);
        for (size_t i = 0; i < motor_count_; ++i) {
            init_positions_[i] = joint_data.q[i];
        }

        initialized_ = true;
        std::cout << "Robot initialized successfully" << std::endl;
        return true;
    }

    void shutdown() {
        if (!initialized_) {
            return;
        }

        std::cout << "Shutting down robot..." << std::endl;
        auto& hw = HardwareInterface::GetInstance();
        hw.Shutdown();
        initialized_ = false;
        std::cout << "Robot shutdown complete" << std::endl;
    }

    void run() {
        if (!initialized_) {
            std::cerr << "Robot not initialized!" << std::endl;
            return;
        }

        std::cout << "\n--- Starting Control Loop ---" << std::endl;
        std::cout << "Amplitude: " << amplitude_ << " rad (" << amplitude_ * 180.0 / M_PI << " deg)" << std::endl;
        std::cout << "Frequency: " << frequency_ << " Hz" << std::endl;
        std::cout << "Control frequency: " << control_frequency_ << " Hz" << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;

        const double control_period = 1.0 / control_frequency_;

        auto& hw = HardwareInterface::GetInstance();
        auto start_time = std::chrono::steady_clock::now();

        int iteration = 0;
        while (running) {
            // 计算当前时间
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();

            // 生成正弦波控制指令
            JointCommand_t joint_cmd(motor_count_);
            generateSineWaveCommand(joint_cmd, elapsed_seconds);

            // 发送关节命令
            hw.SetJointCommand(joint_cmd);

            // 读取当前状态（降低频率以减少开销）
            if (iteration % 10 == 0) {
                JointData_t joint_data;
                ImuData_t imu_data;
                hw.GetSensorsState(joint_data, imu_data);

                std::lock_guard<std::mutex> lock(print_mutex_);
                latest_joint_data_ = joint_data;
                latest_imu_data_ = imu_data;
            }

            // 每1000次迭代打印一次状态
            if (iteration % 1000 == 0) {
                std::lock_guard<std::mutex> lock(print_mutex_);
                std::cout << "Status: Iteration " << iteration
                         << ", Time: " << std::fixed << std::setprecision(1) << elapsed_seconds << "s"
                         << ", Joint[0]: target=" << std::setprecision(4) << joint_cmd.q[0]
                         << ", actual=" << std::setprecision(4) << latest_joint_data_.q[0]
                         << ", error=" << std::setprecision(4) << (joint_cmd.q[0] - latest_joint_data_.q[0])
                         << std::endl;
            }

            // 按照控制频率休眠
            std::this_thread::sleep_for(std::chrono::duration<double>(control_period));

            ++iteration;
        }

        std::cout << "Control loop completed" << std::endl;
    }

private:
    /**
     * @brief 使用正弦波生成关节控制指令
     * @param cmd 输出的控制指令
     * @param time 时间（秒）
     */
    void generateSineWaveCommand(JointCommand_t& cmd, double time) const {
        
        // 全部位置控制模式
        cmd.resize(motor_count_);
        for (size_t i = 0; i < motor_count_; ++i) {
            cmd.modes[i] = 2;   // 2 即 CSP 位置控制模式
        }

        // 方便起见, 我们仅演示手臂+头部关节的sin运动，12 是手臂index的开始下标
        for (size_t i = 12; i < motor_count_; ++i) {
            double phase = 2.0 * M_PI * static_cast<double>(i) / motor_count_;

            // 生成正弦波位置指令
            cmd.q[i] = init_positions_[i] + amplitude_ * std::sin(2.0 * M_PI * frequency_ * time + phase);

            // 计算速度（正弦波的导数）
            cmd.v[i] = 0;

            // 力矩设为0（位置控制模式）
            cmd.tau[i] = 0.0;

            cmd.modes[i] = 2;   // 2 即 CSP 位置控制模式
            cmd.kp[i] = 20.0;  // 位置增益
            cmd.kd[i] = 2.0;   // 速度增益
        }
    }

private:
    bool initialized_;                        // 初始化状态
    size_t motor_count_;                      // 电机数量
    double amplitude_;                        // 振幅 [rad]
    double frequency_;                        // 频率 [Hz]
    double control_frequency_;                // 控制频率 [Hz]
    std::vector<double> init_positions_;      // 初始位置
    mutable std::mutex print_mutex_;          // 打印互斥锁
    JointData_t latest_joint_data_;           // 最新的关节数据
    ImuData_t latest_imu_data_;               // 最新的IMU数据
};

// 全局控制器指针，用于信号处理
static RobotSinController* g_controller = nullptr;

void signalHandler(int signum) {
    std::cout << "\nReceived interrupt signal, stopping..." << std::endl;
    running = false;
}

int main(int argc, char* argv[]) {
    std::cout << "=== hw_wrapped - Robot Sine Wave Control Example (Class-based) ===" << std::endl;
    std::cout << "Features: Sine Wave Control, Position Mode" << std::endl;

    RobotSinController controller;

    // 设置信号处理
    signal(SIGINT, signalHandler);
    g_controller = &controller;

    if (!controller.initialize()) {
        std::cerr << "Failed to initialize robot controller" << std::endl;
        return 1;
    }

    controller.run();

    std::cout << "\nRobot sine wave control example completed successfully!" << std::endl;

    return 0;
}
