#include <ros/ros.h>
#include "hardware_node.h"
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <atomic>

std::atomic<bool> running(true);
bool ready = false;
int test_mode = -1;
int select_motor_id = 0;
int left_right_mode = 0;// 0: left, 1: right
double test_tau = 0;
std::string imu_type_str_ = "xsens";
std::unique_ptr<HighlyDynamic::HardwareNode> hardware_node_ptr;

// 封装的Yaw角漂移检测函数
bool detectYawDrift() {
    const int SAMPLE_COUNT = 100;   // 检测样本数（可根据需求调整）
    const double TIME_INTERVAL = 0.1; // 采样间隔（秒，可根据IMU频率调整）
    const double MAX_ALLOWED_DRIFT = 0.1; // 允许的最大漂移（弧度，约5.7度）

    std::vector<double> yawAngles; // 存储所有采样的yaw角

    std::cout << "正在检测Yaw角稳定性（保持静止）...\r\n";
    Eigen::Vector3d acc, gyro;
    Eigen::Quaterniond quat;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        // 读取最新IMU数据
        bool readflag = (imu_type_str_ == "xsens")? xsens_IMU::getImuDataFrame(acc, gyro, quat) : HIPNUC_IMU::getImuDataFrame(acc, gyro, quat);
        if (!readflag) {
            std::cerr << "检测过程中IMU数据读取失败!\n";
            return false;
        }

        // 将四元数转换为Yaw角（ZYX欧拉角顺序，Yaw为绕Z轴的旋转角）
        Eigen::Vector3d eulerAngles = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
        double yaw = eulerAngles[0];      // Yaw角（单位：弧度）

        yawAngles.push_back(yaw);
        std::cout << "采样 " << i + 1 << "/" << SAMPLE_COUNT << "，Yaw: " << yaw << " rad\r\n";

        // 等待采样间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(TIME_INTERVAL * 1000)));
    }

    // 计算Yaw角的波动范围（最大值-最小值）
    double maxYaw = *std::max_element(yawAngles.begin(), yawAngles.end());
    double minYaw = *std::min_element(yawAngles.begin(), yawAngles.end());
    double yawDrift = maxYaw - minYaw;

    // 判断是否超过漂移阈值
    if (yawDrift > MAX_ALLOWED_DRIFT) {
        std::cerr << "警告：Yaw角漂移检测失败！漂移量：" << yawDrift << " rad（允许最大值：" << MAX_ALLOWED_DRIFT << " rad）\n";
        std::cerr << "可能原因：IMU未校准、陀螺仪零偏过大或设备晃动\n";
        return false;
    } else {
        std::cout << "Yaw角稳定性检测通过！漂移量：" << yawDrift << " rad\n";
        return true;
    }
}

void keyboardInput(size_t num_joints, double dt)
{
    std::cout << "移动到准备姿态 ..." << std::endl;
    std::vector<double> Ready_pos = {
        1.61187, 0.0843823, -60.5829, 107.461, -49.8766, -1.6141,
       -1.45677, -0.076402, -60.6254, 107.512, -49.8855, 1.45869,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        };
    hardware_node_ptr->jointMoveTo(Ready_pos, 60, dt);

    std::ostringstream tips_oss;
    tips_oss << "\033[32m*******检查你的机器人的状态并进行校准*******"
    << "\n1. 输入 't' 进入测试模式"
    << "\n2. 输入 'a' 检测手部电机"
    << "\n3. 输入 'o' 移动到校准姿态，(准备姿态下)输入 'q' 返回准备姿态，或直接输入 'ctrl+c' 退出.."
    << "\n4. 输入 'i' 检测imu"
    << "\n5. 输入 'b' 检测遥控器信号"
    << "\n6. 输入 'x' 关闭当前程序，回退到上一级重新选择"
    << "\n7. 输入 'h' 再次显示此提示。\033[0m\n";
    std::cout << tips_oss.str() << std::endl;

    while (running)
    {
        if (kbhit())
        {
            auto Walk_Command = getchar();
            std::cout << "[keyboard command]: " << Walk_Command << std::endl;
            if (Walk_Command == 'x')
            {
                std::cout << "Sending stop message" << std::endl;
                hardware_node_ptr.reset();
                running = false;
                break;
            }
            else if (Walk_Command == 'h')
            {
                std::cout << tips_oss.str() << std::endl;
            }
            else if (Walk_Command == 'a')
            {
                test_mode = -1;
                usleep(10000);
                std::cout << "检测所有手部关节 ..." << std::endl;
                std::vector<double> arm_test_pos(num_joints, 0.0);

                std::cout << "检测左右手0, 1, 3关节 ..." << std::endl;
                arm_test_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                -10.0, 20.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                -10.0, -20.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                0.0, 0.0};
                hardware_node_ptr->jointMoveTo(arm_test_pos, 60, dt);
                usleep(1000000);

                std::cout << "检测左右手2关节 ..." << std::endl;
                arm_test_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                -10.0, 20.0, -30.0, -30.0, 0.0, 0.0, 0.0,
                                -10.0, -20.0, 30.0, -30.0, 0.0, 0.0, 0.0,
                                0.0, 0.0};
                hardware_node_ptr->jointMoveTo(arm_test_pos, 60, dt);
                usleep(1000000);

                std::cout << "检测左右手4, 5, 6关节 ..." << std::endl;
                arm_test_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                -10.0, 20.0, -30.0, -30.0, -30.0, 15.0, -10.0,
                                -10.0, -20.0, 30.0, -30.0, 30.0, -15.0, -10.0,
                                0.0, 0.0};
                hardware_node_ptr->jointMoveTo(arm_test_pos, 60, dt);
                usleep(2000000);

                std::cout << "复位..." << std::endl;
                arm_test_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0};
                hardware_node_ptr->jointMoveTo(arm_test_pos, 60, dt);
                usleep(10000);
                test_mode = 1;
            }
            else if (Walk_Command == 'o')
            {
                std::cout << "移动到校准姿态 ..." << std::endl;
                std::vector<double> goal_pos(num_joints, 0.0);
                hardware_node_ptr->jointMoveTo(goal_pos, 60, dt);
                ready = true;
                std::cout << "Ready to receive command" << std::endl;
                std::cout << "Press 't' to test the joint move\n";
            }
            else if(Walk_Command == 'i')
            {
                std::cout << "检测imu状态..." << std::endl;
                detectYawDrift();
            }
            else if(Walk_Command == 'b')
            {
                std::cout << "检测遥控器信号..." << std::endl;
                std::cout << "当前未实现..." << std::endl;
            }
            else if (Walk_Command == 't')// 进入测试模式
            {
                std::cout << "Testing joint move" << std::endl;
                test_mode = 1;
            }
            else if (Walk_Command == 'q')// 退出测试模式
            {
                test_mode = -1;
                std::cout << "退出测试模式, 移动到准备姿态 ..." << std::endl;
                hardware_node_ptr->jointMoveTo(Ready_pos, 60, dt);
                std::cout << "cancel testing" << std::endl;
            }
            else if (Walk_Command == 'l')// 测试左侧电机
            {
                std::cout << "Testing left motor" << std::endl;
                left_right_mode = 0;
            }
            else if (Walk_Command == 'r')// 测试右侧电机
            {
                std::cout << "Testing right motor" << std::endl;
                left_right_mode = 1;
            }
            else if (Walk_Command == 'w')// w/s增加和缩小测试扭矩
            {
                test_tau += 1.0;
                std::cout << "inc tau: " << test_tau << std::endl;
            }
            else if (Walk_Command == 's')
            {
                test_tau -= 1.0;
                std::cout << "dec tau: " << test_tau << std::endl;
            }
            else if (Walk_Command == ' ')
            {
                test_tau = 0.0;
                std::cout << "reset tau: " << test_tau << std::endl;
            }
            else
            {
                if (std::isdigit(Walk_Command))
                {
                    auto new_select_motor_id = Walk_Command - '0' - 1;
                    if (new_select_motor_id >= 0 && new_select_motor_id < 6)
                    {
                        select_motor_id = new_select_motor_id;
                        std::cout << "select motor " << select_motor_id + 1 << std::endl;
                    }
                    else
                    {
                        std::cout << "Invalid motor id" << new_select_motor_id + 1 << std::endl;
                    }
                }
            }

        }
        usleep(50000);
    }
}

int main(int argc, char **argv)
{
    double dt = 0.002;
    ros::init(argc, argv, "hardware_test");
    ros::NodeHandle nh;
    nh.setParam("/robot_version", 45);
    nh.setParam("cali", true);
    nh.setParam("cali_arm", true);
    hardware_node_ptr = std::make_unique<HighlyDynamic::HardwareNode>(nh, dt);
    hardware_node_ptr->init();
    ros::spin();
    std::thread spin_thread([]()
                            { ros::spin(); });
    spin_thread.detach();

    size_t num_joints = hardware_node_ptr->get_num_actuated();
    auto motor_info = hardware_node_ptr->get_motor_info();
    RobotVersion rb_version(4, 2);
    imu_type_str_ = motor_info.getIMUType(rb_version);


    std::thread input_thread(keyboardInput, num_joints, dt);

    input_thread.detach();

    std::cout << "\nPress 'x' to exit.\nPress 'o' to move to zero position, then ready to receive command." << std::endl;



    // 等待输入
    while (!ready)
        usleep(1000);
    // 开始接收命令
    hardware_node_ptr->readyToRecvCmd();
    auto jointCmdPub_ = nh.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);

    auto gen_zero_cmd_msg = [&]() -> kuavo_msgs::jointCmd
    {
        kuavo_msgs::jointCmd jointCmdMsg;
        for (int i1 = 0; i1 < num_joints; ++i1)
        {
            jointCmdMsg.joint_q.push_back(0.0);
            jointCmdMsg.joint_v.push_back(0.0);
            jointCmdMsg.tau.push_back(0);
            jointCmdMsg.tau_ratio.push_back(1);
            jointCmdMsg.joint_kp.push_back(0);
            jointCmdMsg.joint_kd.push_back(0);
            jointCmdMsg.tau_max.push_back(motor_info.max_current[i1]);
            jointCmdMsg.control_modes.push_back(CSP);
        }
        return jointCmdMsg;
    };

    double time = 0;
    double test_time = 0;

    std::cout << "start loop" << std::endl;
    // 开始测试
    while (running && ros::ok())
    {
        if (test_mode == -1)
        {
            // auto msg = gen_zero_cmd_msg();
            // jointCmdPub_.publish(msg);
        }
        else if (test_mode == 1)
        {
            auto msg = gen_zero_cmd_msg();
            if (test_time < 2.0)
            {
                msg.control_modes[select_motor_id + left_right_mode * 6] = CST;
                msg.tau[select_motor_id + left_right_mode * 6] = (test_time < 1.0) ? test_tau : -test_tau;
            }
            else
            {
                // select_motor_id = (select_motor_id < 12-1)? select_motor_id + 1 : 0;
                test_time = 0;
                std::cout << "Testing motor " << select_motor_id + left_right_mode * 6 + 1 << " tau: " << test_tau << std::endl;
            }
            jointCmdPub_.publish(msg);
            test_time += dt;
        }
        time += dt;
        usleep(dt * 1000000);
    }

    running = false; // 结束输入线程
    if (input_thread.joinable())
    {
        input_thread.join();
    }

    return 0;
}
