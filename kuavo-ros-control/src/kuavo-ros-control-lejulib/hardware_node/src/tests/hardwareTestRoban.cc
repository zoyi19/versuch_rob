#include <ros/ros.h>
#include "hardware_node.h"
#include <thread>
#include <vector>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <termios.h>
#include <fcntl.h>
#include <cstdio>

// 确保 M_PI 已定义（某些编译器需要）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 波形类型枚举
enum WaveformType {
    WAVEFORM_NONE = -1,
    WAVEFORM_SQUARE = 0,    // 方波
    WAVEFORM_TRIANGLE = 1,  // 三角波
    WAVEFORM_SINE = 2       // 正弦波
};

std::atomic<bool> running(true);
bool ready = false;
int test_mode = -1;
int select_motor_id = 0;
int left_right_mode = 0;  // 0: left, 1: right
double test_time = 0.0;   // 测试时间计数器
std::unique_ptr<HighlyDynamic::HardwareNode> hardware_node_ptr;
bool first_test_print = true;  // 首次进入测试模式的标志
bool motor_select_mode = false;  // 电机选择模式标志（在测试模式下使用）

// 波形测试相关参数
WaveformType waveform_type = WAVEFORM_SQUARE;  // 默认方波
double waveform_amplitude = 1.0;              // 波形幅度（Nm）
double waveform_frequency = 1.0;               // 波形频率（Hz）
double waveform_offset = 0.0;                   // 波形偏移量
double waveform_phase = 0.0;                   // 波形相位

// kbhit函数实现（用于非阻塞键盘输入检测）
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void keyboardInput(size_t num_joints, double dt)
{
    // 直接移动到校准姿态（全零位置）
    std::cout << "移动到校准姿态 ..." << std::endl;
    std::vector<double> goal_pos(num_joints, 0.0);
    hardware_node_ptr->jointMoveTo(goal_pos, 60, dt);
    ready = true;  // 直接设置为ready，不需要用户输入'o'
    std::cout << "已移动到校准姿态，可以开始测试" << std::endl;

    std::ostringstream tips_oss;
    tips_oss << "\033[32m========== 波形测试程序 =========="
    << "\n基本操作："
    << "\n  't' - 进入波形测试模式"
    << "\n  'q' - 退出测试模式，返回校准姿态"
    << "\n  'x' - 退出程序"
    << "\n  'h' - 显示此帮助信息"
    << "\n\n波形类型切换（测试模式下）："
    << "\n  '1' - 方波"
    << "\n  '2' - 三角波"
    << "\n  '3' - 正弦波"
    << "\n\n波形参数调整："
    << "\n  'w'/'s' - 增加/减少波形幅度"
    << "\n  'e'/'d' - 增加/减少波形频率"
    << "\n  'u'/'j' - 增加/减少波形偏移量"
    << "\n  ' ' (空格) - 重置所有波形参数"
    << "\n\n电机选择："
    << "\n  'l'/'r' - 选择左侧/右侧电机"
    << "\n  'm' - 进入电机选择模式（测试模式下），然后输入 '1'-'6' 选择电机"
    << "\n  或直接输入 '4'-'9' 选择电机1-6（测试模式下）"
    << "\n  非测试模式：直接输入 '1'-'6' 选择电机"
    << "\n\n当前状态: "
    << "\n  波形: " << (waveform_type == WAVEFORM_SQUARE ? "方波" : 
                          waveform_type == WAVEFORM_TRIANGLE ? "三角波" : 
                          waveform_type == WAVEFORM_SINE ? "正弦波" : "无")
    << "\n  幅度: " << waveform_amplitude << " Nm"
    << "\n  频率: " << waveform_frequency << " Hz"
    << "\n  偏移: " << waveform_offset << " Nm"
    << "\n  电机: " << (left_right_mode == 0 ? "左侧" : "右侧") << " 电机 " << (select_motor_id + 1)
    << "\033[0m\n";
    std::cout << tips_oss.str() << std::endl;

    while (running)
    {
        if (kbhit())
        {
            auto Walk_Command = getchar();
            std::cout << "[keyboard command]: " << Walk_Command << std::endl;
            if (Walk_Command == 'x')
            {
                std::cout << "退出程序..." << std::endl;
                hardware_node_ptr.reset();
                running = false;
                break;
            }
            else if (Walk_Command == 'h')
            {
                std::cout << tips_oss.str() << std::endl;
            }
            else if (Walk_Command == 't')  // 进入测试模式
            {
                std::cout << "进入波形测试模式" << std::endl;
                std::cout << "当前波形: " << (waveform_type == WAVEFORM_SQUARE ? "方波" : 
                              waveform_type == WAVEFORM_TRIANGLE ? "三角波" : 
                              waveform_type == WAVEFORM_SINE ? "正弦波" : "无") << std::endl;
                std::cout << "当前参数: 幅度=" << waveform_amplitude << " Nm, 频率=" << waveform_frequency << " Hz" << std::endl;
                std::cout << "当前电机: " << (left_right_mode == 0 ? "左侧" : "右侧") << " 电机 " << (select_motor_id + 1) << std::endl;
                std::cout << "开始发送波形控制命令..." << std::endl;
                test_mode = 1;
            }
            else if (Walk_Command == 'q')  // 退出测试模式
            {
                test_mode = -1;
                waveform_phase = 0.0;
                test_time = 0.0;
                first_test_print = true;
                
                std::cout << "退出测试模式, 返回校准姿态 ..." << std::endl;
                std::vector<double> goal_pos(num_joints, 0.0);
                hardware_node_ptr->jointMoveTo(goal_pos, 60, dt);
                std::cout << "测试已取消" << std::endl;
            }
            else if (Walk_Command == 'l')  // 选择左侧电机
            {
                std::cout << "选择左侧电机" << std::endl;
                left_right_mode = 0;
            }
            else if (Walk_Command == 'r' && test_mode != 1)  // 选择右侧电机（非测试模式）
            {
                std::cout << "选择右侧电机" << std::endl;
                left_right_mode = 1;
            }
            // 电机选择模式切换（仅在测试模式下）
            else if (Walk_Command == 'm' && test_mode == 1)
            {
                motor_select_mode = !motor_select_mode;
                if (motor_select_mode)
                {
                    std::cout << "进入电机选择模式，现在可以输入 '1'-'6' 选择电机" << std::endl;
                }
                else
                {
                    std::cout << "退出电机选择模式，'1'-'3' 用于切换波形" << std::endl;
                }
            }
            // 波形类型切换（仅在测试模式下，且不在电机选择模式）
            else if (Walk_Command == '1' && test_mode == 1 && !motor_select_mode)
            {
                waveform_type = WAVEFORM_SQUARE;
                waveform_phase = 0.0;
                std::cout << "切换到方波测试, 幅度: " << waveform_amplitude 
                          << " Nm, 频率: " << waveform_frequency << " Hz" << std::endl;
            }
            else if (Walk_Command == '2' && test_mode == 1 && !motor_select_mode)
            {
                waveform_type = WAVEFORM_TRIANGLE;
                waveform_phase = 0.0;
                std::cout << "切换到三角波测试, 幅度: " << waveform_amplitude 
                          << " Nm, 频率: " << waveform_frequency << " Hz" << std::endl;
            }
            else if (Walk_Command == '3' && test_mode == 1 && !motor_select_mode)
            {
                waveform_type = WAVEFORM_SINE;
                waveform_phase = 0.0;
                std::cout << "切换到正弦波测试, 幅度: " << waveform_amplitude 
                          << " Nm, 频率: " << waveform_frequency << " Hz" << std::endl;
            }
            // 波形参数调整
            else if (Walk_Command == 'w')  // 增加波形幅度
            {
                waveform_amplitude += 0.1;
                std::cout << "波形幅度: " << waveform_amplitude << " Nm" << std::endl;
            }
            else if (Walk_Command == 's')  // 减少波形幅度
            {
                waveform_amplitude = std::max(0.0, waveform_amplitude - 0.1);
                std::cout << "波形幅度: " << waveform_amplitude << " Nm" << std::endl;
            }
            else if (Walk_Command == 'e')  // 增加波形频率
            {
                waveform_frequency += 0.1;
                waveform_frequency = std::min(10.0, waveform_frequency);
                std::cout << "波形频率: " << waveform_frequency << " Hz" << std::endl;
            }
            else if (Walk_Command == 'd')  // 减少波形频率
            {
                waveform_frequency = std::max(0.1, waveform_frequency - 0.1);
                std::cout << "波形频率: " << waveform_frequency << " Hz" << std::endl;
            }
            else if (Walk_Command == 'u')  // 增加波形偏移量
            {
                waveform_offset += 1.0;
                std::cout << "波形偏移: " << waveform_offset << " Nm" << std::endl;
            }
            else if (Walk_Command == 'j')  // 减少波形偏移量
            {
                waveform_offset -= 1.0;
                std::cout << "波形偏移: " << waveform_offset << " Nm" << std::endl;
            }
            else if (Walk_Command == ' ')
            {
                waveform_amplitude = 0.0;
                waveform_frequency = 1.0;
                waveform_offset = 0.0;
                waveform_phase = 0.0;
                std::cout << "重置所有波形参数（幅度=0, 频率=1.0 Hz, 偏移=0）" << std::endl;
            }
            else
            {
                // 电机ID选择
                if (std::isdigit(Walk_Command))
                {
                    int new_select_motor_id = -1;
                    if (test_mode != 1)
                    {
                        // 非测试模式：直接使用'1'-'6'
                        new_select_motor_id = Walk_Command - '0' - 1;
                    }
                    else
                    {
                        // 测试模式
                        if (motor_select_mode)
                        {
                            // 电机选择模式：直接使用'1'-'6'
                            new_select_motor_id = Walk_Command - '0' - 1;
                            motor_select_mode = false;
                        }
                        else if (Walk_Command >= '4' && Walk_Command <= '9')
                        {
                            // 非电机选择模式：'4'-'9'映射到电机1-6
                            new_select_motor_id = Walk_Command - '0' - 4;
                        }
                    }
                    
                    if (new_select_motor_id >= 0 && new_select_motor_id < 6)
                    {
                        select_motor_id = new_select_motor_id;
                        std::cout << "选择电机 " << select_motor_id + 1 << std::endl;
                    }
                    else if (test_mode == 1 && !motor_select_mode && Walk_Command >= '1' && Walk_Command <= '3')
                    {
                        std::cout << "提示：在测试模式下，'1'-'3'用于切换波形类型" << std::endl;
                        std::cout << "      要选择电机，请先输入 'm' 进入电机选择模式，或使用 '4'-'9'" << std::endl;
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
    
    // 从参数服务器读取robot_version
    int robot_version = 4;
    if (!nh.getParam("/robot_version", robot_version)) {
        std::cout << "未找到/robot_version参数，使用默认值4" << std::endl;
        nh.setParam("/robot_version", robot_version);
    }
 
    nh.setParam("cali", true);
    nh.setParam("cali_arm", true);
    hardware_node_ptr = std::make_unique<HighlyDynamic::HardwareNode>(nh, dt);
    hardware_node_ptr->init();
    std::thread spin_thread([]()
                            { ros::spin(); });
    spin_thread.detach();

    size_t num_joints = hardware_node_ptr->get_num_actuated();
    auto motor_info = hardware_node_ptr->get_motor_info();

    std::thread input_thread(keyboardInput, num_joints, dt);
    input_thread.detach();

    std::cout << "\nPress 'x' to exit.\nPress 't' to start waveform test." << std::endl;

    // 提前创建话题发布者
    auto jointCmdPub_ = nh.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
    // 等待一下，让发布者注册到ROS master
    ros::Duration(0.5).sleep();

    // 等待输入
    while (!ready)
        usleep(1000);
    // 开始接收命令
    hardware_node_ptr->readyToRecvCmd();

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

    // 波形生成函数
    auto generateWaveform = [](WaveformType type, double t, double amplitude, double frequency, double offset) -> double {
        double period = 1.0 / frequency;
        double phase = std::fmod(t, period) / period;
        
        double value = 0.0;
        switch (type) {
            case WAVEFORM_SQUARE:
                value = (phase < 0.5) ? amplitude : -amplitude;
                break;
                
            case WAVEFORM_TRIANGLE:
                if (phase < 0.5) {
                    value = -amplitude + 4.0 * amplitude * phase;
                } else {
                    value = amplitude - 4.0 * amplitude * (phase - 0.5);
                }
                break;
                
            case WAVEFORM_SINE:
                value = amplitude * std::sin(2.0 * M_PI * phase);
                break;
                
            default:
                value = 0.0;
                break;
        }
        
        return value + offset;
    };

    std::cout << "start loop" << std::endl;
    // 开始测试
    while (running && ros::ok())
    {
        if (test_mode == -1)
        {
            // 非测试模式，不发送命令
        }
        else if (test_mode == 1)
        {
            auto msg = gen_zero_cmd_msg();
            int motor_idx = select_motor_id + left_right_mode * 6;
            
            // 边界检查
            if (motor_idx < 0 || motor_idx >= static_cast<int>(num_joints))
            {
                std::cerr << "错误：电机索引超出范围！motor_idx=" << motor_idx 
                          << ", num_joints=" << num_joints << std::endl;
                usleep(dt * 1000000);
                continue;
            }
            
            // 设置控制模式为力矩控制
            msg.control_modes[motor_idx] = CST;
            
            // 生成波形信号
            double waveform_value = generateWaveform(
                waveform_type, 
                test_time, 
                waveform_amplitude, 
                waveform_frequency, 
                waveform_offset
            );
            
            msg.tau[motor_idx] = waveform_value;
            
            // 首次进入测试模式时打印详细信息
            if (first_test_print) {
                std::cout << "\n========== 波形测试已启动 ==========" << std::endl;
                std::cout << "控制电机索引: " << motor_idx << " (总关节数: " << num_joints << ")" << std::endl;
                std::cout << "控制模式: 力矩控制 (CST)" << std::endl;
                std::cout << "波形类型: " << (waveform_type == WAVEFORM_SQUARE ? "方波" : 
                                            waveform_type == WAVEFORM_TRIANGLE ? "三角波" : 
                                            waveform_type == WAVEFORM_SINE ? "正弦波" : "无") << std::endl;
                std::cout << "波形幅度: " << waveform_amplitude << " Nm" << std::endl;
                std::cout << "波形频率: " << waveform_frequency << " Hz (周期: " << (1.0 / waveform_frequency) << " 秒)" << std::endl;
                std::cout << "波形偏移: " << waveform_offset << " Nm" << std::endl;
                std::cout << "=====================================\n" << std::endl;
                first_test_print = false;
            }
            
            // 每0.5秒打印一次状态信息
            static double last_print_time = 0.0;
            if (test_time - last_print_time >= 0.5) {
                std::cout << "[测试中] 电机 " << motor_idx + 1 
                          << " | 扭矩: " << std::fixed << std::setprecision(2) << waveform_value 
                          << " Nm | 幅度: " << waveform_amplitude 
                          << " Nm | 频率: " << waveform_frequency << " Hz" << std::endl;
                last_print_time = test_time;
            }
            
            jointCmdPub_.publish(msg);
            test_time += dt;
        }
        time += dt;
        usleep(dt * 1000000);
    }

    running = false;
    if (input_thread.joinable())
    {
        input_thread.join();
    }

    return 0;
}
