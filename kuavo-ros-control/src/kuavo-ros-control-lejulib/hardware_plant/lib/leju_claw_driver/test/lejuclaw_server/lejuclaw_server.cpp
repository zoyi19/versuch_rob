#include "lejuclaw.h"
#include <cstdlib>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <csignal>
#include <cmath>
#include <unistd.h>
#include <sys/file.h>
#include <fcntl.h>

#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"

const std::string COMMAND_FILE = "/tmp/leju_claw_command.json";
const std::string STATUS_FILE = "/tmp/leju_claw_status.json";
const std::string LOCK_FILE = "/tmp/leju_claw.lock";

LeJuClaw actuator;

bool emergency_stop_requested = false;
bool server_running = true;

void signalHandler(int signum)
{
    std::cout << COLOR_YELLOW << "\n[!] 接收到中断信号 (" << signum << ")，启动退出程序..." << COLOR_RESET << std::endl;
    emergency_stop_requested = true;
    server_running = false;
}

// 读取命令文件
bool readCommand(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& torques)
{
    std::ifstream file(COMMAND_FILE);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    std::string content;
    while (std::getline(file, line)) {
        content += line;
    }
    file.close();
    
    // 解析JSON
    size_t pos_start = content.find("\"positions\": [");
    if (pos_start == std::string::npos) {
        return false;
    }
    
    pos_start = content.find("[", pos_start);
    size_t pos_end = content.find("]", pos_start);
    if (pos_end == std::string::npos) {
        return false;
    }
    
    std::string pos_str = content.substr(pos_start + 1, pos_end - pos_start - 1);
    std::istringstream iss(pos_str);
    positions.clear();
    double val;
    while (iss >> val) {
        positions.push_back(val);
        if (iss.peek() == ',') {
            iss.ignore();
        }
    }
    
    // 解析velocities
    size_t vel_start = content.find("\"velocities\": [");
    if (vel_start != std::string::npos) {
        vel_start = content.find("[", vel_start);
        size_t vel_end = content.find("]", vel_start);
        if (vel_end != std::string::npos) {
            std::string vel_str = content.substr(vel_start + 1, vel_end - vel_start - 1);
            std::istringstream vel_iss(vel_str);
            velocities.clear();
            while (vel_iss >> val) {
                velocities.push_back(val);
                if (vel_iss.peek() == ',') {
                    vel_iss.ignore();
                }
            }
        }
    }
    
    // 解析torques
    size_t tor_start = content.find("\"torques\": [");
    if (tor_start != std::string::npos) {
        tor_start = content.find("[", tor_start);
        size_t tor_end = content.find("]", tor_start);
        if (tor_end != std::string::npos) {
            std::string tor_str = content.substr(tor_start + 1, tor_end - tor_start - 1);
            std::istringstream tor_iss(tor_str);
            torques.clear();
            while (tor_iss >> val) {
                torques.push_back(val);
                if (tor_iss.peek() == ',') {
                    tor_iss.ignore();
                }
            }
        }
    }
    
    return !positions.empty();
}

void writeStatus(const std::string& status, const std::vector<double>& positions, const std::string& message = "")
{
    std::ofstream file(STATUS_FILE);
    if (!file.is_open()) {
        return;
    }
    
    file << "{\n";
    file << "  \"status\": \"" << status << "\",\n";
    file << "  \"positions\": [";
    for (size_t i = 0; i < positions.size(); ++i) {
        file << positions[i];
        if (i < positions.size() - 1) {
            file << ", ";
        }
    }
    file << "],\n";
    file << "  \"message\": \"" << message << "\"\n";
    file << "}\n";
    file.close();
}

int main()
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << COLOR_GREEN << "╭────────────────────────────────────────────────────────────╮" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "│              LEJU 夹爪控制服务器程序                       │" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN << "╰────────────────────────────────────────────────────────────╯" << COLOR_RESET << std::endl;
    
    // 初始化夹爪
    bool init_bmapilib = true;
    if (0 != actuator.initialize(init_bmapilib))
    {
        std::cout << COLOR_RED << "[✗] 初始化失败！" << COLOR_RESET << std::endl;
        writeStatus("error", {}, "初始化失败");
        return -1;
    }
    
    std::cout << COLOR_GREEN << "[✓] 系统初始化成功！" << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "[!] 等待系统稳定..." << COLOR_RESET << std::endl;
    usleep(1000000);
    
    std::cout << COLOR_BLUE << "[→] 初始化后运动到50%位置..." << COLOR_RESET << std::endl;
    std::vector<double> init_positions = {50.0, 50.0};
    std::vector<double> init_velocities = {0.0, 0.0};
    std::vector<double> init_torques = {0.0, 0.0};
    
    auto init_result = actuator.move_paw(init_positions, init_velocities, init_torques);
    
    if (init_result == LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_REACHED)
    {
        std::cout << COLOR_GREEN << "[✓] 夹爪已到达初始位置 (50%)" << COLOR_RESET << std::endl;
        writeStatus("ready", init_positions, "初始化完成，已到达50%位置");
    }
    else
    {
        std::cout << COLOR_RED << "[✗] 初始化位置运动失败！" << COLOR_RESET << std::endl;
        writeStatus("error", {}, "初始化位置运动失败");
        actuator.close();
        return -1;
    }
    
    usleep(500000);  // 等待0.5秒
    
    std::cout << COLOR_GREEN << "\n[✓] 服务器已启动，等待控制指令..." << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "[!] 命令文件: " << COMMAND_FILE << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "[!] 状态文件: " << STATUS_FILE << COLOR_RESET << std::endl;
    std::cout << COLOR_YELLOW << "[!] 按 Ctrl+C 退出程序" << COLOR_RESET << std::endl;
    
    std::cout << COLOR_BLUE << "\n[→] 提示：发布目标位置的方式：" << COLOR_RESET << std::endl;
    std::cout << COLOR_BLUE << "    新开终端，运行 Hardware_tool.py，依次选择：o -> a -> 3" << COLOR_RESET << std::endl;
    
    while (server_running && !emergency_stop_requested)
    {
        std::vector<double> positions;
        std::vector<double> velocities = {0.0, 0.0};
        std::vector<double> torques = {0.0, 0.0};
        
        if (readCommand(positions, velocities, torques))
        {
            if (velocities.empty()) {
                velocities.resize(positions.size(), 0.0);
            }
            if (torques.empty()) {
                torques.resize(positions.size(), 0.0);
            }
            
            std::cout << COLOR_BLUE << "[→] 收到控制指令: positions=[";
            for (size_t i = 0; i < positions.size(); ++i) {
                std::cout << positions[i];
                if (i < positions.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << COLOR_RESET << std::endl;
            
            std::remove(COMMAND_FILE.c_str());
            writeStatus("running", positions, "正在运动");
            
            auto result = actuator.move_paw(positions, velocities, torques);
            usleep(50000);
            
            auto current_positions = actuator.get_positions();
            
            if (result == LeJuClaw::PawMoveState::LEFT_REACHED_RIGHT_REACHED)
            {
                std::cout << COLOR_GREEN << "[✓] 夹爪已到达目标位置" << COLOR_RESET << std::endl;
                writeStatus("ready", current_positions, "已到达目标位置");
            }
            else if (result == LeJuClaw::PawMoveState::ERROR)
            {
                std::cout << COLOR_RED << "[✗] 位置命令执行失败！" << COLOR_RESET << std::endl;
                writeStatus("error", current_positions, "位置命令执行失败");
            }
            else
            {
                writeStatus("ready", current_positions, "运动完成");
            }
        }
        
        usleep(100000);
    }
    
    std::cout << COLOR_YELLOW << "\n[!] 执行退出程序..." << COLOR_RESET << std::endl;
    
    std::vector<double> exit_target = {50.0, 50.0};
    std::vector<double> exit_velocity = {0.0, 0.0};
    std::vector<double> exit_torque = {0.0, 0.0};
    
    std::cout << COLOR_YELLOW << "[!] 发送退出位置命令: [50.0, 50.0]" << COLOR_RESET << std::endl;
    actuator.move_paw(exit_target, exit_velocity, exit_torque);
    
    std::cout << COLOR_YELLOW << "[!] 清零夹爪电流..." << COLOR_RESET << std::endl;
    actuator.clear_all_torque();
    usleep(200000);
    
    std::cout << COLOR_GREEN << "[✓] 夹爪电流已清零，正在关闭系统..." << COLOR_RESET << std::endl;
    actuator.close();
    
    std::remove(COMMAND_FILE.c_str());
    std::remove(STATUS_FILE.c_str());
    
    std::cout << COLOR_GREEN << "[✓] 系统已安全关闭" << COLOR_RESET << std::endl;
    return 0;
}
