#include "actuators_interface.h"
#include "utils.h"
#include <filesystem>
#include <iostream>
#include <unistd.h>
#include <cstring>
#include "kuavo_common/common/utils.h"
#include "kuavo_common/common/json_config_reader.hpp"
#include <sys/stat.h>  // for stat
#include <unistd.h>   // for readlink
#include <limits.h>   // for PATH_MAX
#include <libgen.h>   // for dirname
#include <cstring>   // for strerror

pthread_t ecmaster_thread;

std::string getRobotVersionFromEnv()
{
    const char* robot_version = std::getenv("ROBOT_VERSION");
    if (robot_version != nullptr)
    {
        return std::string(robot_version);
    }
    return "";
}

// 专门用于查找 EC XML 配置文件的函数
// 优先检查 installed 路径（开源仓库），找不到则使用原方案（闭源仓库）
std::string findECXmlConfigPath(const std::string& relative_path) {
    // 提取XML文件名
    std::string xml_filename = relative_path;
    size_t last_slash = relative_path.find_last_of("/");
    if (last_slash != std::string::npos) {
        xml_filename = relative_path.substr(last_slash + 1);
    }

    std::cout << "[EC XML] 查找配置文件: " << xml_filename << std::endl;

    // 方法1: 通过可执行文件路径推导 installed 路径 (开源仓库)
    char exe_path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", exe_path, PATH_MAX - 1);
    if (len != -1) {
        exe_path[len] = '\0';
        // 复制一份路径用于 dirname（因为 dirname 会修改原字符串）
        char exe_path_copy[PATH_MAX];
        strcpy(exe_path_copy, exe_path);
        char* exe_dir = dirname(exe_path_copy);

        // 从可执行文件目录推导 installed 目录
        // 可执行文件: /home/lab/xxx/installed/lib/hardware_node/hardware_node
        // installed 目录: /home/lab/xxx/installed
        std::string installed_dir = std::string(exe_dir) + "/../../";
        std::string installed_path = installed_dir + "share/hardware_plant/lib/EC_Master/config/" + xml_filename;

        struct stat buffer;
        if (stat(installed_path.c_str(), &buffer) == 0) {
            std::cout << "[EC XML] ✓ 在 installed 路径找到: " << installed_path << std::endl;
            return installed_path;
        }
        std::cout << "[EC XML] ✗ 未在 installed 路径找到: " << installed_path << std::endl;
    } else {
        std::cerr << "[EC XML] ✗ 无法获取可执行文件路径: " << strerror(errno) << std::endl;
    }

    // 方法2: 使用原有的 GetAbsolutePathHW 方法 (闭源仓库，原方案)
    std::cout << "[EC XML] 使用原方案" << std::endl;
    return GetAbsolutePathHW(relative_path);
}


std::string load_ec_master_license()
{
    std::string ec_master_path = getKuavoEcMasterLicensePath();
    std::string content = "";
    std::ifstream file(ec_master_path);
#define VALID_EC_MASTER_LICENSE_STR_LEN 26
    if (!file)
    {
        std::cerr << "EC_master License: File " << ec_master_path << " does not exist." << std::endl;
    }
    else
    {
        std::string line;
        int lineCount = 0; // Add a counter for lines
        while (std::getline(file, line))
        {
            content += line;
            lineCount++;
        }
        if (lineCount > 1)
        {
            content = "";
            std::cerr << "EC_master License: File Format is wrong, have more than one lines string: " << content << std::endl;
        }
        else
        {
            content.erase(std::remove_if(content.begin(), content.end(), ::isspace), content.end());
            int contentLen = content.length();
            if (contentLen != VALID_EC_MASTER_LICENSE_STR_LEN)
            {
                std::cerr << "EC_master License len is invalid : the license str len is: " << contentLen << " The valid length is: " << VALID_EC_MASTER_LICENSE_STR_LEN << std::endl;
                content = "";
            }
            std::cout << "Extract ec_master license is: " << content << std::endl;
        }
    }
    return content;
}

void *setup_Ec_Master(void *arg)
{
    // thread_rt();
    // cpu_set_t cpu_mask;
    // CPU_ZERO(&cpu_mask);
    // // CPU_SET(active_cpu++, &cpu_mask);
    // CPU_SET(6, &cpu_mask);
    // set_cpu_mask(0, &cpu_mask);
    // get_cpu_mask(0, &cpu_mask);
    // printf("Ec_Master thread:");
    // print_cpu_mask(cpu_mask);

    sched_thread(80);
    // std::filesystem::path curr_path = std::filesystem::current_path();
    // std::filesystem::path config_path = curr_path / "../lib/EC_Master/config/elmo_24_c500.xml";
    // int countECMasters = std::count(HighlyDynamic::motor_info.driver.begin(), HighlyDynamic::motor_info.driver.end(), HighlyDynamic::MotorDriveType::EC_MASTER);
    EcActuatorParams *params = (EcActuatorParams *)arg;
    int countECMasters = params->num_actuators;
    std::string ec_type = params->ec_type;
    
    // 读取 ROBOT_VERSION 环境变量
    std::string robot_version = getRobotVersionFromEnv();
    bool is_robot_version_53 = (robot_version == "53");
    bool is_robot_version_54 = (robot_version == "54");
    
    std::string config_path;
    if (is_robot_version_53)
        config_path = GetAbsolutePathHW("lib/EC_Master/config/Kuavo5_T26_kpkd_ntc.xml");
    else if (is_robot_version_54)
        config_path = GetAbsolutePathHW("lib/EC_Master/config/Kuavo5_T25_V54.xml");
    else if (ec_type == "elmo")
        config_path = GetAbsolutePathHW("lib/EC_Master/config/elmo_" + std::to_string(countECMasters) + "_c500.xml");  //"_busshift.xml"
    else if (ec_type == "youda1" || ec_type == "youda")
        config_path = findECXmlConfigPath("lib/EC_Master/config/yd_"+ std::to_string(countECMasters) + "_c500.xml");
    else if (ec_type == "youda3")
        config_path = findECXmlConfigPath("lib/EC_Master/config/yd300_"+ std::to_string(countECMasters) + "_c500.xml");
    else if (ec_type == "leju")
        config_path = findECXmlConfigPath("lib/EC_Master/config/selfd_"+ std::to_string(countECMasters) + "_c500.xml");
    else if(ec_type == "lunbi")
        config_path = findECXmlConfigPath("lib/EC_Master/config/lb_6_c500.xml");
    else
    {
        std::cerr << "ecmaster_type:"<< ec_type << " is not supported! Please check the config file." << std::endl;
        exit(1);
    }

    std::cout << "Ec_Master config_path: " << config_path << std::endl;

    std::string ec_master_license_str = load_ec_master_license();
    
    // 获取主机名
    char hostname[256] = {0};
    if (gethostname(hostname, sizeof(hostname)) != 0) {
        std::cerr << "Warning: Failed to get hostname, using default network port configuration." << std::endl;
        strcpy(hostname, "unknown");
    }
    std::string hostname_str(hostname);
    std::cout << "Detected hostname: " << hostname_str << std::endl;

    // 根据主机名和ROBOT_VERSION选择网口号
    std::string network_port_arg;
    if (params->robot_module == "LUNBI") {
        network_port_arg = "-i8254x 1 1";
    } else if (hostname_str == "AAEON") {
        if (is_robot_version_53 || params->robot_module == "LUNBI_V62") {
            network_port_arg = "-i8254x 2 1";
        } else {
            network_port_arg = "-i8254x 2 1 -i8254x 3 1";
        }
    } else {
        network_port_arg = "-i8254x 1 1 -i8254x 2 1";
    }
    std::cout << "Selected network port argument: " << network_port_arg << std::endl;

    char *command[] = {
        (char *)"main",
        (char *)network_port_arg.c_str(),
        (char *)"-a",
        (char *)"7",
        (char *)"-v",
        (char *)"2",
        (char *)"-f",
        (char *)config_path.c_str(),
        (char *)"-auxclk",
        (char *)(params->robot_module == "ROBAN2" ? "1000" : "500"),
        (char *)"-dcmmode",
        (ec_type == "elmo")?(char *)"busshift":(char *)"mastershift",
        (char *)"-t",
        (char *)"0",
        (char *)"-lic",
        (char *)ec_master_license_str.c_str(),
    }; // keep this argument at last of command, for when ec_master license is not provided

    int ec_master_argc;

    ec_master_argc = sizeof(command) / sizeof(command[0]);

    if (ec_master_license_str.length() <= 0)
    {
        // ec_master_license is invalid, remove from the argv of ec_master_init;
        ec_master_argc -= 2;
    }

    Ec_Master_init(ec_master_argc, command);
    std::cout << "Ecmaster exited\n";
    std::raise(SIGINT);
    return NULL;
}

int ECMaster_init(EcActuatorParams params)
{
    int ret = 0;
    ret = pthread_create(&ecmaster_thread, NULL, setup_Ec_Master, (void *)&params);
    if (ret != 0)
    {
        printf("Create lcm_thread failed! %s\n", strerror(ret));
        return -1;
    }

    while (1)
    {
        if (isMotorEnable())
        {
            break;
        }
        OsSleep(1000);
    }
    std::cout << "ECMaster_init DONE !\n";
    return 0;
}
void EM_deInit(void)
{
    EcMasterStop();
}
int8_t actuatorsInterfaceSetup(const char *type, ActuatorsInterface_t *interfacePtr)
{
    if (strcmp(type, "real") == 0)
    {
        interfacePtr->init = ECMaster_init;
        interfacePtr->deInit = EM_deInit;
        // interfacePtr->setJointOffset = EM_setPositionsOffset; // 已由EcMaster读取 offset配置文件
        interfacePtr->setJointPosition = motorSetPosition;
        interfacePtr->setJointVelocity = motorSetVelocity;
        // interfacePtr->setJointTorque = motorSetTorque;
        interfacePtr->setJointTorque = motorSetTorqueWithFeedback;
        interfacePtr->setEncoderRange = setEcEncoderRange;

        interfacePtr->getJointData = motorGetData;
        interfacePtr->addIgnore = disableMotor;
        interfacePtr->setRobotMoudle = setRobotMoudle;

        interfacePtr->setJointKp = motorSetkp;
        interfacePtr->setJointKd = motorSetkd;

        interfacePtr->readJointKp = motorReadKp;
        interfacePtr->readJointKd = motorReadKd;

        interfacePtr->writeJointKp = motorWriteKp;
        interfacePtr->writeJointKd = motorWriteKd;
        printf("Info: actuatorsInterfaceSetup success!\n");
        return 0;
    }
    printf("Error: actuatorsInterfaceSetup failed!\n");
    return -1;
}
