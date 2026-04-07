#include "utils.h"
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <unistd.h>
#include <streambuf>
#include <experimental/filesystem>
#include <iomanip>
#include <chrono>
#include <pwd.h>

// std::filesystem::path file_path = __FILE__;
// std::filesystem::path CURRENT_SOURCE_DIR = file_path.parent_path().parent_path();

void setHardwarePlantPath(const std::string &path)
{
    hardware_plant_dir = path;
}

std::string GetAbsolutePathHW(const std::string &path)
{
    std::cout << "hardware GetAbsolutePathHW called" << std::endl;
    std::string abs_path = path;
    if (path[0] != '/')
    {
        abs_path = (hardware_plant_dir + "/" + path);
    }
    std::cout << "hardware abs_path: " << abs_path << std::endl;
    return abs_path;
}
