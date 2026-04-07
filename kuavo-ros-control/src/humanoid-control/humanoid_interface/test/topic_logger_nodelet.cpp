#include <ros/ros.h>
// #include "common/TopicLogger.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <vector>
#include <string>
#include <pwd.h>
#include <filesystem>
#include <set>
#include <utility>
#include <sys/stat.h>

#include "humanoid_interface/common/recorder.h"

#include "rosbag/exceptions.h"
#include <boost/filesystem.hpp>
#include "boost/program_options.hpp"
#include <yaml-cpp/yaml.h>

#include <signal.h>
#include <string>
#include <sstream>
#include <ros/callback_queue.h>
#include <ros/topic.h>
#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
// #include <rostopic/rostopic.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>

namespace po = boost::program_options;
namespace fs = std::filesystem;

//! Parse the command-line arguments for recorder options
rosbag::rosbagNodelet::RecorderOptions parseLoggerOptions(int argc, char **argv)
{
  rosbag::rosbagNodelet::RecorderOptions opts;

  po::options_description desc("Allowed options");

  desc.add_options()("help,h", "produce help message")("all,a", "record all topics")("regex,e", "match topics using regular expressions")("exclude,x", po::value<std::string>(), "exclude topics matching regular expressions")("quiet,q", "suppress console output")("publish,p", "Publish a msg when the record begin")("output-prefix,o", po::value<std::string>(), "prepend PREFIX to beginning of bag name")("output-name,O", po::value<std::string>(), "record bagnamed NAME.bag")("buffsize,b", po::value<int>()->default_value(256), "Use an internal buffer of SIZE MB (Default: 256)")("chunksize", po::value<int>()->default_value(768), "Set chunk size of message data, in KB (Default: 768. Advanced)")("limit,l", po::value<int>()->default_value(0), "Only record NUM messages on each topic")("min-space,L", po::value<std::string>()->default_value("1G"), "Minimum allowed space on recording device (use G,M,k multipliers)")("bz2,j", "use BZ2 compression")("lz4", "use LZ4 compression")("split", po::value<int>()->implicit_value(0), "Split the bag file and continue recording when maximum size or maximum duration reached.")("max-splits", po::value<int>(), "Keep a maximum of N bag files, when reaching the maximum erase the oldest one to keep a constant number of files.")("topic", po::value<std::vector<std::string>>(), "topic to record")("size", po::value<uint64_t>(), "The maximum size of the bag to record in MB.")("duration", po::value<std::string>(), "Record a bag of maximum duration in seconds, unless 'm', or 'h' is appended.")("node", po::value<std::string>(), "Record all topics subscribed to by a specific node.")("tcpnodelay", "Use the TCP_NODELAY transport hint when subscribing to topics.")("udp", "Use the UDP transport hint when subscribing to topics.");

  po::positional_options_description p;
  p.add("topic", -1);

  po::variables_map vm;

  try
  {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  }
  catch (const boost::program_options::invalid_command_line_syntax &e)
  {
    throw ros::Exception(e.what());
  }
  catch (const boost::program_options::unknown_option &e)
  {
    throw ros::Exception(e.what());
  }

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(0);
  }

  if (vm.count("all"))
    opts.record_all = true;
  if (vm.count("regex"))
    opts.regex = true;
  if (vm.count("exclude"))
  {
    opts.do_exclude = true;
    opts.exclude_regex = vm["exclude"].as<std::string>();
  }
  if (vm.count("quiet"))
    opts.quiet = true;
  if (vm.count("publish"))
    opts.publish = true;
  if (vm.count("output-prefix"))
  {
    opts.prefix = vm["output-prefix"].as<std::string>();
    opts.append_date = true;
  }
  if (vm.count("output-name"))
  {
    opts.prefix = vm["output-name"].as<std::string>();
    opts.append_date = false;
  }
  if (vm.count("split"))
  {
    opts.split = true;

    int S = vm["split"].as<int>();
    if (S != 0)
    {
      ROS_WARN("Use of \"--split <MAX_SIZE>\" has been deprecated.  Please use --split --size <MAX_SIZE> or --split --duration <MAX_DURATION>");
      if (S < 0)
        throw ros::Exception("Split size must be 0 or positive");
      opts.max_size = 1048576 * static_cast<uint64_t>(S);
    }
  }
  if (vm.count("max-splits"))
  {
    if (!opts.split)
    {
      ROS_WARN("--max-splits is ignored without --split");
    }
    else
    {
      opts.max_splits = vm["max-splits"].as<int>();
    }
  }
  if (vm.count("buffsize"))
  {
    int m = vm["buffsize"].as<int>();
    if (m < 0)
      throw ros::Exception("Buffer size must be 0 or positive");
    opts.buffer_size = 1048576 * m;
  }
  if (vm.count("chunksize"))
  {
    int chnk_sz = vm["chunksize"].as<int>();
    if (chnk_sz < 0)
      throw ros::Exception("Chunk size must be 0 or positive");
    opts.chunk_size = 1024 * chnk_sz;
  }
  if (vm.count("limit"))
  {
    opts.limit = vm["limit"].as<int>();
  }
  if (vm.count("min-space"))
  {
    std::string ms = vm["min-space"].as<std::string>();
    long long int value = 1073741824ull;
    char mul = 0;
    // Sane default values, just in case
    opts.min_space_str = "1G";
    opts.min_space = value;
    if (sscanf(ms.c_str(), " %lld%c", &value, &mul) > 0)
    {
      opts.min_space_str = ms;
      switch (mul)
      {
      case 'G':
      case 'g':
        opts.min_space = value * 1073741824ull;
        break;
      case 'M':
      case 'm':
        opts.min_space = value * 1048576ull;
        break;
      case 'K':
      case 'k':
        opts.min_space = value * 1024ull;
        break;
      default:
        opts.min_space = value;
        break;
      }
    }
    ROS_DEBUG("Rosbag using minimum space of %lld bytes, or %s", opts.min_space, opts.min_space_str.c_str());
  }
  if (vm.count("bz2") && vm.count("lz4"))
  {
    throw ros::Exception("Can only use one type of compression");
  }
  if (vm.count("bz2"))
  {
    opts.compression = rosbag::compression::BZ2;
  }
  if (vm.count("lz4"))
  {
    opts.compression = rosbag::compression::LZ4;
  }
  if (vm.count("duration"))
  {
    std::string duration_str = vm["duration"].as<std::string>();

    double duration;
    double multiplier = 1.0;
    std::string unit("");

    std::istringstream iss(duration_str);
    if ((iss >> duration).fail())
      throw ros::Exception("Duration must start with a floating point number.");

    if ((!iss.eof() && ((iss >> unit).fail())))
    {
      throw ros::Exception("Duration unit must be s, m, or h");
    }
    if (unit == std::string(""))
      multiplier = 1.0;
    else if (unit == std::string("s"))
      multiplier = 1.0;
    else if (unit == std::string("m"))
      multiplier = 60.0;
    else if (unit == std::string("h"))
      multiplier = 3600.0;
    else
      throw ros::Exception("Duration unit must be s, m, or h");

    opts.max_duration = ros::Duration(duration * multiplier);
    if (opts.max_duration <= ros::Duration(0))
      throw ros::Exception("Duration must be positive.");
  }
  if (vm.count("size"))
  {
    opts.max_size = vm["size"].as<uint64_t>() * 1048576;
    if (opts.max_size <= 0)
      throw ros::Exception("Split size must be 0 or positive");
  }
  if (vm.count("node"))
  {
    opts.node = vm["node"].as<std::string>();
    std::cout << "Recording from: " << opts.node << std::endl;
  }
  if (vm.count("tcpnodelay"))
  {
    opts.transport_hints.tcpNoDelay();
  }
  if (vm.count("udp"))
  {
    opts.transport_hints.udp();
  }

  // Every non-option argument is assumed to be a topic
  if (vm.count("topic"))
  {
    std::vector<std::string> bags = vm["topic"].as<std::vector<std::string>>();
    std::sort(bags.begin(), bags.end());
    bags.erase(std::unique(bags.begin(), bags.end()), bags.end());
    for (std::vector<std::string>::iterator i = bags.begin();
         i != bags.end();
         i++)
      opts.topics.push_back(*i);
  }

  // check that argument combinations make sense
  if (opts.exclude_regex.size() > 0 &&
      !(opts.record_all || opts.regex))
  {
    fprintf(stderr, "Warning: Exclusion regex given, but no topics to subscribe to.\n"
                    "Adding implicit 'record all'.");
    opts.record_all = true;
  }
  return opts;
}

/**
 * Handle SIGTERM to allow the recorder to cleanup by requesting a shutdown.
 * \param signal
 */
// void signal_handler(int signal)
// {
//   // std::cout << "Received signal " << signal << ", requesting shutdown." << std::endl;
//   (void)signal;
//   ros::requestShutdown();
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "record", ros::init_options::AnonymousName);

//   // handle SIGTERM signals
//   signal(SIGTERM, signal_handler);

//   // Parse the command-line options
//   rosbag::RecorderOptions opts;
//   try
//   {
//     opts = parseOptions(argc, argv);
//   }
//   catch (const ros::Exception &ex)
//   {
//     ROS_ERROR("Error reading options: %s", ex.what());
//     return 1;
//   }
//   catch (const boost::regex_error &ex)
//   {
//     ROS_ERROR("Error reading options: %s\n", ex.what());
//     return 1;
//   }

//   // Run the recorder
//   rosbag::Recorder recorder(opts);
//   int result = recorder.run();

//   return result;
// }
rosbag::rosbagNodelet::Recorder *recorder_ptr;
std::thread control_thread;
std::string getHomePath()
{
  uid_t uid;
  char *sudoUser = getenv("SUDO_USER");

  if (sudoUser != NULL)
  {
    struct passwd *pw = getpwnam(sudoUser);
    if (pw)
    {
      return std::string(pw->pw_dir);
    }
  }
  else
  {
    uid = getuid();
    struct passwd *pw = getpwuid(uid);
    if (pw)
    {
      return std::string(pw->pw_dir);
    }
  }
  return "";
}
void cleanRosLogDirectory()
{
  std::string homeDir = getHomePath();
  std::string rosLogDir = homeDir + "/.ros";

  // 检查 .ros 目录是否存在
  if (!fs::exists(rosLogDir) || !fs::is_directory(rosLogDir))
  {
    std::cout << "ROS log directory not found: " << rosLogDir << std::endl;
    return;
  }

  // 从 ROS 参数中读取配置
  size_t maxSize = 10ull * 1024 * 1024 * 1024; // 默认 10GB
  size_t logDeleteLimit = 8ull * 1024 * 1024 * 1024; // 默认 8GB
  
  // 尝试从 ROS 参数读取 maxSize (单位: GB)
  double maxSizeGB = 10.0; // 默认值
  if (ros::param::get("/nodelet_manager/max_log_size_gb", maxSizeGB))
  {
    if (maxSizeGB > 0)
    {
      maxSize = static_cast<size_t>(maxSizeGB * 1024 * 1024 * 1024);
    }
    else
    {
      ROS_ERROR("Invalid max_log_size_gb parameter, using default value: 10GB");
    }
  }
  
  // 尝试从 ROS 参数读取 logDeleteLimit (单位: GB)
  double logDeleteLimitGB = 8.0; // 默认值
  if (ros::param::get("/nodelet_manager/log_delete_limit_gb", logDeleteLimitGB))
  {
    if (logDeleteLimitGB >= 0 && logDeleteLimitGB <= maxSizeGB)
    {
      logDeleteLimit = static_cast<size_t>(logDeleteLimitGB * 1024 * 1024 * 1024);
    }
    else
    {
      ROS_ERROR("Invalid log_delete_limit_gb parameter, using default value: 8GB");
    }
  }

  // 递归获取所有文件及其大小
  std::vector<std::pair<fs::path, size_t>> filesWithSize;
  size_t totalSize = 0;

  std::set<fs::path> directoriesToCheck; // 记录需要检查是否为空目录的文件夹
  
  try
  {
    for (const auto &entry : fs::recursive_directory_iterator(rosLogDir))
    {
      if (fs::is_regular_file(entry))
      {
        // 使用stat获取实际磁盘占用大小
        struct stat st;
        if (stat(entry.path().c_str(), &st) == 0)
        {
          // st.st_blocks 是以512字节块为单位的磁盘占用
          size_t diskSize = st.st_blocks * 512;
          filesWithSize.push_back({entry.path(), diskSize});
          totalSize += diskSize;
        }
        else
        {
          // 如果stat失败，回退到文件大小
          size_t fileSize = fs::file_size(entry.path());
          filesWithSize.push_back({entry.path(), fileSize});
          totalSize += fileSize;
        }
      }
      else if (fs::is_directory(entry) && fs::is_empty(entry))
      {
        directoriesToCheck.insert(entry.path());
      }
    }
  }
  catch (const fs::filesystem_error& e)
  {
    std::cerr << "Error accessing directory: " << e.what() << std::endl;
    return;
  }

  if (totalSize <= maxSize)
  {
    ROS_INFO("Total size of ROS log files %2.2fGB is less than maxSize: %2.2fGB", totalSize / (1024.0 * 1024 * 1024), maxSize / (1024.0 * 1024 * 1024));
    return;
  }

  // 按照最后修改时间排序（最旧的在前，用于倒序删除）
  std::sort(filesWithSize.begin(), filesWithSize.end(), 
    [](const std::pair<fs::path, size_t> &p1, const std::pair<fs::path, size_t> &p2)
    {
      try
      {
        return fs::last_write_time(p1.first) < fs::last_write_time(p2.first);
      }
      catch (const fs::filesystem_error& e)
      {
        std::cerr << "Error comparing timestamps: " << e.what() << std::endl;
        return false;
      }
    });

  // 如果总大小超过 maxSize，则删除文件直到达到 logDeleteLimit
  size_t currentSize = totalSize;
  size_t removedFiles = 0;
  
  // 从最旧的文件开始删除，直到达到目标大小
  for (const auto &fileInfo : filesWithSize)
  {
    const fs::path &file = fileInfo.first;
    size_t fileSize = fileInfo.second;
    
    // 只有当当前总大小超过 logDeleteLimit 时才删除文件
    if (currentSize > logDeleteLimit)
    {
      try
      {
        fs::remove(file);
        removedFiles++;
        currentSize -= fileSize; // 更新总大小
        
        // 记录需要检查的父目录
        fs::path parentDir = file.parent_path();
        while (parentDir != rosLogDir && parentDir.has_parent_path())
        {
          directoriesToCheck.insert(parentDir);
          parentDir = parentDir.parent_path();
        }
      }
      catch (const fs::filesystem_error& e)
      {
        std::cerr << "Error removing file " << file << ": " << e.what() << std::endl;
      }      
    }
    else
    {
      break;
    }
  }

  // 清理空目录（除了 rosLogDir 和其一级子目录）
  size_t removedDirs = 0;
  // for (const auto &dir : directoriesToCheck)
  // {
  //   try
  //   {
  //     // 检查是否是 rosLogDir 或其一级子目录
  //     fs::path relativePath = fs::relative(dir, rosLogDir);
  //     if (!relativePath.empty() && relativePath.string().find('/') != std::string::npos)
  //     {
  //       // 不是一级子目录，可以删除
  //       if (fs::is_empty(dir))
  //       {
  //         fs::remove(dir);        
  //         removedDirs++;
  //       }
  //     }
  //   }
  //   catch (const fs::filesystem_error& e)
  //   {
  //     std::cerr << "Error checking/removing directory " << dir << ": " << e.what() << std::endl;
  //   }
  // }

  std::cout << "ROS log directory cleaning... \n"
            << "Max size: " << (maxSize / (1024 * 1024 * 1024.0)) << "GB, "
            << "Delete limit: " << (logDeleteLimit / (1024 * 1024 * 1024.0)) << "GB\n"
            << "Size before cleaning: " << (totalSize / (1024 * 1024 * 1024.0)) << "GB, "
            << "Size after cleaning: " << (currentSize / (1024 * 1024 * 1024.0)) << "GB, "
            << "Removed " << removedFiles << " files, " << removedDirs << " empty directories" << std::endl;
}

void cleanRosCoredumpDirectory()
{
  std::string homeDir = getHomePath();
  std::string coredumpDir = homeDir + "/.ros/coredumps";

  // 检查 coredump 目录是否存在
  if (!fs::exists(coredumpDir) || !fs::is_directory(coredumpDir))
  {
    std::cout << "ROS coredump directory not found: " << coredumpDir << std::endl;
    return;
  }


  size_t coredumpsToKeep = 20; // 默认值
  double coredumpsToKeepParam = 20.0; // 用于ROS参数的临时变量

  if (ros::param::get("/nodelet_manager/coredumps_to_keep", coredumpsToKeepParam))
  {
    if (coredumpsToKeepParam > 0)
    {
      coredumpsToKeep = static_cast<size_t>(coredumpsToKeepParam);
    }
    else
    {
      ROS_ERROR("Invalid coredumps_to_keep parameter, using default value: 20");
    }
  }

  // 获取所有coredump目录（以数字命名的目录）
  std::vector<fs::path> coredumpDirs;
  size_t totalCoredumps = 0;
  size_t removedCoredumps = 0;

  try
  {
    for (const auto &entry : fs::directory_iterator(coredumpDir))
    {
      if (fs::is_directory(entry))
      {
        std::string dirName = entry.path().filename().string();
        // 检查是否是纯数字目录名（coredump目录）
        if (std::all_of(dirName.begin(), dirName.end(), ::isdigit))
        {
          coredumpDirs.push_back(entry.path());
          totalCoredumps++;
        }
      }
    }
  }
  catch (const fs::filesystem_error& e)
  {
    std::cerr << "Error accessing coredump directory: " << e.what() << std::endl;
    return;
  }

  // 如果coredump数量少于设定值，不需要清理
  if (totalCoredumps <= coredumpsToKeep)
  {
    std::cout << "ROS coredump directory has " << totalCoredumps
              << " coredumps, no cleaning needed." << std::endl;
    return;
  }

  // 按照最后修改时间排序（最新的在前）
  std::sort(coredumpDirs.begin(), coredumpDirs.end(),
    [](const fs::path &p1, const fs::path &p2)
    {
      try
      {
        return fs::last_write_time(p1) > fs::last_write_time(p2);
      }
      catch (const fs::filesystem_error& e)
      {
        std::cerr << "Error comparing timestamps: " << e.what() << std::endl;
        return false;
      }
    });

  for (size_t i = coredumpsToKeep; i < coredumpDirs.size(); ++i)
  {
    try
    {
      // 计算目录大小
      size_t dirSize = 0;
      for (const auto &file : fs::recursive_directory_iterator(coredumpDirs[i]))
      {
        if (fs::is_regular_file(file))
        {
          dirSize += fs::file_size(file);
        }
      }

      if (fs::remove_all(coredumpDirs[i]) > 0)
      {
        removedCoredumps++;
        std::cout << "Removed coredump: " << coredumpDirs[i].filename().string()
                  << " (size: " << (dirSize / (1024.0 * 1024.0)) << "MB)" << std::endl;
      }
    }
    catch (const fs::filesystem_error& e)
    {
      std::cerr << "Error removing coredump directory " << coredumpDirs[i]
                << ": " << e.what() << std::endl;
    }
  }

  std::cout << "ROS coredump directory cleaning...\n"
            << "Kept " << coredumpsToKeep << " newest coredumps\n"
            << "Removed " << removedCoredumps << " old coredumps\n"
            << "Total coredumps after cleaning: "
            << (totalCoredumps - removedCoredumps) << std::endl;
}

void cleanRosStdoutDirectory()
{
    std::string homeDir = getHomePath();
    std::string rosStdoutDir = homeDir + "/.ros/stdout";

    // 检查 .ros/stdout 目录是否存在
    if (!fs::exists(rosStdoutDir) || !fs::is_directory(rosStdoutDir))
    {
        std::cout << "ROS stdout directory not found: " << rosStdoutDir << std::endl;
        return;
    }

    // 获取 stdout 目录下的所有文件夹
    std::vector<fs::path> directories;
    try 
    {
        for (const auto &entry : fs::directory_iterator(rosStdoutDir))
        {
            if (fs::is_directory(entry))
            {
                directories.push_back(entry.path());
            }
        }
    }
    catch (const fs::filesystem_error& e)
    {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        return;
    }

    // 如果文件夹数量小于等于100，不需要清理
    if (directories.size() <= 100)
    {
        std::cout << "ROS stdout directory has " << directories.size() 
                  << " folders, no cleaning needed." << std::endl;
        return;
    }

    // 按照最后修改时间排序（最新的在前）
    std::sort(directories.begin(), directories.end(), 
        [](const fs::path &p1, const fs::path &p2)
        {
            try
            {
                return fs::last_write_time(p1) > fs::last_write_time(p2);
            }
            catch (const fs::filesystem_error& e)
            {
                std::cerr << "Error comparing timestamps: " << e.what() << std::endl;
                return false;
            }
        });

    // 保留最新的100个文件夹，删除其余的
    size_t foldersToKeep = 100;
    size_t removedFolders = 0;

    for (size_t i = foldersToKeep; i < directories.size(); ++i)
    {
        try
        {
            if (fs::remove_all(directories[i]) > 0)
            {
                removedFolders++;
            }
        }
        catch (const fs::filesystem_error& e)
        {
            std::cerr << "Error removing directory " << directories[i] 
                      << ": " << e.what() << std::endl;
        }
    }

    std::cout << "ROS stdout directory cleaning...\n"
              << "Kept " << foldersToKeep << " newest folders\n"
              << "Removed " << removedFolders << " old folders\n"
              << "Total folders after cleaning: "
              << (directories.size() - removedFolders) << std::endl;
}

void cleanRosBagDirectory()
{
  // 从 ROS 参数服务器读取 enable_bag_cleanup 参数，默认值为 false
  bool enable_bag_cleanup = false;
  if (ros::param::get("/nodelet_manager/enable_bag_cleanup", enable_bag_cleanup))
  {
    ROS_INFO_NAMED("cleanRosBagDirectory", "Loaded enable_bag_cleanup from parameter server: %s", enable_bag_cleanup ? "true" : "false");
  }
  else
  {
    ROS_INFO_NAMED("cleanRosBagDirectory", "enable_bag_cleanup parameter not found, using default value: false");
  }

  // 如果参数为 false，直接返回，不执行清理逻辑
  if (!enable_bag_cleanup)
  {
    ROS_INFO_NAMED("cleanRosBagDirectory", "enable_bag_cleanup is false, skip cleanRosBagDirectory.");
    return;
  }

  std::string homeDir = getHomePath();
  std::string rosBagDir = homeDir + "/.ros";

  // 检查 .ros 目录是否存在
  if (!fs::exists(rosBagDir) || !fs::is_directory(rosBagDir))
  {
    std::cout << "ROS bag directory not found: " << rosBagDir << std::endl;
    return;
  }

  // 获取 .ros 目录下所有的.bag 和.bag.active 文件
  std::vector<fs::path> bagFiles;

  try
  {
    for (const auto &entry : fs::directory_iterator(rosBagDir))
    {
      if (fs::is_regular_file(entry))
      {
        std::string filename = entry.path().filename().string();
        // 检查是否是.bag 或.bag.active 文件
        bool isBagFile = false;
        if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bag")
        {
          isBagFile = true;
        }
        else if (filename.size() > 11 && filename.substr(filename.size() - 11) == ".bag.active")
        {
          isBagFile = true;
        }
        
        if (isBagFile)
        {
          bagFiles.push_back(entry.path());
        }
      }
    }
  }
  catch (const fs::filesystem_error& e)
  {
    std::cerr << "Error accessing bag directory: " << e.what() << std::endl;
    return;
  }

  // 如果没有找到任何 bag 文件，直接返回
  if (bagFiles.empty())
  {
    std::cout << "No bag files found in " << rosBagDir << std::endl;
    return;
  }

  size_t bagsToKeep = 8; // 默认保留 8 个最新的 bag 文件
  int bagsToKeepParam = 8; // 用于 ROS 参数的临时变量

  // 尝试从 ROS 参数读取保留数量
  if (ros::param::get("/nodelet_manager/bags_to_keep", bagsToKeepParam))
  {
    if (bagsToKeepParam > 0)
    {
      bagsToKeep = static_cast<size_t>(bagsToKeepParam);
    }
    else
    {
      ROS_ERROR("Invalid bags_to_keep parameter, using default value: 8");
    }
  }

  // 如果 bag 文件数量少于设定值，不需要清理
  if (bagFiles.size() <= bagsToKeep)
  {
    std::cout << "ROS bag directory has " << bagFiles.size()
              << " bag files, no cleaning needed." << std::endl;
    return;
  }

  // 按照最后修改时间排序（最新的在前）
  std::sort(bagFiles.begin(), bagFiles.end(),
    [](const fs::path &p1, const fs::path &p2)
    {
      try
      {
        return fs::last_write_time(p1) > fs::last_write_time(p2);
      }
      catch (const fs::filesystem_error& e)
      {
        ROS_WARN("Error comparing timestamps: %s", e.what());
        return false;
      }
    });

  // 删除超出数量的 bag 文件
  size_t removedBags = 0;
  size_t totalSizeRemoved = 0;

  for (size_t i = bagsToKeep; i < bagFiles.size(); ++i)
  {
    try
    {
      // 检查文件是否仍然存在
      if (!fs::exists(bagFiles[i]))
      {
        ROS_WARN("Bag file %s no longer exists, skipping",
                 bagFiles[i].filename().string().c_str());
        continue;
      }

      size_t fileSize = fs::file_size(bagFiles[i]);
      if (fs::remove(bagFiles[i]))
      {
        removedBags++;
        totalSizeRemoved += fileSize;
        std::cout << "Removed bag file: " << bagFiles[i].filename().string()
                  << " (size: " << (fileSize / (1024.0 * 1024.0)) << "MB)" << std::endl;
      }
    }
    catch (const fs::filesystem_error& e)
    {
      ROS_WARN("Error removing bag file %s: %s",
               bagFiles[i].filename().string().c_str(), e.what());
      // 继续处理下一个文件，不中断整个清理过程
      continue;
    }
  }

  std::cout << "ROS bag directory cleaning...\n"
            << "Kept " << bagsToKeep << " newest bag files\n"
            << "Removed " << removedBags << " old bag files\n"
            << "Total size removed: " << (totalSizeRemoved / (1024.0 * 1024.0)) << "MB\n"
            << "Total bag files after cleaning: "
            << (bagFiles.size() - removedBags) << std::endl;
}

class RosbagNodelet : public nodelet::Nodelet
{
public:
  std::vector<std::string> loadTopicsFromConfig(const std::string &filename)
  {
      std::vector<std::string> topics;
      std::cout << "Loading topics from config file: " << filename << std::endl;
      YAML::Node config = YAML::LoadFile(filename);
      if (config["topics"])
      {
          for (const auto &topic : config["topics"])
          {
              std::string topic_str = topic.as<std::string>();
              topics.push_back(topic_str);  // 添加到topics向量中
          }
      }
      else
      {
          std::cerr << "No 'topics' key found in the configuration file." << std::endl;
      }

      return topics;
  }
  virtual void onInit()
  {
    NODELET_INFO("Initializing RosbagNodelet nodelet...");
    nh = getNodeHandle();
    ros::param::set("/nodelet_manager/logger_state", 0);

    // 从 ROS 参数服务器读取 max_splits 参数，默认值为 60
    int max_splits = 60;
    if (nh.hasParam("/nodelet_manager/max_splits"))
    {
      nh.getParam("/nodelet_manager/max_splits", max_splits);
      NODELET_INFO("Loaded max_splits from parameter server: %d", max_splits);
    }
    else
    {
      NODELET_INFO("max_splits parameter not found, using default value: 60");
    }

    args = {" "};
    args.push_back("--size=500");
    args.push_back("--max-splits=" + std::to_string(max_splits));
    args.push_back("--split");
    // args.push_back("-a");
    if (nh.hasParam("topic_config"))
    {
      std::string topic_config;
      nh.getParam("topic_config", topic_config);
      if (!boost::filesystem::exists(topic_config))
      {          
          std::cerr << "文件不存在: " << topic_config << std::endl;
          return;
      }
      std::vector<std::string> topics = loadTopicsFromConfig(topic_config);
      if (topics.empty())
      {
        NODELET_ERROR("No topics found in the configuration file.");
        return;
      }
      args.push_back("-e");
      std::cout << "Log Topics number: " << topics.size() <<  std::endl;
      for (auto &topic : topics)
      {
        args.push_back(topic);
      }
    }else
    {
      NODELET_ERROR("No topic_config found in the parameter server, NOT recording any topics.");
      return;
    }
    
    // std::cout << "Arg:";
    // for (auto &arg : args)
    // {
    //   std::cout << arg << " ";
    // }
    // std::cout << std::endl;
    argc = args.size();

    NODELET_INFO("========================================");
    NODELET_INFO("      START CLEAN ROS LOG DIRECTORY     ");
    NODELET_INFO("========================================");
    cleanRosLogDirectory();
    cleanRosCoredumpDirectory();
    cleanRosBagDirectory();
    // cleanRosStdoutDirectory();
    NODELET_INFO("========================================");
    NODELET_INFO("     CLEAN ROS LOG DIRECTORY FINISHED   ");
    NODELET_INFO("========================================");
    control_thread = std::thread(&RosbagNodelet::controlLoop, this);
    NODELET_INFO("RosbagNodelet nodelet initialized.");
  }
  ~RosbagNodelet() override
  {
    std::cout << "[RosbagNodelet] nodelet destructor called." << std::endl;
    if (recorder_ptr)
    {
      recorder_ptr->stop();
      control_thread.join();
      recorder_ptr = nullptr;
    }
  }
  static void signal_handler(int signal)
  {

    std::cout << "[RosbagNodelet] Received signal " << signal << ", requesting shutdown." << std::endl;
    recorder_ptr->stop();
    control_thread.join();
    // (void)signal;
    exit(0);
  }

  void stopCallback(const std_msgs::Bool::ConstPtr &msg)
  {

 

    if (recorder_ptr->isStop())
    {
      std::cout << "[RosbagNodelet] recorder already stoped, ignore stop_robot request" << std::endl;
      return;
    }
    bool is_real = false;
    if (ros::param::has("/real"))
    {
      ros::param::get("/real", is_real);
    }

    if (msg->data)
    {
      std::cout << "[RosbagNodelet] Received stop_robot request" << std::endl;
      

      recorder_ptr->stop();
      control_thread.join();
      NODELET_INFO("========================================");
      NODELET_INFO("      START CLEAN ROS DIRECTORIES       ");
      NODELET_INFO("========================================");
      cleanRosLogDirectory();
      cleanRosCoredumpDirectory();
      // cleanRosStdoutDirectory();
      NODELET_INFO("========================================");
      NODELET_INFO("     CLEAN ROS DIRECTORIES FINISHED     ");
      NODELET_INFO("========================================");
      ros::param::set("/nodelet_manager/logger_state", 1);


      // 检查nodelet状态
      const double max_wait_seconds = 20.0; // 最大等待5秒
      const int check_interval_ms = 100;   // 每100ms检查一次
      const int max_checks = static_cast<int>(max_wait_seconds * 1000 / check_interval_ms);
      int check_count = 0;
      
      while (check_count < max_checks && ros::ok() && is_real)
      {
        int nodelet_state;
        if (ros::param::has("/nodelet_manager/hardware_state") && ros::param::get("/nodelet_manager/hardware_state", nodelet_state))
        {
          if (nodelet_state == 1)// 硬件节点已经退出了
          {
            ROS_INFO("[TopicLogger] Nodelets ready to exit, state: 1");
            break;
          }
        }

        usleep(check_interval_ms * 1000); // 转换为微秒
        check_count++;

        // 输出剩余等待时间
        double remaining_time = max_wait_seconds - (check_count * check_interval_ms / 1000.0);
        if (check_count % 10 == 0) // 每秒输出一次
        {
          ROS_INFO_STREAM("[TopicLogger] Waiting for nodelets to be ready... " << std::fixed << std::setprecision(1)
                                                                                << remaining_time << "s remaining, current state: "
                                                                                << nodelet_state);
        }
      }

      if (check_count >= max_checks)
      {
        ROS_WARN("[TopicLogger] Timeout after %.1f seconds waiting for nodelets", max_wait_seconds);
      }
      
      ros::param::set("/nodelet_manager/logger_state", 2);


      ros::requestShutdown();
    }
    return;
  }

private:
  bool pause_flag;
  int argc;
  std::vector<std::string> args;
  ros::NodeHandle nh;
  ros::Subscriber pause_sub;
  ros::Subscriber stop_sub;
  std::chrono::high_resolution_clock::time_point lastTime;
  
  void controlLoop()
  {
    stop_sub = nh.subscribe<std_msgs::Bool>("/stop_robot", 10, &RosbagNodelet::stopCallback, this);

    char **argv = new char *[argc];
    for (int i = 0; i < argc; ++i)
    {
      argv[i] = const_cast<char *>(args[i].c_str());
    }
    // signal(SIGINT, this->signal_handler);
    // Parse the command-line options
    rosbag::rosbagNodelet::RecorderOptions opts;
    try
    {
      opts = parseLoggerOptions(argc, argv);
    }
    catch (const ros::Exception &ex)
    {
      ROS_ERROR("Error reading options: %s", ex.what());
      return;
    }
    catch (const boost::regex_error &ex)
    {
      ROS_ERROR("Error reading options: %s\n", ex.what());
      return;
    }

    // Run the recorder
    recorder_ptr = new rosbag::rosbagNodelet::Recorder(opts);
    int result = recorder_ptr->run(nh);
    std::cout << "controlLoop: Exiting control loop" << std::endl;
  }
};

PLUGINLIB_EXPORT_CLASS(RosbagNodelet, nodelet::Nodelet)
