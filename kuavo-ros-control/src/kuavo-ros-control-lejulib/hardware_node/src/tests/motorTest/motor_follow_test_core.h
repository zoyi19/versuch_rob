#ifndef MOTOR_FOLLOW_TEST_CORE_H
#define MOTOR_FOLLOW_TEST_CORE_H

#include <vector>
#include <string>
#include <map>
#include <jsoncpp/json/json.h>
#include <mutex>
#include <thread>
#include <atomic>
#include <memory>
#include <Eigen/Dense>
#include <kuavo_common/common/sensor_data.h>
#include <chrono>
#include <fstream>
#include <filesystem>

// 前向声明
struct JointCmd_t;

// 独立的数据结构，替代ROS消息类型
struct JointCmd_t {
    std::vector<double> joint_pos;    // 关节位置指令
    std::vector<double> joint_vel;    // 关节速度指令
    std::vector<double> joint_torque; // 关节力矩指令
    std::vector<int> control_mode;    // 控制模式
    double timestamp;                 // 时间戳
};

namespace motor_follow_test {

/**
 * @brief 电机跟随测试核心类 - 解耦的电机指令生成器
 * 
 * 这个类负责：
 * 1. 根据机器人版本和配置文件生成电机指令
 * 2. 处理传感器数据
 * 3. 执行电机跟随测试逻辑
 * 4. 不依赖特定的ROS环境（仿真或实物）
 */
class MotorFollowTestCore {
public:
    struct JointGroupIndices {
        int waist_start, waist_end;
        int left_leg_start, left_leg_end;
        int right_leg_start, right_leg_end;
        int left_arm_start, left_arm_end;
        int right_arm_start, right_arm_end;
        int head_start, head_end;
    };


    struct MotorPair {
        int left_joint;
        int right_joint;
        std::string name;
    };

    /**
     * @brief 测试模式枚举
     */
    enum class TestMode {
        FULL_BODY = 0,  // 全身测试
        LEGS_ONLY = 1,  // 腿部测试
        ARMS_ONLY = 2   // 手臂测试
    };

    struct TestConfig {
        int robot_version;
        int num_joint;
        int num_waist_joints;
        int num_arm_joints;
        int num_head_joints;
        int na_foot;
        std::string config_file_path;
        bool is_real;
        double dt;
        int test_duration_ms;
        TestMode test_mode;  // 测试模式
    };

    /**
     * @brief 带时间戳的数据点
     */
    struct TimedValue {
        double value;
        long long timestamp;
        
        TimedValue() : value(0.0), timestamp(0LL) {}
        TimedValue(double val, long long ts) : value(val), timestamp(ts) {}
    };

    /**
     * @brief 对称性评估指标
     */
    struct SymmetryMetrics {
        double amplitude_ratio;      // 幅值比 (右/左)
        double phase_difference;     // 相位差 (弧度)
        double amplitude_symmetry;   // 幅值对称性 (0-1, 1为完全对称)
        double phase_symmetry;       // 相位对称性 (0-1, 1为完全对称)
        double overall_symmetry;     // 总体对称性 (0-1, 1为完全对称)
    };

    /**
     * @brief 测试结果
     */
    struct TestResult {
        int pair_index;
        int left_joint;
        int right_joint;
        double left_error_variance;
        double right_error_variance;
        double similarity;
        double symmetry;
        bool is_error;
        std::string status;
    };


public:
    MotorFollowTestCore();
    ~MotorFollowTestCore();

    /**
     * @brief 初始化电机跟随测试核心
     * @param config 测试配置
     * @return 初始化是否成功
     */
    bool initialize(const TestConfig& config);

    /**
     * @brief 生成电机指令
     * @param sensor_data 传感器数据
     * @param joint_cmd 输出的关节指令
     * @return 是否成功生成指令
     */
    bool generateMotorCommand(const SensorData_t& sensor_data, JointCmd_t& joint_cmd);

    /**
     * @brief 处理传感器数据
     * @param sensor_data 传感器数据
     * @return 处理后的传感器数据
     */
    SensorData_t processSensorData(const SensorData_t& sensor_data);

    /**
     * @brief 开始电机跟随测试
     * @return 是否成功开始
     */
    bool startTest();
    
    /**
     * @brief 获取当前目标位置
     * @param joint_id 关节ID
     * @return 目标位置
     */
    double getCurrentTargetPosition(int joint_id) const;

    /**
     * @brief 停止电机跟随测试
     */
    void stopTest();

    /**
     * @brief 检查测试是否完成
     * @return 测试是否完成
     */
    bool isTestComplete() const;

    /**
     * @brief 获取当前测试进度
     * @return 测试进度 (0.0 - 1.0)
     */
    double getTestProgress() const;

    /**
     * @brief 获取关节数量
     * @return 关节数量
     */
    int getNumJoints() const { return num_joint_; }

    /**
     * @brief 获取关节组索引
     * @return 关节组索引结构
     */
    const JointGroupIndices& getJointGroupIndices() const { return joint_indices_; }

    /**
     * @brief 获取电机对列表
     * @return 电机对列表
     */
    const std::vector<MotorPair>& getMotorPairs() const { return lr_leg_; }

    /**
     * @brief 获取当前测试的电机对索引
     * @return 当前电机对索引
     */
    int getCurrentMotorPairIndex() const { return index_; }

    /**
     * @brief 获取关节KP参数
     * @param joint_id 关节ID
     * @return KP参数
     */
    double getJointKp(int joint_id) const;

    /**
     * @brief 获取关节KD参数
     * @param joint_id 关节ID
     * @return KD参数
     */
    double getJointKd(int joint_id) const;

    /**
     * @brief 记录关节命令数据
     * @param joint_index 关节索引
     * @param command_value 命令值
     */
    void recordJointCommand(int joint_index, double command_value);

    /**
     * @brief 记录关节响应数据
     * @param joint_index 关节索引
     * @param response_value 响应值
     */
    void recordJointResponse(int joint_index, double response_value);

    /**
     * @brief 保存当前测试对的数据到文件
     * @param pair_index 测试对索引
     */
    void saveTestDataToFile(int pair_index);

    /**
     * @brief 获取测试结果列表
     * @return 测试结果列表
     */
    const std::vector<TestResult>& getTestResults() const { return test_results_; }

    /**
     * @brief 获取测试关节对列表
     * @return 测试关节对列表
     */
    const std::vector<MotorPair>& getTestPairs() const { return lr_leg_; }

    /**
     * @brief 获取当前测试对索引
     * @return 当前测试对索引
     */
    int getCurrentPairIndex() const { return index_; }

    /**
     * @brief 获取当前测试模式
     * @return 测试模式
     */
    TestMode getTestMode() const { return config_.test_mode; }

    /**
     * @brief 生成测试报告
     * @return 测试报告内容
     */
    std::string generateTestReport() const;

    /**
     * @brief 构建文件保存路径
     * @param filename 文件名
     * @return 完整文件路径
     */
    std::string buildFilePath(const std::string& filename) const;

    /**
     * @brief 检查关节是否为当前测试关节
     * @param joint_id 关节ID
     * @return 是否为当前测试关节
     */
    bool isCurrentTestJoint(int joint_id) const;

    /**
     * @brief 打印测试关节信息
     */
    void printTestJointInfo();

    /**
     * @brief 生成测试关节指令（正弦波轨迹）
     * @param joint_id 关节ID
     * @param sensor_data 传感器数据
     * @param joint_cmd 关节指令
     */
    void generateTestJointCommand(int joint_id, const SensorData_t& sensor_data, JointCmd_t& joint_cmd);

    /**
     * @brief 生成非测试关节指令（保持零位置）
     * @param joint_id 关节ID
     * @param sensor_data 传感器数据
     * @param joint_cmd 关节指令
     */
    void generateNonTestJointCommand(int joint_id, const SensorData_t& sensor_data, JointCmd_t& joint_cmd);

private:
    // 配置参数
    TestConfig config_;
    
    // 机器人参数
    int num_joint_;
    int num_waist_joints_;
    int num_arm_joints_;
    int num_head_joints_;
    int na_foot_;
    
    // 关节组索引
    JointGroupIndices joint_indices_;
    
    // 电机对配置
    std::vector<MotorPair> lr_leg_;
    int index_;
    
    // 控制参数
    std::vector<double> action_scale_k_;
    std::vector<double> action_scale_W_;
    Json::Value action_scale_k_config_;
    Json::Value joint_kp_config_;
    Json::Value joint_kd_config_;
    
    // 偏移关节配置
    std::vector<int> joint_offset_;  // 每个关节的偏移标志：0=不偏移，1=偏移
    Json::Value joint_offset_config_;
    
    // 传感器数据处理
    Eigen::VectorXd ankle_qv_old_;
    
    // 测试状态
    std::atomic<bool> test_running_;
    std::atomic<bool> test_complete_;
    double test_start_time_;
    double test_progress_;
    
    // 轨迹生成相关
    double time_offset_;
    int cycleCountL_, cycleCountR_;
    double prevSinL_, prevSinR_;
    int sinNum_;
    std::vector<bool> is_testing_completed_;
    std::vector<double> current_target_positions_;
    std::vector<double> current_target_velocities_;
    
    // 常量
    static constexpr double MAX_VALUE = 1e6;
    
    // 线程安全
    mutable std::mutex data_mutex_;

    // 数据记录相关
    std::vector<TimedValue> l_txBuffer_;  // 左关节命令数据
    std::vector<TimedValue> r_txBuffer_;  // 右关节命令数据
    std::vector<TimedValue> l_rxBuffer_;  // 左关节响应数据
    std::vector<TimedValue> r_rxBuffer_;  // 右关节响应数据
    std::vector<TestResult> test_results_;  // 测试结果
    std::string data_save_path_;  // 数据保存路径

private:
    /**
     * @brief 加载配置文件
     * @return 是否成功加载
     */
    bool loadConfigFile();

    /**
     * @brief 初始化关节组索引
     */
    void initializeJointGroupIndices();

    /**
     * @brief 生成电机对配置
     */
    void generateMotorPairs();
    
    /**
     * @brief 生成全身测试电机对
     */
    void generateFullBodyPairs();
    
    /**
     * @brief 生成腿部测试电机对
     */
    void generateLegPairs();
    
    /**
     * @brief 生成手臂测试电机对
     */
    void generateArmPairs();
    
    /**
     * @brief 获取测试模式名称
     * @return 测试模式名称
     */
    std::string getTestModeName() const;

    /**
     * @brief 从配置文件生成控制参数
     */
    void generateControlParameters();

    /**
     * @brief 电机到关节位置转换
     * @param sensor_data_motor 电机传感器数据
     * @param sensor_data_joint 输出的关节传感器数据
     */
    void motor2joint(const SensorData_t& sensor_data_motor, SensorData_t& sensor_data_joint);

    /**
     * @brief 前向旋转输入处理
     * @return 0: 当前关节对测试完成，继续下一个; 1: 继续当前测试; -1: 所有测试完成
     */
    int forwardRotationInput();

    /**
     * @brief 分析电机性能
     * @return 测试是否结束
     */
    bool analysisMotorPerformance();



    /**
     * @brief 检查关节是否为偏移关节
     * @param joint_id 关节ID
     * @return 是否为偏移关节
     */
    bool isOffsetJoint(int joint_id) const;

    /**
     * @brief 获取毫秒时间戳
     * @return 毫秒时间戳
     */
    long long getMillisecondTimestamp() const;


    /**
     * @brief 保存数据到文件
     * @param valueVec 数据向量
     * @param savePath 保存路径
     */
    void saveToFile(const std::vector<TimedValue>& valueVec, const std::string& savePath) const;

    /**
     * @brief 保存电机对数据到单个文件
     * @param l_tx 左输入数据
     * @param l_rx 左响应数据
     * @param r_tx 右输入数据
     * @param r_rx 右响应数据
     * @param savePath 保存路径
     */
    void saveMotorPairToFile(const std::vector<TimedValue>& l_tx, 
                            const std::vector<TimedValue>& l_rx,
                            const std::vector<TimedValue>& r_tx,
                            const std::vector<TimedValue>& r_rx,
                            const std::string& savePath) const;

    /**
     * @brief 构建文件路径
     * @param exePath 可执行文件路径
     * @param filename 文件名
     * @return 完整文件路径
     */
    std::string buildFilePath(const std::string& exePath, const std::string& filename) const;

};

} // namespace motor_follow_test

#endif // MOTOR_FOLLOW_TEST_CORE_H
