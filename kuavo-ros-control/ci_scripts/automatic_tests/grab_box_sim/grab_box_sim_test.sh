#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../../") # project: kuavo-ros-control
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo_humanoid_sdk"  # project: kuavo_humanoid_sdk
TEST_ROBOT_VERISON=45
HUMANOID_CONTROLLERS_SCREEN_NAME="screen_humanoid_controllers"
ROBOT_STRATEGIES_SCREEN_NAME="screen_robot_strategies"
TF_WEB_REPUBLISHER_SCREEN_NAME="screen_tf_web_republisher"
ROS_APP_TEMP_DIR=$ros_application_team_group_path
RUN_DISPLAY=:1
RUN_PREFIX=""
RUN_LOG_DIR="$auto_test_grab_box_log_path"
DEBUG_MODE=0
if [ -z "$ROS_APP_TEMP_DIR" ]; then
    echo "ROS_APP_TEMP_DIR is not set, using default value: /tmp/ci_grab_box"
    ROS_APP_TEMP_DIR="/tmp/ci_grab_box"
fi
# debug mode
if [ $DEBUG_MODE -eq 1 ]; then
    RUN_PREFIX=""
    RUN_LOG_DIR="/tmp/ci_grab_box"
fi
if [ -z "$RUN_LOG_DIR" ]; then
    echo "RUN_LOG_DIR is not set"
    echo "❌ miss env which is named auto_test_grab_box_log_path"
    # RUN_LOG_DIR="/tmp/ci_grab_box"
    exit 1
fi

# Set up trap for CTRL+C
CLEANUP_FROM_TRAP=""
trap 'CLEANUP_FROM_TRAP=1; cleanup' INT

# Define color echo functions
echo_warn() {
    echo -e "\033[33m$1\033[0m"  # Yellow text
}

echo_success() {
    echo -e "\033[32m$1\033[0m"  # Green text
}

echo_error() {
    echo -e "\033[31m$1\033[0m"  # Red text
}

echo_info() {
    echo -e "$1"  # Blue text
}

#################################################
install_dependencies() {
    echo_success "📦 Installing dependencies..."
    # Check and install ROS debug packages
    packages=(
        "tmux"
        "ros-noetic-rosbridge-server" # kuavo-ros-application
        "ros-noetic-behaviortree-cpp-v3"
    )

    # Array to store packages that need to be installed
    packages_to_install=()

    for pkg in "${packages[@]}"; do
        if ! dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
            packages_to_install+=("$pkg")
        fi
    done

    if [ ${#packages_to_install[@]} -gt 0 ]; then
        sudo apt-get install -y "${packages_to_install[@]}"
    fi
}

# 编译和安装 Kuavo Humanoid SDK
build_humanoid_controllers() {
    echo_success "🔨 Building and installing Kuavo Humanoid SDK..."
    # close screen
    close_launch_screen
    
    # build humanoid_controllers 
    cd $SDK_PROJECT_DIR
    source /opt/ros/noetic/setup.bash

    export ROBOT_VERSION=$TEST_ROBOT_VERISON
    catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
    catkin build humanoid_controllers kuavo_msgs gazebo_sim ar_control grab_box
    if [ $? -ne 0 ]; then
        exit_with_failure "❌ catkin build failed"
    fi
    # install kuavo humanoid sdk
    cd $SDK_PROJECT_DIR
    chmod +x ./install.sh
    ./install.sh
    if [ $? -ne 0 ]; then
        exit_with_failure "❌ install kuavo Humanoid SDK failed"
    fi
    echo_success "✅ Kuavo Humanoid SDK built and installed successfully"
}

build_kuavo_ros_application() {
    echo_success "🔍 Cloning kuavo-ros-application..."
    local application_repo_dir="$ROS_APP_TEMP_DIR/kuavo_ros_application/"
    if [ ! -d "$application_repo_dir/.git" ]; then
            echo_info "🔍 Removing existing directory and re-cloning kuavo-ros-application..."
            rm -rf "$application_repo_dir"
            mkdir -p "$ROS_APP_TEMP_DIR" && cd "$ROS_APP_TEMP_DIR"
            if [ $DEBUG_MODE -eq 1 ]; then
                git clone https://www.lejuhub.com/ros-application-team/kuavo_ros_application.git
            else
                git clone ssh://git@www.lejuhub.com:10026/ros-application-team/kuavo_ros_application.git
            fi
            if [ $? -ne 0 ]; then
                exit_with_failure "❌ Failed to clone kuavo-ros-application"
            fi
        else
            echo_info "🔍 kuavo-ros-application already exists"
    fi

    cd $application_repo_dir
    git config --global --add safe.directory $application_repo_dir
    git reset --hard
    git checkout dev
    git fetch
    git pull
    if [ $? -ne 0 ]; then
        exit_with_failure "❌ Failed to checkout dev"
    fi
    source /opt/ros/noetic/setup.bash
    if [ -d "build" ] || [ -d "devel" ]; then
        catkin clean -y
    fi
    catkin build kuavo_tf2_web_republisher
    if [ $? -ne 0 ]; then
        exit_with_failure "❌ Failed to build kuavo-ros-application"
    fi
    echo_success "✅ kuavo-ros-application cloned successfully"
}

exit_with_failure() {
    echo_error "❌ Error: $1"
    cleanup
    print_robot_log_info
    cd "$PROJECT_DIR"
    ./ci_scripts/wechat_bot_notify.sh "CI 仿真自动化测试搬箱子失败，具体地址 $CI_PROJECT_URL/-/pipelines/$CI_PIPELINE_ID " "$WECHAT_CI_SIMULATOR_ERR_BOT_TOKEN"
    ./ci_scripts/feishu_notify.sh "CI 仿真自动化测试搬箱子失败，具体地址 $CI_PROJECT_URL/-/pipelines/$CI_PIPELINE_ID " "${feishu_notify_webhook:-}"
    exit 1
}

modify_config() {
    echo_success "⚙️ Modifying config..."
    # 1. 修改 /opt/ros/noetic/share/apriltag_ros/config/tags.yaml Tag 信息
    tags_yaml_file="/opt/ros/noetic/share/apriltag_ros/config/tags.yaml"
    if [ -f "$tags_yaml_file" ]; then
        sudo cp "$tags_yaml_file" "$tags_yaml_file.backup"
        sudo cp "$SCRIPT_DIR/tags.yaml" "$tags_yaml_file"
        echo_success "✅ tags.yaml modified successfully"
    else
        exit_with_failure "Error: $tags_yaml_file not found"
    fi

    # 2. 修改 task.info basePitchLimits enable 改成 false
    task_info_file="$PROJECT_DIR/src/humanoid-control/humanoid_controllers/config/kuavo_v$ROBOT_VERSION/mpc/task.info"
    echo_info "Modifying $task_info_file"
    # Change basePitchLimits enable to false
    sed -i '/basePitchLimits/,/enable/ s/enable\s*true/enable                 false/' "$task_info_file"
    echo_success "✅ Config modified successfully"
    echo_info "🔍 Current git status:"
    git status
}

close_launch_screen() {
    tmux_sessions="$HUMANOID_CONTROLLERS_SCREEN_NAME $ROBOT_STRATEGIES_SCREEN_NAME $TF_WEB_REPUBLISHER_SCREEN_NAME"
    for session in $tmux_sessions; do
        if sudo tmux list-sessions 2>/dev/null | grep -q "$session"; then
            sudo tmux kill-session -t "$session"
            echo_success "✅ ${session#screen_} tmux session closed successfully"
        fi
    done
}

cleanup() {
    echo_success "🧹 Cleaning up..."

    # 关闭 screen
    close_launch_screen

    # 还原 tags.yaml
    tags_yaml_file="/opt/ros/noetic/share/apriltag_ros/config/tags.yaml.backup"
    if [ -f "$tags_yaml_file" ]; then
        sudo cp "$tags_yaml_file" "/opt/ros/noetic/share/apriltag_ros/config/tags.yaml"
        echo_success "✅ tags.yaml restored successfully"
    fi
    
    # 还原项目目录权限
    sudo chown -R $USER:$USER $PROJECT_DIR

    # 清理编译缓存
    cd $PROJECT_DIR
    if [ $DEBUG_MODE -eq 0 ]; then
        catkin clean -y  # 需要清理数据
    fi
    # 卸载 Kuavo Humanoid SDK
    if pip show kuavo-humanoid-sdk &> /dev/null; then
        echo_info "🗑️ Uninstalling kuavo-humanoid-sdk..."
        pip uninstall -y kuavo-humanoid-sdk
        if [ $? -ne 0 ]; then
            echo_error "❌ Failed to uninstall kuavo-humanoid-sdk"
        else
            echo_success "✅ Successfully uninstalled kuavo-humanoid-sdk"
        fi
    else
        echo_info "ℹ️ kuavo-humanoid-sdk is not installed"
    fi

    # 还原改动
    if [ $DEBUG_MODE -eq 0 ]; then
        git reset --hard
        git status 
    fi

    # 清除 rosnode 
    echo y | rosnode cleanup
    echo_success "✅ Cleared rosnode"

    # Exit immediately if cleanup was triggered by CTRL+C
    if [ -n "$CLEANUP_FROM_TRAP" ]; then
        echo_info "Received CTRL+C, exiting..."
        exit 1
    fi
}

launch_humanoid_controllers() {
    # Grant read/write permissions for /var/ocs2 directory
    if [ ! -d "/var/ocs2" ]; then
        sudo mkdir -p /var/ocs2
    fi

    # close tmux sessions
     if sudo tmux list-sessions 2>/dev/null | grep -q "$HUMANOID_CONTROLLERS_SCREEN_NAME"; then
        sudo tmux kill-session -t "$HUMANOID_CONTROLLERS_SCREEN_NAME"
        echo_success "✅ ${HUMANOID_CONTROLLERS_SCREEN_NAME#screen_} tmux session closed successfully"
    fi
    
    sudo chown -R $USER:$USER /var/ocs2
    sudo chmod -R 755 /var/ocs2
    echo_success "✅ Granted permissions for /var/ocs2 directory"
    echo_success "🚀 Launching humanoid controllers..."
    sudo tmux new-session -d -s $HUMANOID_CONTROLLERS_SCREEN_NAME bash -c "
        cd $PROJECT_DIR &&
        export ROBOT_VERSION=$TEST_ROBOT_VERISON &&
        export ROS_MASTER_URI=http://localhost:11311 &&
        export ROS_HOSTNAME=localhost &&
        export ROS_IP=localhost &&
        export DISPLAY=$RUN_DISPLAY &&
        source /opt/ros/noetic/setup.bash &&
        source devel/setup.bash &&
        echo '🤖 Starting humanoid controllers with gazebo...' &&
        $RUN_PREFIX roslaunch humanoid_controllers load_kuavo_gazebo_manipulate.launch grab_box_ci_test:=true coredump:=true | tee /tmp/$HUMANOID_CONTROLLERS_SCREEN_NAME.log
    " 
    sleep 5
}

launch_robot_strategies() {
    echo_success "🚀 Launching robot strategies..."
    if sudo tmux list-sessions 2>/dev/null | grep -q "$ROBOT_STRATEGIES_SCREEN_NAME"; then
        sudo tmux kill-session -t "$ROBOT_STRATEGIES_SCREEN_NAME"
        echo_success "✅ ${ROBOT_STRATEGIES_SCREEN_NAME#screen_} tmux session closed successfully"
    fi
    sudo tmux new-session -d -s $ROBOT_STRATEGIES_SCREEN_NAME bash -c "
        cd $PROJECT_DIR &&
        export ROBOT_VERSION=$TEST_ROBOT_VERISON &&
        export ROS_MASTER_URI=http://localhost:11311 &&
        export ROS_HOSTNAME=localhost &&
        export ROS_IP=localhost &&
        export DISPLAY=$RUN_DISPLAY &&
        source /opt/ros/noetic/setup.bash &&
        source devel/setup.bash &&
        echo '🤖 Starting robot strategies...' &&
        $RUN_PREFIX roslaunch ar_control robot_strategies.launch coredump:=true | tee /tmp/$ROBOT_STRATEGIES_SCREEN_NAME.log
    "
}


launch_kuavo_ros_application() {
    echo_success "🚀 Launching kuavo-ros-application..."
    if sudo tmux list-sessions 2>/dev/null | grep -q "$TF_WEB_REPUBLISHER_SCREEN_NAME"; then
        sudo tmux kill-session -t "$TF_WEB_REPUBLISHER_SCREEN_NAME"
        echo_success "✅ ${TF_WEB_REPUBLISHER_SCREEN_NAME#screen_} tmux session closed successfully"
    fi
    # Check if port 9090 is in use and kill the process if needed
    if netstat -tuln | grep -q ":9090 "; then
        echo_info "🔍 Port 9090 is in use, attempting to free it..."
        # Find and kill process using port 9090
        pid=$(lsof -ti:9090)
        if [ ! -z "$pid" ]; then
            kill -9 $pid
            echo_success "✅ Killed process $pid using port 9090"
        fi
    else
        echo_info "✅ Port 9090 is available"
    fi

    sudo tmux new-session -d -s $TF_WEB_REPUBLISHER_SCREEN_NAME bash -c "
        cd $ROS_APP_TEMP_DIR/kuavo_ros_application &&
        source devel/setup.bash &&
        echo '🤖 Starting kuavo-ros-application...' &&
        export ROS_MASTER_URI=http://localhost:11311 &&
        export ROS_HOSTNAME=localhost &&
        export ROS_IP=localhost &&
        roslaunch kuavo_tf2_web_republisher start_websocket_server.launch
    "
}

check_robot_launch() {
    # Check if required screen sessions are running
    echo_info "🔍 Checking if required screen sessions are running..."
    echo_info "🔍 Current tmux sessions:"
    sudo tmux list-sessions

    for i in {1..5}; do
        humanoid_controllers_flag=0
        robot_strategies_flag=0
        tf_web_republisher_flag=0
        echo y | rosnode cleanup
        # Check humanoid_controllers
        if rosnode list | grep -q "/gazebo" && rosnode list | grep -q "/humanoid_sqp_mpc"; then
            echo_success "✅ Found required nodes: /gazebo and /humanoid_sqp_mpc"
            humanoid_controllers_flag=1
        else
            echo_error "❌ Missing required nodes: /gazebo and /humanoid_sqp_mpc"
            echo_info "🔍 Restarting humanoid controllers..."
            launch_humanoid_controllers
            sleep 5
        fi

        # Check kuavo_tf2_web_republisher
        if rosnode list | grep -q "/kuavo_tf2_web_republisher"; then
            echo_success "✅ Found required nodes: /kuavo_tf2_web_republisher"
            tf_web_republisher_flag=1
        else
            echo_error "❌ Missing required nodes: /kuavo_tf2_web_republisher"
            echo_info "🔍 Restarting kuavo-ros-application..."
            launch_kuavo_ros_application
            sleep 5 
        fi

        # Check robot_strategies
        if rosnode list | grep -q "/ar_control_node" && rosnode list | grep -q "/tag_tracker_node"; then
            echo_success "✅ Found required nodes: /ar_control_node and /tag_tracker_node"
            robot_strategies_flag=1
        else
            echo_error "❌ Missing required nodes: /ar_control_node and /tag_tracker_node"
            echo_info "🔍 Restarting robot strategies..."
            launch_robot_strategies
            sleep 5
        fi
        if [ $humanoid_controllers_flag -eq 1 ] && [ $robot_strategies_flag -eq 1 ] && [ $tf_web_republisher_flag -eq 1 ]; then
            break
        else
            echo_error "❌ Robot failed to launch after $i attempts"
        fi
        sleep 1
    done
    echo_success "✅ All required screen sessions are running"
    # Check if nvidia-smi exists and execute if it does
    if command -v nvidia-smi &> /dev/null; then
        echo_info "🔍 GPU information:"
        nvidia-smi
    fi
}

run_grab_box_sim_test() {
    echo_info "🔍 Checking if robot is launched..."
    rosnode list
    echo "---------------------------------------------------------------------------------------"
    ps -ef|grep "roslaunch"
    echo "---------------------------------------------------------------------------------------"
    flag=0
    for i in {1..5}; do
        source $PROJECT_DIR/devel/setup.bash
        python3 $SCRIPT_DIR/check_robot_launch.py
        if [ $? -eq 0 ]; then
            echo_success "✅ Robot launched successfully"
            flag=1
            break
        else    
            echo_error "❌ Robot failed to launch after $i attempts"
        fi
        sleep 1
    done

    if [ $flag -eq 0 ]; then
        exit_with_failure "❌ Robot failed to launch after 10 attempts"
    fi

    echo_success "🚀 Running grab box sim test..."
    for i in {1..10}; do
        echo_info "🔄 执行第 $i 次测试..."
        python3 $SCRIPT_DIR/grab_box_test.py
        if [ $? -ne 0 ]; then
            echo_error "***********************************************"
            echo_error "* ❌ 第 $i 次测试结果: 失败 💔💔💔"
            echo_error "***********************************************"
            exit_with_failure "❌ 第 $i 次测试结果: 失败 💔💔💔"
        else
            echo_success "***********************************************" 
            echo_success "* ✅ 第 $i 次测试结果: 成功 🎉🎉🎉"
            echo_success "***********************************************"
        fi
    done
}

print_robot_log_info() {
    echo_info "🔍 Printing robot log info..."
    # Get the latest two stdout directories
    latest_dirs=($(sudo ls -t /root/.ros/stdout/ | head -2))
    if [ ${#latest_dirs[@]} -eq 0 ]; then
        echo_error "❌ No stdout directories found"
        return 1
    fi

    # Copy latest log
    mkdir -p $RUN_LOG_DIR
    if sudo test -f "/root/.ros/stdout/${latest_dirs[0]}/stdout.log"; then
        sudo cp "/root/.ros/stdout/${latest_dirs[0]}/stdout.log" "$RUN_LOG_DIR/latest_stdout.log"
        echo_warn "📄 Latest stdout.log copied to: $RUN_LOG_DIR/latest_stdout.log"
    else
        echo_error "❌ stdout.log not found in /root/.ros/stdout/${latest_dirs[0]}"
    fi

    # Copy previous log if it exists
    if [ ${#latest_dirs[@]} -gt 1 ] && sudo test -f "/root/.ros/stdout/${latest_dirs[1]}/stdout.log"; then
        sudo cp "/root/.ros/stdout/${latest_dirs[1]}/stdout.log" "$RUN_LOG_DIR/previous_stdout.log"
        echo_warn "📄 Previous stdout.log copied to: $RUN_LOG_DIR/previous_stdout.log" 
    fi
}

# Main function
main() {
    echo "*************************************************"
    echo "* 🤖 CI 自动化测试 - 仿真搬箱子" 
    echo "*************************************************"
    echo_info "📂 PROJECT_DIR: $PROJECT_DIR"
    echo_info "📂 SDK_PROJECT_DIR: $SDK_PROJECT_DIR"
    echo_info "📂 SCRIPT_DIR: $SCRIPT_DIR"
    echo_info "🖥️ DISPLAY: $DISPLAY"
    install_dependencies
    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
    export ROS_IP=localhost
    build_kuavo_ros_application
    build_humanoid_controllers
    modify_config
    launch_humanoid_controllers
    launch_robot_strategies
    launch_kuavo_ros_application
    sleep 5
    check_robot_launch
    echo_success "✅ Humanoid controllers launched successfully"
    run_grab_box_sim_test
    cleanup
    print_robot_log_info
    exit 0
}

main
