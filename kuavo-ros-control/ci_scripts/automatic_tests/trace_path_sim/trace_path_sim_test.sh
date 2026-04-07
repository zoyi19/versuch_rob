#!/bin/bash
SCRIPT_DIR=$(dirname "$(realpath "$0")")
PROJECT_DIR=$(realpath "$SCRIPT_DIR/../../../") # project: kuavo-ros-control
SDK_PROJECT_DIR="$PROJECT_DIR/src/kuavo_humanoid_sdk"  # project: kuavo_humanoid_sdk
TEST_ROBOT_VERISON=45
HUMANOID_CONTROLLERS_SCREEN_NAME="screen_humanoid_controllers"
TEST_SCREEN_NAME="screen_test"
RUN_DISPLAY=:1
RUN_PREFIX=""
RUN_LOG_DIR="$auto_test_trace_path_log_path"
DEBUG_MODE=0
# debug mode
if [ $DEBUG_MODE -eq 1 ]; then
    RUN_PREFIX=""
    RUN_LOG_DIR="/tmp/ci_trace_path"
fi
if [ -z "$RUN_LOG_DIR" ]; then
    echo "RUN_LOG_DIR is not set"
    echo "❌ miss env which is named auto_test_trace_path_log_path"
    # RUN_LOG_DIR="/tmp/ci_trace_path"
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
    cd $PROJECT_DIR
    source /opt/ros/noetic/setup.bash

    export ROBOT_VERSION=$TEST_ROBOT_VERISON
    catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release # Important! 
    catkin build humanoid_controllers kuavo_msgs gazebo_sim trace_path automatic_test ar_control
    if [ $? -ne 0 ]; then
        exit_with_failure "❌ catkin build failed"
    fi
}

exit_with_failure() {
    echo_error "❌ Error: $1"
    cleanup
    print_robot_log_info
    cd "$PROJECT_DIR"
    ./ci_scripts/wechat_bot_notify.sh "CI 仿真自动化测试轨迹跟踪失败，具体地址 $CI_PROJECT_URL/-/pipelines/$CI_PIPELINE_ID " "$WECHAT_CI_SIMULATOR_ERR_BOT_TOKEN"
    ./ci_scripts/feishu_notify.sh "CI 仿真自动化测试轨迹跟踪失败，具体地址 $CI_PROJECT_URL/-/pipelines/$CI_PIPELINE_ID " "${feishu_notify_webhook:-}"
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

    # 2. 拷贝 params_ci.yaml (强制覆盖)
    params_yaml_file="$PROJECT_DIR/src/automatic_test/automatic_test/config/params_ci.yaml"
    # 确保目标目录存在
    sudo mkdir -p "$(dirname "$params_yaml_file")"
    # 直接覆盖文件
    sudo cp "$SCRIPT_DIR/params_ci.yaml" "$params_yaml_file"
    echo_success "✅ params_ci.yaml copied successfully (overwritten)"
}

close_launch_screen() {
    tmux_sessions="$HUMANOID_CONTROLLERS_SCREEN_NAME"
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

    # 还原改动
    if [ $DEBUG_MODE -eq 0 ]; then
        # git reset --hard
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
    
    sudo pkill -f ros
    sudo chown -R $USER:$USER /var/ocs2
    sudo chmod -R 755 /var/ocs2
    echo_success "✅ Granted permissions for /var/ocs2 directory"
    echo_success "🚀 Launching humanoid controllers..."
    sudo tmux new-session -d -s $HUMANOID_CONTROLLERS_SCREEN_NAME bash -c "
        cd $PROJECT_DIR &&
        export ROBOT_VERSION=$TEST_ROBOT_VERISON &&
        export ROS_MASTER_URI=http://localhost:11311 &&
        export PATH=\"/opt/drake/bin\${PATH:+:\${PATH}}\" &&
        export PYTHONPATH=\"/opt/drake/lib/python\$(python3 -c 'import sys; print(\"{0}.{1}\".format(*sys.version_info))')/site-packages\${PYTHONPATH:+:\${PYTHONPATH}}\" &&
        export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin &&
        export PATH=\$LD_LIBRARY_PATH:\$PATH &&
        export LD_PRELOAD=\$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so &&
        export CC=\"ccache gcc\" &&
        export CXX=\"ccache g++\" &&
        export LD_LIBRARY_PATH=/opt/drake/lib/:\$LD_LIBRARY_PATH &&
        source /opt/ros/noetic/setup.bash &&
        source devel/setup.bash &&
        echo '🤖 Starting humanoid controllers with gazebo...' &&
        $RUN_PREFIX roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch trace_path_automatic_test:=true coredump:=true | tee /tmp/$HUMANOID_CONTROLLERS_SCREEN_NAME.log
    " 
    sleep 5
}

launch_test() {
    # Grant read/write permissions for /var/ocs2 directory
    if [ ! -d "/var/ocs2" ]; then
        sudo mkdir -p /var/ocs2
    fi

    # close tmux sessions
     if sudo tmux list-sessions 2>/dev/null | grep -q "$TEST_SCREEN_NAME"; then
        sudo tmux kill-session -t "$TEST_SCREEN_NAME"
        echo_success "✅ ${TEST_SCREEN_NAME#screen_} tmux session closed successfully"
    fi

    if [ -f /tmp/robot_walk_results.yaml ]; then
        sudo rm -f /tmp/robot_walk_results.yaml
    fi
    
    sudo chown -R $USER:$USER /var/ocs2
    sudo chmod -R 755 /var/ocs2
    echo_success "✅ Granted permissions for /var/ocs2 directory"
    echo_success "🚀 Launching test..."
    cd $PROJECT_DIR
    export PATH="/opt/drake/bin${PATH:+:${PATH}}"
    export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/.mujoco/mujoco210/bin
    export PATH=$LD_LIBRARY_PATH:$PATH
    export LD_PRELOAD=$LD_PRELOAD:/usr/lib/x86_64-linux-gnu/libGLEW.so
    export CC="ccache gcc"
    export CXX="ccache g++"
    export LD_LIBRARY_PATH=/opt/drake/lib/:$LD_LIBRARY_PATH    
    source /opt/ros/noetic/setup.bash
    source devel/setup.bash
    roslaunch automatic_test test_robot_walk.launch  test_cmd_vel:=true test_cmd_pose:=true monitor:=false ci_trace_path_test:=true
    sleep 5
    check_results /tmp/robot_walk_results.yaml
    if [[ $? -eq 0 ]]; then
        echo_success "✅ success "
    else
        exit_with_failure "❌ 机器人摔倒或轨迹跟踪失败"
    fi

}

check_robot_launch() {
    # Check if required screen sessions are running
    echo_info "🔍 Checking if required screen sessions are running..."
    echo_info "🔍 Current tmux sessions:"
    sudo tmux list-sessions

    for i in {1..5}; do
        humanoid_controllers_flag=0
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

        if [ $humanoid_controllers_flag -eq 1 ] ; then
            rosparam set /robot_ready true
            break
        else
            echo_error "❌ Robot failed to launch after $i attempts"
        fi

        if [ $i -eq 5 ]; then
            exit_with_failure "❌ Robot failed to launch after $i attempts"
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

check_results() {
    local file=${1:-/tmp/robot_walk_results.yaml}

    if [[ ! -f "$file" ]]; then
        echo "结果文件不存在: $file"
        return 1
    fi

    # 确保目标目录存在
    if [[ -n "$RUN_LOG_DIR" ]]; then
        mkdir -p "$RUN_LOG_DIR"
        sudo cp "$file" "$RUN_LOG_DIR/robot_walk_results.yaml"
        echo_success "✅ 结果文件已复制到: $RUN_LOG_DIR/robot_walk_results.yaml"
    else
        echo_warn "⚠️ RUN_LOG_DIR 未设置，跳过结果文件复制"
    fi

    local success=$(python3 -c "import yaml,sys;print(str(yaml.safe_load(open('$file'))['success']).lower())")

    # 展示结果
    if [[ "$success" == "true" ]]; then
        echo -e "\033[32m================== RESULT SUMMARY (SUCCESS) ==================\033[0m"
        tail -n 6 $file | sed $'s/^/\033[32m/; s/$/\033[0m/'
        echo_success "✅ 测试成功"
        return 0
    else
        echo -e "\033[31m================== RESULT SUMMARY (FAILED) ===================\033[0m"
        tail -n 6 $file | sed $'s/^/\033[31m/; s/$/\033[0m/'
        echo_error "❌ 测试失败"
        return 1
    fi
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
    echo "* 🤖 CI 自动化测试 - 仿真轨迹跟踪" 
    echo "*************************************************"
    echo_info "📂 PROJECT_DIR: $PROJECT_DIR"
    echo_info "📂 SDK_PROJECT_DIR: $SDK_PROJECT_DIR"
    echo_info "📂 SCRIPT_DIR: $SCRIPT_DIR"
    echo_info "🖥️ DISPLAY: $DISPLAY"
    install_dependencies
    source /opt/ros/noetic/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    build_humanoid_controllers
    modify_config
    launch_humanoid_controllers
    sleep 5
    check_robot_launch
    launch_test
    cleanup
    print_robot_log_info
    exit 0
}

main
