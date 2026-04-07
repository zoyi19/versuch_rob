#!/bin/bash
# 环境修复与编译脚本 (基于 docker/Dockerfile 依赖)
# 用法: cd kuavo-ros-control && bash fix_env_and_build.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================="
echo " Step 1: 安装缺失的系统依赖"
echo "========================================="

# 1) 安装缺失的 ROS 包
echo "[1/4] 安装缺失的 ROS 包 ..."
sudo apt-get update -y
sudo apt-get install -y \
    ros-noetic-costmap-2d \
    ros-noetic-octomap-msgs \
    ros-noetic-pybind11-catkin

# 2) Dockerfile 中安装了但宿主机可能缺失的其他包
echo "[2/4] 安装 Dockerfile 中的其他依赖 ..."
sudo apt-get install -y \
    libeigen3-dev libboost-all-dev \
    libyaml-cpp-dev libtinyxml2-dev \
    libncurses5-dev libncursesw5-dev \
    libmodbus-dev libprotobuf-c-dev \
    liblcm-dev libgflags-dev libgoogle-glog-dev liblmdb-dev \
    liburdfdom-dev liboctomap-dev libassimp-dev \
    ros-noetic-rqt-multiplot ros-noetic-grid-map-rviz-plugin \
    ros-noetic-interactive-markers ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-urdf ros-noetic-kdl-parser ros-noetic-robot-state-publisher \
    ros-noetic-rqt-graph ros-noetic-realtime-tools ros-noetic-ros-control \
    ros-noetic-xacro ros-noetic-behaviortree-cpp-v3 \
    ros-noetic-joy ros-noetic-apriltag* \
    libgl1-mesa-dev libgl1-mesa-glx libglew-dev libosmesa6-dev \
    libglfw3-dev python3-tk glpk-utils libglpk-dev \
    ccache bc nano htop

# 3) 确保 ROBOT_VERSION=60 在 bashrc 中
echo "[3/4] 检查环境变量 ..."
if ! grep -q "ROBOT_VERSION=60" ~/.bashrc; then
    # 移除旧的 ROBOT_VERSION 设置
    sed -i '/export ROBOT_VERSION=/d' ~/.bashrc
    echo 'export ROBOT_VERSION=60' >> ~/.bashrc
    echo "  -> 已设置 ROBOT_VERSION=60"
else
    echo "  -> ROBOT_VERSION=60 已存在"
fi
export ROBOT_VERSION=60

# 4) 确保 LD_LIBRARY_PATH 包含 drake 和 mujoco
echo "[4/4] 检查 LD_LIBRARY_PATH ..."
if ! grep -q "/opt/drake/lib/" ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=/opt/drake/lib/:$LD_LIBRARY_PATH' >> ~/.bashrc
fi
if ! grep -q "mujoco210/bin" ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin' >> ~/.bashrc
fi

source ~/.bashrc 2>/dev/null || true
source /opt/ros/noetic/setup.bash

echo ""
echo "========================================="
echo " Step 2: 清理旧的编译缓存并重新编译"
echo "========================================="

# 清理 build/devel/logs
echo "清理 build, devel, logs ..."
rm -rf build devel logs .catkin_tools/profiles/default/packages

# 重新初始化 catkin
catkin init 2>/dev/null || true
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_ASM_COMPILER=/usr/bin/as

echo ""
echo "========================================="
echo " Step 3: 编译 (跳过有已知问题的包)"
echo "========================================="

# 跳过的包及原因:
#   xsens_mti_driver            - CMake get_filename_component 错误 (IMU硬件驱动, 仿真不需要)
#   ruiwo_controller_example    - 缺少 ruiwo_controller_cxx 头文件 (电机驱动 demo)
#   kuavo_arm_collision_check   - 需要 FCL>=0.6.1 但系统只有 0.5.0
#   ocs2_raisim_core            - 需要 raisim 物理引擎 (Dockerfile中也未安装)
#   ocs2_anymal_commands        - 需要 grid_map_filters_rsl (Dockerfile中也未安装)
#   segmented_planes_terrain_model - 需要 convex_plane_decomposition (Dockerfile中也未安装)

catkin config --skiplist \
    xsens_mti_driver \
    ruiwo_controller_example \
    kuavo_arm_collision_check \
    ocs2_raisim_core \
    ocs2_anymal_commands \
    segmented_planes_terrain_model

catkin build \
    -j$(nproc) \
    --continue-on-failure

echo ""
echo "========================================="
echo " 编译完成!"
echo " 请执行: source devel/setup.bash"
echo "========================================="
