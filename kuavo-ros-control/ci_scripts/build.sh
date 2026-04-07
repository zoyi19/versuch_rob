#!/bin/bash
SCIRPT_ENTRY_DIR="$(pwd)"
echo "The script entry directory is: $SCIRPT_ENTRY_DIR"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [ ! -d "$( dirname "${SCRIPT_DIR}" )/installed" ]; then
        mkdir -p "$( dirname "${SCRIPT_DIR}" )/installed"
fi
INSTALL_DIR="$( cd "$( dirname "${SCRIPT_DIR}" )/installed" && pwd )"

# 支持通过环境变量或命令行参数覆盖 catkin build 的并行参数
# 用法: CATKIN_BUILD_ARGS="-j6 -l6" ./build.sh 或 ./build.sh -j6 -l6
CATKIN_BUILD_ARGS="${CATKIN_BUILD_ARGS:-$@}"
if [ -z "$CATKIN_BUILD_ARGS" ]; then
    CATKIN_BUILD_ARGS="-j$(nproc)"
fi
echo "Using catkin build args: $CATKIN_BUILD_ARGS"

source /opt/ros/noetic/setup.bash
export CC="ccache gcc"
export CXX="ccache g++"

cd $SCIRPT_ENTRY_DIR

# 将源码目录加入 ROS_PACKAGE_PATH，确保 rospack 能在 CMake 配置阶段找到工作空间中的包
export ROS_PACKAGE_PATH="${SCIRPT_ENTRY_DIR}/src:${ROS_PACKAGE_PATH}"

# 正常编译所有包防止依赖丢失
catkin config --no-install -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_ASM_COMPILER=/usr/bin/as
if [ $? -ne 0 ]; then
    echo "Error: catkin config failed"
    exit 1
fi

catkin build $CATKIN_BUILD_ARGS humanoid_controllers grab_box
if [ $? -ne 0 ]; then
    echo "Error: catkin build humanoid_controllers failed"
    exit 1
fi

source devel/setup.bash

# 安装模式编译需要安装的包
catkin clean -b -y
if [ $? -ne 0 ]; then
    echo "Error: catkin clean failed"
    exit 1
fi

catkin config --install --install-space installed
if [ $? -ne 0 ]; then
    echo "Error: catkin config --install failed"
    exit 1
fi

catkin build $CATKIN_BUILD_ARGS kuavo_common kuavo_solver hardware_node humanoid_interface humanoid_estimation kuavo_estimation humanoid_interface_drake humanoid_wbc --no-deps
if [ $? -ne 0 ]; then
    echo "Error: catkin build hardware_node and related packages failed"
    exit 1
fi

# 恢复安装模式
catkin config --no-install
if [ $? -ne 0 ]; then
    echo "Error: catkin config --no-install failed"
    exit 1
fi

# 清理build、devel，保留install
catkin clean -b -d -y
EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ]; then
    echo "Error: catkin clean -b -d -y failed"
    exit $EXIT_CODE
fi
