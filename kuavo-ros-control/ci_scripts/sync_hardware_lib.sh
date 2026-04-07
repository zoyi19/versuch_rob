#!/bin/bash

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
KUAVO_ROS_CONTROL_DIR="$(cd "$(dirname "$SCRIPT_DIR")" && pwd)"

rm -rf /tmp/hardware_lib || true
mkdir -p /tmp/hardware_lib || true

cp -r $KUAVO_ROS_CONTROL_DIR/src/kuavo_assets /tmp/hardware_lib/
cp -r $KUAVO_ROS_CONTROL_DIR/src/kuavo_common /tmp/hardware_lib/
cp -r $KUAVO_ROS_CONTROL_DIR/src/kuavo_solver /tmp/hardware_lib/
cp -r $KUAVO_ROS_CONTROL_DIR/src/kuavo-ros-control-lejulib/hardware_plant /tmp/hardware_lib/
cp -r $KUAVO_ROS_CONTROL_DIR/third_party_libs /tmp/hardware_lib/

cp $KUAVO_ROS_CONTROL_DIR/src/kuavo-ros-control-lejulib/hardware_plant/build_hardware_lib.txt /tmp/hardware_lib/CMakeLists.txt

# Build third_party_libs first with testing disabled
cd /tmp/hardware_lib/third_party_libs/cyclonedds-0.10.2

rm -rf build || true
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/hardware_lib/install -DBUILD_TESTING=OFF -DENABLE_TESTING=OFF .. && make -j$(nproc) install
cd ../..

# Build cyclonedds-cxx  
export CMAKE_PREFIX_PATH=/tmp/hardware_lib/install # for cyclonedds-cxx
cd /tmp/hardware_lib/third_party_libs/cyclonedds-cxx-0.10.2
rm -rf build || true
mkdir -p build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/tmp/hardware_lib/install -DBUILD_TESTING=OFF -DENABLE_TESTING=OFF .. && make -j$(nproc) install

cd /tmp/hardware_lib

mkdir build && cd build

cmake .. && make -j$(nproc) install

cd /tmp/hardware_lib

if [ -z "$kuavo_hardware_lib_repo_path" ]; then
    echo "错误: 环境变量 kuavo_hardware_lib_repo_path 未定义"
    echo "请设置目标路径，例如: export kuavo_hardware_lib_repo_path=/path/to/target"
    rm -rf /tmp/hardware_lib || true
    exit 1
fi

if [ ! -d "$kuavo_hardware_lib_repo_path" ]; then
    echo "错误: 目标路径不存在: $kuavo_hardware_lib_repo_path"
    rm -rf /tmp/hardware_lib || true
    exit 1
fi

rm -rf $kuavo_hardware_lib_repo_path/bin || true
rm -rf $kuavo_hardware_lib_repo_path/include || true
rm -rf $kuavo_hardware_lib_repo_path/lib || true
rm -rf $kuavo_hardware_lib_repo_path/share || true
rm -rf $kuavo_hardware_lib_repo_path/tests || true

cp -r install/* $kuavo_hardware_lib_repo_path
cp -r install/.gitignore $kuavo_hardware_lib_repo_path 2>/dev/null || true

if [ -f "$kuavo_hardware_lib_repo_path/tests/build_hardware_plant_test.txt" ]; then
    mv $kuavo_hardware_lib_repo_path/tests/build_hardware_plant_test.txt $kuavo_hardware_lib_repo_path/tests/CMakeLists.txt
else
    echo "警告: build_hardware_plant_test.txt 不存在，跳过移动操作"
    rm -rf /tmp/hardware_lib || true
    exit 1
fi

echo "Kuavo Hardware Lib 同步完成"