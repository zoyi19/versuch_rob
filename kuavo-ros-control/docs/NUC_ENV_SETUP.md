# kuavo-ros-control 环境配置指南（NUC / 宿主机）

> 基于 `docker/Dockerfile` 及实际编译调试过程整理  
> 目标系统: **Ubuntu 20.04 (Focal) + ROS Noetic**  
> 机器人型号: **S60 (ROBOT_VERSION=60)**  
> 整理时间: 2026-03

---

## 0. 前置条件

```bash
# 已安装 ROS Noetic (ros-noetic-desktop-full)
# 已安装 build-essential, cmake, git, python3-pip
```

---

## 1. 系统依赖 (apt)

### 1.1 基础编译依赖

```bash
sudo apt-get update -y
sudo apt-get install -y \
    build-essential cmake git wget curl \
    libeigen3-dev libboost-all-dev \
    libyaml-cpp-dev libtinyxml2-dev \
    libncurses5-dev libncursesw5-dev \
    libmodbus-dev libprotobuf-c-dev \
    liblcm-dev libgflags-dev libgoogle-glog-dev liblmdb-dev \
    liburdfdom-dev liboctomap-dev libassimp-dev \
    libgl1-mesa-dev libgl1-mesa-glx libglew-dev libosmesa6-dev \
    libglfw3-dev \
    glpk-utils libglpk-dev \
    python3-tk \
    ccache bc nano htop
```

### 1.2 ROS 包

```bash
sudo apt-get install -y \
    python3-catkin-tools \
    ros-noetic-interactive-markers \
    ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-urdf ros-noetic-kdl-parser \
    ros-noetic-robot-state-publisher \
    ros-noetic-rqt-graph ros-noetic-rqt-multiplot \
    ros-noetic-realtime-tools ros-noetic-ros-control \
    ros-noetic-xacro \
    ros-noetic-behaviortree-cpp-v3 \
    ros-noetic-joy \
    ros-noetic-apriltag-ros \
    ros-noetic-grid-map-rviz-plugin \
    ros-noetic-plotjuggler-ros
```

### 1.3 编译报错后额外安装的包（关键！）

以下三个包 Dockerfile 中**未显式安装**，但编译时需要：

```bash
sudo apt-get install -y \
    ros-noetic-costmap-2d \
    ros-noetic-octomap-msgs \
    ros-noetic-pybind11-catkin
```

| 包名 | 缺失时的报错 |
|------|-------------|
| `ros-noetic-costmap-2d` | `grid_map_costmap_2d` 找不到 `costmap_2d` |
| `ros-noetic-octomap-msgs` | 找不到 `octomap_msgs` 头文件 |
| `ros-noetic-pybind11-catkin` | `pybind11_catkin` 未找到 |

---

## 2. 第三方库（需从源码或特殊源安装）

### 2.1 Pinocchio（必须锁定版本！）

默认 apt 安装的 pinocchio 版本会导致 OCS2 MPC 节点 `free(): invalid pointer` 崩溃。  
**必须使用 ROS snapshot 源锁定 2.6.21 版本：**

```bash
# 添加 ROS snapshot 源
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

sudo sh -c 'echo "deb http://snapshots.ros.org/noetic/2024-10-31/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros-snapshots.list'

sudo apt-get update -y

# 安装锁定版本的 pinocchio 和 hpp-fcl
sudo apt-get install -y --reinstall \
    ros-noetic-pinocchio=2.6.21-1focal.20240830.092123

sudo apt-get install -y \
    ros-noetic-hpp-fcl \
    ros-noetic-hpp-fcl-dbgsym
```

> **注意**: 安装后建议 `sudo apt-mark hold ros-noetic-pinocchio` 防止被意外升级

### 2.2 Drake (v1.19.0)

```bash
sudo apt-get install -y ca-certificates gnupg lsb-release wget

wget -qO- https://drake-apt.csail.mit.edu/drake.asc \
    | gpg --dearmor - \
    | sudo tee /etc/apt/trusted.gpg.d/drake.gpg >/dev/null

echo "deb [arch=amd64] https://drake-apt.csail.mit.edu/focal focal main" \
    | sudo tee /etc/apt/sources.list.d/drake.list >/dev/null

sudo apt-get update -y
sudo apt-get install -y --no-install-recommends drake-dev=1.19.0-1
```

### 2.3 LCM (从源码编译)

```bash
sudo apt-get install -y build-essential libglib2.0-dev cmake

git clone https://github.com/lcm-proj/lcm.git
cd lcm

# 修复 Python3 查找问题
sed -i '/find_package(GLib2 REQUIRED)/a find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development)' CMakeLists.txt

mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
cd ../..

# 确保链接正确
sudo ln -sf /usr/lib/x86_64-linux-gnu/liblcm.so /usr/local/lib/liblcm.so
sudo ldconfig
```

### 2.4 CasADi (v3.5.5, 从源码编译)

```bash
sudo apt-get install -y gcc g++ gfortran cmake liblapack-dev pkg-config

git clone https://github.com/casadi/casadi.git
cd casadi
git checkout 3.5.5
mkdir build && cd build
cmake .. -DWITH_EXAMPLES=OFF
make -j$(nproc)
sudo make install
cd ../..
```

### 2.5 MuJoCo 210

```bash
mkdir -p ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz \
    -O /tmp/mujoco.tar.gz
tar -xf /tmp/mujoco.tar.gz -C ~/.mujoco
rm /tmp/mujoco.tar.gz
```

### 2.6 Python 包

```bash
pip3 install -U 'mujoco-py<2.2,>=2.1'
pip3 install mujoco psutil pynput
pip3 install scipy==1.10.1 numpy-quaternion==2023.0.4 protobuf==5.27.2
```

---

## 3. 环境变量

添加到 `~/.bashrc`：

```bash
# ROS
source /opt/ros/noetic/setup.bash

# 机器人型号
export ROBOT_VERSION=60

# Drake
export LD_LIBRARY_PATH=/opt/drake/lib/:$LD_LIBRARY_PATH
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

# MuJoCo
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin
export LD_PRELOAD=${LD_PRELOAD:+$LD_PRELOAD:}/usr/lib/x86_64-linux-gnu/libGLEW.so
```

---

## 4. OCS2 源码补丁（编译修复）

编译 ocs2_legged_robot 时报了两个源码级别的错，需要手动修补：

### 4.1 SwingTrajectoryPlanner.h — 缺少成员变量

**文件**: `src/ocs2/.../ocs2_legged_robot/include/.../foot_planner/SwingTrajectoryPlanner.h`

在 `struct Config` 中 `swingHeight = 0.1;` 后面添加一行：

```cpp
scalar_t climbStageSwingHeight = 0.1;
```

**完整上下文**:
```cpp
struct Config {
    scalar_t liftOffVelocity = 0.0;
    scalar_t touchDownVelocity = 0.0;
    scalar_t swingHeight = 0.1;
    scalar_t climbStageSwingHeight = 0.1;   // <-- 添加这行
    scalar_t swingTimeScale = 0.15;
};
```

### 4.2 LeggedRobotQuadraticTrackingCost.h — 虚函数签名不匹配

**文件**: `src/ocs2/.../ocs2_legged_robot/include/.../cost/LeggedRobotQuadraticTrackingCost.h`

`getStateInputDeviation` 函数的 override 签名缺少 `const PreComputation&` 参数。  
修改为：

```cpp
std::pair<vector_t, vector_t> getStateInputDeviation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories,
    const PreComputation& /* preComp */) const override {
```

---

## 5. 编译配置

### 5.1 跳过有已知问题的包

| 包名 | 跳过原因 |
|------|---------|
| `xsens_mti_driver` | CMake `get_filename_component` 错误；IMU 硬件驱动，仿真/NUC 不需要 |
| `ruiwo_controller_example` | 缺少 `ruiwo_controller_cxx` 私有头文件；电机驱动 demo |
| `kuavo_arm_collision_check` | 需要 FCL >= 0.6.1，系统只有 0.5.0 |
| `ocs2_raisim_core` | 需要 raisim 物理引擎（商业软件，Dockerfile 中也未安装） |
| `ocs2_anymal_commands` | 需要 `grid_map_filters_rsl`（Dockerfile 中未安装） |
| `segmented_planes_terrain_model` | 需要 `convex_plane_decomposition`（Dockerfile 中未安装） |

### 5.2 编译命令

```bash
cd /path/to/kuavo-ros-control

# 初始化 catkin workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_ASM_COMPILER=/usr/bin/as

# 设置跳过列表
catkin config --skiplist \
    xsens_mti_driver \
    ruiwo_controller_example \
    kuavo_arm_collision_check \
    ocs2_raisim_core \
    ocs2_anymal_commands \
    segmented_planes_terrain_model

# 编译
catkin build -j$(nproc) --continue-on-failure

# source
source devel/setup.bash
```

### 5.3 编译后可能仍失败的包（非关键）

以下包编译失败不影响仿真和主要功能：

| 包名 | 原因 |
|------|------|
| `ocs2_mpcnet_core` | CMake 配置错误，OCS2 MPC-Net 示例 |
| `ocs2_double_integrator_ros` | pinocchio 链接错误，OCS2 示例 |
| `ocs2_ballbot_mpcnet` | 依赖 `ocs2_mpcnet_core` |

---

## 6. 验证

```bash
# source 环境
source /opt/ros/noetic/setup.bash
source /path/to/kuavo-ros-control/devel/setup.bash

# 检查关键环境变量
echo $ROBOT_VERSION        # 应输出 60
echo $LD_LIBRARY_PATH      # 应包含 /opt/drake/lib/ 和 mujoco210/bin

# 启动 MuJoCo 仿真测试
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

---

## 7. 快速安装脚本

项目根目录已有一键脚本：

```bash
cd kuavo-ros-control
bash fix_env_and_build.sh
```

> **注意**: 该脚本不含 Drake / LCM / CasADi / MuJoCo 的安装（假设已预装），  
> 也不含 pinocchio 版本锁定。首次在全新机器上配置请按本文档第 2 节手动安装。

---

## 附录: 当前已验证的关键版本

| 组件 | 版本 |
|------|------|
| Ubuntu | 20.04.6 LTS (Focal) |
| ROS | Noetic |
| Pinocchio | 2.6.21 (ros-noetic) |
| HPP-FCL | 2.4.5 (ros-noetic) |
| Drake | 1.19.0 |
| LCM | 1.5.0 |
| CasADi | 3.5.5 |
| MuJoCo | 2.1.0 (mujoco210) |
| mujoco (pip) | 3.2.3 |
| scipy | 1.10.1 |
| protobuf | 5.27.2 |
