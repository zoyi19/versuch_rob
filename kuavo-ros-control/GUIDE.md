# NUC 迁移指南：wheel 项目完整部署流程

> **适用场景**：将 wheel 项目从开发机迁移到机器人 NUC 上重新适配  
> **前提**：仓库已克隆到 NUC，需要重新配置环境并编译  
> **目标系统**：Ubuntu 20.04 (Focal) + ROS Noetic  
> **机器人型号**：S60 (`ROBOT_VERSION=60`)

---

## 目录

1. [迁移总览](#1-迁移总览)
2. [第一步：环境依赖安装](#2-第一步环境依赖安装)
3. [第二步：第三方库安装](#3-第二步第三方库安装)
4. [第三步：环境变量配置](#4-第三步环境变量配置)
5. [第四步：OCS2 源码补丁](#5-第四步ocs2-源码补丁)
6. [第五步：编译工程](#6-第五步编译工程)
7. [第六步：需要修改的路径清单](#7-第六步需要修改的路径清单)
8. [第七步：网络与 ROS 通信配置](#8-第七步网络与-ros-通信配置)
9. [第八步：设备路径适配](#9-第八步设备路径适配)
10. [第九步：UMI 遥操作部署](#10-第九步umi-遥操作部署)
11. [验证流程](#11-验证流程)
12. [常见问题](#12-常见问题)
13. [回滚/恢复说明](#13-回滚恢复说明)

---

## 1. 迁移总览

### 项目结构

```
wheel/                              # 仓库根目录
├── kuavo-ros-control/              # 主 catkin 工作区（此目录本身就是 catkin root）
│   ├── src/                        # 所有 ROS 包
│   │   ├── humanoid-control/       # 人形控制（MPC/WBC/接口）
│   │   ├── humanoid-wheel-control/ # 轮臂控制
│   │   ├── mujoco/                 # MuJoCo 仿真节点
│   │   ├── manipulation_nodes/     # IK、VR、websocket、示教
│   │   ├── demo/umi_replay/        # UMI 遥操作/回放 demo
│   │   ├── kuavo_assets/           # 模型、URDF、配置
│   │   ├── kuavo_wheel/            # 轮臂底盘桥接
│   │   └── ...
│   ├── tools/                      # 工厂测试、部署脚本
│   ├── docs/                       # 文档（含 NUC_ENV_SETUP.md）
│   ├── fix_env_and_build.sh        # 一键环境修复+编译脚本
│   └── third_party_libs/           # 第三方库源码
├── gripper_calibration/            # 夹爪标定工具
├── kuavo_ik/                       # 独立逆运动学 Python 包
└── NUC_MIGRATION_GUIDE.md          # 本文档
```

### 迁移步骤概览

```
克隆完成 → 安装系统依赖 → 安装第三方库 → 配置环境变量
         → 打 OCS2 补丁 → 清理并编译 → 修改硬编码路径
         → 配置网络 → 适配设备 → 验证
```

---

## 2. 第一步：环境依赖安装

> 假设 NUC 已安装 `ros-noetic-desktop-full`、`build-essential`、`cmake`、`git`、`python3-pip`。

### 2.1 基础编译依赖

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

### 2.2 ROS 包

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
    ros-noetic-plotjuggler-ros \
    ros-noetic-costmap-2d \
    ros-noetic-octomap-msgs \
    ros-noetic-pybind11-catkin
```

> **注意**：最后三个包（`costmap-2d`、`octomap-msgs`、`pybind11-catkin`）在 Dockerfile 中未显式安装，但编译时必须有。

---

## 3. 第二步：第三方库安装

### 3.1 Pinocchio（必须锁定版本 2.6.21！）

> **关键**：默认 apt 版本会导致 OCS2 MPC 节点 `free(): invalid pointer` 崩溃。

```bash
# 添加 ROS snapshot 源
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA

sudo sh -c 'echo "deb http://snapshots.ros.org/noetic/2024-10-31/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros-snapshots.list'

sudo apt-get update -y

# 安装锁定版本
sudo apt-get install -y --reinstall \
    ros-noetic-pinocchio=2.6.21-1focal.20240830.092123

sudo apt-get install -y ros-noetic-hpp-fcl ros-noetic-hpp-fcl-dbgsym

# 防止被意外升级
sudo apt-mark hold ros-noetic-pinocchio
```

### 3.2 Drake (v1.19.0)

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

### 3.3 LCM (源码编译)

```bash
git clone https://github.com/lcm-proj/lcm.git /tmp/lcm
cd /tmp/lcm

# 修复 Python3 查找
sed -i '/find_package(GLib2 REQUIRED)/a find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter Development)' CMakeLists.txt

mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

sudo ln -sf /usr/lib/x86_64-linux-gnu/liblcm.so /usr/local/lib/liblcm.so
sudo ldconfig
```

### 3.4 CasADi (v3.5.5, 源码编译)

```bash
sudo apt-get install -y gcc g++ gfortran cmake liblapack-dev pkg-config

git clone https://github.com/casadi/casadi.git /tmp/casadi
cd /tmp/casadi
git checkout 3.5.5
mkdir build && cd build
cmake .. -DWITH_EXAMPLES=OFF
make -j$(nproc)
sudo make install
```

### 3.5 MuJoCo 210

```bash
mkdir -p ~/.mujoco
wget https://github.com/google-deepmind/mujoco/releases/download/2.1.0/mujoco210-linux-x86_64.tar.gz \
    -O /tmp/mujoco.tar.gz
tar -xf /tmp/mujoco.tar.gz -C ~/.mujoco
rm /tmp/mujoco.tar.gz
```

### 3.6 Python 包

```bash
pip3 install -U 'mujoco-py<2.2,>=2.1'
pip3 install mujoco psutil pynput
pip3 install scipy==1.10.1 numpy-quaternion==2023.0.4 protobuf==5.27.2
```

---

## 4. 第三步：环境变量配置

将以下内容追加到 `~/.bashrc`：

```bash
# ---- ROS ----
source /opt/ros/noetic/setup.bash

# ---- 机器人型号 ----
export ROBOT_VERSION=60

# ---- Drake ----
export LD_LIBRARY_PATH=/opt/drake/lib/:$LD_LIBRARY_PATH
export PATH="/opt/drake/bin${PATH:+:${PATH}}"
export PYTHONPATH="/opt/drake/lib/python$(python3 -c 'import sys; print("{0}.{1}".format(*sys.version_info))')/site-packages${PYTHONPATH:+:${PYTHONPATH}}"

# ---- MuJoCo ----
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mujoco210/bin
export LD_PRELOAD=${LD_PRELOAD:+$LD_PRELOAD:}/usr/lib/x86_64-linux-gnu/libGLEW.so

# ---- catkin 工作区（编译后取消注释） ----
# source <你的kuavo-ros-control路径>/devel/setup.bash
```

然后执行 `source ~/.bashrc`。

---

## 5. 第四步：OCS2 源码补丁

编译前需要打两个补丁，否则编译 `ocs2_legged_robot` 会报错。

### 5.1 SwingTrajectoryPlanner.h — 缺少成员变量

**文件**：`kuavo-ros-control/src/ocs2/.../ocs2_legged_robot/include/.../foot_planner/SwingTrajectoryPlanner.h`

在 `struct Config` 中 `swingHeight = 0.1;` **后面**添加一行：

```cpp
scalar_t climbStageSwingHeight = 0.1;
```

### 5.2 LeggedRobotQuadraticTrackingCost.h — 虚函数签名不匹配

**文件**：`kuavo-ros-control/src/ocs2/.../ocs2_legged_robot/include/.../cost/LeggedRobotQuadraticTrackingCost.h`

`getStateInputDeviation` 函数签名改为：

```cpp
std::pair<vector_t, vector_t> getStateInputDeviation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories,
    const PreComputation& /* preComp */) const override {
```

---

## 6. 第五步：编译工程

### 方式一：使用一键脚本（推荐）

```bash
cd <你的kuavo-ros-control路径>
bash fix_env_and_build.sh
source devel/setup.bash
```

> **注意**：该脚本会自动清理 `build/devel/logs` 并重新编译，但**不含**第三方库安装和 Pinocchio 锁定。请确保已完成第 2~4 步。

### 方式二：手动编译

```bash
cd <你的kuavo-ros-control路径>

# 清理旧编译产物（必须！旧的 devel 含开发机的绝对路径）
rm -rf build devel logs .catkin_tools/profiles/default/packages

catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release -DCMAKE_ASM_COMPILER=/usr/bin/as

# 与 fix_env_and_build.sh 保持一致（UMI/MPC 轮臂仿真不依赖下列包）
catkin config --skiplist \
    xsens_mti_driver \
    ruiwo_controller_example \
    kuavo_arm_collision_check \
    ocs2_raisim_core \
    ocs2_anymal_commands \
    segmented_planes_terrain_model \
    ocs2_raisim_ros \
    ocs2_quadruped_interface \
    ocs2_mpcnet_core \
    hardware_node

# 可选：若仍出现 ocs2 四足/ballbot/raisim 示例编译失败，可追加跳过（与 UMI 遥操作无关）：
#   ocs2_quadruped_loopshaping_interface ocs2_anymal_mpc ocs2_ballbot_mpcnet ocs2_legged_robot_raisim

catkin build -j$(nproc) --continue-on-failure

source devel/setup.bash
```

### 跳过包说明

| 包名 | 跳过原因 |
|------|---------|
| `xsens_mti_driver` | IMU 驱动，仿真不需要 |
| `ruiwo_controller_example` | 缺私有头文件 |
| `kuavo_arm_collision_check` | 需要 FCL >= 0.6.1 |
| `ocs2_raisim_core` | 需要 raisim 商业引擎 |
| `ocs2_anymal_commands` | 缺 `grid_map_filters_rsl` |
| `segmented_planes_terrain_model` | 缺 `convex_plane_decomposition` |
| `ocs2_raisim_ros` | 依赖已跳过的 `ocs2_raisim_core` |
| `ocs2_quadruped_interface` | 依赖已跳过的 `ocs2_anymal_commands` |
| `ocs2_mpcnet_core` | 需 onnxruntime（未装则跳过） |
| `hardware_node` | 需 Xsens xspublic 完整 SDK；**MuJoCo 轮臂仿真 + UMI 遥操作可不编** |

> **apt**：若 `apt-get update` 因 Intel OpenVINO 源报错，可临时禁用 `/etc/apt/sources.list.d/intel-openvino-*.list` 后再装依赖（见 FAQ/现场记录）。

---

## 7. 第六步：需要修改的路径清单

> **核心原则**：开发机用户 `leju`，NUC 用户通常为 `lab`。所有硬编码的绝对路径需改为 NUC 实际路径。

### 7.1 必须修改（影响运行）

| 类别 | 文件 | 需修改内容 | 改为 |
|------|------|-----------|------|
| **catkin 产物** | `kuavo-ros-control/devel/` 整个目录 | 含开发机绝对路径 | **重新编译后自动生成，无需手改** |
| **夹爪标定** | `gripper_calibration/README.md` | `sys.path.append('/home/leju/catkin_ws/src/wheel/gripper_calibration')` | 改为 NUC 上的实际路径 |
| **systemd 服务** | `src/kuavo_wheel/service/kuavo_wheel_bridge.service` | `KUAVO_ROS_CONTROL_WS_PATH=/home/lab/tn/kuavo-ros-control` | 改为 NUC 上的 `kuavo-ros-control` 路径 |
| **音频路径** | `humanoid_controllers/launch/load_kuavo_real*.launch` | `audio_path` 默认 `/home/lab/.config/lejuconfig/music` | 改为 NUC 实际音频目录，或创建对应目录 |
| **音频 launch** | `ros_audio/kuavo_audio_player/launch/play_music.launch` | `music_path` 默认 `/home/lab/.config/lejuconfig/music` | 同上 |

### 7.2 按需修改（仅影响特定功能）

| 类别 | 文件 | 内容 |
|------|------|------|
| **UMI demo 注释** | `src/demo/umi_replay/scripts/replay_umi_in_mujoco.py` | 注释中 `cd /home/leju/...`，仅影响可读性 |
| **示教面板** | `manipulation_nodes/teach_pendant/.../launch_teach_pendant_record_*.launch` | 默认保存路径 `/home/user/...` |
| **RL 部署** | `motion_capture_ik/scripts/quest3_fsm_node_for_rl.py` | `KUAVO_RL_WS_PATH` 默认 `/home/lab/kuavo-RL/...` |
| **websocket** | `planarmwebsocketservice/scripts/handler.py` | 多处 `/home/lab/...`、`/home/kuavo/...` |
| **IK 包示例** | `kuavo_ik/kuavo_ik/example/read_datasets_and_save_traj.py` | `urdf_file='/home/oem/...'` |
| **远程配置** | `humanoid_controllers/scripts/remote-config.json` | `/home/kuavo/...` 路径 |
| **工厂测试** | `tools/factory_test/factory_test.sh` | 下位机 `/home/lab/kuavo-ros-opensource/` |
| **部署脚本** | `tools/setup-kuavo-ros-control.sh` | 多处 `/home/lab/...` |

### 7.3 无需修改（注释/文档中，不影响运行）

- URDF 中的 `/home/fandes/...`（xacro 自动生成的 XML 注释，不影响加载）
- CI 文件 `.gitlab-ci.yml` 中的路径
- `tools/check_tool/` 下的检测工具（产线使用）

### 7.4 快速查找所有硬编码路径

```bash
# 查找含开发机用户名的文件
grep -rn "/home/leju" kuavo-ros-control/src/ --include="*.py" --include="*.sh" --include="*.launch" --include="*.yaml"

# 查找含 /home/lab 的文件
grep -rn "/home/lab" kuavo-ros-control/src/ --include="*.py" --include="*.sh" --include="*.launch" --include="*.yaml" --include="*.service"
```

---

## 8. 第七步：网络与 ROS 通信配置

### 8.1 NUC 网络拓扑（典型）

```
NUC (上位机)  ←─ 192.168.26.12 ─→  下位机 (192.168.26.1)
      │
      └── hostname: kuavo_master（通过 /etc/hosts 配置）
```

### 8.2 `/etc/hosts` 配置

在 NUC 的 `/etc/hosts` 中添加：

```
127.0.0.1       kuavo_master
192.168.26.1    kuavo_slave    # 下位机（如有）
```

### 8.3 ROS 环境变量

在 `~/.bashrc` 中添加：

```bash
export ROS_MASTER_URI=http://kuavo_master:11311
export ROS_IP=<NUC的实际IP>    # 如 192.168.26.12
```

### 8.4 需确认的网络相关文件

| 文件 | 默认值 | 说明 |
|------|-------|------|
| `kuavo_wheel/service/kuavo_wheel_bridge.service` | `ROS_MASTER_URI=http://kuavo_master:11311` | systemd 服务中的 ROS 配置 |
| `tools/factory_test/factory_test.sh` | `UPPER_IP`、`ROS_MASTER_URI` 多处 | 工厂测试脚本 |
| `planarmwebsocketservice/service/websocket_start.service` | `127.0.0.1:11311` | websocket 服务 |
| `kuavo_wheel/scripts/setup_fixed_ips.sh` | `192.168.26.*` 网段 | 固定 IP 配置 |

---

## 9. 第八步：设备路径适配

NUC 上的外设编号可能与开发机不同。

### 9.1 需检查的设备

| 设备 | 相关文件 | 默认路径 |
|------|---------|---------|
| USB 摄像头 | `noitom_hi5_hand_udp_python/launch/usb_cam_node.launch` | `/dev/video4` |
| USB 摄像头 | `launch_quest3_ik_videostream_usb_cam.launch` | `/dev/video0` |
| 串口设备 | `nlink_parser/launch/*.launch` | `/dev/ttyUSB0` |
| 灵巧手串口 | `hand_sdk/.../gen_dexhand_serial_rules.sh` | `/dev/ttyUSB*` |

### 9.2 建议：使用 udev 规则固定设备名

```bash
# 查看已连接的 USB 设备信息
udevadm info -a /dev/ttyUSB0 | grep -E 'ATTRS{idVendor|idProduct|serial}'

# 创建 udev 规则示例（/etc/udev/rules.d/99-robot-devices.rules）
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="umi_serial"
```

---

## 10. 第九步：UMI 遥操作部署

UMI 相关脚本位于 `kuavo-ros-control/src/demo/umi_replay/scripts/`。

### 10.1 文件说明

| 脚本 | 用途 |
|------|------|
| `umi_realtime_teleop.py` | **实时遥操作**：订阅动捕位姿 → 增量+FHAN 平滑 → MPC 控制左臂 |
| `transform_umi_to_base.py` | **离线处理**：从 rosbag 提取轨迹并转换到 base_link 坐标系 |
| `replay_umi_in_mujoco.py` | **轨迹回放**：在 MuJoCo 仿真中回放 pkl 轨迹 |

### 10.2 依赖

```bash
pip3 install numpy scipy matplotlib
# 若使用夹爪 ArUco 检测
pip3 install opencv-contrib-python
```

### 10.3 实时遥操作流程

```bash
# 终端 1：启动仿真
source kuavo-ros-control/devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch

# 终端 2（可选）：启动夹爪检测
cd <wheel路径>/gripper_calibration
python3 gripper_percent_node.py --calibration gripper_range.json

# 终端 3：启动动捕推流（外部，需发布 /umi_odom 和 /orbbec_odom）

# 终端 4：启动遥操作
cd kuavo-ros-control/src/demo/umi_replay/scripts
python3 umi_realtime_teleop.py [--delta-scale 1.0] [--rate 100] [--no-gripper]
```

### 10.4 离线回放流程

```bash
# 步骤 1：转换 rosbag 轨迹到 base_link
python3 transform_umi_to_base.py \
    --umi-bag ../data/umi_bags/xxx.bag \
    --robot-bag ../data/robot_bags/xxx.bag \
    -o ../data/output/umi_in_base.pkl \
    --plot

# 步骤 2：在仿真中回放
python3 replay_umi_in_mujoco.py \
    --pkl ../data/output/umi_in_base.pkl \
    [--incremental] [--speed 1.0]
```

### 10.5 关键参数说明

| 参数 | 说明 | 默认值 |
|------|------|-------|
| `--head-frame` | 机器人头部 TF frame 名 | `zhead_2_link` |
| `--delta-scale` | 位置增量缩放 | `1.0` |
| `--rate` | 控制频率 (Hz) | `100` |
| `--fhan-r` | FHAN 平滑跟踪速度因子 | `10.0` |
| `--max-delta` | 单帧最大位移 (m) | `0.002` |
| `--no-gripper` | 禁用夹爪控制 | `False` |
| `--no-orient` | 禁用姿态跟踪 | `False` |

> 若 NUC 上机器人 URDF 的 TF link 名称不同（如末端执行器不是 `zarm_l7_link` / `zarm_r7_link`），需在脚本中修改对应常量。

---

## 11. 验证流程

### 11.1 环境验证

```bash
# 检查关键环境变量
echo $ROBOT_VERSION        # 应输出 60
echo $LD_LIBRARY_PATH      # 应包含 /opt/drake/lib/ 和 mujoco210/bin
python3 -c "import drake"  # 无报错
python3 -c "import mujoco" # 无报错
rosversion -d              # 应输出 noetic
```

### 11.2 编译验证

```bash
source kuavo-ros-control/devel/setup.bash
rospack find humanoid_controllers       # 应返回包路径
rospack find humanoid_wheel_interface   # 应返回包路径
```

### 11.3 仿真验证

```bash
source kuavo-ros-control/devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

正常情况下 MuJoCo 窗口弹出，机器人模型加载，MPC 节点运行。

### 11.4 UMI 遥操作验证

```bash
# 在仿真启动后，新终端：
cd kuavo-ros-control/src/demo/umi_replay/scripts
python3 umi_realtime_teleop.py --no-gripper
# 按 Enter 进入 TRACKING 模式，按 q 退出
```

---

## 12. 常见问题

### Q1: `free(): invalid pointer` / MPC 节点崩溃

**原因**：Pinocchio 版本不对。  
**解决**：按第 3.1 节锁定 Pinocchio 2.6.21，然后重新编译。

### Q2: 编译报 `costmap_2d` / `octomap_msgs` / `pybind11_catkin` 找不到

**解决**：`sudo apt-get install -y ros-noetic-costmap-2d ros-noetic-octomap-msgs ros-noetic-pybind11-catkin`

### Q3: devel/setup.bash 中路径仍指向旧机器

**原因**：`devel/` 目录是编译产物，含绝对路径。  
**解决**：`rm -rf build devel logs` 后重新编译。

### Q4: `catkin build` 部分包失败

非关键包失败（如 `ocs2_mpcnet_core`、`ocs2_double_integrator_ros`）不影响主要功能。使用 `--continue-on-failure` 可跳过。

### Q5: MuJoCo 窗口无法显示 / GLX 错误

```bash
# 确认 LD_PRELOAD
echo $LD_PRELOAD  # 应包含 libGLEW.so

# 若通过 SSH 连接，需要 X11 forwarding 或 VNC
ssh -X user@nuc_ip
```

### Q6: `/dev/ttyUSB0` 权限不足

```bash
sudo usermod -aG dialout $USER
# 然后重新登录
```

---

## 13. 回滚/恢复说明

- 如果需要恢复 `kuavo-ros-control` 与 GitLab 的链接，将备份的 `.git` 目录还原即可：
  ```bash
  # .git 备份位置（在开发机上）
  /home/leju/catkin_ws/src/kuavo-ros-control.git.backup.lejuhub
  ```
- NUC 上的仓库是独立副本，对其的任何修改不会影响 GitLab 上的源仓库。

---

## 附录：关键版本对照表

| 组件 | 版本 |
|------|------|
| Ubuntu | 20.04 LTS (Focal) |
| ROS | Noetic |
| Pinocchio | 2.6.21 (ros-noetic, snapshot 源) |
| HPP-FCL | 2.4.5 (ros-noetic) |
| Drake | 1.19.0 |
| LCM | 1.5.0 |
| CasADi | 3.5.5 |
| MuJoCo | 2.1.0 (mujoco210) |
| scipy | 1.10.1 |
| protobuf | 5.27.2 |
| numpy-quaternion | 2023.0.4 |

---

## 附录 B：UMI Teleoperation 完整操作手册

> 本节详细描述在 NUC 上从零开始运行 UMI 遥操作的**每一步操作**，包括仿真验证和真机部署。

### B.0 系统架构与数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                     动捕系统 (OptiTrack 等)                      │
│   跟踪两个刚体: UMI 夹爪 + Orbbec 相机 (固定在机器人头部)        │
└──────────┬──────────────────────────────┬───────────────────────┘
           │ /umi_odom                    │ /orbbec_odom
           │ (nav_msgs/Odometry)          │ (nav_msgs/Odometry)
           ▼                              ▼
┌──────────────────────────────────────────────────────────────────┐
│                   umi_realtime_teleop.py                         │
│                                                                  │
│  1. 从 TF 读 base_link → zhead_2_link (T_base_head)            │
│  2. 用 /orbbec_odom 首帧计算 T_mocap_orbbec                     │
│  3. 桥接: T_base_mocap = T_base_head × inv(T_mocap_orbbec)      │
│  4. 逐帧: T_base_umi = T_base_mocap × T_mocap_umi              │
│  5. 算增量 delta → FHAN 平滑 → 叠加到左臂锚点                    │
│  6. 发布 /mm/two_arm_hand_pose_cmd (frame=2, base_link)         │
└──────────────┬──────────────────────────────────────────────────┘
               │                              ┌───────────────────┐
               ▼                              │ gripper_percent   │
┌─────────────────────────────┐               │ _node.py          │
│  MPC (humanoid_controllers) │               │                   │
│  接收末端位姿指令            │  ┌───────────│ D405 图像 → ArUco │
│  求解全身轨迹               │  │            │ → /gripper_percent│
│  驱动 MuJoCo / 真机         │  │            └───────────────────┘
└─────────────────────────────┘  │                    │
                                 │                    ▼
                                 │         /leju_claw_command
                                 │         (夹爪开合控制)
```

**关键坐标系关系：**

| 符号 | 含义 | 来源 |
|------|------|------|
| `T_base_head` | base_link → zhead_2_link | 机器人 TF 树（仿真/真机自动发布） |
| `T_mocap_orbbec` | mocap_frame → orbbec 刚体 | 动捕系统，/orbbec_odom 首帧 |
| `T_mocap_umi` | mocap_frame → UMI 夹爪刚体 | 动捕系统，/umi_odom 逐帧 |
| `T_base_mocap` | 桥接变换 | 计算得到 = T_base_head × inv(T_mocap_orbbec) |

---

### B.1 前置条件检查

在开始之前，确保以下条件已满足：

```bash
# 1. 编译完成且能 source
source <你的kuavo-ros-control路径>/devel/setup.bash

# 2. 关键环境变量
echo $ROBOT_VERSION        # 应输出 60
echo $LD_LIBRARY_PATH      # 应包含 /opt/drake/lib/ 和 mujoco210/bin

# 3. kuavo_msgs 消息包可用
rosmsg show kuavo_msgs/twoArmHandPoseCmd    # 应显示消息定义
rossrv show kuavo_msgs/changeTorsoCtrlMode  # 应显示服务定义

# 4. Python 依赖
python3 -c "import numpy, scipy; print('OK')"
# 若使用夹爪检测：
python3 -c "import cv2; print(cv2.__version__)"
```

---

### B.2 硬件准备

#### 动捕系统

| 项目 | 要求 |
|------|------|
| 动捕软件 | OptiTrack Motive 或同类，能通过 ROS 发布 Odometry |
| 刚体 1 | **UMI 夹爪** — 标记为动捕刚体，发布到 `/umi_odom` (nav_msgs/Odometry) |
| 刚体 2 | **Orbbec 相机** — 固定在机器人头部 (zhead_2_link)，发布到 `/orbbec_odom` (nav_msgs/Odometry) |
| 坐标系 | 动捕系统内部统一 mocap_frame，两个刚体在同一坐标系下 |

> **重要**：Orbbec 相机必须物理固定在机器人头部，且动捕标记不能移动。

#### D405 相机（可选，用于夹爪开度检测）

| 项目 | 要求 |
|------|------|
| 型号 | Intel RealSense D405 |
| 分辨率 | 848×480 RGB |
| Topic | `/camera/color/image_raw` (sensor_msgs/Image) |
| ArUco 标签 | DICT_4X4_50, Tag 0 在左手指, Tag 1 在右手指, 16mm×16mm |

---

### B.3 第一步：启动 MuJoCo 仿真

打开 **终端 1**：

```bash
source <你的kuavo-ros-control路径>/devel/setup.bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
```

**预期结果**：
- MuJoCo 窗口弹出，显示轮式人形机器人模型
- 终端输出 MPC 相关日志，无报错
- TF 树开始发布（可用 `rostopic echo /tf` 验证）

**验证 TF 可用**（在另一个终端）：

```bash
source <你的kuavo-ros-control路径>/devel/setup.bash
rosrun tf tf_echo base_link zarm_l7_link
# 应能看到实时变换输出
rosrun tf tf_echo base_link zhead_2_link
# 应能看到实时变换输出
```

---

### B.4 第二步：启动动捕推流

打开 **终端 2**（或在动捕主机上）：

启动你的动捕 ROS 驱动，确保以下两个 topic 被发布：

```bash
# 验证 topic 存在
rostopic list | grep -E "umi_odom|orbbec_odom"
# 应看到：
#   /umi_odom
#   /orbbec_odom

# 验证数据在流动
rostopic hz /umi_odom        # 应有稳定频率（如 120Hz）
rostopic hz /orbbec_odom     # 应有稳定频率

# 查看一帧数据
rostopic echo /umi_odom -n 1
# 应看到 pose.pose.position 和 pose.pose.orientation 有合理数值
```

> **如果动捕主机与 NUC 不在同一台机器**，需要配置 ROS 多机通信：
> - NUC 上运行 roscore
> - 动捕主机设置 `ROS_MASTER_URI` 指向 NUC
> - 两台机器的 `ROS_IP` 设为各自实际 IP
> - 确保网络互通（`ping` 测试）

---

### B.5 第三步（可选）：启动夹爪开度检测

如果需要遥操作时同步控制夹爪，打开 **终端 3**：

#### B.5.1 首次使用：标定夹爪

> 只需做一次。已有 `gripper_range.json` 可跳过。

```bash
# 1. 启动 D405 相机（确保 /camera/color/image_raw 有数据）
# 2. 录制夹爪开合的 bag（手动让夹爪全开→全闭 3~5 次，约 10~20 秒）
rosbag record -O gripper_calibration.bag /camera/color/image_raw

# 3. 运行标定
cd <你的wheel路径>/gripper_calibration
python3 calibrate_gripper_from_bag.py \
    --bag gripper_calibration.bag \
    --output gripper_range.json \
    --plot
```

标定输出示例：
```
  有效帧: 473/503 (94.0%)
  宽度范围: 29.91mm ~ 132.28mm
  行程: 102.37mm
```

#### B.5.2 启动实时夹爪检测节点

```bash
cd <你的wheel路径>/gripper_calibration
python3 gripper_percent_node.py --calibration gripper_range.json
```

**预期结果**：
- 终端持续输出 `Gripper: XX.X%  width=XXX.X mm`
- `/gripper_percent` topic 发布 0~100 的浮点值

```bash
# 验证
rostopic echo /gripper_percent
```

---

### B.6 第四步：启动实时遥操作

打开 **终端 4**：

```bash
source <你的kuavo-ros-control路径>/devel/setup.bash
cd <你的kuavo-ros-control路径>/src/demo/umi_replay/scripts

# 完整参数启动
python3 umi_realtime_teleop.py \
    --head-frame zhead_2_link \
    --delta-scale 0.5 \
    --rate 20 \
    --fhan-r 2.0 \
    --fhan-h0-scale 5.0 \
    --max-delta 0.35

# 或最简启动（使用默认参数）
python3 umi_realtime_teleop.py

# 不使用夹爪控制
python3 umi_realtime_teleop.py --no-gripper

# 仅跟踪位置不跟踪姿态
python3 umi_realtime_teleop.py --no-orient
```

**操作流程**：

```
启动后看到:
  ========================================
    UMI Realtime Teleop
    Press Enter to start tracking
    Press q or Esc to stop
    Press Ctrl+C to quit
  ========================================

1. 确认动捕数据正常流入（日志无 "No /umi_odom" 警告）
2. 将 UMI 夹爪放到想要的起始位置
3. 按 Enter 键
   → 程序进入 INIT：
     - 计算 T_base_mocap 桥接变换
     - 记录 UMI 当前位置为锚点
     - 读取机器人左臂当前 TF 位姿为锚点
     - 读取机器人右臂当前 TF 位姿（保持不动）
     - 设置 MPC 模式为 ArmOnly (mode=1)
   → 自动进入 TRACKING：
     - 机器人左臂开始跟随 UMI 夹爪的增量运动
     - 右臂保持 INIT 时的位置
     - 夹爪同步跟随（如启用）

4. 移动 UMI 夹爪，机器人左臂实时跟随
5. 按 q 或 Esc 停止跟踪，回到 IDLE
   → MPC 模式恢复为 0
6. 可再次按 Enter 重新开始（锚点重新计算）
7. Ctrl+C 退出程序
```

**参数调优指南**：

| 参数 | 作用 | 调大 | 调小 | 建议值 |
|------|------|------|------|--------|
| `--delta-scale` | 位置增量缩放比 | 手小幅动→臂大幅动 | 手大幅动→臂小幅动 | 0.3~1.0 |
| `--fhan-r` | FHAN 跟踪加速度上限 | 响应更快但更抖 | 更平滑但延迟大 | 1.0~5.0 |
| `--fhan-h0-scale` | FHAN 滤波器步长比 | 响应更快 | 更平滑 | 3.0~8.0 |
| `--rate` | 控制频率 (Hz) | 更精细但 CPU 开销大 | CPU 友好但精度降 | 20~50 |
| `--max-delta` | 安全位移上限 (m) | 允许更大运动范围 | 更安全 | 0.2~0.5 |

---

### B.7 离线轨迹回放流程（替代方案）

如果不做实时遥操作，而是回放已录制的轨迹：

#### B.7.1 录制数据

在动捕运行时，同时录制两个 bag：

```bash
# 终端 A：录制动捕数据（UMI 和 Orbbec 的位姿）
rosbag record -O umi_data.bag /umi_odom /orbbec_odom /camera/color/image_raw

# 终端 B：录制机器人状态（TF 链）
rosbag record -O robot_data.bag /tf /tf_static
```

#### B.7.2 坐标变换：mocap → base_link

```bash
cd <你的kuavo-ros-control路径>/src/demo/umi_replay/scripts

python3 transform_umi_to_base.py \
    --umi-bag ../data/umi_bags/<你的umi数据>.bag \
    --robot-bag ../data/robot_bags/<你的机器人数据>.bag \
    -o ../data/output/umi_in_base.pkl \
    --head-frame zhead_2_link \
    --plot

# 如需提取夹爪开度
python3 transform_umi_to_base.py \
    --umi-bag ../data/umi_bags/<你的umi数据>.bag \
    --robot-bag ../data/robot_bags/<你的机器人数据>.bag \
    -o ../data/output/umi_in_base.pkl \
    --gripper-cal ../../gripper_calibration/gripper_range.json \
    --plot
```

#### B.7.3 在仿真中回放

```bash
# 确保仿真已启动（B.3 节）

# 绝对位姿模式
python3 replay_umi_in_mujoco.py \
    --pkl ../data/output/umi_in_base.pkl \
    --speed 1.0

# 增量模式（推荐，更平滑）
python3 replay_umi_in_mujoco.py \
    --pkl ../data/output/umi_in_base.pkl \
    --incremental \
    --speed 0.5
```

---

### B.8 NUC 适配注意事项

#### B.8.1 路径适配

以下路径需要按 NUC 实际情况修改：

| 原始路径 | 改为 | 涉及操作 |
|---------|------|---------|
| `/home/leju/catkin_ws/src/wheel/` | NUC 上 wheel 的实际路径 | 所有 `cd` 命令 |
| `gripper_calibration/gripper_range.json` | NUC 上的实际路径 | 夹爪标定与检测 |
| `../data/umi_bags/` | NUC 上 bag 文件的存放位置 | 离线回放 |

#### B.8.2 TF Frame 名适配

如果 NUC 上的机器人 URDF 与开发机不同，以下 TF frame 名可能需要修改：

| 用途 | 默认 frame 名 | 修改位置 |
|------|--------------|---------|
| 头部（桥接变换） | `zhead_2_link` | `--head-frame` 参数 |
| 左臂末端 | `zarm_l7_link` | `umi_realtime_teleop.py` 第 365 行 |
| 右臂末端 | `zarm_r7_link` | `umi_realtime_teleop.py` 第 378 行 |

验证方法：

```bash
# 列出 TF 树中所有 frame
rosrun tf tf_monitor | head -50
# 或
rosrun tf view_frames  # 生成 frames.pdf
```

#### B.8.3 网络配置（多机通信场景）

如果动捕主机、NUC、下位机分布在不同网段：

```bash
# NUC 上 ~/.bashrc
export ROS_MASTER_URI=http://<NUC_IP>:11311
export ROS_IP=<NUC_IP>

# 动捕主机上
export ROS_MASTER_URI=http://<NUC_IP>:11311
export ROS_IP=<动捕主机IP>
```

#### B.8.4 D405 相机设备号

NUC 上 D405 的 `/dev/video*` 编号可能与开发机不同：

```bash
# 查看已连接的视频设备
ls /dev/video*
# 用 v4l2 确认哪个是 D405
v4l2-ctl --list-devices

# 如果 realsense ROS 驱动已装：
roslaunch realsense2_camera rs_camera.launch
# 验证 topic
rostopic hz /camera/color/image_raw
```

---

### B.9 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `No /umi_odom data. Cannot initialize.` | 动捕推流未启动或 topic 名不对 | `rostopic list` 检查；确认 topic 名为 `/umi_odom` |
| `No /orbbec_odom received yet` | Orbbec 刚体未在动捕中被跟踪 | 检查动捕软件中 Orbbec 刚体是否激活 |
| `TF base_link->zhead_2_link not available` | 仿真未启动或 TF frame 名不匹配 | 先确认仿真正常运行；`rosrun tf tf_echo base_link zhead_2_link` |
| `MPC control mode switch failed` | MPC 服务未就绪 | 确认仿真/控制器已完全启动后再按 Enter |
| 机械臂不动 | MPC 模式未切换成功，或指令超出工作空间 | 检查日志中 `MPC control mode set to 1` 是否出现；减小 `--delta-scale` |
| 机械臂抖动严重 | FHAN 参数不合适，或动捕数据噪声大 | 减小 `--fhan-r`（如 1.0）；增大 `--fhan-h0-scale`（如 8.0） |
| 机械臂响应滞后 | FHAN 平滑过度 | 增大 `--fhan-r`；减小 `--fhan-h0-scale` |
| `Gripper: -1.0%` 持续出现 | ArUco 标签未检测到 | 检查光照、标签是否遮挡、D405 对焦；`--marker-size` 是否正确 |
| 夹爪开度映射不准 | 标定文件不匹配 | 在 NUC 上用当前 D405 重新运行标定 |

---

### B.10 完整操作 Checklist

按以下顺序逐项确认：

- [ ] **环境就绪**
  - [ ] `source devel/setup.bash` 无报错
  - [ ] `rosmsg show kuavo_msgs/twoArmHandPoseCmd` 能显示
  - [ ] `echo $ROBOT_VERSION` 输出 60
- [ ] **终端 1：仿真**
  - [ ] `roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch`
  - [ ] MuJoCo 窗口正常显示
  - [ ] TF 可用：`rosrun tf tf_echo base_link zarm_l7_link`
- [ ] **终端 2：动捕**
  - [ ] `/umi_odom` 有数据：`rostopic hz /umi_odom`
  - [ ] `/orbbec_odom` 有数据：`rostopic hz /orbbec_odom`
- [ ] **终端 3（可选）：夹爪**
  - [ ] D405 图像正常：`rostopic hz /camera/color/image_raw`
  - [ ] `gripper_range.json` 存在且是当前 D405 标定的
  - [ ] `python3 gripper_percent_node.py --calibration gripper_range.json` 运行中
  - [ ] `/gripper_percent` 有数据：`rostopic echo /gripper_percent`
- [ ] **终端 4：遥操作**
  - [ ] `python3 umi_realtime_teleop.py` 启动，无报错
  - [ ] 按 Enter → 日志显示 `INIT complete. Entering TRACKING`
  - [ ] 移动 UMI → MuJoCo 中左臂跟随
  - [ ] 按 q → 回到 IDLE，机械臂停止
