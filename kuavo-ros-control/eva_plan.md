# UMI 数据采集 → 策略训练 → MuJoCo 仿真评估 方案

## Context
当前 kuavo-ros-control 项目已经完成 UMI 实时遥操（`src/demo/umi_replay/scripts/umi_realtime_teleop.py`）和 MuJoCo 轨迹回放（`replay_umi_in_mujoco.py`）。下一步：脱离遥操，直接用 UMI 手持夹爪在真实环境中采数 → 训练视觉运动策略（如 Diffusion Policy）→ 在 MuJoCo（`load_kuavo_mujoco_sim_wheel.launch`）中安全地反复评估，避免上实机风险。核心问题：**如何设计一套可重复、可量化、低成本的仿真评估流程**。

> 注：腕部相机已从 GoPro 鱼眼改为 **Intel RealSense D405**（pinhole 模型 + 可选深度），相比 UMI 官方 GoPro 方案省去了鱼眼标定与去畸变环节，sim2real 视觉对齐更简单，但视角（FOV）和工作距离都比 GoPro 窄，需要在采数和 sim 建模两边都注意。

---

## 推荐方案

### 1. 数据采集（UMI 手持夹爪 + D405，无需机器人）
基本流程仍参考 UMI（https://umi-gripper.github.io/），但相机部分按 D405 调整：
- 把 D405 装在 UMI 手持夹爪原 GoPro 位置上（注意机械适配 + 标定相机相对夹爪 TCP 的外参 `T_tcp_cam`，这一步必做，否则训练数据中"看到的物体位置"和"夹爪动作"不在一个系里）
- 采集：
  - `/wrist_cam/color/image_raw`（D405 RGB，1280×720 或 848×480）
  - 可选：`/wrist_cam/depth/image_rect_raw`（D405 的强项，但要先确认训练框架能用上深度，不用就别存）
  - 夹爪 6DoF 轨迹（base_link 系下，与 `transform_umi_to_base.py` 输出一致）
  - gripper width
- D405 没有 IMU，**夹爪 pose 不能依赖相机自身 VIO**，需要：
  - 方案 A：继续用外部 OptiTrack（与现有遥操 mocap 链路一致，最稳）
  - 方案 B：用 ArUco/AprilTag 贴在工作区周围，靠 D405 视觉解算 pose（成本低，但精度差、易丢失）
  - 推荐 A，复用现有 mocap 设备
- 每个任务 50–200 条 demo，覆盖物体位置/光照/干扰
- 数据格式与 UMI 官方 zarr/replay buffer 对齐，方便直接喂 Diffusion Policy

**关键**：训练 obs 是腕部 D405 视角，sim 评估时也必须从机器人末端 D405 安装位置渲染同视角图像，否则 train/test gap 巨大。

### 2. 策略训练
- 推荐 Diffusion Policy（UMI 默认）或 ACT
- 输入：腕部 D405 RGB（必要时多帧；深度可选作为额外通道）+ proprioception（末端 pose、gripper width）
- 输出：未来 N 步末端 pose delta（与 `umi_realtime_teleop.py` 的 frame=2 增量控制天然兼容）
- D405 是 pinhole，**不需要去畸变预处理**，直接 resize/crop 到模型输入分辨率即可

### 3. MuJoCo 仿真评估闭环（核心）

#### 3.1 在 MuJoCo 中加虚拟 D405 腕部相机
- 修改 `src/kuavo_assets/models/biped_s51/xml/biped_s51_mujoco.xml`（或 wheel 版）：在末端 link 上加：
  ```xml
  <camera name="wrist_cam" pos="..." quat="..." fovy="58"/>
  ```
  - **fovy 用 D405 真实参数**：D405 RGB 水平 FOV ≈ 87°，垂直 ≈ 58°（用 vertical FOV 填 fovy）
  - `pos`/`quat` 必须等于真实安装中 `T_endlink_cam`（来自第 1 步的相机-TCP 外参标定结果），这是 sim2real 视觉一致性的命脉
  - 渲染分辨率与训练一致（如 848×480）
- 通过 mujoco ROS 插件发布 `/wrist_cam/color/image_raw`，topic 名与真实 D405 完全一致，policy_runner 不用区分 sim/real
- D405 是 pinhole + 无明显畸变，MuJoCo 默认 pinhole 渲染天然匹配，**这是改用 D405 后最大的好处**——省掉了 GoPro 鱼眼那一套 undistort/重投影
- （可选）若训练用了深度，MuJoCo 也能渲染 depth buffer，发到 `/wrist_cam/depth/image_rect_raw`

#### 3.2 策略 → 机器人控制桥接
- 写一个 `policy_runner.py`：订阅 `/wrist_cam/color/image_raw` + TF（末端 pose）→ 跑模型 → 发布 `twoArmHandPoseCmd`（frame=2，复用 `umi_realtime_teleop.py` 里已经验证的发布逻辑）
- **复用现有代码**：`umi_realtime_teleop.py` 的 FHAN 平滑、增量裁剪（`--max-delta`）、frame=2 协议直接套用，只是把 mocap 输入换成模型输出

#### 3.3 自动化评估场景
- MuJoCo 场景里放置目标物体（box/cup），随机化：
  - 物体初始 pose（每 episode reset 时通过 `mj_setState` 或加载不同 xml）
  - 光照、纹理（domain randomization，缩小 sim2real gap）
- 用 ROS service 或直接 mujoco python API 做 reset，避免每次重启 launch

#### 3.4 成功判据 & 指标
脚本化判定，每个 episode 自动记录：
- **Success rate**（主指标）：物体最终 pose 是否进入目标区域 / gripper 是否抓住
- **Completion time**、**trajectory length**、**平均 jerk**（平滑度）
- **Safety**：是否触发 `--max-delta` 截断、是否自碰撞、关节是否到限
- **Visual divergence**：可选，记录腕部相机 obs 与训练集最近邻距离，监控 OOD

#### 3.5 批量回归测试
- 参考已有 `src/automatic_test/automatic_test/scripts/automatic_test/test_robot_walk.py` 的 pytest + YAML 结果模式，写 `test_policy_pickplace.py`：
  - 参数化：seed × 物体位置 × 光照 → 例如 50 个 episode
  - 输出 YAML/CSV，CI 里 diff 历次成功率
- headless 模式（`mujoco_headless:=true`）+ 多进程，可一晚跑几百次

### 4. 分阶段验收（推荐顺序，逐步降低风险）

| 阶段 | 内容 | 通过门槛 |
|---|---|---|
| A | **Replay 验证**：用 `replay_umi_in_mujoco.py` 把训练集真实轨迹直接在 sim 里回放，看末端是否能跟到目标、能否"假抓"成功 | 物理可行性 OK，sim 动力学没卡死 |
| B | **Open-loop 策略**：模型预测整段轨迹，不闭环视觉，sim 里执行 | 验证模型轨迹合理 |
| C | **Closed-loop 策略**：每步用 sim 渲染的 D405 图像喂模型 | 主评估，跑 50+ episode 出成功率 |
| D | **Domain randomization 鲁棒性**：随机化物体/光照后再跑 | 成功率不能掉太多 |
| E | （可选）**硬件 dry-run**：实机但不接触物体，只看末端轨迹是否安全 | 准备上实机前最后一关 |

### 5. 关键注意点
- **D405 内外参一致性**是 sim2real 关键：
  - **内参**：sim 的 `fovy` 用 D405 真实垂直 FOV（≈58°），分辨率与训练一致
  - **外参**：sim XML 里相机相对末端 link 的 `pos`/`quat` 必须等于真实标定的 `T_endlink_cam`，差几毫米/几度都会让闭环视觉失效
- **D405 工作距离**：最近 ≈7 cm，最远有效 RGB 约 50 cm 内，比 GoPro 视野窄很多——采数时夹爪要靠近物体，sim 里物体也要相应放置在合理距离
- **控制接口对齐**：训练数据 action 是 UMI 夹爪在 base_link 下的绝对/相对 pose；推理时同坐标系，复用 `transform_umi_to_base.py`
- **Gripper 同步**：sim 里 `/leju_claw_command` 已存在，直接发模型预测的 width
- **不要重启 launch 来 reset**：写一个 reset service，否则 50 个 episode 要跑半天

---

## 关键文件
- `src/demo/umi_replay/scripts/umi_realtime_teleop.py` — 复用 frame=2 发布与 FHAN 平滑
- `src/demo/umi_replay/scripts/replay_umi_in_mujoco.py` — 阶段 A 直接可用
- `src/demo/umi_replay/scripts/transform_umi_to_base.py` — 坐标系转换
- `src/kuavo_assets/models/biped_s51/xml/biped_s51_mujoco.xml` — 加 D405 腕部相机
- `src/automatic_test/automatic_test/scripts/automatic_test/test_robot_walk.py` — 自动化测试模板

## 验证方式
1. `roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch mujoco_headless:=true`
2. `rosrun ... policy_runner.py --ckpt xxx.pt`
3. `pytest test_policy_pickplace.py -k pickplace --episodes 50`，看输出 YAML 中 success_rate

---

## 插花任务特化（flower-in-vase）

本节是上面通用方案在"用 UMI 采数 → 训练 Lingbot-VLA → 在仿真里把假花插进花瓶"这一具体任务上的特化。**约束：所有新代码与场景文件集中在新目录 `src/umi_eva/`，不修改任何已有文件。** 现有 `biped_sXX_mujoco.xml`、`umi_realtime_teleop.py`、`automatic_test/` 等都不动，回滚只需删 `src/umi_eva/`。

### 1. Sim 选型 —— MuJoCo
本任务用 MuJoCo（沿用 `roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch`），不切 Isaac。理由：
- 项目已有完整 MuJoCo + ROS + UMI 链路（`replay_umi_in_mujoco.py`、`automatic_test`），Isaac launch 只是骨架，没有 UMI/D405/policy_runner 链路
- ROS topic（`/wrist_cam/color/image_raw`、`twoArmHandPoseCmd`）与真实硬件一致，policy 不区分 sim/real
- D405 是 pinhole，MuJoCo 原生 pinhole 渲染天然匹配，sim2real 视觉 gap 最小
- 训练是 IL（Lingbot-VLA / Pi0.5），不需要 Isaac 的 GPU 并行 env
- headless + 多进程跑批量评估最快，已有 `automatic_test` pytest + YAML 模板可参考
- **唯一推荐 Isaac 的情形**：训练阶段需要 GPU 并行 1000+ env 跑 RL/sim2real 预训练。本任务用不上

### 2. 资产建模（零外部依赖，方案 A 刚体）
不引入任何外部 mesh 文件，全部用 MJCF primitive 拼装。

- **花瓶**：底座 `cylinder`（外径 ~8 cm、高 1 cm）+ 内壁用 8 个薄 `box` 围成开口圆筒（壁厚 3 mm、内半径 3 cm、高 12 cm），固定在桌面 body 上
- **假花（方案 A，刚体）**：
  - 茎：`capsule` 半径 4 mm、长 25 cm
  - 花头：`sphere` 半径 2 cm
  - 整体一个 body + `freejoint`，可被夹爪抓取
  - 外观差异（红/黄/白等）交给 domain randomization 的 `material` 切换，不在几何上死磕
  - 不做柔性/`composite/flex`，插花任务里花本来就不需要弯
- **桌面**：复用现有 demo scene，不动

具体 MJCF 写在 `src/umi_eva/scenes/flower_scene.xml`，由主 scene XML 通过 `<include>` 引入。**本目录不直接修改 `biped_sXX_mujoco.xml`**，需要使用时由用户手动在主 scene 里加一行 `<include file="../../umi_eva/scenes/flower_scene.xml"/>`（路径按实际调整）。

### 3. 腕部相机注入（不改原文件）
D405 腕部相机的 XML 片段单独放在 `src/umi_eva/scenes/wrist_cam_snippet.xml`，内容遵循本文档 §3.1：`<camera name="wrist_cam" fovy="58" pos=... quat=...>`，pos/quat 等于真实标定的 `T_endlink_cam`。使用方式同样是用户手动在主 scene 的末端 link 下 include，**本任务不去改 `biped_s51_mujoco.xml`**。

### 4. Success 判据（脚本化）
每个 step 检查 flower body 的 pose，四条件同时满足并持续 1 s 算成功：
1. 花头 z > 瓶口 z
2. 花茎底端 z < 瓶口 z
3. 花茎底端 (x, y) 距瓶口中心水平距离 < 瓶口内半径
4. gripper width > 松开阈值（如 0.06 m）

实现放在 `src/umi_eva/success_checker.py`，提供两种调用方式：纯函数（输入花/瓶 pose + gripper width，返回 bool）+ ROS 节点（订阅 TF / `/leju_claw_state`，自动持续 1 s 判定）。同时记录本文档 §3.4 的辅助指标：completion time、jerk、`--max-delta` 截断次数、自碰撞。

### 5. Domain randomization（中度）
强度为"中度"——**包括纹理/材质切换**，但不动相机外参和花瓶尺寸。每 episode reset 随机：
- 花瓶在桌面 xy ±5 cm，yaw ±30°
- 花的初始 pose（桌面另一侧 xy ±8 cm，yaw 任意）
- 光源方向 + 强度
- 桌布、瓶身、背景的纹理/材质切换（MuJoCo `material` 列表）

**不**随机：相机外参（破坏 sim2real 视觉一致性）、花瓶尺寸（破坏 success 判据）、花的几何参数。实现放在 `src/umi_eva/domain_randomizer.py`，通过 mujoco python API + reset service 调用，避免每次重启 launch。

### 6. policy_runner（Lingbot-VLA 接入占位）
`src/umi_eva/policy_runner.py`：
- obs：`/wrist_cam/color/image_raw` + language instruction（"insert the flower into the vase"）+ proprio（末端 pose、gripper width）
- action：未来 N 步 end-effector delta pose + gripper width
- 复用 `umi_realtime_teleop.py` 的 frame=2 发布、FHAN 平滑、`--max-delta` 裁剪逻辑（**通过 import 调用，不修改原文件**）
- ckpt 加载与 forward 调用先按 Pi0.5 风格占位（TODO 注释标注），用户后续提供 Lingbot-VLA 的 ckpt + python API 后只替换这两处，其他管线保持不动

### 7. 自动化批量评估
`src/umi_eva/test_policy_flower.py`：pytest 入口，参数化 seed × 物体位置 × 光照，跑 50 个 episode，输出 YAML（success_rate、completion_time、jerk、safety violations）。结构参考 `automatic_test/scripts/automatic_test/test_robot_walk.py` 但不依赖它，独立可跑。

### 8. 分阶段验收

| 阶段 | 内容 | 门槛 |
|---|---|---|
| A | `replay_umi_in_mujoco.py` 回放真实插花 demo | 末端能跟到瓶口、刚体花能被"假抓"住 |
| B | Lingbot-VLA open-loop 整段轨迹回放 | 轨迹形状合理 |
| C | Closed-loop，每步用 sim 渲染 D405 图像，跑 50 ep | 主指标，成功率 > 阈值 |
| D | 加中度 DR 再跑 50 ep | 成功率掉幅 < 20% |
| E | 实机 dry-run（空抓不接触物体） | 末端轨迹安全 |

### 9. `src/umi_eva/` 目录结构

```
src/umi_eva/
├── README.md                       # 目录用途、文件索引、运行方式
├── scenes/
│   ├── flower_scene.xml            # MJCF primitive 花瓶 + 刚体假花，主 scene 手动 include
│   └── wrist_cam_snippet.xml       # D405 腕部相机 XML 片段，主 scene 手动 include
├── policy_runner.py                # Lingbot-VLA 推理桥接（占位骨架）
├── success_checker.py              # 四条件 + 1 s 持续判定
├── domain_randomizer.py            # 中度 DR + reset
└── test_policy_flower.py           # pytest 批量评估入口
```

启动顺序：
```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch mujoco_headless:=true
rosrun umi_eva policy_runner.py --ckpt lingbot_vla_flower.pt   # 等 ckpt 到位
pytest src/umi_eva/test_policy_flower.py --episodes 50
```

### 10. 待用户后续确认
- 花瓶 / 花的具体尺寸是否要按真实物体测量后改写？（当前是合理默认值）
- 评估每阶段 episode 数是否沿用 50？
- 失败 episode 是否需要自动保存视频/轨迹用于回查？
- Lingbot-VLA 的 ckpt 与 python API 规范（输入预处理、history 长度、action 解码方式）

