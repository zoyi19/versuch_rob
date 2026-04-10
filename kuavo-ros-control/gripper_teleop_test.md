# 带夹爪遥操端到端测试方案

## 数据流

```
D405 手腕摄像头 → /camera/color/image_raw
       ↓
gripper_percent_node.py (ArUco tag 0+1 → 开合百分比)
       ↓
/gripper_percent (Float32, 0~100%, 检测失败=-1)
       ↓
umi_realtime_teleop.py (不加 --no-gripper)
       ↓
/leju_claw_command (kuavo_msgs/lejuClawCommand)
  data.name = ["left_claw", "right_claw"]
  data.position = [percent, 0.0]
  data.velocity = [50.0, 50.0]
  data.effort = [1.0, 1.0]
       ↓
/nodelet_manager (仿真控制器已订阅)
```

> MuJoCo 末端非 claw 模型,无法直观看到夹爪闭合。通过监控 `/gripper_percent` 和 `/leju_claw_command` 的数值变化来验证链路。

---

## 前置条件

| 条件 | 命令/操作 |
|------|---------|
| Terminal 1: 仿真已启动 | `roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch` |
| Vicon/mocap 已启动 | `cd ~/ziyi/Vicon && source devel/setup.bash && rosrun motioncapture SampleClient` |
| mocap 数据正常 | `rostopic hz /umi_odom /orbbec_odom` 均 >10Hz |

---

## 测试 1: 标定文件验证

**目标**: 确认 `gripper_range.json` 参数合理

```bash
cat /home/lab/ziyi/versuch_rob/gripper_calibration/gripper_range.json
```

**预期**:
- `left_finger_tag_id`: 0, `right_finger_tag_id`: 1
- `min_width` (~0.030) < `max_width` (~0.132)
- `nominal_z` 在 0.05~0.15 范围 (D405 到夹爪距离,当前 0.088)
- `camera_intrinsics`: fx=425, fy=425, cx=424, cy=240, 分辨率 848x480
- 若换过 UMI 设备,需重新标定

---

## 测试 2: D405 图像流

**目标**: 确认 D405 发布图像话题

**操作**: 打开 D405 手腕摄像头

```bash
rostopic list | grep camera
rostopic hz /camera/color/image_raw
```

**预期**:
- `/camera/color/image_raw` 存在
- 频率 > 15Hz
- 若 topic 不存在,检查 D405 USB 连接和 realsense 驱动

---

## 测试 3: gripper_percent_node 启动

**目标**: ArUco 检测正常,百分比输出

```bash
cd /home/lab/ziyi/versuch_rob/gripper_calibration
python3 gripper_percent_node.py --calibration gripper_range.json
```

**预期**:
- 无 traceback (OpenCV API 已修复)
- 日志: `GripperPercent ready tag 0+1 width [29.9, 132.3] mm nominal_z=0.0882 m topic=/camera/color/image_raw`
- 手动张合夹爪时: `Gripper: XX.X%`

**验证**:
```bash
# 另开终端
rostopic hz /gripper_percent          # 应 > 0 Hz
rostopic echo /gripper_percent        # 张开→接近100, 闭合→接近0
```

**故障排查**:
- 百分比一直 -1 → tag 未被检测到,检查光照/遮挡/tag 打印质量
- 百分比不随手动张合变化 → `nominal_z` 或 `z_tolerance` 不匹配当前安装距离,需重新标定

---

## 测试 4: 带夹爪遥操启动

**目标**: `umi_realtime_teleop.py` 激活夹爪逻辑

```bash
cd /home/lab/ziyi/versuch_rob/kuavo-ros-control
source devel/setup.bash
cd src/demo/umi_replay/scripts
python3 umi_realtime_teleop.py \
    --head-frame zhead_2_link \
    --delta-scale 0.5 \
    --rate 20 \
    --fhan-r 2.0 \
    --fhan-h0-scale 5.0 \
    --max-delta 0.35
```

> 注意:不要加 `--no-gripper`!

**预期**:
- 按 Enter → INIT → TRACKING
- `|delta|` 不为 0 (UMI 在动)
- 无 TF/服务报错

---

## 测试 5: 夹爪命令验证 (核心)

**目标**: `/leju_claw_command` 跟随 `/gripper_percent` 变化

```bash
# 终端 A: 监控夹爪百分比
rostopic echo /gripper_percent

# 终端 B: 监控夹爪指令
rostopic echo /leju_claw_command
```

**操作**: 进入 TRACKING 后,手动张合 UMI 夹爪

**预期**:

| 动作 | /gripper_percent | /leju_claw_command data.position[0] |
|------|-----------------|--------------------------------------|
| 完全张开 | ~100% | ~100.0 |
| 半开 | ~50% | ~50.0 |
| 完全闭合 | ~0% | ~0.0 |
| tag 被遮挡 | -1.0 | 不发新消息 (保持上一次) |

- `data.name` 应为 `["left_claw", "right_claw"]`
- 两个话题数值同步变化 = 链路完整

---

## 测试 6: 异常场景

### 6a. ArUco tag 遮挡
**操作**: 遮住一个手指 tag
**预期**: `/gripper_percent` 发 -1.0,遥操脚本跳过不发 claw 命令 (代码 L447: `if self._latest_gripper >= 0`)

### 6b. D405 未开启
**预期**: `/gripper_percent` 话题无数据,遥操脚本 `_latest_gripper` 保持 `None`,不发 claw 命令 (代码 L446: `if ... self._latest_gripper is not None`)

### 6c. gripper_percent_node 未启动
**预期**: 同 6b,遥操可正常跑(仅手臂跟随,无夹爪)

---

## 端到端成功标准

- [ ] gripper_percent_node 无报错启动
- [ ] /gripper_percent 频率 > 0Hz,数值随手动张合变化 (0~100%)
- [ ] /leju_claw_command 频率与遥操帧率一致 (~20Hz)
- [ ] leju_claw_command.data.position[0] 跟随 gripper_percent 同步变化
- [ ] 同时手臂跟随 UMI 位姿移动 (与 --no-gripper 模式一致)
- [ ] 遮挡 tag 时不发错误命令

---

## 关键文件路径

| 文件 | 作用 |
|------|------|
| `gripper_calibration/gripper_percent_node.py` | D405 ArUco → /gripper_percent |
| `gripper_calibration/gripper_range.json` | 标定参数 |
| `kuavo-ros-control/src/demo/umi_replay/scripts/umi_realtime_teleop.py` | 遥操主脚本 (L446-448 夹爪发送) |

## 启动顺序总结

```
1. roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
2. cd ~/ziyi/Vicon && source devel/setup.bash && rosrun motioncapture SampleClient
3. 打开 D405 手腕摄像头
4. cd ~/ziyi/versuch_rob/gripper_calibration && python3 gripper_percent_node.py --calibration gripper_range.json
5. cd ~/ziyi/versuch_rob/kuavo-ros-control && source devel/setup.bash && cd src/demo/umi_replay/scripts && python3 umi_realtime_teleop.py --head-frame zhead_2_link --delta-scale 0.5 --rate 20 --fhan-r 2.0 --fhan-h0-scale 5.0 --max-delta 0.35
```
