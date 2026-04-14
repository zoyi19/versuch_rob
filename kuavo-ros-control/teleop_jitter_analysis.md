# 遥操平滑性诊断分析报告

**日期**: 2026-04-13  
**数据文件**: `~/ziyi/teleop_diag_20260413_160105.csv`  
**记录时长**: 51.5 秒, 28040 条数据

---

## 1. 速率链路

```
Vicon mocap        ~541 Hz  (事件驱动,无过滤)
  ↓ /umi_odom (nav_msgs/Odometry)
umi_realtime_teleop   20 Hz    (FHAN smoother, delta_scale=0.5, r=2.0, h0=0.25)
  ↓ /mm/two_arm_hand_pose_cmd (kuavo_msgs/twoArmHandPoseCmd)
IK Target             20 Hz    (无插值,直接转发)
  ↓
MPC solver           100 Hz    (task.info: mpcDesiredFrequency=100)
  ↓
MRT execution        500 Hz    (task.info: mrtDesiredFrequency=500)
  ↓
WBC controller       500 Hz    (wbc_frequency=500)
  ↓
MuJoCo physics       500 Hz    (Mujoco Frequency=500)
```

---

## 2. 数据分析结果

### 2.1 各环节位置步长统计

| 指标 | UMI 原始 (mocap) | CMD (smoother 输出) | EE (实际执行) |
|------|------------------|---------------------|---------------|
| 平均步长 | 0.084 mm | 0.023 mm | 0.031 mm |
| 最大步长 | **100.2 mm** | 8.7 mm | 2.0 mm |
| 标准差 | 1.54 mm | 0.29 mm | 0.08 mm |
| >10mm 跳变次数 | **49 次** | 0 | 0 |
| >2mm 跳变次数 | — | **96 次** | 0 |

### 2.2 跟踪误差 (CMD vs EE)

| 指标 | 数值 |
|------|------|
| 平均误差 | 2.34 mm |
| 最大误差 | 38.37 mm |
| 标准差 | 4.67 mm |
| 误差 > 1mm 的比例 | 36.3% |
| 误差 > 5mm 的比例 | 10.8% |
| 误差 > 10mm 的比例 | 5.5% |
| 误差 > 20mm 的比例 | 2.4% |

### 2.3 UMI 原始数据跳变详情

最大的跳变集中在 t=1776067286~287 这 2 秒内(UMI 快速移动或 mocap 遮挡):

```
t=...286.768  dt=0.5ms  jump=52.1mm
t=...287.589  dt=0.9ms  jump=61.8mm
t=...286.700  dt=1.9ms  jump=21.9mm
t=...287.335  dt=3.5ms  jump=34.1mm
```

0.5ms 内跳 52mm 在物理上不可能(相当于 104 m/s 速度),这是 **mocap 系统的测量异常值**。

### 2.4 CMD (smoother 输出) 跳变详情

FHAN smoother 将 mocap 的 100mm 跳变抑制到了 8.7mm,但仍存在 96 次 >2mm 的跳变:

```
t=...287.134  dt=8.3ms  jump=8.5mm   ← smoother 追踪大跳变
t=...287.180  dt=0.5ms  jump=8.7mm
t=...287.231  dt=15.0ms jump=8.6mm   ← 注意 dt 不均匀
t=...287.280  dt=0.5ms  jump=8.3mm
```

每次跳变发生时,连续 10~15 帧 (约 500ms) 都在 >2mm,说明 smoother 在追踪一个大的位移变化。

---

## 3. 根因分析

### 根因排序(数据驱动)

| 排名 | 原因 | 证据 | 影响程度 |
|------|------|------|----------|
| **1** | **mocap 原始数据跳变** | 49 次 >10mm,最大 100mm,0.5ms 内 | 源头污染 |
| **2** | **20Hz 离散发布率** | cmd 每 50ms 跳一次,MPC@100Hz 看到阶梯信号 | 持续性抖动 |
| **3** | **FHAN r=2.0 过快** | 100mm 跳变只衰减到 8.7mm | 大跳变透传 |
| 4 | CPU 偶尔丢帧 | 日志显示偶尔 >20ms | 间歇,非主因 |

### 详细分析

**根因 1: mocap 跳变**

Vicon 动捕系统在特定时刻产生异常大的位置跳变(单帧 100mm)。这些跳变可能由以下原因造成:
- UMI 上的反光标记被部分遮挡,导致刚体解算跳变
- Vicon 系统在多标记间切换解算
- 网络延迟导致帧堆积后批量到达

这些异常值经过 FHAN smoother 后被衰减(100mm→8.7mm),但仍然足以在 MuJoCo 中产生可见的"一抖"。

**根因 2: 20Hz 发布率**

遥操脚本以 20Hz 发布目标位姿,而 MPC 以 100Hz 运行。这意味着:
- MPC 每 5 个计算周期才收到一次新目标
- 目标更新时产生阶梯型跳变
- MPC 急刹/急追 → 视觉上的不平滑

**根因 3: FHAN 参数**

当前 `r=2.0` 表示 smoother 允许的最大加速度为 2.0 m/s²。这个值较大,smoother 响应快但对噪声的抑制不够。

---

## 4. 优化推荐方案

### 方案 A: 调参优化 (零代码改动,立即可试)

只需修改遥操启动命令的参数:

```bash
# 原始参数 (当前)
python3 umi_realtime_teleop.py --no-gripper --rate 20 --fhan-r 2.0

# 优化参数
python3 umi_realtime_teleop.py --no-gripper \
    --rate 50 \         # 20Hz → 50Hz: 减少 MPC 阶梯感
    --fhan-r 1.0 \      # 2.0 → 1.0: 增强噪声抑制
    --fhan-h0-scale 5.0 \
    --delta-scale 0.5 \
    --max-delta 0.35
```

| 参数 | 原值 | 新值 | 效果 |
|------|------|------|------|
| rate | 20 Hz | 50 Hz | MPC 每 2 个周期收到新目标 (原 5 个) |
| fhan-r | 2.0 | 1.0 | 大跳变从 8.7mm 降到 ~4mm |

**预期改善**: cmd 跳变减少 50%+,视觉平滑度明显提升  
**代价**: 跟踪延迟增加 ~30-50ms (快速动作稍慢)  
**风险**: 极低,Python 50Hz 无压力

### 方案 B: mocap 跳变过滤 (需改代码,根治异常值)

在 `umi_realtime_teleop.py` 的 `_cb_umi` 回调中加入跳变检测:

**文件**: `kuavo-ros-control/src/demo/umi_replay/scripts/umi_realtime_teleop.py` L298-299

```python
def _cb_umi(self, msg):
    new_pose = odom_to_pose(msg)
    # 跳变检测: 与上一帧比较,>50mm 直接丢弃
    if self._latest_umi is not None:
        delta = np.linalg.norm(new_pose[0] - self._latest_umi[0])
        if delta > 0.05:  # 50mm
            rospy.logwarn_throttle(1.0, f"UMI jump detected: {delta*1000:.0f}mm, skipping")
            return
    self._latest_umi = new_pose
```

**效果**: 彻底消除 100mm 级异常跳变  
**代价**: 跳变帧被丢弃,但 ~541Hz 丢个别帧影响极小

### 方案 C: SampleClient 过滤无效值 (消除 RViz 刷屏)

在 `SampleClient.cpp` DataHandler 中,发布 odom/TF 前检查有效性:

**文件**: `Vicon/src/automatic_test/motioncapture/src/SampleClient.cpp` L469 之前

```cpp
// 过滤 Seeker SDK 的 9999999 无效占位值
if (std::abs(rb_pose.position.x) > 100000 ||
    std::abs(rb_pose.position.y) > 100000 ||
    std::abs(rb_pose.position.z) > 100000) {
    continue;  // 跳过此刚体
}
```

**效果**: 消除 RViz 的 `invalid quaternion` 刷屏  
**代价**: 无

### 推荐执行顺序

```
第 1 步: 方案 A (改启动参数 --rate 50 --fhan-r 1.0)
         ↓ 记录第二组诊断数据对比
第 2 步: 方案 B (加 mocap 跳变过滤)
         ↓ 记录第三组对比
第 3 步: 方案 C (SampleClient 过滤无效值)
```

---

## 5. 验证方法

每次优化后:

1. 启动仿真 + mocap + 遥操(新参数)
2. 运行 `teleop_diagnostics.py` 记录 30 秒数据
3. 对比关键指标:

| 指标 | 基线 (当前) | 目标 |
|------|------------|------|
| CMD >2mm 跳变次数/30s | ~60 | < 20 |
| 平均跟踪误差 | 2.34 mm | < 1.5 mm |
| 最大跟踪误差 | 38 mm | < 10 mm |
| 视觉感受 | "一抖一抖" | 平滑跟随 |

4. 目视确认 MuJoCo 中手臂运动是否更平滑

---

## 6. 相关文件

| 文件 | 作用 |
|------|------|
| `src/demo/umi_replay/scripts/umi_realtime_teleop.py` | 遥操主脚本 (smoother + 发布) |
| `src/demo/umi_replay/scripts/teleop_diagnostics.py` | 诊断记录工具 |
| `Vicon/src/automatic_test/motioncapture/src/SampleClient.cpp` | mocap 数据发布 |
| `~/ziyi/teleop_diag_20260413_160105.csv` | 基线诊断数据 |
