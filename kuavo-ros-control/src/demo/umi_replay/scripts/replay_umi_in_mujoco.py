#!/usr/bin/env python3
"""
在 MuJoCo 仿真中回放 UMI 夹爪轨迹（仅左臂，仅 MPC）。

支持两种模式:
  - 绝对式 (默认):  将 pkl 中的绝对位姿直接发送给 MPC
  - 增量式 (--incremental):  从轨迹提取帧间增量，以机器人当前末端为锚点累加

增量式逻辑与 VR 增量式遥操作 (IncrementalControlModule / quest3_node_incremental.py) 一致，
使用 FHAN 平滑器 (Han's fastest tracking differentiator) 抑制抖动。
"""

import math
import argparse
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp

import rospy
import tf2_ros
from std_msgs.msg import Header
from kuavo_msgs.msg import (
    twoArmHandPoseCmd,
    armHandPose,
    lejuClawCommand,
    endEffectorData,
)
from kuavo_msgs.srv import (
    changeTorsoCtrlMode,
    changeTorsoCtrlModeRequest,
)

# ---------------------------------------------------------------------------
# 右臂默认位姿（保持不动）
# ---------------------------------------------------------------------------
DEFAULT_RIGHT_EEF_POS = [0.0, -0.3, 0.0]
DEFAULT_RIGHT_EEF_QUAT = [0.0, 0.0, 0.0, 1.0]


# ===========================================================================
# FHAN 平滑器 — 移植自 leju_utils/math.hpp fhan / fhanStepForward
# ===========================================================================

def _sign(x):
    if -1e-6 < x < 1e-6:
        return 0.0
    return 1.0 if x >= 1e-6 else -1.0


def _fsg(x, d):
    return (_sign(x + d) - _sign(x - d)) / 2.0


def _fhan(x1, x2, r, h0):
    """Han's fastest tracking differentiator — acceleration function."""
    d = r * h0 * h0
    a0 = h0 * x2
    y = x1 + a0
    a1 = math.sqrt(d * (d + 8.0 * abs(y)))
    a2 = a0 + _sign(y) * (a1 - d) / 2.0
    sy = _fsg(y, d)
    a = (a0 + y) * sy + a2 * (1.0 - sy)
    sa = _fsg(a, d)
    return -r * (a / d) * sa - r * _sign(a) * (1.0 - sa)


class FHANSmoother:
    """3-axis FHAN smoother: tracks a reference signal with bounded acceleration."""

    def __init__(self, r, h, h0):
        """
        r:  acceleration bound (larger = faster response, more aggressive)
        h:  step size (= 1/rate_hz)
        h0: smoothing factor (typically 2~10 * h)
        """
        self.r = r
        self.h = h
        self.h0 = h0
        self.x = np.zeros(3)
        self.dx = np.zeros(3)

    def step(self, ref):
        """Advance one step toward *ref* (3-vector). Returns smoothed position."""
        for i in range(3):
            fh = _fhan(self.x[i] - ref[i], self.dx[i], self.r, self.h0)
            self.x[i] = self.x[i] + self.h * self.dx[i]
            self.dx[i] = self.dx[i] + self.h * fh
        return self.x.copy()


# ===========================================================================
# Data loading & resampling
# ===========================================================================

def load_pkl(path):
    with open(path, "rb") as f:
        return pickle.load(f)


def resample_trajectory(timestamps, positions, orientations_xyzw,
                        gripper_ts=None, gripper_pct=None,
                        rate_hz=20, speed=1.0):
    t = np.array(timestamps)
    t = (t - t[0]) / speed
    pos = np.array(positions)
    quat = np.array(orientations_xyzw)

    t_end = t[-1]
    t_arr = np.arange(0, t_end, 1.0 / rate_hz)

    pos_arr = np.column_stack([np.interp(t_arr, t, pos[:, i]) for i in range(3)])

    cum_rots = Rot.from_quat(quat)
    slerp = Slerp(t, cum_rots)
    t_clip = np.clip(t_arr, t[0] + 1e-9, t[-1] - 1e-9)
    quat_arr = slerp(t_clip).as_quat()

    grip_arr = None
    if gripper_ts is not None and gripper_pct is not None:
        gt = np.array(gripper_ts)
        gt = (gt - gt[0]) / speed
        gp = np.array(gripper_pct)
        grip_arr = np.interp(t_arr, gt, gp)

    return t_arr, pos_arr, quat_arr, grip_arr


def compute_frame_deltas(positions, orientations, R_align=None):
    """Extract per-frame position and orientation deltas.

    Returns:
        delta_pos:  (N, 3) — position increments per frame (first row is zeros)
        delta_quat: (N, 4) — quaternion increments per frame (xyzw)
    """
    pos = np.array(positions)
    quat = np.array(orientations)

    dp = np.diff(pos, axis=0)
    if R_align is not None:
        dp = (R_align @ dp.T).T
    dp = np.vstack([np.zeros(3), dp])

    rots = Rot.from_quat(quat)
    dq = np.tile([0.0, 0.0, 0.0, 1.0], (len(quat), 1))
    for i in range(1, len(rots)):
        dq[i] = (rots[i] * rots[i - 1].inv()).as_quat()

    return dp, dq


def get_left_eef_from_tf(timeout=3.0):
    """通过 TF 查询机器人当前左臂末端位姿 (base_link → zarm_l7_link)。"""
    buf = tf2_ros.Buffer()
    _ = tf2_ros.TransformListener(buf)
    rospy.sleep(0.5)

    try:
        t = buf.lookup_transform('base_link', 'zarm_l7_link',
                                 rospy.Time(0), rospy.Duration(timeout))
        pos = np.array([t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z])
        quat = np.array([t.transform.rotation.x,
                         t.transform.rotation.y,
                         t.transform.rotation.z,
                         t.transform.rotation.w])
        return pos, quat
    except Exception as e:
        rospy.logwarn(f"TF lookup failed: {e}")
        return None, None


# ===========================================================================
# Service helpers
# ===========================================================================

def set_mpc_control_mode(mode):
    """0=NoControl, 1=ArmOnly, 2=BaseOnly, 3=BaseArm, 4=ArmEeOnly"""
    try:
        rospy.wait_for_service("/mobile_manipulator_mpc_control", timeout=5.0)
        srv = rospy.ServiceProxy("/mobile_manipulator_mpc_control",
                                 changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        req.control_mode = mode
        resp = srv(req)
        if resp.result:
            rospy.loginfo(f"MPC control mode set to {mode}")
        else:
            rospy.logwarn(f"MPC control mode switch failed: {resp.message}")
        return resp.result
    except Exception as e:
        rospy.logerr(f"MPC mode service error: {e}")
        return False


# ===========================================================================
# Message builders
# ===========================================================================

def build_mpc_msg(left_pos, left_quat):
    msg = twoArmHandPoseCmd()
    msg.hand_poses.header = Header()
    msg.hand_poses.header.stamp = rospy.Time.now()
    msg.hand_poses.header.frame_id = "base_link"

    left = armHandPose()
    left.pos_xyz = [float(v) for v in left_pos]
    left.quat_xyzw = [float(v) for v in left_quat]
    left.elbow_pos_xyz = [0.0, 0.0, 0.0]
    left.joint_angles = [0.0] * 7

    right = armHandPose()
    right.pos_xyz = list(DEFAULT_RIGHT_EEF_POS)
    right.quat_xyzw = list(DEFAULT_RIGHT_EEF_QUAT)
    right.elbow_pos_xyz = [0.0, 0.0, 0.0]
    right.joint_angles = [0.0] * 7

    msg.hand_poses.left_pose = left
    msg.hand_poses.right_pose = right

    msg.frame = 2  # local frame (base_link)
    msg.use_custom_ik_param = False
    msg.joint_angles_as_q0 = False

    return msg


def build_claw_msg(left_percent):
    msg = lejuClawCommand()
    msg.header.stamp = rospy.Time.now()
    msg.data = endEffectorData()
    msg.data.name = ["left_claw", "right_claw"]
    msg.data.position = [float(left_percent), 0.0]
    msg.data.velocity = [50.0, 50.0]
    msg.data.effort = [1.0, 1.0]
    return msg


# ===========================================================================
# Replay loops
# ===========================================================================

def replay_absolute_mpc(t_arr, pos_arr, quat_arr, grip_arr,
                        pub_arm, pub_claw, rate_hz, enable_gripper):
    """绝对式 MPC 回放（旧逻辑）"""
    rate = rospy.Rate(rate_hz)
    n = len(t_arr)
    rospy.loginfo(f"Absolute MPC replay: {n} frames, ~{t_arr[-1]:.1f}s, {rate_hz}Hz")

    for i in range(n):
        if rospy.is_shutdown():
            break
        pub_arm.publish(build_mpc_msg(pos_arr[i], quat_arr[i]))
        if enable_gripper and grip_arr is not None:
            pub_claw.publish(build_claw_msg(grip_arr[i]))
        if i % (rate_hz * 2) == 0:
            rospy.loginfo(f"  [{t_arr[i]:.1f}s] pos=({pos_arr[i][0]:.3f}, "
                          f"{pos_arr[i][1]:.3f}, {pos_arr[i][2]:.3f})")
        rate.sleep()


def replay_incremental_mpc(t_arr, delta_pos, delta_quat, grip_arr,
                           anchor_pos, anchor_quat,
                           pub_arm, pub_claw, rate_hz, enable_gripper,
                           fhan_r, fhan_h0_scale, no_orient):
    """
    增量式 MPC 回放：
      target_pos = anchor + FHAN_smooth(cumulative_delta)
      target_quat = cumulative_delta_q * anchor_quat  (或保持 anchor_quat)
    """
    rate = rospy.Rate(rate_hz)
    n = len(t_arr)
    dt = 1.0 / rate_hz
    h0 = fhan_h0_scale * dt

    smoother = FHANSmoother(r=fhan_r, h=dt, h0=h0)

    cum_delta = np.zeros(3)
    q_target = Rot.from_quat(anchor_quat)

    rospy.loginfo(f"Incremental MPC replay: {n} frames, ~{t_arr[-1]:.1f}s, {rate_hz}Hz")
    rospy.loginfo(f"  FHAN params: r={fhan_r}, h={dt:.4f}, h0={h0:.4f}")
    rospy.loginfo(f"  Anchor pos: ({anchor_pos[0]:.3f}, {anchor_pos[1]:.3f}, {anchor_pos[2]:.3f})")
    rospy.loginfo(f"  Anchor quat: ({anchor_quat[0]:.4f}, {anchor_quat[1]:.4f}, "
                  f"{anchor_quat[2]:.4f}, {anchor_quat[3]:.4f})")

    for i in range(n):
        if rospy.is_shutdown():
            break

        cum_delta += delta_pos[i]
        smoothed = smoother.step(cum_delta)
        target_pos = anchor_pos + smoothed

        if not no_orient:
            dq = Rot.from_quat(delta_quat[i])
            q_target = dq * q_target
        target_quat = q_target.as_quat()

        pub_arm.publish(build_mpc_msg(target_pos, target_quat))

        if enable_gripper and grip_arr is not None:
            pub_claw.publish(build_claw_msg(grip_arr[i]))

        log_interval = rate_hz * 2
        if i < 10 or i % log_interval == 0:
            dp_norm = np.linalg.norm(delta_pos[i])
            rospy.loginfo(f"  [{t_arr[i]:.1f}s] f={i}  target=({target_pos[0]:.3f}, "
                          f"{target_pos[1]:.3f}, {target_pos[2]:.3f})  "
                          f"cum=({cum_delta[0]:.3f}, {cum_delta[1]:.3f}, {cum_delta[2]:.3f})  "
                          f"|d|={dp_norm:.4f}")
        rate.sleep()


# ===========================================================================
# Main
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Replay UMI trajectory in MuJoCo via MPC (left arm only).")
    parser.add_argument("--pkl", required=True, help="Path to umi_in_base.pkl")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--rate", type=int, default=20,
                        help="Publish rate Hz (default: 20)")
    parser.add_argument("--no-gripper", action="store_true",
                        help="Disable gripper control")
    parser.add_argument("--no-orient", action="store_true",
                        help="Ignore orientation, keep anchor orientation throughout")

    inc = parser.add_argument_group("incremental mode (recommended)")
    inc.add_argument("--incremental", action="store_true",
                     help="Use incremental mode (frame deltas + FHAN smoothing)")
    inc.add_argument("--fhan-r", type=float, default=2.0,
                     help="FHAN acceleration bound (default: 2.0, larger=faster)")
    inc.add_argument("--fhan-h0-scale", type=float, default=5.0,
                     help="FHAN h0 = scale * dt (default: 5.0)")
    inc.add_argument("--delta-scale", type=float, default=1.0,
                     help="Scale factor for position deltas (default: 1.0)")

    args = parser.parse_args()
    rospy.init_node("umi_replay", anonymous=True)

    # --- Load data ---
    rospy.loginfo(f"Loading {args.pkl} ...")
    data = load_pkl(args.pkl)
    timestamps = data["timestamps"]
    positions = data["positions"]
    orientations = data["orientations_xyzw"]
    gripper_ts = data.get("gripper_timestamps")
    gripper_pct = data.get("gripper_percent")

    rospy.loginfo(f"  Raw frames: {len(timestamps)}, "
                  f"duration: {timestamps[-1] - timestamps[0]:.1f}s")

    # --- Resample ---
    rate_hz = args.rate
    t_arr, pos_arr, quat_arr, grip_arr = resample_trajectory(
        timestamps, positions, orientations,
        gripper_ts, gripper_pct, rate_hz, args.speed)
    rospy.loginfo(f"  Resampled: {len(t_arr)} frames at {rate_hz}Hz, "
                  f"duration: {t_arr[-1]:.1f}s (speed x{args.speed})")

    enable_gripper = (not args.no_gripper) and (grip_arr is not None)

    # --- Setup publisher ---
    pub_arm = rospy.Publisher("/mm/two_arm_hand_pose_cmd",
                              twoArmHandPoseCmd, queue_size=10)
    pub_claw = rospy.Publisher("/leju_claw_command",
                               lejuClawCommand, queue_size=10)
    rospy.sleep(0.5)

    # --- Set MPC control mode ---
    rospy.loginfo("Setting MPC control mode: ArmOnly ...")
    set_mpc_control_mode(1)
    rospy.sleep(1.0)

    # --- Dispatch ---
    try:
        if args.incremental:
            # positions in pkl are already in base_link frame (umi_in_base),
            # so frame-to-frame deltas are also in base_link — no R_align needed.
            delta_pos, delta_quat = compute_frame_deltas(
                positions, orientations, R_align=None)

            rospy.loginfo("  Deltas computed in base_link frame (no R_align)")
            cum_check = np.cumsum(delta_pos, axis=0)
            rospy.loginfo(f"  Delta cumsum range: "
                          f"X=[{cum_check[:,0].min():.3f}, {cum_check[:,0].max():.3f}]  "
                          f"Y=[{cum_check[:,1].min():.3f}, {cum_check[:,1].max():.3f}]  "
                          f"Z=[{cum_check[:,2].min():.3f}, {cum_check[:,2].max():.3f}]")

            if args.delta_scale != 1.0:
                delta_pos *= args.delta_scale
                rospy.loginfo(f"  Delta scale: {args.delta_scale}")

            total_displacement = np.sum(delta_pos, axis=0)
            rospy.loginfo(f"  Total trajectory displacement: "
                          f"({total_displacement[0]:.3f}, "
                          f"{total_displacement[1]:.3f}, "
                          f"{total_displacement[2]:.3f})")

            # Resample deltas to match t_arr rate
            t_raw = timestamps - timestamps[0]
            cum_raw = np.cumsum(delta_pos, axis=0)
            t_query = np.clip(t_arr * args.speed, t_raw[0], t_raw[-1])
            cum_resampled = np.column_stack([
                np.interp(t_query, t_raw, cum_raw[:, i]) for i in range(3)
            ])
            delta_resampled = np.vstack([np.zeros(3), np.diff(cum_resampled, axis=0)])

            # Resample orientation deltas via cumulative rotation
            cum_quat_raw = [Rot.identity()]
            for i in range(1, len(delta_quat)):
                cum_quat_raw.append(Rot.from_quat(delta_quat[i]) * cum_quat_raw[-1])
            cum_rots_raw = Rot.concatenate(cum_quat_raw)
            slerp_q = Slerp(t_raw, cum_rots_raw)
            t_query_clip = np.clip(t_query, t_raw[0] + 1e-9, t_raw[-1] - 1e-9)
            cum_rots_resampled = slerp_q(t_query_clip)
            delta_quat_resampled = np.tile([0.0, 0.0, 0.0, 1.0], (len(t_arr), 1))
            for i in range(1, len(t_arr)):
                dq = cum_rots_resampled[i] * cum_rots_resampled[i - 1].inv()
                delta_quat_resampled[i] = dq.as_quat()

            # Get robot's current EEF as anchor
            rospy.loginfo("Querying robot EEF via TF ...")
            anchor_pos, anchor_quat = get_left_eef_from_tf(timeout=3.0)
            if anchor_pos is None:
                rospy.logwarn("TF lookup failed, using fallback anchor [0, 0.3, 0.3]")
                anchor_pos = np.array([0.0, 0.3, 0.3])
                anchor_quat = np.array([0.0, 0.0, 0.0, 1.0])
            else:
                rospy.loginfo(f"  EEF anchor: ({anchor_pos[0]:.3f}, "
                              f"{anchor_pos[1]:.3f}, {anchor_pos[2]:.3f})")

            rospy.loginfo("=== Starting incremental replay ===")
            replay_incremental_mpc(
                t_arr, delta_resampled, delta_quat_resampled, grip_arr,
                anchor_pos, anchor_quat,
                pub_arm, pub_claw, rate_hz, enable_gripper,
                args.fhan_r, args.fhan_h0_scale, args.no_orient)
        else:
            rospy.loginfo("=== Starting absolute replay ===")
            if args.no_orient:
                rospy.loginfo("  --no-orient: using identity quaternion")
                quat_arr[:] = [0.0, 0.0, 0.0, 1.0]
            replay_absolute_mpc(
                t_arr, pos_arr, quat_arr, grip_arr,
                pub_arm, pub_claw, rate_hz, enable_gripper)
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("=== Replay finished ===")

    # --- Restore ---
    rospy.loginfo("Restoring control mode ...")
    set_mpc_control_mode(0)
    rospy.loginfo("Done.")


if __name__ == "__main__":
    main()


# ===========================================================================
# 使用说明
# ===========================================================================
#
# 前置条件:
#   1. 启动 MuJoCo 仿真:
#      cd /home/leju/catkin_ws/src/wheel/kuavo-ros-control
#      source devel/setup.bash
#      roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
#
#   2. 已生成 umi_in_base.pkl (由 transform_umi_to_base.py 产出)
#
# 增量式 MPC 回放（推荐，不依赖绝对坐标精度）:
#
#   cd kuavo-ros-control/src/demo/umi_replay/scripts
#   python3 replay_umi_in_mujoco.py \
#     --pkl ../data/output/umi_in_base.pkl \
#     --incremental \
#     --rate 20 --speed 0.5 \
#     --no-gripper --no-orient
#
# 绝对式 MPC 回放（旧逻辑，需要精确坐标变换）:
#
#   python3 replay_umi_in_mujoco.py \
#     --pkl ../data/output/umi_in_base.pkl \
#     --rate 20 --speed 0.5
#
# 参数说明:
#   --pkl              输入 pkl 文件路径
#   --speed            回放速度倍率 (默认 1.0, 0.5=半速)
#   --rate             发布频率 Hz (推荐 10~20)
#   --no-gripper       禁用夹爪控制
#   --no-orient        忽略姿态增量，保持锚点姿态不变
#   --incremental      启用增量式回放（推荐）
#   --fhan-r           FHAN 加速度约束 (默认 2.0, 越大越快但越生硬)
#   --fhan-h0-scale    FHAN h0 = scale * dt (默认 5.0, 越大越平滑)
#   --delta-scale      位置增量缩放倍率 (默认 1.0)
#
# 增量式原理:
#   1. 从 UMI 轨迹提取帧间位置差 delta[i] = pos[i] - pos[i-1]
#   2. 通过 TF 查询机器人当前左臂末端位姿作为锚点 (anchor)
#   3. 逐帧累加: target = anchor + FHAN_smooth(sum(delta[0..i]))
#   4. 发布 target 到 MPC，MPC 负责轨迹优化和执行
#
#   优势: 不依赖绝对坐标变换精度，只需运动方向正确
#   FHAN 平滑器: 移植自 leju_utils/math.hpp，与 VR 增量式遥操作一致
#
# ===========================================================================
