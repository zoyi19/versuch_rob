#!/usr/bin/env python3
"""
在 MuJoCo 仿真中回放 UMI 夹爪轨迹。

支持两种回放模式:
  - mpc:  发布末端位姿到 /mm/two_arm_hand_pose_cmd, 由 MPC 在线求解轨迹
  - ik:   离线 IK 预计算关节角, 通过 quickMode 直接发布到 /kuavo_arm_traj
"""

import os
import sys
import argparse
import pickle
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import (
    twoArmHandPoseCmd,
    twoArmHandPose,
    armHandPose,
    ikSolveParam,
    lejuClawCommand,
    endEffectorData,
)
from kuavo_msgs.srv import (
    changeTorsoCtrlMode,
    changeTorsoCtrlModeRequest,
    changeLbQuickModeSrv,
    changeLbQuickModeSrvRequest,
)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


# ---------------------------------------------------------------------------
# Data loading & resampling
# ---------------------------------------------------------------------------

def load_pkl(path):
    with open(path, "rb") as f:
        return pickle.load(f)


def resample_trajectory(timestamps, positions, orientations, gripper_ts, gripper_pct, rate_hz, speed):
    """Resample raw trajectory to uniform rate, accounting for speed factor."""
    t = timestamps - timestamps[0]
    duration = t[-1] / speed
    dt = 1.0 / rate_hz
    t_new = np.arange(0, duration + dt * 0.5, dt)
    t_query = t_new * speed  # map back to original time domain

    t_query = np.clip(t_query, t[0], t[-1])

    pos_new = np.column_stack([
        np.interp(t_query, t, positions[:, i]) for i in range(3)
    ])

    key_rots = Rot.from_quat(orientations)
    slerp = Slerp(t, key_rots)
    t_query_clipped = np.clip(t_query, t[0] + 1e-9, t[-1] - 1e-9)
    quat_new = slerp(t_query_clipped).as_quat()

    grip_new = None
    if gripper_ts is not None and gripper_pct is not None:
        gt = gripper_ts - timestamps[0]
        valid_mask = gripper_pct >= 0
        if valid_mask.sum() > 1:
            grip_new = np.interp(t_query, gt[valid_mask], gripper_pct[valid_mask])
            grip_new = np.clip(grip_new, 0.0, 100.0)

    return t_new, pos_new, quat_new, grip_new


# ---------------------------------------------------------------------------
# EEF offset: UMI tip -> zarm_l7_link
# ---------------------------------------------------------------------------

def apply_eef_offset(positions, orientations, offset):
    """
    UMI tracks the gripper tip (≈ zarm_l7_end_effector).
    IK expects zarm_l7_link frame.  offset = tip_in_l7 = [0, 0.03, -0.17].
    l7_pos = tip_pos + R_tip * (-offset)  (since tip has same orientation as l7)
    """
    offset = np.asarray(offset)
    out_pos = np.empty_like(positions)
    for i in range(len(positions)):
        R_mat = Rot.from_quat(orientations[i]).as_matrix()
        out_pos[i] = positions[i] + R_mat @ (-offset)
    return out_pos


# ---------------------------------------------------------------------------
# IK batch computation
# ---------------------------------------------------------------------------

def compute_ik_batch(positions, orientations, model_type):
    from kuavo_ik.ik_library import IKAnalytical

    n = len(positions)
    joint_angles = np.zeros((n, 14))
    prev_valid = np.zeros(7)
    fail_count = 0

    for i in range(n):
        try:
            q7 = IKAnalytical.compute(
                eef_pos=positions[i],
                eef_quat_xyzw=orientations[i],
                eef_frame="zarm_l7_link",
                model_type=model_type,
                limit=True,
            )
            joint_angles[i, :7] = q7
            prev_valid = q7.copy()
        except Exception as e:
            joint_angles[i, :7] = prev_valid
            fail_count += 1
            if fail_count <= 5:
                rospy.logwarn(f"IK failed at frame {i}: {e}")

    if fail_count > 0:
        rospy.logwarn(f"IK total failures: {fail_count}/{n} frames")
    else:
        rospy.loginfo(f"IK solved all {n} frames successfully")

    return joint_angles


# ---------------------------------------------------------------------------
# Service helpers
# ---------------------------------------------------------------------------

def set_mpc_control_mode(mode):
    """0=NoControl, 1=ArmOnly, 2=BaseOnly, 3=BaseArm, 4=ArmEeOnly"""
    try:
        rospy.wait_for_service("/mobile_manipulator_mpc_control", timeout=5.0)
        srv = rospy.ServiceProxy("/mobile_manipulator_mpc_control", changeTorsoCtrlMode)
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


def set_quick_mode(mode):
    """0=off, 1=legs, 2=arms, 3=both"""
    try:
        rospy.wait_for_service("/enable_lb_arm_quick_mode", timeout=5.0)
        srv = rospy.ServiceProxy("/enable_lb_arm_quick_mode", changeLbQuickModeSrv)
        req = changeLbQuickModeSrvRequest()
        req.quickMode = mode
        resp = srv(req)
        if resp.success:
            rospy.loginfo(f"Quick mode set to {mode}")
        else:
            rospy.logwarn(f"Quick mode switch failed: {resp.message}")
        return resp.success
    except Exception as e:
        rospy.logerr(f"Quick mode service error: {e}")
        return False


# ---------------------------------------------------------------------------
# Message builders
# ---------------------------------------------------------------------------

def build_mpc_msg(pos, quat_xyzw):
    msg = twoArmHandPoseCmd()
    msg.hand_poses.header = Header()
    msg.hand_poses.header.stamp = rospy.Time.now()
    msg.hand_poses.header.frame_id = "base_link"

    left = armHandPose()
    left.pos_xyz = pos.tolist()
    left.quat_xyzw = quat_xyzw.tolist()
    left.elbow_pos_xyz = [0.0, 0.0, 0.0]
    left.joint_angles = [0.0] * 7

    right = armHandPose()
    right.pos_xyz = [0.0, 0.0, 0.0]
    right.quat_xyzw = [0.0, 0.0, 0.0, 1.0]
    right.elbow_pos_xyz = [0.0, 0.0, 0.0]
    right.joint_angles = [0.0] * 7

    msg.hand_poses.left_pose = left
    msg.hand_poses.right_pose = right

    msg.frame = 2  # local frame (base_link)
    msg.use_custom_ik_param = False
    msg.joint_angles_as_q0 = False

    return msg


def build_joint_msg(joint_angles_14):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = [f"joint{i+1}" for i in range(14)]
    msg.position = joint_angles_14.tolist()
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


# ---------------------------------------------------------------------------
# Replay loop
# ---------------------------------------------------------------------------

def replay_mpc(t_arr, pos_arr, quat_arr, grip_arr, pub_arm, pub_claw, rate_hz, enable_gripper):
    rate = rospy.Rate(rate_hz)
    n = len(t_arr)
    rospy.loginfo(f"MPC replay: {n} frames, ~{t_arr[-1]:.1f}s, {rate_hz}Hz")

    for i in range(n):
        if rospy.is_shutdown():
            break
        pub_arm.publish(build_mpc_msg(pos_arr[i], quat_arr[i]))
        if enable_gripper and grip_arr is not None:
            pub_claw.publish(build_claw_msg(grip_arr[i]))
        if i % (rate_hz * 2) == 0:
            elapsed = t_arr[i]
            rospy.loginfo(f"  [{elapsed:.1f}s] pos=({pos_arr[i][0]:.3f}, {pos_arr[i][1]:.3f}, {pos_arr[i][2]:.3f})")
        rate.sleep()


def replay_ik(t_arr, joint_arr, grip_arr, pub_arm, pub_claw, rate_hz, enable_gripper):
    rate = rospy.Rate(rate_hz)
    n = len(t_arr)
    rospy.loginfo(f"IK replay: {n} frames, ~{t_arr[-1]:.1f}s, {rate_hz}Hz")

    for i in range(n):
        if rospy.is_shutdown():
            break
        pub_arm.publish(build_joint_msg(joint_arr[i]))
        if enable_gripper and grip_arr is not None:
            pub_claw.publish(build_claw_msg(grip_arr[i]))
        if i % (rate_hz * 2) == 0:
            elapsed = t_arr[i]
            left_deg = np.degrees(joint_arr[i, :7])
            rospy.loginfo(f"  [{elapsed:.1f}s] L-arm(deg): [{', '.join(f'{a:.1f}' for a in left_deg)}]")
        rate.sleep()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Replay UMI trajectory in MuJoCo simulation.")
    parser.add_argument("--pkl", required=True, help="Path to umi_in_base.pkl")
    parser.add_argument("--mode", required=True, choices=["mpc", "ik"], help="Replay mode: mpc or ik")
    parser.add_argument("--model-type", default="60", help="Robot model type for IK (default: 60)")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")
    parser.add_argument("--rate", type=int, default=20, help="Publish rate Hz (default: 20, ik recommended: 50)")
    parser.add_argument("--eef-offset", type=float, nargs=3, default=[0, 0.03, -0.17],
                        metavar=("X", "Y", "Z"),
                        help="Offset from UMI tip to zarm_l7_link in tip frame (default: 0 0.03 -0.17)")
    parser.add_argument("--no-gripper", action="store_true", help="Disable gripper control")
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

    rospy.loginfo(f"  Raw frames: {len(timestamps)}, duration: {timestamps[-1]-timestamps[0]:.1f}s")

    # --- Resample ---
    rate_hz = args.rate
    if args.mode == "ik" and rate_hz < 30:
        rospy.logwarn(f"IK mode with low rate ({rate_hz}Hz). Consider --rate 50.")

    t_arr, pos_arr, quat_arr, grip_arr = resample_trajectory(
        timestamps, positions, orientations, gripper_ts, gripper_pct, rate_hz, args.speed
    )
    rospy.loginfo(f"  Resampled: {len(t_arr)} frames at {rate_hz}Hz, duration: {t_arr[-1]:.1f}s (speed x{args.speed})")

    enable_gripper = (not args.no_gripper) and (grip_arr is not None)
    if args.no_gripper:
        rospy.loginfo("  Gripper control disabled by --no-gripper")
    elif grip_arr is None:
        rospy.loginfo("  No gripper data in pkl, gripper control disabled")

    # --- IK precompute ---
    joint_arr = None
    if args.mode == "ik":
        rospy.loginfo("Applying EEF offset for IK ...")
        ik_pos = apply_eef_offset(pos_arr, quat_arr, args.eef_offset)
        rospy.loginfo(f"Computing IK for {len(ik_pos)} frames (model_type={args.model_type}) ...")
        joint_arr = compute_ik_batch(ik_pos, quat_arr, args.model_type)
        rospy.loginfo("IK precomputation done.")

    # --- Setup publishers ---
    pub_claw = rospy.Publisher("/leju_claw_command", lejuClawCommand, queue_size=10)

    if args.mode == "mpc":
        pub_arm = rospy.Publisher("/mm/two_arm_hand_pose_cmd", twoArmHandPoseCmd, queue_size=10)
    else:
        pub_arm = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

    rospy.sleep(0.5)

    # --- Set control mode ---
    rospy.loginfo("Setting control mode ...")
    if args.mode == "mpc":
        set_mpc_control_mode(1)  # ArmOnly
    else:
        set_quick_mode(2)  # arm quick mode

    rospy.sleep(1.0)

    # --- Replay ---
    rospy.loginfo("=== Starting replay ===")
    try:
        if args.mode == "mpc":
            replay_mpc(t_arr, pos_arr, quat_arr, grip_arr, pub_arm, pub_claw, rate_hz, enable_gripper)
        else:
            replay_ik(t_arr, joint_arr, grip_arr, pub_arm, pub_claw, rate_hz, enable_gripper)
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("=== Replay finished ===")

    # --- Restore control mode ---
    rospy.loginfo("Restoring control mode ...")
    if args.mode == "mpc":
        set_mpc_control_mode(0)
    else:
        set_quick_mode(0)

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
# MPC 模式回放 (推荐先用此模式测试, 有内置安全约束):
#
#   cd /home/leju/catkin_ws/src/wheel/kuavo-ros-control/src/demo/umi_replay/scripts
#   python3 replay_umi_in_mujoco.py \
#     --pkl ../data/output/umi_in_base.pkl \
#     --mode mpc \
#     --rate 20 \
#     --speed 0.5
#
# IK 模式回放 (直接关节控制, 更精确的轨迹跟踪):
#
#   python3 replay_umi_in_mujoco.py \
#     --pkl ../data/output/umi_in_base.pkl \
#     --mode ik \
#     --model-type 60 \
#     --rate 50 \
#     --speed 0.5
#
# 参数说明:
#   --pkl          输入 pkl 文件路径
#   --mode         回放模式: mpc (末端位姿控制) 或 ik (关节角直控)
#   --model-type   IK 机器人型号: 45, 46, 60 (默认 60, 对应 S60/S61 轮式机器人)
#   --speed        回放速度倍率 (默认 1.0, 0.5=半速)
#   --rate         发布频率 Hz (mpc 推荐 10~20, ik 推荐 50)
#   --eef-offset   UMI 末端到 zarm_l7_link 的偏移 (默认 0 0.03 -0.17)
#   --no-gripper   禁用夹爪控制
#
# ===========================================================================
