#!/usr/bin/env python3
"""
将 UMI 夹爪末端位姿从 mocap_frame 坐标系变换到机器人 base_link 坐标系。

变换原理:
  Orbbec 相机固定在机器人头部，mocap 系统同时跟踪 orbbec 和 UMI 夹爪。
  以 zhead_2_link 作为桥接:
    T_base_umi(t) = T_base_head * inv(T_mocap_orbbec) * T_mocap_umi(t)
"""

import os
import sys
import argparse
import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    import rosbag
except ImportError:
    sys.exit("rosbag not found. Run inside a ROS environment or install with: pip install rosbag")


# ---------------------------------------------------------------------------
# TF chain from base_link to head (default for kuavo wheel robot)
# ---------------------------------------------------------------------------
DEFAULT_TF_CHAIN = [
    ("base_link", "knee_link"),
    ("knee_link", "leg_link"),
    ("leg_link", "waist_link"),
    ("waist_link", "waist_yaw_link"),
    ("waist_yaw_link", "zhead_1_link"),
    ("zhead_1_link", "zhead_2_link"),
]


def make_transform(pos, quat_xyzw):
    """从位置和四元数(xyzw)构建 4x4 齐次变换矩阵."""
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = pos
    return T


def decompose_transform(T):
    """将 4x4 齐次变换矩阵分解为位置和四元数(xyzw)."""
    pos = T[:3, 3].copy()
    quat_xyzw = R.from_matrix(T[:3, :3]).as_quat()
    return pos, quat_xyzw


# ---------------------------------------------------------------------------
# Step 1: 从 robot bag 的 /tf 中读取 TF 链，计算 T_base_head
# ---------------------------------------------------------------------------
def compute_T_base_head(robot_bag_path, head_frame="zhead_2_link"):
    """读取 robot bag 的 /tf, 按链式乘法计算 base_link -> head_frame."""

    # 根据 head_frame 确定链终点
    chain = list(DEFAULT_TF_CHAIN)
    if head_frame == "zhead_1_link":
        chain = chain[:-1]  # 去掉最后一级

    bag = rosbag.Bag(robot_bag_path, "r")
    first_tfs = {}
    for _, msg, _ in bag.read_messages(topics=["/tf"]):
        for tf in msg.transforms:
            key = (tf.header.frame_id, tf.child_frame_id)
            if key not in first_tfs:
                p = tf.transform.translation
                o = tf.transform.rotation
                first_tfs[key] = {
                    "pos": np.array([p.x, p.y, p.z]),
                    "quat": np.array([o.x, o.y, o.z, o.w]),
                }
        if len(first_tfs) >= 30:
            break
    bag.close()

    T = np.eye(4)
    for parent, child in chain:
        key = (parent, child)
        if key not in first_tfs:
            raise RuntimeError(f"TF not found in robot bag: {parent} -> {child}")
        tf = first_tfs[key]
        T = T @ make_transform(tf["pos"], tf["quat"])

    return T


# ---------------------------------------------------------------------------
# Step 2: 从 umi bag 的 /orbbec_odom 中读取 T_mocap_orbbec (固定)
# ---------------------------------------------------------------------------
def compute_T_mocap_orbbec(umi_bag_path, orbbec_topic="/orbbec_odom"):
    """读取 umi bag 的 orbbec odom, 取首帧作为 T_mocap_orbbec."""
    bag = rosbag.Bag(umi_bag_path, "r")
    for _, msg, _ in bag.read_messages(topics=[orbbec_topic]):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        pos = np.array([p.x, p.y, p.z])
        quat = np.array([o.x, o.y, o.z, o.w])
        bag.close()
        return make_transform(pos, quat)
    bag.close()
    raise RuntimeError(f"No messages found on topic {orbbec_topic}")


# ---------------------------------------------------------------------------
# Step 3: 读取 /umi_odom 并逐帧变换
# ---------------------------------------------------------------------------
def transform_umi_trajectory(umi_bag_path, T_base_mocap, umi_topic="/umi_odom"):
    """读取 /umi_odom 并用 T_base_mocap 变换到 base_link 坐标系."""
    bag = rosbag.Bag(umi_bag_path, "r")

    timestamps = []
    positions = []
    orientations = []

    for _, msg, t in bag.read_messages(topics=[umi_topic]):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        T_mocap_umi = make_transform(
            np.array([p.x, p.y, p.z]),
            np.array([o.x, o.y, o.z, o.w]),
        )
        T_base_umi = T_base_mocap @ T_mocap_umi
        pos, quat = decompose_transform(T_base_umi)

        timestamps.append(t.to_sec())
        positions.append(pos)
        orientations.append(quat)

    bag.close()

    if len(timestamps) == 0:
        raise RuntimeError(f"No messages found on topic {umi_topic}")

    return (
        np.array(timestamps),
        np.array(positions),
        np.array(orientations),
    )


# ---------------------------------------------------------------------------
# Step 4 (可选): 从 D405 图像中提取夹爪开度百分比
# ---------------------------------------------------------------------------
def extract_gripper_percent(umi_bag_path, cal_path, image_topic="/camera/color/image_raw"):
    """使用 gripper_calibration 从 bag 图像中提取夹爪开度."""
    try:
        import cv2
        import json
    except ImportError:
        print("[WARN] cv2 not available, skipping gripper extraction")
        return None, None

    if not os.path.isfile(cal_path):
        print(f"[WARN] Calibration file not found: {cal_path}, skipping gripper extraction")
        return None, None

    with open(cal_path, "r") as f:
        cal = json.load(f)

    left_id = int(cal["left_finger_tag_id"])
    right_id = int(cal["right_finger_tag_id"])
    min_w = float(cal["min_width"])
    max_w = float(cal["max_width"])
    nom_z = float(cal["nominal_z"])
    z_tol = float(cal["z_tolerance"])
    w_range = max_w - min_w

    intr = cal.get("camera_intrinsics", {})
    fx = float(intr.get("fx", 425.0))
    fy = float(intr.get("fy", 425.0))
    cx = float(intr.get("cx", 424.0))
    cy = float(intr.get("cy", 240.0))
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.zeros((1, 5), dtype=np.float64)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_param = cv2.aruco.DetectorParameters_create()
    aruco_param.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    marker_size = 0.016

    bag = rosbag.Bag(umi_bag_path, "r")
    g_timestamps = []
    g_percents = []

    for _, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        except Exception:
            g_timestamps.append(t.to_sec())
            g_percents.append(-1.0)
            continue

        corners, ids, _ = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_param)
        if ids is None:
            g_timestamps.append(t.to_sec())
            g_percents.append(-1.0)
            continue

        tag_dict = {}
        for tag_id, tag_corners in zip(ids.flatten(), corners):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                tag_corners.reshape(1, 4, 2), marker_size, K, D
            )
            tag_dict[int(tag_id)] = {"tvec": tvecs.squeeze()}

        width = _get_gripper_width(tag_dict, left_id, right_id, nom_z, z_tol)
        if width is None:
            g_timestamps.append(t.to_sec())
            g_percents.append(-1.0)
        else:
            pct = float(np.clip((width - min_w) / w_range * 100.0, 0.0, 100.0))
            g_timestamps.append(t.to_sec())
            g_percents.append(pct)

    bag.close()
    return np.array(g_timestamps), np.array(g_percents)


def _get_gripper_width(tag_dict, left_id, right_id, nominal_z, z_tolerance):
    zmax = nominal_z + z_tolerance
    zmin = nominal_z - z_tolerance
    left_x = right_x = None
    if left_id in tag_dict:
        tvec = tag_dict[left_id]["tvec"]
        if zmin < tvec[-1] < zmax:
            left_x = tvec[0]
    if right_id in tag_dict:
        tvec = tag_dict[right_id]["tvec"]
        if zmin < tvec[-1] < zmax:
            right_x = tvec[0]
    if left_x is not None and right_x is not None:
        return right_x - left_x
    if left_x is not None:
        return abs(left_x) * 2
    if right_x is not None:
        return abs(right_x) * 2
    return None


# ---------------------------------------------------------------------------
# 可视化
# ---------------------------------------------------------------------------
def plot_trajectory(positions, orientations, title="UMI Trajectory in base_link"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ImportError:
        print("[WARN] matplotlib not available, skipping plot")
        return

    fig = plt.figure(figsize=(12, 5))

    ax1 = fig.add_subplot(121, projection="3d")
    ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], "b-", linewidth=0.8)
    ax1.scatter(*positions[0], color="g", s=60, label="start")
    ax1.scatter(*positions[-1], color="r", s=60, label="end")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.set_title(title)
    ax1.legend()

    ax2 = fig.add_subplot(122)
    labels = ["X", "Y", "Z"]
    for i, lbl in enumerate(labels):
        ax2.plot(positions[:, i], label=lbl)
    ax2.set_xlabel("Frame")
    ax2.set_ylabel("Position (m)")
    ax2.set_title("Position over time")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    plot_path = "umi_trajectory_base_link.png"
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved: {plot_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Transform UMI gripper poses from mocap_frame to robot base_link."
    )
    parser.add_argument("--umi-bag", required=True, help="Path to UMI rosbag (contains /umi_odom, /orbbec_odom)")
    parser.add_argument("--robot-bag", required=True, help="Path to robot rosbag (contains /tf with base_link chain)")
    parser.add_argument("-o", "--output", required=True, help="Output .pkl file path")
    parser.add_argument("--head-frame", default="zhead_2_link", help="Head TF frame name (default: zhead_2_link)")
    parser.add_argument("--gripper-cal", default=None, help="Path to gripper_range.json (optional, for gripper percent)")
    parser.add_argument("--plot", action="store_true", help="Generate trajectory visualization plot")
    args = parser.parse_args()

    # Step 1: T_base_head from robot bag
    print("[1/4] Computing T_base_head from robot bag TF chain...")
    T_base_head = compute_T_base_head(args.robot_bag, head_frame=args.head_frame)
    head_pos, head_quat = decompose_transform(T_base_head)
    head_euler = R.from_quat(head_quat).as_euler("xyz", degrees=True)
    print(f"  T_base_{args.head_frame}:")
    print(f"    pos  = ({head_pos[0]:.4f}, {head_pos[1]:.4f}, {head_pos[2]:.4f})")
    print(f"    euler = ({head_euler[0]:.1f}, {head_euler[1]:.1f}, {head_euler[2]:.1f}) deg")

    # Step 2: T_mocap_orbbec from umi bag
    print("[2/4] Computing T_mocap_orbbec from UMI bag...")
    T_mocap_orbbec = compute_T_mocap_orbbec(args.umi_bag)
    orbbec_pos, orbbec_quat = decompose_transform(T_mocap_orbbec)
    orbbec_euler = R.from_quat(orbbec_quat).as_euler("xyz", degrees=True)
    print(f"  T_mocap_orbbec:")
    print(f"    pos  = ({orbbec_pos[0]:.4f}, {orbbec_pos[1]:.4f}, {orbbec_pos[2]:.4f})")
    print(f"    euler = ({orbbec_euler[0]:.1f}, {orbbec_euler[1]:.1f}, {orbbec_euler[2]:.1f}) deg")

    # Bridge transform
    T_base_mocap = T_base_head @ np.linalg.inv(T_mocap_orbbec)
    print("  Bridge T_base_mocap computed.")

    # Step 3: Transform UMI trajectory
    print("[3/4] Transforming UMI trajectory to base_link...")
    timestamps, positions, orientations = transform_umi_trajectory(args.umi_bag, T_base_mocap)
    print(f"  Transformed {len(timestamps)} frames.")
    print(f"  Position range in base_link:")
    print(f"    X: [{positions[:, 0].min():.4f}, {positions[:, 0].max():.4f}]")
    print(f"    Y: [{positions[:, 1].min():.4f}, {positions[:, 1].max():.4f}]")
    print(f"    Z: [{positions[:, 2].min():.4f}, {positions[:, 2].max():.4f}]")

    # Step 4: Gripper percent (optional)
    gripper_ts = None
    gripper_pct = None
    if args.gripper_cal:
        print("[3.5/4] Extracting gripper opening percent from images...")
        gripper_ts, gripper_pct = extract_gripper_percent(args.umi_bag, args.gripper_cal)
        if gripper_pct is not None:
            valid = gripper_pct[gripper_pct >= 0]
            print(f"  Extracted {len(gripper_pct)} frames, {len(valid)} valid.")
            if len(valid) > 0:
                print(f"  Gripper range: {valid.min():.1f}% ~ {valid.max():.1f}%")

    # Save output
    print(f"[4/4] Saving to {args.output} ...")
    result = {
        "timestamps": timestamps,
        "positions": positions,
        "orientations_xyzw": orientations,
        "T_base_mocap": T_base_mocap,
        "T_base_head": T_base_head,
        "T_mocap_orbbec": T_mocap_orbbec,
        "head_frame": args.head_frame,
        "umi_bag": os.path.abspath(args.umi_bag),
        "robot_bag": os.path.abspath(args.robot_bag),
    }
    if gripper_ts is not None:
        result["gripper_timestamps"] = gripper_ts
        result["gripper_percent"] = gripper_pct

    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "wb") as f:
        pickle.dump(result, f)
    print(f"  Saved. Keys: {list(result.keys())}")

    if args.plot:
        plot_trajectory(positions, orientations)

    print("\nDone!")


if __name__ == "__main__":
    main()


# ===========================================================================
# 使用说明
# ===========================================================================
#
# 1. 环境要求:
#    - Python 3, 在 ROS 环境中运行 (需要 rosbag)
#    - pip 依赖: numpy, scipy
#    - 可选: opencv-python (opencv-contrib-python, 用于夹爪开度提取), matplotlib (用于 --plot)
#
# 2. 基本用法 (仅坐标变换):
#
#    python3 transform_umi_to_base.py \
#      --umi-bag ../data/umi_bags/2026-03-12-18-17-52.bag \
#      --robot-bag ../data/robot_bags/episode_67_unpack_left_width_40_green.bag \
#      -o output/umi_in_base.pkl
#
# 3. 带夹爪开度提取和可视化:
#
#    python3 transform_umi_to_base.py \
#      --umi-bag ../data/umi_bags/2026-03-12-18-17-52.bag \
#      --robot-bag ../data/robot_bags/episode_67_unpack_left_width_40_green.bag \
#      -o output/umi_in_base.pkl \
#      --gripper-cal gripper_calibration/gripper_range.json \
#      --plot
#
# 4. 指定头部坐标系 (如果 orbbec 对应 zhead_1_link 而非 zhead_2_link):
#
#    python3 transform_umi_to_base.py \
#      --umi-bag ... --robot-bag ... -o ... \
#      --head-frame zhead_1_link
#
# 5. 输出文件 (.pkl) 内容:
#
#    import pickle
#    with open("output/umi_in_base.pkl", "rb") as f:
#        data = pickle.load(f)
#
#    data["timestamps"]          # (N,)   时间戳 (float, seconds)
#    data["positions"]           # (N, 3) base_link 下的位置 [x, y, z] (米)
#    data["orientations_xyzw"]   # (N, 4) base_link 下的四元数 [qx, qy, qz, qw]
#    data["T_base_mocap"]        # (4, 4) 桥接变换矩阵
#    data["T_base_head"]         # (4, 4) base_link -> head 变换
#    data["T_mocap_orbbec"]      # (4, 4) mocap -> orbbec 变换
#    # 以下仅在指定 --gripper-cal 时存在:
#    data["gripper_timestamps"]  # (M,)   夹爪检测时间戳
#    data["gripper_percent"]     # (M,)   夹爪开度 0~100%, -1 为检测失败
#
# 6. 坐标变换原理:
#
#    Orbbec 相机固定在机器人头部 (zhead_2_link), mocap 系统同时跟踪
#    orbbec 和 UMI 夹爪, 两者在 mocap_frame 下有已知位姿.
#    Robot bag 提供 base_link -> zhead_2_link 的 TF 链.
#    桥接公式:
#      T_base_mocap = T_base_head * inv(T_mocap_orbbec)
#      T_base_umi(t) = T_base_mocap * T_mocap_umi(t)
#
# 7. 后续步骤:
#    拿到 base_link 下的末端位姿后, 可以:
#    (a) 用 kuavo_ik 做逆运动学得到关节角, 然后通过 /kuavo_arm_traj 驱动 MuJoCo
#    (b) 发送末端位姿到 /mm/two_arm_hand_pose_cmd, 由 MPC 在线求解轨迹
#
# ===========================================================================
