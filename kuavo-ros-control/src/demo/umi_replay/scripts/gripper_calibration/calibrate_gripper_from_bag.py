#!/usr/bin/env python3
"""
从 ROS bag 的 D405 图像中标定夹爪开合范围。

通过检测夹爪两个手指上的 ArUco 标签（Tag 0 / Tag 1），
测量标签间距随夹爪开合的变化，输出 gripper_range.json。

用法:
    python scripts/calibrate_gripper_from_bag.py \
        --bag data/0312/2026-03-12-18-17-28.bag \
        --output gripper_range.json

    # 自定义内参
    python scripts/calibrate_gripper_from_bag.py \
        --bag data/0312/2026-03-12-18-17-28.bag \
        --output gripper_range.json \
        --fx 430 --fy 430 --cx 424 --cy 240

    # 可视化宽度曲线
    python scripts/calibrate_gripper_from_bag.py \
        --bag data/0312/2026-03-12-18-17-28.bag \
        --output gripper_range.json --plot
"""

import sys
import os
import collections
import click
import json
import numpy as np
import cv2
import rosbag


def get_gripper_width(tag_dict, left_id, right_id, nominal_z=0.072, z_tolerance=0.008):
    """根据左右手指 ArUco 标签的 tvec 计算夹爪宽度 (米)。"""
    zmax = nominal_z + z_tolerance
    zmin = nominal_z - z_tolerance

    left_x = None
    if left_id in tag_dict:
        tvec = tag_dict[left_id]['tvec']
        if zmin < tvec[-1] < zmax:
            left_x = tvec[0]

    right_x = None
    if right_id in tag_dict:
        tvec = tag_dict[right_id]['tvec']
        if zmin < tvec[-1] < zmax:
            right_x = tvec[0]

    width = None
    if (left_x is not None) and (right_x is not None):
        width = right_x - left_x
    elif left_x is not None:
        width = abs(left_x) * 2
    elif right_x is not None:
        width = abs(right_x) * 2
    return width


TOPIC_D405 = '/camera/color/image_raw'
ARUCO_DICT_NAME = cv2.aruco.DICT_4X4_50
MARKER_SIZE_M = 0.016  # 16mm finger markers
TAG_PER_GRIPPER = 6


def detect_tags_in_frame(img, aruco_dict, param, K, D, marker_size):
    """检测单帧中的 ArUco 标签并估计 3D 位姿。"""
    corners, ids, _ = cv2.aruco.detectMarkers(
        image=img, dictionary=aruco_dict, parameters=param)
    if ids is None or len(ids) == 0:
        return {}
    tag_dict = {}
    for tag_id, tag_corners in zip(ids.flatten(), corners):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            tag_corners.reshape(1, 4, 2), marker_size, K, D)
        tag_dict[int(tag_id)] = {
            'rvec': rvecs.squeeze(),
            'tvec': tvecs.squeeze(),
            'corners': tag_corners.squeeze()
        }
    return tag_dict


def probe_nominal_z(bag_path, aruco_dict, param, K, D, marker_size,
                     left_id, right_id, max_probe_frames=100):
    """从前 N 帧中探测手指标签的 Z 值分布，自动确定 nominal_z。"""
    z_values = []
    bag = rosbag.Bag(str(bag_path))
    count = 0
    for topic, msg, t in bag.read_messages(topics=[TOPIC_D405]):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        tag_dict = detect_tags_in_frame(img, aruco_dict, param, K, D, marker_size)
        for tid in (left_id, right_id):
            if tid in tag_dict:
                z_values.append(tag_dict[tid]['tvec'][-1])
        count += 1
        if count >= max_probe_frames:
            break
    bag.close()

    if len(z_values) == 0:
        print("警告: 探测阶段未检测到手指标签，使用默认 nominal_z=0.094")
        return 0.094

    z_arr = np.array(z_values)
    median_z = float(np.median(z_arr))
    print(f"  Z 值探测 ({len(z_values)} 个样本, 前 {count} 帧):")
    print(f"    min={z_arr.min():.4f}m, max={z_arr.max():.4f}m, "
          f"median={median_z:.4f}m, std={z_arr.std():.4f}m")
    return median_z


@click.command()
@click.option('--bag', required=True, help='ROS bag 文件路径')
@click.option('--output', '-o', required=True, help='输出 gripper_range.json 路径')
@click.option('--topic', default=TOPIC_D405, help='D405 图像 topic')
@click.option('--fx', type=float, default=425.0, help='D405 焦距 fx')
@click.option('--fy', type=float, default=425.0, help='D405 焦距 fy')
@click.option('--cx', type=float, default=424.0, help='D405 主点 cx')
@click.option('--cy', type=float, default=240.0, help='D405 主点 cy')
@click.option('--marker-size', type=float, default=MARKER_SIZE_M,
              help='手指 ArUco 标签物理尺寸 (米)')
@click.option('--z-tolerance', type=float, default=0.015,
              help='nominal_z 过滤容差 (米)')
@click.option('--tag-det-threshold', type=float, default=0.5,
              help='最低标签检测率阈值')
@click.option('--plot', is_flag=True, help='绘制宽度随时间变化的曲线')
def main(bag, output, topic, fx, fy, cx, cy,
         marker_size, z_tolerance, tag_det_threshold, plot):

    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    D_coeff = np.zeros((1, 5), dtype=np.float64)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
    param = cv2.aruco.DetectorParameters_create()
    param.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    bag_path = os.path.expanduser(bag)
    print(f"Bag 文件: {bag_path}")
    print(f"D405 内参: fx={fx}, fy={fy}, cx={cx}, cy={cy}")
    print(f"标签尺寸: {marker_size*1000:.1f}mm")

    # ---- 第一遍：统计标签出现频率，识别夹爪 ID ----
    print("\n[阶段 1/3] 统计标签检测率...")
    tag_counts = collections.defaultdict(int)
    n_frames = 0
    bag_handle = rosbag.Bag(bag_path)
    for _topic, msg, t in bag_handle.read_messages(topics=[topic]):
        n_frames += 1
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        corners, ids, _ = cv2.aruco.detectMarkers(
            image=img, dictionary=aruco_dict, parameters=param)
        if ids is not None:
            for tag_id in ids.flatten():
                tag_counts[int(tag_id)] += 1
    bag_handle.close()

    if n_frames == 0:
        print(f"错误: 在 topic '{topic}' 中未找到图像消息")
        sys.exit(1)

    print(f"  总帧数: {n_frames}")
    for tid in sorted(tag_counts.keys()):
        rate = tag_counts[tid] / n_frames
        print(f"  Tag {tid}: {tag_counts[tid]}/{n_frames} ({rate*100:.1f}%)")

    if len(tag_counts) == 0:
        print("错误: 未检测到任何 ArUco 标签!")
        sys.exit(1)

    max_tag_id = max(tag_counts.keys())
    max_gripper_id = max_tag_id // TAG_PER_GRIPPER

    gripper_prob_map = {}
    for gid in range(max_gripper_id + 1):
        lid = gid * TAG_PER_GRIPPER
        rid = lid + 1
        left_rate = tag_counts.get(lid, 0) / n_frames
        right_rate = tag_counts.get(rid, 0) / n_frames
        prob = min(left_rate, right_rate)
        if prob > 0:
            gripper_prob_map[gid] = prob

    if not gripper_prob_map:
        print("错误: 未检测到夹爪手指标签对 (Tag 0+1 或 Tag 6+7)!")
        sys.exit(1)

    gripper_id = max(gripper_prob_map, key=gripper_prob_map.get)
    gripper_prob = gripper_prob_map[gripper_id]
    left_id = gripper_id * TAG_PER_GRIPPER
    right_id = left_id + 1
    print(f"\n  检测到夹爪 ID: {gripper_id} "
          f"(Tag {left_id}+{right_id}, 检测率 {gripper_prob*100:.1f}%)")

    if gripper_prob < tag_det_threshold:
        print(f"警告: 检测率 {gripper_prob:.2f} < 阈值 {tag_det_threshold}")
        sys.exit(1)

    # ---- 探测 nominal_z ----
    print("\n[阶段 2/3] 探测手指标签 Z 值...")
    nominal_z = probe_nominal_z(
        bag_path, aruco_dict, param, K, D_coeff, marker_size,
        left_id, right_id)
    print(f"  使用 nominal_z = {nominal_z:.4f}m, z_tolerance = {z_tolerance:.4f}m")

    # ---- 第二遍：计算夹爪宽度 ----
    print(f"\n[阶段 3/3] 计算夹爪宽度...")
    timestamps = []
    gripper_widths = []
    bag_handle = rosbag.Bag(bag_path)
    for _topic, msg, t in bag_handle.read_messages(topics=[topic]):
        ts = t.to_sec()
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        tag_dict = detect_tags_in_frame(
            img, aruco_dict, param, K, D_coeff, marker_size)
        width = get_gripper_width(
            tag_dict, left_id, right_id,
            nominal_z=nominal_z, z_tolerance=z_tolerance)
        timestamps.append(ts)
        gripper_widths.append(width if width is not None else float('nan'))
    bag_handle.close()

    gripper_widths = np.array(gripper_widths)
    valid_mask = ~np.isnan(gripper_widths)
    n_valid = int(valid_mask.sum())

    if n_valid == 0:
        print("错误: 所有帧的宽度计算均失败! 请检查 nominal_z 和 z_tolerance")
        sys.exit(1)

    max_width = float(np.nanmax(gripper_widths))
    min_width = float(np.nanmin(gripper_widths))

    print(f"  有效帧: {n_valid}/{n_frames} ({n_valid/n_frames*100:.1f}%)")
    print(f"  宽度范围: {min_width*1000:.2f}mm ~ {max_width*1000:.2f}mm")
    print(f"  行程: {(max_width - min_width)*1000:.2f}mm")

    # ---- 保存结果 ----
    result = {
        'gripper_id': int(gripper_id),
        'left_finger_tag_id': int(left_id),
        'right_finger_tag_id': int(right_id),
        'max_width': max_width,
        'min_width': min_width,
        'nominal_z': nominal_z,
        'z_tolerance': z_tolerance,
        'valid_frames': n_valid,
        'total_frames': n_frames,
        'camera_intrinsics': {
            'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
            'resolution': '848x480'
        }
    }

    output_path = os.path.expanduser(output)
    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"\n已保存: {output_path}")

    # ---- 可视化 ----
    if plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            ts_arr = np.array(timestamps)
            t_rel = ts_arr - ts_arr[0]

            fig, ax = plt.subplots(figsize=(12, 4))
            ax.plot(t_rel, gripper_widths * 1000, 'b-', linewidth=0.8,
                    label='width')
            ax.axhline(y=max_width * 1000, color='r', linestyle='--',
                       linewidth=0.8, label=f'max={max_width*1000:.1f}mm')
            ax.axhline(y=min_width * 1000, color='g', linestyle='--',
                       linewidth=0.8, label=f'min={min_width*1000:.1f}mm')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Gripper Width (mm)')
            ax.set_title('Gripper Calibration - Width over Time')
            ax.legend()
            ax.grid(True, alpha=0.3)

            plot_path = output_path.replace('.json', '_plot.png')
            fig.tight_layout()
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)
            print(f"已保存曲线图: {plot_path}")
        except ImportError:
            print("警告: matplotlib 未安装，跳过绘图")

    print("\n标定完成!")


if __name__ == "__main__":
    main()
