#!/usr/bin/env python3
"""
success_checker.py — flower-in-vase 成功判定（占位骨架）

四条件同时满足并持续 1 s 即算成功：
  1. 花头 z > 瓶口 z
  2. 花茎底端 z < 瓶口 z
  3. 花茎底端 (x, y) 距瓶口中心水平距离 < 瓶口内半径
  4. gripper width > 松开阈值（默认 0.06 m）

提供两种调用方式：
  - 纯函数 check(flower_pose, vase_pose, gripper_width, params) -> bool
  - ROS 节点（订阅 TF + /leju_claw_state，自动持续 1 s 判定）

状态：骨架，等场景几何最终确定后填阈值。
"""

from dataclasses import dataclass


@dataclass
class VaseGeom:
    rim_z: float = 0.87       # 瓶口绝对高度，TODO 与 flower_scene.xml 一致
    inner_radius: float = 0.03
    center_xy: tuple = (0.4, 0.0)


@dataclass
class CheckParams:
    gripper_release_threshold: float = 0.06
    hold_seconds: float = 1.0


def check_instant(flower_head_xyz, flower_stem_bottom_xyz,
                  gripper_width: float, vase: VaseGeom,
                  params: CheckParams) -> bool:
    """单帧四条件判定。"""
    head_above = flower_head_xyz[2] > vase.rim_z
    bottom_below = flower_stem_bottom_xyz[2] < vase.rim_z
    dx = flower_stem_bottom_xyz[0] - vase.center_xy[0]
    dy = flower_stem_bottom_xyz[1] - vase.center_xy[1]
    bottom_inside = (dx * dx + dy * dy) ** 0.5 < vase.inner_radius
    gripper_open = gripper_width > params.gripper_release_threshold
    return head_above and bottom_below and bottom_inside and gripper_open


# TODO: ROS 节点
#   - 订阅 TF 拿 flower body / vase body 的 pose
#   - 订阅 /leju_claw_state 拿 gripper width
#   - 维护一个连续命中起始时间戳，>= hold_seconds 即触发 success
def run_ros_node():
    raise NotImplementedError
