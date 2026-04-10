#!/usr/bin/env python3
"""
domain_randomizer.py — 中度 DR + reset（占位骨架）

每个 episode reset 时随机：
  - 花瓶在桌面 xy ±5 cm，yaw ±30°
  - 花的初始 pose（桌面另一侧 xy ±8 cm，yaw 任意）
  - 光源方向 + 强度
  - 桌布、瓶身、背景的纹理/材质切换（MuJoCo material 列表）

不随机：
  - 相机外参（破坏 sim2real 视觉一致性）
  - 花瓶尺寸（破坏 success 判据）
  - 花的几何参数

实现方式：通过 mujoco python API 直接改 mjModel/mjData 字段，
然后调 reset service，避免每次重启 launch。
"""

from dataclasses import dataclass, field
from typing import List


@dataclass
class DRConfig:
    vase_xy_range: float = 0.05
    vase_yaw_range_deg: float = 30.0
    flower_xy_range: float = 0.08
    flower_yaw_full: bool = True
    light_intensity_range: tuple = (0.6, 1.4)
    light_dir_jitter_deg: float = 30.0
    vase_materials: List[str] = field(default_factory=lambda: [
        "vase_red", "vase_blue", "vase_white", "vase_ceramic"
    ])
    table_materials: List[str] = field(default_factory=lambda: [
        "table_wood", "table_cloth_a", "table_cloth_b"
    ])
    background_materials: List[str] = field(default_factory=lambda: [
        "bg_lab", "bg_office", "bg_plain"
    ])


def randomize_episode(mj_model, mj_data, cfg: DRConfig, rng):
    """对当前 mj_model/mj_data 应用一次随机化。

    TODO:
        - 改 vase body qpos / 初始 freejoint 位姿
        - 改 flower freejoint qpos
        - 改 mj_model.light_pos / light_dir / light_diffuse
        - 切换 geom material id（mj_model.geom_matid[...]）
    """
    raise NotImplementedError


def reset_via_service():
    """调 ROS reset service，重置仿真而不重启 launch。

    TODO: 用现有 mujoco_ros 的 reset service 名（确认后填）。
    """
    raise NotImplementedError
