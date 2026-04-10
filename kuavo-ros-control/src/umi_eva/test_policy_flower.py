#!/usr/bin/env python3
"""
test_policy_flower.py — flower-in-vase 自动化批量评估（占位骨架）

pytest 入口，参数化 seed × 物体位置 × 光照，跑 N 个 episode，
输出 YAML：success_rate、completion_time、jerk、safety violations。

结构参考 src/automatic_test/automatic_test/scripts/automatic_test/test_robot_walk.py
但不依赖它，独立可跑，也不修改原文件。

前置：
  1. roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch mujoco_headless:=true
  2. rosrun umi_eva policy_runner.py --ckpt <lingbot_vla_ckpt>

状态：骨架，等 policy_runner 与 success_checker 实质实现后再填。
"""

import pytest

from success_checker import VaseGeom, CheckParams, check_instant
# from domain_randomizer import DRConfig, randomize_episode, reset_via_service


N_DEFAULT_EPISODES = 50


@pytest.mark.parametrize("episode_idx", range(N_DEFAULT_EPISODES))
def test_flower_insertion(episode_idx):
    """单个 episode：reset → 等 policy_runner 跑 → 判定 success → 记录指标。

    TODO:
        1. randomize_episode(mj_model, mj_data, dr_cfg, rng=...)
        2. reset_via_service()
        3. 等待 policy_runner 完成（监听 done topic 或超时）
        4. 调 check_instant 持续 1 s 判定
        5. 记录 success / completion_time / jerk / safety 到 YAML
    """
    pytest.skip("等 policy_runner / success_checker 实质实现")


def write_summary_yaml(results, out_path: str):
    """汇总 episode 结果到 YAML。

    TODO: success_rate、mean_completion_time、mean_jerk、safety_violations
    """
    raise NotImplementedError
