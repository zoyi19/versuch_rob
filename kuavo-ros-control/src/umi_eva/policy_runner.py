#!/usr/bin/env python3
"""
policy_runner.py — Lingbot-VLA / Pi0.5 推理桥接（占位骨架）

订阅 /wrist_cam/color/image_raw + TF（末端 pose）+ gripper width，
跑 VLA 推理，发布 twoArmHandPoseCmd (frame=2) 给现有 MuJoCo / 实机。

不修改 src/demo/umi_replay/scripts/umi_realtime_teleop.py，
通过 import 复用其 FHAN 平滑、--max-delta 裁剪、frame=2 发布逻辑。

obs:
  - /wrist_cam/color/image_raw  (D405 RGB, pinhole, 848x480 or 1280x720)
  - language instruction        ("insert the flower into the vase")
  - proprio: end-effector pose (base_link 系) + gripper width
action:
  - 未来 N 步 end-effector delta pose + gripper width

状态：占位骨架，等 Lingbot-VLA ckpt + python API 到位后填实质实现。
按 Pi0.5 风格占位；切换框架只改 load_ckpt() 与 forward() 两处。
"""

# TODO(import): import rospy, cv_bridge, tf2_ros
# TODO(import): from demo.umi_replay.scripts.umi_realtime_teleop import (
#                   FHANSmoother, publish_two_arm_cmd_frame2, clip_delta)


def load_ckpt(ckpt_path: str):
    """加载 Lingbot-VLA / Pi0.5 ckpt。

    TODO: 等用户提供 python API 后实现：
        from lingbot_vla import LingbotVLA
        return LingbotVLA.from_pretrained(ckpt_path)
    """
    raise NotImplementedError("等 Lingbot-VLA python API")


def forward(model, rgb, lang: str, proprio):
    """推理一步。

    Args:
        rgb: HxWx3 uint8，wrist D405 图像
        lang: 任务指令字符串
        proprio: dict { 'ee_pose': (7,), 'gripper_width': float }
    Returns:
        list of (delta_pose_7d, gripper_width) 共 N 步
    """
    raise NotImplementedError("等 Lingbot-VLA forward 接口")


def main():
    # TODO: argparse --ckpt --max-delta --topic-rgb --instruction
    # TODO: rospy.init_node('umi_eva_policy_runner')
    # TODO: 订阅 RGB / TF / gripper state
    # TODO: 推理循环 -> FHAN 平滑 -> clip_delta -> publish frame=2
    raise NotImplementedError


if __name__ == "__main__":
    main()
