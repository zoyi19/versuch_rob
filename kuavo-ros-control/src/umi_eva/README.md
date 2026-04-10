# umi_eva — 插花任务的仿真评估工作区

本目录是 "UMI 采数 → Lingbot-VLA 训练 → MuJoCo 仿真评估" 链路在
**flower-in-vase** 任务上的独立工作区。**不修改任何已有文件**，所有
新代码与场景集中在这里，回滚只需删本目录。

详细背景与决策见 `../../eva_plan.md` 的 `## 插花任务特化（flower-in-vase）` 一节。

## 文件索引
- `scenes/flower_scene.xml` —— MJCF primitive 花瓶 + 刚体假花（方案 A）。
  使用方式：在主 scene XML（如 `src/kuavo_assets/models/biped_s51/xml/biped_s51_mujoco.xml`）
  里手动加 `<include file="..."/>`，本目录不直接修改主 scene。
- `scenes/wrist_cam_snippet.xml` —— D405 腕部相机 XML 片段（fovy=58°），同样手动 include。
- `policy_runner.py` —— Lingbot-VLA / Pi0.5 推理桥接（占位骨架）。
  订阅 `/wrist_cam/color/image_raw` + TF → 推理 → 发布 `twoArmHandPoseCmd` (frame=2)。
  通过 import 复用 `src/demo/umi_replay/scripts/umi_realtime_teleop.py` 的 FHAN 平滑与 `--max-delta` 裁剪。
- `success_checker.py` —— 花头 z > 瓶口 / 茎底 z < 瓶口 / 茎底 xy 在瓶口内 / gripper 松开
  四条件持续 1 s 的成功判定。提供纯函数与 ROS 节点两种调用方式。
- `domain_randomizer.py` —— 中度 DR：物体 pose、光照、纹理/材质切换；reset service 调用入口。
  不随机相机外参与花瓶尺寸。
- `test_policy_flower.py` —— pytest 入口，参数化 seed × 物体位置 × 光照，
  跑 N 个 episode，输出 YAML（success_rate、completion_time、jerk、safety）。

## 运行
```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch mujoco_headless:=true
rosrun umi_eva policy_runner.py --ckpt lingbot_vla_flower.pt
pytest src/umi_eva/test_policy_flower.py --episodes 50
```

## 状态
当前所有 `.py` / `.xml` 都是 **占位骨架**，等 Lingbot-VLA 的 ckpt + python API
到位后再填入实质实现。
