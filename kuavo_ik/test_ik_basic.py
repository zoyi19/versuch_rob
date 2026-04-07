#!/usr/bin/env python3
"""
IK库基础测试脚本
用途：测试给定末端位姿是否可达，并验证IK结果
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'kuavo_ik'))

from kuavo_ik.ik_library import IKAnalytical
from kuavo_ik.fk_tool import FKTool

def test_ik_reachability():
    """测试末端位姿是否可达"""
    
    print("="*60)
    print("测试1：IK解析解 - 验证末端位姿可达性")
    print("="*60)
    
    # === 定义目标末端位姿 ===
    # 目标位置（相对于base_link，单位：米）
    target_pos = np.array([0.4, 0.3, 0.5])  # [x, y, z]
    
    # 目标姿态（欧拉角，单位：弧度）
    target_euler = np.array([0.0, -np.pi/2, 0.0])  # [roll, pitch, yaw]
    target_quat = R.from_euler('xyz', target_euler).as_quat()  # 转换为四元数 [qx,qy,qz,qw]
    
    print(f"\n目标位置: {target_pos}")
    print(f"目标姿态(欧拉角): {np.rad2deg(target_euler)} 度")
    print(f"目标姿态(四元数): {target_quat}")
    
    # === 调用IK求解 ===
    try:
        joint_angles = IKAnalytical.compute(
            eef_pos=target_pos,
            eef_quat_xyzw=target_quat,
            eef_frame='zarm_l7_link',  # 左手末端
            model_type='46',  # 机器人型号
            limit=True  # 启用关节限位检查
        )
        
        print(f"\n✓ IK求解成功！")
        print(f"关节角度（弧度）: {joint_angles}")
        print(f"关节角度（度）: {np.rad2deg(joint_angles)}")
        
    except Exception as e:
        print(f"\n✗ IK求解失败: {e}")
        return False
    
    # === 验证：使用FK正运动学验证结果 ===
    print("\n" + "-"*60)
    print("验证：使用FK正运动学检验IK结果")
    print("-"*60)
    
    fk_tool = FKTool()
    
    # 构造完整关节角（左手7个+右手7个=14个）
    full_joint_angles = np.concatenate([joint_angles, np.zeros(7)])
    
    # 计算FK
    fk_pos, fk_quat = fk_tool.compute(full_joint_angles, frame_name='zarm_l7_link')
    fk_euler = R.from_quat(fk_quat).as_euler('xyz')
    
    print(f"\nFK计算结果:")
    print(f"  实际位置: {fk_pos}")
    print(f"  实际姿态(欧拉角): {np.rad2deg(fk_euler)} 度")
    
    # 计算误差
    pos_error = np.linalg.norm(np.array(fk_pos) - target_pos)
    rot_error = np.linalg.norm(fk_euler - target_euler)
    
    print(f"\n误差分析:")
    print(f"  位置误差: {pos_error*1000:.3f} mm")
    print(f"  姿态误差: {np.rad2deg(rot_error):.3f} 度")
    
    # 判断是否可达
    if pos_error < 0.01 and rot_error < 0.1:  # 位置误差<10mm，姿态误差<5.7度
        print(f"\n✓✓✓ 末端位姿可达！误差在允许范围内。")
        return True
    else:
        print(f"\n⚠ 末端位姿可能不太理想，误差较大。")
        return False


def test_workspace_exploration():
    """测试工作空间探索"""
    
    print("\n\n" + "="*60)
    print("测试2：工作空间探索 - 测试多个位置")
    print("="*60)
    
    # 测试多个位置
    test_positions = [
        ([0.3, 0.2, 0.4], "近处-右侧"),
        ([0.4, 0.3, 0.5], "中距离"),
        ([0.5, 0.0, 0.6], "远处-正前"),
        ([0.35, -0.2, 0.4], "左侧"),
    ]
    
    target_euler = np.array([0.0, -np.pi/2, 0.0])  # 统一姿态：向下
    target_quat = R.from_euler('xyz', target_euler).as_quat()
    
    results = []
    for pos, desc in test_positions:
        try:
            joint_angles = IKAnalytical.compute(
                eef_pos=pos,
                eef_quat_xyzw=target_quat,
                eef_frame='zarm_l7_link',
                model_type='46',
                limit=True
            )
            status = "✓ 可达"
        except Exception as e:
            status = f"✗ 不可达"
            joint_angles = None
        
        results.append((desc, pos, status))
        print(f"{desc:15s} {pos} -> {status}")
    
    print(f"\n可达性总结: {sum(1 for r in results if '✓' in r[2])}/{len(results)} 个位置可达")


def print_usage_guide():
    """打印使用指南"""
    
    print("\n\n" + "="*60)
    print("IK库使用指南")
    print("="*60)
    
    guide = """
📖 基本用法：

1. 导入模块：
   from kuavo_ik.ik_library import IKAnalytical
   from kuavo_ik.fk_tool import FKTool
   from scipy.spatial.transform import Rotation as R

2. 定义目标位姿：
   pos = [x, y, z]  # 单位：米
   euler = [roll, pitch, yaw]  # 单位：弧度
   quat = R.from_euler('xyz', euler).as_quat()

3. 调用IK求解：
   joints = IKAnalytical.compute(
       eef_pos=pos,
       eef_quat_xyzw=quat,
       eef_frame='zarm_l7_link',  # 或 'zarm_r7_link'
       model_type='46',
       limit=True
   )

4. 验证结果（可选）：
   fk = FKTool()
   fk_pos, fk_quat = fk.compute(joints, 'zarm_l7_link')

🎯 关键参数说明：
- eef_pos: 末端位置 [x,y,z]，相对于base_link
- eef_quat_xyzw: 末端姿态四元数 [qx,qy,qz,qw]
- eef_frame: 'zarm_l7_link'(左手) 或 'zarm_r7_link'(右手)
- model_type: '46' (当前机器人版本)
- limit: True表示启用关节限位检查

⚠️  注意事项：
1. 坐标系：所有位姿都相对于base_link
2. 单位：位置用米，角度用弧度
3. 姿态表示：推荐用四元数，避免万向节死锁
4. 关节限位：joint_1到joint_7都有物理限位
"""
    print(guide)


if __name__ == "__main__":
    # 运行测试
    test_ik_reachability()
    test_workspace_exploration()
    print_usage_guide()
    
    print("\n" + "="*60)
    print("测试完成！你已经掌握IK库的基本使用。")
    print("="*60)
