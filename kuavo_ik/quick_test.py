#!/usr/bin/env python3
"""
快速测试脚本 - 检查你的目标位姿是否可达
使用方法：修改下面的 TARGET_POSE 参数，然后运行
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'kuavo_ik'))
from kuavo_ik.ik_library import IKAnalytical
from kuavo_ik.fk_tool import FKTool

# ============================================================
# 📝 在这里修改你要测试的目标位姿
# ============================================================

TARGET_POSE = {
    # 目标位置（单位：米，相对于base_link）
    'position': [0.4, 0.3, 0.5],  # [x, y, z]
    
    # 目标姿态（单位：度）
    'orientation_deg': [0, -90, 0],  # [roll, pitch, yaw]
    
    # 左手或右手
    'arm': 'left',  # 'left' 或 'right'
    
    # 机器人型号
    'model': '46'
}

# ============================================================
# 主程序（无需修改）
# ============================================================

def quick_test():
    """快速测试"""
    
    # 解析参数
    pos = np.array(TARGET_POSE['position'])
    euler_deg = np.array(TARGET_POSE['orientation_deg'])
    euler_rad = np.deg2rad(euler_deg)
    arm = TARGET_POSE['arm']
    model = TARGET_POSE['model']
    
    frame = 'zarm_l7_link' if arm == 'left' else 'zarm_r7_link'
    
    print("="*70)
    print(f"  快速可达性测试 - {arm.upper()} ARM")
    print("="*70)
    print(f"目标位置:  [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] m")
    print(f"目标姿态:  [{euler_deg[0]:.1f}°, {euler_deg[1]:.1f}°, {euler_deg[2]:.1f}°]")
    print(f"机器人型号: {model}")
    print("-"*70)
    
    # IK求解
    try:
        quat = R.from_euler('xyz', euler_rad).as_quat()
        joint_angles = IKAnalytical.compute(
            eef_pos=pos,
            eef_quat_xyzw=quat,
            eef_frame=frame,
            model_type=model,
            limit=True
        )
        print("✓ IK求解成功！\n")
        
        # 打印关节角度
        print("关节角度:")
        for i in range(7):
            print(f"  joint_{i+1}: {joint_angles[i]:>8.4f} rad = {np.rad2deg(joint_angles[i]):>8.2f}°")
        
        # FK验证
        fk_tool = FKTool()
        if arm == 'left':
            full_joints = np.concatenate([joint_angles, np.zeros(7)])
        else:
            full_joints = np.concatenate([np.zeros(7), joint_angles])
        
        fk_pos, fk_quat = fk_tool.compute(full_joints, frame_name=frame)
        fk_euler = R.from_quat(fk_quat).as_euler('xyz')
        
        pos_error = np.linalg.norm(np.array(fk_pos) - pos) * 1000  # mm
        rot_error = np.linalg.norm(fk_euler - euler_rad)  # rad
        
        print(f"\nFK验证:")
        print(f"  实际位置: [{fk_pos[0]:.3f}, {fk_pos[1]:.3f}, {fk_pos[2]:.3f}] m")
        print(f"  实际姿态: [{np.rad2deg(fk_euler[0]):.1f}°, {np.rad2deg(fk_euler[1]):.1f}°, {np.rad2deg(fk_euler[2]):.1f}°]")
        print(f"\n误差:")
        print(f"  位置误差: {pos_error:.4f} mm")
        print(f"  姿态误差: {np.rad2deg(rot_error):.4f}°")
        
        # 判断
        if pos_error < 10 and np.rad2deg(rot_error) < 5:
            print("\n" + "="*70)
            print("✅✅✅  该位姿完全可达！精度优秀！")
            print("="*70)
        elif pos_error < 50 and np.rad2deg(rot_error) < 10:
            print("\n" + "="*70)
            print("✅  该位姿可达，但精度一般。")
            print("="*70)
        else:
            print("\n" + "="*70)
            print("⚠️  该位姿理论上可达，但误差较大，请检查参数。")
            print("="*70)
            
    except Exception as e:
        print(f"✗ IK求解失败！")
        print(f"原因: {e}")
        print("\n" + "="*70)
        print("❌  该位姿不可达！")
        print("="*70)
        print("\n💡 建议:")
        print("  1. 检查位置是否在工作空间范围内")
        print("  2. 机器人左手典型工作范围:")
        print("     X: 0.2~0.6m, Y: -0.1~0.5m, Z: 0.2~0.8m")
        print("  3. 尝试调整姿态（推荐向下: [0, -90, 0]）")


if __name__ == "__main__":
    print("\n" + "╔" + "═"*68 + "╗")
    print("║" + " "*18 + "Kuavo IK 快速测试工具" + " "*28 + "║")
    print("╚" + "═"*68 + "╝\n")
    
    quick_test()
    
    print("\n💡 使用提示:")
    print("  - 修改脚本开头的 TARGET_POSE 参数")
    print("  - 重新运行: python3 quick_test.py")
    print("")
