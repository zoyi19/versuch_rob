#!/usr/bin/env python3
"""
你的专属测试文件 - 在这里测试你自己的位姿
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'kuavo_ik'))

from kuavo_ik.ik_library import IKAnalytical
from kuavo_ik.fk_tool import FKTool

# ============================================================
# 🔥 在这里修改你要测试的位姿
# ============================================================

# 你的目标位置（单位：米）
MY_POSITION = [0.4, 0.3, 0.5]

# 你的目标姿态（单位：度）
MY_ORIENTATION = [0, -90, 0]  # [roll, pitch, yaw]

# 左手还是右手
MY_ARM = 'left'  # 'left' 或 'right'

# ============================================================

def main():
    print(f"\n{'='*60}")
    print(f"  测试位姿: {MY_POSITION}")
    print(f"  测试姿态: {MY_ORIENTATION}°")
    print(f"  使用臂: {MY_ARM.upper()}")
    print(f"{'='*60}\n")
    
    # 转换
    pos = np.array(MY_POSITION)
    euler_rad = np.deg2rad(MY_ORIENTATION)
    quat = R.from_euler('xyz', euler_rad).as_quat()
    frame = 'zarm_l7_link' if MY_ARM == 'left' else 'zarm_r7_link'
    
    # IK求解
    try:
        joints = IKAnalytical.compute(
            eef_pos=pos,
            eef_quat_xyzw=quat,
            eef_frame=frame,
            model_type='46',
            limit=True
        )
        
        print("IK求解成功！")
        print(f"\n关节角度（度）:")
        for i, angle in enumerate(joints):
            print(f"  joint_{i+1}: {np.rad2deg(angle):>8.2f}°")
        
        # FK验证
        fk = FKTool()
        full = np.concatenate([joints, np.zeros(7)]) if MY_ARM == 'left' else np.concatenate([np.zeros(7), joints])
        fk_pos, fk_quat = fk.compute(full, frame)
        
        error = np.linalg.norm(np.array(fk_pos) - pos) * 1000
        print(f"\n✅ FK验证: 位置误差 {error:.4f} mm")
        
        if error < 10:
            print("\n🎉 该位姿完全可达！精度优秀！\n")
        else:
            print(f"\n⚠️  误差较大，请检查参数\n")
            
    except Exception as e:
        print(f"❌ 求解失败: {e}\n")

if __name__ == "__main__":
    main()
