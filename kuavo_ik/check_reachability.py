#!/usr/bin/env python3
"""
末端位姿可达性检查工具
用途：快速检查给定的末端位姿是否可达，并可视化关节配置
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'kuavo_ik'))

from kuavo_ik.ik_library import IKAnalytical
from kuavo_ik.fk_tool import FKTool


class ReachabilityChecker:
    """可达性检查器"""
    
    def __init__(self, model_type='46'):
        self.model_type = model_type
        self.fk_tool = FKTool()
        
        # 关节限位（弧度）
        self.joint_limits = {
            'joint_1': (-3.14159, 1.5708),
            'joint_2': (-0.3491, 2.0944),
            'joint_3': (-1.5708, 1.5708),
            'joint_4': (-2.6180, 0.0),
            'joint_5': (-1.5708, 1.5708),
            'joint_6': (-1.3090, 0.6981),
            'joint_7': (-1.9199, 1.0472)
        }
    
    def check_pose(self, pos, quat_or_euler, frame='zarm_l7_link', 
                   angle_type='euler', verbose=True):
        """
        检查末端位姿是否可达
        
        参数:
            pos: [x, y, z] 位置（米）
            quat_or_euler: 
                - 如果angle_type='euler': [roll, pitch, yaw] (弧度)
                - 如果angle_type='quat': [qx, qy, qz, qw]
            frame: 'zarm_l7_link' 或 'zarm_r7_link'
            angle_type: 'euler' 或 'quat'
            verbose: 是否打印详细信息
        
        返回:
            (is_reachable, joint_angles, error_dict)
        """
        
        # 转换姿态到四元数
        if angle_type == 'euler':
            euler = np.array(quat_or_euler)
            quat = R.from_euler('xyz', euler).as_quat()
        else:
            quat = np.array(quat_or_euler)
            euler = R.from_quat(quat).as_euler('xyz')
        
        pos = np.array(pos)
        
        if verbose:
            print("\n" + "="*70)
            print(f"检查末端位姿可达性 [{frame}]")
            print("="*70)
            print(f"目标位置: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] m")
            print(f"目标姿态(欧拉角): [{np.rad2deg(euler[0]):.1f}°, "
                  f"{np.rad2deg(euler[1]):.1f}°, {np.rad2deg(euler[2]):.1f}°]")
        
        # 尝试IK求解
        try:
            joint_angles = IKAnalytical.compute(
                eef_pos=pos,
                eef_quat_xyzw=quat,
                eef_frame=frame,
                model_type=self.model_type,
                limit=True
            )
            
            if verbose:
                print(f"\n✓ IK求解成功！")
                self._print_joint_angles(joint_angles)
            
        except Exception as e:
            if verbose:
                print(f"\n✗ IK求解失败: {e}")
            return False, None, {'error': str(e)}
        
        # 检查关节限位
        limit_violations = self._check_joint_limits(joint_angles)
        
        # FK验证
        if frame == 'zarm_l7_link':
            full_joints = np.concatenate([joint_angles, np.zeros(7)])
        else:
            full_joints = np.concatenate([np.zeros(7), joint_angles])
        
        fk_pos, fk_quat = self.fk_tool.compute(full_joints, frame_name=frame)
        fk_euler = R.from_quat(fk_quat).as_euler('xyz')
        
        # 计算误差
        pos_error = np.linalg.norm(np.array(fk_pos) - pos)
        rot_error = np.linalg.norm(fk_euler - euler)
        
        if verbose:
            print(f"\n{'FK验证结果':─^70}")
            print(f"实际位置: [{fk_pos[0]:.3f}, {fk_pos[1]:.3f}, {fk_pos[2]:.3f}] m")
            print(f"实际姿态: [{np.rad2deg(fk_euler[0]):.1f}°, "
                  f"{np.rad2deg(fk_euler[1]):.1f}°, {np.rad2deg(fk_euler[2]):.1f}°]")
            print(f"\n误差:")
            print(f"  位置误差: {pos_error*1000:.4f} mm")
            print(f"  姿态误差: {np.rad2deg(rot_error):.4f}°")
        
        # 综合判断
        is_reachable = True
        issues = []
        
        if limit_violations:
            is_reachable = False
            issues.extend(limit_violations)
        
        if pos_error > 0.01:  # 10mm
            is_reachable = False
            issues.append(f"位置误差过大: {pos_error*1000:.2f}mm")
        
        if rot_error > 0.1:  # ~5.7度
            is_reachable = False
            issues.append(f"姿态误差过大: {np.rad2deg(rot_error):.2f}°")
        
        if verbose:
            print(f"\n{'最终结论':═^70}")
            if is_reachable:
                print("✅ 该位姿可达！可以安全使用这组关节角度。")
            else:
                print("❌ 该位姿不可达或存在问题：")
                for issue in issues:
                    print(f"   - {issue}")
            print("="*70 + "\n")
        
        error_dict = {
            'pos_error_mm': pos_error * 1000,
            'rot_error_deg': np.rad2deg(rot_error),
            'issues': issues
        }
        
        return is_reachable, joint_angles, error_dict
    
    def _print_joint_angles(self, joint_angles):
        """打印关节角度"""
        print(f"\n关节角度:")
        print(f"{'关节':<10} {'弧度':<12} {'角度':<12} {'限位范围':<25}")
        print("-" * 70)
        for i, (name, (lower, upper)) in enumerate(self.joint_limits.items()):
            angle_rad = joint_angles[i]
            angle_deg = np.rad2deg(angle_rad)
            limit_str = f"[{np.rad2deg(lower):.1f}°, {np.rad2deg(upper):.1f}°]"
            
            # 检查是否接近限位
            margin_lower = angle_rad - lower
            margin_upper = upper - angle_rad
            min_margin = min(margin_lower, margin_upper)
            
            warning = ""
            if min_margin < 0:
                warning = " ⚠️ 超限!"
            elif min_margin < 0.1:  # 接近限位 (<5.7度)
                warning = " ⚠️ 接近限位"
            
            print(f"{name:<10} {angle_rad:>10.4f}  {angle_deg:>10.2f}°  {limit_str:<25} {warning}")
    
    def _check_joint_limits(self, joint_angles):
        """检查关节限位违规"""
        violations = []
        for i, (name, (lower, upper)) in enumerate(self.joint_limits.items()):
            angle = joint_angles[i]
            if angle < lower:
                violations.append(f"{name} 低于下限: {np.rad2deg(angle):.2f}° < {np.rad2deg(lower):.2f}°")
            elif angle > upper:
                violations.append(f"{name} 超过上限: {np.rad2deg(angle):.2f}° > {np.rad2deg(upper):.2f}°")
        return violations
    
    def batch_check(self, pose_list, frame='zarm_l7_link', angle_type='euler'):
        """
        批量检查多个位姿
        
        参数:
            pose_list: [(pos, angle, description), ...]
        """
        print("\n" + "="*70)
        print(f"批量可达性检查 [{frame}]")
        print("="*70)
        
        results = []
        for pos, angle, desc in pose_list:
            is_reachable, joints, errors = self.check_pose(
                pos, angle, frame, angle_type, verbose=False
            )
            results.append((desc, is_reachable, errors))
        
        # 打印汇总表
        print(f"\n{'描述':<20} {'位置 (m)':<30} {'状态':<15} {'位置误差':<15} {'姿态误差':<15}")
        print("-" * 100)
        for i, (desc, is_reachable, errors) in enumerate(results):
            pos, _, _ = pose_list[i]
            pos_str = f"[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
            status = "✓ 可达" if is_reachable else "✗ 不可达"
            pos_err = f"{errors.get('pos_error_mm', 0):.3f} mm"
            rot_err = f"{errors.get('rot_error_deg', 0):.3f}°"
            print(f"{desc:<20} {pos_str:<30} {status:<15} {pos_err:<15} {rot_err:<15}")
        
        reachable_count = sum(1 for _, r, _ in results if r)
        print(f"\n总结: {reachable_count}/{len(results)} 个位姿可达")
        print("="*70 + "\n")
        
        return results


def example_usage():
    """使用示例"""
    
    checker = ReachabilityChecker(model_type='46')
    
    # === 示例1: 检查单个位姿 ===
    print("\n【示例1】检查单个位姿")
    pos = [0.4, 0.3, 0.5]  # x, y, z (米)
    euler = [0, -np.pi/2, 0]  # roll, pitch, yaw (弧度) - 向下
    
    is_reachable, joints, errors = checker.check_pose(
        pos, euler, 
        frame='zarm_l7_link',
        angle_type='euler',
        verbose=True
    )
    
    # === 示例2: 批量检查多个位姿 ===
    print("\n【示例2】批量检查工作空间")
    
    poses = [
        ([0.3, 0.2, 0.4], [0, -np.pi/2, 0], "近距离-右侧"),
        ([0.4, 0.3, 0.5], [0, -np.pi/2, 0], "中距离-右侧"),
        ([0.5, 0.0, 0.6], [0, -np.pi/2, 0], "远距离-正前"),
        ([0.35, -0.2, 0.4], [0, -np.pi/2, 0], "中距离-左侧"),
        ([0.6, 0.0, 0.7], [0, -np.pi/2, 0], "极限距离"),
    ]
    
    checker.batch_check(poses, frame='zarm_l7_link', angle_type='euler')
    
    # === 示例3: 使用四元数 ===
    print("\n【示例3】使用四元数表示姿态")
    pos = [0.4, 0.2, 0.5]
    quat = [0, -0.7071, 0, 0.7071]  # qx, qy, qz, qw (向下)
    
    is_reachable, joints, errors = checker.check_pose(
        pos, quat,
        frame='zarm_l7_link',
        angle_type='quat',
        verbose=True
    )


if __name__ == "__main__":
    print("""
╔════════════════════════════════════════════════════════════════════╗
║           Kuavo 机器人末端位姿可达性检查工具                         ║
║                                                                    ║
║  用途：验证给定的末端执行器位置和姿态是否可达                          ║
╚════════════════════════════════════════════════════════════════════╝
    """)
    
    example_usage()
    
    print("""
╔════════════════════════════════════════════════════════════════════╗
║  💡 使用提示：                                                      ║
║                                                                    ║
║  1. 修改 example_usage() 中的位置和姿态进行测试                      ║
║  2. 位置单位：米，姿态单位：弧度                                     ║
║  3. 坐标系：相对于 base_link                                        ║
║  4. 推荐姿态：向下抓取 [0, -π/2, 0]                                 ║
╚════════════════════════════════════════════════════════════════════╝
    """)
