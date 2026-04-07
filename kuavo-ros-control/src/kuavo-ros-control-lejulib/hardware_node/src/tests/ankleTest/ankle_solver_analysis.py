#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脚踝解算验证测试分析脚本

该脚本读取ankle solver测试生成的CSV数据文件，进行误差分析和可视化。

使用方法:
    python3 ankle_solver_analysis.py [数据目录路径] [测试类型]

测试类型:
    - position: 位置控制测试
    - velocity: 速度控制测试
    - torque: 力矩控制测试
    - all: 分析所有测试类型（默认）
"""

import os
import sys
import glob

# 依赖检查：缺少时给出安装提示并退出
try:
    import numpy as np
    import pandas as pd
    import matplotlib
    matplotlib.use('Agg')  # 无显示环境也可保存图片，与 visualize_errors 一致
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from scipy import stats
except ModuleNotFoundError as e:
    print("错误：分析脚本缺少 Python 依赖。", file=sys.stderr)
    print("请执行: pip3 install pandas numpy matplotlib scipy", file=sys.stderr)
    print("缺失模块:", e.name, file=sys.stderr)
    sys.exit(1)

from typing import List, Dict, Tuple, Optional
import argparse
from datetime import datetime

# 设置中文字体
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans']
matplotlib.rcParams['axes.unicode_minus'] = False


class AnkleSolverAnalyzer:
    """脚踝解算测试分析器"""
    
    def __init__(self, data_dir: str = None):
        if data_dir is None:
            self.data_dir = self.find_data_directory()
        else:
            self.data_dir = data_dir
        
        self.results = {}
        
    def find_data_directory(self) -> str:
        """自动查找数据目录"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        possible_paths = [
            os.path.join(current_dir, 'data'),
            os.path.join(current_dir, '..', 'data'),
            os.path.join(current_dir, '..', '..', 'data'),
            './data',
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                csv_files = glob.glob(os.path.join(path, "ankle_solver_v17_test_*.csv"))
                if csv_files:
                    print(f"找到数据目录: {path}")
                    return path
        
        print("警告：未找到数据目录，使用默认路径 ./data")
        return "./data"
    
    def load_position_data(self, filepath: str) -> pd.DataFrame:
        """加载位置控制测试数据"""
        try:
            df = pd.read_csv(filepath)
            return df
        except Exception as e:
            print(f"错误：无法读取文件 {filepath}: {e}")
            return pd.DataFrame()
    
    def load_velocity_data(self, filepath: str) -> pd.DataFrame:
        """加载速度控制测试数据"""
        try:
            df = pd.read_csv(filepath)
            return df
        except Exception as e:
            print(f"错误：无法读取文件 {filepath}: {e}")
            return pd.DataFrame()
    
    def load_torque_data(self, filepath: str) -> pd.DataFrame:
        """加载力矩控制测试数据"""
        try:
            df = pd.read_csv(filepath)
            return df
        except Exception as e:
            print(f"错误：无法读取文件 {filepath}: {e}")
            return pd.DataFrame()
    
    def analyze_position_test(self, df: pd.DataFrame) -> Dict:
        """分析位置控制测试"""
        if df.empty:
            return {}
        
        results = {
            'mean_error_pitch_l': 0.0,
            'mean_error_roll_l': 0.0,
            'mean_error_pitch_r': 0.0,
            'mean_error_roll_r': 0.0,
            'std_error_pitch_l': 0.0,
            'std_error_roll_l': 0.0,
            'std_error_pitch_r': 0.0,
            'std_error_roll_r': 0.0,
            'max_error_pitch_l': 0.0,
            'max_error_roll_l': 0.0,
            'max_error_pitch_r': 0.0,
            'max_error_roll_r': 0.0,
            'rmse_pitch_l': 0.0,
            'rmse_roll_l': 0.0,
            'rmse_pitch_r': 0.0,
            'rmse_roll_r': 0.0,
        }
        
        # 检查是否有foot列（新格式）
        if 'foot' in df.columns:
            # 按foot分组分析
            for foot_type in df['foot'].unique():
                df_foot = df[df['foot'] == foot_type]
                
                if foot_type in ['left', 'both']:
                    if 'error_pitch_l' in df_foot.columns:
                        results['mean_error_pitch_l'] = df_foot['error_pitch_l'].mean()
                        results['std_error_pitch_l'] = df_foot['error_pitch_l'].std()
                        results['max_error_pitch_l'] = df_foot['error_pitch_l'].abs().max()
                        results['rmse_pitch_l'] = np.sqrt((df_foot['error_pitch_l']**2).mean())
                    
                    if 'error_roll_l' in df_foot.columns:
                        results['mean_error_roll_l'] = df_foot['error_roll_l'].mean()
                        results['std_error_roll_l'] = df_foot['error_roll_l'].std()
                        results['max_error_roll_l'] = df_foot['error_roll_l'].abs().max()
                        results['rmse_roll_l'] = np.sqrt((df_foot['error_roll_l']**2).mean())
                
                if foot_type in ['right', 'both']:
                    if 'error_pitch_r' in df_foot.columns:
                        results['mean_error_pitch_r'] = df_foot['error_pitch_r'].mean()
                        results['std_error_pitch_r'] = df_foot['error_pitch_r'].std()
                        results['max_error_pitch_r'] = df_foot['error_pitch_r'].abs().max()
                        results['rmse_pitch_r'] = np.sqrt((df_foot['error_pitch_r']**2).mean())
                    
                    if 'error_roll_r' in df_foot.columns:
                        results['mean_error_roll_r'] = df_foot['error_roll_r'].mean()
                        results['std_error_roll_r'] = df_foot['error_roll_r'].std()
                        results['max_error_roll_r'] = df_foot['error_roll_r'].abs().max()
                        results['rmse_roll_r'] = np.sqrt((df_foot['error_roll_r']**2).mean())
        else:
            # 旧格式：直接分析所有数据
            if 'error_pitch_l' in df.columns:
                results['mean_error_pitch_l'] = df['error_pitch_l'].mean()
                results['std_error_pitch_l'] = df['error_pitch_l'].std()
                results['max_error_pitch_l'] = df['error_pitch_l'].abs().max()
                results['rmse_pitch_l'] = np.sqrt((df['error_pitch_l']**2).mean())
            
            if 'error_roll_l' in df.columns:
                results['mean_error_roll_l'] = df['error_roll_l'].mean()
                results['std_error_roll_l'] = df['error_roll_l'].std()
                results['max_error_roll_l'] = df['error_roll_l'].abs().max()
                results['rmse_roll_l'] = np.sqrt((df['error_roll_l']**2).mean())
            
            if 'error_pitch_r' in df.columns:
                results['mean_error_pitch_r'] = df['error_pitch_r'].mean()
                results['std_error_pitch_r'] = df['error_pitch_r'].std()
                results['max_error_pitch_r'] = df['error_pitch_r'].abs().max()
                results['rmse_pitch_r'] = np.sqrt((df['error_pitch_r']**2).mean())
            
            if 'error_roll_r' in df.columns:
                results['mean_error_roll_r'] = df['error_roll_r'].mean()
                results['std_error_roll_r'] = df['error_roll_r'].std()
                results['max_error_roll_r'] = df['error_roll_r'].abs().max()
                results['rmse_roll_r'] = np.sqrt((df['error_roll_r']**2).mean())
        
        # 解算误差分解（需 CSV 含 q_from_cmd_* 列）：总误差 = 解算往返误差 + 电机传播误差
        if all(c in df.columns for c in ['q_from_cmd_pitch_l', 'q_from_cmd_roll_l', 'q_from_cmd_pitch_r', 'q_from_cmd_roll_r']):
            # 解算往返误差 = q_cmd - motor_to_joint(joint_to_motor(q_cmd))，理想应接近 0
            solver_err_pl = (df['pitch_cmd'] - df['q_from_cmd_pitch_l']).values
            solver_err_rl = (df['roll_cmd'] - df['q_from_cmd_roll_l']).values
            solver_err_pr = (df['pitch_cmd'] - df['q_from_cmd_pitch_r']).values
            solver_err_rr = (df['roll_cmd'] - df['q_from_cmd_roll_r']).values
            # 电机传播误差 = q_from_cmd - q_fb（电机未跟到 p_cmd 经正解后的关节差）
            motor_err_pl = (df['q_from_cmd_pitch_l'] - df['q_fb_pitch_l']).values
            motor_err_rl = (df['q_from_cmd_roll_l'] - df['q_fb_roll_l']).values
            motor_err_pr = (df['q_from_cmd_pitch_r'] - df['q_fb_pitch_r']).values
            motor_err_rr = (df['q_from_cmd_roll_r'] - df['q_fb_roll_r']).values
            for name, arr in [
                ('rmse_solver_pitch_l', solver_err_pl), ('rmse_solver_roll_l', solver_err_rl),
                ('rmse_solver_pitch_r', solver_err_pr), ('rmse_solver_roll_r', solver_err_rr),
                ('rmse_motor_pitch_l', motor_err_pl), ('rmse_motor_roll_l', motor_err_rl),
                ('rmse_motor_pitch_r', motor_err_pr), ('rmse_motor_roll_r', motor_err_rr),
            ]:
                results[name] = np.sqrt(np.mean(arr ** 2))
            results['has_solver_decomposition'] = True
        
        return results
    
    def analyze_velocity_test(self, df: pd.DataFrame) -> Dict:
        """分析速度控制测试"""
        if df.empty:
            return {}
        
        results = {
            'mean_error_pitch_l': 0.0,
            'mean_error_roll_l': 0.0,
            'mean_error_pitch_r': 0.0,
            'mean_error_roll_r': 0.0,
            'std_error_pitch_l': 0.0,
            'std_error_roll_l': 0.0,
            'std_error_pitch_r': 0.0,
            'std_error_roll_r': 0.0,
            'max_error_pitch_l': 0.0,
            'max_error_roll_l': 0.0,
            'max_error_pitch_r': 0.0,
            'max_error_roll_r': 0.0,
            'rmse_pitch_l': 0.0,
            'rmse_roll_l': 0.0,
            'rmse_pitch_r': 0.0,
            'rmse_roll_r': 0.0,
        }
        
        # 检查是否有foot列（新格式）
        if 'foot' in df.columns:
            # 按foot分组分析
            for foot_type in df['foot'].unique():
                df_foot = df[df['foot'] == foot_type]
                
                if foot_type in ['left', 'both']:
                    if 'error_pitch_l' in df_foot.columns:
                        results['mean_error_pitch_l'] = df_foot['error_pitch_l'].mean()
                        results['std_error_pitch_l'] = df_foot['error_pitch_l'].std()
                        results['max_error_pitch_l'] = df_foot['error_pitch_l'].abs().max()
                        results['rmse_pitch_l'] = np.sqrt((df_foot['error_pitch_l']**2).mean())
                    
                    if 'error_roll_l' in df_foot.columns:
                        results['mean_error_roll_l'] = df_foot['error_roll_l'].mean()
                        results['std_error_roll_l'] = df_foot['error_roll_l'].std()
                        results['max_error_roll_l'] = df_foot['error_roll_l'].abs().max()
                        results['rmse_roll_l'] = np.sqrt((df_foot['error_roll_l']**2).mean())
                
                if foot_type in ['right', 'both']:
                    if 'error_pitch_r' in df_foot.columns:
                        results['mean_error_pitch_r'] = df_foot['error_pitch_r'].mean()
                        results['std_error_pitch_r'] = df_foot['error_pitch_r'].std()
                        results['max_error_pitch_r'] = df_foot['error_pitch_r'].abs().max()
                        results['rmse_pitch_r'] = np.sqrt((df_foot['error_pitch_r']**2).mean())
                    
                    if 'error_roll_r' in df_foot.columns:
                        results['mean_error_roll_r'] = df_foot['error_roll_r'].mean()
                        results['std_error_roll_r'] = df_foot['error_roll_r'].std()
                        results['max_error_roll_r'] = df_foot['error_roll_r'].abs().max()
                        results['rmse_roll_r'] = np.sqrt((df_foot['error_roll_r']**2).mean())
        else:
            # 旧格式：直接分析所有数据
            if 'error_pitch_l' in df.columns:
                results['mean_error_pitch_l'] = df['error_pitch_l'].mean()
                results['std_error_pitch_l'] = df['error_pitch_l'].std()
                results['max_error_pitch_l'] = df['error_pitch_l'].abs().max()
                results['rmse_pitch_l'] = np.sqrt((df['error_pitch_l']**2).mean())
            
            if 'error_roll_l' in df.columns:
                results['mean_error_roll_l'] = df['error_roll_l'].mean()
                results['std_error_roll_l'] = df['error_roll_l'].std()
                results['max_error_roll_l'] = df['error_roll_l'].abs().max()
                results['rmse_roll_l'] = np.sqrt((df['error_roll_l']**2).mean())
            
            if 'error_pitch_r' in df.columns:
                results['mean_error_pitch_r'] = df['error_pitch_r'].mean()
                results['std_error_pitch_r'] = df['error_pitch_r'].std()
                results['max_error_pitch_r'] = df['error_pitch_r'].abs().max()
                results['rmse_pitch_r'] = np.sqrt((df['error_pitch_r']**2).mean())
            
            if 'error_roll_r' in df.columns:
                results['mean_error_roll_r'] = df['error_roll_r'].mean()
                results['std_error_roll_r'] = df['error_roll_r'].std()
                results['max_error_roll_r'] = df['error_roll_r'].abs().max()
                results['rmse_roll_r'] = np.sqrt((df['error_roll_r']**2).mean())
        
        return results
    
    def analyze_torque_test(self, df: pd.DataFrame) -> Dict:
        """分析力矩控制测试"""
        if df.empty:
            return {}
        
        results = {
            'mean_error_pitch_l': 0.0,
            'mean_error_roll_l': 0.0,
            'mean_error_pitch_r': 0.0,
            'mean_error_roll_r': 0.0,
            'std_error_pitch_l': 0.0,
            'std_error_roll_l': 0.0,
            'std_error_pitch_r': 0.0,
            'std_error_roll_r': 0.0,
            'max_error_pitch_l': 0.0,
            'max_error_roll_l': 0.0,
            'max_error_pitch_r': 0.0,
            'max_error_roll_r': 0.0,
            'rmse_pitch_l': 0.0,
            'rmse_roll_l': 0.0,
            'rmse_pitch_r': 0.0,
            'rmse_roll_r': 0.0,
        }
        
        # 检查是否有foot列（新格式）
        if 'foot' in df.columns:
            # 按foot分组分析
            for foot_type in df['foot'].unique():
                df_foot = df[df['foot'] == foot_type]
                
                if foot_type in ['left', 'both']:
                    if 'error_pitch_l' in df_foot.columns:
                        results['mean_error_pitch_l'] = df_foot['error_pitch_l'].mean()
                        results['std_error_pitch_l'] = df_foot['error_pitch_l'].std()
                        results['max_error_pitch_l'] = df_foot['error_pitch_l'].abs().max()
                        results['rmse_pitch_l'] = np.sqrt((df_foot['error_pitch_l']**2).mean())
                    
                    if 'error_roll_l' in df_foot.columns:
                        results['mean_error_roll_l'] = df_foot['error_roll_l'].mean()
                        results['std_error_roll_l'] = df_foot['error_roll_l'].std()
                        results['max_error_roll_l'] = df_foot['error_roll_l'].abs().max()
                        results['rmse_roll_l'] = np.sqrt((df_foot['error_roll_l']**2).mean())
                
                if foot_type in ['right', 'both']:
                    if 'error_pitch_r' in df_foot.columns:
                        results['mean_error_pitch_r'] = df_foot['error_pitch_r'].mean()
                        results['std_error_pitch_r'] = df_foot['error_pitch_r'].std()
                        results['max_error_pitch_r'] = df_foot['error_pitch_r'].abs().max()
                        results['rmse_pitch_r'] = np.sqrt((df_foot['error_pitch_r']**2).mean())
                    
                    if 'error_roll_r' in df_foot.columns:
                        results['mean_error_roll_r'] = df_foot['error_roll_r'].mean()
                        results['std_error_roll_r'] = df_foot['error_roll_r'].std()
                        results['max_error_roll_r'] = df_foot['error_roll_r'].abs().max()
                        results['rmse_roll_r'] = np.sqrt((df_foot['error_roll_r']**2).mean())
        else:
            # 旧格式：直接分析所有数据
            if 'error_pitch_l' in df.columns:
                results['mean_error_pitch_l'] = df['error_pitch_l'].mean()
                results['std_error_pitch_l'] = df['error_pitch_l'].std()
                results['max_error_pitch_l'] = df['error_pitch_l'].abs().max()
                results['rmse_pitch_l'] = np.sqrt((df['error_pitch_l']**2).mean())
            
            if 'error_roll_l' in df.columns:
                results['mean_error_roll_l'] = df['error_roll_l'].mean()
                results['std_error_roll_l'] = df['error_roll_l'].std()
                results['max_error_roll_l'] = df['error_roll_l'].abs().max()
                results['rmse_roll_l'] = np.sqrt((df['error_roll_l']**2).mean())
            
            if 'error_pitch_r' in df.columns:
                results['mean_error_pitch_r'] = df['error_pitch_r'].mean()
                results['std_error_pitch_r'] = df['error_pitch_r'].std()
                results['max_error_pitch_r'] = df['error_pitch_r'].abs().max()
                results['rmse_pitch_r'] = np.sqrt((df['error_pitch_r']**2).mean())
            
            if 'error_roll_r' in df.columns:
                results['mean_error_roll_r'] = df['error_roll_r'].mean()
                results['std_error_roll_r'] = df['error_roll_r'].std()
                results['max_error_roll_r'] = df['error_roll_r'].abs().max()
                results['rmse_roll_r'] = np.sqrt((df['error_roll_r']**2).mean())
        
        return results
    
    def plot_position_analysis(self, df: pd.DataFrame, output_dir: str):
        """绘制位置控制测试分析图（与 visualize_errors 一致的 3x4 布局，角度单位 deg）"""
        if df.empty or 'pitch_cmd' not in df.columns or 'error_pitch_l' not in df.columns:
            print("位置测试数据为空或缺少必要列，跳过绘图")
            return
        
        pitch_cmd_deg = np.degrees(df['pitch_cmd'].values)
        roll_cmd_deg = np.degrees(df['roll_cmd'].values)
        error_pitch_l_deg = np.degrees(df['error_pitch_l'].values)
        error_roll_l_deg = np.degrees(df['error_roll_l'].values)
        error_pitch_r_deg = np.degrees(df['error_pitch_r'].values)
        error_roll_r_deg = np.degrees(df['error_roll_r'].values)
        has_motor_cols = 'p_cmd_11' in df.columns and 'p_fb_11' in df.columns
        has_q_fb_roll_r = 'q_fb_roll_r' in df.columns
        
        fig = plt.figure(figsize=(20, 16))
        
        # 1. 左脚 Pitch 误差热力图
        ax1 = plt.subplot(3, 4, 1)
        s1 = ax1.scatter(pitch_cmd_deg, roll_cmd_deg, c=np.abs(error_pitch_l_deg),
                         cmap='Reds', s=50, alpha=0.7, vmin=0, vmax=10)
        ax1.set_xlabel('Pitch Command (deg)')
        ax1.set_ylabel('Roll Command (deg)')
        ax1.set_title('Left Pitch Error Heatmap')
        ax1.grid(True, alpha=0.3)
        plt.colorbar(s1, ax=ax1, label='|Error| (deg)')
        
        # 2. 左脚 Roll 误差热力图
        ax2 = plt.subplot(3, 4, 2)
        s2 = ax2.scatter(pitch_cmd_deg, roll_cmd_deg, c=np.abs(error_roll_l_deg),
                         cmap='Reds', s=50, alpha=0.7, vmin=0, vmax=20)
        ax2.set_xlabel('Pitch Command (deg)')
        ax2.set_ylabel('Roll Command (deg)')
        ax2.set_title('Left Roll Error Heatmap')
        ax2.grid(True, alpha=0.3)
        plt.colorbar(s2, ax=ax2, label='|Error| (deg)')
        
        # 3. 右脚 Pitch 误差热力图
        ax3 = plt.subplot(3, 4, 3)
        s3 = ax3.scatter(pitch_cmd_deg, roll_cmd_deg, c=np.abs(error_pitch_r_deg),
                         cmap='Reds', s=50, alpha=0.7, vmin=0, vmax=10)
        ax3.set_xlabel('Pitch Command (deg)')
        ax3.set_ylabel('Roll Command (deg)')
        ax3.set_title('Right Pitch Error Heatmap')
        ax3.grid(True, alpha=0.3)
        plt.colorbar(s3, ax=ax3, label='|Error| (deg)')
        
        # 4. 右脚 Roll 误差热力图
        ax4 = plt.subplot(3, 4, 4)
        s4 = ax4.scatter(pitch_cmd_deg, roll_cmd_deg, c=np.abs(error_roll_r_deg),
                         cmap='Reds', s=50, alpha=0.7, vmin=0, vmax=30)
        ax4.set_xlabel('Pitch Command (deg)')
        ax4.set_ylabel('Roll Command (deg)')
        ax4.set_title('Right Roll Error Heatmap')
        ax4.grid(True, alpha=0.3)
        plt.colorbar(s4, ax=ax4, label='|Error| (deg)')
        
        # 5. 误差分布直方图 - Pitch
        ax5 = plt.subplot(3, 4, 5)
        ax5.hist(error_pitch_l_deg, bins=30, alpha=0.5, label='Left Pitch', density=True, color='blue')
        ax5.hist(error_pitch_r_deg, bins=30, alpha=0.5, label='Right Pitch', density=True, color='red')
        ax5.set_xlabel('Error (deg)')
        ax5.set_ylabel('Density')
        ax5.set_title('Pitch Error Distribution')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        # 6. 误差分布直方图 - Roll
        ax6 = plt.subplot(3, 4, 6)
        ax6.hist(error_roll_l_deg, bins=30, alpha=0.5, label='Left Roll', density=True, color='blue')
        ax6.hist(error_roll_r_deg, bins=30, alpha=0.5, label='Right Roll', density=True, color='red')
        ax6.set_xlabel('Error (deg)')
        ax6.set_ylabel('Density')
        ax6.set_title('Roll Error Distribution')
        ax6.legend()
        ax6.grid(True, alpha=0.3)
        
        # 7. 误差随 Roll 命令变化
        ax7 = plt.subplot(3, 4, 7)
        ax7.scatter(roll_cmd_deg, np.abs(error_roll_l_deg), alpha=0.5, label='Left Roll', s=30)
        ax7.scatter(roll_cmd_deg, np.abs(error_roll_r_deg), alpha=0.5, label='Right Roll', s=30, color='red')
        ax7.set_xlabel('Roll Command (deg)')
        ax7.set_ylabel('|Error| (deg)')
        ax7.set_title('Roll Error vs Roll Command')
        ax7.legend()
        ax7.grid(True, alpha=0.3)
        
        # 8. 误差随 Pitch 命令变化
        ax8 = plt.subplot(3, 4, 8)
        ax8.scatter(pitch_cmd_deg, np.abs(error_pitch_l_deg), alpha=0.5, label='Left Pitch', s=30)
        ax8.scatter(pitch_cmd_deg, np.abs(error_pitch_r_deg), alpha=0.5, label='Right Pitch', s=30, color='red')
        ax8.set_xlabel('Pitch Command (deg)')
        ax8.set_ylabel('|Error| (deg)')
        ax8.set_title('Pitch Error vs Pitch Command')
        ax8.legend()
        ax8.grid(True, alpha=0.3)
        
        # 9. Motor 11 跟踪误差 vs Joint 解算误差（可选）
        ax9 = plt.subplot(3, 4, 9)
        if has_motor_cols:
            motor_error_11 = np.abs(np.degrees(df['p_cmd_11'].values - df['p_fb_11'].values))
            ax9.scatter(motor_error_11, np.abs(error_roll_r_deg), alpha=0.6, s=30)
            ax9.set_xlabel('Motor 11 Tracking Error (deg)')
            ax9.set_ylabel('Right Roll Joint Error (deg)')
            max_val = max(1e-6, np.max(motor_error_11), np.max(np.abs(error_roll_r_deg)))
            ax9.plot([0, max_val], [0, max_val], 'r--', alpha=0.5, label='y=x')
            ax9.legend()
        else:
            ax9.text(0.5, 0.5, 'No p_cmd_11/p_fb_11', ha='center', va='center', transform=ax9.transAxes)
        ax9.set_title('Motor vs Joint Error (Right Roll)')
        ax9.grid(True, alpha=0.3)
        
        # 10. 右脚 Roll 反馈 vs 命令（可选）
        ax10 = plt.subplot(3, 4, 10)
        if has_q_fb_roll_r:
            q_fb_roll_r_deg = np.degrees(df['q_fb_roll_r'].values)
            ax10.plot(np.sort(roll_cmd_deg), np.sort(roll_cmd_deg), 'g--', label='Ideal', linewidth=2)
            ax10.scatter(roll_cmd_deg, q_fb_roll_r_deg, alpha=0.6, s=30, label='Actual')
            ax10.set_xlabel('Roll Command (deg)')
            ax10.set_ylabel('Right Roll Feedback (deg)')
        else:
            ax10.text(0.5, 0.5, 'No q_fb_roll_r', ha='center', va='center', transform=ax10.transAxes)
        ax10.set_title('Right Roll: Command vs Feedback')
        ax10.legend()
        ax10.grid(True, alpha=0.3)
        
        # 11. 左右脚 Roll 误差对比
        ax11 = plt.subplot(3, 4, 11)
        ax11.scatter(np.abs(error_roll_l_deg), np.abs(error_roll_r_deg), alpha=0.6, s=30)
        ax11.set_xlabel('Left Roll |Error| (deg)')
        ax11.set_ylabel('Right Roll |Error| (deg)')
        ax11.set_title('Left vs Right Roll Error')
        max_val = max(np.max(np.abs(error_roll_l_deg)), np.max(np.abs(error_roll_r_deg)))
        ax11.plot([0, max_val], [0, max_val], 'r--', alpha=0.5, label='y=x')
        ax11.legend()
        ax11.grid(True, alpha=0.3)
        
        # 12. 统计信息文本（含解算误差分解）
        ax12 = plt.subplot(3, 4, 12)
        ax12.axis('off')
        stats_text = "误差统计 (deg)\n" + "=" * 40 + "\n\n"
        stats_text += "左脚 Pitch: 均值 %.2f, 标准差 %.2f, 最大|Error| %.2f, RMSE %.2f\n\n" % (
            np.mean(error_pitch_l_deg), np.std(error_pitch_l_deg),
            np.max(np.abs(error_pitch_l_deg)), np.sqrt(np.mean(error_pitch_l_deg ** 2)))
        stats_text += "左脚 Roll:  均值 %.2f, 标准差 %.2f, 最大|Error| %.2f, RMSE %.2f\n\n" % (
            np.mean(error_roll_l_deg), np.std(error_roll_l_deg),
            np.max(np.abs(error_roll_l_deg)), np.sqrt(np.mean(error_roll_l_deg ** 2)))
        stats_text += "右脚 Pitch: 均值 %.2f, 标准差 %.2f, 最大|Error| %.2f, RMSE %.2f\n\n" % (
            np.mean(error_pitch_r_deg), np.std(error_pitch_r_deg),
            np.max(np.abs(error_pitch_r_deg)), np.sqrt(np.mean(error_pitch_r_deg ** 2)))
        stats_text += "右脚 Roll:  均值 %.2f, 标准差 %.2f, 最大|Error| %.2f, RMSE %.2f\n\n" % (
            np.mean(error_roll_r_deg), np.std(error_roll_r_deg),
            np.max(np.abs(error_roll_r_deg)), np.sqrt(np.mean(error_roll_r_deg ** 2)))
        if all(c in df.columns for c in ['q_from_cmd_pitch_l', 'q_from_cmd_roll_l', 'q_from_cmd_pitch_r', 'q_from_cmd_roll_r']):
            solver_pl = np.degrees(np.sqrt(np.mean((df['pitch_cmd'] - df['q_from_cmd_pitch_l'])**2)))
            solver_rl = np.degrees(np.sqrt(np.mean((df['roll_cmd'] - df['q_from_cmd_roll_l'])**2)))
            solver_pr = np.degrees(np.sqrt(np.mean((df['pitch_cmd'] - df['q_from_cmd_pitch_r'])**2)))
            solver_rr = np.degrees(np.sqrt(np.mean((df['roll_cmd'] - df['q_from_cmd_roll_r'])**2)))
            motor_pl = np.degrees(np.sqrt(np.mean((df['q_from_cmd_pitch_l'] - df['q_fb_pitch_l'])**2)))
            motor_rl = np.degrees(np.sqrt(np.mean((df['q_from_cmd_roll_l'] - df['q_fb_roll_l'])**2)))
            motor_pr = np.degrees(np.sqrt(np.mean((df['q_from_cmd_pitch_r'] - df['q_fb_pitch_r'])**2)))
            motor_rr = np.degrees(np.sqrt(np.mean((df['q_from_cmd_roll_r'] - df['q_fb_roll_r'])**2)))
            stats_text += "解算误差分解 RMSE (deg):\n"
            stats_text += "  解算(正逆一致): Lp %.3f Lr %.3f Rp %.3f Rr %.3f\n" % (solver_pl, solver_rl, solver_pr, solver_rr)
            stats_text += "  电机传播:       Lp %.3f Lr %.3f Rp %.3f Rr %.3f\n" % (motor_pl, motor_rl, motor_pr, motor_rr)
        if np.max(np.abs(error_roll_r_deg)) > 20:
            stats_text += "\n⚠️ 右脚 Roll 误差异常大，建议检查 motor_to_joint_position 解算器"
        ax12.text(0.1, 0.5, stats_text, fontsize=9, family='monospace',
                  verticalalignment='center', transform=ax12.transAxes)
        
        plt.tight_layout()
        output_path = os.path.join(output_dir, 'ankle_position_error_analysis.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"✅ 位置误差分析图已保存到: {output_path}")
    
    def plot_velocity_analysis(self, df: pd.DataFrame, output_dir: str):
        """绘制速度曲线：命令 vs 反馈（时间-速度），与力矩图风格一致"""
        if df.empty or 'time' not in df.columns:
            print("速度测试数据为空或缺少 time 列，跳过绘图")
            return
        
        t = df['time'].values
        fig = plt.figure(figsize=(14, 10))
        
        # 左 Pitch：命令 vs 反馈
        ax1 = plt.subplot(2, 2, 1)
        if 'pitch_vel_cmd' in df.columns:
            ax1.plot(t, df['pitch_vel_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'dq_fb_pitch_l' in df.columns:
            ax1.plot(t, df['dq_fb_pitch_l'].values, 'r--', label='Fb L', linewidth=1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Velocity (rad/s)')
        ax1.set_title('Left Pitch Velocity')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 右 Pitch：命令 vs 反馈
        ax2 = plt.subplot(2, 2, 2)
        if 'pitch_vel_cmd' in df.columns:
            ax2.plot(t, df['pitch_vel_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'dq_fb_pitch_r' in df.columns:
            ax2.plot(t, df['dq_fb_pitch_r'].values, 'r--', label='Fb R', linewidth=1)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (rad/s)')
        ax2.set_title('Right Pitch Velocity')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 左 Roll：命令 vs 反馈
        ax3 = plt.subplot(2, 2, 3)
        if 'roll_vel_cmd' in df.columns:
            ax3.plot(t, df['roll_vel_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'dq_fb_roll_l' in df.columns:
            ax3.plot(t, df['dq_fb_roll_l'].values, 'r--', label='Fb L', linewidth=1)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (rad/s)')
        ax3.set_title('Left Roll Velocity')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 右 Roll：命令 vs 反馈
        ax4 = plt.subplot(2, 2, 4)
        if 'roll_vel_cmd' in df.columns:
            ax4.plot(t, df['roll_vel_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'dq_fb_roll_r' in df.columns:
            ax4.plot(t, df['dq_fb_roll_r'].values, 'r--', label='Fb R', linewidth=1)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Velocity (rad/s)')
        ax4.set_title('Right Roll Velocity')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_path = os.path.join(output_dir, 'ankle_solver_velocity_analysis.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"✅ 速度曲线图已保存到: {output_path}")
    
    def plot_torque_analysis(self, df: pd.DataFrame, output_dir: str):
        """绘制力矩曲线：命令 vs 反馈（时间-力矩）"""
        if df.empty or 'time' not in df.columns:
            print("力矩测试数据为空或缺少 time 列，跳过绘图")
            return
        
        t = df['time'].values
        fig = plt.figure(figsize=(14, 10))
        
        # 左 Pitch：命令 vs 反馈
        ax1 = plt.subplot(2, 2, 1)
        if 'pitch_tau_cmd' in df.columns:
            ax1.plot(t, df['pitch_tau_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'tau_fb_pitch_l' in df.columns:
            ax1.plot(t, df['tau_fb_pitch_l'].values, 'r--', label='Fb L', linewidth=1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Torque (N·m)')
        ax1.set_title('Left Pitch Torque')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 右 Pitch：命令 vs 反馈
        ax2 = plt.subplot(2, 2, 2)
        if 'pitch_tau_cmd' in df.columns:
            ax2.plot(t, df['pitch_tau_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'tau_fb_pitch_r' in df.columns:
            ax2.plot(t, df['tau_fb_pitch_r'].values, 'r--', label='Fb R', linewidth=1)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Torque (N·m)')
        ax2.set_title('Right Pitch Torque')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 左 Roll：命令 vs 反馈
        ax3 = plt.subplot(2, 2, 3)
        if 'roll_tau_cmd' in df.columns:
            ax3.plot(t, df['roll_tau_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'tau_fb_roll_l' in df.columns:
            ax3.plot(t, df['tau_fb_roll_l'].values, 'r--', label='Fb L', linewidth=1)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (N·m)')
        ax3.set_title('Left Roll Torque')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 右 Roll：命令 vs 反馈
        ax4 = plt.subplot(2, 2, 4)
        if 'roll_tau_cmd' in df.columns:
            ax4.plot(t, df['roll_tau_cmd'].values, 'b-', label='Cmd', linewidth=1.5)
        if 'tau_fb_roll_r' in df.columns:
            ax4.plot(t, df['tau_fb_roll_r'].values, 'r--', label='Fb R', linewidth=1)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Torque (N·m)')
        ax4.set_title('Right Roll Torque')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        output_path = os.path.join(output_dir, 'ankle_solver_torque_analysis.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"✅ 力矩曲线图已保存到: {output_path}")
    
    def generate_report(self, test_type: str, results: Dict, df: pd.DataFrame) -> str:
        """生成测试报告"""
        report = []
        report.append("="*60)
        report.append(f"脚踝解算验证测试分析报告 - {test_type.upper()}")
        report.append("="*60)
        report.append(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"数据文件: {len(df)} 个数据点")
        report.append("")
        
        if not results:
            report.append("无有效数据")
            return "\n".join(report)
        
        report.append("="*60)
        report.append("统计结果")
        report.append("="*60)
        report.append("")
        report.append("左脚踝:")
        report.append(f"  Pitch - 均值: {results.get('mean_error_pitch_l', 0):.6f} rad, "
                     f"标准差: {results.get('std_error_pitch_l', 0):.6f} rad, "
                     f"最大误差: {results.get('max_error_pitch_l', 0):.6f} rad, "
                     f"RMSE: {results.get('rmse_pitch_l', 0):.6f} rad")
        report.append(f"  Roll  - 均值: {results.get('mean_error_roll_l', 0):.6f} rad, "
                     f"标准差: {results.get('std_error_roll_l', 0):.6f} rad, "
                     f"最大误差: {results.get('max_error_roll_l', 0):.6f} rad, "
                     f"RMSE: {results.get('rmse_roll_l', 0):.6f} rad")
        report.append("")
        report.append("右脚踝:")
        report.append(f"  Pitch - 均值: {results.get('mean_error_pitch_r', 0):.6f} rad, "
                     f"标准差: {results.get('std_error_pitch_r', 0):.6f} rad, "
                     f"最大误差: {results.get('max_error_pitch_r', 0):.6f} rad, "
                     f"RMSE: {results.get('rmse_pitch_r', 0):.6f} rad")
        report.append(f"  Roll  - 均值: {results.get('mean_error_roll_r', 0):.6f} rad, "
                     f"标准差: {results.get('std_error_roll_r', 0):.6f} rad, "
                     f"最大误差: {results.get('max_error_roll_r', 0):.6f} rad, "
                     f"RMSE: {results.get('rmse_roll_r', 0):.6f} rad")
        report.append("")
        
        # 解算误差分解（仅位置测试，且 CSV 含 q_from_cmd 列时）
        if test_type == 'position' and results.get('has_solver_decomposition'):
            report.append("="*60)
            report.append("解算误差分解（总误差 = 解算往返误差 + 电机传播误差）")
            report.append("="*60)
            report.append("  解算往返误差 = q_cmd - motor_to_joint(p_cmd)，理想≈0，反映正逆解一致性")
            report.append("  电机传播误差 = q_from_cmd - q_fb，反映电机未跟到指令经正解后的关节差")
            report.append("")
            report.append("  解算误差 RMSE (rad):")
            report.append(f"    左 Pitch: {results.get('rmse_solver_pitch_l', 0):.6f}  左 Roll: {results.get('rmse_solver_roll_l', 0):.6f}")
            report.append(f"    右 Pitch: {results.get('rmse_solver_pitch_r', 0):.6f}  右 Roll: {results.get('rmse_solver_roll_r', 0):.6f}")
            report.append("  电机传播误差 RMSE (rad):")
            report.append(f"    左 Pitch: {results.get('rmse_motor_pitch_l', 0):.6f}  左 Roll: {results.get('rmse_motor_roll_l', 0):.6f}")
            report.append(f"    右 Pitch: {results.get('rmse_motor_pitch_r', 0):.6f}  右 Roll: {results.get('rmse_motor_roll_r', 0):.6f}")
            report.append("")
        
        # 评估结果
        report.append("="*60)
        report.append("评估结果")
        report.append("="*60)
        
        # 位置测试阈值：1度 = 0.0175 rad
        if test_type == 'position':
            threshold = 0.0175  # 1度
            report.append(f"位置误差阈值: {threshold:.6f} rad (约1度)")
        elif test_type == 'velocity':
            threshold = 0.01  # 0.01 rad/s
            report.append(f"速度误差阈值: {threshold:.6f} rad/s")
        else:  # torque
            threshold = 0.1  # 0.1 N·m
            report.append(f"力矩误差阈值: {threshold:.6f} N·m")
        
        report.append("")
        
        all_pass = True
        for joint in ['pitch_l', 'roll_l', 'pitch_r', 'roll_r']:
            rmse_key = f'rmse_{joint}'
            if rmse_key in results:
                rmse = results[rmse_key]
                status = "✅ 通过" if rmse < threshold else "❌ 失败"
                if rmse >= threshold:
                    all_pass = False
                report.append(f"  {joint.upper()}: RMSE = {rmse:.6f}, {status}")
        
        report.append("")
        if all_pass:
            report.append("🎉 所有测试通过！")
        else:
            report.append("⚠️  部分测试未通过，请检查上述结果")
        
        return "\n".join(report)
    
    def analyze_all(self, test_type: str = 'all'):
        """分析所有测试"""
        # 查找CSV文件
        if test_type == 'all':
            patterns = ['ankle_solver_v17_test_position_*.csv',
                       'ankle_solver_v17_test_velocity_*.csv',
                       'ankle_solver_v17_test_torque_*.csv']
        else:
            patterns = [f'ankle_solver_v17_test_{test_type}_*.csv']
        
        # 输出目录：保存在 data/pics，使用绝对路径确保写入正确
        output_dir = os.path.abspath(os.path.join(self.data_dir, 'pics'))
        os.makedirs(output_dir, exist_ok=True)
        
        all_results = {}
        
        for pattern in patterns:
            csv_files = glob.glob(os.path.join(self.data_dir, pattern))
            if not csv_files:
                continue
            
            # 使用最新的文件；pattern 形如 ankle_solver_v17_test_position_*.csv，取第5段为类型
            csv_file = max(csv_files, key=os.path.getmtime)
            file_type = pattern.split('_')[4]  # position, velocity, or torque
            
            print(f"\n分析 {file_type} 测试数据: {os.path.basename(csv_file)}")
            
            # 加载数据
            if file_type == 'position':
                df = self.load_position_data(csv_file)
                if not df.empty:
                    results = self.analyze_position_test(df)
                    all_results[file_type] = {'results': results, 'data': df}
                    self.plot_position_analysis(df, output_dir)
            elif file_type == 'velocity':
                df = self.load_velocity_data(csv_file)
                if not df.empty:
                    results = self.analyze_velocity_test(df)
                    all_results[file_type] = {'results': results, 'data': df}
                    self.plot_velocity_analysis(df, output_dir)
            elif file_type == 'torque':
                df = self.load_torque_data(csv_file)
                if not df.empty:
                    results = self.analyze_torque_test(df)
                    all_results[file_type] = {'results': results, 'data': df}
                    self.plot_torque_analysis(df, output_dir)
        
        # 生成报告
        print("\n" + "="*60)
        print("分析报告")
        print("="*60)
        
        for test_type, data in all_results.items():
            report = self.generate_report(test_type, data['results'], data['data'])
            print("\n" + report)
        
        # 保存报告
        report_file = os.path.join(self.data_dir, 'ankle_solver_analysis_report.txt')
        with open(report_file, 'w', encoding='utf-8') as f:
            for test_type, data in all_results.items():
                report = self.generate_report(test_type, data['results'], data['data'])
                f.write(report + "\n\n")
        
        print(f"\n✅ 报告已保存到: {report_file}")
        print(f"✅ 图表已保存到: {output_dir}")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='脚踝解算验证测试分析')
    parser.add_argument('data_dir', nargs='?', default=None,
                       help='数据目录路径 (默认: 自动查找)')
    parser.add_argument('--test-type', type=str, default='all',
                       choices=['position', 'velocity', 'torque', 'all'],
                       help='测试类型 (默认: all)')
    
    args = parser.parse_args()
    
    # 创建分析器
    analyzer = AnkleSolverAnalyzer(args.data_dir)
    
    # 检查数据目录是否存在
    if not os.path.exists(analyzer.data_dir):
        print(f"错误：数据目录不存在: {analyzer.data_dir}")
        sys.exit(1)
    
    # 分析所有测试
    analyzer.analyze_all(args.test_type)


if __name__ == "__main__":
    main()
