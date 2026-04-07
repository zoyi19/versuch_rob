#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机跟随测试对称性评估分析脚本

该脚本读取电机跟随测试生成的数据文件，进行对称性评估分析，
包括误差方差、频谱相似度和对称性指标的计算。

使用方法:
    python3 symmetry_analysis.py [数据目录路径]

如果不指定路径，默认使用当前目录下的 file 文件夹。
"""

import os
import sys
import glob
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
from typing import List, Tuple, Dict
import argparse
from datetime import datetime


class TimedValue:
    """带时间戳的数据点"""
    def __init__(self, timestamp: float, value: float):
        self.timestamp = timestamp
        self.value = value


class SymmetryMetrics:
    """对称性评估指标"""
    def __init__(self):
        self.amplitude_ratio = 0.0      # 幅值比 (右/左)
        self.phase_difference = 0.0     # 相位差 (弧度)
        self.amplitude_symmetry = 0.0   # 幅值对称性 (0-1, 1为完全对称)
        self.phase_symmetry = 0.0       # 相位对称性 (0-1, 1为完全对称)
        self.overall_symmetry = 0.0     # 总体对称性 (0-1, 1为完全对称)


class TestResult:
    """测试结果"""
    def __init__(self):
        self.pair_index = 0
        self.left_joint = 0
        self.right_joint = 0
        self.left_error_variance = 0.0
        self.right_error_variance = 0.0
        self.similarity = 0.0
        self.symmetry = 0.0
        self.is_error = False
        self.status = ""


class SymmetryAnalyzer:
    """
    对称性分析器
    
    测试阈值参数说明：
    - ERROR_VAR_THRESHOLD: 误差方差阈值，超过此值认为关节跟随性能差
    - SIMILARITY_THRESHOLD: 相似性阈值，低于此值认为左右关节运动不相似
    - SYMMETRY_THRESHOLD: 对称性阈值，低于此值认为关节运动不对称
    
    修改方法：直接修改下面的类变量值
    """
    
    # 测试阈值参数 - 可根据需要修改
    ERROR_VAR_THRESHOLD = 0.2      # 误差方差阈值
    SIMILARITY_THRESHOLD = 0.95    # 相似性阈值
    SYMMETRY_THRESHOLD = 0.7      # 对称性阈值
    
    def __init__(self, data_dir: str = None, robot_version: int = None):
        if data_dir is None:
            self.data_dir = self.find_data_directory()
        else:
            self.data_dir = data_dir
        
        # 如果没有指定版本，尝试从环境变量读取
        if robot_version is None:
            self.robot_version = self.get_robot_version_from_env()
        else:
            self.robot_version = robot_version
            
        self.results = []
        
        # 根据机器人版本设置关节名称映射
        self.joint_names = self.get_joint_names(self.robot_version)
    
    def get_robot_version_from_env(self) -> int:
        """从环境变量获取机器人版本"""
        import os
        
        # 从环境变量读取ROBOT_VERSION
        robot_version_env = os.getenv("ROBOT_VERSION")
        if robot_version_env:
            try:
                version = int(robot_version_env)
                print(f"从环境变量获取ROBOT_VERSION: {version}")
                return version
            except ValueError:
                print(f"警告：环境变量ROBOT_VERSION值无效: {robot_version_env}")
        
        # 如果环境变量不存在或无效，使用默认值
        print("警告：未设置ROBOT_VERSION环境变量，使用默认版本50")
        return 50
    
    def get_joint_names(self, robot_version: int) -> Dict[int, str]:
        """根据机器人版本获取关节名称映射"""
        if robot_version == 13:
            # 版本13：腰部(0) -> 左腿(1-6) -> 右腿(7-12) -> 左臂(13-16) -> 右臂(17-20) -> 头部(21-22)
            return {
                0: "waist_yaw_joint",
                1: "leg_l1_joint", 2: "leg_l2_joint", 3: "leg_l3_joint", 4: "leg_l4_joint", 5: "leg_l5_joint", 6: "leg_l6_joint",
                7: "leg_r1_joint", 8: "leg_r2_joint", 9: "leg_r3_joint", 10: "leg_r4_joint", 11: "leg_r5_joint", 12: "leg_r6_joint",
                13: "zarm_l1_joint", 14: "zarm_l2_joint", 15: "zarm_l3_joint", 16: "zarm_l4_joint",
                17: "zarm_r1_joint", 18: "zarm_r2_joint", 19: "zarm_r3_joint", 20: "zarm_r4_joint",
                21: "zhead_1_joint", 22: "zhead_2_joint"
            }
        elif robot_version >= 50:
            # 版本50+：左腿(0-5) -> 右腿(6-11) -> 腰部(12) -> 左臂(13-19) -> 右臂(20-26) -> 头部(27-28)
            return {
                0: "leg_l1_joint", 1: "leg_l2_joint", 2: "leg_l3_joint", 3: "leg_l4_joint", 4: "leg_l5_joint", 5: "leg_l6_joint",
                6: "leg_r1_joint", 7: "leg_r2_joint", 8: "leg_r3_joint", 9: "leg_r4_joint", 10: "leg_r5_joint", 11: "leg_r6_joint",
                12: "waist_yaw_joint",
                13: "zarm_l1_joint", 14: "zarm_l2_joint", 15: "zarm_l3_joint", 16: "zarm_l4_joint", 17: "zarm_l5_joint", 18: "zarm_l6_joint", 19: "zarm_l7_joint",
                20: "zarm_r1_joint", 21: "zarm_r2_joint", 22: "zarm_r3_joint", 23: "zarm_r4_joint", 24: "zarm_r5_joint", 25: "zarm_r6_joint", 26: "zarm_r7_joint",
                27: "zhead_1_joint", 28: "zhead_2_joint"
            }
        else:
            # 其他版本（如42, 45等）：左腿(0-5) -> 右腿(6-11) -> 左臂(12-18) -> 右臂(19-25) -> 头部(26-27) （无腰部）
            return {
                0: "leg_l1_joint", 1: "leg_l2_joint", 2: "leg_l3_joint", 3: "leg_l4_joint", 4: "leg_l5_joint", 5: "leg_l6_joint",
                6: "leg_r1_joint", 7: "leg_r2_joint", 8: "leg_r3_joint", 9: "leg_r4_joint", 10: "leg_r5_joint", 11: "leg_r6_joint",
                12: "zarm_l1_joint", 13: "zarm_l2_joint", 14: "zarm_l3_joint", 15: "zarm_l4_joint", 16: "zarm_l5_joint", 17: "zarm_l6_joint", 18: "zarm_l7_joint",
                19: "zarm_r1_joint", 20: "zarm_r2_joint", 21: "zarm_r3_joint", 22: "zarm_r4_joint", 23: "zarm_r5_joint", 24: "zarm_r6_joint", 25: "zarm_r7_joint",
                26: "zhead_1_joint", 27: "zhead_2_joint"
            }
    
    def get_joint_name(self, joint_id: int) -> str:
        """获取关节名称"""
        return self.joint_names.get(joint_id, f"joint_{joint_id}")
    
    def find_data_directory(self) -> str:
        """自动查找数据目录"""
        # 获取当前脚本所在目录
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 可能的数据目录路径
        possible_paths = [
            os.path.join(current_dir, 'file'),  # 脚本所在目录下的file
            os.path.join(current_dir, 'data'),  # 脚本所在目录下的data
            os.path.join(current_dir, '..', 'file'),  # 上级目录的file
            os.path.join(current_dir, '..', 'data'),  # 上级目录的data
            os.path.join(current_dir, '..', '..', 'file'),  # 上上级目录的file
            os.path.join(current_dir, '..', '..', 'data'),  # 上上级目录的data
            './file',  # 当前工作目录下的file
            './data',  # 当前工作目录下的data
        ]
        
        # 查找包含motorPair_*.txt文件的目录
        for path in possible_paths:
            if os.path.exists(path):
                # 检查是否包含电机对数据文件
                motor_pair_files = glob.glob(os.path.join(path, "motorPair_*.txt"))
                if motor_pair_files:
                    print(f"找到数据目录: {path}")
                    return path
        
        # 如果没找到，返回默认路径
        print("警告：未找到数据目录，使用默认路径 ./file")
        return "./file"
        
    def load_data_file(self, filepath: str) -> List[TimedValue]:
        """加载数据文件"""
        data = []
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    parts = line.strip().split('\t')
                    if len(parts) >= 2:
                        timestamp = float(parts[0])
                        value = float(parts[1])
                        data.append(TimedValue(timestamp, value))
        except Exception as e:
            print(f"错误：无法读取文件 {filepath}: {e}")
        return data
    
    def load_motor_pair_file(self, filepath: str) -> Dict[str, List[TimedValue]]:
        """加载电机对数据文件（新格式）"""
        data = {
            'left_input': [],
            'left_response': [],
            'right_input': [],
            'right_response': []
        }
        try:
            with open(filepath, 'r') as f:
                lines = f.readlines()
                if len(lines) < 2:
                    return data
                
                # 跳过表头
                for line in lines[1:]:
                    parts = line.strip().split()
                    if len(parts) >= 5:
                        timestamp = float(parts[0])
                        left_input = float(parts[1])
                        left_response = float(parts[2])
                        right_input = float(parts[3])
                        right_response = float(parts[4])
                        
                        data['left_input'].append(TimedValue(timestamp, left_input))
                        data['left_response'].append(TimedValue(timestamp, left_response))
                        data['right_input'].append(TimedValue(timestamp, right_input))
                        data['right_response'].append(TimedValue(timestamp, right_response))
        except Exception as e:
            print(f"加载电机对文件 {filepath} 失败: {e}")
        return data
    
    def calculate_error_variance(self, input_data: List[TimedValue], 
                                output_data: List[TimedValue]) -> float:
        """计算归一化误差方差 - 除以input_data的动作幅度"""
        if len(input_data) != len(output_data) or len(input_data) == 0:
            return 0.0
        
        # 过滤异常数据（绝对值大于1000或包含NaN/Inf）
        valid_pairs = []
        for i in range(len(input_data)):
            input_val = input_data[i].value
            output_val = output_data[i].value
            
            # 检查数据有效性
            if (abs(input_val) < 1000 and abs(output_val) < 1000 and 
                not (np.isnan(input_val) or np.isnan(output_val) or 
                     np.isinf(input_val) or np.isinf(output_val))):
                valid_pairs.append((input_val, output_val))
        
        # 如果没有有效数据，返回一个大的误差值
        if len(valid_pairs) == 0:
            return 100.0
        
        # 如果有效数据太少（少于10%），也返回大的误差值
        if len(valid_pairs) < len(input_data) * 0.1:
            return 100.0
        
        # 使用有效数据计算
        input_values = [pair[0] for pair in valid_pairs]
        output_values = [pair[1] for pair in valid_pairs]
        
        # 计算input_data的动作幅度（最大值-最小值）
        input_amplitude = max(input_values) - min(input_values)
        
        # 如果动作幅度太小（小于1e-6），使用原始误差方差
        if input_amplitude < 1e-6:
            # 计算原始误差方差
            error_sum = 0.0
            for i in range(len(valid_pairs)):
                error = valid_pairs[i][0] - valid_pairs[i][1]
                error_sum += error * error
            return error_sum / len(valid_pairs)
        
        # 计算误差方差
        error_sum = 0.0
        for i in range(len(valid_pairs)):
            error = valid_pairs[i][0] - valid_pairs[i][1]
            error_sum += error * error
        
        # 归一化：除以动作幅度的平方
        normalized_variance = (error_sum / len(valid_pairs)) / (input_amplitude * input_amplitude)
        
        # 限制最大值为100，避免无穷大
        return min(normalized_variance, 100.0)
    
    def calculate_spectral_similarity(self, signal1: List[TimedValue], 
                                    signal2: List[TimedValue]) -> float:
        """计算频谱相似度 - 改进版本"""
        if len(signal1) != len(signal2) or len(signal1) == 0:
            return 0.0
        
        # 提取信号值
        values1 = np.array([s.value for s in signal1])
        values2 = np.array([s.value for s in signal2])
        
        # 检查数据有效性
        if np.all(values1 == 0) and np.all(values2 == 0):
            return 1.0  # 两个信号都为0，认为完全相似
        
        if np.all(values1 == 0) or np.all(values2 == 0):
            return 0.0  # 一个为0一个不为0，认为不相似
        
        # 检查是否有常数信号
        if np.std(values1) == 0 and np.std(values2) == 0:
            # 两个都是常数，比较数值是否接近
            return 1.0 if abs(np.mean(values1) - np.mean(values2)) < 0.01 else 0.0
        
        if np.std(values1) == 0 or np.std(values2) == 0:
            return 0.0  # 一个是常数一个不是，认为不相似
        
        try:
            # 使用归一化互相关 (NCC) 作为相似性度量
            # 这种方法对信号的幅度变化不敏感，更适合比较波形形状
            
            # 中心化数据
            mean1 = np.mean(values1)
            mean2 = np.mean(values2)
            centered1 = values1 - mean1
            centered2 = values2 - mean2
            
            # 计算归一化互相关
            numerator = np.sum(centered1 * centered2)
            denominator = np.sqrt(np.sum(centered1 ** 2) * np.sum(centered2 ** 2))
            
            if denominator > 0:
                ncc = numerator / denominator
                # 使用NCC的绝对值，这样反相信号也能得到高相似性分数
                # 将|NCC|从[0,1]映射到[0,1]
                similarity = abs(ncc)
                return max(0.0, min(1.0, similarity))
            else:
                return 0.0
                
        except Exception as e:
            print(f"相似性计算异常: {e}")
            return 0.0
    
    def calculate_symmetry_metrics(self, left_error_signal: List[TimedValue], 
                                 right_error_signal: List[TimedValue]) -> SymmetryMetrics:
        """计算对称性指标"""
        metrics = SymmetryMetrics()
        
        if len(left_error_signal) == 0 or len(right_error_signal) == 0:
            return metrics
        
        # 确保两个信号长度相同
        min_size = min(len(left_error_signal), len(right_error_signal))
        if min_size == 0:
            return metrics
        
        # 1. 计算左右误差信号的绝对值
        left_abs_errors = [abs(s.value) for s in left_error_signal[:min_size]]
        right_abs_errors = [abs(s.value) for s in right_error_signal[:min_size]]
        
        # 2. 计算左右误差绝对值的差分
        left_diff = []
        right_diff = []
        for i in range(1, min_size):
            left_diff.append(left_abs_errors[i] - left_abs_errors[i-1])
            right_diff.append(right_abs_errors[i] - right_abs_errors[i-1])
        
        if len(left_diff) == 0 or len(right_diff) == 0:
            return metrics
        
        # 3. 归一化差分序列
        left_diff_mean = np.mean(left_diff)
        right_diff_mean = np.mean(right_diff)
        left_diff_std = np.std(left_diff)
        right_diff_std = np.std(right_diff)
        
        left_diff_normalized = []
        right_diff_normalized = []
        for i in range(len(left_diff)):
            if left_diff_std > 0:
                left_diff_normalized.append((left_diff[i] - left_diff_mean) / left_diff_std)
            else:
                left_diff_normalized.append(0.0)
            
            if right_diff_std > 0:
                right_diff_normalized.append((right_diff[i] - right_diff_mean) / right_diff_std)
            else:
                right_diff_normalized.append(0.0)
        
        # 4. 计算归一化后差分差值的绝对值，然后做积分
        symmetry_integral = 0.0
        for i in range(len(left_diff_normalized)):
            diff_difference = left_diff_normalized[i] - right_diff_normalized[i]
            symmetry_integral += abs(diff_difference)
        
        # 5. 计算左右差分各自的积分（用于归一化）
        left_integral = sum(abs(x) for x in left_diff_normalized)
        right_integral = sum(abs(x) for x in right_diff_normalized)
        
        # 6. 计算数据点数量
        data_point_count = len(left_diff_normalized)
        
        # 7. 计算对称性积分率
        symmetry_integral_rate = symmetry_integral / data_point_count if data_point_count > 0 else 0.0
        
        # 8. 基于归一化差分差值积分的对称性评估
        max_possible_integral = left_integral + right_integral
        if max_possible_integral > 0:
            normalized_symmetry_integral = symmetry_integral / max_possible_integral
            metrics.overall_symmetry = 1.0 - normalized_symmetry_integral
        else:
            metrics.overall_symmetry = 1.0
        
        # 9. 相位差使用对称性积分率
        metrics.phase_difference = symmetry_integral_rate
        
        # 10. 保持兼容性，设置其他字段
        metrics.amplitude_ratio = 1.0
        metrics.amplitude_symmetry = metrics.overall_symmetry
        metrics.phase_symmetry = metrics.overall_symmetry
        
        return metrics
    
    def analyze_test_pair(self, pair_index: int) -> TestResult:
        """分析单个测试对"""
        result = TestResult()
        result.pair_index = pair_index
        
        # 查找对应的电机对数据文件（新格式）
        motor_pair_files = glob.glob(os.path.join(self.data_dir, f"motorPair_{pair_index}_*.txt"))
        
        if len(motor_pair_files) == 0:
            # 尝试旧格式
            input_files = glob.glob(os.path.join(self.data_dir, f"inputData_{pair_index}_*.txt"))
            response_files = glob.glob(os.path.join(self.data_dir, f"responseData_{pair_index}_*.txt"))
            
            if len(input_files) < 2 or len(response_files) < 2:
                print(f"警告：测试对 {pair_index} 的数据文件不完整")
                return result
            
            # 加载旧格式数据
            l_input_data = self.load_data_file(input_files[0])
            r_input_data = self.load_data_file(input_files[1])
            l_response_data = self.load_data_file(response_files[0])
            r_response_data = self.load_data_file(response_files[1])
            
            # 从文件名提取关节ID
            try:
                result.left_joint = int(os.path.basename(input_files[0]).split('_')[2].split('.')[0])
                result.right_joint = int(os.path.basename(input_files[1]).split('_')[2].split('.')[0])
            except:
                result.left_joint = pair_index * 2
                result.right_joint = pair_index * 2 + 1
        else:
            # 加载新格式数据
            pair_data = self.load_motor_pair_file(motor_pair_files[0])
            l_input_data = pair_data['left_input']
            l_response_data = pair_data['left_response']
            r_input_data = pair_data['right_input']
            r_response_data = pair_data['right_response']
            
            # 从文件名提取关节ID
            try:
                filename = os.path.basename(motor_pair_files[0])
                # motorPair_1_L1_R7.txt -> 提取L1和R7
                parts = filename.split('_')
                left_part = parts[2]  # L1
                right_part = parts[3]  # R7.txt
                result.left_joint = int(left_part[1:])  # 去掉L
                result.right_joint = int(right_part[1:].split('.')[0])  # 去掉R和.txt
            except:
                result.left_joint = pair_index * 2
                result.right_joint = pair_index * 2 + 1
        
        # 计算性能指标
        result.left_error_variance = self.calculate_error_variance(l_input_data, l_response_data)
        result.right_error_variance = self.calculate_error_variance(r_input_data, r_response_data)
        
        result.similarity = self.calculate_spectral_similarity(l_response_data, r_response_data)
        
        # 创建误差信号用于对称性检测
        l_error_signal = []
        r_error_signal = []
        
        min_size = min(len(l_input_data), len(l_response_data), 
                      len(r_input_data), len(r_response_data))
        
        for i in range(min_size):
            l_error = TimedValue(l_input_data[i].timestamp, 
                               l_input_data[i].value - l_response_data[i].value)
            r_error = TimedValue(r_input_data[i].timestamp, 
                               r_input_data[i].value - r_response_data[i].value)
            l_error_signal.append(l_error)
            r_error_signal.append(r_error)
        
        # 计算对称性指标
        symmetry_metrics = self.calculate_symmetry_metrics(l_error_signal, r_error_signal)
        result.symmetry = symmetry_metrics.overall_symmetry
        
        # 判断是否有异常
        if (result.left_error_variance > self.ERROR_VAR_THRESHOLD or 
            result.right_error_variance > self.ERROR_VAR_THRESHOLD or
            result.similarity < self.SIMILARITY_THRESHOLD or 
            result.symmetry < self.SYMMETRY_THRESHOLD):
            result.is_error = True
            result.status = "异常"
        else:
            result.status = "正常"
        
        return result
    
    def analyze_all_tests(self) -> List[TestResult]:
        """分析所有测试"""
        self.results = []
        
        # 查找所有测试对（优先查找新格式）
        motor_pair_files = glob.glob(os.path.join(self.data_dir, "motorPair_*.txt"))
        pair_indices = set()
        
        if motor_pair_files:
            # 使用新格式文件
            for file in motor_pair_files:
                try:
                    # motorPair_1_L1_R7.txt -> 提取1
                    pair_index = int(os.path.basename(file).split('_')[1])
                    pair_indices.add(pair_index)
                except:
                    continue
        else:
            # 回退到旧格式文件
            input_files = glob.glob(os.path.join(self.data_dir, "inputData_*.txt"))
            for file in input_files:
                try:
                    pair_index = int(os.path.basename(file).split('_')[1])
                    pair_indices.add(pair_index)
                except:
                    continue
        
        print(f"发现 {len(pair_indices)} 个测试对")
        
        if len(pair_indices) == 0:
            print("没有找到测试数据")
            return self.results
        
        # 分析每个测试对
        for pair_index in sorted(pair_indices):
            print(f"分析测试对 {pair_index}...")
            result = self.analyze_test_pair(pair_index)
            self.results.append(result)
        
        return self.results
    
    def generate_report(self) -> str:
        """生成测试报告"""
        if not self.results:
            return "没有测试结果"
        
        report = []
        report.append("========= 电机跟随测试对称性评估报告 =========")
        report.append(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"机器人版本: {self.robot_version}")
        report.append(f"数据目录: {self.data_dir}")
        report.append(f"总测试对数: {len(self.results)}")
        report.append("")
        report.append("========= 测试标准 =========")
        report.append(f"误差方差阈值: {self.ERROR_VAR_THRESHOLD} (超过此值认为跟随性能差)")
        report.append(f"相似性阈值: {self.SIMILARITY_THRESHOLD} (低于此值认为左右关节不相似)")
        report.append(f"对称性阈值: {self.SYMMETRY_THRESHOLD} (低于此值认为关节运动不对称)")
        report.append("")
        
        # 统计结果
        error_count = sum(1 for r in self.results if r.is_error)
        total_left_variance = sum(r.left_error_variance for r in self.results)
        total_right_variance = sum(r.right_error_variance for r in self.results)
        total_similarity = sum(r.similarity for r in self.results)
        total_symmetry = sum(r.symmetry for r in self.results)
        
        report.append("========= 详细结果 =========")
        for result in self.results:
            left_joint_name = self.get_joint_name(result.left_joint)
            right_joint_name = self.get_joint_name(result.right_joint)
            report.append(f"测试对 {result.pair_index} (关节 {result.left_joint}-{result.right_joint}): {result.status}")
            report.append(f"  左关节: {left_joint_name}")
            report.append(f"  右关节: {right_joint_name}")
            report.append(f"  左误差方差: {result.left_error_variance:.4f}")
            report.append(f"  右误差方差: {result.right_error_variance:.4f}")
            report.append(f"  相似性: {result.similarity:.4f}")
            report.append(f"  对称性: {result.symmetry:.4f}")
            report.append("")
        
        report.append("========= 统计结果 =========")
        report.append(f"异常测试对数: {error_count}/{len(self.results)}")
        report.append(f"平均左误差方差: {total_left_variance/len(self.results):.4f}")
        report.append(f"平均右误差方差: {total_right_variance/len(self.results):.4f}")
        report.append(f"平均相似性: {total_similarity/len(self.results):.4f}")
        report.append(f"平均对称性: {total_symmetry/len(self.results):.4f}")
        
        # 添加失败关节详细总结
        if error_count > 0:
            report.append("")
            report.append("========= 测试失败关节详情 =========")
            failed_pairs = [r for r in self.results if r.is_error]
            for i, result in enumerate(failed_pairs, 1):
                left_joint_name = self.get_joint_name(result.left_joint)
                right_joint_name = self.get_joint_name(result.right_joint)
                report.append(f"{i}. 测试对 {result.pair_index} (关节 {result.left_joint}-{result.right_joint})")
                report.append(f"   左关节: {left_joint_name} (误差方差: {result.left_error_variance:.4f})")
                report.append(f"   右关节: {right_joint_name} (误差方差: {result.right_error_variance:.4f})")
                report.append(f"   相似性: {result.similarity:.4f} (阈值: {self.SIMILARITY_THRESHOLD})")
                report.append(f"   对称性: {result.symmetry:.4f} (阈值: {self.SYMMETRY_THRESHOLD})")
                
                # 分析失败原因
                failure_reasons = []
                if result.left_error_variance >= 100.0:
                    failure_reasons.append("左关节数据异常（包含极大值或无效数据）")
                elif result.left_error_variance > self.ERROR_VAR_THRESHOLD:
                    failure_reasons.append(f"左关节误差方差过高({result.left_error_variance:.4f})")
                
                if result.right_error_variance >= 100.0:
                    failure_reasons.append("右关节数据异常（包含极大值或无效数据）")
                elif result.right_error_variance > self.ERROR_VAR_THRESHOLD:
                    failure_reasons.append(f"右关节误差方差过高({result.right_error_variance:.4f})")
                
                if result.similarity < self.SIMILARITY_THRESHOLD:
                    failure_reasons.append(f"相似性不足({result.similarity:.4f})")
                if result.symmetry < self.SYMMETRY_THRESHOLD:
                    failure_reasons.append(f"对称性不足({result.symmetry:.4f})")
                
                report.append(f"   失败原因: {', '.join(failure_reasons)}")
                report.append("")
        else:
            report.append("")
            report.append("🎉 所有关节测试通过！")
        
        return "\n".join(report)
    
    def save_report(self, filename: str = None):
        """保存报告到文件"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"symmetry_analysis_report_{timestamp}.txt"
        
        report_content = self.generate_report()
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(report_content)
        
        print(f"报告已保存到: {filename}")
    
    def plot_results(self, save_plots: bool = True):
        """绘制电机对波形图（与drawWaveform.py格式一致）"""
        if not self.results:
            print("没有数据可以绘制")
            return
        
        print(f"\n🎨 开始绘制 {len(self.results)} 个电机对的波形图...")
        
        # 为每个电机对绘制波形图
        for result in self.results:
            self.plot_motor_pair_waveform(result, save_plots)
        
        print(f"\n🎉 绘图完成！共生成 {len(self.results)} 张图片")
        if save_plots:
            data_parent_dir = os.path.dirname(self.data_dir)
            pics_dir = os.path.join(data_parent_dir, 'pics')
            print(f"📁 图片保存在: {pics_dir}")
    
    def plot_motor_pair_waveform(self, result: 'TestResult', save_plots: bool = True):
        """绘制单个电机对的波形图"""
        # 查找对应的电机对数据文件
        motor_pair_files = glob.glob(os.path.join(self.data_dir, f"motorPair_{result.pair_index}_*.txt"))
        
        if len(motor_pair_files) == 0:
            print(f"警告：测试对 {result.pair_index} 的数据文件不存在")
            return
        
        # 加载电机对数据
        pair_data = self.load_motor_pair_file(motor_pair_files[0])
        
        if not pair_data['left_input'] or not pair_data['right_input']:
            print(f"警告：测试对 {result.pair_index} 的数据为空")
            return
        
        # 创建子图：左电机和右电机
        left_joint_name = self.get_joint_name(result.left_joint)
        right_joint_name = self.get_joint_name(result.right_joint)
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle(f'Motor Pair {result.pair_index} (L:{result.left_joint} {left_joint_name}, R:{result.right_joint} {right_joint_name})', fontsize=14)
        
        # 绘制左电机波形
        ax_left = axes[0]
        left_timestamps = [tv.timestamp for tv in pair_data['left_input']]
        left_input_values = [tv.value for tv in pair_data['left_input']]
        left_response_values = [tv.value for tv in pair_data['left_response']]
        
        ax_left.plot(left_timestamps, left_input_values, 'b-', label='Target Position', linewidth=1)
        ax_left.plot(left_timestamps, left_response_values, 'r--', label='Actual Position', linewidth=1)
        ax_left.set_title(f'Left Motor {result.left_joint}')
        ax_left.set_xlabel('Time (ms)')
        ax_left.set_ylabel('Signal Value')
        ax_left.grid(True)
        ax_left.legend()
        
        # 绘制右电机波形
        ax_right = axes[1]
        right_timestamps = [tv.timestamp for tv in pair_data['right_input']]
        right_input_values = [tv.value for tv in pair_data['right_input']]
        right_response_values = [tv.value for tv in pair_data['right_response']]
        
        ax_right.plot(right_timestamps, right_input_values, 'b-', label='Target Position', linewidth=1)
        ax_right.plot(right_timestamps, right_response_values, 'r--', label='Actual Position', linewidth=1)
        ax_right.set_title(f'Right Motor {result.right_joint}')
        ax_right.set_xlabel('Time (ms)')
        ax_right.set_ylabel('Signal Value')
        ax_right.grid(True)
        ax_right.legend()
        
        # 调整子图布局
        plt.tight_layout()
        
        if save_plots:
            # 创建输出目录（与数据目录平级的pics文件夹）
            data_parent_dir = os.path.dirname(self.data_dir)
            output_dir = os.path.join(data_parent_dir, 'pics')
            os.makedirs(output_dir, exist_ok=True)
            
            # 保存图片
            output_path = os.path.join(output_dir, f'motor_pair_{result.pair_index}_L{result.left_joint}_R{result.right_joint}.png')
            plt.savefig(output_path, dpi=300, bbox_inches='tight')
            plt.close()
            print(f"✅ 已保存电机对 {result.pair_index} 图片到: {output_path}")
        else:
            plt.show()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='电机跟随测试对称性评估分析')
    parser.add_argument('data_dir', nargs='?', default=None,
                       help='数据目录路径 (默认: 自动查找)')
    parser.add_argument('--robot-version', type=int, default=None,
                       help='机器人版本 (默认: 从环境变量ROBOT_VERSION读取，如果未设置则使用50)')
    parser.add_argument('--no-plot', action='store_true',
                       help='不显示图表')
    parser.add_argument('--no-save', action='store_true',
                       help='不保存报告和图表')
    
    args = parser.parse_args()
    
    # 创建分析器（自动查找数据目录）
    analyzer = SymmetryAnalyzer(args.data_dir, args.robot_version)
    
    # 检查数据目录是否存在
    if not os.path.exists(analyzer.data_dir):
        print(f"错误：数据目录不存在: {analyzer.data_dir}")
        sys.exit(1)
    
    # 分析所有测试
    print("开始分析电机跟随测试数据...")
    results = analyzer.analyze_all_tests()
    
    if not results:
        print("没有找到测试数据")
        sys.exit(1)
    
    # 生成并显示报告
    report = analyzer.generate_report()
    print("\n" + report)
    
    # 在控制台输出失败关节的简要总结
    if analyzer.results:
        failed_pairs = [r for r in analyzer.results if r.is_error]
        if failed_pairs:
            print("\n" + "="*60)
            print("⚠️  测试失败关节总结:")
            print("="*60)
            for i, result in enumerate(failed_pairs, 1):
                left_joint_name = analyzer.get_joint_name(result.left_joint)
                right_joint_name = analyzer.get_joint_name(result.right_joint)
                print(f"{i}. 测试对 {result.pair_index}: {left_joint_name} - {right_joint_name}")
                print(f"   左关节误差方差: {result.left_error_variance:.4f}, 右关节误差方差: {result.right_error_variance:.4f}")
                print(f"   相似性: {result.similarity:.4f}, 对称性: {result.symmetry:.4f}")
            print("="*60)
            print(f"总共 {len(failed_pairs)} 对关节测试失败，请检查上述关节")
        else:
            print("\n" + "="*60)
            print("🎉 所有关节测试通过！")
            print("="*60)
    
    # 保存报告
    if not args.no_save:
        analyzer.save_report()
    
    # 绘制图表
    if not args.no_plot:
        analyzer.plot_results(save_plots=not args.no_save)


if __name__ == "__main__":
    main()
