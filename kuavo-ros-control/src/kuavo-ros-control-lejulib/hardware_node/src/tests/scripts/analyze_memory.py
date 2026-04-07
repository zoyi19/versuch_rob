#!/usr/bin/env python3
"""
内存监控数据分析脚本
用于分析memory_monitor.sh生成的CSV文件
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def analyze_memory_data(csv_file):
    """分析内存监控数据"""

    if not os.path.exists(csv_file):
        print(f"错误：找不到文件 {csv_file}")
        return

    # 读取CSV文件
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"读取CSV文件失败：{e}")
        return

    if df.empty:
        print("CSV文件为空")
        return

    # 转换时间戳列为datetime类型
    df['timestamp'] = pd.to_datetime(df['timestamp'])

    # 转换数值列为float类型
    numeric_columns = ['rss_mb', 'vsz_mb', 'mem_percent']
    for col in numeric_columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')

    print("=== 内存监控数据分析 ===")
    print(f"数据文件：{csv_file}")
    print(f"数据点数量：{len(df)}")
    print(f"时间范围：{df['timestamp'].min()} 到 {df['timestamp'].max()}")
    print()

    # 基本统计信息
    print("=== 基本统计信息 ===")
    for col in numeric_columns:
        if df[col].notna().any():
            print(f"{col}:")
            print(f"  最小值: {df[col].min():.2f}")
            print(f"  最大值: {df[col].max():.2f}")
            print(f"  平均值: {df[col].mean():.2f}")
            print(f"  标准差: {df[col].std():.2f}")
            print()

    # 进程信息
    print("=== 进程信息 ===")
    unique_pids = df['pid'].unique()
    print(f"监控的进程PID：{unique_pids}")
    print(f"进程命令：{df['command'].iloc[0]}")
    print()

    # 创建图表
    try:
        create_plots(df, csv_file)
    except Exception as e:
        print(f"创建图表时出错：{e}")

def create_plots(df, csv_file):
    """创建内存使用趋势图"""

    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))

    # RSS内存使用趋势
    ax1.plot(df['timestamp'], df['rss_mb'], 'b-', linewidth=2)
    ax1.set_ylabel('RSS (MB)')
    ax1.set_title(f'内存使用趋势 - PID {df["pid"].iloc[0]}')
    ax1.grid(True, alpha=0.3)

    # VSZ内存使用趋势
    ax2.plot(df['timestamp'], df['vsz_mb'], 'r-', linewidth=2)
    ax2.set_ylabel('VSZ (MB)')
    ax2.grid(True, alpha=0.3)

    # 内存使用率趋势
    ax3.plot(df['timestamp'], df['mem_percent'], 'g-', linewidth=2)
    ax3.set_ylabel('内存使用率 (%)')
    ax3.set_xlabel('时间')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    # 保存图表
    output_image = csv_file.replace('.csv', '_memory_trend.png')
    plt.savefig(output_image, dpi=300, bbox_inches='tight')
    print(f"图表已保存为：{output_image}")

    # 显示图表（如果在交互环境中）
    try:
        plt.show()
    except:
        pass

def main():
    """主函数"""
    if len(sys.argv) != 2:
        print("用法：python analyze_memory.py <csv_file>")
        print("示例：python analyze_memory.py memory_test.csv")
        sys.exit(1)

    csv_file = sys.argv[1]
    analyze_memory_data(csv_file)

if __name__ == "__main__":
    main()