#!/bin/bash

# 内存监控脚本 - 用于监控ROS程序的内存使用情况

# 默认参数
INTERVAL=1  # 监控间隔（秒）

# 显示帮助信息
show_help() {
    echo "内存监控脚本 - 监控指定进程的内存使用情况"
    echo ""
    echo "用法:"
    echo "  $0 --pid PID               # 按进程ID监控"
    echo "  $0 --name 进程名           # 按进程名监控"
    echo ""
    echo "选项:"
    echo "  -i, --interval SEC         监控间隔（秒），默认1秒"
    echo "  -o, --output FILE          输出到CSV文件"
    echo "  --pid PID                  要监控的进程ID"
    echo "  --name 进程名              要监控的进程名称"
    echo "  -h, --help                 显示帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 --pid 1313942                    # 监控指定PID的进程"
    echo "  $0 --pid 1313942 -i 2              # 每2秒监控一次"
    echo "  $0 --name dexhand_ctrl_test         # 监控dexhand_ctrl_test进程"
    echo "  $0 --name dexhand_ctrl_test -i 5   # 每5秒监控一次"
    echo "  $0 --name dexhand_ctrl_test -o memory.csv -i 3  # 输出到CSV文件"
    echo ""
    echo "CSV分析："
    echo "  使用 analyze_memory.py 脚本分析CSV数据并生成图表"
    echo "  python3 analyze_memory.py memory.csv"
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--interval)
            INTERVAL="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --pid)
            PID="$2"
            shift 2
            ;;
        --name)
            PROCESS_NAME="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

if [[ -z "$PID" ]] && [[ -z "$PROCESS_NAME" ]]; then
    echo "错误：请指定 --pid 或 --name 参数"
    show_help
    exit 1
fi

if [[ -n "$PID" ]] && [[ -n "$PROCESS_NAME" ]]; then
    echo "错误：--pid 和 --name 参数不能同时使用"
    show_help
    exit 1
fi

# 设置监控目标
if [[ -n "$PID" ]]; then
    TARGET="$PID"
    echo "开始监控内存使用情况 - PID: $PID"
else
    TARGET="$PROCESS_NAME"
    echo "开始监控内存使用情况 - 进程名: $PROCESS_NAME"
fi

echo "监控间隔: $INTERVAL 秒"

# 初始化CSV文件
if [[ -n "$OUTPUT_FILE" ]]; then
    echo "输出到CSV文件: $OUTPUT_FILE"
    echo "timestamp,pid,rss_mb,vsz_mb,mem_percent,command" > "$OUTPUT_FILE"
    echo "CSV文件已初始化"
fi

echo "按 Ctrl+C 停止监控"
echo "----------------------------------------"

# 监控循环
while true; do
    if [[ -n "$PID" ]]; then
        # PID模式
        ps -p "$PID" -o pid,rss,vsz,%mem,cmd --no-headers 2>/dev/null | \
        while read -r pid rss vsz percent cmd; do
            if [[ -n "$pid" ]]; then
                rss_mb=$(echo "scale=2; $rss/1024" | bc -l 2>/dev/null || echo "N/A")
                vsz_mb=$(echo "scale=2; $vsz/1024" | bc -l 2>/dev/null || echo "N/A")
                timestamp=$(date "+%Y-%m-%d %H:%M:%S")

                # 控制台输出
                echo "$timestamp | PID:$pid | RSS:${rss_mb}MB | VSZ:${vsz_mb}MB | 内存使用率:${percent}%"

                # CSV文件输出
                if [[ -n "$OUTPUT_FILE" ]]; then
                    echo "\"$timestamp\",$pid,\"$rss_mb\",\"$vsz_mb\",\"$percent\",\"$cmd\"" >> "$OUTPUT_FILE"
                fi
            fi
        done
    else
        # 进程名模式 - 只监控目标可执行文件，排除监控脚本自身
        # 使用pgrep模糊匹配进程名，但排除包含memory_monitor的进程
        pgrep -f "$PROCESS_NAME" | while read -r pid; do
            # 检查是否为主进程（通过检查命令行是否包含可执行路径）
            cmd=$(ps -p "$pid" -o cmd --no-headers 2>/dev/null || echo "")
            # 只监控包含完整路径的目标进程，排除监控脚本自身和shell进程
            if [[ "$cmd" == *"./"*"$PROCESS_NAME"* ]] && [[ "$cmd" != *"memory_monitor"* ]]; then
                ps -p "$pid" -o pid,rss,vsz,%mem,cmd --no-headers 2>/dev/null | \
                while read -r pid rss vsz percent cmd; do
                    if [[ -n "$pid" ]]; then
                        rss_mb=$(echo "scale=2; $rss/1024" | bc -l 2>/dev/null || echo "N/A")
                        vsz_mb=$(echo "scale=2; $vsz/1024" | bc -l 2>/dev/null || echo "N/A")
                        timestamp=$(date "+%Y-%m-%d %H:%M:%S")

                        # 控制台输出
                        echo "$timestamp | PID:$pid | RSS:${rss_mb}MB | VSZ:${vsz_mb}MB | 内存使用率:${percent}%"

                        # CSV文件输出
                        if [[ -n "$OUTPUT_FILE" ]]; then
                            echo "\"$timestamp\",$pid,\"$rss_mb\",\"$vsz_mb\",\"$percent\",\"$cmd\"" >> "$OUTPUT_FILE"
                        fi
                    fi
                done
            fi
        done
    fi

    # 如果没有找到进程，退出
    if [[ -n "$PID" ]]; then
        if ! ps -p "$PID" > /dev/null 2>&1; then
            echo "未找到目标进程，监控结束"
            break
        fi
    else
        if ! pgrep -f "$PROCESS_NAME" > /dev/null 2>&1; then
            echo "未找到目标进程，监控结束"
            break
        fi
    fi

    sleep "$INTERVAL"
done
