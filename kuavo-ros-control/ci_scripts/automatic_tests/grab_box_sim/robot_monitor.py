#!/usr/bin/env python3
import subprocess

class RobotMonitor:
    def __init__(self):
        self.is_passed = False

    def alive(self)->bool:
        # 检查节点是否存在
        try:
            # 使用rosnode list命令获取节点列表
            process = subprocess.Popen(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            output, error = process.communicate()
            
            # 将bytes转换为string并按行分割
            nodes = output.decode('utf-8').split('\n')
            
            if any("/humanoid_sqp_mpc" in node for node in nodes):
                # print("✅ /humanoid_sqp_mpc 节点已启动")
                self.is_passed = True
                return True
            
        except Exception as e:
            print(f"❌ 检查节点时发生错误: {e}")
        
        return False

if __name__ == "__main__":
    checker = RobotMonitor()
    import time
    import sys
    start_time = time.time()
    while time.time() - start_time < 2.0:
        if checker.alive():
            print("✅ 机器人已启动")
            sys.exit(0)
        time.sleep(0.2)
    sys.exit(1)