import time
import random
import click

class MotorTester:
    def __init__(self):
        self.motor_data = [
            (12, 19), (13, 20), (14, 21), (15, 22), 
            (16, 23), (17, 24), (18, 25), (0, 6), 
            (1, 7), (2, 8), (3, 9), (4, 10), (5, 11)
        ]
    
    def print_status_table(self, left_status, right_status):
        print("\n电机状态表:")
        print("-" * 50)
        print(f"{'左侧电机':<15}{'状态':<10}{'右侧电机':<15}{'状态':<10}")
        print("-" * 50)
        for i in range(len(self.motor_data)):
            left, right = self.motor_data[i]
            print(f"Motor {left:<4}{left_status[i]:<10}Motor {right:<4}{right_status[i]:<10}")
        print("-" * 50)
    
    def run_test(self):
        left_status = ["Not Tested"] * len(self.motor_data)
        right_status = ["Not Tested"] * len(self.motor_data)
        
        for i in range(len(self.motor_data)):
            # 左侧电机
            left_status[i] = "Testing"
            self.print_status_table(left_status, right_status)
            time.sleep(random.uniform(0.5, 1.5))
            left_status[i] = "Normal" if random.choice([True, False]) else "Abnormal"
            
            # 右侧电机
            right_status[i] = "Testing"
            self.print_status_table(left_status, right_status)
            time.sleep(random.uniform(0.5, 1.5))
            right_status[i] = "Normal" if random.choice([True, False]) else "Abnormal"
        
        print("\n测试完成！")

@click.command()
@click.option("--waveform", is_flag=True, help="显示波形图（需图形环境）")
def main(waveform):
    tester = MotorTester()
    print("电机跟随性测试启动...")
    tester.run_test()
    
    if waveform:
        # 此处可调用 matplotlib 生成波形图（需 X 环境）
        import matplotlib.pyplot as plt
        plt.plot([random.randint(0, 100) for _ in range(50)])
        plt.title("电机波形图")
        plt.show()

if __name__ == "__main__":
    main()
