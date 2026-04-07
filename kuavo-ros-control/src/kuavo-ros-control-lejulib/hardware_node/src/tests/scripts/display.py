import sys
import random
import time
from threading import Thread
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from msgSubscriber import JointMonitorApp

class MotorTestGUI(QtWidgets.QMainWindow):
    update_status_signal = QtCore.pyqtSignal(int, str, str)  # 行号, 方向, 状态

    def __init__(self):
        super().__init__()
        self.monitor_window = None  # 存储监控窗口的引用

        self.setWindowTitle("电机跟随性测试")
        self.setGeometry(100, 100, 800, 600)

        # 提前定义电机数据
        self.motor_data = [
            (12, 19), (13, 20), (14, 21), (15, 22),
            (16, 23), (17, 24), (18, 25), (0, 6),
            (1, 7), (2, 8), (3, 9), (4, 10), (5, 11)
        ]

        # 设置中文字体
        qApp = QtWidgets.QApplication.instance()
        font = QtGui.QFont("SimHei", 10)  # 使用黑体，大小 10
        qApp.setFont(font)

        self.init_ui()
        self.init_signals()
        self.setup_motors()

        # 测试相关变量
        self.test_thread = None
        self.stop_flag = False

    def create_waveform_icon(self):
        """绘制波形图标"""
        pixmap = QtGui.QPixmap(24, 24)
        pixmap.fill(Qt.transparent)  # 透明背景
        painter = QtGui.QPainter(pixmap)
        painter.setPen(QtGui.QPen(Qt.red, 2))  # 蓝色线条，宽度 2

        # 定义波形路径（简单示例）
        path = QtGui.QPainterPath()
        path.moveTo(0, 12)  # 起点 (0, 中心)
        for x in range(24):
            y = 12 + 6 * (-1) ** (x // 3)  # 波形公式，y 围绕中心 12 波动
            path.lineTo(x, y)
        painter.drawPath(path)
        painter.end()

        icon = QtGui.QIcon(pixmap)
        return icon

    def init_ui(self):
        # 创建中心部件和布局
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QVBoxLayout(central_widget)

        # 创建表格区域
        table_widget = QtWidgets.QWidget()
        table_layout = QtWidgets.QHBoxLayout(table_widget)
        self.left_table = self.create_table()
        self.right_table = self.create_table()
        table_layout.addWidget(self.left_table)
        table_layout.addWidget(self.right_table)

        # 创建按钮区域
        button_widget = QtWidgets.QWidget()
        button_layout = QtWidgets.QHBoxLayout(button_widget)

        # self.waveform_btn = QtWidgets.QPushButton("显示所有波形图")

        # 修改后的波形按钮
        self.waveform_btn = QtWidgets.QPushButton()
        ###########################################################
        self.waveform_btn.clicked.connect(self.open_monitor_window)
        ###########################################################
        self.waveform_btn.setIcon(self.create_waveform_icon())  # 设置图标
        self.waveform_btn.setIconSize(QtCore.QSize(24, 24))  # 设置图标大小

        self.start_btn = QtWidgets.QPushButton("Start Test")
        self.stop_btn = QtWidgets.QPushButton("Stop Test")
        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("background-color: #CCCCCC; border: 1px solid #999;")

        button_layout.addWidget(self.waveform_btn)
        button_layout.addWidget(self.start_btn)
        button_layout.addWidget(self.stop_btn)
        button_layout.addWidget(self.status_label)

        main_layout.addWidget(table_widget)
        main_layout.addWidget(button_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(15)

    def open_monitor_window(self):
        """打开或激活实时监控窗口"""
        try:
            # 如果窗口已存在但被隐藏，则显示它
            if self.monitor_window and not self.monitor_window.isHidden():
                self.monitor_window.activateWindow()  # 激活窗口
                self.monitor_window.raise_()  # 将窗口置于最前
                return
                
            # 如果窗口不存在或已关闭，创建新窗口
            self.monitor_window = JointMonitorApp()
            
            # 为避免ROS节点问题，确保新窗口初始化成功
            if not self.monitor_window.joint_data_received:
                # 强制刷新数据接收状态
                self.monitor_window.joint_data_received = True
                self.monitor_window.sensors_data_received = True
            
            self.monitor_window.show()
            
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "错误", f"打开监控窗口失败: {str(e)}")

    def init_signals(self):
        self.update_status_signal.connect(self.update_table_status)
        self.start_btn.clicked.connect(self.start_test)
        self.stop_btn.clicked.connect(self.stop_test)
        self.waveform_btn.clicked.connect(self.show_waveforms)

    def create_table(self):
        table = QtWidgets.QTableWidget()
        table.setColumnCount(3)  # 新增一列按钮
        table.setHorizontalHeaderLabels(["Motor Index", "Status", "Waveform"])  # 更新表头
        # 原有表格设置保持不变...
        table.setColumnWidth(2, 80)  # 设置按钮列宽度

        # 设置表格宽度为300像素
        table.setFixedWidth(310) 
        table.setFixedHeight(430) 
        # 自动调整列宽和行高
        # table.resizeColumnsToContents()
        # table.resizeRowsToContents()
        return table

    def setup_motors(self):
        for i in range(len(self.motor_data)):
            left_motor, right_motor = self.motor_data[i]
            # 左侧表格
            self.left_table.insertRow(i)
            self.left_table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"Motor {left_motor}"))
            self.left_table.setItem(i, 1, QtWidgets.QTableWidgetItem("Not Tested"))
            self.create_waveform_button(self.left_table, i, left_motor)  # 创建按钮

            # 右侧表格
            self.right_table.insertRow(i)
            self.right_table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"Motor {right_motor}"))
            self.right_table.setItem(i, 1, QtWidgets.QTableWidgetItem("Not Tested"))
            self.create_waveform_button(self.right_table, i, right_motor)  # 创建按钮

    def create_waveform_button(self, table, row, motor_id):
        """为表格的指定行创建波形图按钮"""
        btn = QtWidgets.QPushButton()
        btn.setIcon(self.create_waveform_icon())  # 使用现有波形图标
        btn.setIconSize(QtCore.QSize(16, 16))
        btn.clicked.connect(lambda checked, mid=motor_id: self.show_single_waveform(mid))
        table.setCellWidget(row, 2, btn)

    def update_table_status(self, row, side, status):
        table = self.left_table if side == "left" else self.right_table
        item = table.item(row, 1)
        if not item:
            item = QtWidgets.QTableWidgetItem()
            table.setItem(row, 1, item)

        status_map = {
            "未检测": ("Not Tested", "#FFFFFF"),
            "测试中": ("Testing", "#FFFF99"),
            "正常": ("Normal", "#90EE90"),
            "异常": ("Abnormal", "#FFA07A")
        }
        text, color = status_map.get(status, ("Unknown", "#FFFFFF"))
        item.setText(text)
        item.setBackground(QtGui.QColor(color))

    def simulate_test(self, row, side):
        time.sleep(random.uniform(0.5, 1.5))
        result = random.choice([True, False])
        status = "正常" if result else "异常"
        self.update_status_signal.emit(row, side, status)

    def run_test(self):
        self.stop_flag = False
        self.status_label.setText("Testing...")
        self.status_label.setStyleSheet("background-color: #FFFF99; border: 1px solid #999;")
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)

        for i in range(len(self.motor_data)):
            if self.stop_flag:
                break

            # 测试左侧电机
            self.update_status_signal.emit(i, "left", "测试中")
            self.simulate_test(i, "left")

            if self.stop_flag:
                break

            # 测试右侧电机
            self.update_status_signal.emit(i, "right", "测试中")
            self.simulate_test(i, "right")

        self.test_complete()

    def test_complete(self):
        status = "Test Complete" if not self.stop_flag else "Test Stopped"
        color = "#90EE90" if not self.stop_flag else "#FFA07A"
        self.status_label.setText(status)
        self.status_label.setStyleSheet(f"background-color: {color}; border: 1px solid #999;")
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.stop_flag = False

    def start_test(self):
        if not self.test_thread or not self.test_thread.is_alive():
            self.test_thread = Thread(target=self.run_test, daemon=True)
            self.test_thread.start()

    def stop_test(self):
        self.stop_flag = True
        self.status_label.setText("Stopping test...")
        self.status_label.setStyleSheet("background-color: #FFA07A; border: 1px solid #999;")

    def show_waveforms(self):
        # 模拟波形
        # dialog = QtWidgets.QDialog(self)
        # dialog.setWindowTitle("所有电机波形图")
        # dialog.setGeometry(100, 100, 1000, 800)

        # scroll_area = QtWidgets.QScrollArea(dialog)
        # scroll_area.setWidgetResizable(True)
        # content_widget = QtWidgets.QWidget()
        # scroll_area.setWidget(content_widget)
        # layout = QtWidgets.QVBoxLayout(content_widget)

        # for i, (left, right) in enumerate(self.motor_data):
        #     frame = QtWidgets.QGroupBox(f"电机 {left} 和 {right} 的波形图")
        #     frame_layout = QtWidgets.QHBoxLayout(frame)

        #     fig = Figure(figsize=(9, 3), dpi=100)
        #     ax1 = fig.add_subplot(121)
        #     ax2 = fig.add_subplot(122)

        #     x = list(range(100))
        #     y1 = [random.uniform(-1, 1) for _ in x]
        #     y2 = [random.uniform(-1, 1) for _ in x]

        #     ax1.plot(x, y1); ax1.grid(True)
        #     ax2.plot(x, y2); ax2.grid(True)

        #     canvas = FigureCanvas(fig)
        #     frame_layout.addWidget(canvas)
        #     layout.addWidget(frame)

        # dialog.setLayout(QtWidgets.QVBoxLayout())
        # dialog.layout().addWidget(scroll_area)
        # dialog.exec_()
        pass

    def show_single_waveform(self, motor_id):
        """显示单个电机的波形图"""
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle(f"电机 {motor_id} 波形图")
        dialog.setGeometry(300, 300, 600, 400)

        layout = QtWidgets.QVBoxLayout(dialog)
        frame = QtWidgets.QGroupBox(f"")
        frame_layout = QtWidgets.QHBoxLayout(frame)

        fig = Figure(figsize=(6, 3), dpi=100)
        ax = fig.add_subplot(111)

        x = list(range(100))
        y = [random.uniform(-1, 1) for _ in x]

        ax.plot(x, y); ax.grid(True)

        canvas = FigureCanvas(fig)
        frame_layout.addWidget(canvas)
        layout.addWidget(frame)

        dialog.exec_()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = MotorTestGUI()
    gui.show()
    sys.exit(app.exec_())
