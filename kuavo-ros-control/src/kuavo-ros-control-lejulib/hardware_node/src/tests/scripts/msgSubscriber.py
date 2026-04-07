#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from kuavo_msgs.msg import jointCmd
from kuavo_msgs.msg import sensorsData
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QHBoxLayout,
                            QWidget, QSlider, QLabel, QGridLayout, QSplitter,
                            QCheckBox, QScrollArea, QGroupBox, QPushButton)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = ["SimHei", "WenQuanYi Micro Hei", "Heiti TC"]
plt.rcParams["axes.unicode_minus"] = False

# 全局变量，用于跟踪ROS节点是否已初始化
ROS_NODE_INITIALIZED = False

class JointMonitorApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('机器人关节实时监控系统')
        self.setGeometry(100, 100, 1600, 900)

        # ####################################################################
        # 使用全局变量检查ROS节点是否已初始化
        global ROS_NODE_INITIALIZED
        if not ROS_NODE_INITIALIZED:
            rospy.init_node('joint_monitor', anonymous=True)
            ROS_NODE_INITIALIZED = True
        # ####################################################################

        # rospy.init_node('joint_monitor', anonymous=True)
        self.joint_sub = rospy.Subscriber('/joint_cmd', jointCmd, self.joint_callback)
        # 添加新的订阅器
        self.sensors_sub = rospy.Subscriber('/sensors_data_raw', sensorsData, self.sensors_callback)

        self.buffer_size = 500
        self.joint_data = np.zeros((28, self.buffer_size))  # jointCmd数据
        self.sensors_q_data = np.zeros((28, self.buffer_size))  # sensors_data_raw的joint_q数据
        self.sensors_torque_data = np.zeros((28, self.buffer_size))  # sensors_data_raw的joint_torque数据
        self.time_data = np.arange(self.buffer_size)
        self.current_idx = 0
        self.is_paused = False
        self.selected_indices = list(range(12, 19))
        
        # 记录数据接收状态
        self.joint_data_received = False
        self.sensors_data_received = False

        self.y_axis_range = 1.0
        self.init_ui()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(50)
        rospy.loginfo("系统启动，等待数据...")

    def init_ui(self):
        main_splitter = QSplitter(Qt.Horizontal)
        self.setCentralWidget(main_splitter)

        control_widget = QWidget()
        control_layout = QVBoxLayout(control_widget)

        joint_selection_group = QGroupBox("Select Motor")
        joint_selection_layout = QGridLayout()

        self.joint_checkboxes = []
        for i in range(28):
            checkbox = QCheckBox(f"Motor {i}")
            checkbox.setChecked(i in self.selected_indices)
            checkbox.stateChanged.connect(self.update_selected_indices)
            joint_selection_layout.addWidget(checkbox, i // 4, i % 4)
            self.joint_checkboxes.append(checkbox)

        select_buttons_layout = QHBoxLayout()
        select_all_btn = QPushButton("all")
        select_all_btn.clicked.connect(self.select_all_joints)
        deselect_all_btn = QPushButton("cancel all")
        deselect_all_btn.clicked.connect(self.deselect_all_joints)
        select_buttons_layout.addWidget(select_all_btn)
        select_buttons_layout.addWidget(deselect_all_btn)

        joint_selection_layout.addLayout(select_buttons_layout, 7, 0, 1, 4)
        joint_selection_group.setLayout(joint_selection_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidget(joint_selection_group)
        scroll_area.setWidgetResizable(True)
        control_layout.addWidget(scroll_area)

        slider_control_group = QGroupBox("Icon control")
        slider_control_layout = QVBoxLayout()

        y_axis_layout = QHBoxLayout()
        y_axis_layout.addWidget(QLabel("Y-axis:"))

        self.y_range_slider = QSlider(Qt.Horizontal)
        self.y_range_slider.setMinimum(1)
        self.y_range_slider.setMaximum(50)
        self.y_range_slider.setValue(int(self.y_axis_range * 10))
        self.y_range_slider.setTickInterval(5)
        self.y_range_slider.setSingleStep(1)
        self.y_range_slider.valueChanged.connect(self.set_y_axis_range)
        y_axis_layout.addWidget(self.y_range_slider)

        self.y_range_value = QLabel(f"{self.y_axis_range:.1f}")
        y_axis_layout.addWidget(self.y_range_value)

        slider_control_layout.addLayout(y_axis_layout)

        history_layout = QHBoxLayout()
        history_layout.addWidget(QLabel("X-axis:"))

        self.history_slider = QSlider(Qt.Horizontal)
        self.history_slider.setMinimum(100)
        self.history_slider.setMaximum(1000)
        self.history_slider.setValue(self.buffer_size)
        self.history_slider.setTickInterval(100)
        self.history_slider.setSingleStep(100)
        self.history_slider.valueChanged.connect(self.change_history_length)
        history_layout.addWidget(self.history_slider)

        self.history_value = QLabel(f"{self.buffer_size}")
        history_layout.addWidget(self.history_value)

        slider_control_layout.addLayout(history_layout)

        slider_control_group.setLayout(slider_control_layout)
        control_layout.addWidget(slider_control_group)

        pause_layout = QHBoxLayout()
        self.pause_button = QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self.toggle_pause)
        pause_layout.addWidget(QLabel("status:"))
        pause_layout.addWidget(self.pause_button)
        pause_layout.addStretch()
        control_layout.addLayout(pause_layout)

        control_layout.addStretch()

        self.chart_widget = QWidget()
        self.chart_layout = QVBoxLayout(self.chart_widget)
        self.create_chart_area()

        main_splitter.addWidget(control_widget)
        main_splitter.addWidget(self.chart_widget)
        main_splitter.setSizes([300, 1300])

        self.statusBar().showMessage('READY')

    def create_chart_area(self):
        if hasattr(self, 'fig'):
            self.fig.clear()

        self.fig = Figure(figsize=(15, 10), dpi=100)
        self.canvas = FigureCanvas(self.fig)

        n_selected = len(self.selected_indices)
        n_rows = int(np.ceil(n_selected / 2))

        self.gs = self.fig.add_gridspec(n_rows, 2, hspace=0.5, wspace=0.3)

        self.axes = []
        self.lines = []  # 存储jointCmd数据的线条
        self.sensors_q_lines = []  # 存储sensors_data_raw的joint_q数据的线条
        self.sensors_torque_lines = []  # 存储sensors_data_raw的joint_torque数据的线条
        self.axis_map = {}

        torque_indices = list(range(12)) + [12, 19]

        for i, joint_idx in enumerate(self.selected_indices):
            row = i // 2
            col = i % 2
            ax = self.fig.add_subplot(self.gs[row, col])
            self.axes.append(ax)

            # 绘制jointCmd数据的线条（蓝色）
            line, = ax.plot([], [], 'b-', linewidth=3.0, label='jointCmd')
            self.lines.append(line)

            # 绘制sensors_data_raw的joint_q或joint_torque数据的线条（红色）
            if joint_idx in torque_indices:
                sensors_line, = ax.plot([], [], 'r-', linewidth=1.5, label='sensors torque')
                self.sensors_torque_lines.append(sensors_line)
                self.sensors_q_lines.append(None)  # 扭矩索引不显示joint_q
            else:
                sensors_line, = ax.plot([], [], 'r-', linewidth=1.5, label='sensors q')
                self.sensors_q_lines.append(sensors_line)
                self.sensors_torque_lines.append(None)  # 非扭矩索引不显示joint_torque

            ax.set_title(f'Motor {joint_idx}')
            ax.set_xlim(0, self.buffer_size)
            ax.set_ylim(-self.y_axis_range, self.y_axis_range)

            ax.grid(True, linestyle='--', alpha=0.7)

            ax.axhline(y=0, color='g', linestyle='-', alpha=0.3)

            if row == n_rows - 1:
                ax.set_xlabel('时间点')

            if col == 0:
                ax.set_ylabel('角度 (rad)' if joint_idx not in torque_indices else '扭矩 (N·m)')

            ax.legend(loc='upper right')  # 添加图例

            self.axis_map[joint_idx] = i

        for i in reversed(range(self.chart_layout.count())):
            widget = self.chart_layout.itemAt(i).widget()
            if widget is not None:
                self.chart_layout.removeWidget(widget)
                widget.deleteLater()

        self.chart_layout.addWidget(self.canvas)
        self.canvas.draw()

    def joint_callback(self, msg):
        if self.is_paused:
            return

        try:
            data = np.zeros(28)
            torque_indices = list(range(12)) + [12, 19]

            for i in range(min(len(msg.tau), 28)):
                if i in torque_indices:
                    data[i] = msg.tau[i] if i < len(msg.tau) else 0.0
                else:
                    data[i] = msg.joint_q[i] if i < len(msg.joint_q) else 0.0

            n_joints = min(len(data), self.joint_data.shape[0])
            self.joint_data[:n_joints, self.current_idx] = data[:n_joints]
            self.joint_data_received = True

            # 合并状态消息（直接从sensors_data中获取有效数据长度）
            if self.sensors_data_received:
                # 获取当前索引处的非零数据数量作为有效传感器数据数量
                valid_sensors_q = np.count_nonzero(self.sensors_q_data[:, self.current_idx])
                valid_sensors_torque = np.count_nonzero(self.sensors_torque_data[:, self.current_idx])

                self.statusBar().showMessage(
                    f'接收数据：joint_cmd={n_joints}个, '
                    f'sensors_q={valid_sensors_q}个, sensors_torque={valid_sensors_torque}个'
                )
            else:
                self.statusBar().showMessage(f'接收/joint_cmd数据：{n_joints}个电机')
        except Exception as e:
            rospy.logerr(f"/joint_cmd数据处理错误: {str(e)}")


    def sensors_callback(self, msg):
        if self.is_paused:
            return

        try:
            # 提取joint_data中的joint_q和joint_torque
            joint_q = msg.joint_data.joint_q if hasattr(msg.joint_data, 'joint_q') else []
            joint_torque = msg.joint_data.joint_torque if hasattr(msg.joint_data, 'joint_torque') else []

            # 存储数据
            for i in range(min(len(joint_q), 28)):
                self.sensors_q_data[i, self.current_idx] = joint_q[i]
            for i in range(min(len(joint_torque), 28)):
                self.sensors_torque_data[i, self.current_idx] = joint_torque[i]

            self.sensors_data_received = True  # 标记传感器数据已接收

            # 合并状态消息（移除未定义的joint_cmd_count）
            if self.joint_data_received:
                # 同时接收到两种数据时，显示全部统计
                self.statusBar().showMessage(
                    f'接收数据：joint_cmd={self.joint_data.shape[0]}个, '
                    f'sensors_q={len(joint_q)}个, sensors_torque={len(joint_torque)}个'
                )
            else:
                # 仅接收到传感器数据时，显示传感器统计
                self.statusBar().showMessage(
                    f'接收/sensors_data_raw数据：q={len(joint_q)}个, torque={len(joint_torque)}个'
                )
        except Exception as e:
            rospy.logerr(f"/sensors_data_raw数据处理错误: {str(e)}")


    def update_plots(self):
        if rospy.is_shutdown() or self.is_paused:
            return

        # 仅当接收到数据时才更新索引
        if self.joint_data_received or self.sensors_data_received:
            self.current_idx = (self.current_idx + 1) % self.buffer_size
            self.joint_data_received = False
            self.sensors_data_received = False

        for joint_idx in self.selected_indices:
            if joint_idx not in self.axis_map:
                continue

            axis_idx = self.axis_map[joint_idx]

            if self.current_idx > 0:
                time = self.time_data[:self.current_idx]
                joint_cmd_data = self.joint_data[joint_idx, :self.current_idx]
                sensors_q_data = self.sensors_q_data[joint_idx, :self.current_idx]
                sensors_torque_data = self.sensors_torque_data[joint_idx, :self.current_idx]
            else:
                time = np.concatenate([self.time_data[self.current_idx:],
                                       self.time_data[:self.current_idx]])
                joint_cmd_data = np.concatenate([self.joint_data[joint_idx, self.current_idx:],
                                                self.joint_data[joint_idx, :self.current_idx]])
                sensors_q_data = np.concatenate([self.sensors_q_data[joint_idx, self.current_idx:],
                                                self.sensors_q_data[joint_idx, :self.current_idx]])
                sensors_torque_data = np.concatenate([self.sensors_torque_data[joint_idx, self.current_idx:],
                                                    self.sensors_torque_data[joint_idx, :self.current_idx]])

            # 更新jointCmd数据的线条
            self.lines[axis_idx].set_data(time, joint_cmd_data)

            # 更新sensors_data_raw数据的线条
            if joint_idx in list(range(12)) + [12, 19]:  # 扭矩索引
                if self.sensors_torque_lines[axis_idx] is not None:
                    self.sensors_torque_lines[axis_idx].set_data(time, sensors_torque_data)
            else:  # 非扭矩索引
                if self.sensors_q_lines[axis_idx] is not None:
                    self.sensors_q_lines[axis_idx].set_data(time, sensors_q_data)

        self.canvas.draw()

    def toggle_pause(self, checked):
        self.is_paused = checked
        if checked:
            self.pause_button.setText("继续")
            self.statusBar().showMessage('已暂停数据更新')
        else:
            self.pause_button.setText("暂停")
            self.statusBar().showMessage('数据更新中')

    def change_history_length(self):
        new_size = self.history_slider.value()
        self.history_value.setText(f"{new_size}")

        # 调整所有数据缓冲区大小
        def resize_buffer(buffer):
            new_buffer = np.zeros((buffer.shape[0], new_size))
            for i in range(buffer.shape[0]):
                if self.current_idx > 0:
                    valid_len = min(self.current_idx, new_size)
                    new_buffer[i, :valid_len] = buffer[i, :valid_len]
                else:
                    old_len = buffer.shape[1]
                    copy_len = min(old_len, new_size)
                    if copy_len <= old_len - self.current_idx:
                        new_buffer[i, :copy_len] = buffer[i, self.current_idx:self.current_idx + copy_len]
                    else:
                        first_part = old_len - self.current_idx
                        new_buffer[i, :first_part] = buffer[i, self.current_idx:]
                        new_buffer[i, first_part:copy_len] = buffer[i, :copy_len - first_part]
            return new_buffer

        self.joint_data = resize_buffer(self.joint_data)
        self.sensors_q_data = resize_buffer(self.sensors_q_data)
        self.sensors_torque_data = resize_buffer(self.sensors_torque_data)

        self.buffer_size = new_size
        self.time_data = np.arange(new_size)
        self.current_idx = min(self.current_idx, new_size - 1)

        for ax in self.axes:
            ax.set_xlim(0, new_size)

        self.canvas.draw()

    def set_y_axis_range(self):
        value = self.y_range_slider.value() / 10.0
        self.y_axis_range = value
        self.y_range_value.setText(f"{value:.1f}")

        for ax in self.axes:
            ax.set_ylim(-value, value)

        self.canvas.draw()

    def update_selected_indices(self):
        self.selected_indices = [i for i, checkbox in enumerate(self.joint_checkboxes) if checkbox.isChecked()]
        self.create_chart_area()
        rospy.loginfo(f"更新显示的关节: {self.selected_indices}")

    def select_all_joints(self):
        for checkbox in self.joint_checkboxes:
            checkbox.setChecked(True)
        self.update_selected_indices()

    def deselect_all_joints(self):
        for checkbox in self.joint_checkboxes:
            checkbox.setChecked(False)
        self.update_selected_indices()

    def closeEvent(self, event):
        # 不要在窗口关闭时终止ROS节点，改为暂停订阅
        if self.joint_sub:
            self.joint_sub.unregister()
            self.joint_sub = None
            
        if self.sensors_sub:
            self.sensors_sub.unregister()
            self.sensors_sub = None
            
        # 隐藏窗口而不是销毁它
        self.hide()
        event.ignore()  # 忽略关闭事件

# 添加主函数，允许脚本独立运行
if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        window = JointMonitorApp()
        window.show()
        
        # 捕获关闭信号，确保程序可以正常退出
        def sigint_handler(*args):
            """Handler for the SIGINT signal."""
            sys.stderr.write('\r')
            QApplication.quit()
            
        import signal
        signal.signal(signal.SIGINT, sigint_handler)
        
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"发生错误: {str(e)}")      