#!/usr/bin/env python3
import time
import threading
import os
import copy
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState, KuavoRobotTools, KuavoRobotVision
from kuavo_humanoid_sdk.interfaces.data_types import AprilTagData, PoseQuaternion, KuavoPose
from kuavo_humanoid_sdk.kuavo_strategy.grasp_box.grasp_box_strategy import KuavoGraspBox, BoxInfo
from kuavo_humanoid_sdk.common.logger import SDKLogger
from robot_monitor import RobotMonitor
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion

""" Decorators """
def on_exit(func=None, mark=None, action=None):
    def decorator(func):
        def wrapper(*args, **kwargs):
            try:
                result = func(*args, **kwargs)
            except Exception as e:
                result = False
            finally:
                action(mark, result)  # 无论是否异常都会执行
            return result
        return wrapper
    if func is None:
        return decorator
    return decorator(func)
# ------------------------------------------------------------------------

class GrabBoxResult:
    def __init__(self):
       pass

    def add_attribute(self, name, value):
        setattr(self, name, value)

    def reset(self):
        self.__dict__.clear()

    def __str__(self):
        return f"{self.__dict__}"

grab_box_result = GrabBoxResult()


def print_result(success):
    SDKLogger.info("\033[93m******************************************************\033[0m")
    if success:
        SDKLogger.info("* ✅ 搬箱子任务完成")
    else:
        SDKLogger.info("* ❌ 搬箱子任务失败")
    SDKLogger.info(grab_box_result)
    SDKLogger.info("\033[93m******************************************************\033[0m")
    grab_box_result.reset()

def exit_check(mark, result):
    if not result:
        SDKLogger.info(f"❌ [{mark}] 失败，无法继续执行")
        print_result(False)
        os._exit(1)
    else:
        SDKLogger.info(f"✅ [{mark}] 成功")

class GrabBoxTest:
    def __init__(self):
        self.robot = KuavoRobot()
        self.robot_state = KuavoRobotState()
        self.robot_tools = KuavoRobotTools()
        self.robot_vision = KuavoRobotVision()
        self.grasp_strategy = KuavoGraspBox(self.robot, self.robot_state, self.robot_tools, self.robot_vision)

        ##############################################################################    
        # 🎯 这里定义搬箱子案例的一些信息

        # 粘贴在箱子上的 AprilTag 信息: 需要 ID，尺寸和基于odom坐标系下的大致位姿
        self.box_tag = AprilTagData(
            id=[1],                                 # AprilTag ID
            size=[0.1],                             # AprilTag 标签尺寸
            pose=[PoseQuaternion(
                position=(0.0, -1.5, 1.1),          # 基于odom坐标系下的大致位置, 查找时会对齐到这个方向
                orientation=(0.0, 0.0, 0.0, 1.0)    # 四元数方向
            )]
        )
        
        # 粘贴在放置位置的 AprilTag 信息: 需要 ID，尺寸和基于odom坐标系下的大致位姿
        self.placement_tag = AprilTagData(
            id=[0],                                 # AprilTag ID
            size=[0.1],                             # AprilTag 标签尺寸
            pose=[PoseQuaternion(
                position=(0.0, 1.5, 1.5),           # 基于odom坐标系下的大致位置, 查找时会对齐到这个方向
                orientation=(0.0, 0.0, 1.0, 0.0)    # 四元数方向 - 旋转180度
            )]
        )

        self.box_size = (0.2, 0.3, 0.15) # 箱子尺寸(长、宽、高)，单位：米
        self.box_mass = 2.0              # 箱子重量，单位：千克
        ###############################################################################
    
    def ready_to_grab_box(self):
        self.robot.arm_reset()
        time.sleep(1)
        # !!! Important: 关闭 basePitch 限制, 否则无法进行大幅度的前后倾动作 会导致摔倒
        SDKLogger.info("⚠️ 重要提示: basePitch 限制需要关闭!!!")
        SDKLogger.info("⚠️ 重要提示: basePitch 限制需要关闭!!!")
        SDKLogger.info("⚠️ 重要提示: basePitch 限制需要关闭!!!")
        self.robot.enable_base_pitch_limit(False)  # 关闭 basePitch 限制
        # Retry up to 3 times to check pitch limit status
        for i in range(3):
            if not self.robot_state.pitch_limit_enabled():
                SDKLogger.info("✅ 已关闭 basePitch 限制")
                break
            else:
                if i < 2:  # Don't log on last attempt
                    SDKLogger.info(f"⚠️ 第{i+1}次检查 basePitch 限制状态...")
                time.sleep(0.5)  # Brief pause between retries
        else:  # Loop completed without break
            SDKLogger.info("❌ 关闭 basePitch 限制失败")
            return
        # ----------------------------------------------------------- 

    @on_exit(mark="寻找目标箱子Tag", action=exit_check)
    def find_apriltag(self):
        self.robot.disable_head_tracking()
        SDKLogger.info("已关闭头部追踪")
        retry_count = 1
        max_retry_count = 3
        self.box_tag_data = None
        while retry_count <= max_retry_count:
            if not self.grasp_strategy.head_find_target(
                copy.deepcopy(self.box_tag), 
                max_search_time=15.0,
                search_pattern="rotate_body" #  rotate_head
            ):
                SDKLogger.info(f"❌ 第{retry_count}次寻找目标箱子Tag失败")
                retry_count += 1
                continue
            self.box_tag_data = self.robot_vision.get_data_by_id_from_odom(self.box_tag.id[0])
            if self.box_tag_data is None:
                SDKLogger.info(f"❌ 未识别到 AprilTag ID 为{self.box_tag.id[0]} 的目标箱子")
                retry_count += 1
                continue
            break
        if retry_count > max_retry_count:
            SDKLogger.info("❌ 寻找目标箱子Tag失败")
            return False
        grab_box_result.add_attribute("📦 箱子Tag数据", self.box_tag_data)
        time.sleep(1.5)
        return True
    
    @on_exit(mark="靠近箱子", action=exit_check)
    def walk_approach_box(self):
        # 步骤2：走路接近目标
        retry_count = 1
        max_retry_count = 3
        while retry_count <= max_retry_count:
            if not self.grasp_strategy.walk_approach_target(
                self.box_tag.id[0],
                target_distance=0.4,  # 与目标箱子保持一定的距离 
                approach_speed=0.2    # 接近速度0.2米/秒
            ):
                SDKLogger.info(f"❌ 第{retry_count}次靠近箱子失败")
                retry_count += 1
                pitch_angles_deg = [25, -25]  # 两档pitch角度：抬头和低头，角度制
                yaw_angles_deg = [-30, -15, 0, 15, 30]  # 左右扫描的yaw角度，角度制
                for pitch_deg in pitch_angles_deg:
                    for yaw_deg in yaw_angles_deg:
                        yaw_rad = yaw_deg * 0.0174533  # 度转弧度，math.pi/180
                        pitch_rad = pitch_deg * 0.0174533  # 度转弧度
                        self.robot.control_head(yaw=yaw_rad, pitch=pitch_rad)
                        time.sleep(0.5)
                        target_data = self.robot_vision.get_data_by_id_from_odom(self.box_tag.id[0])
                        SDKLogger.debug(f"target_data: {target_data}")
                        if (target_data is not None and isinstance(target_data, dict) and 
                        'poses' in target_data and len(target_data['poses']) > 0):
                            self.robot.enable_head_tracking(self.box_tag.id[0])
                            break
                continue
            break
        if retry_count > max_retry_count:
            SDKLogger.info("❌ 靠近箱子失败")
            return False
        time.sleep(1)

        grab_box_result.add_attribute("📍 靠近箱子点位姿", self.robot_state.odometry)

        return True
    
    @on_exit(mark="搬箱子", action=exit_check)
    def grab_box(self):
        # 步骤3：手臂移动到抓取位置
        box_info = BoxInfo.from_apriltag(
            self.box_tag_data,
            xyz_offset=(self.box_size[0]/2+0.02, 0.0, -0.05),    # tag 粘贴在箱子正面，为了得到箱子中心点，因此偏移量为箱子宽度的一半
            size=self.box_size,                  # 箱子尺寸(长、宽、高)，单位：米
            mass=self.box_mass                   # 箱子重量，单位：千克
        )
        grab_box_result.add_attribute("📦 箱子信息", box_info)

        SDKLogger.info(f"📦 箱子信息:{box_info}")
        
        if not self.grasp_strategy.arm_move_to_target(
            copy.deepcopy(box_info),
            arm_mode="manipulation_mpc",
            sim_mode=True
        ):
            SDKLogger.info("❌ 手臂移动失败，无法继续执行")
            return False
        
        SDKLogger.info("✅ 手臂已到达抓取位置")        

        # 步骤4：提起箱子
        SDKLogger.info("\n4. 提起箱子...")
        if not self.grasp_strategy.arm_transport_target_up(
            copy.deepcopy(box_info),
            arm_mode="manipulation_mpc",
            sim_mode=True
        ):
            SDKLogger.info("❌ 提起箱子失败")
            return False
                    
        SDKLogger.info("✅ 成功提起箱子")
        time.sleep(1.0)  # 展示一下成功提起的状态
        return True

    @on_exit(mark="寻找放置位置Tag", action=exit_check)
    def find_placement_tag(self):
        self.robot.disable_head_tracking()
        self.placement_tag_data = None
        retry_count = 1
        max_retry_count = 3
        while retry_count <= max_retry_count:
            if not self.grasp_strategy.head_find_target(
                copy.deepcopy(self.placement_tag), 
                max_search_time=15.0,
                search_pattern="rotate_body" #  rotate_head
            ):
                SDKLogger.info(f"❌ 第{retry_count}次寻找放置位置Tag失败")
                retry_count += 1
                continue
            self.placement_tag_data = self.robot_vision.get_data_by_id_from_odom(self.placement_tag.id[0])
            if self.placement_tag_data is None:
                SDKLogger.info(f"❌ 未识别到 AprilTag ID 为{self.placement_tag.id[0]} 的放置位置Tag")
                retry_count += 1
                continue
            break
        if retry_count > max_retry_count:
            SDKLogger.info("❌ 寻找放置位置Tag失败")
            return False
        
        grab_box_result.add_attribute("📦 放置位置Tag数据", self.placement_tag_data)
        return True
    
    @on_exit(mark="靠近放置位置", action=exit_check)
    def walk_approach_placement(self):
        retry_count = 1
        max_retry_count = 3
        while retry_count <= max_retry_count:
            if not self.grasp_strategy.walk_approach_target(
                self.placement_tag.id[0],
                target_distance=0.4,  # 与目标 Apriltag 保持0.5米的距离
                approach_speed=0.2    # 接近速度0.2米/秒
            ):
                SDKLogger.info(f"❌ 第{retry_count}次接近放置位置失败")
                retry_count += 1
                pitch_angles_deg = [25, -25]  # 两档pitch角度：抬头和低头，角度制
                yaw_angles_deg = [-30, -15, 0, 15, 30]  # 左右扫描的yaw角度，角度制
                for pitch_deg in pitch_angles_deg:
                    for yaw_deg in yaw_angles_deg:
                        yaw_rad = yaw_deg * 0.0174533  # 度转弧度，math.pi/180
                        pitch_rad = pitch_deg * 0.0174533  # 度转弧度
                        self.robot.control_head(yaw=yaw_rad, pitch=pitch_rad)
                        time.sleep(0.5)
                        target_data = self.robot_vision.get_data_by_id_from_odom(self.placement_tag.id[0])
                        SDKLogger.debug(f"target_data: {target_data}")
                        if (target_data is not None and isinstance(target_data, dict) and 
                        'poses' in target_data and len(target_data['poses']) > 0):
                            self.robot.enable_head_tracking(self.placement_tag.id[0])
                            break
                continue
            break
        if retry_count > max_retry_count:
            SDKLogger.info("❌ 接近放置位置失败")
            return False
        grab_box_result.add_attribute("📍 靠近放置位置点位姿", self.robot_state.odometry)
        return True
    
    @on_exit(mark="放下箱子", action=exit_check)
    def put_box(self):
        # 步骤6：放下箱子
        SDKLogger.info("\n6. 放下箱子...")
        placement_box_info = BoxInfo.from_apriltag(
            self.placement_tag_data,
            xyz_offset=(self.box_size[0]/2+0.05, 0.0, -0.5), # tag 粘贴在货架上，需要把箱子放下距离货架的高度 -0.5m
            size=self.box_size,  # 箱子尺寸(长、宽、高)，单位：米
            mass=self.box_mass  # 箱子重量，单位：千克
        )
        grab_box_result.add_attribute("📦 放置位置箱子信息", placement_box_info)

        if not self.grasp_strategy.arm_transport_target_down(
            placement_box_info,
            arm_mode="manipulation_mpc"        ):
            SDKLogger.info("❌ 放下箱子失败")
            return False
        time.sleep(1.0)  # 短暂暂停
        return True
    
    def walk_back(self):
        # 步骤7: 回到初始位置
        SDKLogger.info("8. 回到初始位置...")
       # 步骤7: 回到初始位置
        self.robot.disable_head_tracking()
        # 先后退
        start_time = time.time()
        while time.time() - start_time < 5.0:
            self.robot.walk(-0.15, 0.0, 0.0)
            time.sleep(0.1)
        
        # 手臂归位
        self.robot.stance()
        self.robot.arm_reset()
        
        # 回到原点
        self.grasp_strategy.walk_to_pose(
            KuavoPose(
                position=(0.0, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0)
            ),
            target_distance=0.15,
            approach_speed=0.2,
            timeout=20.0
        )
        grab_box_result.add_attribute("📍 回到初始位置点位姿", self.robot_state.odometry)
        time.sleep(1.0)
        return True

def reset_block():
    
    # 等待服务可用
    rospy.wait_for_service('/gazebo/set_model_state', timeout=10.0)
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 设置新的箱子位置和朝向
        new_pose = Pose(
            position=Point(x=0.0, y=-1.7, z=1.2),  # 新坐标
            orientation=Quaternion(x=0, y=0, z=0, w=1)  # 四元数(w=1 表示无旋转)
        )
        
        # 创建请求消息
        model_state = ModelState()
        model_state.model_name = "block"  # 替换为你的箱子名称
        model_state.pose = new_pose
        model_state.reference_frame = "world"  # 参考坐标系
        
        # 调用服务
        resp = set_state(model_state)
        SDKLogger.info(f"箱子位置更新 {'成功' if resp.success else '失败'}！")
    
    except rospy.ServiceException as e:
        rospy.logerr("箱子位置重置服务调用失败: %s", e)

""" Main Function """
grab_box_running = False
grab_box_count = 1
def main():
    global grab_box_running,grab_box_count
    # !!! IMPORTANT 初始化 KuavoSDK 是必须的, 切记!
    if not KuavoSDK.Init(options=KuavoSDK.Options.Normal, log_level="DEBUG"):
        SDKLogger.info("❌ 初始化失败")
        os._exit(1)
    
    grab_box_running = True

    # 创建线程执行抓取策略
    def execute_grab_box():
        try:
            global grab_box_running,grab_box_count
            while grab_box_count <= 1:
                # reset_block()
                SDKLogger.info(f"========== 开始执行箱子抓取策略 {grab_box_count} 次 ==========")  
                grab_box_test = GrabBoxTest()
                grab_box_test.ready_to_grab_box()
                grab_box_test.find_apriltag()
                grab_box_test.walk_approach_box()
                grab_box_test.grab_box()
                grab_box_test.find_placement_tag()
                grab_box_test.walk_approach_placement()
                grab_box_test.put_box()
                grab_box_test.walk_back()
                SDKLogger.info(f"\n========== 搬箱子任务完成 {grab_box_count} 次 ==========")
                print_result(True)
                grab_box_count += 1
        except Exception as e:
            SDKLogger.info(f"搬运过程中发生错误: {e}")
            print_result(False)
            os._exit(1)
        grab_box_running = False

    # 不搬运测试，只测试机器人是否能正常运行
    grab_box_thread = threading.Thread(target=execute_grab_box)
    grab_box_thread.start()

    # main thread, check if robot is alive
    robot_monitor = RobotMonitor()
    start_time = time.time()
    timeout = 4*60  # 4 minutes in seconds
    while True and grab_box_running:
        if not robot_monitor.alive():
            SDKLogger.info("❌ Robot is not alive, exiting...")
            print_result(False)
            os._exit(1)
            
        if time.time() - start_time > timeout:
            SDKLogger.info(f"❌ Task timed out after {timeout/60} minutes, exiting...")
            print_result(False)
            os._exit(1)
        time.sleep(0.5)
    
    # print result
    os._exit(0)

if __name__ == "__main__":
    main()  