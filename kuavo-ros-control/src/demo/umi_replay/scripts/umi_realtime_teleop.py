#!/usr/bin/env python3
"""
实时 UMI 遥操作: 订阅 mocap 位姿，通过增量式 MPC 控制左臂跟随。

数据流:
  /umi_odom + /orbbec_odom  →  坐标变换  →  增量+FHAN  →  MPC (frame=2)
  /gripper_percent           →  夹爪映射  →  /leju_claw_command

状态机:
  IDLE  --[Enter]-->  INIT  --[自动]-->  TRACKING  --[q/Esc]-->  IDLE

前置条件:
  1. MuJoCo 仿真: roslaunch humanoid_controllers load_kuavo_mujoco_sim_wheel.launch
  2. 动捕推流(外部): 发布 /umi_odom, /orbbec_odom
  3. 夹爪检测(可选): python3 gripper_percent_node.py --calibration gripper_range.json
"""

import math
import sys
import select
import termios
import tty
import threading
import enum
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import rospy
import tf2_ros
from std_msgs.msg import Header, Float32
from nav_msgs.msg import Odometry
from kuavo_msgs.msg import (
    twoArmHandPoseCmd,
    armHandPose,
    lejuClawCommand,
    endEffectorData,
)
from kuavo_msgs.srv import changeTorsoCtrlMode, changeTorsoCtrlModeRequest


# ===================================================================
# FHAN smoother (from leju_utils/math.hpp)
# ===================================================================

def _sign(x):
    if -1e-6 < x < 1e-6:
        return 0.0
    return 1.0 if x >= 1e-6 else -1.0


def _fsg(x, d):
    return (_sign(x + d) - _sign(x - d)) / 2.0


def _fhan(x1, x2, r, h0):
    d = r * h0 * h0
    a0 = h0 * x2
    y = x1 + a0
    a1 = math.sqrt(d * (d + 8.0 * abs(y)))
    a2 = a0 + _sign(y) * (a1 - d) / 2.0
    sy = _fsg(y, d)
    a = (a0 + y) * sy + a2 * (1.0 - sy)
    sa = _fsg(a, d)
    return -r * (a / d) * sa - r * _sign(a) * (1.0 - sa)


class FHANSmoother:
    def __init__(self, r, h, h0):
        self.r = r
        self.h = h
        self.h0 = h0
        self.x = np.zeros(3)
        self.dx = np.zeros(3)

    def reset(self):
        self.x[:] = 0.0
        self.dx[:] = 0.0

    def step(self, ref):
        for i in range(3):
            fh = _fhan(self.x[i] - ref[i], self.dx[i], self.r, self.h0)
            self.x[i] += self.h * self.dx[i]
            self.dx[i] += self.h * fh
        return self.x.copy()


# ===================================================================
# Coordinate transform helpers
# ===================================================================

def make_transform(pos, quat_xyzw):
    T = np.eye(4)
    T[:3, :3] = Rot.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = pos
    return T


def decompose_transform(T):
    pos = T[:3, 3].copy()
    quat_xyzw = Rot.from_matrix(T[:3, :3]).as_quat()
    return pos, quat_xyzw


def odom_to_pose(msg):
    """Extract (pos, quat_xyzw) from nav_msgs/Odometry."""
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    return (np.array([p.x, p.y, p.z]),
            np.array([o.x, o.y, o.z, o.w]))


# ===================================================================
# Service / TF helpers
# ===================================================================

def set_mpc_control_mode(mode):
    try:
        rospy.wait_for_service("/mobile_manipulator_mpc_control", timeout=5.0)
        srv = rospy.ServiceProxy("/mobile_manipulator_mpc_control",
                                 changeTorsoCtrlMode)
        req = changeTorsoCtrlModeRequest()
        req.control_mode = mode
        resp = srv(req)
        if resp.result:
            rospy.loginfo(f"MPC control mode set to {mode}")
        else:
            rospy.logwarn(f"MPC control mode switch failed: {resp.message}")
        return resp.result
    except Exception as e:
        rospy.logerr(f"MPC mode service error: {e}")
        return False


def tf_lookup_pose(tf_buf, target, source, timeout=1.0):
    """Lookup TF and return (pos, quat_xyzw) or (None, None)."""
    try:
        t = tf_buf.lookup_transform(target, source,
                                    rospy.Time(0), rospy.Duration(timeout))
        pos = np.array([t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z])
        quat = np.array([t.transform.rotation.x,
                         t.transform.rotation.y,
                         t.transform.rotation.z,
                         t.transform.rotation.w])
        return pos, quat
    except Exception as e:
        rospy.logwarn(f"TF lookup {target}->{source} failed: {e}")
        return None, None


# ===================================================================
# Message builders
# ===================================================================

def build_mpc_msg(left_pos, left_quat, right_pos, right_quat):
    msg = twoArmHandPoseCmd()
    msg.hand_poses.header = Header()
    msg.hand_poses.header.stamp = rospy.Time.now()
    msg.hand_poses.header.frame_id = "base_link"

    left = armHandPose()
    left.pos_xyz = [float(v) for v in left_pos]
    left.quat_xyzw = [float(v) for v in left_quat]
    left.elbow_pos_xyz = [0.0, 0.0, 0.0]
    left.joint_angles = [0.0] * 7

    right = armHandPose()
    right.pos_xyz = [float(v) for v in right_pos]
    right.quat_xyzw = [float(v) for v in right_quat]
    right.elbow_pos_xyz = [0.0, 0.0, 0.0]
    right.joint_angles = [0.0] * 7

    msg.hand_poses.left_pose = left
    msg.hand_poses.right_pose = right
    msg.frame = 2  # LocalFrame (base_link)
    msg.use_custom_ik_param = False
    msg.joint_angles_as_q0 = False
    return msg


def build_claw_msg(left_percent):
    msg = lejuClawCommand()
    msg.header.stamp = rospy.Time.now()
    msg.data = endEffectorData()
    msg.data.name = ["left_claw", "right_claw"]
    msg.data.position = [float(left_percent), 0.0]
    msg.data.velocity = [50.0, 50.0]
    msg.data.effort = [1.0, 1.0]
    return msg


# ===================================================================
# Keyboard reader (non-blocking, separate thread)
# ===================================================================

class KeyboardReader:
    """Non-blocking single-char reader running in a daemon thread."""

    def __init__(self):
        self._key = None
        self._lock = threading.Lock()
        self._old_settings = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running and not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    with self._lock:
                        self._key = ch
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, self._old_settings)

    def get_key(self):
        with self._lock:
            k = self._key
            self._key = None
            return k

    def stop(self):
        self._running = False


# ===================================================================
# Main teleop class
# ===================================================================

class State(enum.Enum):
    IDLE = 0
    INIT = 1
    TRACKING = 2


class UmiTeleop:
    MAX_DELTA_NORM = 0.35  # safety clamp (m)

    def __init__(self, args):
        self.args = args
        self.state = State.IDLE

        # Coordinate transform cache
        self._T_base_mocap = None

        # Anchors (set during INIT)
        self._umi_anchor_pos = None
        self._umi_anchor_quat = None
        self._robot_anchor_pos = None
        self._robot_anchor_quat = None
        self._right_arm_pos = None
        self._right_arm_quat = None

        # Latest data from subscribers
        self._latest_umi = None       # (pos, quat) in mocap frame
        self._latest_orbbec = None    # (pos, quat) in mocap frame
        self._latest_gripper = None   # float 0-100
        self._gripper_stamp = None

        # FHAN smoother
        dt = 1.0 / args.rate
        h0 = args.fhan_h0_scale * dt
        self._smoother = FHANSmoother(r=args.fhan_r, h=dt, h0=h0)

        # Orientation tracking
        self._q_target = None

        # Frame counter for logging
        self._frame_cnt = 0

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf)

        # Publishers
        self._pub_arm = rospy.Publisher("/mm/two_arm_hand_pose_cmd",
                                       twoArmHandPoseCmd, queue_size=10)
        self._pub_claw = rospy.Publisher("/leju_claw_command",
                                        lejuClawCommand, queue_size=10)

        # Subscribers
        rospy.Subscriber("/umi_odom", Odometry, self._cb_umi, queue_size=1)
        rospy.Subscriber("/orbbec_odom", Odometry, self._cb_orbbec, queue_size=1)
        if not args.no_gripper:
            rospy.Subscriber("/gripper_percent", Float32,
                             self._cb_gripper, queue_size=1)

        rospy.loginfo("UmiTeleop initialized. Waiting for data...")

    # ---- Subscriber callbacks ----

    def _cb_umi(self, msg):
        self._latest_umi = odom_to_pose(msg)

    def _cb_orbbec(self, msg):
        self._latest_orbbec = odom_to_pose(msg)

    def _cb_gripper(self, msg):
        self._latest_gripper = msg.data
        self._gripper_stamp = rospy.Time.now()

    # ---- Coordinate transform ----

    def _compute_bridge(self):
        """Compute T_base_mocap from TF (base->head) and orbbec odom."""
        if self._latest_orbbec is None:
            rospy.logwarn("No /orbbec_odom received yet")
            return False

        head_pos, head_quat = tf_lookup_pose(
            self._tf_buf, "base_link", self.args.head_frame, timeout=2.0)
        if head_pos is None:
            rospy.logwarn(f"TF base_link->{self.args.head_frame} not available")
            return False

        T_base_head = make_transform(head_pos, head_quat)
        T_mocap_orbbec = make_transform(*self._latest_orbbec)
        self._T_base_mocap = T_base_head @ np.linalg.inv(T_mocap_orbbec)

        rospy.loginfo(f"  T_base_head: pos=({head_pos[0]:.3f}, "
                      f"{head_pos[1]:.3f}, {head_pos[2]:.3f})")
        orb = self._latest_orbbec[0]
        rospy.loginfo(f"  T_mocap_orbbec: pos=({orb[0]:.3f}, "
                      f"{orb[1]:.3f}, {orb[2]:.3f})")
        return True

    def _umi_to_base(self, umi_pos, umi_quat):
        """Transform a single UMI pose from mocap to base_link."""
        T_mocap_umi = make_transform(umi_pos, umi_quat)
        T_base_umi = self._T_base_mocap @ T_mocap_umi
        return decompose_transform(T_base_umi)

    # ---- State machine transitions ----

    def do_init(self):
        """INIT: record anchors, set MPC mode."""
        rospy.loginfo("--- INIT: computing bridge transform ---")

        if self._latest_umi is None:
            rospy.logwarn("No /umi_odom data. Cannot initialize.")
            self.state = State.IDLE
            return

        if not self._compute_bridge():
            rospy.logwarn("Bridge transform failed. Returning to IDLE.")
            self.state = State.IDLE
            return

        # UMI anchor in base_link
        umi_base_pos, umi_base_quat = self._umi_to_base(*self._latest_umi)
        self._umi_anchor_pos = umi_base_pos
        self._umi_anchor_quat = umi_base_quat
        rospy.loginfo(f"  UMI anchor (base_link): ({umi_base_pos[0]:.3f}, "
                      f"{umi_base_pos[1]:.3f}, {umi_base_pos[2]:.3f})")

        # Robot left arm anchor via TF
        rospy.loginfo("  Querying robot left arm EEF via TF ...")
        left_pos, left_quat = tf_lookup_pose(
            self._tf_buf, "base_link", "zarm_l7_link", timeout=2.0)
        if left_pos is None:
            rospy.logwarn("Left arm TF failed. Using fallback [0, 0.3, 0.3].")
            left_pos = np.array([0.0, 0.3, 0.3])
            left_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._robot_anchor_pos = left_pos
        self._robot_anchor_quat = left_quat
        rospy.loginfo(f"  Robot left anchor: ({left_pos[0]:.3f}, "
                      f"{left_pos[1]:.3f}, {left_pos[2]:.3f})")

        # Robot right arm (hold position)
        rospy.loginfo("  Querying robot right arm EEF via TF ...")
        right_pos, right_quat = tf_lookup_pose(
            self._tf_buf, "base_link", "zarm_r7_link", timeout=2.0)
        if right_pos is None:
            rospy.logwarn("Right arm TF failed. Using fallback [0, -0.3, 0.3].")
            right_pos = np.array([0.0, -0.3, 0.3])
            right_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._right_arm_pos = right_pos
        self._right_arm_quat = right_quat
        rospy.loginfo(f"  Robot right anchor: ({right_pos[0]:.3f}, "
                      f"{right_pos[1]:.3f}, {right_pos[2]:.3f})")

        # Reset smoother and orientation state
        self._smoother.reset()
        self._q_target = Rot.from_quat(self._robot_anchor_quat)
        self._frame_cnt = 0

        # Activate MPC
        rospy.loginfo("  Setting MPC mode: ArmOnly ...")
        set_mpc_control_mode(1)
        rospy.sleep(0.3)

        rospy.loginfo("--- INIT complete. Entering TRACKING ---")
        rospy.loginfo("  Press 'q' or Esc to stop tracking.")
        self.state = State.TRACKING

    def do_stop(self):
        """Stop tracking, restore MPC mode."""
        rospy.loginfo("--- Stopping tracking ---")
        set_mpc_control_mode(0)
        self.state = State.IDLE
        rospy.loginfo("Returned to IDLE. Press Enter to start again.")

    # ---- Per-frame tracking ----

    def do_track_frame(self):
        """Compute one tracking frame and publish."""
        if self._latest_umi is None:
            return

        # Current UMI pose in base_link
        cur_pos, cur_quat = self._umi_to_base(*self._latest_umi)

        # Position delta relative to UMI anchor
        delta = cur_pos - self._umi_anchor_pos
        delta_scaled = delta * self.args.delta_scale

        # Safety clamp
        norm = np.linalg.norm(delta_scaled)
        if norm > self.MAX_DELTA_NORM:
            delta_scaled = delta_scaled / norm * self.MAX_DELTA_NORM

        # FHAN smooth
        smoothed = self._smoother.step(delta_scaled)
        target_pos = self._robot_anchor_pos + smoothed

        # Orientation
        if not self.args.no_orient:
            r_cur = Rot.from_quat(cur_quat)
            r_anchor = Rot.from_quat(self._umi_anchor_quat)
            delta_rot = r_cur * r_anchor.inv()
            self._q_target = delta_rot * Rot.from_quat(self._robot_anchor_quat)
        target_quat = self._q_target.as_quat()

        # Publish arm
        self._pub_arm.publish(build_mpc_msg(
            target_pos, target_quat,
            self._right_arm_pos, self._right_arm_quat))

        # Publish gripper
        if not self.args.no_gripper and self._latest_gripper is not None:
            if self._latest_gripper >= 0:
                self._pub_claw.publish(build_claw_msg(self._latest_gripper))

        # Logging
        self._frame_cnt += 1
        if self._frame_cnt <= 5 or self._frame_cnt % (self.args.rate * 2) == 0:
            rospy.loginfo(
                f"  f={self._frame_cnt}  target=({target_pos[0]:.3f}, "
                f"{target_pos[1]:.3f}, {target_pos[2]:.3f})  "
                f"|delta|={norm:.4f}  |smoothed|={np.linalg.norm(smoothed):.4f}")

    # ---- Main loop ----

    def run(self, keyboard):
        rate = rospy.Rate(self.args.rate)

        rospy.loginfo("========================================")
        rospy.loginfo("  UMI Realtime Teleop")
        rospy.loginfo("  Press Enter to start tracking")
        rospy.loginfo("  Press q or Esc to stop")
        rospy.loginfo("  Press Ctrl+C to quit")
        rospy.loginfo("========================================")

        while not rospy.is_shutdown():
            key = keyboard.get_key()

            if self.state == State.IDLE:
                if key == '\r' or key == '\n':
                    self.state = State.INIT

            elif self.state == State.INIT:
                self.do_init()

            elif self.state == State.TRACKING:
                if key == 'q' or key == '\x1b':  # q or Esc
                    self.do_stop()
                else:
                    self.do_track_frame()

            rate.sleep()


# ===================================================================
# Main
# ===================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Real-time UMI teleoperation via incremental MPC.")
    parser.add_argument("--head-frame", default="zhead_2_link",
                        help="Head TF frame for bridge transform (default: zhead_2_link)")
    parser.add_argument("--delta-scale", type=float, default=0.5,
                        help="Position delta scale factor (default: 0.5)")
    parser.add_argument("--rate", type=int, default=20,
                        help="Control loop rate Hz (default: 20)")
    parser.add_argument("--fhan-r", type=float, default=2.0,
                        help="FHAN acceleration bound (default: 2.0)")
    parser.add_argument("--fhan-h0-scale", type=float, default=5.0,
                        help="FHAN h0 = scale * dt (default: 5.0)")
    parser.add_argument("--no-gripper", action="store_true",
                        help="Disable gripper control")
    parser.add_argument("--no-orient", action="store_true",
                        help="Ignore orientation, track position only")
    parser.add_argument("--max-delta", type=float, default=0.35,
                        help="Max allowed delta norm in meters (default: 0.35)")
    args = parser.parse_args()

    rospy.init_node("umi_realtime_teleop", anonymous=True)

    teleop = UmiTeleop(args)
    UmiTeleop.MAX_DELTA_NORM = args.max_delta

    kb = KeyboardReader()
    try:
        teleop.run(kb)
    except rospy.ROSInterruptException:
        pass
    finally:
        kb.stop()
        set_mpc_control_mode(0)
        rospy.loginfo("UMI teleop shutdown.")


if __name__ == "__main__":
    main()
