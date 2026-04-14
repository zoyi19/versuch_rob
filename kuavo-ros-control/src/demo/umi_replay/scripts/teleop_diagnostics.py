#!/usr/bin/env python3
"""
Teleoperation diagnostics recorder.

Subscribes to key topics in the teleop pipeline, logs position data
to a CSV file for offline jitter analysis, and prints live summaries.

Usage:
    python3 teleop_diagnostics.py
    # Ctrl+C to stop and save CSV

Output CSV columns:
    timestamp, umi_x, umi_y, umi_z, cmd_x, cmd_y, cmd_z, ee_x, ee_y, ee_z
"""

import os
import csv
import time
import threading
from datetime import datetime

import numpy as np
import rospy
from nav_msgs.msg import Odometry

# Try importing project-specific messages
try:
    from kuavo_msgs.msg import twoArmHandPoseCmd
    HAS_KUAVO_MSGS = True
except ImportError:
    HAS_KUAVO_MSGS = False
    rospy.logwarn("kuavo_msgs not found; /mm/two_arm_hand_pose_cmd will not be recorded")

from std_msgs.msg import Float64MultiArray


class TeleopDiagnostics:
    def __init__(self):
        rospy.init_node("teleop_diagnostics", anonymous=True)

        # Latest values (thread-safe via GIL for simple assignments)
        self._umi = None       # (x, y, z)
        self._cmd = None       # (x, y, z)
        self._ee = None        # (x, y, z)

        # Counters for hz measurement
        self._counts = {"umi": 0, "cmd": 0, "ee": 0}
        self._lock = threading.Lock()

        # CSV output
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.expanduser(f"~/ziyi/teleop_diag_{ts}.csv")
        self._csv_file = open(self._csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "timestamp",
            "umi_x", "umi_y", "umi_z",
            "cmd_x", "cmd_y", "cmd_z",
            "ee_x", "ee_y", "ee_z",
        ])
        self._row_count = 0

        # Subscribers
        rospy.Subscriber("/umi_odom", Odometry, self._cb_umi, queue_size=1)

        if HAS_KUAVO_MSGS:
            rospy.Subscriber("/mm/two_arm_hand_pose_cmd", twoArmHandPoseCmd,
                             self._cb_cmd, queue_size=1)

        rospy.Subscriber("/humanoid_wheel/eePoses", Float64MultiArray,
                         self._cb_ee, queue_size=1)

        # Summary timer
        self._summary_timer = rospy.Timer(rospy.Duration(2.0), self._print_summary)

        rospy.loginfo(f"Diagnostics recording to: {self._csv_path}")
        rospy.loginfo("Subscribed to: /umi_odom, /mm/two_arm_hand_pose_cmd, /humanoid_wheel/eePoses")
        rospy.loginfo("Press Ctrl+C to stop and save.")

    # ---- Callbacks ----

    def _cb_umi(self, msg):
        p = msg.pose.pose.position
        self._umi = (p.x, p.y, p.z)
        with self._lock:
            self._counts["umi"] += 1
        self._write_row()

    def _cb_cmd(self, msg):
        lp = msg.hand_poses.left_pose.pos_xyz
        self._cmd = (lp[0], lp[1], lp[2])
        with self._lock:
            self._counts["cmd"] += 1
        self._write_row()

    def _cb_ee(self, msg):
        # Float64MultiArray: first 3 values assumed to be left EE xyz
        if len(msg.data) >= 3:
            self._ee = (msg.data[0], msg.data[1], msg.data[2])
            with self._lock:
                self._counts["ee"] += 1
            self._write_row()

    # ---- Recording ----

    def _write_row(self):
        t = rospy.Time.now().to_sec()
        umi = self._umi or (float("nan"),) * 3
        cmd = self._cmd or (float("nan"),) * 3
        ee = self._ee or (float("nan"),) * 3
        self._csv_writer.writerow([
            f"{t:.6f}",
            f"{umi[0]:.6f}", f"{umi[1]:.6f}", f"{umi[2]:.6f}",
            f"{cmd[0]:.6f}", f"{cmd[1]:.6f}", f"{cmd[2]:.6f}",
            f"{ee[0]:.6f}", f"{ee[1]:.6f}", f"{ee[2]:.6f}",
        ])
        self._row_count += 1

    # ---- Summary ----

    def _print_summary(self, event):
        with self._lock:
            counts = dict(self._counts)
            for k in self._counts:
                self._counts[k] = 0

        hz_umi = counts["umi"] / 2.0
        hz_cmd = counts["cmd"] / 2.0
        hz_ee = counts["ee"] / 2.0

        # Tracking error
        err_str = "N/A"
        if self._cmd is not None and self._ee is not None:
            err = np.linalg.norm(
                np.array(self._cmd) - np.array(self._ee))
            err_str = f"{err:.4f} m"

        rospy.loginfo(
            f"[diag] umi={hz_umi:.0f}Hz  cmd={hz_cmd:.0f}Hz  ee={hz_ee:.0f}Hz  "
            f"track_err={err_str}  rows={self._row_count}")

    # ---- Shutdown ----

    def shutdown(self):
        self._csv_file.close()
        rospy.loginfo(f"Saved {self._row_count} rows to {self._csv_path}")


def main():
    diag = TeleopDiagnostics()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        diag.shutdown()


if __name__ == "__main__":
    main()
