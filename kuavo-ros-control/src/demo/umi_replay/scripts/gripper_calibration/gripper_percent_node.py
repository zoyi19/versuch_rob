#!/usr/bin/env python3
"""
ROS node: real-time gripper opening percentage from D405 ArUco detection.

Subscribes to D405 colour images, detects the two finger ArUco tags,
computes the gripper width, and publishes a 0-100 % opening percentage
(or -1 when detection fails).

Usage:
    python scripts/gripper_percent_node.py \
        --calibration data/0312/gripper_range.json

    # With custom image topic
    python scripts/gripper_percent_node.py \
        --calibration data/0312/gripper_range.json \
        --image-topic /camera/color/image_raw
"""

import json
import argparse
import numpy as np
import cv2
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image


MARKER_SIZE_M = 0.016  # 16 mm finger markers
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50


def get_gripper_width(tag_dict, left_id, right_id, nominal_z, z_tolerance):
    zmax = nominal_z + z_tolerance
    zmin = nominal_z - z_tolerance

    left_x = None
    if left_id in tag_dict:
        tvec = tag_dict[left_id]['tvec']
        if zmin < tvec[-1] < zmax:
            left_x = tvec[0]

    right_x = None
    if right_id in tag_dict:
        tvec = tag_dict[right_id]['tvec']
        if zmin < tvec[-1] < zmax:
            right_x = tvec[0]

    if (left_x is not None) and (right_x is not None):
        return right_x - left_x
    if left_x is not None:
        return abs(left_x) * 2
    if right_x is not None:
        return abs(right_x) * 2
    return None


class GripperPercentNode:
    def __init__(self, calibration_path, image_topic, marker_size):
        with open(calibration_path, 'r') as f:
            cal = json.load(f)

        self.left_id = int(cal['left_finger_tag_id'])
        self.right_id = int(cal['right_finger_tag_id'])
        self.min_width = float(cal['min_width'])
        self.max_width = float(cal['max_width'])
        self.nominal_z = float(cal['nominal_z'])
        self.z_tolerance = float(cal['z_tolerance'])
        self.width_range = self.max_width - self.min_width
        self.marker_size = marker_size

        intr = cal.get('camera_intrinsics', {})
        fx = float(intr.get('fx', 425.0))
        fy = float(intr.get('fy', 425.0))
        cx = float(intr.get('cx', 424.0))
        cy = float(intr.get('cy', 240.0))
        self.K = np.array([[fx, 0, cx],
                           [0, fy, cy],
                           [0,  0,  1]], dtype=np.float64)
        self.D = np.zeros((1, 5), dtype=np.float64)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self.aruco_param = cv2.aruco.DetectorParameters_create()
        self.aruco_param.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.pub = rospy.Publisher('/gripper_percent', Float32, queue_size=1)
        self.sub = rospy.Subscriber(image_topic, Image, self._cb, queue_size=1,
                                    buff_size=2**24)

        rospy.loginfo(
            "GripperPercent ready  tag %d+%d  width [%.1f, %.1f] mm  "
            "nominal_z=%.4f m  topic=%s",
            self.left_id, self.right_id,
            self.min_width * 1000, self.max_width * 1000,
            self.nominal_z, image_topic)

    # ------------------------------------------------------------------
    def _cb(self, msg):
        try:
            if msg.encoding in ('rgb8',):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding in ('bgr8',):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            else:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Image decode error: %s", e)
            self.pub.publish(Float32(data=-1.0))
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            image=img, dictionary=self.aruco_dict,
            parameters=self.aruco_param)

        if ids is None or len(ids) == 0:
            self.pub.publish(Float32(data=-1.0))
            rospy.loginfo_throttle(2.0, "No ArUco tags detected")
            return

        tag_dict = {}
        for tag_id, tag_corners in zip(ids.flatten(), corners):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                tag_corners.reshape(1, 4, 2),
                self.marker_size, self.K, self.D)
            tag_dict[int(tag_id)] = {
                'tvec': tvecs.squeeze(),
                'rvec': rvecs.squeeze(),
            }

        width = get_gripper_width(
            tag_dict, self.left_id, self.right_id,
            self.nominal_z, self.z_tolerance)

        if width is None:
            self.pub.publish(Float32(data=-1.0))
            rospy.loginfo_throttle(2.0, "Width calculation failed (z filter)")
            return

        pct = (width - self.min_width) / self.width_range * 100.0
        pct = float(np.clip(pct, 0.0, 100.0))

        self.pub.publish(Float32(data=pct))
        rospy.loginfo_throttle(0.5,
                               "Gripper: %.1f%%  width=%.1f mm", pct, width * 1000)


def main():
    parser = argparse.ArgumentParser(
        description='ROS node: gripper opening percentage from ArUco tags')
    parser.add_argument('--calibration', required=True,
                        help='Path to gripper_range.json')
    parser.add_argument('--image-topic', default='/camera/color/image_raw',
                        help='D405 colour image topic')
    parser.add_argument('--marker-size', type=float, default=MARKER_SIZE_M,
                        help='ArUco marker physical size in metres')

    args, _ = parser.parse_known_args()

    rospy.init_node('gripper_percent_node', anonymous=False)

    GripperPercentNode(
        calibration_path=args.calibration,
        image_topic=args.image_topic,
        marker_size=args.marker_size,
    )

    rospy.spin()


if __name__ == '__main__':
    main()
