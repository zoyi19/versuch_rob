# 夹爪开合百分比标定系统

通过 D405 相机检测夹爪两手指上的 ArUco 标签间距，映射为 0~100% 开合百分比。

---

## 目录结构

```
gripper_calibration/
├── README.md                       # 本文件
├── calibrate_gripper_from_bag.py   # 离线标定（运行一次）
├── gripper_percent_node.py         # 实时 ROS 节点（长期运行）
└── gripper_range.json              # 标定结果文件
```

---

## 硬件前提

| 项目 | 规格 |
|------|------|
| 相机 | Intel RealSense D405，848x480 RGB |
| 标签类型 | ArUco，字典 DICT_4X4_50 |
| 左手指标签 | Tag ID 0，物理尺寸 16mm×16mm |
| 右手指标签 | Tag ID 1，物理尺寸 16mm×16mm |
| 图像 Topic | `/camera/color/image_raw` (sensor_msgs/Image) |

---

## 依赖

```
numpy
opencv-python          # cv2，需包含 aruco 模块（opencv-contrib-python）
rosbag                 # 仅标定脚本需要
rospy                  # 仅实时节点需要
std_msgs               # 仅实时节点需要
sensor_msgs            # 仅实时节点需要
click                  # 仅标定脚本需要
matplotlib             # 可选，标定脚本 --plot 功能
```

---

## 工作流程

```
步骤1：离线标定（只需做一次）
  录制夹爪开合 bag → calibrate_gripper_from_bag.py → gripper_range.json

步骤2：实时使用（长期运行）
  D405 实时图像 + gripper_range.json → gripper_percent_node.py → /gripper_percent topic (0~100%)
```

---

## 程序 1：calibrate_gripper_from_bag.py（离线标定）

### 用途

从录制好的 ROS bag（包含夹爪多次开合的 D405 图像）中，自动检测 ArUco 标签并计算夹爪宽度的最小/最大值，生成标定文件。

### 输入

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `--bag` | 文件路径 | 是 | - | 包含 D405 图像的 ROS bag 文件 |
| `--output` / `-o` | 文件路径 | 是 | - | 输出 gripper_range.json 路径 |
| `--topic` | 字符串 | 否 | `/camera/color/image_raw` | D405 图像 topic 名称 |
| `--fx` | float | 否 | 425.0 | D405 焦距 fx |
| `--fy` | float | 否 | 425.0 | D405 焦距 fy |
| `--cx` | float | 否 | 424.0 | D405 主点 cx |
| `--cy` | float | 否 | 240.0 | D405 主点 cy |
| `--marker-size` | float | 否 | 0.016 | ArUco 标签物理尺寸（米） |
| `--z-tolerance` | float | 否 | 0.015 | 深度过滤容差（米） |
| `--tag-det-threshold` | float | 否 | 0.5 | 最低标签检测率 |
| `--plot` | flag | 否 | - | 生成宽度-时间曲线图 |

### 输出

`gripper_range.json` 文件，加 `--plot` 时额外生成 PNG 曲线图。

### 运行示例

```bash
python calibrate_gripper_from_bag.py \
  --bag /path/to/gripper_open_close.bag \
  --output gripper_range.json \
  --plot
```

### 录制 bag 要求

bag 中需包含 `/camera/color/image_raw` topic（D405 RGB 图像），录制过程中夹爪需完成若干次完整的全闭合→全张开循环。建议 3~5 次循环，时长 10~20 秒。

---

## 程序 2：gripper_percent_node.py（实时 ROS 节点）

### 用途

订阅 D405 实时图像，逐帧检测 ArUco 标签，将夹爪宽度映射为 0~100% 开合百分比，发布到 ROS topic。

### 输入

| 来源 | 说明 |
|------|------|
| ROS Topic `/camera/color/image_raw` | D405 实时 RGB 图像 (sensor_msgs/Image) |
| `gripper_range.json` | 标定文件（启动时加载） |

### 输出

| ROS Topic | 类型 | 说明 |
|-----------|------|------|
| `/gripper_percent` | std_msgs/Float32 | 0.0~100.0 = 开合百分比；-1.0 = 检测失败 |

### 命令行参数

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `--calibration` | 文件路径 | 是 | - | gripper_range.json 路径 |
| `--image-topic` | 字符串 | 否 | `/camera/color/image_raw` | D405 图像 topic |
| `--marker-size` | float | 否 | 0.016 | ArUco 标签物理尺寸（米） |

### 运行示例

```bash
# 确保 roscore 已启动
python gripper_percent_node.py \
  --calibration gripper_range.json
```

### 输出含义

| 值 | 含义 |
|----|------|
| 0.0 | 夹爪完全闭合 |
| 100.0 | 夹爪完全张开 |
| 50.0 | 夹爪张开一半 |
| -1.0 | 本帧检测失败（标签未检出或深度过滤） |

---

## gripper_range.json 格式说明

```json
{
  "gripper_id": 0,
  "left_finger_tag_id": 0,
  "right_finger_tag_id": 1,
  "max_width": 0.13228,
  "min_width": 0.02991,
  "nominal_z": 0.08816,
  "z_tolerance": 0.015,
  "valid_frames": 473,
  "total_frames": 503,
  "camera_intrinsics": {
    "fx": 425.0,
    "fy": 425.0,
    "cx": 424.0,
    "cy": 240.0,
    "resolution": "848x480"
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| `gripper_id` | int | 夹爪编号（0 = 第一个夹爪） |
| `left_finger_tag_id` | int | 左手指 ArUco 标签 ID |
| `right_finger_tag_id` | int | 右手指 ArUco 标签 ID |
| `max_width` | float | 全张开时两标签间距（米），对应 100% |
| `min_width` | float | 全闭合时两标签间距（米），对应 0% |
| `nominal_z` | float | 标签在相机坐标系中的典型 Z 深度（米） |
| `z_tolerance` | float | Z 深度过滤容差（米） |
| `camera_intrinsics` | object | 标定时使用的 D405 相机内参 |

---

## 从其他 Python 程序调用

### 方法 1：订阅 ROS Topic

最简单的方式——启动 `gripper_percent_node.py` 后，在你的程序中订阅 `/gripper_percent`：

```python
import rospy
from std_msgs.msg import Float32

current_percent = -1.0

def gripper_cb(msg):
    global current_percent
    current_percent = msg.data  # 0~100 或 -1

rospy.Subscriber('/gripper_percent', Float32, gripper_cb, queue_size=1)

# 在你的控制循环中使用
if current_percent >= 0:
    print(f"夹爪开度: {current_percent:.1f}%")
```

### 方法 2：直接 import 核心函数

如果不想运行独立节点，可以把核心计算逻辑直接嵌入你的程序：

```python
import json
import numpy as np
import cv2

# ---- 从 gripper_percent_node.py 复制的核心函数 ----
def get_gripper_width(tag_dict, left_id, right_id, nominal_z, z_tolerance):
    zmax = nominal_z + z_tolerance
    zmin = nominal_z - z_tolerance
    left_x = right_x = None
    if left_id in tag_dict:
        tvec = tag_dict[left_id]['tvec']
        if zmin < tvec[-1] < zmax:
            left_x = tvec[0]
    if right_id in tag_dict:
        tvec = tag_dict[right_id]['tvec']
        if zmin < tvec[-1] < zmax:
            right_x = tvec[0]
    if left_x is not None and right_x is not None:
        return right_x - left_x
    if left_x is not None:
        return abs(left_x) * 2
    if right_x is not None:
        return abs(right_x) * 2
    return None

# ---- 初始化（只需一次）----
with open('gripper_range.json') as f:
    cal = json.load(f)

LEFT_ID = cal['left_finger_tag_id']      # 0
RIGHT_ID = cal['right_finger_tag_id']    # 1
MIN_W = cal['min_width']                 # 0.02991
MAX_W = cal['max_width']                 # 0.13228
NOM_Z = cal['nominal_z']                 # 0.08816
Z_TOL = cal['z_tolerance']              # 0.015
W_RANGE = MAX_W - MIN_W

intr = cal['camera_intrinsics']
K = np.array([[intr['fx'], 0, intr['cx']],
              [0, intr['fy'], intr['cy']],
              [0, 0, 1]], dtype=np.float64)
D = np.zeros((1, 5), dtype=np.float64)

ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAM = cv2.aruco.DetectorParameters_create()
ARUCO_PARAM.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
MARKER_SIZE = 0.016  # 16mm

# ---- 单帧计算函数 ----
def compute_gripper_percent(img):
    """
    输入: img -- numpy array (H, W, 3), D405 RGB 图像
    返回: float, 0~100 为开合百分比, -1 为检测失败
    """
    corners, ids, _ = cv2.aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAM)
    if ids is None:
        return -1.0
    tag_dict = {}
    for tag_id, tag_corners in zip(ids.flatten(), corners):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            tag_corners.reshape(1, 4, 2), MARKER_SIZE, K, D)
        tag_dict[int(tag_id)] = {'tvec': tvecs.squeeze()}
    width = get_gripper_width(tag_dict, LEFT_ID, RIGHT_ID, NOM_Z, Z_TOL)
    if width is None:
        return -1.0
    pct = (width - MIN_W) / W_RANGE * 100.0
    return float(np.clip(pct, 0.0, 100.0))

# ---- 使用示例 ----
# img = <从 D405 获取的 RGB 图像>
# percent = compute_gripper_percent(img)
```

### 方法 3：作为模块导入

将本目录加入 Python 路径后直接 import：

```python
import sys
sys.path.append('/home/leju/catkin_ws/src/wheel/gripper_calibration')

from gripper_percent_node import GripperPercentNode, get_gripper_width
```

---

## 百分比计算公式

```
percent = clamp((width - min_width) / (max_width - min_width) * 100, 0, 100)
```

其中：
- `width` = 当前帧两 ArUco 标签的 X 方向间距（米），由 `estimatePoseSingleMarkers` 的 tvec 计算
- `min_width` = 标定时测得的全闭合间距（0.02991m）
- `max_width` = 标定时测得的全张开间距（0.13228m）

---

## 注意事项

1. **相机内参**：当前使用 D405 在 848x480 下的典型近似值（fx=fy=425, cx=424, cy=240）。如果更换相机或分辨率，需重新标定。
2. **标定一致性**：标定和实时运行必须使用同一台 D405 + 同一组内参，否则百分比映射会不准。
3. **标签要求**：ArUco DICT_4X4_50，Tag 0 在左手指，Tag 1 在右手指，物理尺寸 16mm。
4. **OpenCV 版本**：使用 `cv2.aruco.DetectorParameters_create()`（兼容 OpenCV 4.2.0）。
5. **检测失败**：当输出 -1.0 时表示本帧未检测到标签或深度过滤未通过，调用方应做容错处理。
