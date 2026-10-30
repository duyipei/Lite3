#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Foxy
source /opt/ros/galactic/setup.bash

# 默认查看 Mid-360 原始点云
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/rslidar_points}"
FIXED_FRAME="${FIXED_FRAME:-rslidar}"

CONFIG="/tmp/lite3_lidar_view.rviz"

if ! command -v rviz2 >/dev/null 2>&1; then
    echo "[ERROR] 找不到 rviz2，请检查 ROS 2 Foxy 环境。"
    exit 1
fi

echo "======================================"
echo " Lite3 Mid-360 三维点云查看器"
echo "======================================"
echo "PointCloud topic: ${POINTCLOUD_TOPIC}"
echo "Fixed frame:     ${FIXED_FRAME}"
echo
echo "当前话题信息："
ros2 topic info "${POINTCLOUD_TOPIC}" || true
echo

cat > "${CONFIG}" <<RVIZ
Panels:
  - Class: rviz_common/Displays
    Name: Displays

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Autocompute Intensity Bounds: false
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      History Length: 1
      Name: Mid360 PointCloud
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.03
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: ${POINTCLOUD_TOPIC}
      Use Fixed Frame: true
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 160
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: <Fixed Frame>
      Value: true

  Enabled: true
  Global Options:
    Background Color: 35; 35; 35
    Fixed Frame: ${FIXED_FRAME}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 6
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Near Clip Distance: 0.01
      Pitch: 0.65
      Target Frame: <Fixed Frame>
      Yaw: 0.75
    Saved: ~

RVIZ

echo "[INFO] 正在启动 RViz2..."
exec rviz2 -d "${CONFIG}"
