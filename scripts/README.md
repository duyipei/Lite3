# scripts 目录说明

此目录只保留四条正式工作流的入口和直接依赖。现场优先使用入口脚本，不要单独运行
后端辅助脚本。

`docs/` 中按日期保存的旧实验记录可能仍提到已经删除的专项诊断或冒烟测试脚本；这些
记录用于追溯历史，不代表当前入口。

## 1. 室外建图

入口：`outdoor_lidar_visual_mapping.sh`

它调用共享采集入口 `round3a_capture.sh`，在 Jetson 同步记录 Mid360、IMU、RGB、Depth
和时间探针。移动只使用官方遥控器。

## 2. 室内建图

入口：`indoor_lidar_visual_mapping.sh`

采集后端与室外相同，但默认生成 `indoor_YYYYMMDD_HHMMSS` 编号。

## 3. 地图生成

按数据处理阶段保留：

- `merge_round3a_lio_bag.py`：生成 LiDAR + IMU 派生 bag；
- `round3a_lio_offline_backend.sh`：运行隔离 FAST-LIVO2；
- `analyze_lio_odometry.py`、`count_fast_livo_outputs.py`、`monitor_pid.py`、
  `profile_fast_livo.py`：离线 LIO 后端的直接依赖；
- `render_round3a_map.py`：输出点云与轨迹预览；
- `build_rgbd_visual_map.py`：生成视觉关键帧数据库；
- `validate_visual_map_db.py`、`visualize_rgbd_visual_map.py`：视觉地图质量检查；
- `build_area_traversability_map.py`：生成可通行二维栅格图；
- `make_safe_nav2_config.py`：生成低速 Nav2 参数。

详细命令见 `../docs/2026-09-12_indoor_map_generation_navigation.md`。

## 4. 导航启动

入口：`indoor_nav_start.sh`。虽然保留了历史文件名，但通过显式 MAP_ID 同时支持室内和
室外地图。

其直接依赖是：

- `leg_odom_tf_bridge.py`
- `goal_pose_bridge.py`
- `navigation_result_notifier.py`
- `rviz_pointcloud_relay.py`

当前室外导航命令：

```bash
LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./indoor_nav_start.sh start outdoor_20260921_003_area_review
```

`start` 完成后速度门仍为 DISARMED，不会自动让机器狗运动。

## 共享采集后端

以下文件由两个建图入口间接调用，不是冗余文件：

- `round3a_capture.sh`
- `round3a_capture_backend.sh`
- `round3a_live_video.py`
- `mid360_isolated.launch.py`
- `time_probe.py`
