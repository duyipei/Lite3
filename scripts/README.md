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

## 5. 多点巡航与转向诊断（2026-10-09）

在项目根目录执行 `bash scripts/patrol_route.sh check` 可以离线校验任务。
导航终端达到 `NAVIGATION READY` 且速度门为 DISARMED 后，在另一个终端执行
`bash scripts/patrol_route.sh run`；现场保护就绪后输入 ARM，授权本次 11 点巡航。
运行中不要在 RViz 另发目标；失败后不会自动重试或重新解锁。

10 月 8 日 16:57 日志显示前 4 点到达，第 5 点的目标方位误差起初为 +82.8 度，
门后角速度却主要为 -0.0267 rad/s，航向从 -8.7 度降至约 -149 度。
这与更早“第 1 点轮廓碰到地图障碍”的失败不同，也不支持直接反转底盘角速度符号。
尾部日志截断，不能从该日志断言最后是否已确认停止。

本次离线修正：

- 当前室外 `nav2_safe.yaml` 和参数生成器统一采用 PathAlign/PathDist 的 `last`
  聚合，避免距离采样模式下 `sum` 对采样点多的转向轨迹额外罚分。
- `min_speed_xy=0.03`、`min_speed_theta=0.10` 联合过滤微小平移且微小旋转的
  候选；仍允许零平移的正常纯旋转，不在速度门后强行增大角速度。
- PathAlign/GoalAlign 前视点由 0.10 m 调为 0.325 m，改善 0.10 m 栅格中的方向区分。
- 多点执行器在低平移速度、明显朝向误差下，若连续 15 秒既未接近目标 0.15 m，
  也未改善朝向误差 0.10 rad，则走原有停止/取消/锁定流程。
  该保护可能中止有意绕远的转向，不会代替 Nav2 规划或强制走直线。
- 运动日志增加原始 Nav2 速度、门后速度及样本时效。保留碰撞检查、定位门限、
  最大线速度 0.15 m/s、最大角速度 0.40 rad/s 以及任务原始坐标/航向。

这是待实机验收的参数修正，不是整条路线成功的证明。历史日志没有候选轨迹评分，
尚不能排除转弯现场代价地图对方向选择的影响。参数须结束旧导航会话后重新 start
才会部署并生效；只重启 `patrol_route.sh` 不会更新正在运行的 Nav2 参数。

离线回归命令（不会连接或驱动机器狗）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_patrol_turning.py
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_patrol_route.py
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_patrol_route_prompt.py
```

机制核对来源：[Foxy 速度采样器](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_plugins/src/xy_theta_iterator.cpp)、
[轨迹采样](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp)、
[栅格评分](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_critics/src/map_grid.cpp)。
原厂分支的实际表现仍须在 Jetson 验证。

## 共享采集后端

以下文件由两个建图入口间接调用，不是冗余文件：

- `round3a_capture.sh`
- `round3a_capture_backend.sh`
- `round3a_live_video.py`
- `mid360_isolated.launch.py`
- `time_probe.py`
