# scripts 目录说明

此目录保留建图、地图生成、导航、多点巡航入口及直接依赖。现场优先使用入口脚本，
不要单独运行后端辅助脚本。离线测试已移至 `../tests/`，手动时钟维护工具移至
`../tools/sync_jetson_time.py`。整理记录见 `../docs/2026-10-13_项目文件整理.md`。

常用入口：统一建图 `mapping_start.sh`，
导航 `navigation_start.sh`，多点巡航 `patrol_route.sh`，预览 `camera_view.sh`。
旧名称 `indoor_nav_start.sh`、`round3a_capture.sh`、`round3a_live_video.py` 仅保留兼容转发，
分别对应 `navigation_start.sh`、`mapping_capture.sh`、`camera_preview.py`，不是重复实现。
Jetson 后端的 `round3a_*` 名称涉及部署路径与进程识别，暂时保留，不应手动删除。

`docs/` 中按日期保存的旧实验记录可能仍提到已经删除的专项诊断或冒烟测试脚本；这些
记录用于追溯历史，不代表当前入口。

## 1. 统一建图采集

入口：`mapping_start.sh`

它调用共享采集入口 `mapping_capture.sh`，在 Jetson 同步记录 Mid360、IMU、RGB、Depth
和时间探针。移动只使用官方遥控器。

室内外共用此入口，默认生成 `mapping_YYYYMMDD_HHMMSS` 编号。
原室内、室外两个入口已删除，不保留转发；共享采集和 Jetson 后端仍是必要依赖。
支持 start、camera2-start、video、status、stop、abort、install、verify。

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

入口：`navigation_start.sh`。通过显式 MAP_ID 同时支持室内和室外地图；默认地图及
视觉库选择行为保持原样，室外运行务必显式指定地图和视觉库。

其直接依赖是：

- `leg_odom_tf_bridge.py`
- `goal_pose_bridge.py`
- `navigation_result_notifier.py`
- `rviz_pointcloud_relay.py`

当前室外导航命令：

```bash
LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./navigation_start.sh start outdoor_20260921_003_area_review
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
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_patrol_turning.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_patrol_route.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_patrol_route_prompt.py
```

机制核对来源：[Foxy 速度采样器](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_plugins/src/xy_theta_iterator.cpp)、
[轨迹采样](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp)、
[栅格评分](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_critics/src/map_grid.cpp)。
原厂分支的实际表现仍须在 Jetson 验证。

## 6. 终点转向修正与共享采集后端

### 2026-10-12：第 11 点终点转向修正（待实机验证）

15:57 巡航前 10 点成功；第 11 点位置曾进入 0.35 m 容差，但最终朝向未达
0 rad。Nav2 反复报 `No valid trajectories out of 336`，约 85 秒后中止，
不是执行器 120 秒超时，也不是转向无进展保护触发。

当前室外配置与配置生成器已同步修改：

- `FollowPath.xy_goal_tolerance=0.20`：靠近到 0.20 m 后进入最终转向，
  相对于验收位置容差预留 0.15 m 余量。这不是消除真实位移/定位漂移的保证。
- `goal_checker.stateful=false`：位置与朝向必须同时满足，验收仍为 0.35 m / 0.20 rad。
- `GoalAlign.scale=0`：取消机头朝向目标位置的偏好，避免与最终指定朝向竞争。
  这是整个室外控制器的评分配置变更，需同时回归中间点；不是仅对 wp11 动态切换。
  保留 PathAlign、PathDist、GoalDist、RotateToGoal、Oscillation 和 ObstacleFootprint。
- `debug_trajectory_details=true`：下次无有效轨迹时保留评分项拒绝原因。
- 预检查允许转向半径小于验收半径；旧的严格相等检查已同步修正。

第 11 点的位置与 0° 朝向、限速、地图、原厂运动链路均未改动。
参数修正针对已确认的评分机制和日志现象；历史数据不足以确定每次候选轨迹
被拒绝的具体评分项，不能将离线参数测试当作实机恢复成功。
必须在旧任务停止、DISARMED 后退出旧导航会话并重新 start；只重启巡航执行器
不会更新运行中的 Nav2 参数。需要重测第 11 点及完整路线。

机制来源：[Foxy DWB setPlan 会重置评分器状态](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)、
[GoalAlign 的机头目标位置评分](https://github.com/ros-navigation/navigation2/blob/foxy-devel/nav2_dwb_controller/dwb_critics/src/goal_align.cpp)。

以下文件由统一建图入口间接调用，不是冗余文件：

- `mapping_capture.sh`
- `round3a_capture_backend.sh`
- `camera_preview.py`
- `mid360_isolated.launch.py`
- `time_probe.py`
