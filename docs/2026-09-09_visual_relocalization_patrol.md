# 2026-09-09 视觉初始重定位与多点巡航

适用地图：`site_fusion_20260909_001_map_05x`。运行目录使用短名称
`site_fusion_20260909_001`。

当前已经完成：

- 10 cm 雷达定位点云和二维占据栅格已部署到 Jetson。
- RGB-D 视觉地图包含 199 个关键帧，数据库完整性检查通过。
- 离线留出帧测试 12 个位置中有 8 个产生位姿，8 个均通过严格误差门限；其余 4 个被拒绝，没有输出错误位姿。
- 视觉地图位姿直接由本次 FAST-LIVO2 轨迹生成，因此 `visual_map` 与本次 LiDAR 地图在构建定义上是同一坐标系，平面变换为零。
- 43 航点任务 dry-run 已通过，但任务仍为 `enabled: false`。

## 0. 现场安全条件

首次测试只允许机器狗站立静止。确保有人持有官方遥控器并能立即急停；测试区清空行人、车辆和活动障碍物。完成视觉与 NDT 双重确认之前，不启动 Nav2、不启动速度门禁、不发送航点。

## 1. 恢复并验证 RealSense

当前相机曾出现 UVC watchdog 后停止出帧。若彩色或深度无帧，在 Jetson 执行：

```bash
sudo systemctl restart realsense_ros2.service
```

然后验证两个数据流至少持续 60 秒：

```bash
source /opt/ros/foxy/setup.bash
source /home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash

timeout 60s ros2 topic hz /camera/color/image_raw
timeout 60s ros2 topic hz /camera/depth/image_rect_raw
```

两项都应持续接近 30 Hz，中间不能变成无输出。若再次卡死，应先处理 USB/驱动稳定性，不能进入自动巡航。

## 2. 启动 LiDAR NDT 定位（终端 1）

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001
bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh
```

脚本会先检查 `/rslidar_points` 和 `/imu/data` 是否有实时数据，再启动本地图的 `hdl_localization`。该脚本不启动 Nav2，也不发布速度。

## 3. 启动视觉全局定位（终端 2）

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001
bash /home/ysc/lite3_patrol_project/navigation/start_visual_localizer.sh
```

另开终端观察结果：

```bash
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
ros2 topic echo /visual_localization/state std_msgs/msg/String
```

有效结果形如：

```text
POSE:kf=123,inliers=48,x=12.34,y=1.56,yaw=0.15
```

`REJECTED:no_pnp_solution` 表示当前位置画面无法可靠匹配，节点会拒绝输出位姿；`ERROR:image_timeout` 表示相机再次停止出帧。二者都不能进入下一步。机器狗可由现场人员原地缓慢改变朝向，让摄像头看到建图时相同方向的稳定纹理，再等待连续、稳定的 `POSE`。

## 4. 视觉初值注入与 NDT 二次确认（终端 3）

只有首次在线视觉位姿验证通过后，才把以下配置中的 `configured` 改为 `true`：

```text
/home/ysc/lite3_patrol_project/navigation/site_fusion_20260909_001/visual_initialization.yaml
```

启动守卫节点：

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001
bash /home/ysc/lite3_patrol_project/navigation/start_visual_initialization.sh
```

另开终端显式请求一次初始化：

```bash
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
ros2 service call /patrol/request_visual_initialization std_srvs/srv/Trigger '{}'
ros2 topic echo /patrol/visual_init_state std_msgs/msg/String
```

守卫节点会收集 10 个稳定视觉位姿，向 `/initialpose` 注入均值，再要求 NDT `/odom` 连续 5 次与视觉初值一致。只有出现下面状态才算定位完成：

```text
READY:visual_plus_ndt_confirmed_...
```

同时确认：

```bash
ros2 topic echo --once /patrol/localization_ready std_msgs/msg/Bool
ros2 run tf2_ros tf2_echo map base_link
```

期望分别看到 `data: true` 和连续的 `map -> base_link` 变换。机器狗不必放回建图原点，但当前位置必须属于已扫描区域，并且相机应能看到数据库中可区分的场景。

## 5. 启动隔离 Nav2（终端 4，仍不能驱动机器狗）

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001
bash /home/ysc/lite3_patrol_project/navigation/start_nav_isolated.sh
```

该脚本要求视觉加 NDT 已确认，并把 Nav2 的 `/cmd_vel` 重映射为 `/patrol/cmd_vel_raw`。此时真实控制话题尚未连接，机器狗不会移动。

检查地图、全局路径和局部障碍层后，先进行任务 dry-run：

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001
bash /home/ysc/lite3_patrol_project/navigation/mission_dry_run.sh
```

## 6. A 到 B 首次实测门槛

首次实测不要直接运行 43 点全任务。先从 `mission_draft.yaml` 复制一个只含相邻 A、B 两点的任务，检查两点位于白色可通行走廊中心、路径不穿过黑色障碍或灰色未知区，并保持：

```yaml
enabled: false
review_required: true
```

完成以下人工检查后才能改为 `enabled: true`、`review_required: false`：

1. 当前视觉位姿与现场位置方向一致。
2. NDT 点云与定位点云无明显重影，`localization_ready` 持续为真。
3. Nav2 全局路径完全位于已扫描的自由走廊。
4. `/rslidar_points`、`/odom` 持续更新，现场急停可用。
5. 真实底盘速度话题和方向已用零速度验证。

之后启动速度门禁，它默认保持 DISARMED 并持续发布零速度：

```bash
bash /home/ysc/lite3_patrol_project/navigation/start_gate_disarmed.sh
```

实际 ARM 和 `patrol_runner --execute` 必须在现场完成上述检查后显式执行。任何相机超时、NDT 跳变、雷达或里程计超时都会撤销就绪状态或触发门禁停机。

## 7. 当前文件

- LiDAR 定位点云：`navigation/site_fusion_20260909_001/localization_map.pcd`
- 二维地图：`navigation/site_fusion_20260909_001/patrol_map.yaml`
- Nav2 参数：`navigation/site_fusion_20260909_001/nav2_safe.yaml`
- 视觉数据库：`results/visual_maps/site_fusion_20260909_001/visual_map_dense.db`
- 视觉初始化：`navigation/site_fusion_20260909_001/visual_initialization.yaml`
- 全路线草稿：`navigation/site_fusion_20260909_001/mission_draft.yaml`
