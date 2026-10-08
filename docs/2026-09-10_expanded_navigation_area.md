# 2026-09-10 扩展导航区域 v4

## 结果与边界

候选地图 ID：`site_fusion_20260909_001_area_v4_review`。

该地图不是把整张点云投影后全部放行为可通行，而是只接受同时满足以下条件的栅格：

1. 原始 Mid360 射线重复观测为空闲；
2. 附近存在局部地面点支撑；
3. 地面坡度不超过 20 度且粗糙度通过；
4. 不属于重复观测到的障碍；
5. 与机器狗实际走过的路线处于同一连通区域。

白色为候选自由区，黑色为障碍，灰色仍为未知区。Nav2 配置为
`allow_unknown: false`，不会穿越灰色区域。

离线检查结果：

- 自由面积由保守走廊的 75.76 平方米扩大到 711.94 平方米；
- 0.30 米净空下，与原路线连通的安全面积为 488.33 平方米；
- 0.40 米净空下，与原路线连通的安全面积为 432.85 平方米；
- 原 43 个巡航点及其 0.8 x 0.6 米含 padding 足迹全部通过；
- Jetson Map Server 已成功读取 683 x 337、0.1 米/格地图。

详细数值见候选目录的 `navigation_area_validation.json`，新旧区域叠加见
`navigation_area_comparison.png`。

## 视觉定位覆盖范围

扩展的是 LiDAR 二维导航自由区，不是视觉关键帧覆盖范围。候选地图复用
`site_fusion_20260909_001` 的视觉数据库；因此机器狗开机后的视觉全局重定位，
仍应先放在此次采集时相机看过、纹理稳定的路段附近。完成视觉初值和 NDT
确认后，LiDAR NDT 可以在扩展自由区内继续跟踪定位。

如果要求机器狗在扩展区任意位置开机后都能仅靠相机自动找回初始位置，必须
补采扩展区的多方向 RGB-D 关键帧并重建视觉数据库；二维导航地图变宽本身不会
自动产生这些视觉关键帧。

## 现场只定位流程（不启动运动）

所有终端都使用同一地图 ID：

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
```

终端 1，启动 LiDAR/IMU 和 NDT：

```bash
bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh
```

终端 2，启动视觉定位：

```bash
bash /home/ysc/lite3_patrol_project/navigation/start_visual_localizer.sh
```

终端 3，在在线视觉位姿已人工核对后启动初始化守卫：

```bash
bash /home/ysc/lite3_patrol_project/navigation/start_visual_initialization.sh
```

当前候选目录的 `visual_initialization.yaml` 仍为 `configured: false`。在现场
首次视觉位姿与实际位置、方向核对通过前，不得改为 `true`。通过后请求初始化：

```bash
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
ros2 service call /patrol/request_visual_initialization std_srvs/srv/Trigger '{}'
ros2 topic echo --once /patrol/localization_ready std_msgs/msg/Bool
ros2 run tf2_ros tf2_echo map base_link
```

只有看到 `localization_ready` 为 `true`，且 `map -> base_link` 与现场一致，才进入
导航规划检查。

## 扩展区规划检查（仍不连接真实底盘）

终端 4：

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_nav_isolated.sh
```

该脚本将速度输出隔离在 `/patrol/cmd_vel_raw`，没有启动门禁时不会转发到真实
底盘。此时可在 RViz 中显示 `/map`，用 `2D Goal Pose` 选择白色区域内的 B 点，
检查全局路径必须全程位于白色区域并与黑色、灰色边界留出足迹净空。

首次现场测试只选距当前位置 1 至 2 米的 A、B 点。不要直接启用 43 点任务，也
不要把目标放在白色区域的锯齿边缘、狭窄缺口或只被一侧雷达看见的位置。

## 真机运动前的最后门槛

地图候选目录中的 `mission_draft.yaml` 保持 `enabled: false` 和
`review_required: true`。只有在现场完成以下检查后，才另行制作两点任务并启动
速度门禁：

1. 视觉初值、NDT 点云和实际朝向一致；
2. `/rslidar_points`、`/imu/data`、`/odom` 持续更新；
3. 局部代价地图能实时标出临时障碍物；
4. 全局路径不进入灰色未知区或黑色障碍区；
5. 官方遥控器和急停由现场人员持续掌握。

候选地图通过离线检查不等于已经完成真机避障验收。
