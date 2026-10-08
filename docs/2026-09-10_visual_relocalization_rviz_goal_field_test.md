# Lite3 视觉初始重定位与 RViz 随机目标点导航现场流程

适用日期：2026-09-10 之后  
候选地图 ID：`site_fusion_20260909_001_area_v4_review`  
Jetson：`ysc@192.168.1.103`  
项目：`/home/ysc/lite3_patrol_project`

本文的目标是：机器狗在建图现场开机后，先由相机给出全局初值，再由 LiDAR
NDT 确认和持续定位；随后在 RViz 地图的白色自由区内点击一个目标点，让 Nav2
规划并在显式安全门禁允许后低速行走。

> 首次测试不要把“随机目标”理解成任意栅格。只允许点击白色自由区内部、距
> 黑色障碍和灰色未知区至少约 0.5 米的位置。第一次真机目标距当前位置只取
> 1 至 2 米。现场必须清空行人和车辆，并由一人持续持有官方遥控器和急停。

## 1. 系统结构和安全边界

本流程中的速度链路为：

```text
RViz Goal
  -> Nav2
  -> /patrol/cmd_vel_raw
  -> velocity_gate（默认 DISARMED）
  -> /cmd_vel_corrected
  -> 原厂 /motion_sender
  -> Lite3 底盘
```

必须始终满足：

- `/cmd_vel` 的发布者数量为 0，避免绕过安全门禁；
- `/cmd_vel_corrected` 只有 `velocity_gate` 发布、`motion_sender` 订阅；
- `velocity_gate` 只有收到新鲜的 LiDAR、里程计、Nav2 命令和
  `/patrol/localization_ready=true` 才允许 ARM；
- 门禁最大速度限制为前后 `0.15 m/s`、横向 `0.03 m/s`、旋转 `0.30 rad/s`；
- `mission_draft.yaml` 不参与单点 RViz 测试，继续保持禁用。

## 2. 到场摆放和人工准备

1. 将机器狗放在 `site_fusion_20260909_001` 采集时相机实际看过的路段附近。
   不要求回到建图原点，但相机必须能看到视觉数据库中存在的场景和方向。
2. 机器狗保持静止站立，周围至少留出 2 米安全空间。
3. 使用官方遥控器进入厂商允许的行走状态；现场人员全程握住遥控器，确保可以
   立即急停。不要使用未知的终端姿态或运动命令。
4. 首次测试只使用地面平整、纹理清楚且 LiDAR 无遮挡的位置。

## 3. 终端 0：连接 Jetson 并执行总预检

在本地电脑打开终端：

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
```

在 Jetson 中执行：

```bash
export PROJECT=/home/ysc/lite3_patrol_project
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
export MAP_DIR="$PROJECT/navigation/$LITE3_PATROL_MAP_ID"

source "$PROJECT/navigation/patrol_env.sh"

test -r "$MAP_DIR/patrol_map.yaml"
test -r "$MAP_DIR/patrol_map.pgm"
test -r "$MAP_DIR/localization_map.pcd"
test -r "$MAP_DIR/nav2_safe.yaml"
test -r "$MAP_DIR/visual_initialization.yaml"
test -r "$PROJECT/results/visual_maps/$LITE3_PATROL_MAP_ID/visual_map_dense.db"

echo "MAP_DIR=$MAP_DIR"
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_corrected
```

开始前应看到：

```text
/cmd_vel              Publisher count: 0
/cmd_vel_corrected    Publisher count: 0
/cmd_vel_corrected    Subscription count: 1
```

该订阅者应为原厂 `/motion_sender`。如果 `/cmd_vel` 或
`/cmd_vel_corrected` 已有未知发布者，停止流程并找出该节点：

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_corrected -v
```

### 3.1 相机和 IMU 检查

```bash
systemctl status realsense_ros2.service --no-pager
timeout 15s ros2 topic hz /camera/color/image_raw
timeout 15s ros2 topic hz /camera/depth/image_rect_raw
timeout 15s ros2 topic hz /imu/data
```

彩色、深度应持续接近 30 Hz；IMU 必须连续有数据。若相机无输出：

```bash
sudo systemctl restart realsense_ros2.service
sleep 8
timeout 30s ros2 topic hz /camera/color/image_raw
timeout 30s ros2 topic hz /camera/depth/image_rect_raw
```

相机仍超时则停止当天自动导航测试。

## 4. 终端 1：启动 LiDAR、IMU 和 NDT 定位

重新打开一个 SSH 终端：

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh
```

该命令持续占用终端，不要关闭。它会启动 Mid360 驱动并检查
`/rslidar_points`、`/imu/data`，然后启动 `hdl_localization`。此时没有 Nav2，
也不会发布底盘速度。

在终端 0 验证：

```bash
timeout 15s ros2 topic hz /rslidar_points
timeout 15s ros2 topic hz /odom
ros2 node list | grep patrol_localization
```

此时 `/odom` 的初值尚未经过视觉确认，不得据此启动导航。

## 5. 终端 2：启动视觉全局定位

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_visual_localizer.sh
```

该候选地图复用原 `site_fusion_20260909_001` 的视觉数据库。节点只发布视觉
位姿，不发布速度。

终端 0 中观察视觉状态：

```bash
ros2 topic echo /visual_localization/state std_msgs/msg/String
```

有效状态示例：

```text
POSE:kf=123,inliers=48,x=12.34,y=1.56,yaw=0.15
```

至少连续观察 5 至 10 个 `POSE`，要求：

- `x`、`y` 和 `yaw` 稳定；
- 位置与现场在地图上的大致位置一致；
- 朝向与机器狗实际朝向一致；
- `inliers` 不低于节点门限，且不能在相距很远的位置间跳变。

以下状态不能进入下一步：

- `REJECTED:no_pnp_solution`：当前画面不能可靠匹配；
- `REJECTED:ambiguous_...`：场景重复，候选位置存在歧义；
- `ERROR:image_timeout`：相机停止出帧。

如果持续 REJECTED，使用官方遥控器人工移动到原相机采集路线，停稳后再观察。
不要在视觉尚未成功时用 Nav2 自动寻找位置。

## 6. 终端 3：启动视觉初值守卫并临时解锁

启动守卫节点：

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_visual_initialization.sh
```

首次启动出现下面信息是正常的：

```text
BLOCKED:map_transform_not_configured
```

这是防止未经现场核对就注入视觉初值。只有完成第 5 节人工核对后，才在终端 0
动态设置本次进程的 `configured` 参数；该操作不会永久修改 YAML：

```bash
ros2 param set /visual_pose_initializer configured true
ros2 param get /visual_pose_initializer configured
```

若第一次因 ROS 发现延迟返回 `Node not found`，等待 2 秒后原样重试一次。已经在
独立 ROS Domain 中实测该参数可由 `False` 动态设为 `True`。必须看到
`Set parameter successful` 和 `Boolean value is: True`，随后才开始一次受守卫的
视觉初始化：

```bash
ros2 service call /patrol/request_visual_initialization std_srvs/srv/Trigger '{}'
ros2 topic echo /patrol/visual_init_state std_msgs/msg/String
```

状态应依次经历：

```text
COLLECTING:.../10
WAITING_NDT:x=...,y=...,yaw=...
READY:visual_plus_ndt_confirmed_...
```

出现 READY 后按 `Ctrl+C` 结束上面的持续 echo，然后检查：

```bash
ros2 topic echo --once /patrol/localization_ready std_msgs/msg/Bool
timeout 10s ros2 run tf2_ros tf2_echo map base_link
```

要求：

- `localization_ready` 为 `data: true`；
- `map -> base_link` 连续输出且不跳变；
- 坐标和朝向与视觉状态、现场实际位置一致。

如果出现 `FAILED:ndt_confirmation_timeout` 或 NDT 差值超限，执行复位后回到第 5
节重新检查，不能强行进入导航：

```bash
ros2 service call /patrol/reset_visual_initialization std_srvs/srv/Trigger '{}'
```

## 7. 终端 4：启动隔离 Nav2

只有第 6 节全部通过才执行：

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_nav_isolated.sh
```

Nav2 的速度被重映射到 `/patrol/cmd_vel_raw`。当前尚未启动 `velocity_gate`，
所以命令无法到达 `/cmd_vel_corrected`，机器狗不会因为 RViz 点击而移动。

终端 0 检查：

```bash
ros2 action list | grep navigate_to_pose
ros2 topic info /map
ros2 topic info /plan
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_corrected
ros2 node list | grep -E 'map_server|planner_server|controller_server|bt_navigator'
```

必须继续满足：

```text
/cmd_vel Publisher count: 0
/cmd_vel_corrected Publisher count: 0
```

## 8. 终端 5（本地电脑）：通过 X11 打开 Jetson 上的 RViz

本地电脑目前无需安装 ROS/RViz；使用 Jetson 已安装的 RViz2。先确认本地 Ubuntu
桌面终端中 `echo $DISPLAY` 非空，然后执行：

```bash
ssh -Y -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
rviz2 -d /home/ysc/lite_cog_ros2/nav/src/dr_nav2/rviz/dr_nav2.rviz
```

若出现 OpenGL 渲染错误，改用软件渲染：

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d /home/ysc/lite_cog_ros2/nav/src/dr_nav2/rviz/dr_nav2.rviz
```

RViz 中确认：

1. `Global Options -> Fixed Frame` 为 `map`；
2. `Map` 话题为 `/map`；
3. `Global Costmap` 为 `/global_costmap/costmap`；
4. `Path` 为 `/plan`；
5. `Local Costmap` 为 `/local_costmap/costmap`；
6. `Local Plan` 为 `/local_plan`；
7. `PointCloud2` 为 `/rslidar_points`；
8. 右侧存在 `Navigation 2` 面板。

地图颜色语义：

- 白色：可规划自由区；
- 黑色：静态障碍；
- 灰色：未知区，禁止规划；
- 彩色局部代价：实时 LiDAR 障碍层。

机器狗模型或 TF 坐标必须落在与现场相符的白色区域。如果位置明显错误，关闭
Nav2，回到视觉/NDT 初始化，不要用 RViz 的目标点“试试看”。

## 9. 第一次点击：只检查路径，禁止真机运动

确认 `velocity_gate` 尚未启动：

```bash
ros2 node list | grep patrol_velocity_gate || echo GATE_NOT_RUNNING
ros2 topic info /cmd_vel_corrected
```

在 RViz 使用 `Nav2 Goal` 或 `2D Goal Pose`：

1. 点击机器人附近 1 至 2 米处的白色区域中心；
2. 拖动箭头指定目标朝向；
3. 观察 `/plan` 全局路径；
4. 检查整条路径不进入黑色或灰色区域，并与边界保持足迹净空；
5. 检查局部代价地图能显示现场临时障碍物；
6. 在 `Navigation 2` 面板点击 `Cancel` 取消该目标。

此阶段即使 `/patrol/cmd_vel_raw` 出现非零命令，也没有发布者向
`/cmd_vel_corrected` 转发，机器狗不应运动。如果机器狗发生任何动作，立即使用
官方遥控器急停，并停止全部测试。

取消后确认：

```bash
ros2 topic info /cmd_vel_corrected
```

其发布者仍必须为 0。

## 10. 终端 6：启动保持 DISARMED 的速度门禁

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
bash /home/ysc/lite3_patrol_project/navigation/start_gate_disarmed.sh
```

启动日志必须出现：

```text
velocity gate started DISARMED
zero-command heartbeat active on /cmd_vel_corrected
```

终端 0 检查门禁和控制链：

```bash
ros2 topic echo --once /patrol/armed std_msgs/msg/Bool
ros2 topic echo --once /patrol/gate_state std_msgs/msg/String
ros2 topic info /cmd_vel -v
ros2 topic info /cmd_vel_corrected -v
```

期望：

- `/patrol/armed` 为 `false`；
- `/patrol/gate_state` 为 `DISARMED:...`；
- `/cmd_vel` 仍为 0 个发布者；
- `/cmd_vel_corrected` 为 1 个发布者（`patrol_velocity_gate`）和 1 个订阅者
  （`motion_sender`）。

可抽查门禁输出为零：

```bash
ros2 topic echo --once /cmd_vel_corrected geometry_msgs/msg/Twist
```

## 11. 第二次点击并显式 ARM：第一次真机 A 到 B

1. 现场人员握住官方遥控器并口头确认可以立即急停。
2. RViz 再次点击第 9 节已经检查过的同一个 1 至 2 米目标。
3. 确认全局路径与第一次一致。
4. 在终端 0 观察 Nav2 正在产生原始命令：

```bash
ros2 topic echo --once /patrol/cmd_vel_raw geometry_msgs/msg/Twist
```

5. 只有以上检查全部通过，才执行唯一的 ARM 命令：

```bash
ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: true}'
```

必须返回：

```text
success: true
message: ARMED
```

从此刻起机器狗会低速运动。现场同时观察：

```bash
ros2 topic echo /patrol/gate_state std_msgs/msg/String
```

正常状态为 `ARMED`。以下任一情况立即急停并 DISARM：

- 定位点云和地图出现明显重影或跳变；
- 机器狗方向与 RViz 路径不一致；
- 局部障碍未进入代价地图；
- 人、车辆或动物进入测试区；
- 相机、LiDAR、`/odom` 或网络异常；
- 机器狗偏离白色自由区。

到达目标且 Nav2 停止发送命令后，门禁会因命令超时自动撤销 ARM。仍应人工执行
一次 DISARM：

```bash
ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: false}'
ros2 topic echo --once /patrol/gate_state std_msgs/msg/String
```

## 12. 第二个及后续随机点

首次 1 至 2 米测试成功后，再逐步增加距离。每个新目标都先执行“隔离规划检查”，
不能直接点击远处后立即 ARM。

如果上一个目标结束后门禁显示 `DISARMED:nav_command_stale` 等锁存故障，先清除
故障；清除后仍保持 DISARMED：

```bash
ros2 service call /patrol/clear_fault std_srvs/srv/Trigger '{}'
ros2 topic echo --once /patrol/gate_state std_msgs/msg/String
```

然后按以下顺序重复：

1. 在 RViz 点击新的白色内部目标；
2. 查看 `/plan`，确认不穿越黑色或灰色；
3. 确认实时局部障碍层正常；
4. 确认 `/patrol/localization_ready=true`；
5. 执行 `/patrol/set_armed {data: true}`；
6. 到达后执行 `/patrol/set_armed {data: false}`。

视觉地图关键帧主要位于原采集路线。机器狗进入扩展自由区后，持续定位主要依赖
LiDAR NDT；如果中途完全丢失定位，不保证能在扩展区仅靠相机重新找回，应先急停，
再由人工遥控回到视觉覆盖路线。

## 13. 紧急停止命令

最高优先级始终是现场人员使用官方遥控器急停。终端 0 同时准备好以下命令：

```bash
ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: false}'
```

然后在 RViz `Navigation 2` 面板点击 `Cancel`。不要先关闭门禁进程再 DISARM；
保持门禁运行可以继续向 `/cmd_vel_corrected` 输出零速度心跳。

如果服务无响应，先用官方遥控器急停，再在终端 6 对门禁按 `Ctrl+C`。门禁退出
前会额外发布一次零速度，但网络或进程故障时不能用它代替物理急停。

## 14. 正常结束全部测试

严格按顺序执行：

1. 终端 0 DISARM：

   ```bash
   ros2 service call /patrol/set_armed std_srvs/srv/SetBool '{data: false}'
   ros2 topic echo --once /patrol/armed std_msgs/msg/Bool
   ```

2. RViz `Navigation 2` 面板点击 `Cancel`。
3. 终端 6 按 `Ctrl+C`，停止速度门禁。
4. 终端 4 按 `Ctrl+C`，停止 Nav2。
5. 终端 3 按 `Ctrl+C`，停止视觉初始化守卫；它会发布
   `localization_ready=false`。
6. 终端 2 按 `Ctrl+C`，停止视觉定位。
7. 终端 1 按 `Ctrl+C`，停止 NDT 和本流程启动的 Mid360 驱动。
8. 关闭 RViz。

最终检查控制发布者：

```bash
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_corrected
```

两者的发布者数量都应为 0。

## 15. 常见故障快速处理

### 视觉始终 REJECTED

- 停止自动流程；
- 确认镜头无遮挡、光照接近采集时；
- 使用官方遥控器人工回到原相机路线并停稳；
- 改变朝向，使相机看到采集时的方向；
- 不要手工猜测 `/initialpose`。

### 请求初始化后 NDT 超时

```bash
ros2 service call /patrol/reset_visual_initialization std_srvs/srv/Trigger '{}'
```

检查视觉坐标、LiDAR 点云、外参和地图重影。未找到原因前不要重新 ARM。

### Nav2 拒绝目标

- 目标可能在黑色障碍或灰色未知区；
- 目标周围没有足够的足迹净空；
- 当前位姿和目标不在同一白色连通区域；
- 将目标移到更靠近白色区域中心的位置，不要开启 `allow_unknown`。

### 门禁无法 ARM

查看：

```bash
ros2 topic echo --once /patrol/gate_state std_msgs/msg/String
ros2 topic echo --once /patrol/localization_ready std_msgs/msg/Bool
timeout 5s ros2 topic hz /odom
timeout 5s ros2 topic hz /rslidar_points
```

门禁还要求 Nav2 正在持续发布 `/patrol/cmd_vel_raw`，所以应先发送已经检查过的
目标，再执行 ARM。不要通过修改门禁参数绕过检查。

## 16. 本次使用的文件

```text
/home/ysc/lite3_patrol_project/navigation/
  site_fusion_20260909_001_area_v4_review/
    patrol_map.yaml
    patrol_map.pgm
    localization_map.pcd
    nav2_safe.yaml
    visual_initialization.yaml
    navigation_area_comparison.png
    navigation_area_validation.json

/home/ysc/lite3_patrol_project/results/visual_maps/
  site_fusion_20260909_001_area_v4_review/
    visual_map_dense.db  -> 复用原视觉数据库
```

该候选地图已通过离线地图和足迹检查，但本文首次真机流程完成前，不能视为已经
完成自动避障和全区域导航验收。
