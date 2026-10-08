# Lite3 当前 ROS 架构与功能边界

本文件描述 2026-09-01 审计时的真实运行状态，以及磁盘中存在但未运行的厂商基线。两者不得混写。

## 1. 当前运行架构

```text
Intel RealSense D435i
  └─ realsense2_camera /camera/camera
       ├─ /camera/color/image_raw       Image 424x240@~30
       ├─ /camera/color/camera_info     CameraInfo
       ├─ /camera/depth/image_rect_raw  Image 424x240@~30
       └─ camera internal TF only

Lite3 Motion Host .120 / proprietary jy_exe
  └─ UDP state -> Jetson .103:43897
       └─ transfer /motion_receiver
            ├─ /imu/data       Imu @~200
            ├─ /leg_odom       PoseWithCovarianceStamped @~200
            ├─ /leg_odom2      Odometry @~200
            ├─ /joint_states
            ├─ /handle_state
            └─ /us_publisher/ultrasound_distance

ROS command interfaces (currently publisher count 0)
  ├─ /cmd_vel
  ├─ /cmd_vel_corrected
  ├─ /simple_cmd
  └─ /complex_cmd
       └─ transfer /motion_sender
            └─ UDP command -> Motion Host .120:43893 -> Lite3 Motion System

Livox Mid-360 .201
  ├─ raw point UDP -> Jetson .103:56301  [data present]
  └─ raw IMU UDP   -> Jetson .103:56401  [data present]
       └─ ROS driver [NOT RUNNING]
            ├─ /rslidar_points [0 publisher]
            └─ /livox/imu      [0 publisher/currently absent]

SLAM [NOT RUNNING]
Localization [NOT RUNNING]
Navigation [NOT RUNNING]
VOA [NOT RUNNING]
```

当前 ROS graph 只有 5 个节点，无 action server。Topic 名存在不代表数据存在，尤其 `/rslidar_points` 只是 `/sensor_checker` 创建了订阅端。

## 2. 当前节点与接口

| Node | Package/Executable | 关键输入 | 关键输出 | 当前状态 |
|---|---|---|---|---|
| `/camera/camera` | `realsense2_camera` | USB D435i | RGB/Depth/CameraInfo/内部 TF | RUNNING |
| `/motion_receiver` | `transfer` / Jetson2Motion | UDP `.120→.103:43897` | IMU、leg odom、joint、ultrasound | RUNNING |
| `/motion_sender` | `transfer` / Jetson2Motion | cmd_vel/simple/complex | UDP `.103→.120:43893` | RUNNING；本轮未触发 |
| `/sensor_checker` | `transfer` | sensor topics | `/sensor_status/*` | RUNNING；lidar=false |
| `/app_receiver` | `transfer` | app UDP/接口 | app-related | RUNNING，非本轮重点 |

## 3. 厂商 Mapping 基线（磁盘存在，当前未运行）

### 3.1 算法身份

**CONFIRMED：Faster-LIO**，不是 FAST-LIO、FAST-LIO2 或 FAST-LIVO2。

证据：

- package：`faster_lio`，描述明确写明 “FasterLIO is developed from FastLIO2”。
- Git remote：`git@github.com:shitoujie/faster-lio.git`，commit `bf235f4`。
- node executable：`run_mapping_online`，launch node name `faster_lio_online`。
- 源码采用 Faster-LIO 的 incremental voxel (`ivox`) 与 IEKF。

### 3.2 配置链

```text
/rslidar_points  sensor_msgs/PointCloud2
       +
/imu/data        sensor_msgs/Imu（Lite3 Motion IMU）
       ↓
faster_lio / faster_lio_online
       ├─ /Odometry                      nav_msgs/Odometry
       ├─ /cloud_registered              PointCloud2
       ├─ /cloud_registered_body         PointCloud2
       ├─ /cloud_registered_effect_world PointCloud2
       ├─ /path                          nav_msgs/Path
       ├─ TF camera_init -> body
       └─ /home/ysc/lite_cog_ros2/system/map/lite3.pcd
              ↓
          pcd2grid /projected_map
              ↓
          nav2 map_saver
              ├─ lite3.pgm
              └─ lite3.yaml
```

关键文件：

- Launch：`/home/ysc/lite_cog_ros2/slam/src/faster_lio/launch/mapping_c16.launch.py`
- Config：`/home/ysc/lite_cog_ros2/slam/src/faster_lio/config/c16.yaml`
- Source：`.../src/laser_mapping.cc`、`.../src/pointcloud_preprocess.cc`
- Scripts：`/home/ysc/lite_cog_ros2/system/scripts/slam/`

### 3.3 实机适配冲突

| 项目 | 实机/driver | Faster-LIO 当前配置 | 判断 |
|---|---|---|---|
| LiDAR | Livox Mid-360 | `lidar_type: 4` = Leishen C16 | CONFLICT |
| LiDAR msg | PointCloud2，含 tag/line/timestamp | C16 handler 只复制 xyz/intensity | CONFLICT |
| 逐点时间 | driver 提供 `timestamp` field | C16 handler 未给 `curvature` 赋逐点时间 | FAIL for proper deskew |
| Livox CustomMsg | driver可输出 | Faster-LIO AVIA 订阅代码被注释 | NOT AVAILABLE |
| IMU | 当前 `/imu/data` 来自 Motion Host | 配置默认使用它 | CONFIRMED but not synchronized |
| time sync | 无统一时基 | `time_sync_en: true` 软件差值模式 | 未验证，不能等价于真实同步 |
| LiDAR-IMU extrinsic | 未确认 Mid-360↔Motion IMU标定 | 使用 C16 的 T/R，且 online estimate=true | CONFLICT |

因此：厂商工作区表明“设计基线是 Faster-LIO”，但当前实机配置不能视为正确的 Mid-360 LIO。过去短时能输出 odometry 只证明程序可消费点云，不证明去畸变、外参和时间轴正确。

## 4. Localization（磁盘存在，当前未运行）

### 4.1 算法与数据链

```text
lite3.pcd -> globalmap_server -> /globalmap
/rslidar_points ------------------------------┐
/imu/data ------------------------------------┼─> hdl_localization (UKF + NDT_OMP)
/leg_odom2 (twist；robot odom prediction关闭) ┘
                                              ├─ /odom (map -> base_link pose)
                                              ├─ /aligned_points
                                              ├─ /status
                                              └─ TF map -> base_link
```

配置/源码：

- Launch：`/home/ysc/lite_cog_ros2/nav/src/hdl_localization/launch/lite_localization.launch.py`
- Source：`.../apps/hdl_localization_app.hpp`
- Map：`/home/ysc/lite_cog_ros2/system/map/lite3.pcd`
- Registration：`NDT_OMP`，resolution 1.5 m，DIRECT1。

### 4.2 重启后的定位方式

- 当前配置 `specify_init_pose=True`，每次默认 `(0,0,0,qw=1)` 初始化。
- 源码支持 `/initialpose`（RViz 2D Pose Estimate）重新初始化。
- `use_global_localization=False`，所以 `/relocalize` 服务和 hdl_global_localization 不会启用。
- 没有发现持久保存/自动恢复上次位姿的逻辑。
- 当前 `system/map` 目录为空，没有 `lite3.pcd`、`lite3.yaml`、`lite3.pgm`。

结论：**独立 Localization 代码已存在，但当前不可运行；机器人重启后的自动重定位未实现。** 正式使用需有地图并人工/外部提供可信初始位姿，不能假设会自动找回地图坐标。

## 5. Navigation Stack（磁盘存在，当前未运行）

### 5.1 身份

**CONFIRMED：ROS 2 Nav2 Foxy**，不是当前 ROS1 move_base，也不是完全自研导航。

厂商封装 `dr_nav2`，通过 `nav2_bringup/bringup_launch.py` 启动：

- map_server
- planner_server：`NavfnPlanner`
- controller_server：`DWBLocalPlanner`
- bt_navigator：`navigate_w_replanning_and_recovery.xml`
- recoveries：spin/back_up/wait
- local/global costmap

### 5.2 计划的数据链

```text
lite3.yaml/pgm -> Nav2 map_server -> /map
hdl_localization -> /odom + TF map->base_link
/rslidar_points -> STVL in local/global costmap
Goal -> bt_navigator -> NavFn global path -> DWB controller
     -> /cmd_vel -> /motion_sender -> UDP -> Lite3 Motion
```

动态障碍来源：**LiDAR `/rslidar_points` PointCloud2**。local/global costmap 都配置 `spatio_temporal_voxel_layer`，高度 0.15–1.20 m，obstacle range 4 m；没有把 depth camera 配进 Nav2 costmap。

### 5.3 Nav2 配置风险

- `controller_server.use_sim_time=True`，而其他组件为 `False`：**CONFLICT**，实机时钟/超时可能异常。
- Nav2 和 hdl_localization 需要 `base_link↔rslidar`；Nav2 launch 中 static TF 被注释。
- Livox launch 发布的是零外参；另一些旧配置为 C16 的 `(0.16,0,0.47,yaw=1.57)`：**CONFLICT**。
- 地图文件不存在。
- 当前 `/odom`、LiDAR和完整 TF 都不存在。

因此 Nav2 是“安装和配置过”，不是“当前已具备可验证自主导航”。

## 6. 独立 VOA 安全避障（磁盘存在，当前未运行）

厂商还有一个不同于 Nav2 costmap 的 `voa`：

```text
/camera/depth/color/points
  -> PointCloudProcessor
  -> /camera/depth/color/transformed_points
  -> GridMapper -> /grid_map
                    + /ultrasound + /leg_odom2 + /cmd_vel
                    -> SafetyController -> /cmd_vel_corrected
```

状态与风险：

- 当前 RealSense launch 设置 `enable_pointcloud=false`，所以输入点云不存在。
- VOA 没有运行。
- VOA launch 中存在一个 base→camera 候选静态变换 `(0.25489,0,0.07249,pitch=0.34907)`，但未发现校准报告，不能当作确认标定。
- `/motion_sender` 同时订阅 `/cmd_vel` 与 `/cmd_vel_corrected`，两者走同一个发送回调。若原始与修正命令同时存在，原始命令可能绕过安全修正或与其竞争：**CONFLICT / safety review required**。

结论：VOA 是代码级候选能力，不是当前生效的安全层。本轮禁止启动验证。

## 7. TF 与标定现状

### 7.1 当前运行 TF

```text
camera_link
  ├─ camera_color_frame -> camera_color_optical_frame
  └─ camera_depth_frame -> camera_depth_optical_frame

[断开] base_link
[缺失] rslidar / imu / odom / map
```

`base_link -> camera_link` 查询失败；`motion_receiver` 虽创建 `/tf` publisher，但相应 broadcaster 代码未形成可用的 base tree。

### 7.2 标定状态

| 标定 | 状态 | 证据/问题 |
|---|---|---|
| Camera intrinsics | FOUND | 当前 CameraInfo 有效，424×240 |
| Camera depth↔color factory extrinsic | FOUND | RealSense internal topic/TF |
| base↔camera | PARTIAL | VOA launch 有候选值，无校准报告，当前未发布 |
| base↔LiDAR | CONFLICT | zero、C16 旧值、MID360 JSON 非零值、ROS1 注释候选互相冲突 |
| LiDAR↔Motion IMU | CONFLICT | `c16.yaml` 有 C16 T/R，不适用于已确认的 Mid-360组合 |
| LiDAR↔Mid-360 internal IMU | UNKNOWN | 没有明确外参文件；可能是设备内部固定关系，但未读取厂商标定证据 |
| LiDAR↔Camera | NOT FOUND | 无直接标定文件/报告 |

FAST-LIVO2 所需的 `Rcl/Pcl` 不能通过拼接这些冲突候选来“推算”。必须进行针对当前安装姿态的联合标定。

## 8. 多点巡航能力

| 能力 | 当前判断 | 依据 |
|---|---|---|
| 单点导航 | CODE PRESENT / NOT VERIFIED | Nav2 bt_navigator 与 NavigateToPose 基础组件存在，当前无运行证据 |
| 多点导航 | BASE EXECUTABLE PRESENT / UNCONFIGURED | `nav2_waypoint_follower` executable 已安装，但厂商 config 无 waypoint_follower 段 |
| 循环巡航 | NOT FOUND | 未找到 Lite3 patrol/loop task code |
| 任务取消 | Nav2基础能力存在 / NOT VERIFIED | 无厂商任务层与实机验证 |
| 失败处理 | 仅单目标 BT recovery | 未找到多点 skip/retry/abort policy |
| 巡检任务持久化 | NOT FOUND | 无 waypoint DB/mission format |

结论：不能把“安装了 waypoint_follower”写成“已经支持校园循环巡检”。任务编排、循环、取消、失败策略、续航/回充和告警均尚未证实。

## 9. 当前功能成熟度回答

| 功能 | 当前运行 | 静态能力 | 审计结论 |
|---|---|---|---|
| SLAM | 无 | Faster-LIO 存在但 Mid-360适配冲突 | NOT OPERATIONAL |
| Localization | 无 | hdl_localization + NDT_OMP 存在 | BLOCKED by map/TF/LiDAR/time |
| Navigation | 无 | Nav2 Foxy/NavFn/DWB/STVL 存在 | BLOCKED by localization/map/TF/LiDAR/config conflict |
| Dynamic avoidance | 无 | Nav2 LiDAR STVL + 独立 D435i/ultrasound VOA | NOT OPERATIONAL；命令链需安全审查 |
| Multi-waypoint patrol | 无 | waypoint follower binary only | NOT IMPLEMENTED as project capability |
