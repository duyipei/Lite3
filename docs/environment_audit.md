# Lite3 LiDAR 校园巡检：第一轮实机环境审计

审计日期：2026-09-01（Asia/Shanghai）  
审计性质：只读调查；未启动 LiDAR/SLAM/Localization/Navigation/VOA，未发送任何运动消息，未修改厂商配置。

## 1. 结论摘要

- **CONFIRMED** 当前 shell 位于开发电脑的 VMware Ubuntu 虚拟机，不是机器人机载平台。
- **CONFIRMED** 实机包含两级机载计算：`192.168.1.103` 的 Jetson Xavier NX 负责 ROS/感知，`192.168.1.120` 的 RK3588 运行厂商底层运动程序。
- **CONFIRMED** 当前在线 ROS 是 Jetson 上的 ROS 2 Foxy + CycloneDDS；ROS 1 Noetic 仅安装/留有旧工作区，没有作为当前运行时。
- **CONFIRMED** 当前相机与 Lite3 状态桥运行；LiDAR、SLAM、定位和导航节点均未运行。
- **CONFIRMED** LiDAR 原始 UDP 正在从 `.201` 单播到 Jetson `.103`，所以 Windows `.102` 与 DEV-VM `.104` 正常情况下看不到该单播数据。
- **CONFIRMED** 当前传感器时钟不可直接用于 FAST-LIVO2：Jetson 系统日期错误、LiDAR/内部 IMU 未外部同步，Camera、Lite3 IMU、LiDAR 使用的时间基准不统一。

状态含义：`CONFIRMED`=运行时或源码直接证实；`LIKELY`=多项证据一致但缺少设备级确认；`UNKNOWN`=证据不足；`CONFLICT`=现有证据相互冲突。

## 2. 真实 Host 与网络拓扑

```text
Windows Host（用户提供：192.168.1.102/24）
        │ VMware bridged Ethernet
        ▼
DEV-VM ubuntu（192.168.1.104/24，ens33）
        │ SSH / ROS 2 DDS，192.168.1.0/24
        ├──────────────────────────────────────────┐
        ▼                                          ▼
ROBOT-PERCEPTION lite（Jetson Xavier NX）           ROBOT-MOTION ysc（RK3588）
192.168.1.103/24                                   192.168.1.120/24
ROS 2 / Camera / sensor bridge                     jy_exe / CAN / motion system
        │                                          ▲
        │ UDP state :43897                         │ UDP command :43893
        └──────────────────────────────────────────┘
        ▲
        │ LiDAR UDP point 56300→56301
        │ LiDAR UDP IMU   56400→56401
Livox Mid-360（192.168.1.201）
```

证据：

- **CONFIRMED** DEV-VM：`hostname=ubuntu`、VMware hypervisor、`ens33=192.168.1.104/24`；`ens37=172.16.11.130/24` 通过 NAT 网关 `172.16.11.2` 上网。
- **CONFIRMED** Jetson：SSH 到 `.103` 后 `hostname=lite`、`eth0=192.168.1.103/24`，机器人局域网无默认路由。
- **CONFIRMED** Motion Host：SSH 到 `.120` 后 `hostname=ysc`、`eth1=192.168.1.120/24`（同一接口还配置 `192.168.137.120/24`）。
- **CONFIRMED** 双向运动桥参数完全配对：Jetson `transfer_launch.py` 指向 `.120:43893`、监听 `43897`；Motion Host `/home/ysc/jy_exe/conf/network.toml` 指向 `.103:43897`、监听 `43893`。
- **CONFIRMED** `tcpdump` 在 Jetson `eth0` 实时收到 `.201:56300 -> .103:56301` 的 1380-byte 点云包及 `.201:56400 -> .103:56401` 的 60-byte IMU 包。
- **CONFIRMED** Windows `.102` 可 ping `.201` 但抓不到 UDP，与“LiDAR 单播目标是 `.103`”一致，不是虚拟机桥接丢包。

## 3. 计算环境

### 3.1 HOST=DEV-VM

| 项目 | 结果 | 状态/证据 |
|---|---|---|
| OS | Ubuntu 20.04.6 LTS | CONFIRMED，`/etc/os-release` |
| Kernel / Arch | 5.15.0-139 / x86_64 | CONFIRMED |
| Compute | VMware VM，4 vCPU；宿主 CPU 型号透传为 i5-1240P | CONFIRMED；不是 Lite3 算力 |
| RAM | 7.7 GiB，Swap 923 MiB | CONFIRMED |
| Storage | 根分区约 20 GiB，已用约 95%，余量约 1.1 GiB | CONFIRMED；不适合存大 rosbag |
| GPU | VMware SVGA；无 NVIDIA/CUDA | CONFIRMED |
| ROS | ROS 2 Galactic，CycloneDDS 可用；无 ROS 1 | CONFIRMED |
| Network | ens33=机器人网；ens37=NAT/默认路由 | CONFIRMED |

本机发现 `/home/duyipei/Lite3_Projects/Lite3_MotionSDK`，但未发现 Lite3 的 ROS 感知/导航工作区。`cyberdog_remote_ws` 等目录属于其他机器人项目，不应混入 Lite3 审计。

### 3.2 HOST=ROBOT-PERCEPTION (`192.168.1.103`)

| 项目 | 结果 | 状态/证据 |
|---|---|---|
| OS | Ubuntu 20.04.6 LTS | CONFIRMED |
| Kernel / Arch | 5.10.120-tegra / aarch64 | CONFIRMED |
| Compute | NVIDIA Jetson Xavier NX Developer Kit，6 CPU cores | CONFIRMED，设备树与 Tegra 信息 |
| RAM | 6.7 GiB 可见（8GB 级），Swap 3.3 GiB | CONFIRMED |
| Storage | eMMC/SD 根分区约 116.5 GiB，余约 52.5 GiB | CONFIRMED |
| JetPack / L4T | JetPack 5.1.2 / L4T R35.4.1 | CONFIRMED，dpkg |
| CUDA / cuDNN / TensorRT | CUDA 11.4.315；cuDNN 8.6；TensorRT 8.5.2.2 | CONFIRMED |
| Power mode | 20W 6CORE | CONFIRMED，`nvpmodel` |
| ROS | ROS 2 Foxy + ROS 1 Noetic 均安装 | CONFIRMED |
| 当前 ROS | ROS 2 Foxy，`ROS_DOMAIN_ID=0`，`rmw_cyclonedds_cpp`，`ROS_LOCALHOST_ONLY=0` | CONFIRMED |
| 当前系统时间 | 2025-09-16；NTP enabled 但 `NTPSynchronized=no` | CONFLICT，与真实审计日期相差近一年 |

### 3.3 HOST=ROBOT-MOTION (`192.168.1.120`)

| 项目 | 结果 | 状态/证据 |
|---|---|---|
| OS | Ubuntu 20.04.5 LTS | CONFIRMED |
| Kernel / Arch | 5.10.160 / aarch64 | CONFIRMED |
| Compute | Rockchip RK3588，板卡 `WEB-S3588-YSC-V10`，8 CPU cores | CONFIRMED |
| RAM / Storage | 3.8 GiB，无 Swap；根分区约 27.9 GiB | CONFIRMED |
| GPU/CUDA | 未发现 CUDA/NVIDIA；RK3588 GPU/NPU 软件状态未调查 | UNKNOWN；与本轮 SLAM 部署无直接关系 |
| ROS | `/opt/ros` 不存在，当前无 ROS 环境 | CONFIRMED |
| Motion | `jy_exe.service`、`track.service`、CAN0-3 | CONFIRMED；厂商自定义系统 |
| 底层硬件/关节控制细节 | 闭源 `jy_exe` 内部实现 | UNKNOWN；本轮未触碰 |

## 4. 当前 ROS 运行时

当前 Jetson 节点：

```text
/camera/camera
/motion_receiver
/motion_sender
/sensor_checker
/app_receiver
```

当前没有：

```text
LiDAR publisher
Faster-LIO / FAST-LIO / FAST-LIO2 / FAST-LIVO2
hdl_localization
Nav2 planner/controller/bt_navigator
VOA
任何 ROS action server
```

只有两个厂商 systemd 服务在维持关键 ROS 数据：

- `realsense_ros2.service`：启动 RealSense ROS 2 driver。
- `transfer_ros2.service`：启动 Robot Motion UDP ↔ ROS 2 bridge。

## 5. 实机关键硬件与数据

### 5.1 LiDAR

| 项目 | 结果 | 状态/证据 |
|---|---|---|
| 型号 | Livox Mid-360（非雷神 C16） | CONFIRMED：MID360 SDK/配置、`.201` 实时 Livox 数据端口、此前正确驱动可连接；未读取铭牌序列号 |
| IP | 192.168.1.201 | CONFIRMED |
| Raw data | 点云 UDP 56300→56301；内部 IMU UDP 56400→56401 | CONFIRMED，实时抓包 |
| Driver | `livox_ros_driver2` | CONFIRMED，厂商目录与启动脚本 |
| 配置 topic | `/rslidar_points`，`sensor_msgs/msg/PointCloud2`，10 Hz，frame `rslidar` | CONFIRMED，launch/source |
| 配置 PointCloud2 fields | x/y/z/intensity/tag/line/timestamp | CONFIRMED，driver source |
| 当前 ROS publisher | 0 | CONFIRMED；`/rslidar_points` 仅因 sensor_checker 订阅而显示 |
| 当前 ROS 数据 | FAIL | CONFIRMED；raw UDP 有，ROS driver 未运行 |

厂商同时保留了雷神 C16 驱动和配置，但它与当前 `.201` Livox 实机不匹配。历史 `poll() timeout 2368/2369` 是启动错误驱动后的必然结果，不能用来否定 Mid-360 UDP 链路。

### 5.2 Camera

| 项目 | 结果 | 状态/证据 |
|---|---|---|
| 型号 | Intel RealSense D435i，USB ID `8086:0b3a` | CONFIRMED，USB 与 librealsense 日志 |
| Driver | realsense2_camera 3.2.3 / librealsense 2.50.0 | CONFIRMED |
| Firmware | 05.13.00.50 | CONFIRMED，driver log |
| RGB | `/camera/color/image_raw`，RGB8，424×240，约 30 Hz | CONFIRMED，运行时 |
| Depth | `/camera/depth/image_rect_raw`，Z16，424×240，约 30 Hz | CONFIRMED，运行时 |
| CameraInfo | `/camera/color/camera_info` | CONFIRMED，有 publisher 和有效 K |
| RGB frame | `camera_color_optical_frame` | CONFIRMED |
| Depth frame | `camera_depth_optical_frame` | CONFIRMED |
| Point cloud | launch 设置 `enable_pointcloud=false` | CONFIRMED；`/camera/depth/color/points` 当前无 publisher |
| Hardware sync | `enable_sync=False`，日志 `Sync Mode: Off` | CONFIRMED |
| D435i IMU | 设备具备 motion module，但当前未启用 gyro/accel topic | CONFIRMED |

当前 RGB CameraInfo：`fx=301.9205, fy=301.5201, cx=213.3062, cy=129.1182`，畸变模型 `plumb_bob`，424×240。它是当前分辨率下可用的相机内参证据，不可替换为公开案例中另一台 D435i 的数值。

### 5.3 IMU

存在两个不同来源，必须区分：

1. **Lite3 Motion IMU**
   - `/imu/data`，`sensor_msgs/msg/Imu`，约 200 Hz，frame `base_link`。
   - 数据由 `.120` 的厂商状态 UDP 经 `.103 /motion_receiver` 转换。
   - 物理 IMU 芯片型号：**UNKNOWN**。
   - ROS stamp 使用 Jetson `RCL_STEADY_TIME`，不是统一的系统 epoch。

2. **Mid-360 internal IMU**
   - 实时 raw UDP 已确认；厂商 Livox driver 正常运行时可发布 `/livox/imu`、`sensor_msgs/msg/Imu`，约 200 Hz，frame `livox_frame`。
   - 当前 driver 未运行，所以当前无该 ROS topic publisher。
   - 物理 IMU 芯片型号：**UNKNOWN**。
   - 后续 FAST-LIVO2 优先候选应是该 IMU，而不是默认把 Lite3 `/imu/data` 当作同源 IMU。

### 5.4 Odometry 与 Motion Interface

- `/leg_odom`：`geometry_msgs/msg/PoseWithCovarianceStamped`，约 200 Hz，来源是 Motion Host。
- `/leg_odom2`：`nav_msgs/msg/Odometry`，约 200 Hz，来源是 Motion Host；运行时样本的 `header.frame_id` 与 `child_frame_id` 都为空，属于接口缺陷。
- `/odom`：当前不存在；仅在 `hdl_localization` 运行时由它发布。
- `/Odometry`：当前不存在；仅在 Faster-LIO 运行时由它发布。
- 控制入口只读确认：`/cmd_vel`、`/cmd_vel_corrected`、`/simple_cmd`、`/complex_cmd` 被 `/motion_sender` 消费并转换为 `.103 -> .120:43893` 的厂商 UDP。审计未发布任何控制信息。

## 6. Workspace 与 Package

```text
/home/ysc/lite_cog_ros2                       # 当前 ROS 2 Foxy 厂商树
├── driver/mid360_ws                         # livox_ros_driver2（实机 LiDAR）
├── driver/leishen_ws                        # lslidar_driver/msg（遗留/不匹配实机）
├── driver/realsense_ws                      # realsense2_camera/msg/description
├── driver/orbbec_ws                         # Orbbec 驱动（安装但非实机相机）
├── transfer                                 # transfer + transfer_interfaces，Motion UDP 桥
├── slam                                     # faster_lio, pcd2grid, octomap
├── nav                                      # dr_nav2, hdl_localization/global_localization
├── navigation2-foxy                         # 厂商编译的部分 Nav2
├── voa                                      # Depth/ultrasound safety correction
├── fast_gicp / ndt_omp / grid_map_foxy      # 定位/避障依赖
└── system                                   # 启动脚本、map、log

/home/ysc/lite_cog                            # ROS 1 Noetic 历史/遗留树
├── driver
├── slam (faster-lio 等)
├── nav (move_base/TEB/hdl localization 等)
└── transfer/pipeline/voa
```

判定：机器是“ROS1 + ROS2 都安装”，但当前运行时是 **ROS 2 only**；不能写成当前 ROS1/ROS2 bridge 混合运行。

## 7. 时间同步审计

| 链路 | 分类 | 证据 |
|---|---|---|
| Mid-360 point ↔ internal IMU | `LIKELY_SAME_CLOCK / NEED_VERIFY` | 两类原始包 timestamp 瞬时接近，但 `time_type=0`，未做长时间漂移测试 |
| Mid-360 ↔ external reference | `UNSYNCED` | 点云与 IMU包 `time_type=0`；无 PTP/gPTP/PPS 配置证据 |
| RealSense RGB ↔ Mid-360 | `UNSYNCED` | RealSense sync off；相机使用 Jetson epoch，LiDAR 使用设备时钟 |
| Lite3 `/imu/data` ↔ Camera | `UNSYNCED` | `/imu/data` 使用 STEADY_TIME；Camera 使用错误的 Jetson system epoch |
| Jetson system clock | `UNSYNCED` | 日期为 2025-09-16，NTP 未同步；真实日期为 2026-09-01 |
| Camera + LiDAR + IMU 整体 | **`UNSYNCED`** | 不存在一个已证实的共同物理时钟或已验证的软件校正 |

另外，当前 Livox driver 的 ROS 2 PointCloud2 分支把 header stamp 改为 `RCL_STEADY_TIME.now()`，但 Livox IMU 与 CustomMsg 使用设备 timestamp。这会在不同输出格式间制造额外时间语义差异，后续必须在独立 driver/adapter 中修正，不能直接沿用。

## 8. 证据边界与未做事项

- 未读取或改变 LiDAR 固件设置；LiDAR firmware 版本仍 `UNKNOWN`。
- 未读取相机/雷达/IMU序列号，不在报告保存设备身份信息。
- 未启动任何 mapping/localization/nav/VOA launch。
- 未调用 service/action，未发送 `/cmd_vel`。
- 未更改 systemd、网络、ROS 环境或 `.bashrc`。
- 未编译、安装或替换任何厂商 workspace。
