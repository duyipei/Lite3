# 第二轮实施计划：FAST-LIVO2 独立验证与 A/B

## 1. 第二轮目标与边界

第二轮仍不以“立刻自主巡检”为目标，而以以下可验收结果为目标：

```text
建立可复现的 Camera + Mid-360 point + Mid-360 IMU 数据集
        ↓
完成时间轴与标定闭环
        ↓
在隔离环境跑通 FAST-LIVO2 offline
        ↓
在 Xavier NX 完成静态 live 验证
        ↓
人工遥控采集 mapping 数据
        ↓
与厂商 Faster-LIO 做同数据 A/B
        ↓
形成长期地图与定位接口方案
        ↓
只读接入 Nav2 验证，最后才批准低速运动测试
```

任何阶段失败都停在该阶段，不以调大阈值、关闭安全检查或替换厂商系统强行推进。

## 2. 必须保留的厂商组件

以下内容保持原位、只读基线，不覆盖、不混编：

1. `/home/ysc/lite_cog_ros2/transfer`：Lite3 Motion UDP↔ROS接口。
2. `/home/ysc/lite_cog_ros2/slam`：Faster-LIO/pcd2grid 基线和当前地图格式参考。
3. `/home/ysc/lite_cog_ros2/nav` 与 `navigation2-foxy`：hdl_localization/Nav2现有接口。
4. `/home/ysc/lite_cog_ros2/driver/*`：厂商 driver 源码和原始 launch/config，只做对照。
5. `/home/ysc/lite_cog_ros2/system`：厂商启动脚本、服务和地图路径；第二轮也不直接编辑。
6. `.120` 上 `jy_exe`、CAN、motion config：完全不改。

在第二轮开始前记录上述目录的 git commit、关键 config checksum 和 systemd unit内容，作为回滚/差异基线；不需要复制整个 workspace。

## 3. 独立 Workspace 与版本策略

建议在 Robot Perception Host `.103` 新建：

```text
/home/ysc/lite3_patrol_project/
├── fast_livo2_ws/       # DeepRobotics fork + 独立 livox driver/adapter
├── calibration/         # intrinsics/extrinsics及报告
├── bags/                # 小规模验证bag；大bag放外部SSD/NAS
├── config/              # 只保存项目自己的配置
├── scripts/             # 只读检查、record、offline launch
├── logs/
└── results/             # A/B metrics/maps/trajectory
```

基础实现：

- 主基础：`DeepRoboticsLab/fast-livo2-deep-robotics` 的 **soft-sync** 分支，固定到经验证的 commit。
- 算法对照：`hku-mars/FAST-LIVO2` 固定 commit，不直接混入厂商 workspace。
- 不在第二轮初期采用 upstream ROS1 live部署；它会引入 ROS1/ROS2 bridge、TF和message转换的额外变量。

ROS选择必须先做一个短技术验证：

1. **优先方案**：在独立 workspace把 Deep fork最小 backport到 ROS2 Foxy，使其直接与当前厂商 ROS2 graph共存。
2. **备选方案**：独立 Ubuntu22/ROS2 Humble容器，仅运行 driver + FAST-LIVO2；使用 host network和明确 device权限，禁止覆盖 host依赖。
3. 不升级 Jetson host OS/ROS，不把 Humble包安装进 Foxy系统路径。

选择门槛：能在 public bag上复现、消息类型正确、没有修改 `/opt/ros/foxy` 或厂商 install tree。若 Foxy backport维护成本高，再选择容器；不要同时维护两条正式路线。

## 4. 需要新增的依赖

只在选定的独立环境中安装，并先形成锁定清单：

- Deep fork自带/配套：Livox-SDK2、`livox_ros_driver2`、FAST-LIVO2 ROS2 package。
- ROS：PCL ROS bridge、compressed image transport、cv_bridge/image transport、rosbag2工具。
- 算法：PCL、Eigen、OpenCV、Sophus、Vikit/rpg_vikit对应ROS2 port。
- 评估：evo或等价轨迹工具、PCD/map质量统计脚本、CPU/RAM/thermal采样脚本。
- 标定：相机标定工具、FAST-Calib及选定的 LiDAR-IMU标定工具。

先做 `rosdep` dry-run/依赖矩阵，再经批准安装。禁止大范围 `apt upgrade`、系统 OpenCV/CUDA替换或覆盖 `/usr/local` 中现有 librealsense。

## 5. 真实 Topic 接入设计

FAST-LIVO2 项目内部统一放入 namespace，避免与厂商输出/TF冲突：

| FAST-LIVO2 input | 真实来源 | 第二轮处理 |
|---|---|---|
| `/fast_livo2/livox/lidar` | `.201` Mid-360 | 独立 livox_ros_driver2输出 `livox_ros_driver2/msg/CustomMsg`，保留 timebase/offset_time/tag/line |
| `/fast_livo2/livox/imu` | Mid-360 internal IMU | 独立 driver输出 `sensor_msgs/msg/Imu`；不用现有 Motion `/imu/data`代替 |
| `/camera/color/image_raw` | D435i current RGB | 初期保留 424×240×30；配置模型必须完全匹配 CameraInfo |
| Camera model file | `/camera/color/camera_info` | 生成项目自己的 pinhole yaml；运行时算法未必直接订阅 CameraInfo |

注意：同一台 Mid-360 UDP端口不能由厂商 driver和项目 driver同时绑定。第二轮建立明确的 driver ownership：

- 采集/FAST-LIVO2窗口只启动项目 driver；厂商 LiDAR driver保持停止。
- 厂商基线 A/B优先使用同一 rosbag回放。
- 若将来必须 live共享，只允许“一个 driver、多消费者”或经验证的消息转换器，不允许两个 driver抢端口。

## 6. 输出接口与 TF 隔离

验证阶段输出：

```text
/fast_livo2/odometry          nav_msgs/msg/Odometry
/fast_livo2/cloud_registered  sensor_msgs/msg/PointCloud2
/fast_livo2/path              nav_msgs/msg/Path
/fast_livo2/status            项目健康/同步/丢帧状态（adapter产生）
```

验证 frame：

```text
fast_livo2_map -> fast_livo2_body
```

在通过 A/B之前：

- 不发布 `map -> odom`、`odom -> base_link` 等生产 frame。
- 不覆盖 `/odom`、`/Odometry` 或厂商 TF。
- Nav2不订阅 FAST-LIVO2结果。

进入长期定位后，目标规范才调整为：

```text
map -> odom_livo        # global localization correction
odom_livo -> base_link  # smooth local odometry
base_link -> lidar/camera/imu  # confirmed static calibration
```

每条 TF只能有一个权威 publisher。

## 7. 标定计划

### 7.1 Camera intrinsics

1. 冻结 RGB profile：第一轮使用当前 424×240×30。
2. 保存 CameraInfo与温度/对焦状态；用棋盘/AprilGrid复核重投影误差。
3. 生成 `camera_pinhole_424x240.yaml`，记录日期、设备、分辨率和 RMS。
4. 如果视觉质量/特征数不足，再单独评估 640×480×30；切分辨率必须重新生成/验证内参，不能沿用公开 D435i参数。

### 7.2 LiDAR ↔ Camera

1. 使用当前安装姿态采集联合标定数据。
2. 以 FAST-Calib为首选工具，得到 `Rcl/Pcl`、残差和可视化叠加。
3. 做独立 validation bag：墙面边缘、门框和地面在图像/点云投影中一致。
4. 保存原始数据、结果、坐标方向定义和单位；只保存最终通过门槛的版本。

### 7.3 LiDAR ↔ IMU

1. 明确选用 Mid-360 internal IMU。
2. 从 Livox设备/官方 driver确认轴定义、单位、时间戳语义和设备固定变换。
3. 用经验证的 LiDAR-IMU标定流程估计/复核 extrinsic；静止段检查重力方向，受控多轴运动检查 deskew。
4. 禁止复用 `c16.yaml` 的 `[0.075,0,0.023]` 和旋转矩阵。

### 7.4 base frames

单独测量并验证：

- `base_link ↔ lidar`
- `base_link ↔ camera_link`
- `base_link ↔ imu`

这些用于机器人/导航 TF，不等价于 FAST-LIVO2内部 LiDAR-camera/IMU外参。将候选值、最终值和来源分开管理。

## 8. 时间同步处理

### 8.1 先测量，不先调参数

建立 10–20 分钟静态/缓慢运动bag，记录：

```text
CustomMsg timebase与每点offset
Mid-360 IMU header stamp
D435i image header stamp
Jetson CLOCK_REALTIME / CLOCK_MONOTONIC
arrival time
```

统计：初始 offset、jitter、drift ppm、timestamp jump、丢帧率。把 LiDAR↔internal IMU 与 Camera↔LiDAR分开评估。

### 8.2 软件同步候选

在确认实机是多时钟后，验证 Deep fork `soft-sync`：

- IMU-LiDAR：EMA动态 offset。
- Camera-LiDAR：固定 `img_time_offset`，由 bag拟合后写入项目 config。
- 运行中记录残余 offset和校正状态，不能只看地图是否“像”。

### 8.3 决策门槛

- 若静态与低速走廊运动中 residual稳定、无持续 drift，进入 live静态阶段。
- 若快速转向出现明显彩色点云错位或漂移，停止 live mapping；研究 PPS/GPIO硬同步、外部同步板或可硬触发相机。
- D435i RGB硬同步能力是已知限制；必要时按官方建议评估可硬触发的相机方案，而不是永久依赖不断增大的 time offset。

同时修复 Jetson系统时间服务，但要明确：系统 NTP正确并不自动实现传感器硬同步。

## 9. Offline 验证

### Gate O1：官方 public bag

- 固定 repo commit和依赖版本。
- 复现官方 Retail_Street/FAST-LIVO2 dataset。
- 验证 `/cloud_registered`、odometry、path、无异常timestamp reset。
- 记录 Xavier NX与开发环境的 build/runtime差异。

### Gate O2：实机静态 bag

- Robot上电但固定支撑，禁止运动命令。
- 检查 10 Hz LiDAR、约200 Hz internal IMU、30 Hz RGB，消息类型、frame、单调时间。
- FAST-LIVO2静态姿态漂移、地图厚度、内存增长和温度均达标后通过。

### Gate O3：已有/手持数据

- 先移动传感器组件或使用已采集数据，不接运动控制。
- 验证视觉开/关、LiDAR-only和full LIVO三组，定位是视觉增益还是时间误差造成异常。

## 10. 实机静态验证

只允许启动传感器与 FAST-LIVO2，不启动 Nav2/VOA，不连接算法输出到 motion：

1. 检查 driver ownership和端口唯一性。
2. 保持机器人趴下/支撑、急停可用。
3. 启动顺序：project Livox driver → existing camera → FAST-LIVO2；每一步先核对 publisher/type/hz/stamp。
4. 监测 Xavier NX CPU/GPU/RAM、swap、温度、降频、DDS丢帧和 end-to-end latency。
5. 连续运行至少 30 分钟；检查内存增长、timestamp jump和 odometry静漂。

若 Xavier NX资源不足，依次减少可视化/dense map/保存开销和输出频率；不先牺牲原始时间戳或传感器频率。仍不足则将 Compute评级改为 `INSUFFICIENT` 并评估 Orin升级。

## 11. 人工遥控 Mapping

该阶段必须另行批准，并满足：开阔走廊、现场安全员、手柄优先、急停、限速、无行人、记录完整bag。

建议顺序：

1. 直线低速往返，检查尺度、重力和deskew。
2. 大半径转弯，检查camera-LiDAR时差。
3. 单层闭环走廊，回到起点检查位置/姿态闭合误差。
4. 重复同一路线不同光照，检查视觉退化。
5. 最后才做室外短路线；不在第二轮直接做无人巡检。

FAST-LIVO2只记录输出，不发送运动命令。机器人运动全部由人员手柄控制。

## 12. 与当前 SLAM 做 A/B

使用同一 rosbag和同一空间裁剪范围，至少三组：

```text
A0 厂商 Faster-LIO as-is（记录其 C16/Mid-360输入缺陷）
A1 独立 overlay 中正确保留Mid-360逐点时间的 Faster-LIO（若完成）
B  FAST-LIVO2 soft-sync
```

不修改厂商源树。指标：

- trajectory ATE/RPE或闭环首尾误差；
- 地面/墙面厚度、重影、动态物体拖影；
- LiDAR退化/弱纹理/逆光条件下丢失次数；
- timestamp residual、丢帧率、处理延迟；
- CPU/RAM/swap/温度/功耗；
- 30分钟与长路线稳定性；
- 地图重启复现性。

只有 B在多场景显著优于有效基线且资源可接受，才进入定位接口设计。

## 13. 长期地图与 Localization

FAST-LIVO2本身先作为 mapping/local odometry，不直接宣称解决重启后的全局定位。

1. 从通过A/B的mapping run冻结版本化 PCD、轨迹、标定和参数。
2. 生成与现有系统兼容的2D occupancy `pgm/yaml`，但不覆盖 `system/map/lite3.*`。
3. 先在独立 namespace回放验证 `hdl_localization` 对新 PCD的 NDT匹配。
4. 实现/验证重启初始化：人工 `/initialpose`基线 → hdl_global/其他全局重定位 → 错误恢复。
5. 将 global localization输出规范化为 `map->odom_livo`，FAST-LIVO2提供 `odom_livo->base_link`；避免两个节点同时发布 `map->base_link`。
6. 重复路线验证跨日、光照、移动家具、室内/室外连接处的重定位成功率。

验收前保持人工 initial pose为安全fallback。

## 14. 最后接入 Navigation Stack

只读/仿真顺序：

1. rosbag/静态 graph验证 Nav2能收到 `/map`、`/odom`、TF和LiDAR障碍数据。
2. 修正项目 overlay中的 `use_sim_time`一致性，不编辑厂商 yaml。
3. 验证 STVL输入高度、frame和更新率；Depth VOA作为独立安全项目评估，不与Nav2同时无审查接管命令。
4. 重新设计单一路径：`Nav2 /cmd_vel_raw -> safety mux -> /cmd_vel_safe -> motion_sender`。Motion sender不得再同时接受可绕过安全层的raw/safe消息。
5. 单点导航先在仿真/离线测试取消、超时、定位丢失、障碍停止。
6. 经安全评审后才做围栏场地低速单点实机。
7. 单点稳定后增加 waypoint follower任务层：点位版本、循环次数、取消、skip/retry/abort、定位质量门槛、人工接管。
8. 最后才进入楼道循环和校园室外巡检；视觉检测/后台告警另立阶段。

## 15. 第二轮“新增/修改/保留/验证”清单（Q6）

### 新增

- 独立 FAST-LIVO2 workspace与锁定版本。
- 独立 Livox CustomMsg/internal IMU数据链。
- Camera/LiDAR/IMU标定文件、原始数据和质量报告。
- time offset/drift测量与soft-sync状态记录。
- rosbag录制、资源监控、A/B评估脚本。
- namespaced odometry/cloud/path/TF接口。

### 修改（仅项目 overlay/独立配置）

- FAST-LIVO2 topic、camera model、intrinsics、extrinsics和time offsets。
- Foxy兼容层或Humble容器配置。
- 后期 navigation overlay的 TF、time mode、odom与costmap输入。
- 后期命令mux，使安全修正成为唯一运动出口。

### 保留

- 厂商 transfer/motion接口、Faster-LIO、hdl_localization、Nav2、driver源码和system脚本。
- `.120` 底层运动服务和参数。
- 当前可工作的 D435i RGB/Depth基础服务，除非经批准做隔离profile测试。

### 验证

- 数据真实存在、消息类型/频率/frame/timestamp。
- 每对传感器的offset/jitter/drift。
- 外参重投影与deskew。
- Xavier NX 30分钟以上资源/热稳定性。
- 同bag Faster-LIO vs FAST-LIVO2地图/轨迹质量。
- 重启后的全局重定位。
- TF单一权威、Nav2静态graph、动态障碍输入。
- 命令唯一通道、取消/失败/人工接管；通过安全评审后才进行低速运动。

## 16. 第二轮停止条件

出现下列任一情况立即停在当前 Gate：

- 需要覆盖厂商 workspace、升级 host OS/ROS或修改 Motion Host才能继续；
- timestamp非单调、offset持续漂移或图像/点云明显错位；
- calibration无可复现质量报告；
- TF有多 publisher、frame断裂或坐标约定不明；
- Xavier NX持续swap/降频/丢帧；
- 任何算法输出被意外连接到 `/cmd_vel` 或 `.120`；
- 安全mux未完成却准备启动 Nav2/VOA运动。

停止后记录 `BLOCKED`、证据和下一项建议，不以实机运动“试试看”。
