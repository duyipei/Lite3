# Lite3 室外雷达建图、视觉初始重定位与安全导航项目交接

更新日期：2026-09-21（Asia/Shanghai）

用途：将本文全文提供给新的对话版 GPT，使其快速理解当前工程、已经完成的工作、
安全边界和下一步操作。以下内容是当前事实基线；如果现场输出与本文冲突，应以新的
实测结果为准，并保留原始日志后再判断。

## 1. 用户目标

在 Lite3 机器狗上实现以下完整流程：

1. 同步采集 Mid360、IMU、RealSense RGB-D 数据；
2. 用 LiDAR + IMU 生成定位点云和二维栅格导航地图；
3. 用同步 RGB-D 数据和同一条 LIO 轨迹建立视觉关键帧数据库；
4. 机器狗重新上电或在路线中途启动时，用前置相机检索视觉地图，得到粗初值；
5. 把视觉粗初值交给 Mid360 NDT，并且只有 NDT 连续确认后才允许进入导航；
6. Nav2 输出必须先经过默认 DISARMED 的速度门，操作员现场二次确认 ARM 后机器狗
   才能运动。

当前重点不是重新建图，而是使用已经生成并人工编辑过的室外地图完成视觉初始定位和
安全的 RViz 点选导航。

## 2. 运行环境和连接关系

### 本地开发虚拟机

- 用户：`duyipei`
- 项目根目录：
  `/home/duyipei/Lite3_Projects/lite3_patrol_project`
- 本地 RViz 使用 ROS 2 Galactic。
- SSH 私钥路径：`/home/duyipei/.ssh/id_ed25519_cyberdog`
- 不要在文档、聊天或日志中输出私钥内容。

### Jetson 机器人主机

- SSH：`ysc@192.168.1.103`
- Jetson 项目目录：`/home/ysc/lite3_patrol_project`
- 机器人原厂软件目录：`/home/ysc/lite_cog_ros2`
- ROS 2：Foxy
- RealSense 工作空间：
  `/home/ysc/lite_cog_ros2/driver/realsense_ws/install/setup.bash`
- RealSense systemd 服务：`realsense_ros2.service`
- 相机：Intel RealSense D435I，序列号 `243322074594`
- ROS 图像话题：`/camera/color/image_raw`
- ROS 深度话题：`/camera/depth/image_rect_raw`

不要把“电脑能用相机预览”和“Jetson ROS 正在发布图像”混为一谈。相机重新接回
Jetson 后，应关闭其他独占相机的预览程序，再启动 Jetson 的 RealSense 服务。

## 3. 当前使用的数据集和地图

### 数据采集编号

```text
outdoor_20260921_003
```

这是当前室外路线的 LiDAR、IMU、RGB-D 同步采集编号。

### 导航地图编号

```text
outdoor_20260921_003_area_review
```

本地目录：

```text
/home/duyipei/Lite3_Projects/lite3_patrol_project/navigation/
  outdoor_20260921_003_area_review/
```

关键文件：

- `localization_map.pcd`：NDT 定位点云，约 80 MiB；
- `patrol_map_area.pgm`：人工编辑后的二维栅格图；
- `patrol_map.pgm`：指向 `patrol_map_area.pgm` 的符号链接；
- `patrol_map.yaml`：Nav2 实际加载的地图描述；
- `patrol_map_area.yaml`：同一编辑地图的辅助描述；
- `nav2_safe.yaml`：低速、安全优先的 Nav2 参数；
- `visual_initialization.yaml`：视觉粗定位到雷达地图的变换与确认门限；
- `traversability_summary.json`：地图边界和生成统计；
- `traversability_diagnostics.png`、`traversability_evidence.npz`：可通行性诊断证据。

当前栅格地图参数：

```text
分辨率：0.1 m/cell
尺寸：654 x 342
地图边界：x = [-13.1, 52.2] m，y = [-15.1, 19.0] m
occupied_thresh：0.65
free_thresh：0.196
negate：0
```

`free_thresh` 已从 `0.25` 修正为 `0.196`，目的是让灰色 205 保持 unknown，不能把
未知区域错误当作自由空间。不要把该值改回 0.25。

人工编辑后的 PGM SHA-256：

```text
20c075056762c1f50019a8656e3d9adc45e26529836630d2a490375a5aa81d81
```

该编辑结果已经作为当前导航地图使用，不要重新用旧图覆盖。

### 视觉地图

视觉地图编号：

```text
outdoor_20260921_003
```

本地数据库：

```text
/home/duyipei/Lite3_Projects/lite3_patrol_project/results/visual_maps/
  outdoor_20260921_003/visual_map.db
```

当前数据库约 5.6 MiB，共 107 个关键帧，包含 `keyframes` 和 `metadata` 表。数据库
SHA-256：

```text
c1513babf1e089f02be07b73c85b33fe99d742aaea18d5ab82702f88d92e9e57
```

可视化结果：

- `views/visual_keyframe_contact_sheet.png`
- `views/visual_map_overview.png`

视觉数据库和 NDT 点云由同一条 FAST-LIVO 轨迹生成，因此当前配置使用：

```text
T_lidar_map_visual_map = identity
map_transform_x/y/z/yaw = 0
```

不要在没有新标定证据时随意修改这个变换。

## 4. 核心架构和设计思路

整体链路：

```text
RealSense RGB
    -> 视觉关键帧检索（ORB / PnP / 多帧一致性）
    -> 视觉地图中的粗位姿
    -> visual_pose_initializer 发布 /initialpose
    -> Mid360 NDT 连续确认
    -> /patrol/localization_ready = true
    -> 启动隔离 Nav2
    -> /patrol/cmd_vel_raw
    -> 默认 DISARMED 的 velocity_gate
    -> /cmd_vel_corrected
    -> 机器狗底盘
```

关键原则：

- 视觉只负责提供粗初值，不直接作为持续导航位姿；
- NDT 必须用真实、连续、未过期的扫描结果确认视觉初值；
- 视觉或 NDT 失败时不能绕过确认门；
- `start` 只把系统带到 NAVIGATION READY，速度门仍是 DISARMED；
- 真正运动前必须人工检查 RViz 中地图、点云、机器人位置和规划路径，然后输入两次
  `ARM` 确认；
- 不允许直接向 `/cmd_vel` 发布测试速度；
- 机器狗物理运动始终要求现场人员持有官方遥控器并能立即急停。

### 视觉检索策略

默认：

```text
LITE3_VISUAL_SEARCH_STRATEGY=startup_then_global
```

含义：先尝试起点附近关键帧；8 秒内无法确认时切换到全库检索。还支持：

- `startup_only`：明确知道机器狗位于建图起点；
- `global`：明确是路线中途重启，直接全库检索。

全库模式不是任意视角必然成功。相机画面必须与至少一个建图关键帧有足够重叠；白墙、
玻璃、强逆光、动态人群和重复地砖都会降低可靠性。

### 视觉到 NDT 的确认门

当前 `visual_initialization.yaml` 的重要门限：

```text
视觉稳定样本：10
视觉稳定持续时间：4.0 s
NDT 确认样本：15
NDT 最大 matching error：0.15
视觉粗位姿与 NDT 最大位置差：0.65 m
视觉粗位姿与 NDT 最大航向差：0.27 rad
定位后最大单帧位置跳变：0.50 m
定位后里程计最大允许陈旧时间：0.70 s
```

初始化通过后会持续监测 NDT 状态、腿里程计、地图边界和位姿异常。定位失败会锁存，
应停止会话并重新执行视觉初始化；`CLEAR` 只清速度门故障，不能恢复定位失败。

### Nav2 安全参数摘要

- 机体 footprint：约 `0.6 m x 0.4 m`，额外 padding `0.05 m`；
- 最大前进速度：`0.15 m/s`；
- 最大后退速度由速度门限制为 `0.08 m/s`；
- 最大角速度：`0.40 rad/s`；
- 最大前向加速度：`0.20 m/s²`；
- 全局规划 `allow_unknown: false`；
- 局部障碍层使用 `/aligned_points`；
- 膨胀半径：`0.30 m`；
- 速度命令超时：`0.5 s`；
- 定位 READY 心跳超时：`1.5 s`。

当前避障仍主要依赖经过 NDT 链路的降采样点云。不要声称已经完成独立高密度避障点云、
自体裁剪或所有细小障碍物验证。

## 5. 一键启动脚本

主入口：

```text
/home/duyipei/Lite3_Projects/lite3_patrol_project/scripts/indoor_nav_start.sh
```

尽管文件名含 `indoor`，当前通过显式 MAP_ID 和视觉地图 ID 使用室外地图。

`start` 自动执行：

1. 同步运行脚本和地图配置到 Jetson；
2. 运行静态和实时预检查；
3. 启动 Mid360、IMU 和 NDT；
4. 启动视觉关键帧检索；
5. 启动视觉初始化守卫并请求一次初始化；
6. 等待 NDT 确认并得到 `localization_ready=true`；
7. 启动本地 RViz 和地图显示；
8. 切换到完整但隔离的 Nav2；
9. 启动目标桥和默认 DISARMED 的速度门；
10. 进入交互控制循环。

### 当前正确的预检查命令

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project

LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./scripts/indoor_nav_start.sh preflight outdoor_20260921_003_area_review
```

### 当前正确的启动命令

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project

LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./scripts/indoor_nav_start.sh start outdoor_20260921_003_area_review
```

视觉初始化期间机器狗必须静止，前置相机无遮挡，光照和建图时尽量接近。

### 会话内控制命令

主脚本进入 `导航控制>` 后支持：

- `STATUS`：读取实时定位、速度门、目标和规划状态；
- `CLEAR`：仅清除速度门锁存故障，仍保持 DISARMED；
- `ARM`：二次确认后允许真实运动；
- `DISARM`：停止向底盘放行运动速度；
- `CANCEL`：先 DISARM，再取消当前目标；
- `QUIT`：DISARM 并退出本次会话。

### 独立管理命令

```bash
LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./scripts/indoor_nav_start.sh status outdoor_20260921_003_area_review

LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./scripts/indoor_nav_start.sh disarm outdoor_20260921_003_area_review

LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
./scripts/indoor_nav_start.sh stop outdoor_20260921_003_area_review
```

`stop` 只停止本项目的定位、地图显示和导航进程；不要用模糊的全局 `pkill` 杀掉
Jetson 上无关服务。

## 6. 2026-09-21 相机问题的结论和脚本修复

曾出现以下表象：

```text
[PASS] RealSense 服务在线
[FAIL] /camera/color/image_raw 无实时帧
[FAIL] /camera/depth/image_rect_raw 无实时帧
```

期间排查过 USB 枚举、服务进程、软启停和物理拔插。用户随后确认相机在电脑上可以
稳定显示实时画面；重新接回 Jetson 并启动服务后，ROS 彩色图像也能被实际读取。

最终发现主脚本存在确定的误判逻辑：它用 `ros2 topic hz` 判断相机是否有帧，而本项目
已有代码明确记录，这台 Jetson 的 Foxy 中 `ros2 topic hz` 可能在消息实际到达时仍保持
静默。因此不能仅凭没有 `average rate:` 判断断流。

已修改 `scripts/indoor_nav_start.sh`：

- 不再使用 `ros2 topic hz` 作为相机启动门；
- 改为在 20 秒内读取并反序列化一帧
  `/camera/color/image_raw`，首行应为 `header:`；
- 在线视觉定位实际只消费 RGB，因此不再把深度流设为导航启动的强制条件；
- `navigation/start_visual_localizer.sh` 原本就使用等待真实图像帧的方式，无需同类修改。

修改后已实际运行完整 `preflight`，结果全部通过，包括：

```text
[PASS] RealSense 服务在线
[PASS] 没有遗留的定位/导航进程
[PASS] /camera/color/image_raw 有实时帧
[PASS] /cmd_vel 无发布者
[PASS] /cmd_vel_corrected 无发布者
```

这说明当前彩色图像链路和修复后的脚本预检查均已验证。深度流不是本次在线视觉初始
定位的依赖，不应因为没有单独验证深度帧而阻止雷达 + RGB 视觉导航；但以后若重新采集
RGB-D 建图数据，深度流仍必须作为采集预检的强制项。

RealSense 服务命令：

```bash
sudo systemctl restart realsense_ros2.service
```

注意：`systemctl is-active` 只能证明服务进程存在，不能证明有图像。导航侧仍必须读取
一帧真实 RGB 消息。电脑预览程序和 Jetson ROS 驱动也不应同时独占同一台 USB 相机。

## 7. 当前已验证状态

最近一次只读预检查的结果：

- 当前五个导航/定位文件均可读；
- `nav2_safe.yaml` 可解析且速度、足迹和避障边界有效；
- 视觉数据库可读；
- 视觉地图到雷达地图变换已配置；
- `realsense_ros2.service` active；
- `/camera/color/image_raw` 能读取真实实时帧；
- 没有遗留的本项目定位/导航进程；
- `/cmd_vel` 和 `/cmd_vel_corrected` 均无发布者；
- 预检查返回码为 0。

尚未在本次相机脚本修复之后执行完整 `start -> 视觉定位 -> NDT 确认 -> RViz 点目标 ->
ARM -> 实际行走`。因此下一步应该是从干净状态运行 `start`，先观察视觉和 NDT 结果，
不要一开始就 ARM。

## 8. 下一步建议顺序

1. 保持机器狗静止，运行一次 `preflight`；
2. 全部 PASS 后运行 `start`；
3. 等待终端明确显示视觉粗定位已由 NDT 确认、`NAVIGATION READY`；
4. 在 RViz 核对机器人位置、朝向、点云和人工编辑后的地图是否与现场一致；
5. 第一个目标只选白色自由区内、距离约 1～2 m 的短距离目标；
6. 查看规划路径，现场净空并手持遥控器后，才在主终端输入 `ARM`，并按提示再次输入
   `ARM`；
7. 观察低速运动、避障和定位稳定性；任何异常立即 `DISARM`，必要时使用官方遥控器
   急停；
8. 首次短距离 A 到 B 测试通过后，再逐步扩大距离，不要直接执行长路线或多航点巡逻。

如果视觉初始化失败：

- 不要手工伪造 `/initialpose` 来绕过守卫；
- 保持机器狗静止，检查画面是否与已有关键帧重叠；
- 可小角度调整朝向后停稳，再用 `global` 模式重试；
- 查看临时日志目录中的 `visual_localizer.log`、`visual_initializer.log` 和
  `localization.log`；
- `waiting for initial pose input` 本身只是 NDT 在等视觉/人工初值，不是根因；应继续
  向上查视觉图像、视觉匹配和初始化请求。

## 9. 采集和内存/磁盘策略

现场采集入口：

```text
scripts/outdoor_lidar_visual_mapping.sh
```

采集只记录传感器，不发布运动命令；机器狗移动只使用官方遥控器。后端把统一 rosbag
写成最大 512 MiB 的 `/dev/shm` 分段，并由 spooler 顺序复制已关闭分段到持久存储，
避免 Foxy SQLite 随机写入阻塞，也限制单个内存分段的大小。正式采集要求至少 10 GB
项目盘空闲，并应在剩余空间低于 5 GB 前停止。

不要因为当前导航只需要 RGB，就删除采集脚本中的深度检查：RGB-D 深度数据仍是生成
视觉地图三维关键点的必要输入。导航预检和建图采集预检是两个不同场景。

Jetson 曾清理过重复 bag 和中间产物。与旧数据备份相关的已知状态：

- `site_fusion_20260908_001` 已压缩到本地并校验后删除远端源；
- `site_fusion_20260908_002` 的一次 rsync 曾中断，本地可能留有不完整副本，远端源当时
  未删除；
- 不要假设所有旧 bag 都已完整备份或已删除；再次清理前必须先列目录、比较大小并做
  校验。

## 10. 重要限制和禁止事项

- 不要自动 ARM；
- 不要绕过 `/patrol/localization_ready`；
- 不要在定位未确认时启动完整 Nav2；
- 不要把 unknown 栅格改成 free；
- 不要用扩大近场删除范围或抬高障碍物最低高度来掩盖点云/自体点问题；
- 不要把 `waiting for initial pose input` 当成 NDT 崩溃；
- 不要仅用服务 active 或话题 publisher count 判断相机正常；必须读取真实消息；
- 不要用这台 Jetson 上可能静默的 `ros2 topic hz` 作为唯一实时性判据；
- 不要删除 Jetson 数据，除非本地副本已完成、校验通过且删除目标经过明确确认；
- 不要覆盖人工编辑后的 `patrol_map_area.pgm`；
- 不要声称深度流当前已验证。当前已验证的是导航所需的 RGB 实时帧；
- 不要在没有现场证据时改相机外参、雷达外参、固定高度补偿或机体 footprint。

## 11. 给后续 GPT 的工作方式建议

1. 先复述准备执行的动作和是否可能让机器狗运动；
2. 优先读取已有脚本和日志，不要凭印象重写整个系统；
3. 检查应尽量少，但必须能证明关键链路真的有数据；
4. 对相机、雷达和定位使用“实际收到消息”而不是“节点/服务存在”作为判断依据；
5. 任何失败都先保持 DISARMED，并从最上游传感器向下排查；
6. 修改前保留用户已编辑的地图和不相关文件；
7. 给命令时明确是在本地虚拟机还是 Jetson 执行；
8. 对任何删除、固件升级、系统重启、外参修改或真实运动操作，都应先说明影响并取得
   明确授权；
9. 用户偏好“只做必要检查”，避免重复跑长时间或无关诊断；
10. 当前最合理的下一动作是完整启动但保持 DISARMED，验证视觉初值和 NDT 确认，
    而不是重新建图或再次修改地图。

## 12. 关键代码与文档索引

- 一键导航：`scripts/indoor_nav_start.sh`
- 室外采集入口：`scripts/outdoor_lidar_visual_mapping.sh`
- 室内采集入口：`scripts/indoor_lidar_visual_mapping.sh`
- 采集后端：`scripts/round3a_capture_backend.sh`
- NDT 定位启动：`navigation/start_localization_only.sh`
- 视觉检索启动：`navigation/start_visual_localizer.sh`
- 视觉初始化守卫：`navigation/start_visual_initialization.sh`
- 隔离 Nav2：`navigation/start_nav_isolated.sh`
- 默认 DISARMED 速度门：`navigation/start_gate_disarmed.sh`
- 视觉数据库校验：`scripts/validate_visual_map_db.py`
- 当前安全修复说明：`docs/2026-09-21_navigation_guard_repair.md`
- 视觉初始化原理与旧室内实测：`docs/2026-09-14_visual_initial_relocalization.md`
- 采集流程：`docs/2026-09-12_indoor_lidar_rgbd_mapping.md`

---

给新对话的建议开场白：

```text
下面是一份 Lite3 机器狗室外雷达建图、视觉初始重定位和 Nav2 安全导航项目的最新交接
文档。请先阅读并以它作为当前事实基线。不要自动让机器狗运动，不要绕过视觉 + NDT
定位确认或默认 DISARMED 的速度门。后续请区分本地虚拟机命令和 Jetson 命令，并只做
必要检查。
```
# 历史记录提示

本文为历史调试记录。2026-10-13 起室内、室外扫图入口已删除并统一为
`bash scripts/mapping_start.sh start`。当前命令以
[Jetson同步与建图导航命令](Jetson同步与建图导航命令.md) 为准。

