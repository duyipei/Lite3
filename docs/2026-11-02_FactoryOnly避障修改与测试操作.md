# Lite3 Factory Only 避障反应与通行测试

> **后续变更（2026-11-02）：** 用户要求删除本地栅格速度约束，现已从速度门、配置、启动器、只读探针和录包话题中移除；不再有该层 0.25 秒检查、扫掠停车或三帧放行。原厂 VOA 高程感知、投影、Nav2 全局/局部代价地图、代价地图同步、定位检查、投影心跳与默认 DISARMED 保留。当前版本见 [障碍同步与近障绕行修改](2026-11-02_障碍同步与近障绕行修改.md)。以下内容保留为删除前的历史测试记录，其中本地停车行为、测试结果和限制不能当作当前版本的验收结论；膨胀现值为 0.50 m。本次未替用户进行测试。

本轮修改调整 Factory Only 动态导航覆盖配置和隔离的原厂感知程序，增加直接读取原厂 VOA 栅格的本地停车检查，并完善终点几何验收。静态地图、NDT、重定位算法、原巡航路线及足迹没有重建或缩小。部署不等于运动验收；所有启动仍然默认 DISARMED。

**本轮现场结果：代码已部署，站立状态的 60 秒感知时序检查通过；但前方 0.35–0.55 m、左右各 0.25 m 的采样区域全部缺少深度观测，本地停车检查因此持续阻止前进。现有雷达局部地面模型也未通过只读验收，未启用补盲。当前禁止据此进入运动或完整巡航。以下运动步骤是补盲并完成静止验收后的测试流程。**

## 本轮实测记录

- 本地回归 113 项通过，含实际配置生成、共享解码、速度门、终点检查、NavFn 与 DWB 原生离线测试；合成扫掠留出样本没有检测到漏掉的物理接触。真实尺寸障碍尚未验收。
- Jetson 隔离原厂 VOA 编译通过，最终强制重编译以确保最新队列修复进入二进制。原厂安装没有被覆盖，源码与已安装模块/程序哈希记录于 `verified_runtime_manifest.json`。
- 站立 60 秒收到 1782 帧栅格，无拒绝帧；源年龄 P95 约 38 ms、最大约 183 ms；最大接收间隔约 184 ms。本地虚拟指令评估最大耗时约 30 ms。此测试没有 NDT/Nav2 运动负载，不能当作完整控制负载验收。
- 30 个保存样本中，前方 `0.65≤x≤1.10 m、|y|≤0.20 m` 的已观测疑似地面，首帧均值约 7.8 mm、P95 约 11.3 mm；末帧均值约 9.0 mm、P95 约 12.1 mm。ROI 仅按 XY 选择，未按高度删点，仍不是独立测量的地面真值。
- 这些站立样本的近场盲区一直无观测。1782 次虚拟 0.10 m/s 检查全部因障碍/未知区扫掠条件停车，不能将时序 PASS 当作通行 PASS。
- 趴下时的首轮记录保留于 `perception_static.json`：少量帧过期，最大源年龄约 363 ms，前方有约 29 cm 高程。用户后续确认该轮是趴下状态；不能把不同姿态、启动阶段和不同负载的两轮统计当作严格受控的性能提升比例。
- 额外启动正常雷达驱动做 25 秒只读诊断，225 帧均为 `no_physically_supported_ground_plane`，未强行放宽先验或接入控制。正常数据年龄约 78 ms，IMU 配对误差约 2 ms；时序正常不能代替地面几何正确。
- 仅运行了输出至私有测试话题的速度门，ARM 服务也重映射到私有测试名；底盘速度话题发布者为零。所有本轮自建感知/测试进程随后停止，厂家相机服务保持原样。

证据位于本机及 Jetson 的 `results/avoidance_response_20261102/`：`standing_static.json`、`standing_ground_and_blindspot.json`、`standing_grids/`、`lidar_blindspot_shadow_sensor_on.json`、构建日志和回归日志。完整 Nav2 代价地图消费、负载下重规划、实物停车和巡航未完成本轮验证。

## 修改内容

| 项目 | 修改前 | 修改后 |
|---|---|---|
| 局部代价地图更新 | v2 为 2 Hz | 至少 5 Hz |
| 全局代价地图更新 | v2 为 1 Hz | 至少 2 Hz |
| 代价地图对外发布 | v2 为 0.5 Hz | 2 Hz，改善显示及录包，不等于内部更新频率 |
| 动态膨胀 | 强制至少 0.60 m | Factory Only 默认局部、全局均 0.55 m |
| 位置到达容差 | v2 为 0.35 m | 私有动态配置为 0.12 m，goal checker 与 DWB 一致 |
| 终点验收 | 目标中心周围 0.45 m 圆 | 根据实际足迹、padding、终点朝向、XY 与 yaw 容差检查矩形包络 |
| 本地停车 | 依赖投影云与代价地图链 | 速度门直接订阅 VOA GridMap，无需等待地图投影 |

原厂安装不被覆盖。隔离程序位于 `factory_voa_safety_ws`，`start_factory_voa.sh` 优先选择它；原厂速度输出仍被隔离。首次构建工具为 `tools/build_factory_voa_safety_overlay.py`，修复观测掩码、保留纳秒时间戳，并将点云处理/栅格入口改为只保留最新帧，不修改厂家速度选择算法。

统一候选参数位于 `navigation/obstacle_response.yaml`。Factory Only 以外的 private_combined 配置保留原对照行为。不要只改 `nav2_safe.yaml` 的膨胀值：动态配置由 `scripts/make_dynamic_nav2_config.py` 重新生成。

停车检查沿当前速度对应的曲线检查完整矩形扫掠；考虑栅格尺寸、源数据年龄、采样间隔、近期已发布速度与制动时间。只允许将 vx、vy、wz 同比例乘以 1、0.5 或 0，保持原轨迹曲率，不选择绕行方向。障碍解除需三个不同时间戳的安全栅格后恢复；传感器故障仍锁存 DISARMED。

局部栅格源时间上限为 0.25 s，地图投影心跳原有 0.5 s 保护保留。本地停车检查将未知区域及超出栅格范围视为不允许进入。Factory Only 地图桥仅标记已观测障碍，不再把整片相机未观测区域伪装成实体墙；它不发布清空观测，静态地图的未知区保持不变。当前机器人占用矩形内部的未知格按自遮挡处理，但不会因此将周边未知格变为自由空间；实测障碍即使位于该矩形内也会停车。已发现原厂 CV 转换会丢失 NaN：本轮增加隔离 VOA 感知补丁，保留原始观测掩码并恢复未观测格 NaN。速度门要求 observation_validity 层与高程一致，旧程序缺少该层时拒绝运行许可。

本地模型使用 Nav2 与原厂 VOA 两者中更大的碰撞矩形；v2 对应至少 0.70×0.50 m。真实腿部扫掠若超出该矩形，必须扩大模型。当前相机前向覆盖不能证明后方安全，因此 Factory Only 动态配置继续禁止倒车。

`linear_braking_mps2=0.15`、`angular_braking_radps2=0.4` 是候选保守假设，需要测量实际制动验证。它们不是厂家认证数据。新停车逻辑与规划共享同一相机及 VOA 感知，不能克服相机盲区或将此系统变成独立安全传感器。

## 文件入口与备份

- `scripts/make_dynamic_nav2_config.py`：Factory Only 覆盖配置、受限参数读取。
- `patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/voa_grid.py`：桥接与停车共用的栅格解码，处理 Eigen 存储、环形索引和未知值。
- 同目录 `obstacle_safety.py`：本地扫掠、同轨迹减速、停车与恢复。
- 同目录 `velocity_gate.py`：直接订阅栅格并执行最终速度限制，仍是唯一输出者。
- `scripts/goal_pose_bridge.py`：根据最终动态配置检查带容差的终点足迹，覆盖手动及任务目标；新路径允许有效的空障碍云，不允许旧源时间伪装新数据。
- `scripts/navigation_start.sh`、`navigation/start_gate_disarmed.sh`：部署新文件并传入实际地图参数，缺少 grid_map 依赖时明确失败。
- `scripts/check_avoidance_readiness.py`：只读静止时序测试与本地虚拟指令检查，不创建任何运动发布者。
- `scripts/record_avoidance_test.sh`：录包，不 ARM、不发目标。

本机修改前备份：`results/avoidance_response_20261102/source_before.tar.gz`。Jetson 修改前备份及部署清单位于同名结果目录。未执行 Git 操作。

## 测试前准备

使用 `ssh ysc@10.119.1.171`。本机项目在 `/home/duyipei/Lite3_Projects/lite3_patrol_project`，Jetson 项目在 `/home/ysc/lite3_patrol_project`。

下面以 `patrol_map_20261031_01_area_v2_review` 举例。实机导航必须确认当前现场与这张地图匹配，机器狗正常站立；感知静止检查本身不证明处于地图正确位置。

先完成相机连续出帧、无 watchdog、传感器时间正常、急停可用及控制发布者唯一性检查。先用软箱体，不用人站在运动路径上测试停车。建议测试通道先约 1.2 m，再到 1.0 m、0.9 m；只有上一档通过且物理包络有净空才继续，不把这些尺寸当成保证可通过的门槛。

## 第一步 检查并启动默认未解锁的导航

虚拟机终端：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
export ROBOT=ysc@10.119.1.171
export LITE3_VISUAL_BACKEND=xfeat_lighterglue
bash scripts/navigation_dynamic.sh check patrol_map_20261031_01_area_v2_review
bash scripts/navigation_dynamic.sh start patrol_map_20261031_01_area_v2_review
```

`check` 只生成检查用配置；`start` 会部署并启动定位、视觉初始化、Nav2 和默认 DISARMED 速度门。保持静止直至初始化完成。不要为继续测试跳过初始化或自动解锁。

## 第二步 静止感知与端点检查

Jetson 另一终端：

```bash
cd /home/ysc/lite3_patrol_project
source navigation/patrol_env.sh
source /home/ysc/lite_cog_ros2/grid_map_foxy/install/setup.bash
OUT=results/avoidance_response_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
python3 scripts/check_avoidance_readiness.py --seconds 60 \
  --nav-config navigation/patrol_map_20261031_01_area_v2_review/nav2_dynamic.yaml \
  --output "$OUT/static_timing.json" --save-grids "$OUT/grids"
python3 scripts/check_obstacle_costmap_consumption.py --seconds 20 \
  --output "$OUT/costmap_consumption.json"
```

检查结果必须没有非零 corrected 速度及 ARMED 记录，本地栅格没有超过 0.25 s 的间隔/源年龄，投影没有超过 0.5 s 的间隔/源年龄。`--perception-only` 仅用于未启动导航时的相机栅格检查，不代表全链通过。

在视野内放入、移开软箱体，分别确认原始高程、投影障碍及两个代价地图中位置吻合。确认局部停车诊断随障碍变化。读取对外代价地图的频率不能证明内部 5 Hz 更新按时完成；同时观察控制器与代价地图超周期日志，并在运动负载下复核。

只读探针显示的本地 scale 是虚拟 0.10 m/s 指令的模拟结果，绝不发布该速度。STOP 可能是合法障碍/未知区，不等于算法错误；时序 PASS 也不代表该场景可通行。Factory Only 中没有雷达障碍云是预期行为，不能把缺少雷达云误认为相机验收失败。

## 第三步 录包后做低速单点测试

在发出任何 ARM 之前，Jetson 开启独立录包终端：

```bash
cd /home/ysc/lite3_patrol_project
bash scripts/record_avoidance_test.sh \
  results/avoidance_response_bag_$(date +%Y%m%d_%H%M%S) 300
```

录制覆盖 VOA 栅格、投影、心跳、本地停车诊断、速度指令、地图、TF、里程计、目标与 ROS 日志。录包本身增加负载，验收必须在录包开启时复核数据年龄。

按如下顺序逐项通过；操作终端保留既有人工 ARM 流程：

1. 空场短距离直行和 90° 转向，无目标时零速。
2. 视野内软箱体挡路，先检查减速/停车，测量接近速度、从发现到零速命令延迟、命令归零后实际滑行距离和最终净空。
3. 软箱体留出一侧宽通路，目标放在障碍后方开阔区，验证 Nav2 绕行；本地停车层不能主动转向。
4. 完全封路，必须停车/等待；移除障碍后先确认至少三帧局部安全数据，再观察规划与运动恢复。
5. 测试较窄通道，目标先放在通道外的开阔区域。终点包络包含停车误差，不能用把目标放在狭窄处被拒绝来判断整个通道不可通过。
6. 终点附近有障碍或未知区时应拒绝目标；改变终点朝向应按真实足迹改变验收结果。

不要重复原碰撞目标 15/6，除非重新测量并证明目标及接近路径有净空。180° 转向先拆为两个 90°，当前修改没有宣称根治 DWB 掉头振荡。相机近场、侧方、后方覆盖需单独验证，前向测试成功不等于全向避障成功。

## 第四步 进一步缩小规划膨胀

只有直行、转弯、绕障、停车、移除恢复及实时性都通过后，才将 `navigation/obstacle_response.yaml` 的 `local_inflation_m` 从 0.55 改为 0.50，先保持全局 0.55 与 cost scaling 不变。脚本下一次启动会生成新动态配置；不在运动中改参数。

比较同一场景的路径、最小实际净空、规划成功率、停车次数及控制时延。不能以接触次数为代价换取通过率，也不能因窄通道失败缩小真实足迹、关闭未知区或取消本地停车。不同膨胀值可能只改变路径偏好而不改变硬碰撞约束。

## 中止与验收边界

在操作终端执行 DISARM 或使用遥控器急停。也可在虚拟机新终端请求：

```bash
ROBOT=ysc@10.119.1.171 bash scripts/navigation_dynamic.sh disarm patrol_map_20261031_01_area_v2_review
```

局部栅格过期、地图投影过期、定位失效、发布者冲突应停止并锁存故障；恢复数据不会自动重新 ARM。不要故意在真实运动中关闭相机来验证这类故障，先在模拟或输出隔离的测试环境做注入。

阶段性结果保存于 `results/avoidance_response_20261102/`，应分别报告代码回归、静止感知、完整静止导航、低速运动与完整巡航。当前修改不构成 11 点动态避障已经通过的证明。
