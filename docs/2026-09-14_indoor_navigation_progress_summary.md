# Lite3 室内导航阶段工作总结

更新时间：2026-09-14  
当前室内数据编号：`indoor_fusion_20260912_001`  
当前导航地图编号：`indoor_fusion_20260912_001_area_review`

## 1. 当前结论

室内导航已经完成从统一传感器采集、0.5× 离线雷达建图、二维可通行区域生成、人工初值
NDT 重定位、Nav2 路径规划、跨 ROS 版本目标桥接到安全速度门的大部分链路。

现场已经确认：

- RViz 中地图能够正常显示；
- 人工设置初始位置后，雷达点云能够与地图基本对齐；
- NDT 能够自动选择稳定候选并持续输出 `map -> base_link`；
- RViz 点击的目标能够送达 Jetson Foxy Nav2；
- Nav2 能生成方向和位置正确的全局路线；
- 速度门能够保持默认断开，也曾成功返回 `ARMED`；
- 定位或导航状态异常时，速度门和目标桥能够自动撤销运动许可。

尚未完成的最终验收是：机器狗在真实室内环境中连续完成一次可见的短距离 A→B 运动。
目前遇到的两次不运动，分别由瞬时定位状态撤销和目标失效后 `nav_command_stale` 引起，
不是地图或全局路线错误。对应守卫逻辑已经修正并完成独立测试，但修正后的整套真机运动尚待
重新验证。

## 2. 当前系统链路

```text
Mid360 + IMU + 腿部里程计
        |
        v
人工 2D Pose Estimate + NDT/腿里程计融合
        |
        +--> /odom + map -> base_link
        |
        v
持续 NDT 守卫 --> /patrol/localization_ready
        |
        v
本地 Galactic RViz /patrol/goal_request
        |
        v
Jetson Foxy 目标桥 --> /navigate_to_pose
        |
        +--> /patrol/goal_active
        |
        v
Nav2 --> /patrol/cmd_vel_raw
        |
        v
安全速度门（默认 DISARMED）
        |
        v
/cmd_vel_corrected --> 原厂 motion_sender --> Lite3
```

`/cmd_vel` 保持没有发布者，Nav2 的速度只能经过安全门，不能绕过门直接进入底盘。

## 3. 各阶段完成状态

| 阶段 | 状态 | 说明 |
|---|---|---|
| LiDAR、IMU、彩色和深度相机统一采集 | 已完成 | 已取消 330 秒限制，正式采集无时限，人工停止 |
| 0.5× 离线 LIO 建图 | 已完成 | 已生成 PCD、轨迹、俯视图和里程计摘要 |
| 宽区域二维导航地图 | 已完成 | 使用射线、地面支撑、坡度及连通性生成，不再只沿采集轨迹开窄通道 |
| 人工初值 + NDT 重定位 | 已验证 | 多次成功自动重锚并进入 READY |
| 视觉数据采集 | 已完成 | 统一 bag 中保留彩色、深度及相机内外参话题 |
| RGB-D 视觉数据库 | 未完成 | 输出目录存在，但当前没有 `visual_map.db`；不能宣称视觉自动初始重定位已跑通 |
| RViz 全局路径规划 | 已验证 | 现场看到正确目标和正确路线 |
| Galactic RViz 到 Foxy Nav2 目标桥 | 已验证 | Nav2 日志确认接受桥接目标 |
| ARM 前目标与速度预检 | 已修改并独立验证 | 无活动目标或无非零原始速度时不再调用 ARM 服务 |
| 速度隔离、限速和自动断开 | 已验证 | 默认 DISARMED，故障会锁存或拒绝 ARM |
| 瞬时 NDT 抖动过滤 | 已修改并独立验证 | 1～2 帧速度尖峰不撤销，连续 3 帧才撤销 |
| 瞬时 NDT 状态过滤 | 已修改并独立验证 | 1～4 帧不收敛保持 READY，连续 5 帧才撤销 |
| 真机短距离 A→B | 尚未通过 | 需要使用最新版守卫重新进行现场运动验证 |
| 动态避障 | 尚未验收 | Nav2 已加载 STVL 障碍层和膨胀层，但尚无真机绕障验收结果 |
| 多点自动巡航 | 尚未开始最终验收 | 应在单点 A→B 稳定通过后再启用 |

## 4. 地图与数据处理工作

### 4.1 统一采集

已经把 Mid360、IMU、彩色图、深度图、相机参数和诊断状态录入同一个传感器 bag，避免
之前 LiDAR 与相机分别录制后难以按时间对齐。采集脚本包含传感器预检、磁盘监测、日志、
手动停止和异常中止流程。

主要文档：

- `docs/2026-09-12_indoor_lidar_rgbd_mapping.md`
- `scripts/round3a_capture.sh`
- `scripts/round3a_capture_backend.sh`

### 4.2 0.5× 雷达地图

离线回放使用 `0.5×`，降低 Jetson 实时计算压力。当前地图产物位于：

```text
/home/duyipei/maps/indoor_fusion_20260912_001_map_05x/
  map_raw.pcd
  map_voxel_10cm.pcd
  trajectory.csv
  odometry_summary.json
  map_top_xy.png
  map_side_xz.png
```

处理流程由以下脚本完成：

- `scripts/merge_round3a_lio_bag.py`
- `scripts/round3a_lio_offline_backend.sh`
- `scripts/render_round3a_map.py`

### 4.3 宽区域导航地图

原始二维图主要沿机器狗实走轨迹生成自由通道。之后增加了
`build_area_traversability_map.py`，利用雷达射线、地面观测、障碍高度、坡度、粗糙度以及
与实走路线的连通性扩展自由区域。未获得足够地面证据的位置仍保持未知，不会因为雷达看到
远处墙面就直接允许通行。

当前地图参数和结果：

- 分辨率：`0.10 m/cell`；
- 网格：`277 × 321`；
- 地图范围：`x=-13.2~14.4 m`，`y=-14.9~17.1 m`；
- 连通自由单元：`961`；
- 占用单元：`1264`；
- 未知单元：`86692`；
- 最大雷达证据距离：`12 m`；
- 障碍膨胀：`0.15 m`；
- 实走路线保证半径：`0.45 m`。

导航地图位于：

```text
navigation/indoor_fusion_20260912_001_area_review/
  localization_map.pcd
  patrol_map.yaml
  patrol_map_area.pgm
  patrol_map_area_preview.png
  traversability_diagnostics.png
  traversability_summary.json
  nav2_safe.yaml
```

白色区域只是离线候选可通行区，不等于已经通过真机地面、台阶和动态障碍验收。

## 5. 重定位与 NDT 修改

### 5.1 当前采用人工初值

现阶段正式启动流程使用 RViz `2D Pose Estimate` 提供绝对地图初值，然后由 NDT 在初值
附近匹配。视觉自动初值暂未作为正式启动条件。

### 5.2 稳定候选自动重锚

NDT 收到人工初值后：

1. 将人工位姿与当前 `/leg_odom2` 建立锚点；
2. 等待连续 30 个稳定 NDT 候选；
3. 只允许在人工初值附近自动重锚一次；
4. 重锚后由腿部里程计提供连续运动，NDT 只做小幅地图修正。

当前主要参数：

- 人工位置门限：`0.50 m`；
- 人工航向门限：当前启动配置约 `12°`；
- 自动重锚最大平移：`0.75 m`；
- 自动重锚最大航向：`15°`；
- NDT 融合增益：`0.05`；
- 单次最大平移修正：`0.01 m`；
- 单次最大航向修正：`0.2°`。

修改位置：

- `vendor_patch/hdl_localization_app.hpp`
- `patrol_nav_ws/src/lite3_patrol_nav/launch/patrol_navigation.launch.py`
- `scripts/leg_odom_tf_bridge.py`

此前隔离定位和 Nav2 满负载静止测试曾获得约 `0.009 m` 最大单步、约 `0.065 m` 全窗口
范围和 100% 收敛率，证明连续预测与有界修正方向有效。

## 6. 持续定位守卫修改

`scripts/manual_ndt_guard.py` 持续检查地图范围、坐标系、LiDAR、NDT 状态、NDT 里程计、
腿部里程计和位姿连续性，并发布：

```text
/patrol/localization_ready
/patrol/manual_ndt_state
```

仍然立即撤销 READY 的硬故障：

- LiDAR、NDT 里程计、腿部里程计或 NDT 状态超过 `0.70 s` 未更新；
- 单帧平移超过 `0.50 m`；
- 单帧航向变化超过约 `20°`；
- 位姿超出地图范围或坐标系错误；
- NDT 与腿部里程计预计位置相差超过 `0.60 m`；
- NDT 与腿部里程计预计航向相差超过约 `20°`。

为避免 Jetson 高负载下的单帧误判，新增两级防抖：

- 位姿速度超过 `1.0 m/s` 时，连续 3 帧才撤销 READY；
- NDT `has_converged=false` 时，连续 5 帧才撤销 READY。

独立 ROS 域测试结果：

```text
两帧 2.0 m/s 往返抖动       PASS，保持 READY
连续三帧 1.5 m/s 异常       PASS，撤销 READY
单帧 0.51 m 跳变            PASS，立即撤销 READY
连续四帧 NDT 不收敛         PASS，保持 READY
连续五帧 NDT 不收敛         PASS，撤销 READY
启动阶段 NDT 短暂异常        PASS，继续保持 NOT READY
```

最后一项 NDT 状态防抖已经部署，并曾通过热重启进入 READY；尚需在下一次完整真机 A→B 中
确认不会再因单帧状态波动取消目标。

## 7. Nav2、RViz 和跨版本目标桥

本地电脑使用 ROS 2 Galactic RViz，Jetson 使用 Foxy。两个版本的
`NavigateToPose` action 定义不完全一致，直接从本地 RViz 调 Foxy action 会失败。

为此新增 `scripts/goal_pose_bridge.py`：

```text
Galactic RViz /patrol/goal_request（PoseStamped）
            -> Jetson Foxy goal_pose_bridge
            -> Foxy /navigate_to_pose action
```

桥节点还发布持久化的 `/patrol/goal_bridge_ready`。最初总控脚本使用
`ros2 action info` 和 `ros2 topic info` 做自检，Jetson 高负载时连续 3 秒超时而误报失败；
现已改为桥节点自身检查 action server，并由轻量 rclpy 探针等待 ready 状态。

桥节点现在还发布持久化的 `/patrol/goal_active`：目标被 Nav2 接受后为 true，目标完成、
取消、失败或定位许可撤销后恢复 false。旧目标的延迟结果不会再覆盖新目标状态。
为避免 RViz 或操作重复发布造成同一目标立即抢占自身，桥节点还会忽略 1 秒内位置相差
不超过 `0.05 m`、航向相差不超过 `5°` 的重复目标。

现场日志已经确认：

- 桥节点成功发现 Foxy action server；
- RViz 目标被 Foxy Nav2 接受；
- RViz 中显示的全局路线和目标正确。

这证明目标输入与全局规划链路已经打通，但不等于局部控制和底盘运动已经验收。

## 8. 速度隔离和安全门

Nav2 的 `/cmd_vel` 已重映射到：

```text
/patrol/cmd_vel_raw
```

安全门 `velocity_gate.py` 默认 DISARMED，只向 `/cmd_vel_corrected` 发布零速心跳。只有人工
两次确认 `ARM`，且定位、LiDAR、里程计和 Nav2 原始命令都新鲜时，才转发非零速度。

当前限速：

- 前后：`0.15 m/s`；
- 横向：`0.03 m/s`；
- 转向：`0.30 rad/s`；
- 输入新鲜度：`0.50 s`。

总控台命令已经整理为：

```text
STATUS / CLEAR / ARM / DISARM / CANCEL / QUIT
```

- `CLEAR` 只清除锁存故障，仍保持 DISARMED；
- `CANCEL` 取消当前目标并关闭速度门；
- `QUIT` 先关闭速度门，再清理本次启动的全部节点。

执行 `ARM` 二次确认后，总控脚本会先运行轻量预检，必须同时满足：

- `/patrol/localization_ready=true`；
- `/patrol/goal_active=true`；
- 速度门没有需要 `CLEAR` 的锁存故障；
- `/patrol/cmd_vel_raw` 正在产生非零速度。

任一条件不满足时不调用 ARM 服务，机器狗继续保持 DISARMED，并直接输出对应原因。

## 9. 一键总控脚本

当前总控脚本：

```text
scripts/indoor_nav_start.sh
```

它统一完成：

1. 同步运行文件到 Jetson；
2. 静态地图和依赖检查；
3. 启动 Mid360、IMU、NDT 和腿部里程计桥；
4. 临时显示地图并打开本地 RViz；
5. 等待人工设置初始位置；
6. 执行 60 秒静止稳定检查；
7. 启动持续定位守卫；
8. 启动与底盘隔离的 Nav2；
9. 启动 Foxy 目标桥并验证连接；
10. 执行 Nav2 负载下的稳定检查；
11. 启动默认 DISARMED 的速度门；
12. 提供交互式安全控制台。

启动命令：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
./scripts/indoor_nav_start.sh start indoor_fusion_20260912_001_area_review
```

## 10. 现场问题与对应修改

### 10.1 节点未启动

早期只运行检查脚本，导致 Mid360、NDT、视觉节点和守卫并未启动。现已将部署、启动、检查、
RViz 和退出清理合并到 `indoor_nav_start.sh`。

### 10.2 NDT 大范围发散

曾出现数十万米坐标和数百米单帧跳变。之后加入人工地图锚、稳定候选自动重锚、腿部里程计
连续预测、NDT 有界融合、地图边界和 NDT/腿里程计一致性检查。

### 10.3 静止检查因约 0.44～0.47 米波动失败

检查门限一度过严，且原 NDT 输出会在多个候选间抖动。当前不是简单放宽所有检查，而是用
腿部里程计维持连续运动，并限制 NDT 每次修正量；硬跳变门限保持 `0.50 m`。

### 10.4 Foxy 目标桥自检误报

Jetson 高负载下 ROS CLI 图查询经常超过 3 秒。已新增 `/patrol/goal_bridge_ready`，取消循环
CLI 查询，桥接动态测试通过。

### 10.5 第一次 ARM 后不运动

第一次成功 ARM 后约 `0.34 s`，守卫因单帧 `2.013 m/s` 位姿速度值撤销 READY，目标桥随即
取消导航。现已改为连续 3 帧确认。

### 10.6 再次 ARM 返回 `nav_command_stale`

目标已被一次约 `0.2 s` 的 NDT 状态波动取消，ARM 时 Nav2 已没有原始速度。发现守卫虽然
累计 5 个坏状态才报硬故障，但定时器在第 1 个坏状态就发布 false。现已修正为运行中连续
1～4 帧保持 READY，第 5 帧才撤销。

之后又增加了 ARM 前目标/速度预检。目标已经结束、取消或局部控制器没有产生非零速度时，
总控脚本会在调用速度门之前停止，并提示重新发送目标或先处理局部规划问题。

### 10.7 Jetson 高负载与间歇断流

完整 NDT + Nav2 运行时曾观察到负载约 `10～13`、NDT 单帧处理超过 `100 ms`、SSH banner
超时、LiDAR/里程计短时 stale 和 RViz 点云队列丢帧。当前已将全局代价地图改为只使用
静态地图和膨胀层，动态 Mid360 障碍物只进入局部 STVL；局部/全局更新频率分别降到
`3 Hz`/`1 Hz`，并关闭体素图及 DWB 高开销调试输出。降载效果仍需下一轮现场复测。

### 10.8 局部规划器恢复

日志中出现过：

```text
Failed to make progress
None of the points of the global plan were in the local costmap
No valid trajectories
```

其中多数发生在目标已经发送但速度门仍为 DISARMED 的等待阶段。全局路线正确，但下一轮
真机测试仍需确认 local costmap、机器人足迹和实际控制输出。

### 10.9 目标在人工 ARM 前触发无进展

原进度检查要求 10 秒内移动 `0.5 m`。目标在速度门仍为 DISARMED 时已经开始计时，人工
观察路径和二次确认 ARM 容易超过 10 秒，随后 Nav2 会进入恢复流程。最初曾把等待时间延长
到 `30 s`；当前进一步改为目标先排队、首次 ARM 后才提交给 Nav2，因此人工确认不再消耗
Nav2 的进度计时窗口。

### 10.10 运动中误停、重复原地旋转与连续目标操作

真机日志确认，一次未到点是持续守卫把 NDT 的正常累计地图修正误判为相对腿部里程计
发散（约 `0.60～0.71 m`），从而撤销 READY 并取消目标。底层定位器本身已经对每帧原始
NDT 修正执行 `0.50 m` 和约 `12°` 的门限，因此删除了持续守卫中数学上不成立的固定锚点
重复检查；地图边界、单帧跳变、速度、收敛和数据新鲜度保护仍然保留。

局部控制器已移除强制到点转向的 `RotateToGoal`，加入路径朝向和防振荡评分，将最大角
速度限制为 `0.25 rad/s`，并忽略 RViz 目标箭头的最终朝向。空间到点半径由 `0.40 m` 收紧为
`0.25 m`，规划替代终点容差由 `1.0 m` 收紧为 `0.20 m`；低速进度门限调整为 45 秒内移动
`0.20 m`。

速度门现在采用“单次 ARM 会话”：第一次目标仍需人工 ARM；目标结束后保持 ARMED 但强制
发布零速，下一次只需在 RViz 点击目标即可立即执行。任何定位、传感器或位姿跳变故障仍会
自动 DISARM 并锁存，必须 CLEAR 后重新 ARM。

### 10.11 只侧移、不朝向路径

现场发现局部控制器仍允许 `vy=±0.03 m/s`，同时 `PreferForward.strafe_x=0.3` 高于机器狗
的最大前进速度 `0.15 m/s`，导致前向评价器无法区分正常前进和低速横移，DWB 因而可能
持续选择侧向轨迹。当前改为正向行走模式：DWB 的横向速度范围固定为 0、`vy_samples=1`，
速度门也将 `linear.y` 硬限制为 0。前向阈值修正为 `0.05 m/s`，最大转向速度设置为
`0.25 rad/s`；机器狗会先进行必要的有限转向，对准路径后向前行走，到点后仍不会为了
RViz 箭头方向额外原地旋转。

### 10.12 只前进、不按路径转向

现场日志确认，问题不在速度门或底盘角速度映射，而是局部控制器实际只生成了固定的
`linear.x=0.064286 m/s`，没有生成非零 `angular.z`。原因是为抑制原地旋转而设置的角速度
代价过大：`PreferForward.scale × theta_scale = 180`，再叠加 `Twirling.scale=20`，任何转弯
轨迹都会输给直行轨迹。

当前改为前进转向模式：横移仍硬限制为 0；角速度上限保持 `0.25 rad/s`；角速度采样从 11
增加到 15；`PathAlign` 增强并加入 `GoalAlign` 和 `Oscillation`；总转向惩罚从约 200 降到 5，
预测时间从 3 秒缩短为 2 秒。由此允许正常的前进加转向弧线，在航向误差较大时允许有限
原地调整，同时继续忽略目标箭头的最终航向，避免到点后二次转圈。

### 10.13 最低速度原地踏步与大角度不转向

第二次现场复测中，目标位于 `(0.58, 1.73)`，路径初段约为 `+90°`，机器人航向约为
`-5°`，但 DWB 在 334 秒内只输出 `linear.x=0.021429 m/s`（偶尔 `0.042857 m/s`），没有
输出非零 `angular.z`。这两个线速度正是 `0～0.15 m/s`、8 个采样点中的前两档，低于
Lite3 的有效行走速度，因此机器狗表现为腿部踏步而机身几乎不动；Nav2 每 45 秒报告一次
`Failed to make progress`，最终 ABORTED。

当前进一步改为：`min_speed_xy=0.06 m/s`、`min_speed_theta=0.08 rad/s`，线速度采样固定为
`0/0.03/0.06/0.09/0.12/0.15 m/s`，从而排除两档无效前进速度；角速度采样增至 21，预测
时间增至 4 秒；删除 `PreferForward` 和 `Twirling` 两个额外角速度惩罚项，以标准
`PathAlign + GoalAlign + PathDist + GoalDist + Oscillation` 组合决定转向。局部代价地图重新
加入静态地图层，转向轨迹使用 `ObstacleFootprint` 做完整足迹碰撞检查。

## 11. 视觉工作的真实状态

当前已经完成：

- 彩色和深度相机在线检查；
- 约 30 Hz 彩色、深度数据验证；
- 相机内参、深度到彩色外参和系统保留外参的使用方案；
- RGB-D 视觉地图构建、校验、可视化和视觉定位节点代码；
- 统一 bag 中相机与 LiDAR/IMU 同步保存。

当前尚未完成：

- `results/visual_maps/indoor_fusion_20260912_001/` 中没有实际生成的 `visual_map.db`；
- 没有完成多个不同起点的视觉检索成功率和误匹配测试；
- 视觉结果尚未作为正式的自动初始位姿来源接入本次室内总控。

因此目前正式可用的是“人工初值 + LiDAR NDT + 腿部里程计”，不能把当前状态描述为已经
实现视觉自主初始重定位。

## 12. 下一步最小验收顺序

1. 使用最新版总控脚本干净启动，确认 60 秒和 Nav2 负载检查通过；
2. 机器狗用原厂遥控器进入允许自主行走的站立状态；
3. RViz 选择实走路线附近、距当前位置 `1～2 m` 的白色区域中心；
4. 只发送一次目标，在 DISARMED 状态快速确认路径和现场净空；
5. 在目标仍活动并持续产生 `/patrol/cmd_vel_raw` 时立即完成两次 `ARM` 输入；
6. 同时记录 `/patrol/cmd_vel_raw`、`/cmd_vel_corrected`、`/odom` 和门状态；
7. 若两个速度话题均为非零但机器狗仍不动，再检查原厂 `motion_sender`、自主模式和运动板；
8. 单点 A→B 连续成功至少 3 次后，再测试更远随机点；
9. 随机点稳定后，再启用多点巡航；
10. 最后单独生成并验证 RGB-D 数据库，再接入视觉自动初始重定位。

## 13. 当前不能宣称完成的能力

- 不能宣称已经完成视觉自动初始重定位；
- 不能宣称雷达看到的全部区域都可安全导航；
- 不能宣称动态避障已完成真机验收；
- 不能宣称多点无人巡航已经跑通；
- 不能绕过速度门直接把 Nav2 连接到底盘。

当前最准确的阶段描述是：**地图、人工 NDT 重定位、全局路径和安全控制链已经基本打通，
正在完成局部控制到真实底盘运动的最后一次短距离闭环验收。**
