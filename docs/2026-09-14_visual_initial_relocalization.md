# 室内视觉初始重定位测试流程（2026-09-14）

适用地图：`indoor_fusion_20260912_001_area_review`

视觉数据库：
`/home/ysc/lite3_patrol_project/results/visual_maps/indoor_fusion_20260912_001/visual_map.db`

## 当前实现

- 起点启动：先检查起点静止阶段的 6 个关键帧，并结合起点附近及闭环返回帧确认场景；连续确认后给出建图起点粗位姿。
- 中途重启：如果 8 秒内不能确认起点，自动切换到全数据库检索，通过 ORB 匹配、PnP 和连续 3 帧一致性给出粗位姿。
- 视觉只提供粗定位，最终必须由 Mid360 NDT 确认。
- 当前视觉到 NDT 的一次性修正窗口为 0.65 m、15°；NDT 运行中的单帧跳变保护仍为 0.5 m。
- NDT 确认后不再用固定的起点视觉先验监督运动过程，以免机器狗离开起点后错误撤销定位状态。
- 本流程不启动 Nav2、不启动速度门，不会让机器狗运动。

## 已完成的验证

- v2 数据库：56 个 RGB-D 关键帧，完整性检查通过。
- 离线抽样：20 帧中 13 帧得到解，12 帧满足 1 m/20°；成功解中位位置误差 0.075 m，中位航向误差 0.9°。
- 现场起点识别输出：`x=-0.00084 m, y=-0.00353 m, yaw=0.05°`。
- NDT 30 秒静止检查：收敛率 100%，中位匹配误差 0.028，净漂移 0.007 m。
- 现场最终状态：`READY:visual_plus_ndt_confirmed_0.520m_0.185rad`。

## 从干净状态启动

以下命令分别放在本地 Ubuntu 的不同终端中执行。

### 1. 启动 Mid360 和 NDT（终端 1）

```bash
ssh -t -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103 \
  'export LITE3_PATROL_MAP_ID=indoor_fusion_20260912_001_area_review
   export LITE3_NDT_RESOLUTION=1.0
   export LITE3_DOWNSAMPLE_RESOLUTION=0.3
   exec bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh'
```

保持该终端运行。

### 2. 启动视觉关键帧检索（终端 2）

自动判断“起点或中途重启”：

```bash
ssh -t -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103 \
  'export LITE3_PATROL_MAP_ID=indoor_fusion_20260912_001
   export LITE3_VISUAL_SEARCH_STRATEGY=startup_then_global
   exec bash /home/ysc/lite3_patrol_project/navigation/start_visual_localizer.sh'
```

明确知道机器狗在建图起点时，可把策略改为：

```bash
export LITE3_VISUAL_SEARCH_STRATEGY=startup_only
```

明确知道机器狗在路线中途重启时，可跳过起点等待：

```bash
export LITE3_VISUAL_SEARCH_STRATEGY=global
```

### 3. 启动视觉初始位姿守卫（终端 3）

```bash
ssh -t -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103 \
  'export LITE3_PATROL_MAP_ID=indoor_fusion_20260912_001_area_review
   exec bash /home/ysc/lite3_patrol_project/navigation/start_visual_initialization.sh'
```

### 4. 请求一次视觉初始化（终端 4）

```bash
source /opt/ros/galactic/setup.bash

ros2 service call /patrol/request_visual_initialization \
  std_srvs/srv/Trigger '{}'
```

如果服务命令停在 `making request`，另开终端观察状态；Jetson Foxy 与本地 Galactic 偶尔不返回 CLI 响应，但服务端仍会收到请求。

```bash
source /opt/ros/galactic/setup.bash

ros2 topic echo /visual_localization/state std_msgs/msg/String
```

另一个终端观察融合状态：

```bash
source /opt/ros/galactic/setup.bash

ros2 topic echo /patrol/visual_init_state std_msgs/msg/String
```

预期顺序：

```text
POSE:mode=startup,...
COLLECTING:1/10 ... COLLECTING:10/10
WAITING_NDT:x=...,y=...,yaw=...
READY:visual_plus_ndt_confirmed_...
```

最后确认：

```bash
ros2 topic echo /patrol/localization_ready std_msgs/msg/Bool
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_corrected
```

只有 `localization_ready` 持续为 `true` 才算重定位通过。视觉测试阶段两个速度话题的发布者都应为 0。

## 中途重启测试

1. 机器狗停稳，保持现场光照和建图时接近。
2. 停止旧的视觉定位进程，再以 `LITE3_VISUAL_SEARCH_STRATEGY=global` 启动终端 2。
3. 依次执行终端 3、终端 4。
4. 等待全库 PnP 连续 3 次一致，再由 NDT 确认。
5. 如果持续显示 `REJECTED:global:no_pnp_solution`，说明当前位置没有足够重叠关键帧。保持原地，用遥控器小角度改变朝向后停稳再试，不要直接启动导航。

## 停止本次测试

按 `Ctrl+C` 依次退出终端 3、终端 2、终端 1。随后确认：

```bash
ssh -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103 \
  'source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
   ros2 topic info /cmd_vel
   ros2 topic info /cmd_vel_corrected'
```

## 当前限制

- 全库重定位并非任意视角都能成功；相机画面需要与某个关键帧有足够重叠。
- 地砖等重复纹理可能产生错误匹配，所以全库模式不能绕过连续一致性和 NDT 确认。
- 当前起点画面相对建图时向下约 10°～15°，纯 PnP 较弱；起点模式采用场景投票确认后给出固定起点粗先验，再交给 NDT 修正。
- 若要提高中途任意位置成功率，应补建更密的关键帧库，尤其是转角、门口和纹理较少区域。
