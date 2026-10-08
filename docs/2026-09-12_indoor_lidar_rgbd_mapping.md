# Lite3 室内雷达 + RGB-D + IMU 建图采集命令

本文只负责现场数据采集。地图离线生成、部署、重定位和导航见
`2026-09-12_indoor_map_generation_navigation.md`。

当前方案的“雷达 + 视觉建图”含义是：一次 rosbag 同步记录 Mid360、IMU、RGB、
Depth、相机内参和深度到彩色相机外参；随后用 LiDAR + IMU 生成 FAST-LIVO2
坐标系和点云地图，再把 RGB-D 关键帧按同一条 LIO 轨迹写入视觉数据库。
相机并不直接参与 FAST-LIVO2 点云配准。

## 1. 现场安全与路线要求

- 所有命令均在本地 Ubuntu 执行，移动只使用官方遥控器。
- 脚本不发布运动指令；必须由现场人员全程握住遥控器并能急停。
- 使用全新的 `RUN_ID`，脚本拒绝覆盖同名数据。
- 室内路线尽量形成闭环：从 A 点静止出发，最终回到 A 点并恢复相近朝向。
- 起点和终点各静止至少 25 秒；低速行走，转弯减速。
- 走廊、门口和计划导航区域尽量从两个方向通过；相机同时观察门框、标牌、
  柱子等稳定纹理，避免只拍白墙、玻璃、强逆光和动态人员。
- 如果希望以后在某处通过视觉找初值，该处必须在本次路线中被相机清晰看到，
  最好从多个方向重复观察。

## 2. 首次使用或脚本更新后安装

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh install
```

该命令会同步无时限采集后端、bag 合并工具、0.5× 离线建图脚本和本手册到 Jetson。

## 3. 设置本次唯一编号

```bash
RUN_ID=indoor_fusion_20260912_001
```

日期和序号按实际情况修改。之后同一个终端始终使用同一个 `RUN_ID`。

## 4. 出发前预检

机器狗站立静止，在本地终端 A 执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh preflight
```

预检应确认 Mid360、IMU、彩色图、深度图和相机参数都有数据，并确认没有 Nav2、
建图回放或运动话题冲突。出现任何 `[ERROR]` 时不要出发。

可在本地终端 B 打开相机预览：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh view-video
```

## 5. 开始无时限统一采集

本地终端 A：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh start "$RUN_ID"
```

只有看到下面一行后才可用官方遥控器移动：

```text
GO: recording is healthy; on-site operator may now use the official controller
```

正式记录没有 330 秒或其他固定时限，但受 Jetson 磁盘容量限制。统一数据保存在：

```text
/home/ysc/lite3_patrol_project/bags/$RUN_ID/sensors/
```

不要另外执行第二个 `ros2 bag record`，否则会重复写盘并增加丢帧风险。

## 6. 行走中检查

在本地终端 B 随时执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh status "$RUN_ID"
```

必须看到 `sensor_recorder` 和 `spooler` 都为 `RUNNING`，并确保项目盘仍有至少
5 GiB 空闲空间。如果 recorder 或 spooler 停止，立即让机器狗停稳并结束采集。

## 7. 回到起点后正常停止

先用官方遥控器回到 A 点、恢复接近出发时的朝向并完全停稳，再执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh stop "$RUN_ID"
./round3a_capture.sh verify "$RUN_ID"
```

`stop` 会保留约 25 秒终点静止段并完整封卷。`verify` 至少应满足：

- Mid360 接近 10 Hz；
- IMU 接近 200 Hz；
- RGB 通常约 25~30 Hz；
- Depth 持续有数据；
- `/camera/extrinsics/depth_to_color` 至少 1 条；
- 各传感器的起止时间跨度基本一致。

## 8. 紧急中止

先用官方遥控器停止机器狗，再执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh abort "$RUN_ID"
```

`abort` 只停止采集，不会让正在运动的机器狗自动停车；已写入的部分 bag 会保留。

## 9. 采集完成后的唯一下一步

不要在现场直接删除 bag。先执行 `verify`，然后按照地图生成手册依次生成：

1. LiDAR + IMU 派生 bag；
2. 0.5× FAST-LIVO2 点云和完整轨迹；
3. RGB-D 视觉关键帧数据库；
4. Nav2 二维地图与安全参数。
