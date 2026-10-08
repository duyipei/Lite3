# Lite3 融合建图命令手册（2026-09-08 复盘版）

本手册用于下一次现场执行 Mid360、内置 IMU、D435i RGB-D 同步采集，随后以
0.5x 回放生成 LIO 点云地图。所有运动都由现场人员使用官方遥控器完成；采集脚本
不发布运动命令。

## 1. 本次复盘得到的关键结论

- 旧版正式 recorder 带有 330 秒超时，导致机器狗返回原点之前 LiDAR、IMU 和 RGB
  已停止，得到的不是完整闭环数据。
- 新版正式采集没有固定时间限制，由现场人员明确执行 `stop` 才结束。
- LiDAR、Camera/IMU、Depth 由一个 rosbag2 recorder 统一采集。它在 `/dev/shm`
  中按 512 MiB 自动分卷，后台只把已封卷文件顺序复制到 Jetson 持久磁盘，既避免
  Foxy SQLite 直接写 eMMC 导致严重降频，也不会让 RAM 随路线时长无限增长。
- 正常 `stop` 会先关闭当前分卷，再把最后一卷和 `metadata.yaml` 完整归档到持久磁盘。
- Depth 已集成到一键采集，无需再手动启动第二个 `ros2 bag record`。
- 18 秒预检仍有 `timeout`，它只是静止传感器质量检查，不限制正式路线时间。
- 无时间限制不等于无限存储。当前组合数据率约 15 MB/s，10 分钟约 9 GB，20 分钟
  约 18 GB。建议开始前至少空闲 30 GB，并在路线中定期执行 `status`。

## 2. 每次出发前：安装最新版脚本

在本地 Ubuntu（DEV-VM）执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh install
```

该命令会同步采集后端和 LIO bag 合并工具到 Jetson。

## 3. 设置唯一 RUN_ID

每次使用新名称，不允许覆盖旧数据：

```bash
RUN_ID=site_fusion_YYYYMMDD_001
```

例如：

```bash
RUN_ID=site_fusion_20260909_001
```

## 4. 现场预检与开始采集

在本地终端 A：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
RUN_ID=site_fusion_20260909_001

./round3a_capture.sh preflight
./round3a_capture.sh start "$RUN_ID"
```

`start` 会完成以下动作：

1. 检查 VM 与 Jetson 时间；
2. 检查没有 Nav2、VOA、FAST-LIVO2 或运动话题发布者；
3. 检查磁盘至少空闲 15 GB；
4. 启动隔离的 Mid360 driver 和 time probe；
5. 用 `/dev/shm` 做一次 18 秒静止短包检查；
6. 启动一个包含全部传感器、无时间限制的分卷 recorder 和持久化搬运进程；
7. 静止采集 25 秒并检查统一 bag 持续增长。

必须等到终端明确显示：

```text
GO: recording is healthy; on-site operator may now use the official controller
```

才可以使用官方遥控器移动机器狗。

## 5. 路线要求

- 起点静止 25 秒；终点回到同一位置、同一朝向后再次完全静止。
- 低速行走，转弯时减速，不原地高速旋转。
- 关键路口、走廊、坡道和导航点尽量从多个方向观察。
- 若计划视觉重定位，应确保相机能看到纹理、门框、立柱、标牌等稳定特征。
- 避免镜头过曝、强烈逆光、大面积无纹理墙面和大量动态人员。

在另一个本地终端随时检查状态：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh status "$RUN_ID"
```

确认以下 recorder 为 `RUNNING`：

```text
sensor_recorder
spooler
```

同时确认 `/dev/shm` 没有持续逼近满容量。磁盘空闲空间接近 5 GB，或 `spooler`
显示 `STOPPED` 时，必须立即结束路线并执行 `stop`。

## 6. 正常停止与验包

先用官方遥控器让机器狗在原点完全停止，再执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project/scripts
./round3a_capture.sh stop "$RUN_ID"
./round3a_capture.sh verify "$RUN_ID"
```

`stop` 会先录制 25 秒末段静止数据，然后停止统一 recorder。验证必须同时检查：

- LiDAR 约 10 Hz；
- IMU 约 200 Hz；
- RGB 不低于约 24 Hz（通常约 25–30 Hz）；
- Depth 不低于约 10 Hz；
- `depth_to_color` 外参至少 1 条；
- 所有传感器来自同一 bag，覆盖范围一致。

紧急情况：先用官方遥控器停止机器狗，再执行：

```bash
./round3a_capture.sh abort "$RUN_ID"
```

`abort` 只停止采集，不会停止机器狗运动；所有已写入的部分 bag 会保留。

## 7. Jetson 上合并 LiDAR 与 IMU

从本地登录 Jetson：

```bash
ssh -t -i ~/.ssh/id_ed25519_cyberdog ysc@192.168.1.103
```

在 Jetson 中执行：

```bash
RUN_ID=site_fusion_20260909_001
PROJECT=/home/ysc/lite3_patrol_project
DERIVED=$PROJECT/results/round3a_lio_motion/${RUN_ID}_lio.db3

cd "$PROJECT"

python3 scripts/merge_round3a_lio_bag.py \
  "$PROJECT/bags/$RUN_ID" \
  "$DERIVED"
```

要求输出：

```text
integrity=ok
```

并核对 LiDAR、IMU 消息数和时间跨度。

## 8. 使用 0.5x 离线生成 PCD

先确认 FAST-LIVO2 临时 PCD 目录没有遗留文件：

```bash
find \
  /home/ysc/lite3_patrol_project/fast_livo2_ws/fast-livo2-deep-robotics/src/FAST-LIVO2/Log/PCD \
  -maxdepth 1 -type f -name '*.pcd' -print
```

如果有输出，先人工检查并转存，不要直接覆盖。

执行正式 0.5x 建图：

```bash
RUN_ID=site_fusion_20260909_001
PROJECT=/home/ysc/lite3_patrol_project
DERIVED=$PROJECT/results/round3a_lio_motion/${RUN_ID}_lio.db3
OUT_ID=${RUN_ID}_map_05x_pcd

cd "$PROJECT"

scripts/round3a_lio_offline_backend.sh \
  "$DERIVED" \
  "$OUT_ID" \
  0.5 \
  0.5 \
  1
```

参数依次为：输入 bag、输出编号、LIO voxel、播放速度、是否保存 PCD。这里播放速度
明确为 `0.5x`。

## 9. 生成 10 cm PCD 和地图图片

```bash
RUN_ID=site_fusion_20260909_001
PROJECT=/home/ysc/lite3_patrol_project
OUT=$PROJECT/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd

cd "$PROJECT"

pcl_voxel_grid \
  "$OUT/map_raw.pcd" \
  "$OUT/map_voxel_10cm_compressed.pcd" \
  -leaf 0.1,0.1,0.1

pcl_convert_pcd_ascii_binary \
  "$OUT/map_voxel_10cm_compressed.pcd" \
  "$OUT/map_voxel_10cm.pcd" \
  1

python3 scripts/render_round3a_map.py \
  "$OUT/map_voxel_10cm.pcd" \
  "$OUT/trajectory.csv" \
  "$OUT/map_views"
```

主要输出：

```text
map_raw.pcd
map_voxel_10cm.pcd
trajectory.csv
odometry_summary.json
map_views/map_top_xy.png
map_views/map_side_xz.png
```

## 10. 闭环质量检查

```bash
cat "$OUT/odometry_summary.json"
```

如果现场实际回到原点，重点检查：

```text
closure_xy_m
closure_z_m
closure_yaw_deg
non_monotonic_stamps
step_m.max
```

同时查看俯视图中绿色起点与蓝色终点是否重合。若数据录制在返回途中结束，或者闭环误差
明显不可接受，禁止生成导航任务。

注意：当前 LIO-only 流程本身不包含回环位姿图优化。完整记录是必要条件，但不能保证
闭环误差严格为零；正式导航地图还应增加回环优化和实地定位验证。

## 11. 把地图复制到本地电脑

退出 Jetson，在本地执行：

```bash
RUN_ID=site_fusion_20260909_001
ROBOT=ysc@192.168.1.103
REMOTE_OUT=/home/ysc/lite3_patrol_project/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd
LOCAL_OUT=/home/duyipei/maps/${RUN_ID}_map_05x

mkdir -p "$LOCAL_OUT/map_views"

scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_OUT/map_raw.pcd" "$LOCAL_OUT/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_OUT/map_voxel_10cm.pcd" "$LOCAL_OUT/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_OUT/trajectory.csv" "$LOCAL_OUT/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_OUT/odometry_summary.json" "$LOCAL_OUT/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_OUT/map_views/map_top_xy.png" "$LOCAL_OUT/map_views/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_OUT/map_views/map_side_xz.png" "$LOCAL_OUT/map_views/"
```

## 12. 视觉初始重定位数据链路

新版一次采集会生成：

```text
bags/$RUN_ID/sensors
```

其中 RGB、Depth、相机内参、`depth_to_color` 外参和同期 LIO 轨迹可用于建立视觉关键帧
数据库。运行时视觉定位节点应输出 `/visual_localization/pose`，再由现有
`visual_pose_initializer` 转换为 `/initialpose`，最后由 NDT 二次确认。

在视觉数据库、视觉地图到 LiDAR 地图变换和多起点实测尚未完成前，不允许将
`visual_initialization.yaml` 的 `configured` 设置为 `true`。
