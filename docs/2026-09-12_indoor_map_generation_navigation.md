# Lite3 室内地图生成、重定位与导航测试命令

本文接续 `2026-09-12_indoor_lidar_rgbd_mapping.md`。示例使用：

```bash
RUN_ID=indoor_fusion_20260912_001
MAP_ID=${RUN_ID}_area_review
```

必须把这两个变量替换为本次真实编号。所有命令均拒绝静默覆盖旧结果；如果名称已存在，
应换新编号并保留旧数据，而不是直接删除。

## 1. 在 Jetson 合并 LiDAR + IMU 派生 bag

从本地 Ubuntu 登录 Jetson：

```bash
ssh -t -i ~/.ssh/id_ed25519_cyberdog ysc@10.119.1.171
```

在 Jetson 终端执行：

```bash
RUN_ID=indoor_fusion_20260912_001
PROJECT=/home/ysc/lite3_patrol_project
DERIVED=$PROJECT/results/round3a_lio_motion/${RUN_ID}_lio.db3

cd "$PROJECT"
python3 scripts/merge_round3a_lio_bag.py \
  "$PROJECT/bags/$RUN_ID" \
  "$DERIVED"
```

必须看到 `integrity=ok`，并确认 LiDAR 和 IMU 都有消息。这里传入的是完整运行目录，
不是直接输入 `bags/$RUN_ID/sensors` 这一串文字。

## 2. 在 Jetson 使用 0.5× 生成 LIO 点云地图

先确认 FAST-LIVO2 的临时 PCD 目录为空：

```bash
PCD_TMP=/home/ysc/lite3_patrol_project/fast_livo2_ws/fast-livo2-deep-robotics/src/FAST-LIVO2/Log/PCD
find "$PCD_TMP" -maxdepth 1 -type f -name '*.pcd' -print
```

如果有输出，先人工转存并确认属于哪次实验；不要直接覆盖。确认无遗留文件后执行：

```bash
RUN_ID=indoor_fusion_20260912_001
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

参数依次为：输入 bag、输出目录名、LIO voxel、回放速度、是否保存 PCD。最后两个
`0.5` 含义不同；第四个参数才是 0.5× 播放速度。新版轨迹采集器随 rosbag 播放结束
才停止，不再受 900 秒限制。

生成目录：

```text
/home/ysc/lite3_patrol_project/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd/
```

## 3. 在 Jetson 生成 10 cm PCD 和图片

```bash
RUN_ID=indoor_fusion_20260912_001
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

mkdir -p "$OUT/map_views"
python3 scripts/render_round3a_map.py \
  "$OUT/map_voxel_10cm.pcd" \
  "$OUT/trajectory.csv" \
  "$OUT/map_views"

cat "$OUT/odometry_summary.json"
```

检查 `closure_xy_m`、`closure_yaw_deg`、`step_m.max`、`non_monotonic_stamps`，并查看：

```text
map_views/map_top_xy.png
map_views/map_side_xz.png
```

如果实际回到了起点，但闭环位置或角度明显不一致、出现大跳变或俯视图严重重影，
不要继续制作导航地图。

## 4. 把原始 bag 和地图复制到本地 Ubuntu

退出 Jetson，在本地执行：

```bash
RUN_ID=indoor_fusion_20260912_001
ROBOT=ysc@10.119.1.171
REMOTE_PROJECT=/home/ysc/lite3_patrol_project
LOCAL_MAP=/home/duyipei/maps/${RUN_ID}_map_05x
LOCAL_BAG=/home/duyipei/Transfer/lite3_bags/$RUN_ID

mkdir -p "$LOCAL_MAP/map_views" "$LOCAL_BAG"

scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_PROJECT/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd/map_raw.pcd" \
  "$LOCAL_MAP/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_PROJECT/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd/map_voxel_10cm.pcd" \
  "$LOCAL_MAP/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_PROJECT/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd/trajectory.csv" \
  "$ROBOT:$REMOTE_PROJECT/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd/odometry_summary.json" \
  "$LOCAL_MAP/"
scp -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_PROJECT/results/round3a_lio_motion/${RUN_ID}_map_05x_pcd/map_views/*.png" \
  "$LOCAL_MAP/map_views/"

scp -r -i ~/.ssh/id_ed25519_cyberdog \
  "$ROBOT:$REMOTE_PROJECT/bags/$RUN_ID/sensors" \
  "$LOCAL_BAG/"
```

视觉数据库和扩展导航区域需要原始统一 bag，因此确认这些步骤完成前不要删除 Jetson
中的 `bags/$RUN_ID`。

## 5. 在本地生成 RGB-D 视觉关键帧数据库

当前默认外参沿用已验证可接受的系统参数。若相机或 Mid360 的安装位置发生变化，
必须重新验证外参，不能继续沿用下列默认值。

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project

RUN_ID=indoor_fusion_20260912_001
LOCAL_MAP=/home/duyipei/maps/${RUN_ID}_map_05x
LOCAL_BAG=/home/duyipei/Transfer/lite3_bags/$RUN_ID/sensors
VIS_DIR=$PWD/results/visual_maps/$RUN_ID

mkdir -p "$VIS_DIR/views"

python3 scripts/build_rgbd_visual_map.py \
  "$LOCAL_BAG" \
  "$LOCAL_MAP/trajectory.csv" \
  "$VIS_DIR/visual_map.db" \
  --playback-rate 0.5 \
  --keyframe-distance 1.0 \
  --keyframe-yaw-deg 15 \
  --base-camera-xyz 0.25489 0.0 0.07249 \
  --base-camera-rpy-deg 0.0 20.0 0.0

python3 scripts/validate_visual_map_db.py \
  "$VIS_DIR/visual_map.db" \
  --queries 12

python3 scripts/visualize_rgbd_visual_map.py \
  "$VIS_DIR/visual_map.db" \
  "$VIS_DIR/views"
```

至少检查：

- 输出 `integrity=ok`；
- 关键帧数量明显大于 10；
- `visual_map_overview.png` 覆盖实际路线；
- `visual_keyframe_contact_sheet.png` 中画面清晰且不是大量重复白墙。

视觉数据库已生成不代表“任意位置自动视觉初始重定位”已经通过。当前真机启动脚本
默认使用 RViz 手动初值 + 修正后的 NDT；视觉数据库保留用于后续多位置验证。

## 6. 生成推荐的宽区域二维导航地图

宽区域生成器只把同时具有重复雷达射线、地面支撑、合理坡度/粗糙度并与实走路线连通
的区域设为白色自由区。灰色未知区仍禁止规划。

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project

RUN_ID=indoor_fusion_20260912_001
MAP_ID=${RUN_ID}_area_review
LOCAL_MAP=/home/duyipei/maps/${RUN_ID}_map_05x
LOCAL_BAG=/home/duyipei/Transfer/lite3_bags/$RUN_ID/sensors
NAV_DIR=$PWD/navigation/$MAP_ID

python3 scripts/build_area_traversability_map.py \
  "$LOCAL_BAG" \
  "$LOCAL_MAP/trajectory.csv" \
  "$NAV_DIR" \
  --playback-rate 0.5 \
  --reference-pcd "$LOCAL_MAP/map_voxel_10cm.pcd" \
  --resolution 0.10 \
  --max-range 12.0

ln -s patrol_map_area.pgm "$NAV_DIR/patrol_map.pgm"
cp "$NAV_DIR/patrol_map_area.yaml" "$NAV_DIR/patrol_map.yaml"
cp "$LOCAL_MAP/map_voxel_10cm.pcd" "$NAV_DIR/localization_map.pcd"

python3 scripts/make_safe_nav2_config.py \
  navigation/site_fusion_20260909_001_area_v4_review/nav2_safe.yaml \
  "$NAV_DIR/patrol_map.yaml" \
  "$NAV_DIR/nav2_safe.yaml"
```

`patrol_map.yaml` 内引用的是同目录的 `patrol_map_area.pgm`，所以保留这两个原始文件。
人工查看：

```text
navigation/$MAP_ID/patrol_map_area_preview.png
navigation/$MAP_ID/traversability_diagnostics.png
navigation/$MAP_ID/traversability_summary.json
```

白色仅表示离线候选自由区，不等于已通过真机避障验收。室内第一次测试只选择实走路线
附近、空间宽阔且与黑色/灰色边缘有足够距离的位置。

## 7. 部署导航地图和启动脚本到 Jetson

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project

RUN_ID=indoor_fusion_20260912_001
MAP_ID=${RUN_ID}_area_review
NAV_DIR=$PWD/navigation/$MAP_ID
ROBOT=ysc@10.119.1.171
REMOTE_NAV=/home/ysc/lite3_patrol_project/navigation/$MAP_ID

ssh -i ~/.ssh/id_ed25519_cyberdog "$ROBOT" "mkdir -p '$REMOTE_NAV'"
scp -r -i ~/.ssh/id_ed25519_cyberdog "$NAV_DIR/"* "$ROBOT:$REMOTE_NAV/"

./scripts/indoor_nav_start.sh install
```

正式到场前可做一次不启动定位/导航节点的静态预检：

```bash
./scripts/indoor_nav_start.sh preflight "$MAP_ID"
```

## 8. 到现场一键启动重定位和导航环境

把机器狗放在地图覆盖的室内区域，站立静止，现场人员握住官方遥控器。在本地 Ubuntu
桌面终端只执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
MAP_ID=indoor_fusion_20260912_001_area_review
./scripts/indoor_nav_start.sh start "$MAP_ID"
```

脚本会按顺序完成：

1. 同步最新版运行脚本；
2. 检查地图、磁盘、Realsense 和修正后的 NDT；
3. 启动 Mid360、IMU、NDT；
   室内默认使用 `NDT_RESOLUTION=0.5`、`DOWNSAMPLE_RESOLUTION=0.2`，以减少长走廊中的航向跳变；
4. 只启动临时 Map Server，使 RViz 显示 `/map`（此时不启动 Nav2 规划器和代价地图）；
5. 打开 RViz，等待人工点击 `2D Pose Estimate`；
6. 完成 60 秒 NDT 静止检查；
7. 启动持续 NDT 防发散守卫；
8. 关闭临时 Map Server，再启动与底盘隔离的完整 Nav2；
9. 启动默认 `DISARMED` 的限速门。

脚本要求输入 `READY` 前，在 RViz 中确认实时点云和地图墙体基本重合。检查完成后，
控制台只接受 `STATUS`、`ARM`、`DISARM`、`QUIT`。

## 9. 第一次 RViz A 到 B 测试

1. 确认现场净空并全程手持官方遥控器。
2. 在 RViz 白色自由区中心选择距当前位置 1~2 米的目标。
3. 此时门为 `DISARMED`，只观察全局路径和局部代价地图，机器狗不会运动。
4. 路径不得进入黑色障碍或灰色未知区；确认后 `Cancel`。
5. 再次发送同一个目标，回到总控终端输入 `ARM`，并按提示第二次输入 `ARM`。
6. 到达后立即输入 `DISARM`。
7. 首个短距离测试成功后才逐步增加距离；不要第一次就点击地图远端。

任何点云重影、位姿跳变、路线方向错误、障碍未进入局部代价地图或人员进入场地，
都应先使用官方遥控器急停，再输入 `DISARM`。

## 10. 查看状态和正常结束

另开本地终端可查看：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
./scripts/indoor_nav_start.sh status "$MAP_ID"
```

在总控终端输入：

```text
QUIT
```

脚本会先 DISARM，再关闭本次启动的速度门、NDT 守卫、Nav2、NDT 和 Mid360。
若总控终端异常关闭，可执行兜底停止：

```bash
./scripts/indoor_nav_start.sh stop "$MAP_ID"
```

最后用 `status` 确认相关节点已经消失，且 `/cmd_vel`、`/cmd_vel_corrected` 均无发布者。
