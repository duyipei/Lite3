# Site fusion visual relocalization and multi-waypoint patrol

This overlay deliberately keeps navigation output away from the real motion
topic until an operator completes the physical safety checks.

## Artifacts

- `site_fusion_20260909_001/localization_map.pcd`: 10 cm PCD for NDT localization.
- `site_fusion_20260909_001/patrol_map.yaml`: conservative 2D corridor map.
- `site_fusion_20260909_001/mission_draft.yaml`: 43-waypoint out-and-back draft. It is disabled.
- `site_fusion_20260909_001/nav2_safe.yaml`: low-speed Nav2 parameters.
- `site_fusion_20260909_001_area_v4_review/`: expanded LiDAR-observed
  traversability map. It is a review candidate and does not replace the
  conservative corridor map. Gray cells remain unknown and are not
  traversable.
- Detailed procedure: `docs/2026-09-09_visual_relocalization_patrol.md`.
- Full on-site visual initialization and RViz click-to-navigate procedure:
  `docs/2026-09-10_visual_relocalization_rviz_goal_field_test.md`.
- Indoor two-stage visual initial relocalization (start-area priority plus
  global keyframe fallback):
  `docs/2026-09-14_visual_initial_relocalization.md`.

The expanded candidate can be selected without changing the scripts:

```bash
export LITE3_PATROL_MAP_ID=site_fusion_20260909_001_area_v4_review
bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh
```

Its `navigation_area_comparison.png` overlays the previous corridor in blue
on the newly accepted free area in pale green. The candidate keeps
`mission_draft.yaml` disabled until an on-site A-to-B test has passed.

## Terminal sequence

1. Localization only:

   `bash /home/ysc/lite3_patrol_project/navigation/start_localization_only.sh`

2. Start the RGB visual localizer in a second terminal:

   `bash /home/ysc/lite3_patrol_project/navigation/start_visual_localizer.sh`

3. Start the guarded visual initialization bridge:

   `bash /home/ysc/lite3_patrol_project/navigation/start_visual_initialization.sh`

   It remains blocked until the run-specific `visual_initialization.yaml` contains a
   verified visual localizer topic and visual-map to LiDAR-map transform.
   Request initialization with `/patrol/request_visual_initialization`. It
   publishes `/patrol/localization_ready=true` only after stable visual input
   and matching NDT odometry. Manual `/initialpose` remains a fallback, not the
   normal patrol startup path.

4. Isolated Nav2 (still cannot move the robot):

   `bash /home/ysc/lite3_patrol_project/navigation/start_nav_isolated.sh`

5. Validate the mission without sending goals:

   `bash /home/ysc/lite3_patrol_project/navigation/mission_dry_run.sh`

6. Only after the on-site safety checkpoint, start the gate. It starts
   DISARMED and emits only zero commands:

   `bash /home/ysc/lite3_patrol_project/navigation/start_gate_disarmed.sh`

Do not arm or execute `mission_draft.yaml` until the map overlay, current
visual-plus-NDT pose, emergency stop, and fenced test area have been reviewed.
The velocity gate also requires `/patrol/localization_ready=true`. The mission
requires both `enabled: true` and
`review_required: false`, plus the runner's `--execute` flag.

## RViz 手动初始定位（保留原视觉入口）

在开发虚拟机的对应项目目录执行；新版和独立的 11 月 2 日副本均支持：

```bash
LITE3_OBSTACLE_BACKEND=factory_only LITE3_DYNAMIC_OBSTACLES=1 \
  LITE3_USE_VENDOR_VOA=0 \
  bash scripts/navigation_start.sh start-manual patrol_map_20261031_01_area_v2_review
```

1. 先结束其他导航会话，保持机器狗静止且不 ARM。
2. 启动器先打开静态地图和 RViz，不加载 XFeat 或视觉地图数据库。
3. 使用 **2D Pose Estimate** 在地图上拖出机器狗当前的位置和朝向。
   这是地图中 `base_link` 的人工初值，不再应用视觉/IMU 参考坐标偏移。
   RViz 通常给出平面位姿；NDT 是否能从该初值正确收敛仍需实机验证。
4. 初值经 `/patrol/manual_initialpose_request` 送到 Jetson 初始化守卫，
   经确认静止启动阶段或新鲜 DISARMED 状态检查后，才向 `/initialpose` 注入。
   输入命令采用 Jetson 接收时的 ROS 时间；NDT 扫描边界仍使用单调时间，
   不重写任何传感器或 TF 时间戳。
5. 保留 NDT 内部稳定确认、外层独立扫描确认、原地图距离/yaw 阈值，
   以及 READY 后的定位、里程计新鲜度和静止漂移保护。点选本身不会置 READY。
6. 通过确认后启动原 Nav2、目标桥和 DISARMED 速度门，再由操作者选择目标并授权。
   初始化完成后，后续 2D Pose Estimate 点选会被拒绝；需要重新定位时退出会话重启。

原视觉流程继续使用 `start`（默认）或显式的 `start-visual`；
`LITE3_INITIALIZATION_MODE=manual|visual` 也可选择模式。
手动入口只跳过视觉定位，不关闭 Factory Only 避障所需的深度相机和 VOA。

两份虚拟机项目彼此独立，但启动器仍会将运行文件同步到同一个 Jetson 项目目录。
不要同时运行两套启动器；修改开发端文件本身不会部署或重启实机。

离线测试（只使用 Mock，不启动 ROS 节点）：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_manual_initialization.py'
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_navigation_manual_mode.py'
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p 'test_rviz_ssh_bridge.py'
```
