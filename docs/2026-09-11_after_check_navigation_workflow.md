# 一键检查通过后的导航流程

适用地图：`site_fusion_20260909_001_area_v4_review`

## 为什么保留 5 分钟确认

必须保留。2026-09-10 的 NDT 不是启动时立即失败，而是在首次 `READY` 后数分钟才
发散到地图外。短时间 PASS 无法发现这一问题。

现在采用两层保护：

1. 启动前静止观察 300 秒；
2. 进入 `READY` 后持续检查里程计断流、地图越界、单帧跳变、速度异常、视觉过期及
   视觉/NDT 不一致，异常时立即撤销 `localization_ready`。

后续经过多次现场测试证明持续守卫可靠后，可以再评估缩短 300 秒等待时间。

## 运行一键脚本

### 首次使用前只部署一次

本次持续守卫代码尚未写入当前离线的 Jetson。机器狗重新联网且所有定位/导航进程已
停止后，先执行一次：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
./scripts/deploy_navigation_safety_update.sh
```

必须看到最后的 `[PASS] 部署完成`。这不是每次到现场都要执行的步骤。

### 日常现场运行

前提：相机、Mid360、视觉定位、NDT 和视觉初始化守卫已经启动，视觉初始重定位已经
完成，机器狗保持静止。

本地电脑只执行：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
./scripts/field_relocalization_check.sh
```

脚本自动完成：

- SSH、地图文件、磁盘和相机服务检查；
- 相机、Mid360、IMU、视觉位姿和 NDT 频率/断流检查；
- 300 秒持续定位稳定性检查，并每 10 秒显示进度；
- `localization_ready`、地图边界、TF、位姿跳变和静止波动检查；
- NDT 收敛率、匹配误差和内点率检查；
- 持续防发散守卫是否已部署的检查；
- 自动启动隔离 Nav2；
- 检查六个 Nav2 生命周期节点；
- 验证 `/cmd_vel` 和 `/cmd_vel_corrected` 没有意外发布者；
- 自动请求一次到已知安全航点的全局路径，仅规划、不运动；
- 自动启动速度门，并确认它处于 `DISARMED`。

任一项目出现 `WARN` 或 `FAIL`，脚本都会停止在导航放行之前。

不要用下面的方式缩短检查后继续导航：

```bash
CHECK_DURATION_S=60 ./scripts/field_relocalization_check.sh
```

短检查可用于诊断，但脚本会因为不足 300 秒而拒绝准备导航。

## 脚本出现 NAVIGATION READY 后

保持一键脚本终端打开。它负责维持 Nav2、速度门和安全控制台；关闭该终端会先
DISARM，并结束由它启动的导航会话。

另开一个本地桌面终端：

```bash
ssh -Y -i ~/.ssh/id_ed25519_cyberdog ysc@10.119.1.171
source /home/ysc/lite3_patrol_project/navigation/patrol_env.sh
rviz2 -d /home/ysc/lite_cog_ros2/nav/src/dr_nav2/rviz/dr_nav2.rviz
```

若 RViz 出现 OpenGL 错误：

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d /home/ysc/lite_cog_ros2/nav/src/dr_nav2/rviz/dr_nav2.rviz
```

在 RViz 确认：

- `Fixed Frame` 为 `map`；
- 机器人位于与现场一致的白色自由区；
- 实时点云与地图基本重合；
- 局部代价地图能显示现场障碍物。

## 第一次目标：先预演，再运动

1. 在 RViz 点击机器人附近 1~2 米的白色区域。
2. 此时速度门是 `DISARMED`，只生成路径，机器狗不会运动。
3. 检查路径不进入黑色障碍或灰色未知区。
4. 在 Navigation 2 面板点击 `Cancel`。
5. 再次点击同一个目标。
6. 回到一键脚本终端输入：

   ```text
   ARM
   ```

7. 脚本要求再次确认时，再输入一次 `ARM`。

只有服务返回 `success: true` 和 `message: ARMED` 后，机器狗才会以限速运行。

ARM 不能取消：它保证随机点击错误目标、路径预览异常或误操作时，单次点击不会直接
驱动机器狗。

## 到达目标

回到一键脚本终端输入：

```text
DISARM
```

即使速度门已经因为 Nav2 命令超时自动撤销 ARM，也要人工执行一次 DISARM。

## 后续随机点

每个新目标重复：

1. `DISARM`；
2. RViz 点击目标，只检查路径；
3. `Cancel`；
4. 再次点击同一目标；
5. 现场确认净空；
6. 在脚本终端连续两次输入 `ARM`；
7. 到达后输入 `DISARM`。

只能点击白色自由区，不能点击黑色障碍、灰色未知区、地图边缘或未经实地确认的区域。

## 结束

在一键脚本控制台输入：

```text
QUIT
```

脚本会先发送 DISARM，再关闭速度门和由它启动的 Nav2。

紧急情况始终优先使用官方遥控器急停；SSH 和脚本不能作为唯一急停手段。
