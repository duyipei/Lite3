# Lite3 校园自主巡检

基于云深处绝影 Lite3 激光版的校园自主巡检研究项目，包含传感器采集、激光建图、
视觉地图构建、视觉辅助初始定位、激光定位、单点导航和多点自动巡航。

> **状态更新：2026-10-10。** 本仓库是实机适配项目，不是开箱即用的机器人系统。
> 单点导航已实机验证；多点自动巡航尚未完成整条路线验收。
> 默认禁止运动，执行任务需要现场确认并显式 ARM。

## 项目进展

| 功能 | 当前状态 |
| --- | --- |
| Mid-360、IMU、D435i 接入与同步数据采集 | 已开展实机采集验证 |
| 激光雷达与 IMU 建图、二维导航地图生成 | 已生成室内和室外地图 |
| RGB-D 视觉关键帧数据库 | 已构建，可用于现有相机的视觉初始化 |
| 视觉辅助初始定位与 NDT 联合确认 | 已开展实机验证；起点外观候选不等同于 PnP 成功 |
| 单点 A–B 导航 | 已在室外测试成功 |
| 多点自动巡航 | 已实现逐点调度、显式授权、任务心跳及失败停止；整条路线待验收 |
| 转向与路径跟随优化 | 已完成参数修正和离线回归，待实机验证 |
| 广角相机视觉重定位 | 计划重新标定、采集、建库并测试，尚未完成 |
| 电脑无线连接与控制 | 待完善网络配置与现场验证 |

当前建图主流程是激光雷达与 IMU 的 LIO 建图，配合 RGB-D 视觉建库。
不能将保存的完整 LIVO 配置理解为已完成视觉、激光、IMU 全融合标定与验收。

## 系统结构

```text
开发电脑 / Ubuntu 虚拟机
  ├─ 采集、部署和任务脚本
  ├─ RViz 地图显示与目标设置
  └─ 多点巡航执行器
              │ SSH / ROS 2 通信
              ▼
Jetson：传感器、定位与导航
  ├─ Mid-360 + IMU → LIO 建图 → PCD / 二维地图
  ├─ RGB-D → 视觉数据库 → 初始位姿候选 → NDT 确认
  └─ 目标桥 → Nav2 → /patrol/cmd_vel_raw
                           │
                    安全速度门（默认 DISARMED）
                           │
                    /cmd_vel_corrected
                           ▼
              原厂 motion_sender → UDP → 运动主机
```

项目输出机体速度，不直接控制腿部关节或替换原厂步态控制器。
RViz 全局路径只是规划信息，仍需局部控制器完成避障和速度生成。

## 目录结构

```text
scripts/                 建图、导航、多点巡航入口及离线测试
config/                  雷达网络、采集和 LIO 配置
calibration/             历史内参及标定状态记录
navigation/              地图、导航参数、任务路线、Jetson 启动脚本
patrol_nav_ws/src/        ROS 2 视觉定位、初始化守卫、速度门、巡航执行器
fast_livo2_ws/            从实机补回的建图源码、依赖源码与第三方许可证
vendor_patch/            原厂 NDT 定位相关修改
results/visual_maps/     视觉关键帧数据库
docs/                    实验记录、操作说明、复现与上传指南
reproduction_manifest.json  归档文件 SHA-256 清单
```

原始 bag、运行日志、编译产物与凭据不属于 Git 上传内容。
`ros_ws/FAST-LIVO2` 是历史源码片段；完整建图工作空间快照位于 `fast_livo2_ws`。

## 环境与复现前提

- 机器人：Lite3 激光版，Jetson 计算主机与原厂运动主机。
- 实测 Jetson 环境：Ubuntu 20.04.6 / aarch64 / L4T 35.4.1 / ROS 2 Foxy。
- 当前开发端 RViz 流程：Ubuntu 虚拟机 / ROS 2 Galactic。
- 外部依赖：原厂 Nav2/STVL、定位包、Mid-360/RealSense 驱动、运动桥与服务。
- Python 功能按模块依赖 ROS 2、NumPy、OpenCV、PyYAML、Pillow、Matplotlib 等。

脚本仍包含设备相关路径、用户和网络地址，换设备需适配。
`indoor_nav_start.sh` 用于已有环境的增量部署，**不是全新机器的一键安装程序**。
`setup.py` 的 launch 安装声明等首次部署缺口也需要处理。

首次使用请阅读 [复现补全与环境限制](docs/2026-10-10_GitHub复现补全说明.md)。
其中记录源码基础版本、本地修改、缺少的外部依赖及未完成的标定。

## 快速检查：不连接或驱动机器人

以下命令均在包含本 README 的项目根目录执行；需已安装测试所需 Python 依赖。

```bash
python3 scripts/verify_reproduction_bundle.py
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_patrol_turning.py
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_patrol_route.py
PYTHONDONTWRITEBYTECODE=1 python3 scripts/test_patrol_route_prompt.py
bash scripts/patrol_route.sh check
```

离线测试使用模拟接口，并非实机测试。SHA-256 校验只证明归档文件完整性；
修改归档文件后需更新清单，不能把校验通过当作环境或导航安全验收。

## 主要工作流

### 1. 室内 / 室外采集与建图

在已配置好的开发电脑执行；采集阶段由操作员使用原厂遥控器控制运动。
每次使用新的 RUN_ID，避免覆盖旧数据。

```bash
# 相机 1：D435i。省略 RUN_ID 时自动生成新的采集编号。
bash scripts/outdoor_lidar_visual_mapping.sh start
bash scripts/outdoor_lidar_visual_mapping.sh status
# 机器人静止后结束采集
bash scripts/outdoor_lidar_visual_mapping.sh stop
```

室内采集使用 `scripts/indoor_lidar_visual_mapping.sh`。
采集入口不会自动完成所有地图生成步骤；后续包括派生 bag、LIO 回放、点云处理、
视觉建库和栅格地图生成，见 [地图生成流程](docs/2026-09-12_indoor_map_generation_navigation.md)。

相机 2 的图像采集接口不等于建库功能已经完整支持。
广角/单目相机不能直接套用 D435i RGB-D 建库程序及旧内外参。

### 2. 视觉辅助定位与导航启动

示例使用现有室外导航地图与对应视觉库。在开发电脑执行：

```bash
LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
LITE3_VISUAL_SEARCH_STRATEGY=startup_then_global \
bash scripts/indoor_nav_start.sh start outdoor_20260921_003_area_review
```

虽然入口保留 `indoor` 命名，但支持显式指定室外地图。
等待 `NAVIGATION READY`；确认真实位置与地图一致，此时速度门仍为 DISARMED。
RViz 目标使用 `/patrol/goal_request`，不能绕过桥接直接向运动话题发指令。

单点操作见 [导航操作说明](docs/outdoor_navigation_operator_guide.md)。
历史记录中的旧参数不覆盖当前代码；最新多点转向说明见 [scripts 说明](scripts/README.md)。

### 3. 多点自动巡航

保持导航终端运行，另开开发电脑终端，进入同一项目根目录：

```bash
bash scripts/patrol_route.sh run \
  outdoor_20260921_003_area_review \
  mission_recovered_20261008.yaml
```

任务包含 11 个导航点。输入 ARM 授权本次执行后才会尝试解锁；
默认逐点超时 120 秒，失败停止，全部完成后锁定。
该任务仍是草稿；一次执行授权不代表路线已认证。
运行期间不要另发 RViz 目标，现场须持遥控器准备急停。

## 安全与已知限制

- 保留足迹碰撞检测、定位守卫、速度隔离和显式 ARM，不以关闭避障解决无法通行。
- 当前室外导航最大前进速度 0.15 m/s，最大角速度 0.40 rad/s。
- 原厂 VOA 与项目速度门不能同时竞争运动输出；服务切换必须经现场确认。
- 多点任务失败不自动重新解锁。新增低速转向无进展保护，仍需实机验证其适用性。
- 场景重复、相机朝向、光照和外参误差会影响视觉重定位；NDT READY 也不是绝对定位正确的保证。
- 原始数据未随仓库完整提供，复用现有地图与从原始观测重建地图是不同的复现层级。

## 下一步工作

1. 使用广角摄像头重新标定、采集并构建视觉地图，测试视觉重定位效果。
2. 完善机器狗联网与电脑端无线连接、控制。
3. 复测转向与路径跟随修正，完成多点自动巡航全流程验证。
4. 逐步开展巡检目标识别与任务联动测试。

## 文档与第三方代码

- [脚本入口与转向调试说明](scripts/README.md)
- [文件补全、版本与复现限制](docs/2026-10-10_GitHub复现补全说明.md)
- [更新项目到 GitHub](docs/GitHub上传指南.md)

本项目尚未声明覆盖全部原创代码的统一开源许可证；公开可见不等于任意使用授权。
第三方源码保留各自 LICENSE 和版权声明，不能用本项目的包元数据覆盖第三方许可。
公开地图和视觉图像前，请确认场地及图像数据的公开范围。
