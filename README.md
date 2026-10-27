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
| 电脑无线连接与控制 | 无线 SSH 已验证，项目连接脚本默认使用校园 Wi-Fi；ROS 2 无线通信及导航待现场验证 |

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
scripts/                 建图、导航、多点巡航入口及运行依赖
tests/                   离线回归测试（不连接机器狗）
tools/                   手动维护工具（例如停机后的时钟修复）
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
`navigation_start.sh` 用于已有环境的增量部署，**不是全新机器的一键安装程序**。
`setup.py` 的 launch 安装声明等首次部署缺口也需要处理。

首次使用请阅读 [复现补全与环境限制](docs/2026-10-10_GitHub复现补全说明.md)。
其中记录源码基础版本、本地修改、缺少的外部依赖及未完成的标定。

## 无线连接 Jetson

项目连接脚本默认使用 `ysc@10.119.1.171`（校园 Wi-Fi 固定地址，2026-10-27 更新）。
建图、导航、巡航、VOA 部署及时间同步工具均支持 `LITE3_ROBOT_HOST` 和
`LITE3_ROBOT_USER`；导航、巡航和 VOA 部署还保留 `ROBOT=user@host` 覆盖方式，
其优先级高于上述变量。

```bash
ssh -o ConnectTimeout=8 -i ~/.ssh/id_ed25519_cyberdog ysc@10.119.1.171
# 如终端曾导出旧地址，先清除覆盖；不设置变量时也使用固定地址：
unset ROBOT
export LITE3_ROBOT_HOST=10.119.1.171
```

机器狗连接地址固定为 `10.119.1.171`；旧实验记录中的连接地址以此处为准。
Mid-360 配置里的 `192.168.1.103` 是 Jetson 机内有线接口地址，继续保留；
运动主机 `192.168.1.120` 也属于机内网络。无线 SSH 连通不代表 ROS 2 DDS
跨校园网通信已经验证。

视觉初始化在 ORB 匹配失败时可使用本机 LoFTR 备用匹配。首次安装需在已有
PyTorch 的 Jetson 上执行 `bash tools/install_visual_recovery.sh`，下载依赖与
已校验的室外模型；正常定位期间不下载模型，也不上传相机图像。
独立运行备用匹配默认需要两个关键帧的几何位姿一致。导航入口可接受单个关键帧
的 PnP 位姿，但必须继续经过连续样本和独立 NDT 确认后才允许运动。
它使用附近 RGB-D 地标深度进行近似关联，仍需现场验收；缺少可选依赖时保留
ORB 流程并在日志说明原因。室内场景的备用模型尚未现场验证。

## 快速检查：不连接或驱动机器人

以下命令均在包含本 README 的项目根目录执行；需已安装测试所需 Python 依赖。

```bash
python3 scripts/verify_reproduction_bundle.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_patrol_turning.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_patrol_route.py
PYTHONDONTWRITEBYTECODE=1 python3 tests/test_patrol_route_prompt.py
bash scripts/patrol_route.sh check
```

离线测试使用模拟接口，并非实机测试。SHA-256 校验只证明归档文件完整性；
修改归档文件后需更新清单，不能把校验通过当作环境或导航安全验收。

## 虚拟机与 Jetson 文件分工

虚拟机保留完整开发项目、测试、文档、可视化工具及数据归档。Jetson 保留导航、
传感器采集和建图实际使用的程序、参数、地图、视觉库及模型依赖；历史录包和实验
产物归档到虚拟机后再清理。原厂 `/home/ysc/lite_cog_ros2`、校园网认证
`/home/ysc/network_tools`、系统服务和 CUDA/PyTorch 环境不属于项目清理范围。

Jetson 的 `patrol_nav_ws/install_merged` 通过 egg-link 引用 `build_merged` 和
`src`，三者须一起保留。FAST-LIVO 的动态库路径经过 `src/FAST-LIVO2` 目录，
也须保留这个目录层级及 `Log/PCD` 输出目录。只清理编译源码时，后续重建程序前
须从虚拟机同步完整源码；已有安装程序继续使用原路径。

2026-10-15 清理记录及完整清理前快照位于
`archives/jetson_20261015_cleanup/`。该目录已排除在 Git 外；快照包含录包、
Jetson 本地源码和构建产物，属于私有归档。`source_manifest.json` 保存逐文件
SHA-256，`verification.json` 记录归档验证结果，`cleanup_plan.json` 和
`cleanup_report.json` 记录计划与实际删除范围。当前开发文件不会被旧快照覆盖。

本次实际删除 Jetson 冗余文件 2,591 个，文件总大小约 14.70 GB，磁盘可用空间
由 40.74 GB 增至 55.45 GB。完整压缩快照约 7.07 GB，已通过逐文件 SHA-256、
压缩流 SHA-256 和 gzip CRC 校验。历史录包现在保存在虚拟机归档中；需要离线
重建旧地图时，应先从归档恢复所需录包。

维护工具 `tools/jetson_project_archive.py` 负责只读归档/校验，
`tools/jetson_runtime_cleanup.py` 只清理已验证备份中列出的文件；运行文件、
保留的符号链接目标、正在使用或备份后发生变化的文件会阻止删除。

## 可切换的 XFeat 初始重定位模式

默认 `LITE3_VISUAL_BACKEND=orb` 保留原 ORB 与可选 LoFTR 路径。
`xfeat_lighterglue` 使用官方 XFeat 和配套 LighterGlue，通过已有几何检查后
输出 `source=xfeat_lighterglue_pnp`；该模式不回退 ORB、LoFTR 或起点外观初值。
ORB（含 LoFTR）和 XFeat 均在 NDT 确认 READY 后暂停视觉计算，定位失效本身不会重新启动视觉搜索；只有显式
请求视觉初始化才恢复。运动中仍由 NDT 和腿部里程计定位，速度门独立控制。
两种模式的导航启动流程均只请求一次视觉初始化。Nav2 启动后保留已确认位姿，
观察 NDT + 里程计持续就绪 10 秒，不重新清空视觉样本或注入初值；守卫或里程计
消息停止更新时不能通过检查，定位守卫报告失效时阻止进入导航就绪状态。

室外库仍使用 `outdoor_20260921_003/visual_map.db`。原始 RGB-D 不完整时，
当前接入使用旧库稀疏深度关联，属于实验路径，不代表完整 XFeat 地图库或现场
精度已经验证。这里不生成或替换原 ORB 数据库。

首次部署（虚拟机执行，Jetson 必须可通过 SSH 访问）：

```bash
bash tools/deploy_xfeat_runtime.sh
# 默认连接固定地址 ysc@10.119.1.171
```

部署只复制必要运行代码、固定版本依赖与权重，备份旧运行文件，保留系统
PyTorch/OpenCV 和旧 LoFTR 依赖；执行参考图自匹配检查，不启动 ROS 或机器人运动。
官方源码固定为 `e92685f57f8318b18725c5c8c0bd28c7fe188d9a`，模型和依赖均校验 SHA-256。

静止测试初始定位（不启动 Nav2 或速度门）：

```bash
LITE3_VISUAL_BACKEND=xfeat_lighterglue bash scripts/navigation_start.sh relocalize outdoor_20260921_003_area_review
```

使用原模式测试：

```bash
LITE3_VISUAL_BACKEND=orb bash scripts/navigation_start.sh relocalize outdoor_20260921_003_area_review
```

模式选择适用于 `start` 和 `relocalize`；未指定时始终使用 ORB。
虚拟机模型接口检查记录在 `logs/xfeat_deployment_20261016/vm_inference.json`，
参考图自匹配只验证模型/几何接口，不能证明不同光照下的定位精度。

2026-10-16 已完成 Jetson 部署；该次记录使用的旧地址为 `10.120.76.129`。
当前执行上述命令默认连接固定地址 `10.119.1.171`。
Jetson CUDA 的 4 张参考图自匹配/PnP 检查通过，实时相机接入及 READY 暂停、
显式请求恢复检查通过。45 秒实时 XFeat 测试没有输出通过校验的 PnP 位姿，
日志中的部分候选存在有效深度对应不足；短时 ORB 对照也未输出有效位姿。
这些结果仅确认部署和运行接口，尚未完成现场初始定位验证。本次未启动导航或运动。
报告位于 `logs/xfeat_deployment_20261016/jetson_inference.json`、
`jetson_live.json` 和 `jetson_orb.json`；远端旧运行文件备份位于
`/home/ysc/lite3_patrol_project/logs/xfeat_deployment_20261016_205425`。

## 主要工作流

### 1. 室内 / 室外采集与建图

在已配置好的开发电脑执行；采集阶段由操作员使用原厂遥控器控制运动。
每次使用新的 RUN_ID，避免覆盖旧数据。

```bash
# 相机 1：D435i。省略 RUN_ID 时自动生成新的采集编号。
bash scripts/mapping_start.sh start
bash scripts/mapping_start.sh status
# 机器人静止后结束采集
bash scripts/mapping_start.sh stop
```

室内外统一使用 `scripts/mapping_start.sh`，默认批次名为 `mapping_日期时间`。
采集入口不会自动完成所有地图生成步骤；后续包括派生 bag、LIO 回放、点云处理、
视觉建库和栅格地图生成，见 [地图生成流程](docs/2026-09-12_indoor_map_generation_navigation.md)。

相机 2 的图像采集接口不等于建库功能已经完整支持。
广角/单目相机不能直接套用 D435i RGB-D 建库程序及旧内外参。

### 2. 视觉辅助定位与导航启动

只测试室内初始重定位，不启动导航和运动，在开发电脑执行：

```bash
bash scripts/navigation_start.sh relocalize indoor_fusion_20260912_001_area_review
```

静止测试与正式导航使用相同的地图定位参数：当前室内/室外地图为 4 次视觉稳定采样、NDT 内部 15 次稳定确认和外层 3 次有效扫描确认；静止测试要求就绪 10 秒。
测试结束自动关闭本次定位节点；正式导航继续使用地图配置中的采样参数。

示例使用现有室外导航地图与对应视觉库。在开发电脑执行：

```bash
LITE3_VISUAL_MAP_ID=outdoor_20260921_003 \
LITE3_VISUAL_SEARCH_STRATEGY=startup_then_global \
bash scripts/navigation_start.sh start outdoor_20260921_003_area_review
```

导航入口统一为 `navigation_start.sh`，通过地图参数选择已有地图。
等待 `NAVIGATION READY`；确认真实位置与地图一致，此时速度门仍为 DISARMED。
RViz 目标使用 `/patrol/goal_request`，不能绕过桥接直接向运动话题发指令。

室内地图 `indoor_fusion_20260912_001_area_review` 允许途中倒车；RViz 箭头表示
最终机头朝向。到达须同时满足位置误差不超过 0.35 m、朝向误差小于 0.10 rad
（约 5.7°）。末端由 Nav2 转向并继续检查机身碰撞；配置变更须重新启动导航生效。

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

## 可选动态避障模式（2026-10-22）

新增实时障碍参与全局绕行与局部避障、阻塞等待、VOA 近距离减速/停车模式。
本次仅完成本地接入与离线测试，尚未部署或进行行人、车辆实测；不包含语义识别或运动预测。

```bash
# 无需机器狗的本地配置检查
bash scripts/navigation_dynamic.sh check outdoor_20260921_003_area_review
# 连接恢复并完成私有 VOA 构建后启动；保持默认 DISARMED
bash scripts/navigation_dynamic.sh start outdoor_20260921_003_area_review
```

详见[动态避障与 VOA 接入](docs/2026-10-22_动态避障与VOA接入.md)。

2026-10-27：动态模式新增 VOA 相机障碍同时接入全局/局部 Nav2 代价地图，解决相机只触发停车而规划器看不到障碍的问题。本机修改与离线测试已完成，尚待 Jetson 在线验收；详见 [VOA 障碍接入 Nav2](docs/2026-10-27_VOA障碍接入Nav2.md)。
原 `navigation_start.sh` 默认模式与原巡航任务保留。

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
- [项目文件整理与命名对照](docs/2026-10-13_项目文件整理.md)
- [文件补全、版本与复现限制](docs/2026-10-10_GitHub复现补全说明.md)
- [更新项目到 GitHub](docs/GitHub上传指南.md)

本项目尚未声明覆盖全部原创代码的统一开源许可证；公开可见不等于任意使用授权。
第三方源码保留各自 LICENSE 和版权声明，不能用本项目的包元数据覆盖第三方许可。
公开地图和视觉图像前，请确认场地及图像数据的公开范围。
