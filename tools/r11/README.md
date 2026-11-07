# R1.1 原地旋转控制链路审查与采样

本轮只在开发端审查、准备工具与离线测试。未连接 Jetson、未部署、未运行 ROS 业务节点、未发送目标/速度/ARM/模式指令。

## 结论与证据边界

目前不能确定单一根因。现有证据证明 Nav2 曾生成纯旋转，Velocity Gate 曾放行正反纯角速度；未找到足以修改业务逻辑的角速度过滤或映射错误。因此本轮不改任何业务源码或导航参数。优先补测 MotionSender 接收/UDP 出口、官方自主模式、实际 yaw 响应；同时保留已有 Nav2 碰撞拒绝证据。

三个状态必须分开：遥控器设置的官方自主模式；Velocity Gate 软件 ARM；Nav2 当前有效速度。ARMED 不是运动主机模式确认。当前项目没有用已验证的模式 ACK 阻止/允许 ARM。

### 已有实机录包的离线复核

数据：`/home/duyipei/Lite3_T4_results/cloud_costmap_20261107_143213_4916/raw_capture/bag`。只取第三个目标结束 `1793425855.169304752` 及以前，排除之后用户确认的遥控移动。

| 数据 | 纯旋转条数 | 正向 | 反向 | angular.z 范围 |
|---|---:|---:|---:|---|
| `/patrol/cmd_vel_raw` | 36 | 28 | 8 | -0.106667～0.400000 rad/s |
| `/cmd_vel_corrected` | 127 | 99 | 28 | -0.106667～0.400000 rad/s |

31 条纯旋转输入在其后 150 ms 的观察窗口内找到同角速度输出。其余 5 条不能据此认定被滤掉：两个话题接收排序、队列及采样缺失尚未完全区分。输出20 Hz、输入约5 Hz，条数不应一一相等。已有 `/odom` 的一些纯旋转片段出现正反 yaw 变化，但没有同期 IMU 与 UDP，不能把 NDT yaw 当成独立的真实旋转验收。

之前用户已确认第三个点后用遥控器移动；该段 `odom_speed_1.395mps` 不再用于证明自主原地旋转失败或定位自行漂移。

## 源码调用链

| 环节 | 文件/函数 | 核查结果 |
|---|---|---|
| Factory Only 参数 | `scripts/make_dynamic_nav2_config.py::build()` | 基线 min_vel_x=-0.08，生成配置提升至0；max_vel_y=0，不排除纯转；不改最大角速度 |
| 导航速度隔离 | `patrol_navigation.launch.py` 的 `GroupAction/SetRemap` | Controller 与 Recovery 的 `/cmd_vel` 统一重映射 `/patrol/cmd_vel_raw` |
| 目标 | `scripts/goal_pose_bridge.py::goal_callback/send_navigation_goal` | PoseStamped 的方向传给 NavigateToPose，没有只保留目标XY；pending/active 由桥发布 |
| 输入 | `velocity_gate.py::raw_callback()` | 检查 x/y/w 有限，保存消息和单调接收时间，不要求x或y非零 |
| 输出 | `VelocityGate::timer_callback()` | ARMED+active+健康+新鲜才放行，w独立限幅至±0.4，无角速度死区或角加速度平滑 |
| 保护 | `sensor_health/trip/arm_callback()` | VOA 0.5s、定位、传感器、发布者冲突、锁存原样保留；raw超过0.5s只输出零，恢复等待有界 |
| 官方参考 | `MotionSender::CmdVelCallback()` | 先发320=x，再325=y，再321=-w，三个20字节UDP；零平移不跳过角速度 |
| 运动主机 | 当前无内部源码/实机验证 | 未证明实际SDK版本、协议接收/模式ACK、步态对低速纯转的执行能力 |

官方参考来源：

- Lite3 ROS `ros2-foxy` 固定提交 [`e4ab5dc1c7f3ff46274162a1ef302253936578aa`](https://github.com/DeepRoboticsLab/Lite3_ROS/tree/e4ab5dc1c7f3ff46274162a1ef302253936578aa)。[MotionSender](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/e4ab5dc1c7f3ff46274162a1ef302253936578aa/src/transfer/src/Jetson2Motion.cpp#L314)、[协议](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/e4ab5dc1c7f3ff46274162a1ef302253936578aa/src/transfer/include/protocol.hpp#L28)、[App](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/e4ab5dc1c7f3ff46274162a1ef302253936578aa/src/transfer/src/Jetson2App.cpp#L36)。
- Nav2 `foxy-devel` 固定提交 [`0c9917ffc3cd53175a066b87659ea3920e3a4f20`](https://github.com/ros-navigation/navigation2/tree/0c9917ffc3cd53175a066b87659ea3920e3a4f20)。[XYThetaIterator](https://github.com/ros-navigation/navigation2/blob/0c9917ffc3cd53175a066b87659ea3920e3a4f20/nav2_dwb_controller/dwb_plugins/src/xy_theta_iterator.cpp#L95)、[RotateToGoal](https://github.com/ros-navigation/navigation2/blob/0c9917ffc3cd53175a066b87659ea3920e3a4f20/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp#L79)、[GoalChecker](https://github.com/ros-navigation/navigation2/blob/0c9917ffc3cd53175a066b87659ea3920e3a4f20/nav2_controller/plugins/simple_goal_checker.cpp#L84)。

这些是固定版本的公开参考，不是本轮证明的 Jetson 安装版本。源码文件哈希一致也不能单独证明二进制由它构建。

### Nav2 参数与三种旋转

V2 `nav2_safe.yaml` 通过 Factory Only 生成器得出：

| 参数 | 数值/含义 |
|---|---|
| min_vel_x/min_vel_y/max_vel_y | 0 / 0 / 0 |
| max_vel_theta | 0.4 rad/s；Foxy角速度采样区间有正负方向，不依赖另设正的min_vel_theta |
| min_speed_xy/min_speed_theta | 0.03 m/s / 0.1 rad/s；Foxy联合最小速度规则用AND，不是分别禁止零平移 |
| acc_lim_theta/decel_lim_theta | +0.5 / -0.5 rad/s² |
| vtheta_samples | 31 |
| sim_time | 2.5 s预测时域，不是180°旋转总时间上限 |
| controller_frequency | 5 Hz配置值，已有日志有超期记录 |
| RotateToGoal | 已启用，scale32、slowing_factor5、lookahead_time=-1 |
| RotateToGoal xy窗口 | 0.2 m；近目标且已停止平移时拒绝非纯旋转候选 |
| GoalChecker | xy0.35 m、yaw0.2 rad（约11.46°）、stateful=false |

DWB 路径跟踪能够产生纯转，但候选仍可能被碰撞/振荡等critic拒绝。现有业务BT只有Wait恢复；生成器明确只保留wait插件，未启用Spin。不能把“等待时不主动转圈”判为Spin故障，也不应直接发独立Spin Action绕过goal_active约束。

180°朝向目标应由操作者在同一当前位置选择目标箭头朝向；它走NavigateToPose，和Spin恢复不同。5°等小目标可能已在11.46°容差内，测试从±30°起。最大0.4 rad/s下180°单是理想恒速就需约7.85s，还需计入加减速、规划和容差。

### MotionSender与官方模式

公开参考订阅`cmd_vel`与`cmd_vel_corrected`，Best Effort、depth1、同一个回调；同时订阅simple_cmd/complex_cmd。三轴无显式限幅、死区或超时定时器，角速度数值取反后传输，没有代码级角度单位换算。负号是官方既有转换，不能未经坐标验证直接改成正号。

项目未启动/重启厂家transfer服务，MotionSender由已有部署管理；导航脚本不保证其已实际接收速度。项目禁止公共`/cmd_vel`发布者并限制corrected唯一发布者；官方sender的第二入口和simple/complex仍是需核对的潜在外部控制通道。当前项目的门状态并未确认运动主机模式，也未验证simple/complex不存在其他发布者。不能把存在这些入口直接等同于当时有竞争输入。

公开RobotState有basic/gait/policy/motion等原始字段，但本轮未找到可信的ROS模式ACK或枚举语义。App源码中的navigation布尔变量不能当作硬件自主模式反馈。工具不会把ARMED、handle_state、节点存在或操作者标记转换成`autonomy_confirmed=true`。

## 离线测试

开发端执行（不启动ROS节点）：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p test_r11_rotation.py -v
```

新增30项包括：实际VelocityGate函数配Fake ROS；正/负/极小角速度、限幅、ARM、撤销、VOA超期、定位失效、raw超时、锁存恢复；官方固定回调配Mock sendto的C++测试；Foxy实际一维采样器的离线C++测试；PCAP解析与诊断工具安全测试。C++临时产物只在临时目录，不编译项目、不联网、不建socket。

已执行既有回归：Factory Only 17/17通过；turning 16/16通过；vendor_voa_gate 15/16通过。唯一失败为既有`test_two_navigation_sources_rejected_before_arm`仍期待两个raw发布者冲突，而当前源码允许controller与recoveries两个、拒绝第三个；该过时断言和业务逻辑均未修改。Factory Only已有专门的“两者允许、第三者拒绝”测试通过。

Mock/参考代码执行不证明Jetson固件能完成旋转。Foxy实机集成、UDP及官方模式检查本次未执行。

## 下一轮实机采样准备（本轮不执行）

仅需将`tools/r11/capture_rotation.py`与`rotation_audit.py`放到Jetson项目的`tools/r11/`；部署另行授权。它们不改任何业务模块。先由操作者确认宽敞平地、站立稳定，使用既有遥控器与急停手段。所有运动仍由操作者和既有导航终端/Velocity Gate完成。

### 1. 只读部署与端点核查

在Jetson手动执行，使用项目现有环境：

```bash
cd /home/ysc/lite3_patrol_project
source navigation/patrol_env.sh
python3 tools/r11/capture_rotation.py inspect --output /tmp/lite3_r11_preflight
```

工具读取进程exe/hash/命令行/ROS环境白名单、transfer服务和UDP socket信息、ROS端点及QoS。不会调用控制服务。若已找到实际SDK源码，可附`--motion-source /实际路径/Jetson2Motion.cpp`。当前ROS shell overlay不一定是已运行transfer进程的overlay，需对照报告。

从实际部署文件或只读参数查询确认target_ip、target_port；不要直接把官方默认IP当作实际地址：

```bash
ros2 param get /motion_sender target_ip
ros2 param get /motion_sender target_port
```

这两个是操作者未来执行的只读参数查询，不切换任何模式。本工具不会自动切换网卡/DDS、重启节点或安装依赖。

### 2. 开始一次限时记录

将下面IP和interface替换为核实的运动主机地址和通信接口；不是SSH地址。tcpdump必须事先可用且当前用户已有被动抓包权限。工具不sudo、不setcap、不安装软件。缺少权限会在启动ROS录包前报错。

```bash
R11_PEER='核实后的运动主机IPv4'
R11_IFACE='核实后的网卡名称'
python3 tools/r11/capture_rotation.py record \
  --output /tmp/lite3_r11_turn_01 --duration 120 \
  --udp --interface "$R11_IFACE" --motion-peer "$R11_PEER"
```

端口若不同，附`--motion-port 实际端口 --state-port 实际反馈端口`。输出目录必须是/tmp下的新目录。默认命令端口43893、反馈端口43897只是公开协议参考，须确认。`record`使用已验证的C++rosbag接口并保持自动话题发现，确认实际写入后打印`VERIFIED_ROS_RECORDING`；不要把文件存在当作录制成功。

录制限30～180秒、诊断进程nice10、持续CPU超预算自动结束自身记录。每2秒保存计数和时钟样本，每10秒打印摘要。不会停止业务节点。采集开销仍需实测，不能保证无调度影响。

不带`--udp`可记录ROS，但报告明确为部分结果，不能判断MotionSender UDP发送。传感器消息均以原Header保存；PCAP与bag接收时间是墙上时间，额外记录同机单调/墙上时间及boot_id配对，不能把不同时间域直接相减。

### 3. 操作者执行旋转步骤

1. 启动既有导航流程，保持默认DISARMED。操作者通过遥控器手动进入官方自主模式；软件不模拟此动作。
2. 另一个终端记人工观察，**该命令只写标记，不是模式切换或模式ACK**：
   `python3 tools/r11/capture_rotation.py mark --output /tmp/lite3_r11_turn_01 --step official_auto_enabled_by_operator`
3. 地图位置/朝向、感知和定位健康后，在RViz选择尽量相同的当前位置，只将目标箭头转正向约30°；操作者按既有导航终端二次确认ARM。工具不发目标或ARM。
4. 记`--step positive_small`；完成后再用当前姿态的反向约30°目标，记`negative_small`。
5. 各步稳定后，由操作者依次选择90°、180°目标，分别记`yaw90`、`yaw180`。不要通过2D Pose Estimate重置真实yaw反馈。
6. 同一ARM会话后续目标可能直接执行，逐步确认现场安全，勿同时排队。保护触发时按既有流程由操作者处理，工具不自动CLEAR/ARM、不延长超时。
7. 结束记`test_finished`，授权退出由操作者完成。记录器到时只结束自己的两个诊断PID。

`mark`都可加`--note '方向/现场观察/是否实际转动'`。操作者应描述实物是否转动；NDT yaw不是独立真值。

### 4. 采集内容与仍缺少的数据

| 观测 | 记录方式 | 边界 |
|---|---|---|
| raw/corrected、公共cmd_vel | ROS bag Twist | 确认Nav2输出与门输出，不等于MotionSender实收 |
| ARM、gate原因、目标状态/原目标朝向 | ROS bag | 可区分idle/active、故障锁存、目标的yaw |
| odom、leg_odom2、IMU | ROS bag | 分别比较源时间/坐标系与yaw，不能混用rpy_vel的未知单位 |
| MotionSender回调进入次数 | transfer journal已有日志（若当前版本会打印） | 不能从仅linear.x日志补出实际收到的angular.z |
| 运动UDP出口 | tcpdump被动PCAP | 统计320/325/321、角速度数值；不等于固件收到/执行，也不能仅凭PCAP证明PID归属 |
| 原始反馈UDP | 同一PCAP | 原样保留，不猜测模式字段枚举，不当作自主模式ACK |
| 官方自主模式 | 操作者标记 + 待核实反馈接口 | 目前`autonomy_confirmed=null` |

**目前不能无侵入获得MotionSender每次回调的实际angular.z。** 公共参考只打印callback进入和linear.x，工具据实标“unmeasured”。如果ROS输出正常而没有相应UDP，下一阶段必须先核实实机SDK，再单独授权在回调入口与sendto返回处增加限量记录；本轮未改SDK、未生成虚假回调数据。BPF只观察流量，不创建UDP运动socket。

### 5. 收集与离线分析

待采样结束，按既有文件传输方式复制整个/tmp采样目录到开发端；本工具不含SSH、不自动传输或删除原始数据。复制前后比较文件大小与SHA-256，SQLite消息数与metadata一致才宣称文件完整。禁止把含速度记录的bag无筛选接入实机回放。

开发端：

```bash
source /opt/ros/galactic/setup.bash
python3 tools/r11/rotation_audit.py /本地采样目录/bag \
  --pcap /本地采样目录/motion_udp.pcap \
  --motion-peer '已核实的运动主机IPv4' \
  --output /tmp/lite3_r11_analysis_01
```

分析只反序列化文件，不初始化ROS节点。支持经典PCAP Ethernet/Linux cooked v1/v2 IPv4 UDP；其他链路或PCAPNG报错，不默认为零丢包。UDP解析是固定公开参考格式（pack4、3×int32+double），与实际SDK/固件格式一致性仍需核实。

输出`rotation_report.json`和`udp_decoded.json`。包含正反纯转计数、raw到corrected观察匹配、门状态转换、纯转片段与各yaw反馈变化、UDP转向包统计。未测量T1、MotionSender回调或模式ACK不填零，不把UDP出口等同运动主机确认，不把观察不匹配直接判为门过滤。yaw片段使用bag接收窗口近似对齐；源Header仍在原包中，回调排队时不能把该近似当作精确的控制响应时延。

## 故障定位矩阵与下一步决策

| 观察 | 优先定位 | 不能据此断言 |
|---|---|---|
| raw无w，Nav2报无有效轨迹 | 生成/critic/目标/状态 | 不能证明底盘不支持旋转 |
| raw有w而corrected零，门有故障 | 对应ARM/感知/定位/raw时效 | 不允许绕过门 |
| corrected有w，无UDP | MotionSender订阅、实际版本、调度/发送错误 | ROS旁路订阅不是实际callback |
| UDP有参考转向值，实体不转 | 官方模式、接收端/固件、低速执行 | 单凭出口不能证明运动主机已收 |
| 实体转而odom不转 | NDT/反馈时间/坐标 | 不能盲改MotionSender符号 |
| IMU/leg/odom yaw互相矛盾 | 反馈发布者、时间、单位、定位 | 本轮不改NDT/腿部TF |

本轮不修改Footprint、Padding、Inflation、VOA/STVL/NDT/腿部TF、最大角速度、安全期限或ARM逻辑。新增内容仅用于获取下一阶段证据，等待用户审查后再决定部署与实机测试。
