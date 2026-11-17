Jetson 时间同步与 GPU NDT 验证记录（2026-11-16）
================================================

操作边界
--------

操作者明确确认静止、无人操作遥控器、导航已退出、DISARMED。
本轮不 ARM、不发送目标/速度、不修改运动主机或速度门，不切换默认 CPU 定位。
Git HEAD 为 5647a93201aac03a86eff926adca888ffe99edc0；保留原有 experiments/
和两份 GPU 测试文件。NDT P0/P1、VOA 和腿部 TF 核心源码保持原哈希。

时间同步：已完成系统修复及在线验证
--------------------------------

修改前，timesyncd 已启用并运行，但 Packet count=0，NTPSynchronized=no。
原配置仅指定 ntp.aliyun.com 和 time.cloudflare.com；日志持续 IPv4 应答超时。
RTC0 为 2000-01-01，系统依靠保存的旧时间启动，日期仍为 2026-10-31。
DNS 与 HTTPS 正常，不应把 IPv4 NTP 探测失败描述为所有 NTP 都被禁用。

新增清华 TUNA 后，实际通过 IPv6 地址
2402:f000:1:416:101:6:6:172 获得 stratum 2 授时。17:09:15 首次同步，
随后继续收到应答，偏差由首次约 16 天下降到微秒/毫秒量级。
17:10:05 的第二个应答 Offset=+224us、Delay=5.163ms，系统与 RTC 都正确。
这证明当前网络存在可用 IPv6 授时源；尚不能定位旧 IPv4 路径的具体丢包设备。

操作者在 Jetson 终端交互执行 sudo。未在聊天输入或保存密码。
配置备份：/var/backups/lite3-time-20261031T064201-29006。
安装文件：

* /etc/systemd/timesyncd.conf.d/90-lite3-time.conf
* /usr/local/libexec/lite3-save-rtc
* /etc/systemd/system/lite3-save-rtc.service
* /etc/systemd/system/lite3-save-rtc.timer

保留 NVIDIA 原有配置和服务；没有安装 Chrony，也没有第二个 NTP 客户端。
轮询最小 32 秒、最大 256 秒，避免原先无应答后等待 2048 秒。
RTC 保存任务仅在 NTPSynchronized=yes 时执行，不把手动改日期伪装成同步。
timesyncd 和 RTC timer 均已 enabled；RTC service 实际执行成功。
操作者随后于约 18:05 断电再开机，boot ID 从
f56be58a-993d-4098-a521-7aefdef282a8 变为
57fae2e9-5a33-431c-ba88-5b92fd806370。内核在联网校时前已从 RTC0
读取 2026-11-16T10:04:52 UTC；18:05:37 自动通过 TUNA 同步。
因此本次断电后的 RTC 保持、服务开机自启及联网同步已有实际证据。
没有执行人为断网再恢复的专项测试，不能替代这项验收。

另验证 ntp.ubuntu.com 的 IPv6 NTP 也能返回 stratum 2 应答，并加入主列表。
systemd 245 会合并多个文件的 NTP=，因此最终配置先用空 NTP= 重置旧列表，
再列出 TUNA、Ubuntu、Aliyun、Cloudflare。18:19:54 服务重启后立即同步，
消除了先等旧不可达源约 30 秒的问题。最终有效列表已用 show-timesync 核对。
备份另有 /var/backups/lite3-time-20261116T100112-35261 和
/var/backups/lite3-time-20261116T101946-6662；恢复原始配置应使用最早备份。
修复程序输出的 reboot_verified=false 只表示该单次调用不执行重启；
本报告的断电验收由操作者确认及内核、当前开机日志共同支持。

回退系统配置（在静止、DISARMED、相关业务退出后，于 Jetson 执行）：

::

  sudo python3 /tmp/lite3_time_repair_20261116_ksq6a947.py rollback \
    --backup /var/backups/lite3-time-20261031T064201-29006 \
    --operator-confirmed-safe

脚本只恢复所备份配置及服务启用状态，不把系统日期或 RTC 改回错误的旧日期。
源码永久保存在项目 tools/repair_jetson_time.py，可重新复制，先检查文件哈希。

项目启动保护
------------

scripts/check_system_time.py 不校时、不发布 ROS 数据、不调用机器人服务。
要求日期合理、时区正确、NTPSynchronized=yes，并存在本次开机产生的
/run/systemd/timesync/synchronized。只有日期看起来正常不算同步成功。

定位、视觉/手动初始化、隔离 Nav2 和速度门启动入口均在加载 ROS 环境前检查。
开发端 navigation_start.sh 同步该检查器，并在 preflight 中检查 Jetson 时间。
定位启动器记录开机 ID、系统时间和单调时间，监督自己的定位 launch PID；
若两种时钟的相对偏移变化超过 0.5 秒，检查器返回失败，由启动器原有 EXIT
清理停止自己的子进程。没有绕过 Gate，也没有修改其 0.35/0.5 秒安全期限。

Jetson 启动脚本备份：
/home/ysc/lite3_patrol_project/results/time_guard_20261116_ksq6a947/backup。
部署前确认远端原文件与当前 HEAD 内容相同；未覆盖额外远端修改。

无授时源且 RTC 不可信时，正式定位会拒绝启动，不虚构 READY。
网络恢复依赖原有 timesyncd 网络检测和有界轮询重试；实际断网恢复未验收。
本轮校时后单次消息检查：/leg_odom2 保持单调时间，源年龄约 12.8 ms；
RGB 图像保持系统时间，源年龄约 63.5 ms。未统一或改写这些不同时间域。
因此不能把此前所有 TF future 或投影断档都归因于系统日期错误。

GPU 实验范围与进度
------------------

复用 experiments/gpu_ndt，实验目录为：
/home/ysc/lite3_patrol_project/results/gpu_ndt_20261116_ksq6a947。

CPU 对照使用正式安装的 ndt_omp、DIRECT1、resolution=1.0、epsilon=0.01、
默认 maximum_iterations=35、step_size=0.1、OMP_NUM_THREADS=1。
正式 audited 程序及 CPU 基准的优化等级保持 -O1 -DNDEBUG。
GPU 使用独立复制的 fast_gicp 源码、私有动态库、CUDA 11.4、sm_72。
系统 fast_gicp、PCL、CPU audited 前缀和 ndt_realtime_current 链接不变。

原生构建只使用一个构建任务，并持续检查业务进程；若业务重新启动，
只结束实验工具自己创建的构建或基准子进程，不停止业务。
本轮补齐无 .py 后缀的 ROS Python 入口识别，例如 velocity_gate。

目标地图在同一后端进程内设置一次；源点每次新建，避免重复数据掩盖源体素开销。
GPU 明确选择 P2D/D2D；CPU 默认值及融合、质量门限不变。
目标函数和收敛条件不同，内部分数不能直接跨后端比较；独立几何验收保留。
单独的配准基准不代表 UKF、TF、锚点和 VOA 端到端验收。

目前准备的历史室内、室外数据各 6 帧、2 种初值，所有后端使用相同文件与哈希。
室外 6 帧跨约 4.30 秒，间隔不均匀；重复运行不等于连续运动数据。
另有 19 帧诊断跨 62.6 秒，间隔 2.1～7.3 秒且缺少完整源点/发布位姿配对，
不能直接当成 10 Hz 连续序列。地图、坐标和运动场景证据不足时不强行拼接。

原生构建与 Kernel：已验证
--------------------------

CUDA core、PCL 封装和 ARM64 独立基准原生构建成功。cuobjdump 证实 sm_72
cubin；运行设备为 Xavier、compute capability 7.2。P2D、D2D 均实际执行。
Nsight 捕获 P2D 导数相关 Kernel 1172 次、D2D 1488 次（各含 24 次冒烟配准、
预热及初始化；不是未跟踪性能统计口径），原始 .nsys-rep 和 CSV 已保存。

实际编译发现 Foxy ndt_omp 导出 ndt_omp::ndt_omp target，但不设置旧式
ndt_omp_LIBRARIES。已修正实验 CMake，确保 CPU 基准链接正式原库，未换算法。
同时补齐外部 CPU 动态库的独立保护清单：原节点清单没有列入此库。
当前库 SHA-256 与先前记录一致：
00318664bca0005516217ecea384261b52eeac3e1b117ce50d1e36d832695aed。
正式 CPU runtime_manifest.json 仍为原哈希，ndt_realtime_current 未切换。

室外未跟踪性能对照
------------------

同一 Jetson、地图、6 帧输入、2 种初值，正反顺序各 5 次重复。
下表每后端 120 条稳定测量，不能当成 120 帧独立连续运动数据。
单位为毫秒，配准调用含主机优化控制及必要 CUDA 同步，不是纯 Kernel 时间。

.. list-table:: 室外记录输入下的原生测量
   :header-rows: 1

   * - 指标
     - CPU NDT_OMP
     - GPU P2D
     - GPU D2D
   * - 配准平均
     - 63.918
     - 73.265
     - 86.166
   * - 配准 P50
     - 68.152
     - 80.066
     - 77.616
   * - 配准 P95
     - 133.204
     - 124.331
     - 162.288
   * - 配准 P99
     - 140.746
     - 197.044
     - 185.387
   * - 配准最大
     - 140.942
     - 269.846
     - 186.658
   * - 独立配准与公共几何检查平均
     - 103.071
     - 115.130
     - 126.380
   * - 每次进程 CPU 时间平均
     - 101.663
     - 87.246
     - 95.158
   * - 收敛/共同空间检查通过
     - 120/120
     - 120/120
     - 106/120

独立基准期间整机 CPU 忙碌率约 42.94%、41.48%、37.14%；GPU 粗采样平均
约 11.48%、24.20%、27.80%，包含后台负载。各后端没有限为相同输出频率，
不能把这组利用率直接称为正式导航的 CPU 降幅。CPU/P2D/D2D 的 RSS 峰值
分别约 167.69/186.75/187.21 MiB；NX 共享内存，不能把 RSS 与 CUDA 已用
内存简单相加。采样 CPU 温度 46～48°C、GPU 42.5～44°C。INA3221 功耗读取
需要 root，未修改权限或运行 root 基准，因此系统功耗没有实际数据。

GPU 初始化约 208～223 ms；静态地图上传调用约 18.5～20.8 ms，体素构建
约 29.9～35.0 ms。目标更新计数保持 1，并没有每帧重新上传目标地图。
完整的源准备、源上传、源体素、优化器、显式 barrier、公共几何检查和总耗时
分布在 comparison_summary.json 及各后端 JSONL 中。预处理仅是已记录源点的
复制，不是当前原始激光处理；完整定位总耗时、TF 源龄及 VOA 断档均未验证。

正确性和性能边界
----------------

首轮较少预热的室外冒烟还出现 P2D 1 次、D2D 2 次不收敛；保留失败样本，
未只统计成功结果。室内小地图重复对照：CPU/P2D/D2D 配准 P50 约
5.088/42.920/22.988 ms，收敛数分别 120/115/117（每组 120）。

室外 CPU/GPU 最大位置差约 P2D 0.447 m、D2D 0.453 m；带初始误差样本的
差异常包含约 0.24～0.41 m 的 Z 分量。部分 GPU 结果的共同几何误差更低，
不能仅凭与 CPU 不同就认定 GPU 错误，也不能据此认定 GPU 绝对正确。
现有 VOA 使用全三维注册位姿，这些差异必须进一步解释，不可通过 Z 清零、
修改时间戳或放宽门限消除。室内最大位置差约 0.72 m，同样需要独立验证。

Nsight 显示频繁主机开销：P2D 的单份追踪有 6446 次 cudaMalloc、6447 次
cudaFree、19345 次 cudaStreamSynchronize；D2D 分别为 8367、8368、25384。
这些计数包含初始化及预热；追踪计时有额外开销，不能直接用于性能表。
上游导数计算使用 Thrust transform_reduce，内核确实运行，但分配、Kernel
提交、同步、主机 LM 优化也占用时间。后续值得优先研究有界临时工作区复用，
并验证数值等价性；本轮没有扩大到 CUDA 分配器或优化器重构。

结论：P2D 比 D2D 更值得继续研究室外场景，但当前实现没有稳定降低配准时延，
P99 长尾恶化，位姿差异尚未验明。P2D 每次独立流程 CPU 时间下降约 14%，
不等于完整导航已降低负载。按任务的条件验收要求，停止进入实际定位集成。
完整 GPU 定位仅生成隔离源码，未构建/加载 ROS 定位节点，未启动视觉初始化、
VOA、Nav2 或速度门，未发布正式 TF/odom/cmd_vel。默认仍是正式 CPU audited。

没有伪造长连续运动数据，也没有运动测试。当前不满足替换 CPU 的条件。

本轮分析与测试原始记录
----------------------

开发端临时目录：/tmp/lite3_time_gpu_20261116_ksq6a947。
含初始工作区哈希、IPv4 NTP 探测、HTTPS Date 对照、实际消息时间域检查、
JUnit 测试报告，以及原有未跟踪 GPU 文件的可验证备份。
UKF 测试的结果文件重定向到该目录，未覆盖旧实验结果。

最终测试与交付
--------------

新增62项离线测试通过；包含既有回归在内最终392项通过、2项跳过、0失败。
GPU专项共130项，仍属于Mock/静态/配置测试；机载原生运行和Nsight证据单列。
两个跳过的旧基线精确对照需要 LITE3_NDT_BASELINE_HEADER，当前没有该旧头文件。
Python语法、Shell语法和 git diff --check 均通过。

原始结果与完整修改差异永久保存于：
/home/duyipei/Lite3_GPU_NDT_results/20261116_ksq6a947

其中 comparison_summary.json 为结构化对照，native_results/workspace/runs 为
未跟踪JSONL与资源记录，native_results/workspace/traces 为真实Kernel追踪。
implementation.diff 包含本轮新增文件和原先未跟踪文件的增量比较；
change_inventory.json 提供逐文件新增/删除行数。未创建分支、提交或推送。

本轮结束不启动新GPU/ROS实验。正式默认仍是 audited CPU，通常不需要任何
“切回”动作；ndt_realtime_current 仍指向 ndt_realtime_20261112_7iwl6ku。
