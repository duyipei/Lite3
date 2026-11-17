当前室内 v2 地图的 CPU / GPU NDT 对照
===================================

用途与边界
----------

地图固定为 patrol_map_20261031_01_area_v2_review。
不切换正式定位后端，不启动/停止导航，不发送 ARM、目标、速度或模式指令。
capture 仅订阅点云、完整注册位姿及 READY，并只读查询定位参数。
compare 运行已封存的无 ROS 基准；每次只运行一个配准后端。

为了避免负载相互干扰，先在当前 CPU 定位 READY、静止且 DISARMED 时采样，
采样完成后由操作者正常退出导航/定位，再进行离线测速。
两个子命令均要求 --operator-confirmed-disarmed，该标志不替代运动主机状态反馈，
也不会发送 DISARM。发现业务重新启动时，只退出自己的实验，不停止业务。

准备的数据
----------

工具通过 /patrol_localization/get_parameters 核查真实运行地图、CPU NDT_OMP、
DIRECT1、NDT resolution=1.0、源和地图 leaf=0.3、fine ICP=false、UTM=false。
参数不符时明确拒绝，不能把其他地图、0.5m 旧实验当作当前 v2 对照。
结束采样时再次核查参数和地图哈希。

Foxy 的组合程序包含定位和地图两个节点；全局 __node 重映射使二者均名为
patrol_localization。参数服务的一次回复因此可能只含一个组件的声明，其他项
为 PARAMETER_NOT_SET。采样工具在最多16次/15秒的只读查询中收集两类实际值，
缺失不填默认值；同名参数冲突立即拒绝，不修改正在运行的任何参数。

/aligned_points 是未降采样的显示/障碍点云；只与相同原始纳秒时间戳的
/patrol/registration_odom 完整 XYZ/四元数配对，不使用 Z=0 的 /odom。
要求新鲜 READY，不做不同时间戳的近似配对。每个流缓存最多 8 条消息。
默认每 0.5 秒保存一帧，20 帧后停止，最长 30 秒；这不是完整 10Hz 连续回放。
同时保存地图快照、真实参数、源时间、文件哈希，目录已存在则拒绝覆盖。

准备阶段逆变换到注册坐标系，再用独立 PCL VoxelGrid 进行 0.3m 源降采样。
PCL 辅助程序在全新的结果目录编译，不安装任何文件，不重新编译正式 NDT。
逆变换经过浮点往返，可能影响恰在体素边界上的点；不声称与原始激光/IMU
预处理逐点相同。但三个后端使用完全相同的最终源点、目标地图和初始位姿。
每帧保留 CPU 已接受位姿及显式 0.20m/0.10rad 偏移两种初值，均不是定位真值。

执行命令（Jetson，操作者自行运行）
--------------------------------

先启动现有 CPU 定位流程并等待 READY，保持 DISARMED。
在另一个 Jetson 终端执行：

::

  cd ~/lite3_patrol_project
  source navigation/patrol_env.sh
  NDT_TEST_TOOL="$HOME/lite3_patrol_project/results/gpu_ndt_20261116_ksq6a947/indoor_v2_tooling_20261116/indoor_compare.py"
  NDT_TEST_CAPTURE="$HOME/lite3_patrol_project/results/ndt_v2_capture_$(date +%Y%m%d_%H%M%S)"
  python3 "$NDT_TEST_TOOL" capture --output "$NDT_TEST_CAPTURE" \
    --seconds 30 --frames 20 --operator-confirmed-disarmed

采样完成后，在原导航终端正常退出导航和定位。不要关闭上面的采样终端，
其中保留了 NDT_TEST_CAPTURE 变量。随后在采样终端执行：

::

  python3 "$NDT_TEST_TOOL" compare --capture "$NDT_TEST_CAPTURE" \
    --warmup 2 --repeats 3 --operator-confirmed-disarmed

如果业务进程仍在运行，工具会拒绝测速，不提供绕过参数。
不需要也不应为了测速启动 GPU ROS 定位节点或发送运动指令。
默认复用 /home/ysc/lite3_patrol_project/results/gpu_ndt_20261116_ksq6a947/workspace
下已经原生构建并封存的 lite3_ndt_benchmark。完整检查源码、库和正式 CPU 文件
哈希；CPU 对照仍链接原 ndt_omp 动态库，OMP=1，DIRECT1。

CPU/P2D/D2D 顺序执行，再以 D2D/P2D/CPU 复测，减少顺序/温度偏差。
没有锁定频率、改变功耗模式或限制到相同输出频率；资源利用率不等于正式导航
负载改善。GPU LM 和 CPU 优化器/收敛准则不同，不能直接比较内部目标分数。

结果
----

终端显示 align 的 P50/P95/P99/最大值、公共几何通过数、独立流程平均耗时
及离线配准等效速率 Hz=1000/平均配准ms。该速率不代表ROS实时定位输出频率。
结果保存于 workspace/indoor_v2_tests/新时间编号/：

* comparison.txt：终端表格。
* comparison.json：合并统计、每轮位姿/航向差、失败样本及资源信息。
* forward/、reverse/：原始 JSONL、stderr、tegrastats 和 CPU/GPU/RAM 快照。
* dataset/：三后端相同的源点、地图、初始位姿及哈希。
* source_filter_*.log：仅辅助点云工具的构建日志。

延迟单位为 ms，越低越快；必须同时看失败率及长尾，不能只选最小耗时。
默认20帧、两种初值、每轮3次稳定重复、正反两轮，为每后端240条稳定测量，
不是240个独立真实扫描。预热不混入稳定统计；不删除不收敛样本。
配准耗时包括必要 CUDA 同步；独立流程还包含源准备和公共几何检查。
这不是 UKF、TF、VOA 或完整导航性能/精度验收。

内存
----

::

  python3 "$NDT_TEST_TOOL" memory

该命令不创建 CUDA 上下文，只读 /proc。Xavier NX 的 CPU/GPU 共享物理 RAM，
本实验没有配置专用显存额度，没有预留“2GB/4GB给NDT”。CUDA 按需申请内存。
MemAvailable 是整机可用估计，不能当 NDT 独占 GPU 内存；RSS 也不能当显存。
基准 JSONL 中 cudaMemGetInfo 的 free/total 是 GPU 可分配共享内存的快照，
同样不是该进程独占显存计数，不与 RSS 简单相加。

离线验证范围
------------

新工具通过 Python 配对、缓存、参数、时间戳、哈希及执行隔离测试。
开发端还使用真实 PCL 编译辅助程序，验证非零 Z/roll 的逆投影、0.3m
体素质心和禁止覆盖行为。ROS 2 Foxy 的现场采样、新地图 CUDA 性能由操作者
按上述命令运行；不能把这些离线测试称为该地图已经 GPU 测速成功。
