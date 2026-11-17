Lite3 机载 GPU NDT 独立实验
========================

本目录不被正式 navigation_start.sh 引用。默认仍为 NDT_OMP。
prepare.py 只生成新的实验工作空间，不编译、不 SSH、不部署、不切换正式版本。
本轮 Jetson 仅执行环境、文件、符号和 cubin/PTX 的只读检查。

审查结论（2026-11-14）
--------------------

条件可行。Jetson 的 L4T 为 R35.4.1，nvcc 为 CUDA 11.4.315；
fast_gicp 已 BUILD_VGICP_CUDA=ON，libfast_gicp.so 确实导出
NDTCuda<PointXYZI,PointXYZI>，libfast_vgicp_cuda.so 提供 CUDA 实现。
实际链接的 PCL 为 1.10，ABI 符号使用 boost::shared_ptr；实验使用 PCL
基类 Ptr 构造派生对象，同时测试 boost/std 两种指针布局。
已有库仅发现 sm_52 cubin 和 PTX。PTX 可供 Volta 前向 JIT；这不是 sm_72
原生构建或 Kernel 已成功执行的证明。可选的独立 fast_gicp 源码复制会添加
compute_72/sm_72 和 compute_72 PTX，不覆盖机载安装库。
开发 VM 没有 CUDA Toolkit；完整 ROS/PCL/GPU 构建和原生 CUDA 运行需另行授权。
实机 Eigen 3.3.7、PCL 1.10.0；未安装 nvidia-jetpack 元包，JetPack 的完整包版本
没有直接确认。实际动态链接的 CUDA core 与安装库 SHA-256 相同。

公开依据：

* fast_gicp，审查 commit 0e7ec1441c99f7be453db2ea216d5de029387417：
  https://github.com/koide3/fast_gicp/tree/0e7ec1441c99f7be453db2ea216d5de029387417
  ndt_cuda.hpp、ndt_cuda_impl.hpp、ndt_cuda.cu、lsq_registration_impl.hpp。
* CUDA 11.4 Volta 兼容说明：
  https://docs.nvidia.com/cuda/archive/11.4.4/volta-compatibility-guide/index.html

实现边界
--------

GPU 包括目标体素、P2D/D2D 对应关系和导数等核心计算；优化器控制、数据转换、
主机传输、UKF、腿部预测、质量检查及 ROS 发布仍在 CPU。GPU P2D 使用源点对
目标分布；D2D 使用源/目标体素分布，另有每帧源体素构建成本。GPU 的 LM、
停止条件及原始目标分数与 CPU NDT 不同，不能直接比较内部目标分数。
外部 XYZ/L2 误差、有效范围、重叠率和所有现有定位安全检查保留。

实验生成器从当前工作区覆盖 NDT 源码，保留扫描双槽、同帧 TF、源龄检查、
UKF 事务验收、三维融合、状态/锚点/TF/odom 顺序和最后的显示点云。
仅替换配准器工厂，在等待传感器 TF 之前增加 CUDA 检查。全局重定位用的
DeltaEstimater 保持 CPU，不再初始化第二个 GPU 配准器。GPU 异常直接传播，
停止该定位进程；不静默回退 CPU，不发布未确认定位。

同一目标指针不会再次上传；目标内容变化时沿用原有 GlobalMapCallback 更新。
CUDA core 对目标体素缓存，源点每帧上传；D2D 每帧重建源体素。上下文不重置。
运行仍有同步等待，不能声称消除了 ROS 回调占用或保证 CPU 下降。

生成与构建（未来单独授权后在目标 CUDA 主机执行）
-------------------------------------------------

先准备完整 audited hdl_localization 源码快照。prepare.py 会覆盖当前 vendor_patch
中的全部已验证修复，绝不从远程 GitHub 覆盖项目。输出目录必须不存在。

::

  python3 experiments/gpu_ndt/prepare.py \
    --base-package /path/to/audited/ws/src/hdl_localization \
    --output /tmp/lite3_gpu_ndt_ws_unique

如要独立重建 sm_72 依赖，加 --fast-gicp-source /path/to/complete/fast_gicp。
该目录必须包含 Eigen/nvbio 等原工程依赖；不自动下载或安装。
下面只是在新工作空间构建；所有前缀保持独立，不修改 ndt_realtime_current：

::

  cd /tmp/lite3_gpu_ndt_ws_unique
  colcon build --executor sequential --packages-select hdl_localization \
    --cmake-args -DLITE3_GPU_NDT_ENABLED=ON -DBUILD_TESTING=OFF \
    '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG'

若同时复制了 fast_gicp，应使用 --packages-up-to hdl_localization 并加
-DBUILD_VGICP_CUDA=ON。CPU 对照使用另一个新输出目录且
-DLITE3_GPU_NDT_ENABLED=OFF。与当前 CPU 构建采用相同优化等级、OMP 和分辨率。
不更改依赖的编译优化等级来混淆后端收益。CUDA 不可用时显式配置/链接失败。

同数据基准
----------

dataset.py 接受已有 frame_*.npz/report.json，保留原始源时间标识。
它用完整 published xyz/quaternion 的逆将录制的 aligned 点还原到配准坐标系，
不会猜测相机/IMU 外参。它不是当前原始扫描预处理回放，不能当绝对真值。
每帧有 recorded_seed 和显式偏移 0.20 m/0.10 rad 两种初值。
不再次降采样已预处理的源点；地图 leaf 必须显式提供且各后端相同。
生成的数据集使用相对扫描路径，可整体复制到另一台主机；复制后仍检查全部哈希。
样本与其历史日志明确记录的地图配对，不按结果目录的模糊名称推断地图。
start_ndt_baseline 属于旧室内地图；ndt_geometry_final_20261027 实际属于室外地图。

::

  python3 experiments/gpu_ndt/dataset.py \
    --capture results/indoor_ground_validation_20261027/start_ndt_baseline \
    --map navigation/indoor_fusion_20260912_001_area_review/localization_map.pcd \
    --map-leaf 0.5 --resolution 1.0 --output /tmp/ndt_dataset_unique
  python3 experiments/gpu_ndt/run.py \
    --binary /tmp/lite3_gpu_ndt_ws_unique/install/hdl_localization/lib/hdl_localization/lite3_ndt_benchmark \
    --dataset /tmp/ndt_dataset_unique --output /tmp/ndt_comparison_unique
  python3 experiments/gpu_ndt/analyze.py /tmp/ndt_comparison_unique

室外同数据实验可将 capture 改为 results/ndt_geometry_final_20261027，地图改为
navigation/outdoor_20260921_003_area_review/localization_map.pcd，保留 map-leaf=0.5、
resolution=1.0，使用另一个全新输出目录。每组内 CPU/P2D/D2D 参数完全相同。

各后端顺序、独立进程执行；不同时占用 GPU。每次试验重新创建源指针，避免
重复样本复用 D2D 源体素而虚报性能。目标地图只设一次。预热与稳定阶段分开，
目标体素在首次 setInputTarget 时构建并单独记录；初始化、地图准备、上传、
目标体素和各帧源体素分别计时。预热可包含剩余的首次 Kernel/JIT 开销。
source_set_sync_ms 含主机转换和同步上传调用，并非纯 DMA；optimizer_sync_ms
含 GPU 导数和主机优化控制。计时边界调用 cudaDeviceSynchronize，不只测提交。
run.py 用 0.5 秒周期、最多 4096 条系统资源快照；短峰值可能漏采。
RSS 和 device_free_bytes 不能相加当成 NX 独立显存；NX 为共享内存。

结果完整性与解释
----------------

analyze.py 给出 n/均值/P50/P95/P99/最大值、收敛数、独立空间质量检查结果、
CPU/GPU 位姿差异。缺后端、缺摘要、重复样本、非法指标、非零退出等为 partial。
空间质量检查不包括完整 UKF/腿部/时间安全状态机，不等于正式定位验收。
结果相似不等于绝对准确；历史静止样本不能替代运动、稀疏区域和地图边缘数据。
本轮没有 GPU 原生数据；GPU 延迟、显存、收敛率和端到端收益均为未验证。

完整定位实验入口（未来单独授权后）
---------------------------------

::

  python3 experiments/gpu_ndt/runtime.py seal --workspace /tmp/lite3_gpu_ndt_ws_unique
  python3 experiments/gpu_ndt/runtime.py launch --workspace /tmp/lite3_gpu_ndt_ws_unique \
    --backend NDT_CUDA_P2D --domain 151 --map /absolute/path/localization_map.pcd \
    --scan-leaf 0.5 --map-leaf 0.5

启动前校验源码和二进制/动态库哈希及实际 package prefix。强制独立 ROS domain
100..200、ROS_LOCALHOST_ONLY=1；仅定位，不包含 Nav2、速度门或运动接口。
生成的 launch 自身也检查隔离环境。默认输入 /gpu_ndt/input/points、
/gpu_ndt/input/imu；需在该独立 domain 提供合法原始时间、同帧腿部 TF、IMU、
静态外参和初始化输入，不能将历史扫描改成当前 stamp 绕过新鲜度检查。
该入口不自动建立视觉/手动初始化守卫；完整系统验证须另外规划，不能声称
单独启动配准节点就验证了全部初始化及导航链路。

实机验证顺序
------------

1. 未来授权后先在 DISARMED、独立目录完成 sm_72 构建、链接、CUDA 初始化及 Kernel 验证。
2. 相同地图/扫描先比较 CPU/P2D/D2D 的冷启动、稳定耗时和位姿；用 Nsight 确认 CUDA Kernel。
3. 用独立低负载测量拆分预测、纯 align、外部质量检查、融合、发布及订阅调度。
4. 比较系统 CPU、GPU/EMC、内存及 NDT 源龄/锚点有效率，检查同帧腿部 TF 和质量拒绝。
5. 正式流水线的 DISARMED 对照仍须另行授权，观察 VOA 超过 0.35/0.5 秒的断档。
6. 所有静止、定位正确性和故障停车检查通过后，再单独授权受控运动；不自动 ARM。

测试
----

::

  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
    tests/test_gpu_ndt_experiment.py tests/test_ndt_scan_candidates_native.py \
    tests/test_ndt_output_priority_native.py tests/test_scan_readiness_native.py \
    tests/test_ndt_pose_fusion_native.py

本地测试包括真实 C++ 工厂/包装器在 CUDA/PCL Mock 下的执行，以及现有算法
提取回归；不能标为 Jetson/Foxy/CUDA 原生通过。本轮 CPU 稳定源码、启动默认值、
NDT 质量门限、0.35/0.5 秒安全期限以及 VOA/Nav2/Velocity Gate 均保持原状。

本轮本地验证结果
----------------

* GPU 专项 94 项通过；相关 NDT、UKF、初始化、时间戳与 TF 回归 168 项通过、
  2 项跳过。两项需要 P0/P1 之前的基线头文件；当前会话未提供该文件。
  UKF 历史测试的报告输出通过临时 pytest 插件重定向到 /tmp，历史结果未覆盖。
* 实际 Jetson fast_gicp 头文件与本地 PCL 的 C++ 接口语法检查通过，CUDA API
  使用声明替身，未链接 CUDA。它不等同于 GPU 原生编译。
* 独立基准的 CPU 模式用真实 PCL 1.10 编译、运行通过。VM 使用未改动的
  ndt_omp 上游头文件，commit 5495fd9214945afcb4b35d5a1da385e405c52bf9；
  不是 Jetson 二进制性能测试。CPU-only 二进制选择 GPU 时也已验证明确退出。
* 两组历史静止样本各为 6 帧、2 个初值、每例 2 次预热及 5 次稳定重复。
  旧室内地图降采样后 831 点、室外地图 130410 点；不能混同最近室内 v3 地图。
  各 60 次稳定调用均收敛且通过独立空间检查，未执行完整 UKF/初始化验收。
* 最终纯 align 的 P50/P95/P99/最大值：室内 1.199/2.591/2.962/3.217 ms；
  室外 21.417/42.619/47.578/50.915 ms。仅 VM CPU、OMP=1、-O1，GPU 无实测。
  这些数值不能与 Jetson 含锁等待和预测的 match 日志直接相减比较。
* CPU/P2D/D2D 对照整体仍为 partial；GPU 原生编译、Kernel、UKF、TF、锚点、
  VOA 连续性及真实 CPU 降负载均未验证。摘要见 evidence_20261114.json。

原生验证准备补充（2026-11-14，导航运行期间未执行机载实验）
-----------------------------------------------------------

只读检查确认当前 audited CPU 后端使用正式 ndt_omp.so，SHA-256 为
00318664bca0005516217ecea384261b52eeac3e1b117ce50d1e36d832695aed。
正式工厂为 NDT_OMP、DIRECT1、epsilon=0.01；分辨率显式使用 1.0，
OMP_NUM_THREADS=1。实际 ndt_omp 实现默认 max_iterations=35、step_size=0.1。
CPU 基准复用该类及参数；后续机载构建直接链接上述安装库，不使用 VM 的参考头文件。
GPU 上游默认 LM 为 64 次、translation epsilon=0.0005、rotation epsilon=0.002，
其停止准则与 CPU 不等价。没有为对齐数字而修改任何后端或安全质量门限。

导航启动后的只读 /proc 核查确认 PID 8075 正在使用同一 SHA 的 ndt_omp.so，
参数为 NDT_OMP、DIRECT1、NDT resolution=1.0、OMP=1。
当前运行地图是 patrol_map_20261031_01_area_v2_review，源/地图降采样均为 0.3。
历史室外六帧基准采用 0.5 的地图降采样，源点也已历史预处理；二者不能宣称是
相同的当前完整导航配置。该六帧只能先做固定输入的三后端冒烟及重复性比较。
如需认证当前 0.3 预处理配置，应从配套原始扫描、IMU、外参快照重新准备数据，
不能把已有较稀疏源点再次降采样就当作还原了更密的扫描。
当前 CPU 进程映射了含 CUDA 的 fast_gicp 动态库，但选择的是 NDT_OMP；
库被加载不等于 CUDA NDT 正在执行。

native.py 只构建 fast_gicp 和无 ROS 的独立基准，不构建/启动定位节点。
它使用全新目录复制依赖源码，添加 sm_72 cubin/PTX，私有安装动态库，
核查 CPU/GPU 动态库实际解析路径和正式运行清单的哈希。
构建与运行进程使用独立环境、CUDA 缓存及单构建任务；没有 sudo、全局安装、
系统库路径配置、功耗/时钟调整、节点启停或运动接口。
build/run 均要求人工确认安全状态，且检测到导航、定位或另一个基准时拒绝执行；
途中发现导航启动，会结束本工具自己的子进程，不终止业务进程。
该确认标志不能替代真实运动模式反馈，也不会发送 DISARM。

以下命令只供导航结束、重新确认 DISARMED 且无人操纵后执行。
假定工具位于独立会话的 tooling 目录，workspace 尚不存在；本轮没有执行这些命令。

::

  python3 "$GPU_NDT_SESSION/tooling/native.py" build \
    --root "$GPU_NDT_SESSION/workspace" \
    --fast-source /home/ysc/lite_cog_ros2/fast_gicp/src/fast_gicp \
    --ndt-prefix /home/ysc/lite_cog_ros2/ndt_omp/install/ndt_omp \
    --official-manifest /home/ysc/lite3_patrol_project/results/ndt_realtime_current/runtime_manifest.json \
    --operator-confirmed-disarmed
  python3 "$GPU_NDT_SESSION/tooling/native.py" check --root "$GPU_NDT_SESSION/workspace"
  python3 "$GPU_NDT_SESSION/tooling/native.py" run \
    --root "$GPU_NDT_SESSION/workspace" \
    --dataset "$GPU_NDT_SESSION/dataset_outdoor" \
    --output "$GPU_NDT_SESSION/workspace/runs/smoke" \
    --warmup 1 --repeats 1 --operator-confirmed-disarmed

正式性能比较使用未启用 Nsight 的运行；Kernel 证明另做短 CUDA trace。
第一轮顺序 CPU/P2D/D2D，复测使用相反顺序，以检查温度、动态频率和顺序偏差。
nsys 存在不代表 NDT Kernel 已运行，仍需实际 trace 中的 NDT 内核事件。

新增测量字段：扫描读盘、原生后端参数、GPU 型号/计算能力、显式同步等待。
explicit_sync_wait_ms 只是显式 barrier 的子集，不含所有隐式同步，不能再加到 align 上。
prepare_ms 仍只是复制已预处理源点，不是完整原始 LiDAR 预处理。
CPU 的 map_set_sync_ms 包含目标网格设置/体素构建；未独立拆分的 GPU 专属地图
上传/构建字段在 CPU 结果中为 null，不填成零。
run.py 在 Jetson 存在 tegrastats 时独立采样 1 Hz，保存 EMC、GPU、RAM、VDD_IN 原文；
只停止自己创建的采样进程，资源缺失时不填零。分析器保留失败样本，并同时统计
全部有限位姿差与双方空间检查通过的位姿差，避免只看成功样本掩盖错误匹配。

连续室外数据尚未齐备。已找到的约 169 秒 FAST-LIVO bag 只有 CustomMsg 和 IMU，
没有本次 NDT 配准坐标源点、全三维配准位姿及其与室外地图的有效配对证据。
不能使用它的室内地图标签或平面 /odom 代替这些数据。
后续应保留连续、同源 stamp 的 aligned_points 与 /patrol/registration_odom 全三维位姿、
status 和地图哈希；检查时间间隔、漏帧及场景标签后再还原配准源点。
六帧的重复执行只能用于冒烟和重复性测量，不能称为长连续扫描验收。

本轮原生验证结果（2026-11-16）
----------------------------

已在 Jetson 的独立目录完成 CUDA 11.4 / sm_72 原生构建、P2D/D2D 实际运行，
并用 Nsight 记录到实际 NDT 导数 Kernel。不是仅头文件、Mock 或 PTX 可用性检查。
发现并修正 Foxy ndt_omp 只导出 ndt_omp::ndt_omp target、不设置旧式库变量的
兼容问题；CPU 对照仍链接正式原库，DIRECT1、epsilon=0.01、OMP=1 等不变。
保护清单额外固定外部 CPU 动态库哈希，未覆盖原厂或 audited CPU 文件。

室内、室外各六帧，两种初值、正反后端顺序各五次重复，每组每后端120条稳定样本。
室外配准 P50/P95/P99/最大值（ms）：

* CPU：68.152 / 133.204 / 140.746 / 140.942。
* P2D：80.066 / 124.331 / 197.044 / 269.846。
* D2D：77.616 / 162.288 / 185.387 / 186.658。

室外重复对照收敛数为120/120、120/120、106/120；较少预热的首轮冒烟中，
P2D也有一次不收敛，未删除失败样本。室内两种GPU后端均比CPU慢。
P2D每次独立流程CPU时间约由101.7降到87.2ms，但流程总时延更长；
不能称为正式导航已降负载。GPU与CPU的位置差异最大约0.45m（室外），
部分Z分量差异约0.4m；公共几何分数不能代替外部真值。

因此按条件验收停止进入实际定位集成：完整GPU定位仅生成隔离源码，
未原生构建/启动ROS定位节点，未替换默认CPU，TF/锚点/VOA改善未验证。
当前版本不满足替换CPU的条件。后续优先研究上游CUDA临时分配及同步开销，
并检查位姿差异和长连续运动数据，不能靠放宽安全/质量门限完成切换。

新增 trace.py 可在已封存的独立基准上取得短CUDA追踪；与性能采样分开：

::

  python3 experiments/gpu_ndt/trace.py \
    --root "$GPU_NDT_SESSION/workspace" \
    --dataset "$GPU_NDT_SESSION/dataset_outdoor" \
    --output "$GPU_NDT_SESSION/workspace/traces/new_p2d" \
    --backend NDT_CUDA_P2D --operator-confirmed-disarmed

追踪工具只处理自己的Nsight/基准进程，兼容Nsight为目标创建不同进程组的行为。
未启用GPU硬件性能计数器或修改权限；功耗传感器因需要root读取而未取得数据。

完整报告见 docs/Jetson_时间同步与GPU_NDT验证_20261116.rst，
结构化摘要见 evidence_20261116.json。原始结果永久保存于：
/home/duyipei/Lite3_GPU_NDT_results/20261116_ksq6a947
