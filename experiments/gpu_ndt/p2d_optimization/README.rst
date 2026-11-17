Lite3 GPU P2D：error-only 与实例工作区
===================================

状态与范围（2026-11-17）
-----------------------

两项实验源码补丁及离线测试已实现。本机没有 CUDA Toolkit，Jetson SSH 两次
连接超时；本轮没有原生 CUDA 编译、Kernel 执行或性能结果。不能据此宣称加速。
正式 CPU NDT、NDT P0/P1、融合、VOA、速度门、时间戳及安全期限均不修改。

本目录包含可直接物化的真实源码补丁，不是在运行中替换系统动态库：

* A：原始 GPU P2D，保留原有完整导数路径。
* B：仅增加 error-only；完整导数和 D2D 计算函数不变。
* C：B 加实例工作区；只让 P2D 使用持久缓冲，D2D 沿用原路径。
* CPU：原有独立基准的正式 ``ndt_omp.so``、DIRECT1、epsilon=0.01、OMP=1。

``baseline.json`` 固定了此前 Jetson 保存的五份真实源码的 SHA-256；
``baseline/`` 内另外八份头文件来自上游固定 commit
``0e7ec1441c99f7be453db2ea216d5de029387417``，只供离线编译替身测试。
它们尚未与 Jetson 已安装头文件逐项核对。原生准备时读取完整机载源码中的
实际头文件，检查补丁插入点；绝不拿这些参考头文件覆盖机载安装。
上游来源和哈希见 ``baseline.json``，BSD-3-Clause 声明保存在 ``baseline/LICENSE``。

核心改动
--------

``patch.py`` 在全新目录复制指定的完整 fast_gicp 源码，再应用以下片段：

* ``error_functor.inc``、``error_function.inc``、``error_declaration.inc``：
  FP32 标量误差、原有 Cauchy 权重、协方差逆、有效体素条件；不构造或归约 H/b。
  只在 H 和 b 同时为空时调用，使用当前线性化已经建立的对应关系，不重新搜索。
* ``workspace.inc``、``workspace_search.inc``、``workspace_error.inc``：
  每个 Core 懒创建一份 P2D 工作区，持有一个搜索位姿、两个评估位姿以及
  对齐分配的主机评估位姿；对应关系首次创建时按当前源点数和 offsets 预留容量，
  后续保留原容器的 capacity 复用和扩容策略，不逐轮强制精确 reserve。
  搜索仍 resize、重新写入并压缩有效长度，不把容量当作当前对应关系数量。
* ``B_error_only.patch`` 和 ``C_workspace.patch``：便于审查的增量完整 diff。
  原生操作应使用带哈希检查的物化工具，不应盲目将 diff 应用到未知版本。

CPU LM、6×6 求解、接受比、收敛判断、完整导数数学和质量门限不变。
没有删除 CUDA 同步：原搜索完成事件、主机读取归约结果的完成依赖均保留。
Thrust 内部归约临时分配仍可能存在，本轮没有更换它的分配器或删除必要等待。

同一个 NDT 实例的操作必须串行，保持原实现的使用条件；工作区不支持同一实例
并发 align。多个实例各自拥有工作区，不共享静态缓冲。保留的对应关系容量上界
取决于该实例见过的最大扫描和邻域规模，与原搜索 resize 的最大需求一致。
不增加地图副本、匹配线程或发布消息。

C 改变 ``NDTCudaCore`` 成员布局。必须同时重建私有 ``libfast_gicp.so``、
``libfast_vgicp_cuda.so``、安装头文件和实验程序，禁止单独替换某个 .so。
``native.py`` 默认构建入口保持兼容，新增可选私有 benchmark 源目录；封存
完整源文件、安装头文件、动态库及正式 CPU 文件哈希，并核对 sm_72 和链接路径。

离线验证的含义
--------------

``tests/p2d_host_check.cpp`` 将真实生成的 .cu 和原始 LM 函数体作为主机 C++
编译，使用明确标记的 Thrust、TF 无关的 API 替身及真实 Eigen 运算：

* 200 组候选位姿误差检查，含非零旋转、各向异性协方差和较大残差。
* 无效体素、空对应关系、部分 H/b 请求、不同扫描尺寸和对应关系压缩。
* 五种初值的 LM 决策、次数和最终位姿一致性。
* 两个实例独立生命周期、P2D/D2D 模式切换和重复调用。
* 主机地址/未定义行为检查；只检查主机替身内存，不是 GPU memcheck。

主机顺序归约的数值一致不能证明 CUDA 并行归约一致。标量和 tuple 归约在 GPU
上可能采用不同的加法顺序；原生阶段必须重新检查 LM 决策，不能只比较最终耗时。
25 轮搜索+完整评估+候选评估在主机替身中的显式位姿容器分配从 75 次降为 0，
只是代码路径验证，不是 cudaMalloc 实测数量或性能收益。

执行离线测试：

::

  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider \
    tests/test_gpu_ndt_p2d_optimization.py tests/test_gpu_ndt_p2d_tools.py

Jetson 后续原生执行流程（本轮未执行）
------------------------------------

仅在已重新确认静止、DISARMED、导航和定位已退出时执行。
操作标志不会发送 DISARM，也不能证明运动主机模式。工具会拒绝已知导航、定位或
另一个基准进程；途中发现干扰只结束它自己创建的子进程。
构建为 -j1、独立库前缀和 CUDA 缓存，不改系统安装、ROS Overlay 或 GPU 频率。

以下所有命令在项目根目录运行。输出目录必须全新，失败结果保留，不覆盖重跑。
先检查这些历史输入路径仍存在、哈希与配置仍对应当前正式 CPU：

::

  cd "$HOME/lite3_patrol_project"
  P2D_OLD="$HOME/lite3_patrol_project/results/gpu_ndt_20261116_ksq6a947/workspace"
  P2D_BASE="$P2D_OLD/deps/src/fast_gicp"
  P2D_DATA="$HOME/lite3_patrol_project/results/gpu_ndt_20261116_ksq6a947/dataset_outdoor"
  P2D_CPU_PREFIX="$HOME/lite_cog_ros2/ndt_omp/install/ndt_omp"
  P2D_CPU_MANIFEST="$HOME/lite3_patrol_project/results/ndt_realtime_current/runtime_manifest.json"
  P2D_SESSION="$HOME/lite3_patrol_project/results/p2d_optimization_$(date +%Y%m%d_%H%M%S)"
  mkdir "$P2D_SESSION"

先构建 A、B 并执行数值验证；这里的计时包含双重误差评估和 LM 输出，不能用于性能比较：

::

  for P2D_VARIANT in A B; do
    python3 -m experiments.gpu_ndt.p2d_optimization.patch \
      --base-fast-gicp "$P2D_BASE" \
      --output "$P2D_SESSION/sources/$P2D_VARIANT" --variant "$P2D_VARIANT" || break
    python3 -m experiments.gpu_ndt.p2d_optimization.experiment build \
      --source "$P2D_SESSION/sources/$P2D_VARIANT" --root "$P2D_SESSION/$P2D_VARIANT" \
      --ndt-prefix "$P2D_CPU_PREFIX" --official-manifest "$P2D_CPU_MANIFEST" \
      --operator-confirmed-disarmed || break
    python3 -m experiments.gpu_ndt.p2d_optimization.experiment run \
      --root "$P2D_SESSION/$P2D_VARIANT" --dataset "$P2D_DATA" \
      --output "$P2D_SESSION/$P2D_VARIANT/verify" --mode verify \
      --warmup 2 --repeats 3 --operator-confirmed-disarmed || break
  done

另做短 Nsight 跟踪，必须看到新的 ``p2d_ndt_error_only_kernel`` 执行事件：

::

  env -u LITE3_P2D_VERIFY python3 -m experiments.gpu_ndt.trace \
    --root "$P2D_SESSION/B" --dataset "$P2D_DATA" \
    --output "$P2D_SESSION/B/traces/scalar" --backend NDT_CUDA_P2D \
    --operator-confirmed-disarmed
  python3 -m experiments.gpu_ndt.p2d_optimization.experiment validate \
    --a-run "$P2D_SESSION/A/verify" --other-run "$P2D_SESSION/B/verify" \
    --kernel-evidence "$P2D_SESSION/B/traces/scalar/evidence.json" \
    --output "$P2D_SESSION/AB_verified.json"

验证要求：每次候选 scalar 与完整路径差值不超过
``2e-5 + 2e-6 * max(1, abs(reference))``；这个界限只比较误差计算一致性，
不是新的定位质量门限。必须同时满足 LM rho 符号/内循环序列、次数、收敛状态、
几何检查结果一致，最终位置差和旋转矩阵 Frobenius 差各不超过 1e-4。
任何失败都停止后续阶段，不通过放宽收敛或质量条件使其通过。

只有上述 B 原生验收凭据通过后才允许构建 C：

::

  python3 -m experiments.gpu_ndt.p2d_optimization.patch \
    --base-fast-gicp "$P2D_BASE" --output "$P2D_SESSION/sources/C" --variant C
  python3 -m experiments.gpu_ndt.p2d_optimization.experiment build \
    --source "$P2D_SESSION/sources/C" --root "$P2D_SESSION/C" \
    --ndt-prefix "$P2D_CPU_PREFIX" --official-manifest "$P2D_CPU_MANIFEST" \
    --b-receipt "$P2D_SESSION/AB_verified.json" --operator-confirmed-disarmed
  python3 -m experiments.gpu_ndt.p2d_optimization.experiment run \
    --root "$P2D_SESSION/C" --dataset "$P2D_DATA" --output "$P2D_SESSION/C/verify" \
    --mode verify --warmup 2 --repeats 3 --operator-confirmed-disarmed
  env -u LITE3_P2D_VERIFY python3 -m experiments.gpu_ndt.trace \
    --root "$P2D_SESSION/C" --dataset "$P2D_DATA" \
    --output "$P2D_SESSION/C/traces/scalar" --backend NDT_CUDA_P2D \
    --operator-confirmed-disarmed
  python3 -m experiments.gpu_ndt.p2d_optimization.experiment validate \
    --a-run "$P2D_SESSION/A/verify" --other-run "$P2D_SESSION/C/verify" \
    --kernel-evidence "$P2D_SESSION/C/traces/scalar/evidence.json" \
    --output "$P2D_SESSION/AC_verified.json"

再进行不插入 Nsight、不开 LM 日志、不做双重误差计算的正常性能测量：

::

  python3 -m experiments.gpu_ndt.p2d_optimization.experiment run \
    --root "$P2D_SESSION/A" --dataset "$P2D_DATA" --output "$P2D_SESSION/A/measure" \
    --mode measure --cpu-reference --warmup 2 --repeats 5 --operator-confirmed-disarmed
  python3 -m experiments.gpu_ndt.p2d_optimization.experiment run \
    --root "$P2D_SESSION/B" --dataset "$P2D_DATA" --output "$P2D_SESSION/B/measure" \
    --mode measure --receipt "$P2D_SESSION/AB_verified.json" \
    --warmup 2 --repeats 5 --operator-confirmed-disarmed
  python3 -m experiments.gpu_ndt.p2d_optimization.experiment run \
    --root "$P2D_SESSION/C" --dataset "$P2D_DATA" --output "$P2D_SESSION/C/measure" \
    --mode measure --receipt "$P2D_SESSION/AC_verified.json" \
    --warmup 2 --repeats 5 --operator-confirmed-disarmed
  python3 -m experiments.gpu_ndt.p2d_optimization.report \
    --a-run "$P2D_SESSION/A/measure" --b-run "$P2D_SESSION/B/measure" \
    --c-run "$P2D_SESSION/C/measure" --output "$P2D_SESSION/comparison.json"

每条失败后应停止，不继续复制后续命令。验收凭据固定原生运行文件和证据哈希，
不能使用主机替身的结果冒充。六帧数据只能用于冒烟和重复性检查；需要更长、连续且
同源时间戳的真实扫描序列才能研究室外长尾。换数据集时重新执行 A/B/C 数值验证。
短 Kernel 跟踪限制为不超过 32 个 case，可为较长数据集另外保留小型跟踪子集。

计时与负载解释
--------------

* ``align_sync_ms``：含必要同步的完整 align 主机耗时。
* ``align_process_cpu_ms``：仅 align 段的进程 CPU 时间，排除后续几何质量检查。
* ``process_cpu_ms``：沿用原基准整帧流程 CPU 时间，包含质量检查；勿与前者混用。
* ``candidate_error_ms``：候选误差调用耗时，包含位姿更新和归约等待，不是纯 Kernel 时间。
* ``lm_linearizations`` 与 ``lm_error_evaluations``：实际调用计数，不用最大迭代参数代替。
* 每帧前后读取 GPU 频率，另保存 0.5s 资源采样和 1Hz tegrastats 的温度、EMC、功耗。
  端点和粗采样不能证明整个 align 中频率一致。正序和反序复测用不同结果目录，
  频率/温度/后台负载不相近时不把差值归因于补丁；本工具不调整频率或功耗模式。
* ``explicit_sync_wait_ms`` 只是既有显式 barrier 的子集。隐式 CUDA 等待仍在 align 内；
  不可再把该字段加到 align 上，也不能当作所有同步开销。
* ``trace.py`` 保存 CUDA API 总计。``report.api_totals`` 可读取分配/同步次数和总耗时，
  但包括地图初始化、预热和所有重复匹配，不将其直接除成单帧开销。

剩余原生验收：在 Jetson 上以 compute-sanitizer 检查 C 的不同源点数、空匹配、
多实例及异常退出；观察重复 align 后内存是否稳定。主机 ASan 不能替代此项。
若 B 改变 LM 决策，先保存失败样本并检查原版重复性与归约差异，不自动进入 C。
若 C 仍慢于 CPU，优先根据追踪检查剩余 Thrust 临时分配、必要的 CPU/GPU 往返、
LM 实际迭代和 GPU 调频，不自动扩展为新的 CUDA 重构。

回退
----

本轮没有切换正式运行后端。停止实验进程即可；仍使用原正式 CPU 启动入口。
历史原版 GPU 实验目录也未覆盖，可单独选用 A；不需要替换系统库或回退 Git。
