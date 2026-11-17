GPU P2D C 编译部署记录（2026-11-17）
===================================

总体结果
--------

已完成 Jetson CUDA 11.4/sm_72 原生编译、两个私有 GPU 动态库和正式
hdl_localization 节点的完整重建、链接核对、启动配置部署。
下一次使用默认启动入口时选择 GPU P2D C；显式 CPU 回退保留原 audited 二进制。
本轮未运行测试、benchmark、CUDA Kernel 数值/性能验证、定位节点、Nav2、
ARM、导航目标或速度指令。编译部署成功不代表定位精度或实时性验收通过。

GPU 构建 ID：GPU_P2D_C_20261117_s14eaj55。
实际构建根目录：/home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu。
CPU 原目录：/home/ysc/lite3_patrol_project/results/ndt_realtime_20261112_7iwl6ku。
CPU current 符号链接未改，原 manifest、二进制和依赖库哈希均已复核。
原启动文件备份：/home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/before_deploy。

实际接入与质量控制
------------------

GPU 专用 Launch 设置既有参数 reg_method=NDT_CUDA_P2D，节点通过
CreateRegistration 返回现有 CheckedCudaNdt/PCL Registration 对象。
PoseEstimator::correct 继续 setInputSource -> align -> getFinalTransformation，
然后执行原 UKF、腿部预测、事务式几何验收、融合、状态/锚点/TF/odom 发布。
原 CPU Launch 字节未变；CPU 模式使用原可用构建，而不是新 GPU 节点模拟 CPU。

GPU 内部 Cauchy 目标值没有替代外层质量分数：外层仍使用原最近邻几何 MSE、
重叠率和收敛状态，保留扫描年龄、15 帧确认、漂移、原阈值和异常拒绝。
UKF、腿部融合、TF、VOA、速度门、Nav2 参数和足迹没有修改。
GPU 库增加的导出 build_id 函数只做身份标记，不改计算。
工厂成功构造 GPU 注册器后打印 build_id 与 dladdr 实际库路径；失败不回退 CPU。

CPU LM、6×6 求解、位姿更新、接受/拒绝、收敛仍在 CPU；对应关系和
P2D 误差/梯度/Hessian 仍在 GPU，包含 C 的 error-only 和工作区复用。

启动与回退
----------

在 Ubuntu 开发虚拟机执行，不在 Jetson 直接运行 VM 启动器。
以下命令均未由 Codex 执行；保持原来的人工 ARM 流程。

GPU 显式启动：

::

  cd /home/duyipei/Lite3_Projects/lite3_patrol_project
  source /opt/ros/galactic/setup.bash
  LITE3_OBSTACLE_BACKEND=factory_only bash scripts/navigation_gpu_p2d.sh \
    start-manual outdoor_20260921_003_area_review

原入口当前默认也使用 GPU：

::

  LITE3_OBSTACLE_BACKEND=factory_only bash scripts/navigation_start.sh \
    start-manual outdoor_20260921_003_area_review

CPU 回退：先在原导航终端按 Ctrl+C，等待原有 DISARM/取消/清理结束，再执行：

::

  cd /home/duyipei/Lite3_Projects/lite3_patrol_project
  source /opt/ros/galactic/setup.bash
  LITE3_OBSTACLE_BACKEND=factory_only bash scripts/navigation_cpu_omp.sh \
    start-manual outdoor_20260921_003_area_review

无需重编译、换库、安装依赖或 Git 回退。CPU 入口只覆盖本次会话选择，
不修改 GPU 默认配置。GPU 错误会拒绝启动；CPU 回退不会读取 GPU 发布清单。

日志应显示 NDT_BACKEND=GPU_P2D_C 或 NDT_BACKEND=CPU_NDT_OMP、build_id、
二进制与库路径。GPU 节点还会打印自身工厂日志；CPU 保留 NDT_OMP is selected。
本轮仅验证了配置/环境/文件/包索引/ldd，不是节点执行日志或 Kernel 证明。

编译命令
--------

以下为机载实际执行命令，完整环境和输出保存在本次构建记录中。
均为单任务构建，BUILD_TESTING=OFF，没有 colcon test、ctest 或试运行。

::

  cmake -S /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/deps/src/fast_gicp -B /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/deps/build -DBUILD_VGICP_CUDA=ON -DBUILD_apps=OFF -DBUILD_test=OFF -DBUILD_TESTING=OFF -DBUILD_PYTHON_BINDINGS=OFF -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.4 -DCMAKE_INSTALL_PREFIX=/home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/deps/install
  cmake --build /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/deps/build -- -j1
  cmake --install /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/deps/build
  colcon --log-base /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/ws/log build --base-paths /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/ws/src --build-base /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/ws/build --install-base /home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/ws/install --executor sequential --packages-select hdl_localization --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release '-DCMAKE_CXX_FLAGS_RELEASE=-O1 -DNDEBUG' -Dfast_gicp_DIR=/home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu/deps/install/share/fast_gicp/cmake -Dndt_omp_DIR=/home/ysc/lite_cog_ros2/ndt_omp/install/ndt_omp/share/ndt_omp/cmake

产物与链接
----------

* 节点：gpu/ws/install/hdl_localization/lib/hdl_localization/hdl_localization_composition。
* GPU 库：gpu/deps/install/lib/libfast_gicp.so、libfast_vgicp_cuda.so。
* 原 CPU ndt_omp：/home/ysc/lite_cog_ros2/ndt_omp/install/ndt_omp/lib/libndt_omp.so。
* CPU 原 fast_gicp 库实际解析到 /home/ysc/lite_cog_ros2/fast_gicp/build/fast_gicp。
* sm_72 cubin、error-only 符号、构建身份符号及 Node 链接均已静态核对。
* 生成时原有质量/融合/输出函数逐段比较不变，PoseEstimator 和 UKF 源文件哈希不变。
* 两种后端和默认值的环境选择检查均退出 0，CPU/GPU 包前缀及库路径匹配。
* 核对结束后未发现项目定位、Nav2、Gate 或 benchmark 进程。

处理的编译/链接问题
------------------

隔离加载路径起初遗漏 /opt/ros/foxy/lib/aarch64-linux-gnu，导致静态 ldd
显示 console_bridge.so.1.0 缺失。已补齐环境路径后重新核对通过；无需改系统库。
CPU 保留完整 setup 链，避免只 source local_setup 时遗漏原 NDT/fast_gicp 依赖。

保留的非致命警告包括第三方 Eigen host/device 注解、原有未使用变量/参数、
生成 C 接口收到 -std=c++14，以及 raw CMake fast_gicp 前缀没有 colcon 根 setup。
最终依赖没有 not found，未通过禁用安全检查或改质量参数消除问题。

未验证项目
----------

CUDA Kernel 实际执行/数值、CPU/GPU 位姿对照、真实室外性能、P95/P99、
初始化与导航稳定性、GPU 内存/异步、TF/注册锚点/VOA 时效均未在本轮验证。
默认值是经授权的下一次人工实验配置，不是已经通过导航验收的稳定发布。

详细记录保存在开发端：
/home/duyipei/Lite3_GPU_NDT_results/gpu_p2d_integration_20261117_s14eaj55/jetson_records。
