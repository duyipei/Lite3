GPU P2D C：正式链路的人工实验发布与 CPU 回退
=========================================

本入口复用 ``p2d_optimization`` 的 C 版，生成独立的 hdl_localization 构建。
它与 standalone benchmark 不同：原项目 ``start_localization_only.sh`` 实际选择
该构建，原 ``PoseEstimator`` 通过 PCL Registration 接口取得 GPU 配准位姿，
随后仍执行原来的 UKF、腿部里程计融合、完整几何验收、注册锚点及 TF 发布。

本轮按授权只编译、链接检查和部署。未运行 benchmark、Kernel 数值/性能测试、
ROS 定位节点、Nav2 或运动控制。发布清单的 ``runtime_tested``、数值和性能验收
标志均为 false；编译成功不能解释为室外定位已验收。

实现范围
--------

* ``build.py deps``：复制已核对哈希的 fast_gicp 源码，应用 C；只追加一个导出的
  构建身份函数，不改目标函数或 LM。用 CUDA 11.4、sm_72、-j1 私有重建两个库。
* ``build.py node``：复制原 CPU audited 源包并覆盖当前 vendor_patch，保留所有
  P0/P1/UKF 修复；仅替换注册器工厂和 GPU 启动预检，复制独立 GPU Launch。
* ``build.py seal``：仅进行哈希、ELF、ldd 和版本清单检查，不启动生成的程序。
* ``deploy.py``：保留所有被更新启动文件的原件，先核对两条路径的链接和清单，
  最后写入下一次启动的 GPU 默认选择；任何激活异常都恢复先前文件。

CUDA 注册器继续沿用已有 ``CheckedCudaNdt``，不将 LM 移到 GPU、不改变收敛参数。
工厂在成功构造 GPU 对象后打印 ``NDT_BACKEND=GPU_P2D_C``，通过导出的构建 ID
和 dladdr 记录实际库。CPU 选择运行原二进制，启动器打印
``NDT_BACKEND=CPU_NDT_OMP``；其原有 ``NDT_OMP is selected`` 日志继续保留。
GPU 失败直接报错，无运行中或静默 CPU 回退。

原 ``TargetDistances``、几何 MSE、重叠率、收敛状态、0.5 秒扫描年龄、15 帧确认、
静止漂移和融合函数都不更改。GPU 的 Cauchy 内部误差不会被当作原几何质量分数。
原 CPU Launch 不修改；GPU Launch 只是其副本中选择 ``NDT_CUDA_P2D``，
默认不自行启动 Nav2。正常项目导航仍由原有启动器管理 Nav2、VOA 和 Gate。

部署布局
--------

机载构建根目录：

``/home/ysc/lite3_patrol_project/results/gpu_p2d_navigation_20261117_s14eaj55/gpu``

* GPU 库：``deps/install/lib/libfast_gicp.so``、``libfast_vgicp_cuda.so``。
* GPU ROS 节点：``ws/install/hdl_localization/lib/hdl_localization/hdl_localization_composition``。
* GPU Launch：``ws/install/hdl_localization/share/hdl_localization/launch/gpu_p2d_navigation.launch.py``。
* 原 CPU 构建仍在 ``results/ndt_realtime_20261112_7iwl6ku``；
  ``results/ndt_realtime_current`` 不改指向。

配置为 ``navigation/cpu_baseline/runtime.json``、
``navigation/gpu_p2d_trial/runtime.json`` 和 ``navigation/ndt_backend.json``。
两个后端使用不同安装前缀和固定依赖哈希。CPU 选择不读取 GPU 清单，因此 GPU
文件损坏不会妨碍显式 CPU 回退；默认配置缺失或 GPU 检查失败时拒绝启动。
不更新系统 PCL、CUDA、原厂库、CPU 库、DDS 或功耗配置。

``scripts/select_ndt_backend.py`` 只检查文件和链接。
``navigation/ndt_runtime_env.sh`` 选择 Overlay 并检查 ament 实际包前缀。
CPU 保留原完整 setup 链；GPU 使用私有 local_setup。双方在启动子进程前固定
各自的 LD_LIBRARY_PATH，不需要用户手工交换动态库。

人工启动命令
------------

以下在开发端 Ubuntu 虚拟机执行，本轮没有执行。先正常关闭旧导航会话。
示例保持原 Factory Only、RViz 手动初始化和室外地图；不自动 ARM。

GPU 显式选择：

::

  cd /home/duyipei/Lite3_Projects/lite3_patrol_project
  source /opt/ros/galactic/setup.bash
  LITE3_OBSTACLE_BACKEND=factory_only bash scripts/navigation_gpu_p2d.sh \
    start-manual outdoor_20260921_003_area_review

使用发布文件中的默认选择（成功部署后为 GPU）：

::

  LITE3_OBSTACLE_BACKEND=factory_only bash scripts/navigation_start.sh \
    start-manual outdoor_20260921_003_area_review

CPU 一键回退启动：

::

  cd /home/duyipei/Lite3_Projects/lite3_patrol_project
  source /opt/ros/galactic/setup.bash
  LITE3_OBSTACLE_BACKEND=factory_only bash scripts/navigation_cpu_omp.sh \
    start-manual outdoor_20260921_003_area_review

回退前在原导航终端按 Ctrl+C，等待原有 DISARM/取消和子进程清理完成。
不要让两个定位进程同时运行。必要时使用原启动器的 stop 命令结束旧会话后，
再执行 CPU 命令；没有重编译、Git 回退或依赖安装步骤。
CPU 命令是本次会话的显式选择，不修改全局 GPU 默认值。

确认日志：最近会话 ``logs/lite3_indoor_nav_outdoor_.../localization.log`` 应有
配置标签、构建 ID、二进制和库路径。GPU 还应有节点工厂自身的 GPU C 身份日志；
CPU 应有 ``NDT_OMP is selected``。配置标签本身不证明已执行 CUDA Kernel。

后续人工验收仍包括 Kernel/数值一致性、室外位姿、长尾、初始化、静止漂移、
TF 和 VOA 时效。若存在问题先停止会话切回 CPU，不放宽任何安全门限。
