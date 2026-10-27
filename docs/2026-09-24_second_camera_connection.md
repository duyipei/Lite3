# 第二台摄像头连接及视觉重定位迁移记录

## 2026-10-12：从虚拟机启动 LRCP 新地图采集

2026-10-13 起统一入口支持交互选择：运行
`bash scripts/mapping_start.sh start`，输入 1 使用 D435i，输入 2 选择 LRCP
预采集；回车默认1，q取消。未输入就关闭标准输入会取消，不会自动启动。
`video` 同样可以交互选择。命令行最后的相机编号优先，其次是环境变量
`LITE3_MAPPING_CAMERA`，两者均未指定才显示菜单。新批次默认按时间命名。
选择2仍需按下面流程在另一终端运行 `camera2-start`，且仍保留相机1数据；
并不是独立使用广角相机替换 RGB-D 建图算法。stop/status 不需要再次选择。

LRCP 实际连接在运动主机 192.168.1.120，Jetson 192.168.1.103 接收网络视频。
以下命令只采集，不启动导航、不发送速度、不停止运动主机的原厂服务。
先结束旧导航/采集会话，保持机器狗静止。相机1仍是必需输入，不要拔掉 D435i。

虚拟机终端1（自动同步两个相机桥接文件到 Jetson，运行期间保持打开）：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
bash scripts/mapping_start.sh camera2-start
```

看到 `RTSP connected` 后，在虚拟机终端2录制一个新批次：

```bash
cd /home/duyipei/Lite3_Projects/lite3_patrol_project
bash scripts/mapping_start.sh start outdoor_cam2_$(date +%Y%m%d_%H%M%S) 2
```

记录终端输出的 RUN_ID。只有出现 GO 后才用原厂遥控器缓慢采集，尽量覆盖有建筑、
门窗、标志等特征的视角，结束时回到起点并静止。脚本保留原有传感器录包检查。
可在另一终端预览：`bash scripts/mapping_start.sh video 2`。

停止并保存（先不要关闭终端1）：

```bash
bash scripts/mapping_start.sh stop
```

待停止验证完成，再在终端1按 Ctrl+C 关闭桥接。数据在 Jetson 的
`/home/ysc/lite3_patrol_project/bags/<RUN_ID>/`，旧地图不覆盖。
如提示桥接已运行，复用原桥接，不要重复启动。首次启动后端可能提示时钟同步，
只在没有导航等运行任务、机器狗静止时处理；不要在录制中调整时钟。

**这不是完整的 LRCP 视觉地图生成。** 保存的是雷达/IMU、D435i，以及 LRCP JPEG
图像；LRCP 时间戳是网络接收时刻，不能视为曝光同步。标定前采集只作预采集，
不保证以后可用于视觉重定位。正式采集建议先完成内参/畸变、雷达外参及延迟验证，
再实现雷达关联或多视图三角化的三维视觉特征库。当前 RGB-D 构建器继续拒绝相机2
批次，不冒用相机1的深度、内参和外参。桥接修改通过离线检查不代表实机验收。

## 固定编号和查看入口

- 相机1 = RealSense D435i，ROS2 彩色话题 /camera/color/image_raw。
- 相机2 = LRCP F1080P，ROS2 彩色话题 /front_camera/image_raw。
- 虚拟机项目目录运行 `bash scripts/camera_view.sh`，每次交互选择，回车默认1。
- 直接选择：`bash scripts/camera_view.sh 1` 或 `bash scripts/camera_view.sh 2`。
- 入口只选择预览订阅源，不停止其他相机，不改变导航使用的数据库或标定。相机2需要 Jetson 的 start_front_camera.sh 已运行。

用户希望未来视觉重定位使用这台摄像头。本记录只保存连接事实与待办，尚未切换导航配置；不保存 SSH 密码。

## 已确认连接

- 运动主机：192.168.1.120，用户名 ysc；Jetson 感知主机：192.168.1.103。
- 第二台相机：LRCP F1080P，USB ID 1bcf:2cc8，采集节点 /dev/video0。
- /dev/video0 和 /dev/video1 属于同一台 LRCP 相机，不是两台相机。
- 视频地址：rtsp://192.168.1.120:8554/test
- 已有管线：V4L2 MJPEG 1280x720，配置 30 fps，经 mpph264enc 编码，由 MediaMTX 提供 RTSP。
- 从 Jetson 经 RTSP/TCP 成功解码 60 帧；虚拟机 OpenCV 窗口连续显示超过 100 帧。配置帧率不等于实测稳定帧率。
- 不需要 SSH 登录即可读取当前 RTSP 流；避免重复打开 /dev/video0 或重启运动主机服务。
- 第一台相机是 Jetson 上的 D435i，提供 /camera/color/image_raw 与 /camera/depth/image_rect_raw。

## 迁移需要修改的环节

1. 增加 Jetson 端 RTSP-to-ROS2 接入节点，使用独立话题（建议 /front_camera/image_raw 和 /front_camera/camera_info）、光学坐标系；限制解码积压、处理断流和陈旧帧。接收时间不等于曝光时间，需评估流延迟及时间同步。
2. 标定 LRCP 相机内参、畸变及相机到 base/LiDAR 的外参；分辨率改变时同步更新内参。先去畸变或在求解及误差计算中一致使用畸变模型。
3. rgbd_visual_localizer.py 在线仅订阅彩色图，不依赖实时深度，但从数据库读 color_k 与 base_to_color_optical，solvePnPRansac/projectPoints 当前传入空畸变参数。迁移需支持正确的查询相机标定与模型。
4. build_rgbd_visual_map.py 当前依赖 D435i 彩色、深度、CameraInfo 和 depth_to_color。LRCP 没有已验证的深度输出，不能把 D435i 深度直接当它的深度。需新增地图构建路线：标定同步后的 LiDAR 投影关联，或多视图三角化并与雷达地图对齐。必须验证可见性、遮挡、覆盖及几何质量。
5. 重采 LRCP 参考图像并建立独立视觉数据库，保留原库。原雷达点云及导航栅格在环境/坐标未变化时可以复用，但新视觉地图必须注册到该坐标系。旧库跨相机匹配仅可作为实验，不能默认兼容。
6. navigation/start_visual_localizer.sh 将固定彩色话题和检查改为相机可配置；scripts/indoor_nav_start.sh 的 RealSense 服务及图像检查按选定视觉源分支，其他实际依赖深度的组件仍保留检查。
7. 室内/室外建图脚本添加 LRCP 图像、标定、时间信息及配套雷达位姿录制；检查 visual_initialization.yaml 中新视觉地图到雷达地图的变换。
8. 保持静止验证多位置多朝向重定位及 NDT 确认；不放宽安全门控，不因视频能显示就允许导航运动。

下一阶段建议先完成稳定 ROS2 接入与标定，再选择并实现新的 3D 视觉地图构建路线。

## ROS2 接入已实施（2026-09-24）

### 建图采集接入

在虚拟机项目目录运行：

```bash
bash scripts/mapping_start.sh start outdoor_cam2_001 2
# 或室内：
bash scripts/mapping_start.sh start indoor_cam2_001 2
```

相机编号省略默认1，也支持环境变量 LITE3_MAPPING_CAMERA=2。相机2模式保留原 D435i、雷达、IMU，额外记录 /front_camera/image_raw/compressed（JPEG质量90）以及存在时的 /front_camera/camera_info。必须先在 Jetson 启动更新后的 start_front_camera.sh；目前已运行。录包的 camera_selection.yaml 保存选择，stop/verify 无需重复指定相机编号。video 2 选择相机2预览。

这是相机2视觉地图的原始数据采集接入，不是完整视觉地图生成：相机2内外参、曝光/接收延迟及3D特征生成尚未完成。旧 build_rgbd_visual_map.py 检测到相机2采集标记时拒绝生成，避免误用 D435i 图像。标记应与 sensors 目录一起归档，不应删除来绕过检查。标定前采集的数据不保证可用于最终定位。新增压缩图像在有订阅者时编码；仍有额外CPU与存储成本，正式建图前沿用原传感器质量门控。

- 新增 lite3_patrol_nav/rtsp_camera.py 及 navigation/start_front_camera.sh，已同步 Jetson。
- Jetson 启动：`bash /home/ysc/lite3_patrol_project/navigation/start_front_camera.sh`。当前已后台启动，不要重复运行；不是开机自启。
- 节点 /front_camera_bridge，话题 /front_camera/image_raw，sensor_msgs/msg/Image，1280x720 bgr8，SensorData/BEST_EFFORT QoS。
- 目标发布频率 10 Hz；10 秒订阅实测 65 帧且时间戳全部不同，接收频率约 6.53 Hz。并非保证 10 Hz。
- 日志 /tmp/lite3_front_camera_bridge.log。未改导航、D435i 或运动主机服务。
- 使用最新帧单槽缓存、5 秒打开/读取超时、2 秒重连；接收时间戳不能消除编码和网络延迟，断流重连逻辑尚未主动断网测试。
- 尚无标定，因此不发布 CameraInfo 或 TF；不能直接作为已标定视觉定位输入。
- setup.py 本机已注册入口，但未重建部署整个 ROS 工作区，目前用上述脚本直接启动已同步的模块。
