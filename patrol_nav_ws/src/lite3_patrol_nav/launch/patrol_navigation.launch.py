import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    globalmap_pcd = LaunchConfiguration("globalmap_pcd")
    params_file = LaunchConfiguration("params_file")
    lidar_topic = LaunchConfiguration("lidar_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    start_localization = LaunchConfiguration("start_localization")
    start_navigation = LaunchConfiguration("start_navigation")
    specify_init_pose = LaunchConfiguration("specify_init_pose")
    use_imu = LaunchConfiguration("use_imu")
    convert_utm_to_local = LaunchConfiguration("convert_utm_to_local")
    ndt_resolution = LaunchConfiguration("ndt_resolution")
    downsample_resolution = LaunchConfiguration("downsample_resolution")

    localization = Node(
        condition=IfCondition(start_localization),
        package="hdl_localization",
        executable="hdl_localization_composition",
        name="patrol_localization",
        output="screen",
        parameters=[{
            "globalmap_pcd": globalmap_pcd,
            "convert_utm_to_local": convert_utm_to_local,
            "odom_child_frame_id": "base_link",
            "use_imu": use_imu,
            "invert_acc": False,
            "invert_gyro": False,
            "cool_time_duration": 2.0,
            "enable_robot_odometry_prediction": True,
            "robot_odom_frame_id": "robot_odom",
            "robot_odom_child_frame_id": "leg_base_link",
            "reg_method": "NDT_OMP",
            "ndt_neighbor_search_method": "DIRECT1",
            "ndt_neighbor_search_radius": 3.0,
            "ndt_resolution": ndt_resolution,
            "downsample_resolution": downsample_resolution,
            # Manual placement in RViz is the absolute map anchor.  NDT may
            # refine it locally but must never jump to another similar room.
            "manual_pose_translation_gate": 0.50,
            "manual_pose_yaw_gate": 0.20943951,
            # During the isolated initialization phase, keep publishing the
            # coarse seed until a bounded NDT candidate remains stable for
            # 30 scans. Then promote that candidate exactly once as the new
            # map/leg-odometry anchor. Runtime divergence gates stay strict.
            "auto_reanchor_enabled": True,
            "auto_reanchor_samples": 30,
            "auto_reanchor_position_span": 0.20,
            "auto_reanchor_yaw_span": 0.04363323,
            "auto_reanchor_max_translation": 0.75,
            "auto_reanchor_max_yaw": 0.26179939,
            "auto_reanchor_max_fitness": 0.15,
            "manual_gate_violation_samples": 3,
            # After acquisition, leg odometry supplies continuous motion and
            # NDT updates the map anchor only in small bounded increments.
            "ndt_fusion_gain": 0.05,
            "ndt_fusion_max_translation_step": 0.01,
            "ndt_fusion_max_yaw_step": 0.003490659,
            "specify_init_pose": specify_init_pose,
            "use_global_localization": False,
            "t_diff": 0.25,
        }],
        remappings=[
            ("/velodyne_points", lidar_topic),
            ("/gpsimu_driver/imu_data", imu_topic),
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py")
        ),
        condition=IfCondition(start_navigation),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": "False",
            "params_file": params_file,
            "autostart": "True",
            "map_subscribe_transient_local": "True",
        }.items(),
    )
    isolated_navigation = GroupAction([
        SetRemap(src="/cmd_vel", dst="/patrol/cmd_vel_raw"),
        # Foxy bt_navigator also accepts PoseStamped on /goal_pose. Block that
        # legacy entry point so every RViz request must pass through the
        # ARM-delayed /patrol/goal_request bridge and the action client.
        SetRemap(src="/goal_pose", dst="/patrol/nav2_direct_goal_blocked"),
        nav2_launch,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("globalmap_pcd"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("lidar_topic", default_value="/rslidar_points"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("start_localization", default_value="True"),
        DeclareLaunchArgument("start_navigation", default_value="True"),
        DeclareLaunchArgument("specify_init_pose", default_value="False"),
        DeclareLaunchArgument("use_imu", default_value="False"),
        DeclareLaunchArgument("convert_utm_to_local", default_value="False"),
        DeclareLaunchArgument("ndt_resolution", default_value="1.0"),
        DeclareLaunchArgument("downsample_resolution", default_value="0.3"),
        localization,
        isolated_navigation,
    ])
