"""Launch only an automatically activated static map server for RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    map_server_name = "manual_initial_pose_map_server"

    return LaunchDescription(
        [
            DeclareLaunchArgument("map_yaml"),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name=map_server_name,
                output="screen",
                parameters=[{"yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="manual_initial_pose_lifecycle_manager",
                output="screen",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [map_server_name],
                    }
                ],
            ),
        ]
    )
