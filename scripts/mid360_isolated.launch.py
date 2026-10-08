"""Round-2A read-only Mid-360 acquisition launch.

This starts only livox_ros_driver2.  It does not publish TF, start SLAM, or
connect anything to the robot motion stack.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


CONFIG = "/home/ysc/lite3_patrol_project/config/mid360_jetson_103.json"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            namespace="fast_livo2",
            name="mid360_driver",
            output="screen",
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "fast_livo2_lidar",
                "lvx_file_path": "",
                "user_config_path": CONFIG,
                "cmdline_input_bd_code": "livox0000000001",
            }],
        )
    ])

