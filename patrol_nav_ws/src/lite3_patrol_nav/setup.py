from setuptools import find_packages, setup


package_name = "lite3_patrol_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/patrol_navigation.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="duyipei",
    maintainer_email="duyipei@localhost",
    description="Guarded multi-waypoint patrol overlay for Lite3",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "front_camera_bridge = lite3_patrol_nav.rtsp_camera:main",
            "patrol_runner = lite3_patrol_nav.patrol_runner:main",
            "velocity_gate = lite3_patrol_nav.velocity_gate:main",
            "visual_pose_initializer = lite3_patrol_nav.visual_pose_initializer:main",
            "rgbd_visual_localizer = lite3_patrol_nav.rgbd_visual_localizer:main",
        ],
    },
)
