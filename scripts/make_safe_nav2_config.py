#!/usr/bin/env python3
"""Create a low-speed Nav2 overlay from the installed vendor YAML."""

import argparse
import os

import yaml


def recursively_disable_sim_time(value):
    if isinstance(value, dict):
        for key, child in value.items():
            if key == "use_sim_time":
                value[key] = False
            else:
                recursively_disable_sim_time(child)
    elif isinstance(value, list):
        for child in value:
            recursively_disable_sim_time(child)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("vendor_yaml")
    parser.add_argument("map_yaml")
    parser.add_argument("output_yaml")
    args = parser.parse_args()
    with open(args.vendor_yaml, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    recursively_disable_sim_time(config)

    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    planner["allow_unknown"] = False
    planner["tolerance"] = 0.40
    controller = config["controller_server"]["ros__parameters"]
    controller["odom_topic"] = "/odom"
    controller["controller_frequency"] = 5.0
    controller["goal_checker"]["xy_goal_tolerance"] = 0.35
    # Patrol arrows are meaningful headings, especially before a corner.
    controller["goal_checker"]["yaw_goal_tolerance"] = 0.20
    follow = controller["FollowPath"]
    follow.update({
        # Turn and move forward; do not traverse the next leg backwards.
        "min_vel_x": 0.0,
        "min_vel_y": 0.0,
        "max_vel_x": 0.15,
        "max_vel_y": 0.0,
        "max_vel_theta": 0.40,
        "min_speed_xy": 0.0,
        "max_speed_xy": 0.15,
        "min_speed_theta": 0.04,
        "acc_lim_x": 0.20,
        "acc_lim_y": 0.10,
        "acc_lim_theta": 0.50,
        "decel_lim_x": -0.25,
        "decel_lim_y": -0.10,
        "decel_lim_theta": -0.50,
        "vx_samples": 11,
        "vy_samples": 1,
        "vtheta_samples": 31,
        "sim_time": 2.5,
        "critics": [
            "ObstacleFootprint",
            "Oscillation",
            "RotateToGoal",
            "PathAlign",
            "GoalAlign",
            "PathDist",
            "GoalDist",
        ],
        "xy_goal_tolerance": 0.35,
        "trans_stopped_velocity": 0.03,
        "RotateToGoal.scale": 32.0,
        "RotateToGoal.slowing_factor": 5.0,
        "RotateToGoal.lookahead_time": -1.0,
        "GoalDist.scale": 24.0,
        "PathAlign.scale": 64.0,
        "PathAlign.forward_point_distance": 0.10,
        "GoalAlign.scale": 48.0,
        "GoalAlign.forward_point_distance": 0.10,
        "PathDist.scale": 32.0,
        "ObstacleFootprint.scale": 0.02,
        "ObstacleFootprint.max_scaling_factor": 0.2,
        "ObstacleFootprint.scaling_speed": 0.15,
        "publish_evaluation": True,
        "publish_trajectories": False,
        "publish_cost_grid_pc": False,
    })

    global_costmap = config["global_costmap"]["global_costmap"]["ros__parameters"]
    global_costmap["footprint_padding"] = 0.05
    global_costmap["rolling_window"] = False
    global_costmap["track_unknown_space"] = True
    global_costmap["static_layer"]["track_unknown_space"] = True
    global_costmap["static_layer"]["unknown_cost_value"] = 255
    # The global map is the reviewed offline static map.  Live 3-D obstacles
    # belong only in the rolling local costmap; keeping STVL global can leave
    # stale remote obstacles and unnecessarily expand resource use.
    global_costmap["plugins"] = [
        plugin for plugin in global_costmap["plugins"]
        if plugin != "stvl_obstacle_layer"
    ]
    global_costmap.pop("stvl_obstacle_layer", None)
    local_costmap = config["local_costmap"]["local_costmap"]["ros__parameters"]
    local_costmap["footprint_padding"] = 0.05
    local_costmap["track_unknown_space"] = True
    obstacle_layer = local_costmap["stvl_obstacle_layer"]
    obstacle_layer["voxel_decay"] = 1.0
    obstacle_layer["rslidar_points"].update({
        "topic": "/aligned_points",
        "sensor_frame": "rslidar",
        "filter": "passthrough",
        "voxel_min_points": 0,
        "observation_persistence": 0.0,
        "clear_after_reading": True,
        "expected_update_rate": 0.5,
    })
    local_plugins = local_costmap["plugins"]
    if "static_layer" not in local_plugins:
        local_plugins.insert(0, "static_layer")

    recoveries = config["recoveries_server"]["ros__parameters"]
    recoveries["recovery_plugins"] = ["wait"]
    recoveries.pop("spin", None)
    recoveries.pop("backup", None)
    config["map_server"]["ros__parameters"]["yaml_filename"] = os.path.abspath(args.map_yaml)
    config["waypoint_follower"] = {"ros__parameters": {
        "use_sim_time": False,
        "loop_rate": 10,
        "stop_on_failure": True,
    }}

    os.makedirs(os.path.dirname(os.path.abspath(args.output_yaml)), exist_ok=True)
    with open(args.output_yaml, "w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    print(os.path.abspath(args.output_yaml))


if __name__ == "__main__":
    main()
