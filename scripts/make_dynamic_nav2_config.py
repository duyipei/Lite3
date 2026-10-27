#!/usr/bin/env python3
"""Opt-in live-obstacle overlay; leave the reviewed map and baseline untouched."""
import argparse
import copy
from pathlib import Path
import yaml


def build(config, behavior_tree, ground_relative=False):
    result = copy.deepcopy(config)
    local = result['local_costmap']['local_costmap']['ros__parameters']
    global_map = result['global_costmap']['global_costmap']['ros__parameters']
    controller = result['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    if 'ObstacleFootprint' not in follow['critics']:
        raise ValueError('footprint collision critic is required')
    if not (-.08 <= follow['min_vel_x'] <= follow['max_vel_x'] <= .15
            and follow['max_vel_x'] > 0):
        raise ValueError('unsupported baseline speed limits')
    # The indoor baseline allows reverse. This private VOA integration has
    # forward geometry only: narrow its command range without editing baseline.
    reverse_baseline = follow['min_vel_x'] < 0
    follow['min_vel_x'] = max(0.0, follow['min_vel_x'])
    if reverse_baseline:
        # The indoor baseline deliberately uses a short nose projection for
        # reverse tracking. With reverse disabled, use the existing outdoor
        # forward-tracking balance: reward heading/progress without changing
        # terminal yaw, footprint checks, speeds or trajectory sampling.
        follow['PathAlign.forward_point_distance'] = .325
        follow['PathAlign.scale'] = 32.0
        follow['GoalDist.scale'] = 48.0
    if follow['min_vel_y'] != 0 or follow['max_vel_y'] != 0:
        raise ValueError('dynamic mode does not support lateral motion')
    if not (0 < follow['max_vel_theta'] <= .4):
        raise ValueError('angular speed must be <= 0.4 rad/s')
    source = local['stvl_obstacle_layer']
    if source['rslidar_points']['topic'] != '/aligned_points':
        raise ValueError('expected the existing map-aligned point cloud')
    for costmap in (local, global_map):
        layer = copy.deepcopy(source)
        # Linear expiry releases space after a moving object has left. Never
        # erase the static map: max-combination overlays live observations.
        layer.update(enabled=True, combination_method=1, decay_model=0,
                     voxel_decay=1.0, mapping_mode=False, publish_voxel_map=False)
        layer['rslidar_points'].update(observation_persistence=0.0,
                                      clear_after_reading=True,
                                      expected_update_rate=0.5)
        if ground_relative:
            # Private 2-D projection has already applied the map's saved
            # ground-relative height band; its synthetic Z is 0.5 m.
            layer['rslidar_points']['topic'] = '/patrol/costmap_obstacles'
        costmap['stvl_obstacle_layer'] = layer
        # Use exactly the finite obstacle cells that VOA sees, after conversion
        # into map coordinates. A separate short-lived grid retains the camera's
        # 5 cm cells without changing the existing LiDAR ground/height handling.
        voa = copy.deepcopy(layer)
        voa.pop('rslidar_points')
        voa.update(observation_sources='voa_grid', voxel_size=.05, voxel_decay=1.5)
        voa['voa_grid'] = dict(
            topic='/patrol/voa/costmap_obstacles', sensor_frame='base_link',
            data_type='PointCloud2', observation_persistence=0.0,
            expected_update_rate=.5, clear_after_reading=True, enabled=True,
            min_obstacle_height=.4, max_obstacle_height=.6,
            marking=True, clearing=False, inf_is_valid=False,
            obstacle_range=4.0, filter='passthrough', voxel_min_points=0,
            model_type=0)
        costmap['voa_obstacle_layer'] = voa
        plugins = [p for p in costmap['plugins']
                   if p not in ('stvl_obstacle_layer', 'voa_obstacle_layer')]
        if 'static_layer' not in plugins or 'inflation_layer' not in plugins:
            raise ValueError('static map and inflation layers are required')
        plugins.insert(plugins.index('inflation_layer'), 'stvl_obstacle_layer')
        plugins.insert(plugins.index('inflation_layer'), 'voa_obstacle_layer')
        costmap['plugins'] = plugins
        # More clearance for unclassified moving obstacles; keep footprint.
        costmap['inflation_layer']['inflation_radius'] = max(
            .55, costmap['inflation_layer']['inflation_radius'])
    navigator = result['bt_navigator']['ros__parameters']
    navigator['default_bt_xml_filename'] = str(behavior_tree)
    for plugin in ('nav2_rate_controller_bt_node', 'nav2_pipeline_sequence_bt_node',
                   'nav2_recovery_node_bt_node', 'nav2_wait_action_bt_node'):
        if plugin not in navigator['plugin_lib_names']:
            navigator['plugin_lib_names'].append(plugin)
    # Wait and replan; never clear observed obstacles just to force progress.
    recovery = result['recoveries_server']['ros__parameters']
    recovery['recovery_plugins'] = ['wait']
    recovery.pop('spin', None)
    recovery.pop('backup', None)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('base_yaml', type=Path)
    parser.add_argument('output_yaml', type=Path)
    parser.add_argument('--behavior-tree', required=True, type=Path)
    parser.add_argument('--ground-relative', action='store_true')
    args = parser.parse_args()
    if args.base_yaml.resolve() == args.output_yaml.resolve():
        parser.error('output must not overwrite the baseline')
    if not args.behavior_tree.is_file():
        parser.error('behavior tree is missing')
    config = build(yaml.safe_load(args.base_yaml.read_text()), args.behavior_tree.resolve(), args.ground_relative)
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(yaml.safe_dump(config, sort_keys=False))
    print('[PASS] Dynamic obstacle overlay: ' + str(args.output_yaml))


if __name__ == '__main__':
    main()
