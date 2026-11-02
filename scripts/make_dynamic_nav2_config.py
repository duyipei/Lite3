#!/usr/bin/env python3
"""Opt-in live-obstacle overlay; leave the reviewed map and baseline untouched.

--factory-only keeps the vendor VOA camera grid in both costmaps and omits the
unverified Mid-360 ground-classification layer entirely.
"""
import argparse
import copy
from pathlib import Path
import yaml


FACTORY_VOA_BASE = {
    'plugin': 'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer',
    'transform_tolerance': 0.3,
    'enabled': True,
    'publish_voxel_map': False,
    'voxel_size': .05,
    'combination_method': 1,
    'mark_threshold': 0,
    'update_footprint_enabled': True,
    'track_unknown_space': False,
    'decay_model': 0,
    'voxel_decay': 1.5,
    'mapping_mode': False,
    'map_save_duration': 60.0,
}


def voa_layer(source=None):
    """Build the camera-grid STVL layer used in both integration modes.

    The layer consumes only the map-frame cloud from voa_costmap_bridge.py.
    Factory-only passes no source at all; it must not read any Mid-360 topic or
    ground-height setting, even as a template.
    """
    if source is None:
        layer = copy.deepcopy(FACTORY_VOA_BASE)
    else:
        layer = copy.deepcopy(source)
        layer.pop('rslidar_points', None)
    layer.update(enabled=True, observation_sources='voa_grid',
                 combination_method=1, decay_model=0, voxel_size=.05,
                 voxel_decay=1.5, mapping_mode=False,
                 publish_voxel_map=False)
    layer['voa_grid'] = dict(
        topic='/patrol/voa/costmap_obstacles',
        sensor_frame='patrol_voa_costmap_origin' if source is None else 'base_link',
        data_type='PointCloud2', observation_persistence=0.0,
        expected_update_rate=.5, clear_after_reading=True, enabled=True,
        min_obstacle_height=.4, max_obstacle_height=.6,
        marking=True, clearing=False, inf_is_valid=False,
        obstacle_range=4.0, filter='passthrough', voxel_min_points=0,
        model_type=0)
    return layer


def load_response_profile(path=None):
    path = path or Path(__file__).resolve().parents[1]/'navigation/obstacle_response.yaml'
    profile = yaml.safe_load(Path(path).read_text())
    limits = {'local_update_hz': (5., 10.), 'global_update_hz': (1., 3.),
              'costmap_publish_hz': (1., 5.), 'local_inflation_m': (.50, .65),
              'global_inflation_m': (.50, .65), 'goal_xy_tolerance_m': (.08, .15),
              'goal_extra_margin_m': (.03, .10), 'blocked_progress_timeout_s': (15.,45.)}
    for key, (low, high) in limits.items():
        if not low <= float(profile[key]) <= high:
            raise ValueError('response profile outside tested bounds: '+key)
    return profile


def build(config, behavior_tree, ground_relative=False, factory_only=False, response_profile=None):
    result = copy.deepcopy(config)
    local = result['local_costmap']['local_costmap']['ros__parameters']
    global_map = result['global_costmap']['global_costmap']['ros__parameters']
    controller = result['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    if factory_only and ground_relative:
        raise ValueError('factory-only mode has no LiDAR ground projection')
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
    # Factory-only keeps the vendor camera grid but deliberately withholds the
    # unverified Mid-360 ground classification from both costmaps. It uses a
    # self-contained VOA layer definition so no LiDAR topic, layer or ground
    # setting is read even as a template.
    stvl_source = None
    if factory_only:
        voa_template = voa_layer()
    else:
        stvl_source = local.get('stvl_obstacle_layer')
        if stvl_source is None:
            raise ValueError('baseline LiDAR obstacle layer is missing')
        if stvl_source['rslidar_points']['topic'] != '/aligned_points':
            raise ValueError('expected the existing map-aligned point cloud')
        voa_template = voa_layer(stvl_source)
    for costmap in (local, global_map):
        if factory_only:
            costmap.pop('stvl_obstacle_layer', None)
        else:
            layer = copy.deepcopy(stvl_source)
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
        costmap['voa_obstacle_layer'] = copy.deepcopy(voa_template)
        plugins = [p for p in costmap['plugins']
                   if p not in ('stvl_obstacle_layer', 'voa_obstacle_layer')]
        if 'static_layer' not in plugins or 'inflation_layer' not in plugins:
            raise ValueError('static map and inflation layers are required')
        if not factory_only:
            plugins.insert(plugins.index('inflation_layer'), 'stvl_obstacle_layer')
        plugins.insert(plugins.index('inflation_layer'), 'voa_obstacle_layer')
        costmap['plugins'] = plugins
        # More clearance for unclassified moving obstacles; keep footprint.
        # Slightly wider than the previous .55 m floor: the collision review
        # showed the leg sweep can contact obstacles before the inflated
        # footprint critic reacts. Keep this a small increase so narrow
        # corridors remain plannable.
        if factory_only:
            profile = load_response_profile(response_profile)
            key = 'local' if costmap is local else 'global'
            costmap['inflation_layer']['inflation_radius'] = profile[key+'_inflation_m']
            costmap['update_frequency'] = max(costmap['update_frequency'], profile[key+'_update_hz'])
            costmap['publish_frequency'] = profile['costmap_publish_hz']
        else:
            costmap['inflation_layer']['inflation_radius'] = max(
                .60, costmap['inflation_layer']['inflation_radius'])
    if factory_only:
        # The goal bridge reads this same effective overlay, not the baseline.
        profile = load_response_profile(response_profile)
        controller['goal_checker']['xy_goal_tolerance'] = profile['goal_xy_tolerance_m']
        follow['xy_goal_tolerance'] = profile['goal_xy_tolerance_m']
        # If a valid obstacle keeps the gate at zero, let Nav2's existing BT
        # stop/wait/replan earlier. Never clear costmaps, spin or reverse out.
        controller['progress_checker']['movement_time_allowance'] = profile['blocked_progress_timeout_s']
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
    parser.add_argument('--response-profile', type=Path)
    parser.add_argument('--factory-only', action='store_true',
                        help='VOA camera grid only; no Mid-360 obstacle layer')
    args = parser.parse_args()
    if args.base_yaml.resolve() == args.output_yaml.resolve():
        parser.error('output must not overwrite the baseline')
    if not args.behavior_tree.is_file():
        parser.error('behavior tree is missing')
    if args.factory_only and args.ground_relative:
        parser.error('--factory-only and --ground-relative are mutually exclusive')
    config = build(yaml.safe_load(args.base_yaml.read_text()),
                   args.behavior_tree.resolve(), args.ground_relative,
                   args.factory_only, args.response_profile)
    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    args.output_yaml.write_text(yaml.safe_dump(config, sort_keys=False))
    mode = 'factory-only' if args.factory_only else 'private-combined'
    print('[PASS] Dynamic obstacle overlay (' + mode + '): ' +
          str(args.output_yaml))


if __name__ == '__main__':
    main()
