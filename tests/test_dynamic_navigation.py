"""Offline checks: independent baseline, live-obstacle route and gated braking."""
import copy
import os
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

PROFILE = load('dynamic_profile', 'scripts/make_dynamic_nav2_config.py')
PROTOCOL = load('brake_protocol', 'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav/voa_protocol.py')
BUDGET = load('recovery_budget', 'scripts/navigation_recovery_budget.py')
BASE = ROOT / 'navigation/outdoor_20260921_003_area_review/nav2_safe.yaml'
TREE = ROOT / 'navigation/navigate_dynamic_obstacles.xml'


class DynamicNavigationTests(unittest.TestCase):
    def setUp(self):
        self.base = yaml.safe_load(BASE.read_text())

    def test_overlay_preserves_baseline_and_motion_acceptance(self):
        before = copy.deepcopy(self.base)
        result = PROFILE.build(self.base, TREE)
        self.assertEqual(self.base, before)
        for key in ('controller_server', 'planner_server', 'map_server'):
            self.assertEqual(result[key], self.base[key])
        for key in ('local_costmap', 'global_costmap'):
            original = self.base[key][key]['ros__parameters']
            changed = result[key][key]['ros__parameters']
            self.assertEqual(original['footprint'], changed['footprint'])
            self.assertEqual(original['footprint_padding'], changed['footprint_padding'])
            self.assertEqual(original['static_layer'], changed['static_layer'])

    def test_live_obstacles_affect_global_and_local_routes_then_expire(self):
        result = PROFILE.build(self.base, TREE)
        for key in ('local_costmap', 'global_costmap'):
            costmap = result[key][key]['ros__parameters']
            plugins = costmap['plugins']
            self.assertLess(plugins.index('static_layer'), plugins.index('stvl_obstacle_layer'))
            self.assertLess(plugins.index('stvl_obstacle_layer'), plugins.index('inflation_layer'))
            layer = costmap['stvl_obstacle_layer']
            self.assertEqual(layer['combination_method'], 1)
            self.assertEqual(layer['decay_model'], 0)
            self.assertGreater(layer['voxel_decay'], 0)
            self.assertFalse(layer['mapping_mode'])
            self.assertEqual(layer['rslidar_points']['topic'], '/aligned_points')
            self.assertTrue(layer['rslidar_points']['marking'])

    def test_recovery_waits_without_erasing_obstacles_or_blind_motion(self):
        tree = ET.parse(TREE)
        tags = {node.tag for node in tree.iter()}
        self.assertTrue({'ComputePathToPose', 'FollowPath', 'Wait', 'RateController'} <= tags)
        self.assertFalse({'ClearEntireCostmap', 'Spin', 'BackUp'} & tags)
        self.assertEqual(tree.find('.//RateController').attrib['hz'], '1.0')

    def test_voa_obstacles_reach_both_costmaps_with_independent_expiry(self):
        result = PROFILE.build(self.base, TREE)
        for key in ('local_costmap', 'global_costmap'):
            costmap = result[key][key]['ros__parameters']
            plugins = costmap['plugins']
            self.assertLess(plugins.index('static_layer'), plugins.index('voa_obstacle_layer'))
            self.assertLess(plugins.index('voa_obstacle_layer'), plugins.index('inflation_layer'))
            layer = costmap['voa_obstacle_layer']
            self.assertEqual(layer['observation_sources'], 'voa_grid')
            self.assertEqual(layer['combination_method'], 1)
            self.assertEqual(layer['voxel_size'], .05)
            self.assertGreater(layer['voxel_decay'], 1/costmap['update_frequency'])
            self.assertEqual(layer['decay_model'], 0)
            source = layer['voa_grid']
            self.assertEqual(source['topic'], '/patrol/voa/costmap_obstacles')
            self.assertEqual(source['sensor_frame'], 'base_link')
            self.assertTrue(source['marking'])
            self.assertFalse(source['clearing'])
            self.assertLess(source['min_obstacle_height'], .5)
            self.assertGreater(source['max_obstacle_height'], .5)
            self.assertEqual(source['observation_persistence'], 0)
            self.assertTrue(source['clear_after_reading'])
            self.assertEqual(source['expected_update_rate'], .5)

    def test_command_wait_budget_tracks_behavior_tree_changes(self):
        self.assertEqual(BUDGET.command_wait_budget(TREE), 60.)
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory)/'tree.xml'
            changed.write_text(TREE.read_text().replace('wait_duration="3"', 'wait_duration="10"'))
            self.assertEqual(BUDGET.command_wait_budget(changed), 170.)
            changed.write_text(TREE.read_text().replace('wait_duration="3"', 'wait_duration="nan"'))
            with self.assertRaises(ValueError):
                BUDGET.command_wait_budget(changed)
            changed.write_text(TREE.read_text().replace('<Wait wait_duration="3"/>',
                '<Sequence><Wait wait_duration="3"/><Wait wait_duration="3"/></Sequence>'))
            with self.assertRaises(ValueError):
                BUDGET.command_wait_budget(changed)

    def test_ground_projection_changes_only_observation_topic(self):
        original = PROFILE.build(self.base, TREE)
        projected = PROFILE.build(self.base, TREE, ground_relative=True)
        for key in ('local_costmap', 'global_costmap'):
            source = projected[key][key]['ros__parameters']['stvl_obstacle_layer']['rslidar_points']
            self.assertEqual(source['topic'], '/patrol/costmap_obstacles')
            source['topic'] = '/aligned_points'
        self.assertEqual(projected, original)

    def test_actual_launcher_selects_relative_ground_for_both_maps(self):
        source = (ROOT/'navigation/start_nav_isolated.sh').read_text()
        fragment = source.split('PARAMS_FILE=$MAP_DIR/nav2_safe.yaml', 1)[1].split('nav_pid=', 1)[0]
        # Run only the configuration fragment in an isolated temp project.
        # No sensing, ROS processes, SSH, or navigation commands are executed.
        with tempfile.TemporaryDirectory() as d:
            project = Path(d)
            (project/'scripts').symlink_to(ROOT/'scripts')
            (project/'navigation').mkdir()
            (project/'navigation/navigate_dynamic_obstacles.xml').symlink_to(TREE)
            for map_id in ['indoor_fusion_20260912_001_area_review', 'outdoor_20260921_003_area_review']:
                directory = project/'navigation'/map_id; directory.mkdir()
                for name in ['nav2_safe.yaml', 'patrol_map.yaml', 'traversability_summary.json', 'traversability_evidence.npz']:
                    (directory/name).symlink_to(ROOT/'navigation'/map_id/name)
                command = 'set -eu\nPROJECT=$1\nMAP_ID=$2\nMAP_DIR=$PROJECT/navigation/$MAP_ID\n' + fragment
                run = subprocess.run(['bash','-c',command,'fixture',str(project),map_id],
                    env=dict(os.environ,LITE3_DYNAMIC_OBSTACLES='1'),capture_output=True,text=True)
                self.assertEqual(run.returncode,0,run.stderr)
                config=yaml.safe_load((directory/'nav2_dynamic.yaml').read_text())
                for layer in ['local_costmap','global_costmap']:
                    self.assertEqual(config[layer][layer]['ros__parameters']['stvl_obstacle_layer']['rslidar_points']['topic'],
                                     '/patrol/costmap_obstacles')
                (directory/'traversability_evidence.npz').unlink()
                run = subprocess.run(['bash','-c',command,'fixture',str(project),map_id],
                    env=dict(os.environ,LITE3_DYNAMIC_OBSTACLES='1'),capture_output=True,text=True)
                self.assertNotEqual(run.returncode,0)

    def test_generator_cli_and_no_baseline_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'dynamic.yaml'
            command = ['python3', str(ROOT/'scripts/make_dynamic_nav2_config.py'),
                       str(BASE), str(output), '--behavior-tree', str(TREE)]
            subprocess.run(command, check=True, capture_output=True)
            self.assertEqual(yaml.safe_load(output.read_text()), PROFILE.build(self.base, TREE))
            command[3] = str(BASE)
            rejected = subprocess.run(command, capture_output=True)
            self.assertNotEqual(rejected.returncode, 0)

    def test_unsupported_motion_or_missing_collision_checks_rejected(self):
        for field, value in [('min_vel_x', -.2), ('max_vel_x', .5),
                             ('max_vel_y', .1), ('max_vel_theta', .8), ('critics', [])]:
            config = copy.deepcopy(self.base)
            config['controller_server']['ros__parameters']['FollowPath'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                PROFILE.build(config, TREE)

    def test_indoor_profile_disables_reverse_without_changing_targets_or_baseline(self):
        path = ROOT/'navigation/indoor_fusion_20260912_001_area_review/nav2_safe.yaml'
        base = yaml.safe_load(path.read_text())
        original = copy.deepcopy(base)
        result = PROFILE.build(base, TREE)
        self.assertEqual(base, original)
        controller = result['controller_server']['ros__parameters']
        self.assertEqual(controller['FollowPath']['min_vel_x'], 0.0)
        self.assertEqual(controller['goal_checker'], base['controller_server']['ros__parameters']['goal_checker'])
        self.assertEqual(controller['FollowPath']['max_vel_x'], .15)
        follow = controller['FollowPath']
        self.assertGreater(follow['PathAlign.forward_point_distance'], follow['xy_goal_tolerance'] + .10)
        self.assertEqual(follow['PathAlign.scale'], 32.)
        self.assertEqual(follow['GoalDist.scale'], 48.)
        for key in ('critics', 'ObstacleFootprint.scale', 'max_vel_theta',
                    'RotateToGoal.scale', 'xy_goal_tolerance', 'GoalAlign.scale'):
            self.assertEqual(follow[key], base['controller_server']['ros__parameters']['FollowPath'][key])

    def test_curvature_preserving_slowdown(self):
        brake = PROTOCOL.braking_correction
        self.assertEqual(brake((.1, 0, .2), (.05, 0, .1)), (.05, 0, .1))
        self.assertEqual(brake((0, 0, -.4), (0, 0, -.2)), (0, 0, -.2))
        self.assertEqual(brake((.1, 0, 0), (0, 0, 0)), (0, 0, 0))

    def test_no_steering_override_acceleration_reverse_or_nan(self):
        for proposal in ((.1, 0, .2), (.15, 0, 0), (-.1, 0, 0),
                         (.1, .1, 0), (float('nan'), 0, 0)):
            with self.subTest(proposal=proposal):
                self.assertEqual(PROTOCOL.braking_correction((.1, 0, 0), proposal), (0, 0, 0))
        self.assertEqual(PROTOCOL.braking_correction((0, 0, 0), (.1, 0, 0)), (0, 0, 0))

    def test_experimental_refinement_cannot_start_patrol(self):
        result = subprocess.run(['bash', str(ROOT/'scripts/navigation_start.sh'), 'start'],
            env=dict(os.environ,LITE3_NDT_FINE_REFINE='true'),
            text=True,capture_output=True,timeout=5)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('只允许 relocalize', result.stderr)

    def test_dynamic_mode_rejects_disabled_voa_before_connecting(self):
        result = subprocess.run(['bash', str(ROOT/'scripts/navigation_start.sh'), 'preflight'],
            env=dict(os.environ, LITE3_DYNAMIC_OBSTACLES='1', LITE3_USE_VENDOR_VOA='0'),
            text=True, capture_output=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('需要 VOA', result.stderr)

    def test_existing_entry_remains_opt_in(self):
        env = dict(os.environ)
        env.pop('LITE3_DYNAMIC_OBSTACLES', None)
        env.pop('LITE3_USE_VENDOR_VOA', None)
        command = 'entry=$1; set -- --help; source "$entry" >/dev/null; echo "$DYNAMIC_OBSTACLES:$USE_VENDOR_VOA"'
        result = subprocess.run(['bash', '-c', command, 'test', str(ROOT/'scripts/navigation_start.sh')],
            env=env, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '0:0')

    def test_wrapper_defaults_to_dynamic_outdoor_and_does_not_arm(self):
        result = subprocess.run(['bash', str(ROOT/'scripts/navigation_dynamic.sh'), '--help'],
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('LITE3_DYNAMIC_OBSTACLES=1', result.stdout)
        self.assertIn('DISARMED', result.stdout)


if __name__ == '__main__':
    unittest.main()
