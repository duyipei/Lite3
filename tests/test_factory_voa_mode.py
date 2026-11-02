"""Factory Only regressions: config, velocity gate, launchers, no motion.

These tests run the actual configuration builder, the actual velocity-gate
module and the actual shell supervisors. They do not connect to the robot,
start ROS, publish a velocity or make any network call.
"""
import copy
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace as NS
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'patrol_nav_ws/src/lite3_patrol_nav/lite3_patrol_nav'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PROFILE = load('factory_profile', 'scripts/make_dynamic_nav2_config.py')
TREE = ROOT / 'navigation/navigate_dynamic_obstacles.xml'
INDOOR = ROOT / 'navigation/indoor_fusion_20260912_001_area_review/nav2_safe.yaml'
OUTDOOR = ROOT / 'navigation/outdoor_20260921_003_area_review/nav2_safe.yaml'


class FactoryConfigTests(unittest.TestCase):
    def setUp(self):
        self.base = yaml.safe_load(INDOOR.read_text())

    def test_both_costmaps_have_voa_and_no_mid360_layer(self):
        before = copy.deepcopy(self.base)
        result = PROFILE.build(self.base, TREE, factory_only=True)
        self.assertEqual(self.base, before)
        text = yaml.safe_dump(result)
        self.assertNotIn('/aligned_points', text)
        self.assertNotIn('/patrol/costmap_obstacles', text)
        self.assertNotIn('stvl_obstacle_layer', text)
        for key in ('local_costmap', 'global_costmap'):
            costmap = result[key][key]['ros__parameters']
            self.assertEqual(costmap['plugins'],
                             ['static_layer', 'voa_obstacle_layer', 'inflation_layer'])
            self.assertIn('static_layer', costmap)
            self.assertIn('inflation_layer', costmap)
            layer = costmap['voa_obstacle_layer']
            self.assertEqual(layer['observation_sources'], 'voa_grid')
            self.assertEqual(layer['voa_grid']['topic'],
                             '/patrol/voa/costmap_obstacles')
            self.assertEqual(layer['voxel_size'], .05)
            self.assertEqual(layer['combination_method'], 1)
            self.assertTrue(layer['voa_grid']['marking'])
            self.assertFalse(layer['voa_grid']['clearing'])
        controller = result['controller_server']['ros__parameters']['FollowPath']
        original = self.base['controller_server']['ros__parameters']['FollowPath']
        self.assertIn('ObstacleFootprint', controller['critics'])
        self.assertEqual(controller['max_vel_x'], original['max_vel_x'])
        self.assertEqual(controller['max_vel_theta'], original['max_vel_theta'])
        navigator = result['bt_navigator']['ros__parameters']
        self.assertEqual(navigator['default_bt_xml_filename'], str(TREE))
        recovery = result['recoveries_server']['ros__parameters']
        self.assertEqual(recovery['recovery_plugins'], ['wait'])

    def test_factory_only_reads_no_lidar_layer_template(self):
        without_stvl = copy.deepcopy(self.base)
        del without_stvl['local_costmap']['local_costmap']['ros__parameters']['stvl_obstacle_layer']
        result = PROFILE.build(without_stvl, TREE, factory_only=True)
        self.assertNotIn('stvl_obstacle_layer',
                         result['local_costmap']['local_costmap']['ros__parameters'])
        with self.assertRaises(ValueError):
            PROFILE.build(without_stvl, TREE)

    def test_factory_only_and_ground_relative_are_mutually_exclusive(self):
        for base in (INDOOR, OUTDOOR):
            with self.assertRaises(ValueError):
                PROFILE.build(yaml.safe_load(base.read_text()), TREE,
                              ground_relative=True, factory_only=True)

    def test_cli_factory_only_and_baseline_protection(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'factory.yaml'
            base = yaml.safe_load(INDOOR.read_text())
            subprocess.run([
                'python3', str(ROOT / 'scripts/make_dynamic_nav2_config.py'),
                str(INDOOR), str(output), '--behavior-tree', str(TREE),
                '--factory-only'], check=True, capture_output=True)
            self.assertEqual(yaml.safe_load(output.read_text()),
                             PROFILE.build(base, TREE, factory_only=True))
            rejected = subprocess.run([
                'python3', str(ROOT / 'scripts/make_dynamic_nav2_config.py'),
                str(INDOOR), str(output), '--behavior-tree', str(TREE),
                '--factory-only', '--ground-relative'], capture_output=True)
            self.assertNotEqual(rejected.returncode, 0)
            rejected = subprocess.run([
                'python3', str(ROOT / 'scripts/make_dynamic_nav2_config.py'),
                str(INDOOR), str(INDOOR), '--behavior-tree', str(TREE),
                '--factory-only'], capture_output=True)
            self.assertNotEqual(rejected.returncode, 0)

    def test_private_combined_keeps_lidar_and_voa_layers(self):
        result = PROFILE.build(self.base, TREE)
        for key in ('local_costmap', 'global_costmap'):
            costmap = result[key][key]['ros__parameters']
            self.assertIn('stvl_obstacle_layer', costmap)
            self.assertIn('voa_obstacle_layer', costmap)
            self.assertEqual(costmap['stvl_obstacle_layer']['rslidar_points']['topic'],
                             '/aligned_points')
            self.assertLess(costmap['plugins'].index('stvl_obstacle_layer'),
                            costmap['plugins'].index('voa_obstacle_layer'))
            self.assertLess(costmap['plugins'].index('voa_obstacle_layer'),
                            costmap['plugins'].index('inflation_layer'))


def launcher_fragment():
    source = (ROOT / 'navigation/start_nav_isolated.sh').read_text()
    return source.split('PARAMS_FILE=$MAP_DIR/nav2_safe.yaml', 1)[1].split('nav_pid=', 1)[0]


class FactoryLauncherConfigTests(unittest.TestCase):
    def run_fragment(self, root, backend, ground_mode='map'):
        command = ('set -eu\nPROJECT=$1\nMAP_ID=$2\nMAP_DIR=$PROJECT/navigation/$MAP_ID\n'
                   + launcher_fragment())
        return subprocess.run(
            ['bash', '-c', command, 'fixture', str(root),
             'indoor_fusion_20260912_001_area_review'],
            env=dict(os.environ, LITE3_DYNAMIC_OBSTACLES='1',
                     LITE3_OBSTACLE_BACKEND=backend, LITE3_GROUND_MODE=ground_mode),
            capture_output=True, text=True, timeout=10)

    def test_factory_launcher_needs_no_ground_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'scripts').symlink_to(ROOT / 'scripts')
            (root / 'navigation').mkdir()
            (root / 'navigation/navigate_dynamic_obstacles.xml').symlink_to(TREE)
            map_dir = root / 'navigation/indoor_fusion_20260912_001_area_review'
            map_dir.mkdir()
            (map_dir / 'nav2_safe.yaml').symlink_to(INDOOR)
            run = self.run_fragment(root, 'factory_only')
            self.assertEqual(run.returncode, 0, run.stderr)
            config = yaml.safe_load((map_dir / 'nav2_dynamic.yaml').read_text())
            for key in ('local_costmap', 'global_costmap'):
                costmap = config[key][key]['ros__parameters']
                self.assertEqual(costmap['plugins'],
                                 ['static_layer', 'voa_obstacle_layer', 'inflation_layer'])
            self.assertIn('factory_only', run.stdout)

    def test_private_combined_launcher_still_requires_ground_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'scripts').symlink_to(ROOT / 'scripts')
            (root / 'navigation').mkdir()
            (root / 'navigation/navigate_dynamic_obstacles.xml').symlink_to(TREE)
            map_dir = root / 'navigation/indoor_fusion_20260912_001_area_review'
            map_dir.mkdir()
            (map_dir / 'nav2_safe.yaml').symlink_to(INDOOR)
            run = self.run_fragment(root, 'private_combined')
            self.assertNotEqual(run.returncode, 0)
            self.assertIn('ground', run.stderr.lower())


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.params = {}
        self.outputs = {}
        self.clock_time = 10.
        self.counts = {
            '/cmd_vel': 0,
            '/cmd_vel_corrected': 1,
            '/patrol/voa/corrected': 0,
            '/patrol/voa/input': 0,
        }

    def declare_parameter(self, key, value):
        self.params[key] = value

    def get_parameter(self, key):
        return NS(value=self.params[key])

    def create_publisher(self, kind, topic, qos):
        self.outputs[topic] = []
        return NS(publish=lambda msg: self.outputs[topic].append(copy.deepcopy(msg)))

    def create_subscription(self, *args):
        pass

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def count_publishers(self, topic):
        return self.counts.get(topic, 0)

    def get_logger(self):
        return NS(info=lambda s: None, warn=lambda s: None, error=lambda s: None)

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: NS(sec=10, nanosec=0)))


class Twist:
    def __init__(self):
        self.linear = NS(x=0., y=0., z=0.)
        self.angular = NS(x=0., y=0., z=0.)


class Stamped:
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.twist = Twist()


def module(name, **kwargs):
    result = types.ModuleType(name)
    result.__dict__.update(kwargs)
    return result


class FactoryGateTests(unittest.TestCase):
    def setUp(self):
        fake = {
            'rclpy': module('rclpy'),
            'rclpy.node': module('rclpy.node', Node=FakeNode),
            'rclpy.qos': module(
                'rclpy.qos', QoSProfile=lambda **kw: NS(),
                DurabilityPolicy=NS(TRANSIENT_LOCAL=1),
                ReliabilityPolicy=NS(RELIABLE=1, BEST_EFFORT=2)),
            'geometry_msgs.msg': module('geometry_msgs.msg', Twist=Twist,
                                        TwistStamped=Stamped),
            'nav_msgs.msg': module('nav_msgs.msg', Odometry=NS),
            'std_msgs.msg': module('std_msgs.msg', Bool=NS, Empty=NS,
                                   Header=NS, String=NS),
            'std_srvs.srv': module('std_srvs.srv', SetBool=NS, Trigger=NS),
            'factory_gate_test': module('factory_gate_test',
                                        __path__=[str(PACKAGE)]),
        }
        self.patch = patch.dict(sys.modules, fake)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        spec = importlib.util.spec_from_file_location(
            'factory_gate_test.velocity_gate', PACKAGE / 'velocity_gate.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.gate = self.module.VelocityGate()
        self.gate.now_monotonic = lambda: self.gate.clock_time
        self.gate.localization_ready = True
        self.gate.last_ready_time = self.gate.last_odom_time = \
            self.gate.last_lidar_time = 10.
        self.gate.require_voa_costmap = True
        self.gate.counts.update({
            '/patrol/voa/costmap_heartbeat': 1,
            '/patrol/voa/costmap_obstacles': 1,
        })

    def output(self):
        v = self.gate.outputs['/cmd_vel_corrected'][-1]
        return v.linear.x, v.linear.y, v.angular.z

    def heartbeat(self, stamp=10.):
        self.gate.voa_costmap_callback(NS(
            frame_id='map',
            stamp=NS(sec=int(stamp), nanosec=round((stamp - int(stamp)) * 1e9))))

    def arm(self):
        self.gate.goal_pending = True
        return self.gate.arm_callback(NS(data=True), NS())

    def test_factory_perception_does_not_need_private_voa_protocol(self):
        self.assertFalse(self.gate.use_vendor_voa)
        self.assertNotIn('/patrol/voa/input', self.gate.outputs)
        self.assertNotIn('/patrol/voa/corrected', self.gate.outputs)
        self.gate.timer_callback()
        self.assertFalse(self.arm().success)
        self.assertEqual(self.gate.sensor_health()[1],
                         'voa_costmap_not_ready_or_stale')
        self.heartbeat()
        self.assertTrue(self.arm().success)
        self.gate.goal_active_callback(NS(data=True))
        command = Twist()
        command.linear.x = .1
        self.gate.raw_callback(command)
        self.gate.timer_callback()
        self.assertEqual(self.output(), (.1, 0, 0))

    def test_factory_perception_timeout_stops_and_latches(self):
        self.heartbeat()
        self.assertTrue(self.arm().success)
        self.gate.goal_active_callback(NS(data=True))
        command = Twist()
        command.linear.x = .1
        self.gate.raw_callback(command)
        self.gate.timer_callback()
        self.assertEqual(self.output(), (.1, 0, 0))
        self.gate.clock_time = 10.51
        self.gate.last_ready_time = self.gate.last_odom_time = \
            self.gate.last_lidar_time = 10.51
        self.gate.timer_callback()
        self.assertFalse(self.gate.armed)
        self.assertEqual(self.gate.fault, 'voa_costmap_not_ready_or_stale')
        self.assertEqual(self.output(), (0, 0, 0))
        self.heartbeat(10.51)
        self.assertFalse(self.arm().success)  # latched; no automatic re-arm

    def test_public_command_and_corrected_output_conflicts_block_arm(self):
        self.heartbeat()
        self.gate.counts['/cmd_vel'] = 1
        self.assertEqual(self.gate.sensor_health()[1],
                         'public_cmd_vel_publisher_conflict')
        self.assertFalse(self.arm().success)
        self.gate.counts['/cmd_vel'] = 0
        self.gate.counts['/cmd_vel_corrected'] = 2
        self.assertEqual(self.gate.sensor_health()[1],
                         'corrected_command_publisher_conflict')
        self.assertFalse(self.arm().success)

    def test_bridge_publisher_conflict_blocks_arm(self):
        self.heartbeat()
        self.gate.counts['/patrol/voa/costmap_obstacles'] = 2
        self.assertEqual(self.gate.sensor_health()[1],
                         'voa_costmap_publisher_conflict')

    def test_two_nav2_raw_publishers_are_allowed_but_third_is_conflict(self):
        # Foxy Nav2 creates controller_server and recoveries_server publishers
        # on the remapped raw command topic; both are expected.
        self.heartbeat()
        self.gate.counts['/patrol/cmd_vel_raw'] = 2
        self.assertTrue(self.gate.sensor_health()[0])
        self.gate.counts['/patrol/cmd_vel_raw'] = 3
        self.assertEqual(self.gate.sensor_health()[1],
                         'nav_command_publisher_conflict')
        self.assertFalse(self.arm().success)


def write_executable(path, text):
    path.write_text(text)
    path.chmod(0o755)


SLEEPING = ('import os, time\n'
            'print("PID=" + str(os.getpid()), flush=True)\n'
            'print("ARGS=" + " ".join(__import__("sys").argv[1:]), flush=True)\n'
            'time.sleep(30)\n')


class FactoryVoaLauncherTests(unittest.TestCase):
    def test_vendor_command_topics_are_isolated_and_children_cleaned(self):
        with tempfile.TemporaryDirectory(prefix='factory_voa_') as directory:
            root = Path(directory)
            (root / 'bin').mkdir()
            (root / 'navigation').mkdir()
            (root / 'scripts').mkdir()
            (root / 'navigation/patrol_env.sh').write_text(
                'export PATH="%s/bin:$PATH"\n' % root)
            (root / 'navigation/vendor_voa.yaml').write_text('true\n')
            (root / 'scripts/voa_camera_tf.py').write_text(SLEEPING)
            (root / 'scripts/voa_costmap_bridge.py').write_text(
                'import os, time\n'
                'print("BRIDGE_PID=" + str(os.getpid()), flush=True)\n'
                'print("BRIDGE_STARTED", flush=True)\n'
                'time.sleep(.2)\n'
                'raise SystemExit(7)\n')
            prefix = root / 'factory/install/voa'
            executable = prefix / 'lib/voa/voa_composition'
            executable.parent.mkdir(parents=True)
            executable.write_text('#!/usr/bin/env python3\n' + SLEEPING)
            executable.chmod(0o755)
            setup = root / 'factory/install/setup.bash'
            setup.parent.mkdir(parents=True, exist_ok=True)
            setup.write_text('export FAKE_VOA_PREFIX="%s"\n'
                             'export PATH="%s/bin:$PATH"\n'
                             % (prefix, root))
            write_executable(root / 'bin/ros2', (
                '#!/bin/sh\n'
                'if [ "$1" = pkg ] && [ "$2" = prefix ] && [ "$3" = voa ]; then\n'
                '  echo "$FAKE_VOA_PREFIX"; exit 0\n'
                'fi\n'
                'exit 1\n'))
            write_executable(root / 'bin/systemctl', '#!/bin/sh\nexit 3\n')
            write_executable(root / 'bin/pgrep', '#!/bin/sh\nexit 1\n')
            launcher = root / 'launcher.sh'
            launcher.write_text(
                (ROOT / 'navigation/start_factory_voa.sh').read_text().replace(
                    'PROJECT=/home/ysc/lite3_patrol_project',
                    'PROJECT=' + str(root)))
            env = dict(os.environ, PATH=str(root / 'bin') + ':' + os.environ['PATH'],
                       LITE3_FACTORY_VOA_SETUP=str(setup),
                       LITE3_GRID_MAP_SETUP='/nonexistent')
            check = subprocess.run(['bash', str(launcher), '--check'], env=env,
                                   capture_output=True, text=True, timeout=10)
            self.assertEqual(check.returncode, 0, check.stderr)
            self.assertIn('factory VOA environment and exclusivity checks passed',
                          check.stdout)
            result = subprocess.run(['bash', str(launcher)], env=env,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 7, result.stderr)
            self.assertIn('BRIDGE_STARTED', result.stdout)
            for remap in (
                '/grid_map:=/patrol/voa/grid_map',
                '/cmd_vel:=/patrol/voa/disabled_cmd_vel',
                '/handle_state:=/patrol/voa/disabled_handle_state',
                '/cmd_vel_corrected:=/patrol/voa/unused_corrected',
                '/marker:=/patrol/voa/marker',
                '/camera/depth/color/transformed_points:=/patrol/voa/transformed_points',
            ):
                self.assertIn(remap, result.stdout)
            pids = []
            for line in result.stdout.splitlines():
                for prefix_name in ('PID=', 'BRIDGE_PID='):
                    if line.startswith(prefix_name):
                        pids.append(int(line[len(prefix_name):]))
            self.assertGreaterEqual(len(pids), 3)
            for pid in pids:
                with self.assertRaises(ProcessLookupError):
                    os.kill(pid, 0)

    def test_gate_supervisor_reports_perception_exit_and_requests_disarm(self):
        with tempfile.TemporaryDirectory(prefix='factory_gate_') as directory:
            root = Path(directory)
            (root / 'bin').mkdir()
            (root / 'navigation').mkdir()
            (root / 'scripts').mkdir()
            (root / 'navigation/patrol_env.sh').write_text(
                'export PATH="%s/bin:$PATH"\n' % root)
            (root / 'scripts/navigation_recovery_budget.py').write_text(
                'print(60.0)\n')
            factory = root / 'navigation/start_factory_voa.sh'
            factory.write_text(
                '#!/usr/bin/env bash\n'
                'if [[ ${1:-} == --check ]]; then echo FACTORY_CHECK_OK; exit 0; fi\n'
                'echo FACTORY_VOA_STARTED\n'
                'sleep 0.2\n'
                'exit 9\n')
            factory.chmod(0o755)
            write_executable(root / 'bin/ros2', (
                '#!/bin/sh\n'
                'if [ "$1" = service ]; then exit 0; fi\n'
                'if [ "$1" = run ]; then\n'
                '  echo "GATE_ARGS=$*"\n'
                '  echo GATE_STARTED; echo GATE_PID=$$; sleep 1; exit 0\n'
                'fi\n'
                'exit 0\n'))
            gate = root / 'start_gate.sh'
            gate.write_text(
                (ROOT / 'navigation/start_gate_disarmed.sh').read_text().replace(
                    'PROJECT=/home/ysc/lite3_patrol_project',
                    'PROJECT=' + str(root)))
            env = dict(os.environ, PATH=str(root / 'bin') + ':' + os.environ['PATH'],
                       LITE3_OBSTACLE_BACKEND='factory_only')
            env.pop('LITE3_DYNAMIC_OBSTACLES', None)
            env.pop('LITE3_USE_VENDOR_VOA', None)
            result = subprocess.run(['bash', str(gate)], env=env,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('FACTORY_CHECK_OK', result.stdout)
            self.assertIn('GATE_STARTED', result.stdout)
            self.assertIn('use_vendor_voa:=false', result.stdout)
            self.assertIn('require_voa_costmap:=true', result.stdout)
            self.assertIn('require_lidar_costmap:=false', result.stdout)
            self.assertIn('VOA perception supervisor exited', result.stderr)
            self.assertNotIn('ARMED', result.stdout)
            for line in result.stdout.splitlines():
                if line.startswith('GATE_PID='):
                    with self.assertRaises(ProcessLookupError):
                        os.kill(int(line.split('=', 1)[1]), 0)


class EntryPointTests(unittest.TestCase):
    def test_wrapper_defaults_to_factory_only_and_keeps_disarmed_help(self):
        env = dict(os.environ)
        env.pop('LITE3_OBSTACLE_BACKEND', None)
        result = subprocess.run(
            ['bash', str(ROOT / 'scripts/navigation_dynamic.sh'), '--help'],
            env=env, capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('factory_only', result.stdout)
        self.assertIn('LITE3_DYNAMIC_OBSTACLES=1', result.stdout)
        self.assertIn('DISARMED', result.stdout)

    def test_main_launcher_normalizes_both_backends(self):
        with tempfile.TemporaryDirectory() as directory:
            key = Path(directory) / 'key'
            key.write_text('')
            command = (
                'set -- --help; source "$ENTRY" >/dev/null; '
                'trap - EXIT INT TERM HUP; SSH=(true); validate_inputs; '
                'echo "NORMALIZED:$DYNAMIC_OBSTACLES:$USE_VENDOR_VOA:$OBSTACLE_BACKEND"')
            for backend, expected in (
                    ('factory_only', 'NORMALIZED:1:0:factory_only'),
                    ('private_combined', 'NORMALIZED:1:1:private_combined')):
                result = subprocess.run(
                    ['bash', '-c', command], text=True, capture_output=True, timeout=5,
                    env=dict(os.environ, ENTRY=str(ROOT / 'scripts/navigation_start.sh'),
                             SSH_KEY=str(key), LITE3_OBSTACLE_BACKEND=backend))
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), expected)

    def test_factory_only_rejects_private_voa_before_connecting(self):
        result = subprocess.run(
            ['bash', str(ROOT / 'scripts/navigation_start.sh'), 'preflight'],
            env=dict(os.environ, LITE3_OBSTACLE_BACKEND='factory_only',
                     LITE3_USE_VENDOR_VOA='1'),
            capture_output=True, text=True, timeout=5)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('factory_only', result.stderr)


if __name__ == '__main__':
    unittest.main()
