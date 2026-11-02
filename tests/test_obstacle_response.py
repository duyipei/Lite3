"""Synthetic geometry and actual gate integration; no robot motion or ROS bus."""
import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'patrol_nav_ws/src/lite3_patrol_nav'))
sys.path.insert(0, str(ROOT/'scripts'))
from lite3_patrol_nav.obstacle_safety import footprint_extents, terminal_extents
from test_voa_costmap_bridge import grid
from make_dynamic_nav2_config import build

PROFILE = ROOT/'navigation/obstacle_response.yaml'


def scene(points=(), stamp=100., order='F', start=(0, 0)):
    data = np.zeros((120, 80))
    for x, y, z in points:
        data[int(math.floor((3-x)/.05)), int(math.floor((2-y)/.05))] = z
    msg = grid(data, order=order, start=start, stamp=stamp)
    msg.info.resolution = .05
    msg.info.length_x, msg.info.length_y = 6., 4.
    mask = copy.deepcopy(msg.data[0])
    mask.data = np.isfinite(mask.data).astype(float).tolist()
    msg.layers.append('observation_validity'); msg.data.append(mask)
    return msg


def test_planner_known_only_does_not_invent_unknown_walls_or_clear_obstacles():
    from lite3_patrol_nav.voa_grid import grid_obstacles
    msg = scene([(.4,0,float('nan')),(.8,0,.2)])
    known,stats = grid_obstacles(msg,.1,include_unknown=False)
    assert len(known) == 1 and stats['unknown'] == 1
    conservative,_ = grid_obstacles(msg,.1,include_unknown=True)
    assert len(conservative) == 2


@pytest.mark.parametrize('height', [.10, .16, .20])
def test_true_height_obstacles_are_never_removed_to_improve_passage(height):
    # Float32 .10 is strictly above the factory double threshold, matching its
    # original predicate. No obstacle threshold has been raised by this change.
    from lite3_patrol_nav.voa_grid import grid_obstacles
    points,_ = grid_obstacles(scene([(.4,0,height)]),.1,include_unknown=False)
    assert len(points) == 1


def test_profile_changes_only_private_overlay_and_uses_shared_goal_tolerance():
    base = yaml.safe_load((ROOT/'navigation/patrol_map_20261031_01_area_v2_review/nav2_safe.yaml').read_text())
    before = copy.deepcopy(base)
    result = build(base,ROOT/'navigation/navigate_dynamic_obstacles.xml',factory_only=True)
    assert base == before
    for key,hz in [('local_costmap',5.),('global_costmap',2.)]:
        c = result[key][key]['ros__parameters']
        assert c['update_frequency'] >= hz
        assert c['publish_frequency'] == 2.
        assert c['inflation_layer']['inflation_radius'] == .50
        assert c['footprint'] == base[key][key]['ros__parameters']['footprint']
    controller = result['controller_server']['ros__parameters']
    assert controller['goal_checker']['xy_goal_tolerance'] == .12
    assert controller['FollowPath']['xy_goal_tolerance'] == .12
    assert controller['FollowPath']['min_vel_x'] == 0.
    assert controller['progress_checker']['movement_time_allowance'] == 15.
    hx,hy = footprint_extents(result['local_costmap']['local_costmap']['ros__parameters'])
    assert hx >= .35 and hy >= .25


def test_actual_gate_keeps_command_publisher_checks_without_direct_grid_input():
    from test_factory_voa_mode import FactoryGateTests
    fixture = FactoryGateTests(); fixture.setUp()
    try:
        g = fixture.gate
        g.get_publishers_info_by_topic = lambda topic: [NS(node_name='controller_server'),NS(node_name='unknown_teleop')]
        assert g.sensor_health()[1] == 'nav_command_publisher_conflict'
        g.get_publishers_info_by_topic = lambda topic: [NS(node_name='controller_server'),NS(node_name='recoveries_server')]
        fixture.heartbeat()
        assert fixture.arm().success
        g.goal_active_callback(NS(data=True))
        command = fixture.module.Twist(); command.linear.x = .15; g.raw_callback(command)
        g.timer_callback()
        assert fixture.output() == (.15,0.,0.)
        # Projection heartbeat safety is independent of the removed grid veto.
        g.clock_time = 10.6
        g.timer_callback()
        assert not g.armed
        assert g.fault == 'voa_costmap_not_ready_or_stale'
        assert fixture.output() == (0.,0.,0.)
    finally:
        fixture.doCleanups()


def test_actual_goal_bridge_oriented_extent_unknown_boundary_and_empty_cloud():
    from test_patrol_route import Tests
    fixture = Tests(); fixture.setUp()
    b = fixture.bridge
    try:
        b.goal_geometry = terminal_extents(.35,.25,.2,.12,.03)
        b.map_msg = NS(header=NS(frame_id='map'), info=NS(resolution=.1,width=50,height=50,
            origin=NS(position=NS(x=-2.5,y=-2.5),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=[0]*2500)
        b.require_voa_obstacles = False
        assert b.goal_clearance(0.,0.,0.)[0]
        assert not b.goal_clearance(2.4,0.,0.)[0]
        b.map_msg.data[25*50+25] = -1
        assert b.goal_clearance(0.,0.,0.)[1] == 'goal_footprint_contains_unknown'
        b.map_msg.data[25*50+25] = 0
        # Beyond side extent at yaw zero, but in front extent after 90 deg.
        col,row = 25,30
        b.map_msg.data[row*50+col] = 100
        assert b.goal_clearance(0.,0.,0.)[0]
        assert not b.goal_clearance(0.,0.,math.pi/2)[0]
        b.map_msg.data[row*50+col] = 0
        b.require_voa_obstacles = True
        import test_patrol_route
        now = test_patrol_route.BUS.now
        b.voa_cloud_time = now
        b.voa_cloud = NS(header=NS(frame_id='map',stamp=NS(sec=int(now),nanosec=0)),
            fields=[NS(name=k,datatype=7,offset=i*4) for i,k in enumerate(('x','y','z'))],
            is_bigendian=False,height=1,width=0,point_step=12,row_step=0,data=b'')
        assert b.goal_clearance(0.,0.,0.)[0]  # Fresh observed empty cloud is valid.
        b.voa_cloud.header.stamp.sec -= 1
        assert b.goal_clearance(0.,0.,0.)[1] == 'voa_obstacles_stale'
    finally:
        fixture.doCleanups()


def test_goal_bridge_constructor_with_actual_dynamic_yaml_preserves_ros_path_type(tmp_path):
    # Exercise the real startup branch. Setting goal_geometry manually misses
    # filesystem Path being shadowed by nav_msgs.msg.Path at import time.
    from test_patrol_route import bridge, Node, Message
    base = yaml.safe_load((ROOT/'navigation/patrol_map_20261031_01_area_v2_review/nav2_safe.yaml').read_text())
    nav = tmp_path/'nav2_dynamic.yaml'
    nav.write_text(yaml.safe_dump(build(base,ROOT/'navigation/navigate_dynamic_obstacles.xml',factory_only=True)))
    overrides = {'goal_nav2_file':str(nav),'goal_response_file':str(PROFILE)}
    declare = Node.declare_parameter
    publisher = Node.create_publisher
    kinds = {}
    def with_overrides(self,key,default):
        return declare(self,key,overrides.get(key,default))
    def capture_kind(self,kind,topic,qos):
        kinds[topic] = kind
        return publisher(self,kind,topic,qos)
    with patch.object(Node,'declare_parameter',with_overrides), patch.object(Node,'create_publisher',capture_kind):
        b = bridge.GoalPoseBridge()
    assert b.goal_geometry[0] > .5
    assert b.goal_geometry[1] > .4
    assert kinds['/plan'] is Message
    assert bridge.Path is Message
