"""Actual spatial synchronization and gate behavior; synthetic maps, no ROS."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'patrol_nav_ws/src/lite3_patrol_nav'))
from lite3_patrol_nav.costmap_sync import CostmapSync


def cloud(points=(),stamp=100.):
    a=np.asarray(points,dtype='<f4').reshape(-1,3)
    return NS(header=NS(frame_id='map',stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
        height=1,width=len(a),point_step=12,row_step=len(a)*12,data=a.tobytes(),is_bigendian=False,
        fields=[NS(name=k,offset=i*4,datatype=7) for i,k in enumerate(('x','y','z'))])


def grid(occupied=(),stamp=1700000000.,origin=(-2.,-2.)):
    a=np.zeros((40,40),dtype=int)
    for x,y,value in occupied:
        col,row=np.floor((np.array([x,y])-origin)/.1).astype(int)
        a[row,col]=value
    return NS(header=NS(frame_id='map',stamp=NS(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))),
        info=NS(resolution=.1,width=40,height=40,origin=NS(position=NS(x=origin[0],y=origin[1]),
            orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=a.ravel().tolist())


def test_projection_alone_does_not_prove_local_map_representation():
    s=CostmapSync();assert s.receive_cloud(cloud([(.85,.05,.5)]),100.1)
    assert s.receive_grid(grid(),100.1,1700000000.1)
    ok,reason,metrics=s.check((0.,0.),100.1,1700000000.1)
    assert not ok and reason=='waiting_obstacles_in_local_costmap'
    assert metrics['unrepresented_points']==1
    assert s.receive_grid(grid([(.85,.05,100)],stamp=1700000000.2),100.2,1700000000.2)
    assert s.check((0.,0.),100.2,1700000000.2)[0]


def test_clock_domains_replay_and_stale_maps_are_not_relabelled():
    s=CostmapSync();s.receive_cloud(cloud(),100.1)
    assert not s.receive_grid(grid(),100.1,100.1)  # Wrong clock domain.
    assert s.receive_grid(grid(),100.1,1700000000.1)
    assert not s.receive_grid(grid(),100.5,1700000000.5)  # Replay cannot renew arrival.
    s.receive_cloud(cloud(stamp=100.9),100.9)
    assert s.healthy(100.9,1700000000.9)==(False,'local_costmap_missing_or_stale')


def test_only_quantization_distance_not_arbitrary_neighbor_cost_counts():
    for x,expected in [(.099,True),(.049,False)]:
        s=CostmapSync();s.receive_cloud(cloud([(x,.05,.5)]),100.1)
        s.receive_grid(grid([(.15,.05,100)],origin=(0.,0.)),100.1,1700000000.1)
        assert s.check((0.,0.),100.1,1700000000.1)[0] is expected
    s=CostmapSync();s.receive_cloud(cloud([(.85,.05,.5)]),100.1)
    s.receive_grid(grid([(.85,.05,99)]),100.1,1700000000.1)
    assert not s.check((0.,0.),100.1,1700000000.1)[0]


def test_empty_cloud_valid_far_points_not_a_near_motion_veto_unknown_not_occupied():
    s=CostmapSync();s.receive_grid(grid(),100.1,1700000000.1)
    s.receive_cloud(cloud(),100.1)
    assert s.check((0.,0.),100.1,1700000000.1)[0]
    s.receive_cloud(cloud([(2.,0.,.5)],stamp=100.2),100.2)
    assert s.check((0.,0.),100.2,1700000000.2)[0]
    assert not s.receive_cloud(cloud([(float('nan'),0.,.5)],stamp=100.3),100.3)
    assert not s.healthy(100.3,1700000000.3)[0]


def test_foxy_raw_costmap_native_costs_and_ros_timestamp():
    visual=grid([(.85,.05,100)])
    raw=NS(header=visual.header,metadata=NS(resolution=.1,size_x=40,size_y=40,origin=visual.info.origin),
           data=[254 if v==100 else v for v in visual.data])
    s=CostmapSync();s.receive_cloud(cloud([(.85,.05,.5)]),100.1)
    assert s.receive_raw(raw,100.1,1700000000.1)
    assert s.check((0.,0.),100.1,1700000000.1)[0]
    raw.header.stamp=NS(sec=0,nanosec=0)
    t=CostmapSync();assert not t.receive_raw(raw,100.1,1700000000.1)
    assert t.grid is None


def test_actual_gate_waits_for_costmap_then_resumes_and_latches_map_loss():
    from test_factory_voa_mode import FactoryGateTests
    f=FactoryGateTests();f.setUp()
    try:
        g=f.gate
        g.require_costmap_sync=True;g.costmap_sync=CostmapSync()
        g.ros_seconds=lambda:1700000000.+g.clock_time-10.
        g.last_pose=(0.,0.,0.)
        g.costmap_sync.receive_cloud(cloud([(.85,.05,.5)],stamp=10.),10.)
        g.costmap_sync.receive_grid(grid(),10.,1700000000.)
        f.heartbeat();assert f.arm().success
        g.goal_active_callback(NS(data=True))
        command=f.module.Twist();command.linear.x=.1;g.raw_callback(command)
        g.timer_callback()
        assert f.output()==(0.,0.,0.) and g.armed
        assert 'WAITING_OBSTACLE_COSTMAP_SYNC' in g.outputs['/patrol/gate_state'][-1].data
        g.clock_time=10.05
        g.costmap_sync.receive_grid(grid([(.85,.05,100)],stamp=1700000000.05),10.05,1700000000.05)
        g.timer_callback()
        assert f.output()==(.1,0.,0.)
        g.clock_time=10.9
        f.heartbeat(10.9)
        g.costmap_sync.receive_cloud(cloud(stamp=10.9),10.9)
        g.timer_callback()
        assert f.output()==(0.,0.,0.) and not g.armed
        assert g.fault=='local_costmap_missing_or_stale'
    finally:
        f.doCleanups()
