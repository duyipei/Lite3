"""Obstacle projection concurrency and marking-origin integrity."""
import threading
import time

import numpy as np

from test_voa_costmap_bridge import grid, M


def test_projection_does_not_block_receiving_or_fill_both_executor_workers():
    from test_voa_costmap_bridge import RuntimeTests
    f=RuntimeTests();f.setUp()
    try:
        b=f.bridge;f.now.return_value=100.15;f.tf_until=101.
        b.receive(grid(stamp=100.))
        lookup=b.buffer.lookup_transform
        entered,resume=threading.Event(),threading.Event()
        def slow(*args):
            entered.set(); assert resume.wait(2)
            return lookup(*args)
        b.buffer.lookup_transform=slow
        worker=threading.Thread(target=b.publish);worker.start();assert entered.wait(1)
        try:
            b.receive(grid(stamp=100.1))
            # Another timer must skip, rather than wait on the publishing lock.
            start=time.monotonic();b.publish();assert time.monotonic()-start<.1
            assert b.last_received==100.1
        finally:
            resume.set();worker.join(2)
        assert not worker.is_alive()
        assert M.stamp_seconds(f.outputs[M.TOPIC][-1].header.stamp)==100.
        b.buffer.lookup_transform=lookup;b.publish()
        assert M.stamp_seconds(f.outputs[M.TOPIC][-1].header.stamp)==100.1
    finally:
        f.doCleanups()


def test_2d_marking_origin_uses_projecting_pose_and_synthetic_height_only():
    from test_voa_costmap_bridge import transform
    pose=transform(yaw=.4,pitch=.2,x=7.,y=-2.)
    pose.translation.z=10.
    xyz,q=M.marking_origin(pose)
    np.testing.assert_allclose(xyz,[7.,-2.,.5])
    np.testing.assert_allclose(q,[0.,0.,np.sin(.2),np.cos(.2)])
    # Synthetic marking Z cannot accidentally exclude obstacles by distance
    # merely because the map terrain itself is ten metres above its origin.
    assert np.linalg.norm(np.array([7.2,-2.,.5])-xyz)<.3


def test_marking_origin_failure_cannot_emit_cloud_or_heartbeat():
    from test_voa_costmap_bridge import RuntimeTests
    f=RuntimeTests();f.setUp()
    try:
        seen=[]
        def fail(stamp,pose):
            seen.append(M.stamp_seconds(stamp));raise RuntimeError('TF publisher failed')
        f.bridge.origin_publisher=fail
        f.bridge.receive(grid(stamp=100.));f.bridge.publish()
        assert seen==[100.] and not f.outputs[M.TOPIC] and not f.outputs[M.HEARTBEAT]
    finally:f.doCleanups()
