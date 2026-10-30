import ast
import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace as N
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from check_localization_runtime import verify
from check_obstacle_costmap_consumption import coverage


def test_compiled_runtime_tampering_is_detected(tmp_path):
    binary=tmp_path/'runtime';binary.write_bytes(b'build A')
    manifest=tmp_path/'manifest.json'
    manifest.write_text(json.dumps({'version':1,'files':{str(binary):hashlib.sha256(binary.read_bytes()).hexdigest()}}))
    verify(manifest)
    binary.write_bytes(b'build B')
    with pytest.raises(ValueError):verify(manifest)


def test_actual_costmap_coordinates_and_unknown_are_reported():
    grid=N(info=N(origin=N(position=N(x=1.,y=2.),orientation=N(x=0.,y=0.,z=0.,w=1.)),
                  resolution=.1,width=2,height=2),data=[100,0,-1,0])
    row=coverage(np.array([[1.05,2.05,0],[1.15,2.05,0],[1.05,2.15,0],[5,5,0]]),grid)
    assert row==dict(input=4,inside=3,outside=1,occupied=1,unknown=1,below_occupied=1)


def test_old_leg_samples_cannot_be_restamped_as_current():
    path=ROOT/'scripts/leg_odom_tf_bridge.py';tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef));cls.bases=[]
    cls.body=[n for n in cls.body if getattr(n,'name','')!='__init__']
    clock=N(value=100.);scope=dict(math=math,copy=copy,time=N(monotonic=lambda:clock.value))
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(path),'exec'),scope)
    node=scope['LegOdomTfBridge']();node.last_source_stamp=0.;node.last_message=0.;node.latest_message=None
    node.get_logger=lambda:N(error=lambda message:None)
    pose=N(position=N(x=0.,y=0.,z=0.),orientation=N(x=0.,y=0.,z=0.,w=1.))
    msg=N(header=N(stamp=N(sec=90,nanosec=0)),pose=N(pose=pose))
    node.callback(msg);assert node.latest_message is None
    msg.header.stamp.sec=100;node.callback(msg);assert node.last_source_stamp==100.
    clock.value=101.
    # No publisher/clock is provided: stale input must return before restamping.
    node.publish_latest()
    assert node.last_source_stamp==100.
