#!/usr/bin/env python3
"""Bounded camera-only probe; pose/state topics isolated from the initializer."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
from lite3_patrol_nav.rgbd_visual_localizer import VisualLocalizer


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=45.)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--backend',choices=['orb','xfeat_lighterglue'],default='xfeat_lighterglue')
    args=parser.parse_args()
    rclpy.init(args=['--ros-args','-r','__node:=xfeat_deployment_camera_probe',
        '-r','/visual_localization/state:=/xfeat_validation/state',
        '-p','matching_backend:='+args.backend,
        '-p','database:=/home/ysc/lite3_patrol_project/results/visual_maps/outdoor_20260921_003/visual_map.db',
        '-p','pose_topic:=/xfeat_validation/pose','-p','processing_rate_hz:=1.0',
        '-p','min_matches:=18','-p','min_inliers:=15','-p','ratio_test:=0.80',
        '-p','confirmation_samples:=3','-p','learned_fallback:=false'])
    start=time.monotonic(); node=VisualLocalizer()
    report=dict(backend=args.backend,startup_seconds=time.monotonic()-start,states=[],poses=[],
        scope='Camera-only visual frontend; no /initialpose, navigation, ARM or motion commands.')
    original=node.publish_state
    def record(value):
        if value.startswith(('POSE:', 'REJECTED:', 'ERROR:', 'PAUSED:', 'WAITING_FOR_IMAGE')):
            report['states'].append(value)
        original(value)
    node.publish_state=record
    def pose(msg):
        p=msg.pose.pose
        report['poses'].append(dict(x=p.position.x,y=p.position.y,z=p.position.z,
            qz=p.orientation.z,qw=p.orientation.w))
    node.create_subscription(PoseWithCovarianceStamped,'/xfeat_validation/pose',pose,10)
    end=time.monotonic()+args.seconds
    try:
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.2)
        report['received_image']=node.last_image_time>0
        report['pnp_pose_observed']=any('source=xfeat_lighterglue_pnp' in s for s in report['states'])
        if args.backend=='xfeat_lighterglue':
            # Invoke only this node's callbacks. Do not publish readiness to the robot.
            node.ready_callback(Bool(data=True))
            stamp=node.last_processed_stamp
            node.processing_callback()
            report['ready_pauses_visual']=node.visual_paused and node.last_processed_stamp==stamp
            node.ready_callback(Bool(data=False))
            report['loss_does_not_auto_resume']=node.visual_paused
            node.initializer_state_callback(String(data='COLLECTING_VISUAL_POSES'))
            report['explicit_request_resumes']=not node.visual_paused and node.latest_image is None
        report['runtime_errors']=[s for s in report['states'] if s.startswith('ERROR:')]
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('states','poses')},indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
