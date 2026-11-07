"""Offline R1.1 tests; actual gate with fake ROS, reference C++ with fake sendto."""
import ast
import importlib.util
import io
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml
import test_vendor_voa_gate as gate_fixture

ROOT=Path(__file__).resolve().parents[1]

def load(name,file):
    spec=importlib.util.spec_from_file_location(name,ROOT/file)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

audit=load('r11_audit','tools/r11/rotation_audit.py')
capture=load('r11_capture','tools/r11/capture_rotation.py')


class RotationGateTests(unittest.TestCase):
    def setUp(self):
        self.fixture=gate_fixture.GateTests()
        self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.g=self.fixture.gate
        self.g.use_vendor_voa=False;self.g.require_voa_costmap=True
        self.g.params['max_linear_y']=0.0
        self.g.counts.update({'/patrol/cmd_vel_raw':2,'/patrol/voa/costmap_heartbeat':1,'/patrol/voa/costmap_obstacles':1})
        self.heartbeat(10.0)
    def heartbeat(self,t):
        sec=int(t);self.g.voa_costmap_callback(types.SimpleNamespace(frame_id='map',stamp=types.SimpleNamespace(sec=sec,nanosec=round((t-sec)*1e9))))
    def arm(self):
        self.g.goal_pending=True
        response=self.g.arm_callback(types.SimpleNamespace(data=True),types.SimpleNamespace())
        self.assertTrue(response.success)
        self.g.goal_active_callback(types.SimpleNamespace(data=True))
    def command(self,w):
        m=gate_fixture.Twist();m.angular.z=w;self.g.raw_callback(m);self.g.timer_callback()
        return self.fixture.output()
    def test_a_positive_pure_rotation(self):
        self.arm();self.assertEqual(self.command(.2),(0,0,.2))
    def test_b_negative_pure_rotation(self):
        self.arm();self.assertEqual(self.command(-.2),(0,0,-.2))
    def test_c_tiny_rotation_has_no_gate_deadband(self):
        self.arm();self.assertEqual(self.command(1e-7),(0,0,1e-7))
    def test_d_positive_limit_is_clamp_not_zero(self):
        self.arm();self.assertEqual(self.command(2.0),(0,0,.4))
    def test_d_negative_limit_is_clamp_not_zero(self):
        self.arm();self.assertEqual(self.command(-2.0),(0,0,-.4))
    def test_e_arm_requires_healthy_source_and_accepts_rotation(self):
        self.arm();self.command(.15);self.assertTrue(self.g.armed)
    def test_f_disarmed_blocks_rotation(self):
        self.assertEqual(self.command(.2),(0,0,0))
    def test_f_explicit_disarm_blocks_rotation(self):
        self.arm();self.g.arm_callback(types.SimpleNamespace(data=False),types.SimpleNamespace())
        self.assertEqual(self.command(.2),(0,0,0))
    def test_g_expired_voa_latches_and_blocks(self):
        self.arm();self.g.clock_time=10.5001
        self.assertEqual(self.command(.2),(0,0,0));self.assertFalse(self.g.armed)
        self.assertEqual(self.g.fault,'voa_costmap_not_ready_or_stale')
    def test_g_recovery_does_not_auto_arm(self):
        self.arm();self.g.clock_time=10.6;self.command(.2);self.heartbeat(10.6)
        self.assertEqual(self.command(.2),(0,0,0));self.assertFalse(self.g.armed)
    def test_localization_loss_blocks_rotation(self):
        self.arm();self.g.localization_ready_callback(types.SimpleNamespace(data=False))
        self.assertEqual(self.command(.2),(0,0,0));self.assertEqual(self.g.fault,'localization_ready_revoked')
    def test_stale_raw_yields_zero_during_bounded_wait(self):
        self.arm();self.command(.2);self.g.clock_time=10.6;self.heartbeat(10.6)
        self.g.timer_callback();self.assertEqual(self.fixture.output(),(0,0,0))
    def test_idle_session_cannot_run_untracked_spin(self):
        self.arm();self.g.goal_active_callback(types.SimpleNamespace(data=False))
        self.assertEqual(self.command(.2),(0,0,0))
    def test_nan_command_latches(self):
        self.arm();self.assertEqual(self.command(float('nan')),(0,0,0));self.assertEqual(self.g.fault,'invalid_velocity_command')


class ReferenceNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler=shutil.which('g++')
        if not compiler:raise unittest.SkipTest('C++ compiler absent; no dependency installation')
        cls.tmp=tempfile.TemporaryDirectory(prefix='lite3_r11_native_')
        cls.binary=Path(cls.tmp.name)/'reference'
        subprocess.run([compiler,'-std=c++14','-O0',str(ROOT/'tests/fixtures/r11_motion_sender_reference.cpp'),'-o',str(cls.binary)],check=True,capture_output=True,text=True)
    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()
    def packets(self,w):
        r=subprocess.run([str(self.binary),'0','0',str(w)],check=True,capture_output=True,text=True)
        return [audit.decode_motion_payload(bytes.fromhex(line)) for line in r.stdout.splitlines()]
    def test_h_upstream_callback_sends_zero_xy_and_negated_positive_yaw(self):
        r=self.packets(.2);self.assertEqual([p['code'] for p in r],[320,325,321])
        self.assertEqual([p['wire_value'] for p in r],[0,0,-.2])
    def test_h_upstream_negative_rotation(self):
        self.assertEqual(self.packets(-.2)[2]['wire_value'],.2)
    def test_h_upstream_tiny_rotation_not_dropped(self):
        self.assertEqual(self.packets(1e-7)[2]['wire_value'],-1e-7)
    def test_h_reference_inverse_sign_is_preserved_in_decode(self):
        self.assertEqual(self.packets(.4)[2]['ros_reference_value'],.4)
    def test_native_foxy_samples_both_pure_rotation_directions(self):
        r=subprocess.run([str(self.binary),'dwb'],capture_output=True,text=True,check=True)
        values=[float(v) for v in r.stdout.splitlines()]
        self.assertTrue(any(v<-.1 for v in values));self.assertTrue(any(v>.1 for v in values))
        self.assertTrue(all(abs(v)+1e-5>=.1 and abs(v)<=.400001 for v in values))
    def test_factory_config_preserves_rotation_and_disables_spin_recovery(self):
        generator=load('r11_generator','scripts/make_dynamic_nav2_config.py')
        base=yaml.safe_load((ROOT/'navigation/patrol_map_20261031_01_area_v2_review/nav2_safe.yaml').read_text())
        result=generator.build(base,ROOT/'navigation/navigate_dynamic_obstacles.xml',factory_only=True)
        f=result['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(f['min_vel_x'],0);self.assertEqual(f['max_vel_theta'],.4)
        self.assertEqual(f['min_speed_theta'],.1);self.assertEqual(f['vtheta_samples'],31)
        self.assertIn('RotateToGoal',f['critics'])
        self.assertEqual(result['recoveries_server']['ros__parameters']['recovery_plugins'],['wait'])


class DiagnosticTests(unittest.TestCase):
    def test_wire_decoder_rejects_invalid_layout(self):
        self.assertIsNone(audit.decode_motion_payload(b'\0'*19))
        self.assertIsNone(audit.decode_motion_payload(struct.pack('<iiid',321,4,1,.2)))
        self.assertIsNone(audit.decode_motion_payload(struct.pack('<iiid',321,8,1,float('nan'))))
    def test_pcap_ethernet_and_cooked_decode(self):
        payload=struct.pack('<iiid',321,8,1,-.2)
        udp=struct.pack('!HHHH',5555,43893,len(payload)+8,0)+payload
        ip=bytearray(20);ip[0]=0x45;ip[9]=17;ip[2:4]=struct.pack('!H',20+len(udp));ip[12:16]=bytes([192,0,2,1]);ip[16:20]=bytes([192,0,2,2])
        for link,head in [(1,b'\0'*12+b'\x08\x00'),(113,b'\0'*14+b'\x08\x00'),(276,b'\x08\x00'+b'\0'*18)]:
            with self.subTest(link=link),tempfile.TemporaryDirectory() as d:
                frame=head+bytes(ip)+udp;f=Path(d)/'a.pcap'
                f.write_bytes(b'\xd4\xc3\xb2\xa1'+struct.pack('<HHIIII',2,4,0,0,65535,link)+struct.pack('<IIII',10,500000,len(frame),len(frame))+frame)
                rows=list(audit.read_pcap(f,'192.0.2.2'));self.assertEqual(rows[0]['reference_command']['ros_reference_value'],.2)
                self.assertIsNone(rows[0]['autonomy_confirmed'])
    def test_pcap_rejects_truncation(self):
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'a';f.write_bytes(b'bad')
            with self.assertRaises(ValueError):list(audit.read_pcap(f))
    def test_output_path_stays_in_tmp(self):
        with self.assertRaises(ValueError):capture.validate_output('/home/user/capture')
        with self.assertRaises(ValueError):capture.validate_output('/tmp')
    def test_capture_duration_is_bounded(self):
        self.assertEqual(capture.duration('45'),45)
        for v in ['0','29','181']:
            with self.assertRaises(Exception):capture.duration(v)
    def test_udp_command_requires_explicit_peer_and_no_auto_sudo(self):
        a=types.SimpleNamespace(motion_peer=None,interface='any',motion_port=43893,state_port=43897)
        with self.assertRaises(ValueError):capture.pcap_command(a,Path('/tmp/evidence'))
    def test_no_tools_publish_call_control_or_use_ssh(self):
        for name in ['capture_rotation.py','rotation_audit.py']:
            text=(ROOT/'tools/r11'/name).read_text();tree=ast.parse(text)
            attrs={node.attr for node in ast.walk(tree) if isinstance(node,ast.Attribute)}
            self.assertFalse(attrs.intersection({'create_publisher','create_client','call_async','send_goal_async','sendto','sendall'}))
            self.assertNotIn("['ssh'",text);self.assertNotIn("['sudo'",text)
    def test_synthetic_pure_rotation_matching_and_feedback(self):
        r=[]
        for t in range(6):
            r.extend([dict(topic='/patrol/cmd_vel_raw',receive_ns=t*100000000,x=0,y=0,w=.2),
                      dict(topic='/cmd_vel_corrected',receive_ns=t*100000000+10000000,x=0,y=0,w=.2),
                      dict(topic='/odom',receive_ns=t*100000000+10000000,yaw=t*.02)])
        out=audit.summarize(r)
        self.assertEqual(out['raw_pure_matched_output_within_150ms'],6)
        self.assertEqual(out['output']['positive'],6)
        self.assertAlmostEqual(out['pure_output_segments'][0]['yaw']['/odom']['delta_rad'],.1)
        self.assertIsNone(out['motion_mode_feedback']['autonomy_confirmed'])
    def test_unmatched_output_is_not_claimed_gate_filtering(self):
        out=audit.summarize([dict(topic='/patrol/cmd_vel_raw',receive_ns=10,x=0,y=0,w=.2)])
        self.assertEqual(len(out['unmatched_raw_pure']),1);self.assertIn('not proof',out['unmatched_interpretation'])
    def test_mock_recorder_exit_does_not_signal_business_or_start_ros(self):
        with tempfile.TemporaryDirectory(prefix='lite3_r11_test_') as d:
            out=Path(d)/'capture';proc=types.SimpleNamespace(pid=123,poll=lambda:2)
            args=types.SimpleNamespace(output=str(out),udp=False,motion_source=None,duration=30,motion_peer=None,motion_port=43893,state_port=43897)
            with patch.object(capture,'inspect_processes',return_value={}),patch.object(capture,'command_output',return_value={}),patch.object(capture.subprocess,'Popen',return_value=proc),patch('sys.stdout',new=io.StringIO()):
                capture.record(args)
            self.assertEqual(json.loads((out/'capture.json').read_text())['reason'],'recorder_exited')

if __name__=='__main__':
    unittest.main()
