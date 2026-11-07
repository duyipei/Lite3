#!/usr/bin/env python3
"""Prepare / record a read-only R1.1 test on the local Jetson; no SSH or control."""
import argparse
import hashlib
import ipaddress
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

TOPICS = ['/patrol/goal_request','/patrol/cmd_vel_raw','/cmd_vel','/cmd_vel_corrected','/patrol/armed',
          '/patrol/gate_state','/patrol/goal_pending','/patrol/goal_active',
          '/patrol/goal_bridge_state','/patrol/localization_ready',
          '/patrol/voa/costmap_heartbeat','/odom','/leg_odom2','/imu/data',
          '/handle_state','/navigate_to_pose/_action/status']
REFERENCE_SHA256 = '169383cc460a702bcf2374a5a5321889f57111eb98033e5330c93591c69c9edc'
REFERENCE_COMMIT = 'e4ab5dc1c7f3ff46274162a1ef302253936578aa'
NATIVE_RECORDER = '''from rosbag2_transport import rosbag2_transport_py as t
import sys,json
t.record(uri=sys.argv[1], storage_id="sqlite3", serialization_format="", node_prefix="_lite3_r11_readonly", compression_mode="NONE", compression_format="", no_discovery=False, polling_interval=1000, max_bagfile_size=268435456, max_cache_size=0, topics=json.loads(sys.argv[2]), include_hidden_topics=True, qos_profile_overrides={})
'''


def validate_output(value):
    p = Path(value).expanduser().resolve()
    if p == Path('/tmp') or os.path.commonpath(['/tmp',str(p)]) != '/tmp':
        raise ValueError('Jetson capture output must be a new directory under /tmp')
    return p


def duration(value):
    value=int(value)
    if not 30<=value<=180:
        raise argparse.ArgumentTypeError('duration must be 30..180 seconds')
    return value


def ipv4(value):
    return str(ipaddress.IPv4Address(value))


def command_output(args, limit=300000):
    try:
        p=subprocess.run(args,capture_output=True,text=True,timeout=8)
        return dict(command=args,returncode=p.returncode,stdout=p.stdout[:limit],stderr=p.stderr[:4000])
    except (OSError,subprocess.TimeoutExpired) as error:
        return dict(command=args,error=type(error).__name__+': '+str(error))


def inspect_processes(source=None):
    result={'clock':clock_sample(),'motion_host_mode_confirmed':None,
            'mode_note':'No motion-host mode ACK inferred from ARM, node existence, or operator annotation.',
            'processes':[], 'shell_environment':{k:os.environ.get(k) for k in
                ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','CYCLONEDDS_URI','AMENT_PREFIX_PATH','PYTHONPATH')}}
    for p in Path('/proc').glob('[0-9]*'):
        try:
            raw=(p/'cmdline').read_bytes().split(b'\0')
            argv=[v.decode(errors='replace') for v in raw if v]
            if not argv or not any('jetson2motion' in v or Path(v).name=='motion_sender' for v in argv[:3]):
                continue
            exe=(p/'exe').resolve()
            env={}
            for entry in (p/'environ').read_bytes().split(b'\0'):
                key,sep,value=entry.partition(b'=')
                if sep and key.decode(errors='replace') in ('ROS_DISTRO','RMW_IMPLEMENTATION','ROS_DOMAIN_ID','AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH','CYCLONEDDS_URI'):
                    env[key.decode()]=value.decode(errors='replace')
            result['processes'].append(dict(pid=int(p.name),argv=argv,executable=str(exe),
                executable_sha256=hashlib.sha256(exe.read_bytes()).hexdigest(),environment=env,
                binary_source_correspondence='unproven; a source hash is not a binary build attestation'))
        except (OSError,PermissionError):
            continue
    if source:
        source=Path(source)
        digest=hashlib.sha256(source.read_bytes()).hexdigest()
        result['source']=dict(path=str(source.resolve()),sha256=digest,
                              matches_upstream_reference=digest==REFERENCE_SHA256,
                              reference_commit=REFERENCE_COMMIT)
    for args in [['systemctl','show','transfer_ros2.service','-p','ActiveState','-p','MainPID','-p','ExecStart'],
                 ['ss','-unap']]:
        result.setdefault('system',[]).append(command_output(args))
    return result


def clock_sample():
    before=time.monotonic_ns();wall=time.time_ns();after=time.monotonic_ns()
    return dict(monotonic_before_ns=before,wall_ns=wall,monotonic_after_ns=after,
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def inspect_graph():
    import rclpy
    rclpy.init()
    node=rclpy.create_node('lite3_r11_readonly_graph_inspector')
    result={}
    try:
        deadline=time.monotonic()+2.0
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        result['nodes']=[list(v) for v in node.get_node_names_and_namespaces()]
        result['topic_types']=node.get_topic_names_and_types()
        for topic in ['/patrol/cmd_vel_raw','/cmd_vel','/cmd_vel_corrected','/simple_cmd','/complex_cmd','/imu/data','/leg_odom2']:
            row={}
            for label,method in [('publishers',node.get_publishers_info_by_topic),('subscriptions',node.get_subscriptions_info_by_topic)]:
                row[label]=[dict(node_name=e.node_name,node_namespace=e.node_namespace,type=e.topic_type,
                    reliability=str(e.qos_profile.reliability),durability=str(e.qos_profile.durability),depth=e.qos_profile.depth) for e in method(topic)]
            result[topic]=row
        return result
    finally:
        node.destroy_node();rclpy.shutdown()


def pcap_command(args, output):
    if not args.motion_peer:
        raise ValueError('--udp requires --motion-peer read from the actual MotionSender configuration')
    if not args.interface or not all(c.isalnum() or c in '_.:-' for c in args.interface):
        raise ValueError('--udp requires an explicit interface name or any')
    binary=shutil.which('tcpdump')
    if not binary:
        raise ValueError('tcpdump unavailable; no dependency is installed automatically')
    bpf='udp and ((dst host %s and dst port %d) or dst port %d)' % (args.motion_peer,args.motion_port,args.state_port)
    return ['nice','-n','10',binary,'-i',args.interface,'-nn','-s','0','-U','-w',str(output/'motion_udp.pcap'),bpf]


def stop_own(process):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        # Never escalate to killing a business process, process group, or service.
        raise RuntimeError('own recorder still finalizing; inspect its recorded PID')


def record(args):
    output=validate_output(args.output)
    udp_cmd=pcap_command(args,output) if args.udp else None
    output.mkdir(parents=True,exist_ok=False)
    (output/'record_native.py').write_text(NATIVE_RECORDER)
    (output/'preflight_processes.json').write_text(json.dumps(inspect_processes(args.motion_source),ensure_ascii=False,indent=2))
    before=command_output(['journalctl','-u','transfer_ros2.service','-n','30','--no-pager','-o','json'])
    (output/'sender_journal_before.json').write_text(json.dumps(before,ensure_ascii=False))
    recorder=None;sniffer=None;logs=[];samples=[];reason='duration_limit'
    metadata=dict(start_clock=clock_sample(),topics=TOPICS,duration_s=args.duration,
                  udp_requested=args.udp,motion_peer=args.motion_peer,motion_port=args.motion_port,
                  state_port=args.state_port,only_readonly_recording=True,autonomy_confirmed=None,
                  actual_sender_callback_twist='unmeasured unless existing sender logs expose it')
    try:
        if udp_cmd:
            log=(output/'tcpdump.log').open('x');logs.append(log)
            sniffer=subprocess.Popen(udp_cmd,stdout=log,stderr=subprocess.STDOUT)
            time.sleep(.5)
            if sniffer.poll() is not None:
                raise RuntimeError('passive UDP capture failed; inspect tcpdump.log (no sudo or capability changes attempted)')
        log=(output/'rosbag.log').open('x');logs.append(log)
        recorder=subprocess.Popen(['nice','-n','10',sys.executable,'-u',str(output/'record_native.py'),str(output/'bag'),json.dumps(TOPICS)],stdout=log,stderr=subprocess.STDOUT)
        metadata.update(recorder_pid=recorder.pid,sniffer_pid=sniffer.pid if sniffer else None)
        (output/'capture.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
        print(json.dumps(dict(output=str(output),pids=[metadata['recorder_pid'],metadata['sniffer_pid']],note='No ARM, goal, velocity, or mode command is sent. Operator controls the test.'),ensure_ascii=False),flush=True)
        start=time.monotonic();prev_at=start;prev_cpu=0.;high=0;hz=os.sysconf('SC_CLK_TCK');verified=False
        while time.monotonic()-start<args.duration:
            if recorder.poll() is not None:
                reason='recorder_exited';break
            if sniffer and sniffer.poll() is not None:
                reason='udp_capture_exited';break
            time.sleep(2);now=time.monotonic()
            r=dict(elapsed_s=now-start,clock=clock_sample(),message_counts={})
            try:
                stat=Path('/proc/%d/stat'%recorder.pid).read_text().split(') ',1)[1].split()
                used=(int(stat[11])+int(stat[12]))/hz
                r['recorder_cpu_one_core_pct']=100*(used-prev_cpu)/(now-prev_at);prev_cpu=used;prev_at=now
                high=high+1 if now-start>8 and r['recorder_cpu_one_core_pct']>20 else 0
            except OSError:
                pass
            for file in (output/'bag').glob('*.db3'):
                try:
                    db=sqlite3.connect('file:'+str(file)+'?mode=ro',uri=True,timeout=.1)
                    for name,count in db.execute('select t.name,count(m.id) from topics t left join messages m on t.id=m.topic_id group by t.id'):
                        r['message_counts'][name]=r['message_counts'].get(name,0)+count
                    db.close()
                except sqlite3.Error:
                    pass
            samples.append(r)
            (output/'status.json').write_text(json.dumps(r,ensure_ascii=False))
            observed=all(r['message_counts'].get(t,0)>0 for t in ['/cmd_vel_corrected','/patrol/gate_state','/odom'])
            if observed and not verified:
                verified=True;print('VERIFIED_ROS_RECORDING '+json.dumps(r,ensure_ascii=False),flush=True)
            if len(samples)%5==0:
                print(json.dumps(r,ensure_ascii=False),flush=True)
            if high>=3:
                reason='diagnostic_cpu_budget';break
            if now-start>30 and not r['message_counts']:
                reason='empty_recording';break
    except BaseException as error:
        reason='interrupted' if isinstance(error,KeyboardInterrupt) else 'capture_error'
        metadata['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        # These are only the two diagnostic PIDs created above.
        finalization_errors=[]
        for p in (recorder,sniffer):
            try:
                stop_own(p)
            except RuntimeError as error:
                finalization_errors.append(str(error))
        for f in logs:
            f.close()
        metadata.update(end_clock=clock_sample(),reason=reason,samples=samples,
                        recorder_returncode=recorder.poll() if recorder else None,
                        sniffer_returncode=sniffer.poll() if sniffer else None,
                        finalization_errors=finalization_errors)
        (output/'capture.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
        since='@%.9f'%(metadata['start_clock']['wall_ns']/1e9)
        after=command_output(['journalctl','-u','transfer_ros2.service','--since',since,'--no-pager','-o','json'])
        (output/'sender_journal_during.json').write_text(json.dumps(after,ensure_ascii=False))
        print(json.dumps(dict(finished=str(output),reason=reason),ensure_ascii=False),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    inspect=sub.add_parser('inspect',help='read local deployment and ROS endpoints; start no business nodes')
    inspect.add_argument('--output',required=True)
    inspect.add_argument('--motion-source',type=Path)
    capture=sub.add_parser('record',help='bounded subscriber + optional passive PCAP; never executes rotation')
    capture.add_argument('--output',required=True)
    capture.add_argument('--duration',type=duration,default=45)
    capture.add_argument('--motion-source',type=Path)
    capture.add_argument('--udp',action='store_true')
    capture.add_argument('--interface')
    capture.add_argument('--motion-peer',type=ipv4)
    capture.add_argument('--motion-port',type=int,default=43893)
    capture.add_argument('--state-port',type=int,default=43897)
    mark=sub.add_parser('mark',help='write an operator observation; this does not switch any robot mode')
    mark.add_argument('--output',required=True)
    mark.add_argument('--step',required=True,choices=['official_auto_enabled_by_operator','positive_small','negative_small','yaw90','yaw180','test_finished'])
    mark.add_argument('--note',default='')
    args=p.parse_args()
    try:
        if args.command=='record':
            if not (1<=args.motion_port<=65535 and 1<=args.state_port<=65535):
                raise ValueError('ports must be 1..65535')
            record(args)
        elif args.command=='inspect':
            output=validate_output(args.output);output.mkdir(parents=True,exist_ok=False)
            result=inspect_processes(args.motion_source)
            try:
                result['graph']=inspect_graph()
            except (ImportError,AttributeError,RuntimeError) as error:
                result['graph_error']=type(error).__name__+': '+str(error)
            (output/'deployment.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(str(output/'deployment.json'))
        else:
            output=validate_output(args.output)
            if not (output/'capture.json').is_file():
                raise ValueError('mark requires an existing R1.1 capture directory')
            event=dict(step=args.step,note=args.note,clock=clock_sample(),operator_report_only=True)
            with (output/'operator_annotations.jsonl').open('a') as f:
                f.write(json.dumps(event,ensure_ascii=False)+'\n')
            print(json.dumps(event,ensure_ascii=False))
    except (ValueError,RuntimeError,OSError) as error:
        p.exit(1,json.dumps(dict(error=str(error)),ensure_ascii=False)+'\n')

if __name__=='__main__':
    main()
