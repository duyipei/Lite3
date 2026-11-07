#!/usr/bin/env python3
"""Offline ROS-bag / PCAP rotation audit. Never starts a node or sends packets."""
import argparse
import bisect
import json
import math
import socket
import sqlite3
import struct
from pathlib import Path

REFERENCE_COMMIT = 'e4ab5dc1c7f3ff46274162a1ef302253936578aa'
COMMANDS = {320: 'linear_x', 325: 'linear_y', 321: 'angular_z_wire'}


def decode_motion_payload(payload):
    """Decode only the audited packed-4 upstream command layout, not firmware ACK."""
    if len(payload) != 20:
        return None
    code, size, kind, value = struct.unpack('<iiid', payload)
    if code not in COMMANDS or size != 8 or kind != 1 or not math.isfinite(value):
        return None
    return dict(code=code, component=COMMANDS[code], wire_value=value,
                ros_reference_value=-value if code == 321 else value,
                layout='upstream_reference_only', reference_commit=REFERENCE_COMMIT)


def udp_from_frame(data, linktype):
    if linktype == 1:  # Ethernet, including bounded VLAN tags.
        if len(data) < 14:
            return None
        protocol = struct.unpack('!H', data[12:14])[0]
        offset = 14
        for _ in range(2):
            if protocol not in (0x8100, 0x88a8):
                break
            if len(data) < offset + 4:
                return None
            protocol = struct.unpack('!H', data[offset+2:offset+4])[0]
            offset += 4
        if protocol != 0x0800:
            return None
    elif linktype == 113:  # Linux cooked v1.
        if len(data) < 16 or data[14:16] != b'\x08\x00':
            return None
        offset = 16
    elif linktype == 276:  # Linux cooked v2.
        if len(data) < 20 or data[:2] != b'\x08\x00':
            return None
        offset = 20
    elif linktype in (101, 228):
        offset = 0
    else:
        raise ValueError('unsupported PCAP linktype: %s' % linktype)
    ip = data[offset:]
    if len(ip) < 20 or ip[0] >> 4 != 4 or ip[9] != 17:
        return None
    ihl = (ip[0] & 15) * 4
    total = struct.unpack('!H', ip[2:4])[0]
    if ihl < 20 or len(ip) < total or total < ihl+8:
        return None
    if struct.unpack('!H', ip[6:8])[0] & 0x3fff:  # Do not invent fragment reassembly.
        return None
    sport, dport, length, _ = struct.unpack('!HHHH', ip[ihl:ihl+8])
    if length < 8 or ihl+length > total:
        return None
    return dict(source=socket.inet_ntoa(ip[12:16]), target=socket.inet_ntoa(ip[16:20]),
                source_port=sport, target_port=dport, payload=ip[ihl+8:ihl+length])


def read_pcap(path, peer=None, port=43893):
    magics = {b'\xd4\xc3\xb2\xa1': ('<', 1000000), b'\xa1\xb2\xc3\xd4': ('>', 1000000),
              b'\x4d\x3c\xb2\xa1': ('<', 1000000000), b'\xa1\xb2\x3c\x4d': ('>', 1000000000)}
    with Path(path).open('rb') as f:
        magic = f.read(4)
        if magic not in magics:
            raise ValueError('classic PCAP required; PCAPNG / invalid file is not silently accepted')
        endian, scale = magics[magic]
        header = f.read(20)
        if len(header) != 20:
            raise ValueError('truncated PCAP header')
        linktype = struct.unpack(endian+'HHIIII', header)[-1]
        while True:
            h = f.read(16)
            if not h:
                break
            if len(h) != 16:
                raise ValueError('truncated PCAP record header')
            seconds, fraction, captured, original = struct.unpack(endian+'IIII', h)
            if captured > 65535*4:
                raise ValueError('unexpected PCAP record size')
            frame = f.read(captured)
            if len(frame) != captured:
                raise ValueError('truncated PCAP record data')
            packet = udp_from_frame(frame, linktype)
            if packet is None:
                continue
            payload = packet.pop('payload')
            packet.update(receive_ns=seconds*1000000000+int(fraction*1000000000//scale),
                          captured_bytes=captured, original_bytes=original)
            command = decode_motion_payload(payload)
            packet['payload_hex'] = payload.hex()
            packet['reference_command'] = (command if packet['target_port'] == port and
                                           (peer is None or packet['target'] == peer) else None)
            # RobotState enums / control-mode ACK are not documented by this decoder.
            packet['autonomy_confirmed'] = None
            yield packet


def pure(v, epsilon=1e-8):
    return abs(v['x']) <= epsilon and abs(v['y']) <= epsilon and abs(v['w']) > epsilon


def summarize(records, max_angular=0.4, udp_packets=()):
    records = sorted(records, key=lambda r: r['receive_ns'])
    raw = [r for r in records if r['topic'] == '/patrol/cmd_vel_raw']
    output = [r for r in records if r['topic'] == '/cmd_vel_corrected']
    report = {'clock': 'bag / PCAP receive wall time; original source stamps retained',
              'reference_commit': REFERENCE_COMMIT,
              'motion_sender_callback_twist': 'unmeasured; a separate ROS subscriber is not MotionSender',
              'motion_mode_feedback': {'autonomy_confirmed': None,
                  'reason': 'operator annotations are not a motion-host ACK; no documented mode enum decoded'},
              'limits': 'No packet capture means UDP delivery is unmeasured; egress is not receiver ACK.'}
    for name, rows in [('raw', raw), ('output', output)]:
        rotations = [r for r in rows if pure(r)]
        report[name] = dict(messages=len(rows), pure_rotation=len(rotations),
                            positive=sum(r['w'] > 0 for r in rotations),
                            negative=sum(r['w'] < 0 for r in rotations),
                            min_w=min((r['w'] for r in rotations), default=None),
                            max_w=max((r['w'] for r in rotations), default=None))
    output_times = [r['receive_ns'] for r in output]
    matches = 0
    unmatched = []
    for r in raw:
        if not pure(r):
            continue
        start = bisect.bisect_left(output_times, r['receive_ns'])
        stop = bisect.bisect_right(output_times, r['receive_ns']+150000000)
        expected = max(-max_angular, min(max_angular, r['w']))
        if any(pure(o) and abs(o['w']-expected)<1e-8 for o in output[start:stop]):
            matches += 1
        else:
            unmatched.append(dict(receive_ns=r['receive_ns'], w=r['w']))
    report['raw_pure_matched_output_within_150ms'] = matches
    report['unmatched_raw_pure'] = unmatched
    report['unmatched_interpretation'] = 'observation gaps / scheduling / gate state need review; not proof of filtering'
    transitions = []
    last = {}
    for r in records:
        if 'state' in r and last.get(r['topic']) != r['state']:
            transitions.append(r)
            last[r['topic']] = r['state']
    report['state_transitions'] = transitions
    packets = list(udp_packets)
    commands = [p for p in packets if p.get('reference_command')]
    report['udp'] = dict(packets=len(packets), reference_commands=len(commands),
                         yaw_packets=sum(p['reference_command']['code'] == 321 for p in commands),
                         status='observed egress; not a motion-host ACK' if commands else 'unmeasured or no supported packets',
                         process_attribution='PCAP alone does not prove which PID sent a packet')
    segments = []
    current = []
    for r in output:
        if pure(r):
            if current and (r['receive_ns']-current[-1]['receive_ns']>250000000 or r['w']*current[-1]['w']<0):
                segments.append(current); current=[]
            current.append(r)
        elif current:
            segments.append(current); current=[]
    if current:
        segments.append(current)
    measured = []
    for segment in segments:
        begin, end = segment[0]['receive_ns'], segment[-1]['receive_ns']
        if end-begin < 300000000:
            continue
        item=dict(start_receive_ns=begin, end_receive_ns=end,
                  duration_s=(end-begin)/1e9, mean_command_w=sum(r['w'] for r in segment)/len(segment), yaw={})
        for topic in ('/odom', '/leg_odom2', '/imu/data'):
            poses=[r for r in records if r['topic']==topic and 'yaw' in r and begin<=r['receive_ns']<=end]
            if len(poses)<2:
                item['yaw'][topic]=dict(status='unmeasured', samples=len(poses))
                continue
            delta=sum(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])) for a,b in zip(poses,poses[1:]))
            item['yaw'][topic]=dict(status='observed feedback; not independent physical proof',samples=len(poses),
                                    delta_rad=delta,delta_deg=math.degrees(delta))
        measured.append(item)
    report['pure_output_segments'] = measured
    return report


def load_bag(path, until_wall=None):
    # Deserializing stored messages does not call rclpy.init or create a node.
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    root=Path(path)
    files=[root] if root.suffix=='.db3' else sorted(root.glob('*.db3'))
    if not files:
        raise ValueError('no rosbag SQLite database at '+str(root))
    rows, counts, errors = [], {}, []
    for file in files:
        db=sqlite3.connect('file:'+str(file)+'?mode=ro',uri=True)
        topics={r[0]:(r[1],r[2]) for r in db.execute('select id,name,type from topics')}
        for tid,(name,typ) in topics.items():
            query='select timestamp,data from messages where topic_id=?'
            args=[tid]
            if until_wall is not None:
                query+=' and timestamp<=?';args.append(round(until_wall*1e9))
            query+=' order by timestamp'
            if name not in ('/patrol/cmd_vel_raw','/cmd_vel_corrected','/patrol/gate_state','/patrol/armed',
                            '/patrol/goal_bridge_state','/patrol/localization_ready','/patrol/goal_request',
                            '/odom','/leg_odom2','/imu/data'):
                continue
            try:
                cls=get_message(typ)
                for t,data in db.execute(query,args):
                    m=deserialize_message(data,cls);counts[name]=counts.get(name,0)+1
                    r=dict(topic=name,receive_ns=t)
                    if hasattr(m,'header'):
                        r.update(source_stamp_ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec,
                                 frame=m.header.frame_id)
                    if name in ('/patrol/cmd_vel_raw','/cmd_vel_corrected'):
                        r.update(x=m.linear.x,y=m.linear.y,w=m.angular.z)
                    elif name in ('/odom','/leg_odom2','/imu/data','/patrol/goal_request'):
                        q=m.orientation if name=='/imu/data' else m.pose.orientation if name=='/patrol/goal_request' else m.pose.pose.orientation
                        if name=='/imu/data' and m.orientation_covariance[0]<0:
                            continue
                        values=(q.x,q.y,q.z,q.w)
                        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1)>0.02:
                            continue
                        r['yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    else:
                        r['state']=m.data
                    rows.append(r)
            except (ImportError,AttributeError,ValueError,RuntimeError) as error:
                errors.append(dict(topic=name,error=type(error).__name__+': '+str(error)))
        db.close()
    return rows,counts,errors


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path)
    p.add_argument('--pcap',type=Path)
    p.add_argument('--motion-peer')
    p.add_argument('--motion-port',type=int,default=43893)
    p.add_argument('--until-wall',type=float)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    records,counts,errors=load_bag(args.bag,args.until_wall)
    packets=list(read_pcap(args.pcap,args.motion_peer,args.motion_port)) if args.pcap else []
    report=summarize(records,udp_packets=packets)
    report.update(topic_counts=counts,decode_errors=errors,completeness='partial' if errors or not packets else 'ROS and UDP observed; callback/mode ACK still unmeasured')
    capture_root=args.bag.parent if args.bag.name=='bag' or args.bag.suffix=='.db3' else args.bag
    annotations=capture_root/'operator_annotations.jsonl'
    report['operator_annotations']=[json.loads(line) for line in annotations.read_text().splitlines() if line] if annotations.exists() else []
    journal=capture_root/'sender_journal_during.json'
    report['motion_sender_callback_entry_logs']=None
    if journal.exists():
        text=json.loads(journal.read_text()).get('stdout','')
        report['motion_sender_callback_entry_logs']=text.count('recevied cmd_vel')+text.count('received cmd_vel')
        report['sender_log_limit']='A callback-entry count does not expose its angular.z or prove successful board execution.'
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'rotation_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    (args.output/'udp_decoded.json').write_text(json.dumps(packets,ensure_ascii=False,indent=2))
    print(json.dumps({k:report[k] for k in ('raw','output','raw_pure_matched_output_within_150ms','udp','completeness')},ensure_ascii=False,indent=2))

if __name__=='__main__':
    main()
