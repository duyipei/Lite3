#!/usr/bin/env python3
"""Manual clock maintenance only while mapping/navigation are stopped."""
import getpass
import os
import shlex
import subprocess
import time


def main():
    synced = subprocess.check_output(
        ['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'], text=True).strip()
    if synced != 'yes':
        raise SystemExit('Refusing: VM clock is not NTP synchronized.')
    ssh = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-i',
           os.environ.get('LITE3_SSH_KEY', os.path.expanduser('~/.ssh/id_ed25519_cyberdog')),
           os.environ.get('LITE3_ROBOT_USER', 'ysc') + '@' +
           os.environ.get('LITE3_ROBOT_HOST', '10.119.1.171')]

    def remote(command, password=None):
        return subprocess.run(ssh + [command], input=password, text=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=25, check=True).stdout.strip()

    processes = remote('ps -eo comm=,args=')
    blockers = ('hdl_localization', 'ros2 bag record', 'ros2 bag play', 'planner_server',
                'controller_server', 'visual_pose_initializer', 'rgbd_visual_localizer',
                'fastlio', 'fast_livo', 'fasterlio', 'faster_lio', 'lio_mapping',
                'velocity_gate', 'patrol_runner')
    active = [line for line in processes.splitlines()
              if any(pattern in line.lower() for pattern in blockers)]
    if active:
        raise SystemExit('Stop mapping/navigation before changing time:\n' + '\n'.join(active))
    password = getpass.getpass('Jetson sudo password (not saved): ') + '\n'
    remote("sudo -S -p '' -v", password)
    before = time.time()
    epoch = before + 0.15
    command = ('date -u -s @%.6f && hwclock --systohc --utc && '
               'touch /var/lib/systemd/timesync/clock') % epoch
    print(remote("sudo -S -p '' bash -c " + shlex.quote(command), password))
    password = None
    before = time.time()
    robot = float(remote('date -u +%s.%N'))
    after = time.time()
    offset = robot - (before + after) / 2
    print('Jetson - VM: %.3f s; measurement RTT %.3f s' % (offset, after - before))
    if abs(offset) > 2:
        raise SystemExit('Clock offset exceeds 2 s; retry before starting ROS tasks.')
    print('PASS: system clock, RTC and recovery timestamp updated. Public NTP may still be offline.')


if __name__ == '__main__':
    main()
