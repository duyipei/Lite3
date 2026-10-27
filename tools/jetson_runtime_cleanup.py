#!/usr/bin/env python3
"""Plan/apply an archived-files-only cleanup of this project's Jetson copy.

Run plan on the VM after jetson_project_archive.py verify. Run apply on the
Jetson with that exact plan. Unclassified assets and runtime dependencies stay.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import sys
import time

ROOT = Path('/home/ysc/lite3_patrol_project')
LIVO = 'fast_livo2_ws/fast-livo2-deep-robotics/'
SCRIPTS = {
    'goal_pose_bridge.py', 'leg_odom_tf_bridge.py', 'rviz_ssh_bridge.py',
    'round3a_capture_backend.sh', 'round3a_lio_offline_backend.sh',
    'merge_round3a_lio_bag.py', 'mid360_isolated.launch.py', 'time_probe.py',
    'count_fast_livo_outputs.py', 'analyze_lio_odometry.py', 'profile_fast_livo.py',
    'monitor_pid.py', 'build_rgbd_visual_map.py', 'validate_visual_map_db.py',
    'make_safe_nav2_config.py', 'check_vendor_voa.py', 'render_round3a_map.py',
}
MAPS = {'indoor_fusion_20260912_001_area_review', 'outdoor_20260921_003_area_review'}
VISUAL = {'indoor_fusion_20260912_001', 'outdoor_20260921_003'}
PROTECTED = ('patrol_nav_ws/install_merged/', 'patrol_nav_ws/build_merged/',
             'patrol_nav_ws/src/', 'voa_overlay_ws/install/', LIVO+'install/',
             'fast_livo2_ws/deps_install/')
KEEP_DIRS = {'bags', 'logs', 'docs', 'scripts', 'tools', 'transfer_archives',
             LIVO.rstrip('/'), LIVO+'src', LIVO+'src/FAST-LIVO2',
             LIVO+'src/FAST-LIVO2/Log', LIVO+'src/FAST-LIVO2/Log/PCD'}


def reason_for(relative):
    p = PurePosixPath(relative)
    name = p.name
    if any(key in name.upper() for key in ('LICENSE', 'COPYING', 'COPYRIGHT', 'NOTICE')):
        return None
    if relative.startswith(PROTECTED):
        return None
    if relative.startswith(('bags/site_fusion_20260908_002/', 'bags/site_fusion_20260909_001/')):
        return 'historical_recordings'
    if relative.startswith(('logs/', 'transfer_archives/')):
        return 'historical_logs_and_transfer_files'
    if relative.startswith('docs/') and relative != 'docs/2026-09-12_indoor_lidar_rgbd_mapping.md':
        return 'documentation_kept_on_vm'
    if relative.startswith('tools/pnp_lab/') and not relative.startswith('tools/pnp_lab/deps/'):
        return 'superseded_pnp_experiments'
    if relative.startswith('results/navigation_area/'):
        return 'intermediate_map_reviews'
    if relative.startswith('results/visual_maps/'):
        parts = p.parts
        if len(parts) == 4 and parts[2] in VISUAL and name == 'visual_map.db':
            return None
        return 'inactive_visual_maps_and_previews'
    if relative.startswith('navigation/extrinsic_verify'):
        return 'old_extrinsic_experiments'
    if len(p.parts) >= 3 and p.parts[0] == 'navigation' and p.parts[1] in MAPS:
        if name in ('traversability_summary.json', 'patrol_map_summary.json'):
            return None
        if p.suffix in ('.pcd', '.pgm', '.yaml'):
            return None
        return 'map_review_images_and_evidence'
    if relative == 'navigation/README.md' or relative.startswith('calibration/reference/'):
        return 'documentation_kept_on_vm'
    if len(p.parts) == 2 and p.parts[0] == 'scripts' and name not in SCRIPTS:
        return 'vm_frontends_and_obsolete_debug_scripts'
    if relative.startswith(('patrol_nav_ws/build/', 'patrol_nav_ws/install/',
                            'patrol_nav_ws/log/', 'patrol_nav_ws/log_merged/')):
        return 'unused_unmerged_build_and_build_logs'
    if relative.startswith(('voa_overlay_ws/build/', 'voa_overlay_ws/log/', 'voa_overlay_ws/src/')):
        return 'voa_build_sources_and_logs'
    if relative.startswith((LIVO+'build/', LIVO+'build_livox_sdk2/', LIVO+'log/')):
        return 'mapping_build_cache_and_logs'
    if '.git' in p.parts or '__pycache__' in p.parts or p.suffix == '.pyc':
        return 'repository_history_and_python_cache'
    if relative.startswith((LIVO+'images/', LIVO+'src/FAST-LIVO2/Supplementary/',
                            LIVO+'src/FAST-LIVO2/pics/', LIVO+'src/Livox-SDK2/samples/')):
        return 'papers_images_and_sdk_examples'
    if relative.startswith(LIVO+'src/FAST-LIVO2/Log/') and p.suffix in ('.txt', '.csv', '.log', '.pcd'):
        return 'mapping_debug_output'
    if relative.startswith((LIVO+'src/', 'fast_livo2_ws/deps/')) and p.suffix.lower() in (
            '.cpp', '.cc', '.c', '.cxx', '.h', '.hh', '.hpp', '.hxx', '.cu'):
        return 'compiled_cpp_sources_kept_on_vm'
    return None


def checked_path(root, relative):
    p = PurePosixPath(relative)
    if not p.parts or p.is_absolute() or '..' in p.parts or str(p) != relative:
        raise ValueError('invalid cleanup path: '+relative)
    path = root.joinpath(*p.parts)
    for parent in path.parents:
        if parent == root:
            break
        if parent.is_symlink():
            raise ValueError('refusing path inside a symlink directory: '+relative)
    return path


def content_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def make_plan(directory):
    verification = json.loads((directory/'verification.json').read_text())
    manifest_path = directory/'source_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    if verification.get('verified') is not True or manifest['source_root'] != str(ROOT):
        raise ValueError('a fully verified Jetson archive is required')
    archive = Path(verification['archive'])
    if (not archive.is_file() or archive.stat().st_size != verification['archive_bytes']
            or archive.stat().st_mtime_ns != verification['archive_mtime_ns']):
        raise ValueError('verified archive is missing or changed')
    archived_digest = (directory/(archive.name+'.sha256')).read_text().split()[0]
    if verification['archive_sha256'] != archived_digest:
        raise ValueError('archive checksum changed after verification')
    if content_hash(manifest_path) != verification['source_manifest_sha256']:
        raise ValueError('verified source manifest changed')
    prefix = ROOT.name+'/'
    entries = manifest['entries']
    selected = {}
    summary = {}
    for name, detail in entries.items():
        if not name.startswith(prefix) or detail['kind'] == 'directory':
            continue
        relative = name[len(prefix):]
        reason = reason_for(relative)
        if not reason:
            continue
        data = dict(detail, reason=reason)
        if data['kind'] == 'hardlink':
            target = entries[data['linkname']]
            while target['kind'] == 'hardlink':
                target = entries[target['linkname']]
            if target['kind'] != 'file':
                raise ValueError('invalid archived hardlink')
            data.update(sha256=target['sha256'], size=target['size'])
        selected[relative] = data
        group = summary.setdefault(reason, {'files': 0, 'bytes': 0})
        group['files'] += 1
        group['bytes'] += data.get('size', 0)
    plan = {'root': str(ROOT), 'backup_verified': True, 'backup': str(archive),
            'source_manifest_sha256': content_hash(manifest_path),
            'archive_sha256': archived_digest,
            'summary': summary, 'entries': selected}
    output = directory/'cleanup_plan.json'
    output.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'plan': str(output), 'summary': summary}, ensure_ascii=False, indent=2))


def assert_idle():
    active = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit() or int(path.name) == os.getpid():
            continue
        try:
            argv = (path/'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        text = ' '.join(x.decode(errors='replace') for x in argv)
        # Commands using python -c may contain the search terms as literal source.
        if len(argv) > 1 and argv[1] == b'-c':
            continue
        if any(s in text for s in ('fastlivo_mapping', 'ros2 bag record', 'ros2 bag play',
                                   'controller_server', 'rgbd_visual_localizer', 'patrol_runner')):
            active.append({'pid': path.name, 'command': text[:250]})
    if active:
        raise RuntimeError('active robot tasks; cleanup refused: '+json.dumps(active))


def check_entry(path, detail):
    status = path.lstat()
    if status.st_mtime_ns != detail['mtime_ns']:
        raise RuntimeError('changed since backup: '+str(path))
    if detail['kind'] == 'symlink':
        if not stat.S_ISLNK(status.st_mode) or os.readlink(path) != detail['linkname']:
            raise RuntimeError('symlink differs from backup: '+str(path))
    elif detail['kind'] in ('file', 'hardlink'):
        if not stat.S_ISREG(status.st_mode) or status.st_size != detail['size']:
            raise RuntimeError('file differs from backup: '+str(path))
        if content_hash(path) != detail['sha256']:
            raise RuntimeError('checksum differs from backup: '+str(path))
    else:
        raise ValueError('unsupported deletion type')


def apply_plan(plan_path, report_path):
    plan = json.loads(plan_path.read_text())
    if plan.get('backup_verified') is not True or plan['root'] != str(ROOT) or not ROOT.is_dir():
        raise ValueError('verified backup and exact Jetson root required')
    assert_idle()
    candidates = {checked_path(ROOT, relative): detail for relative, detail in plan['entries'].items()}
    if not candidates:
        raise ValueError('empty cleanup plan')
    for path, detail in candidates.items():
        expected = reason_for(path.relative_to(ROOT).as_posix())
        if not expected or expected != detail['reason']:
            raise ValueError('plan attempts to remove protected/unclassified asset: '+str(path))
    baseline_links = {}
    for folder, dirs, files in os.walk(ROOT, followlinks=False):
        for name in dirs+files:
            path = Path(folder)/name
            if path.is_symlink() and path not in candidates:
                target = path.resolve()
                if target in candidates or any(target in x.parents for x in candidates):
                    raise RuntimeError('retained symlink depends on deletion candidate: '+str(path))
                baseline_links[str(path)] = path.exists()
    open_files = []
    names = {str(p) for p in candidates}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            descriptors = list((proc/'fd').iterdir())
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                target = os.readlink(descriptor)
                if target in names:
                    open_files.append((proc.name, target))
            except OSError:
                pass
    if open_files:
        raise RuntimeError('cleanup candidate is in use: '+repr(open_files[:10]))
    before = shutil.disk_usage(ROOT)
    progress = 0
    last = time.monotonic()
    for path, detail in candidates.items():
        check_entry(path, detail)
        progress += detail.get('size', 0)
        if time.monotonic()-last > 15:
            print(json.dumps({'predelete_verified_bytes': progress}), flush=True)
            last = time.monotonic()
    assert_idle()
    affected_directories = {parent for path in candidates for parent in path.parents
                            if parent != ROOT and ROOT in parent.parents}
    deleted = []
    for path, detail in candidates.items():
        # Recheck metadata immediately before unlinking; full hashes passed above.
        if path.lstat().st_mtime_ns != detail['mtime_ns']:
            raise RuntimeError('file changed after predelete verification: '+str(path))
        path.unlink()
        deleted.append(str(path.relative_to(ROOT)))
    # Keep all top-level deployment destinations and hardcoded output directories.
    for path in sorted(affected_directories, key=lambda p: len(p.parts), reverse=True):
        relative = path.relative_to(ROOT).as_posix()
        if path.is_symlink() or relative in KEEP_DIRS or '/' not in relative:
            continue
        if any(relative+'/' == p or relative.startswith(p) for p in PROTECTED):
            continue
        try:
            path.rmdir()
        except OSError:
            pass
    broken = [p for p, existed in baseline_links.items() if existed and not Path(p).exists()]
    report = {'deleted_count': len(deleted), 'deleted_bytes': sum(d.get('size', 0) for d in candidates.values()),
              'disk_free_before': before.free, 'disk_free_after': shutil.disk_usage(ROOT).free,
              'new_broken_symlinks': broken, 'deleted_paths': deleted,
              'backup': plan['backup'], 'archive_sha256': plan['archive_sha256']}
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    if broken:
        raise RuntimeError('unexpected broken links: '+repr(broken))
    print(json.dumps({k: v for k, v in report.items() if k != 'deleted_paths'}, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    plan = sub.add_parser('plan')
    plan.add_argument('directory', type=Path)
    apply = sub.add_parser('apply')
    apply.add_argument('plan', type=Path)
    apply.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    if args.action == 'plan':
        make_plan(args.directory)
    else:
        apply_plan(args.plan, args.report)


if __name__ == '__main__':
    main()
