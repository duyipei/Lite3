#!/usr/bin/env python3
"""Stream a private Jetson project snapshot and verify every archived file.

This tool never removes source files. Archives include machine-specific assets
and recorded images and must stay outside public source control.
"""
import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import shlex
import shutil
import subprocess
import sys
import tarfile
import threading
import time

REMOTE_ROOT = Path('/home/ysc/lite3_patrol_project')
MANIFEST = '__jetson_snapshot_manifest__.json'
CHUNK = 1024 * 1024


def emit(value, stream=sys.stdout):
    print(json.dumps(value, ensure_ascii=False), file=stream, flush=True)


def snapshot(root):
    root = root.resolve(strict=True)
    if root != REMOTE_ROOT:
        raise ValueError('unexpected Jetson project root')
    paths = [root]
    for folder, dirs, files in os.walk(root, followlinks=False):
        dirs.sort()
        files.sort()
        paths.extend(Path(folder)/name for name in dirs+files)
    total = sum(p.lstat().st_size for p in paths if p.is_file() and not p.is_symlink())
    progress = {'source_bytes_read': 0, 'source_bytes_total': total}
    last = [time.monotonic()]
    entries = {}

    class Reader:
        def __init__(self, stream):
            self.stream = stream
            self.digest = hashlib.sha256()

        def read(self, size):
            data = self.stream.read(size)
            self.digest.update(data)
            progress['source_bytes_read'] += len(data)
            if time.monotonic()-last[0] > 15:
                emit({'snapshot_progress': progress}, sys.stderr)
                last[0] = time.monotonic()
            return data

    emit({'snapshot_start': str(root), 'source_bytes': total, 'paths': len(paths)}, sys.stderr)
    with gzip.GzipFile(filename='', fileobj=sys.stdout.buffer, mode='wb', compresslevel=1,
                       mtime=0) as compressed, \
            tarfile.open(fileobj=compressed, mode='w|') as archive:
        for path in paths:
            relative = path.relative_to(root)
            name = root.name if relative == Path('.') else root.name+'/'+relative.as_posix()
            before = path.lstat()
            info = archive.gettarinfo(str(path), arcname=name)
            detail = {'mtime_ns': before.st_mtime_ns}
            if info.isfile():
                with path.open('rb') as stream:
                    reader = Reader(stream)
                    archive.addfile(info, reader)
                after = path.lstat()
                if (before.st_size, before.st_mtime_ns, before.st_ino) != (
                        after.st_size, after.st_mtime_ns, after.st_ino):
                    raise RuntimeError('file changed during snapshot: '+str(relative))
                detail.update(kind='file', size=info.size, sha256=reader.digest.hexdigest())
            elif info.issym() or info.islnk():
                archive.addfile(info)
                detail.update(kind='symlink' if info.issym() else 'hardlink', linkname=info.linkname)
            elif info.isdir():
                archive.addfile(info)
                detail.update(kind='directory')
            else:
                raise RuntimeError('unexpected special file: '+str(path))
            entries[name] = detail
        manifest = json.dumps({'source_root': str(root), 'entries': entries},
                              ensure_ascii=False, sort_keys=True).encode()
        info = tarfile.TarInfo(MANIFEST)
        info.size = len(manifest)
        info.mode = 0o600
        archive.addfile(info, io.BytesIO(manifest))
    emit({'snapshot_complete': len(entries), **progress}, sys.stderr)


def ssh_args(args):
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15',
            '-o', 'ServerAliveCountMax=3', '-i', str(args.key), args.robot]


def receive(args):
    output = args.archive.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(output.name+'.partial')
    if output.exists() or partial.exists():
        raise FileExistsError('refusing to overwrite archive: '+str(output))
    command = ['python3', args.remote_script, 'send']
    process = subprocess.Popen(ssh_args(args)+[shlex.join(command)], stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE)
    def diagnostics():
        for line in iter(process.stderr.readline, b''):
            print(line.decode(errors='replace').rstrip(), flush=True)
    reader = threading.Thread(target=diagnostics, daemon=True)
    reader.start()
    digest = hashlib.sha256()
    count = 0
    last = time.monotonic()
    try:
        with partial.open('xb') as stream:
            os.chmod(partial, 0o600)
            while True:
                block = process.stdout.read(CHUNK)
                if not block:
                    break
                stream.write(block)
                digest.update(block)
                count += len(block)
                if time.monotonic()-last > 15:
                    free = shutil.disk_usage(output.parent).free
                    emit({'archive_bytes_received': count, 'vm_free_bytes': free})
                    if free < 2*1024**3:
                        raise RuntimeError('VM reserve below 2 GiB; backup stopped before cleanup')
                    last = time.monotonic()
            stream.flush()
            os.fsync(stream.fileno())
        status = process.wait(timeout=30)
        reader.join(timeout=3)
        if status:
            raise RuntimeError('remote snapshot failed, status '+str(status))
        partial.rename(output)
        output.with_name(output.name+'.sha256').write_text(digest.hexdigest()+'  '+output.name+'\n')
        emit({'archive_received': str(output), 'bytes': count, 'sha256': digest.hexdigest()})
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def verify(args):
    archive_status = args.archive.stat()
    compressed_digest = hashlib.sha256()

    class DigestReader:
        def __init__(self, stream):
            self.stream = stream

        def read(self, size=-1):
            data = self.stream.read(size)
            compressed_digest.update(data)
            return data

    observed = {}
    manifest = None
    count = 0
    total = 0
    last = time.monotonic()
    with args.archive.open('rb') as raw, \
            gzip.GzipFile(fileobj=DigestReader(raw), mode='rb') as compressed:
        with tarfile.open(fileobj=compressed, mode='r|') as archive:
            for info in archive:
                name = info.name
                path = PurePosixPath(name)
                if path.is_absolute() or '..' in path.parts:
                    raise ValueError('unsafe archive path: '+name)
                if name == MANIFEST:
                    if manifest is not None or info.size > 16*1024*1024:
                        raise ValueError('invalid embedded manifest')
                    manifest = json.load(archive.extractfile(info))
                    continue
                if name in observed:
                    raise ValueError('duplicate archive member: '+name)
                if info.isfile():
                    digest = hashlib.sha256()
                    size = 0
                    stream = archive.extractfile(info)
                    while True:
                        data = stream.read(CHUNK)
                        if not data:
                            break
                        digest.update(data)
                        size += len(data)
                        total += len(data)
                        if time.monotonic()-last > 15:
                            emit({'verification_source_bytes': total, 'members': count})
                            last = time.monotonic()
                    if size != info.size:
                        raise ValueError('truncated archive member: '+name)
                    detail = {'kind': 'file', 'size': size, 'sha256': digest.hexdigest()}
                elif info.issym() or info.islnk():
                    detail = {'kind': 'symlink' if info.issym() else 'hardlink', 'linkname': info.linkname}
                elif info.isdir():
                    detail = {'kind': 'directory'}
                else:
                    raise ValueError('unexpected archive member: '+name)
                observed[name] = detail
                count += 1
        # Consume through the gzip footer, checking the complete compressed stream CRC.
        while compressed.read(CHUNK):
            pass
    if manifest is None:
        raise ValueError('missing source manifest')
    digest = compressed_digest.hexdigest()
    sidecar = args.archive.with_name(args.archive.name+'.sha256')
    if sidecar.exists() and sidecar.read_text().split()[0] != digest:
        raise ValueError('compressed archive differs from received stream')
    if archive_status != args.archive.stat():
        # Access time may change on read, so compare only content identity below.
        current = args.archive.stat()
        if (archive_status.st_size, archive_status.st_mtime_ns, archive_status.st_ino) != (
                current.st_size, current.st_mtime_ns, current.st_ino):
            raise ValueError('archive changed during verification')
    expected = {k: {field: value for field, value in detail.items() if field != 'mtime_ns'}
                for k, detail in manifest['entries'].items()}
    if observed != expected:
        bad = [k for k in set(observed)|set(expected) if observed.get(k) != expected.get(k)]
        raise ValueError('archive verification mismatch: '+repr(bad[:10]))
    manifest_path = args.archive.with_name('source_manifest.json')
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    report = {'verified': True, 'archive': str(args.archive.resolve()),
              'source_root': manifest['source_root'], 'members': count, 'source_bytes': total,
              'archive_bytes': archive_status.st_size, 'archive_mtime_ns': archive_status.st_mtime_ns,
              'archive_sha256': digest,
              'source_manifest_sha256': hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
              'verification': 'per-file SHA-256, gzip CRC and compressed stream SHA-256'}
    args.archive.with_name('verification.json').write_text(json.dumps(report, indent=2)+'\n')
    emit(report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('send')
    backup = sub.add_parser('backup')
    backup.add_argument('archive', type=Path)
    backup.add_argument('--robot', default='ysc@10.119.1.171')
    backup.add_argument('--key', type=Path, default=Path.home()/'.ssh/id_ed25519_cyberdog')
    backup.add_argument('--remote-script', default='/tmp/lite3_jetson_project_archive.py')
    check = sub.add_parser('verify')
    check.add_argument('archive', type=Path)
    args = parser.parse_args()
    if args.action == 'send':
        snapshot(REMOTE_ROOT)
    elif args.action == 'backup':
        receive(args)
    else:
        verify(args)


if __name__ == '__main__':
    main()
