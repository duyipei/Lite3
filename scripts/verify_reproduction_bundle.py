#!/usr/bin/env python3
"""Read-only verification of the archived reproduction assets; no ROS/SSH."""
import hashlib
import json
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root/'reproduction_manifest.json').read_text())
    failures = []
    for name, expected in manifest['sha256'].items():
        path = root/name
        try:
            path.resolve().relative_to(root)
        except ValueError:
            failures.append(name + ': path outside project')
            continue
        try:
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            if digest.hexdigest() != expected:
                failures.append(name + ': checksum mismatch (or LFS pointer)')
        except OSError as error:
            failures.append(name + ': ' + str(error))
    for failure in failures:
        print('[FAIL]', failure)
    print('{}: {} archived files, {} failures; no hardware checks'.format(
        'FAIL' if failures else 'PASS', len(manifest['sha256']), len(failures)))
    raise SystemExit(bool(failures))


if __name__ == '__main__':
    main()
