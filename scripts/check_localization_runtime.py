#!/usr/bin/env python3
"""Verify the exact compiled runtime selected for the audited launch mode."""
import argparse
import hashlib
import json
from pathlib import Path


def verify(path):
    manifest=json.loads(Path(path).read_text())
    if manifest.get('version') != 1 or not manifest.get('files'):
        raise ValueError('missing runtime inventory')
    for name, expected in manifest['files'].items():
        actual=hashlib.sha256(Path(name).read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('runtime/source changed since build: '+name)
    return manifest


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest',type=Path)
    args=parser.parse_args()
    result=verify(args.manifest)
    print('[PASS] audited localization build: '+result.get('build_id','unknown'))
