#!/usr/bin/env python3
"""Compatibility entry for historical preview commands."""
import runpy
from pathlib import Path

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).with_name('camera_preview.py')), run_name='__main__')
