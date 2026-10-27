#!/usr/bin/env python3
"""Derive the zero-speed command-wait budget from our dynamic recovery tree."""
import argparse
import math
import xml.etree.ElementTree as ET


def command_wait_budget(tree):
    root = ET.parse(tree).getroot()
    recovery = root.findall('.//RecoveryNode')
    waits = root.findall('.//Wait')
    # Fail visibly if the tree's structure changes: don't silently underestimate
    # a nested/alternative recovery policy with this simple duration model.
    if len(recovery) != 1 or len(waits) != 1 or len(recovery[0]) != 2:
        raise ValueError('expected one recovery node with one Wait recovery action')
    if recovery[0][1] is not waits[0]:
        raise ValueError('Wait must be the recovery action')
    retries = int(recovery[0].attrib['number_of_retries'])
    duration = float(waits[0].attrib['wait_duration'])
    if not (0 <= retries <= 100 and math.isfinite(duration) and 0 < duration <= 60):
        raise ValueError('invalid recovery retries/duration')
    # Reserve the entire configured wait time, the same amount again for
    # planning/scheduling, plus ten seconds. This is a bounded stall fallback,
    # not a claim that Nav2 planning has a hard worst-case execution time.
    return max(60.0, math.ceil((2*retries*duration+10)/10)*10.0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('behavior_tree')
    args = parser.parse_args()
    print(command_wait_budget(args.behavior_tree))
