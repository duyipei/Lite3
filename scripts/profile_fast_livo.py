#!/usr/bin/env python3
"""Summarize FAST-LIVO2 timing tables and lightweight resource samples."""

import argparse
import csv
import json
import math
import re
import statistics


ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
FLOAT = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"


def percentile(values, fraction):
    if not values:
        return None
    values = sorted(values)
    pos = fraction * (len(values) - 1)
    low = int(math.floor(pos))
    high = int(math.ceil(pos))
    if low == high:
        return values[low]
    return values[low] * (high - pos) + values[high] * (pos - low)


def summarize(values):
    return {
        "count": len(values),
        "mean_s": statistics.fmean(values) if values else None,
        "p50_s": percentile(values, 0.50),
        "p95_s": percentile(values, 0.95),
        "p99_s": percentile(values, 0.99),
        "max_s": max(values) if values else None,
    }


def parse_log(path):
    stage = None
    timings = {"lio": [], "vio": []}
    sync_lag = []
    jumps = 0
    backward = 0
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            line = ANSI.sub("", raw)
            if "LIO Mapping Time" in line:
                stage = "lio"
            elif "VIO Time" in line:
                stage = "vio"
            elif "Current Total Time" in line and stage:
                match = re.search(r"Current Total Time\s*\|\s*" + FLOAT, line)
                if match:
                    timings[stage].append(float(match.group(1)))
            match = re.search(r"not synced! delta time:\s*" + FLOAT, line, re.I)
            if match:
                sync_lag.append(float(match.group(1)))
            jumps += int("time stamp Jumps" in line or "timestamp jump" in line.lower())
            backward += int("loop back" in line.lower() or "backward" in line.lower())
    return {
        "timing": {name: summarize(values) for name, values in timings.items()},
        "sync_lag_warning_count": len(sync_lag),
        "sync_lag_s": summarize(sync_lag),
        "timestamp_jump_warning_count": jumps,
        "timestamp_backward_warning_count": backward,
    }


def parse_resources(path):
    elapsed = []
    cpu = []
    rss = []
    swap = []
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            try:
                elapsed.append(float(row["elapsed_s"]))
                cpu.append(float(row["cpu_percent_one_core"]))
                rss.append(int(row["vmrss_kib"]))
                swap.append(int(row["vmswap_kib"]))
            except (KeyError, TypeError, ValueError):
                continue
    valid_rss = [(t, v) for t, v in zip(elapsed, rss) if v > 0]
    return {
        "samples": len(elapsed),
        "span_s": (elapsed[-1] - elapsed[0]) if len(elapsed) > 1 else 0.0,
        "cpu_percent_one_core_mean": statistics.fmean(cpu) if cpu else None,
        "cpu_percent_one_core_p95": percentile(cpu, 0.95),
        "rss_initial_mib": valid_rss[0][1] / 1024 if valid_rss else None,
        "rss_final_mib": valid_rss[-1][1] / 1024 if valid_rss else None,
        "rss_peak_mib": max((v for _, v in valid_rss), default=0) / 1024,
        "swap_initial_mib": swap[0] / 1024 if swap else None,
        "swap_final_mib": swap[-1] / 1024 if swap else None,
        "swap_peak_mib": max(swap, default=0) / 1024,
    }


def parse_tegrastats(path):
    cpu_temp = []
    gpu_temp = []
    ram = []
    swap = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            match = re.search(r"RAM\s+(\d+)/(\d+)MB", line)
            if match:
                ram.append(int(match.group(1)))
            match = re.search(r"SWAP\s+(\d+)/(\d+)MB", line)
            if match:
                swap.append(int(match.group(1)))
            match = re.search(r"CPU@" + FLOAT + r"C", line)
            if match:
                cpu_temp.append(float(match.group(1)))
            match = re.search(r"GPU@" + FLOAT + r"C", line)
            if match:
                gpu_temp.append(float(match.group(1)))
    return {
        "samples": len(ram),
        "ram_peak_mib": max(ram, default=None),
        "swap_initial_mib": swap[0] if swap else None,
        "swap_final_mib": swap[-1] if swap else None,
        "swap_peak_mib": max(swap, default=None),
        "cpu_temp_peak_c": max(cpu_temp, default=None),
        "gpu_temp_peak_c": max(gpu_temp, default=None),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True)
    parser.add_argument("--resources", required=True)
    parser.add_argument("--tegrastats", required=True)
    parser.add_argument("--output")
    args = parser.parse_args()
    result = {
        "algorithm": parse_log(args.log),
        "resources": parse_resources(args.resources),
        "tegrastats": parse_tegrastats(args.tegrastats),
    }
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as handle:
            handle.write(rendered + "\n")


if __name__ == "__main__":
    main()

