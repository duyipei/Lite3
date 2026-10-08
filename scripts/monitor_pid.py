#!/usr/bin/env python3
"""Record CPU/RAM/swap for one validation process."""

import argparse
import csv
import os
import time


def status(pid):
    values = {}
    with open(f"/proc/{pid}/status", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith(("VmRSS:", "VmHWM:", "VmSwap:")):
                key, value, _unit = line.split()
                values[key.rstrip(":")] = int(value)
    with open(f"/proc/{pid}/stat", encoding="utf-8") as handle:
        fields = handle.read().split()
    values["cpu_ticks"] = int(fields[13]) + int(fields[14])
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pid", type=int)
    parser.add_argument("output")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=300.0)
    args = parser.parse_args()

    clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    started = time.monotonic()
    previous_time = None
    previous_ticks = None
    with open(args.output, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "elapsed_s", "realtime_ns", "cpu_percent_one_core",
            "vmrss_kib", "vmhwm_kib", "vmswap_kib", "load1",
        ])
        writer.writeheader()
        while time.monotonic() - started <= args.duration:
            try:
                now = time.monotonic()
                sample = status(args.pid)
            except FileNotFoundError:
                break
            cpu = 0.0
            if previous_time is not None:
                cpu_seconds = (sample["cpu_ticks"] - previous_ticks) / clock_ticks
                cpu = 100.0 * cpu_seconds / (now - previous_time)
            writer.writerow({
                "elapsed_s": f"{now - started:.3f}",
                "realtime_ns": time.time_ns(),
                "cpu_percent_one_core": f"{cpu:.2f}",
                "vmrss_kib": sample.get("VmRSS", 0),
                "vmhwm_kib": sample.get("VmHWM", 0),
                "vmswap_kib": sample.get("VmSwap", 0),
                "load1": f"{os.getloadavg()[0]:.3f}",
            })
            handle.flush()
            previous_time = now
            previous_ticks = sample["cpu_ticks"]
            time.sleep(args.interval)


if __name__ == "__main__":
    main()

