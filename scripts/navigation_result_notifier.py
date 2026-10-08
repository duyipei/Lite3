#!/usr/bin/env python3
"""Print concise operator reminders when a bridged Nav2 goal finishes."""

import argparse
import os
import re
import time


RESULT_PATTERN = re.compile(
    r"Nav2 bridged goal (?P<goal>\d+) finished with status (?P<status>\d+)"
)


def result_message(goal, status):
    if status == 4:
        return (
            "[NAV DONE] 目标 #{} 已到达；机器狗保持 ARMED 零速待命，"
            "可直接在 RViz 点击下一个目标。".format(goal)
        )
    if status == 5:
        return "[NAV CANCELED] 目标 #{} 已取消，机器狗保持零速。".format(goal)
    if status == 6:
        return (
            "[NAV FAILED] 目标 #{} 导航失败；请先输入 STATUS 检查状态，"
            "不要直接重发远距离目标。".format(goal)
        )
    return "[NAV ENDED] 目标 #{} 已结束，Nav2 状态码={}。".format(goal, status)


def follow(log_path):
    while not os.path.exists(log_path):
        time.sleep(0.1)
    with open(log_path, "r", encoding="utf-8", errors="replace") as stream:
        stream.seek(0, os.SEEK_END)
        while True:
            line = stream.readline()
            if not line:
                time.sleep(0.1)
                continue
            match = RESULT_PATTERN.search(line)
            if match is None:
                continue
            message = result_message(
                int(match.group("goal")), int(match.group("status"))
            )
            print("\n" + message, flush=True)
            print("导航控制> ", end="", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True)
    args = parser.parse_args()
    follow(args.log)


if __name__ == "__main__":
    main()
