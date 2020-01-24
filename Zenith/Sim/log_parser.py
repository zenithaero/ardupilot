#!/usr/bin/env python
"""
@brief Ardupilot log parser
@date Created Jan 22, 2020
@author Bertrand Bevillard <bertrand@zenithaero.com>
"""

import os
import sys
from os.path import dirname
import subprocess
import json
import argparse
from scipy import io as scipyio

ROOT_DIR = os.path.join(os.path.dirname(__file__), "../..")
MAVLOG_PATH = os.path.join(ROOT_DIR, "modules/mavlink/pymavlink/tools/mavlogdump.py")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Z1 simulation")
    parser.add_argument("path", type=str, help="path of the log file")
    args = parser.parse_args()

    print("Parsing logs... ")
    cmd = [sys.executable, MAVLOG_PATH, "--planner", "--format", "json", args.path]
    out = subprocess.check_output(cmd).decode("utf-8")
    logs = {}
    start_time = None
    for l in out.split("\n"):
        if not l:
            continue
        msg = json.loads(l)
        msg_type = msg["meta"]["type"]
        msg_timestamp = msg["meta"]["timestamp"]
        if not start_time:
            start_time = msg_timestamp
        msg_timestamp -= start_time
        msg_data = msg["data"]
        if msg_type not in logs:
            logs[msg_type] = {}
        type_dict = logs[msg_type]
        msg_data["timestamp"] = msg_timestamp
        for k, v in msg_data.items():
            if k not in type_dict:
                type_dict[k] = []
            type_dict[k].append(v)
    # Now export the logs as a matlab matrix
    logdir = os.path.dirname(args.path)
    logpath = os.path.join(logdir, "log.mat")
    scipyio.savemat(logpath, logs)
    print("Done")
