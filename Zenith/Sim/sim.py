#!/usr/bin/env python
"""
@brief Simulator execution script
@date Created Jan 22, 2020
@author Bertrand Bevillard <bertrand@zenithaero.com>
"""

import argparse
import os
import json
import subprocess
import shutil
import atexit

LOCAL_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.join(LOCAL_DIR, "../..")
SIM_PATH = os.path.join(ROOT_DIR, "Tools/autotest/sim_vehicle.py")
PARSER_PATH = os.path.join(LOCAL_DIR, "log_parser.py")
FP_PATH = os.path.join(LOCAL_DIR, "../flightplans/takeoff.txt")


def parse_logs():
    print("Sim exited")
    LOG_DIR = os.path.join(LOCAL_DIR, "logs")
    # Find last log
    fname = os.path.join(LOG_DIR, "LASTLOG.txt")
    if not os.path.isfile(fname):
        raise Exception("LASTLOG register not found: {}".format(fname))
    with open(fname, "r") as f:
        val = int(f.read())
    fname = os.path.join(LOG_DIR, "{:08d}.BIN".format(val))
    if not os.path.isfile(fname):
        raise Exception("Log file not found: {}".format(fname))
    # Parse log binary
    cmd = [PARSER_PATH, fname]
    os.system(" ".join(cmd))
    # subprocess.check_output(cmd)
    # with open(fname, "r")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Z1 simulation")
    parser.add_argument("--joystick", action="store_true", help="load joystick module")
    parser.add_argument("--fp", action="store_true", help="load flight plan")
    parser.add_argument("--speedup", action="store_true", help="enable speedup")
    parser.add_argument("--clear", action="store_true", help="clear sim data")
    parser.add_argument("--jsbsim", action="store_true", help="enable speedup")
    parser.add_argument("--test", action="store_true", help="enable test case")
    args = parser.parse_args()

    # Clear data if needed
    if args.clear:
        shutil.rmtree(os.path.join(LOCAL_DIR, "logs"))
        shutil.rmtree(os.path.join(LOCAL_DIR, "terrain"))
        os.remove(os.path.join(LOCAL_DIR, "eeprom.bin"))
        print("Sim data cleared")
        exit(-1)

    # Build sim command
    script = [SIM_PATH]
    frame = "plane" if args.jsbsim else "Z1"
    sim_args = ["-v", "ArduPlane", "-f", frame, "--no-rebuild", "--wipe-eeprom"]
    if args.joystick:
        sim_args += ["--joystick"]
    if args.speedup:
        sim_args += ["--speedup", "10"]
    if args.test:
        sim_args += ["--test-case", "test_case.json"]
    mav_arg_list = [
        "--logfile logs/flight.tlog",
        '--cmd-imu-ready "wp load {}"'.format(FP_PATH),
        '--cmd-fp-ready "mode auto; arm throttle"',
    ]
    mav_args = ["--mavproxy-args", json.dumps(" ".join(mav_arg_list))] if args.fp else []
    # print(" ".join(mav_args)); exit(-1)
    cmd = script + sim_args + mav_args

    atexit.register(parse_logs)
    os.system(" ".join(cmd))

    # Execute sim command. TODO: run in subprocess
    # p = subprocess.Popen(cmd, cwd=working_directory)
    # p.wait()
