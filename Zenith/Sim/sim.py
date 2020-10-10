#!/usr/bin/env python
"""
@brief Simulator execution script
@date Created Jan 22, 2020
@author Bertrand Bevillard <bertrand@zenithaero.com>
"""

import argparse
import os
import json
import shutil
import atexit
import subprocess

LOCAL_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.join(LOCAL_DIR, "../..")
SIM_PATH = os.path.join(ROOT_DIR, "Tools/autotest/sim_vehicle.py")
PARSER_PATH = os.path.join(LOCAL_DIR, "log_parser.py")
FP_PATH = os.path.join(LOCAL_DIR, "../FlightPlans/loiter.txt")
MAVPROXY_PATH = os.path.join(ROOT_DIR, "../MAVProxy/MAVProxy/mavproxy.py")
OUTPUT_DIR = os.path.join(LOCAL_DIR, "out")
LOG_DIR = os.path.join(OUTPUT_DIR, "logs")


def parse_logs():
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
    parser.add_argument("--test", action="store_true", help="enable test case")
    parser.add_argument("--debug", action="store_true", help="debug mode")
    parser.add_argument("--matlab", action="store_true", help="matlab socket communication")
    parser.add_argument("--plane", action="store_true", help="fly regular plane")
    parser.add_argument("--no-logs", action="store_true", help="don't parse logs")
    parser.add_argument("--headless", action="store_true", help="headless sim")
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
    frame = "Z1_Lookup"
    if args.matlab:
        frame = "Z1_Matlab"
    elif args.plane:
        frame = "plane"

    sim_args = ["-v", "ArduPlane", "-f", frame, "--no-rebuild", "--wipe-eeprom"]
    if args.joystick:
        sim_args += ["--joystick"]
    if args.speedup:
        sim_args += ["--speedup", "10"]
    if args.test:
        sim_args += ["--test-case", "test_case.json"]
        print(
            "WARNING: test cases should set SIM_SERVO_SPEED = -1 in parms"
        )  # TODO: create special parm for test cases
    if args.debug:
        sim_args += ["--lldb"]
    if args.headless:
        sim_args += ["--headless"]
    # Point to the right mavproxy
    sim_args += ["--mavproxy-path", MAVPROXY_PATH]
    # Takeoff at specified location
    sim_args += ["--location", "Malesherbes"]
    # Create mav args
    mav_arg_list = ["--logfile logs/flight.tlog"]
    # mav_arg_list += ["--logfile logs/flight.tlog"]
    if args.fp:
        mav_arg_list += [
            '--cmd-imu-ready "wp load {}"'.format(FP_PATH),
            '--cmd-fp-ready "mode auto; arm throttle"',
        ]
    else:
        mav_arg_list += ['--cmd "arm throttle"']
    mav_args = ["--mavproxy-args", json.dumps(" ".join(mav_arg_list))]
    # Register exit hook
    if not args.no_logs:
        atexit.register(parse_logs)
    # Run command
    cmd = ["cd", OUTPUT_DIR, ";"] + script + sim_args + mav_args
    os.system(" ".join(cmd))
    # p = subprocess.Popen(cmd, cwd=LOCAL_DIR)
    # p.wait()

    # p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)
    # grep_stdout = p.communicate(input=b'one\ntwo\nthree\nfour\nfive\nsix\n')[0]
    # Execute sim command. TODO: run in subprocess
    # p = subprocess.Popen(cmd, cwd=working_directory)
    # p.wait()

    # if True:
    #     proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    #     while True:
    #         output = proc.stdout.readline()
    #         if output == "" and proc.poll() is not None:
    #             # Terminate
    #             print("PROCESS TERMINATED")
    #             break
    #         # Handle output
    #         elif output:
    #             output_str = str(output.decode("ascii")).replace("\n", "")
    #             print("> {}".format(output_str))
    #             # Handle
    #             if "online system 1" in output_str:
    #                 print("### SENDING CMD")
    #                 fp_cmd = "wp load {}".format(FP_PATH).encode()
    #                 proc.stdin.writelines([fp_cmd])
    #                 print("### CMD SENT!")
