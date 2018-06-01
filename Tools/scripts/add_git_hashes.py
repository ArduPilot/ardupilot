#!/usr/bin/env python
'''
Add git hashes to .px4 file for PX4/Pixhawk build
Written by Jon Challinger January 2015
'''

import json
import sys
import os
import subprocess
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('input_file')
parser.add_argument('output_file')

parser.add_argument('--ardupilot')
parser.add_argument('--px4')
parser.add_argument('--nuttx')
parser.add_argument('--uavcan')

args = parser.parse_args()

f = open(args.input_file,'r')
fw_json = json.load(f)
f.close()

if args.ardupilot is not None:
    try:
        fw_json["ardupilot_git_hash"] = subprocess.check_output(["git", "--git-dir", os.path.join(args.ardupilot,".git"), "rev-parse", "HEAD"]).strip().decode('ascii')
    except:
        print("Failed to get apm hash")
        
if args.px4 is not None:
    try:
        fw_json["px4_git_hash"] = subprocess.check_output(["git", "--git-dir", os.path.join(args.px4,".git"), "rev-parse", "HEAD"]).strip().decode('ascii')
    except:
        print("Failed to get px4 hash")

if args.nuttx is not None:
    try:
        fw_json["nuttx_git_hash"] = subprocess.check_output(["git", "--git-dir", os.path.join(args.nuttx,".git"), "rev-parse", "HEAD"]).strip().decode('ascii')
    except:
        print("Failed to get nuttx hash")

if args.uavcan is not None:
    try:
        fw_json["uavcan_git_hash"] = subprocess.check_output(["git", "--git-dir", os.path.join(args.uavcan,".git"), "rev-parse", "HEAD"]).strip().decode('ascii')
    except:
        print("Failed to get uavcan hash")

f=open(args.output_file,'w')
json.dump(fw_json,f,indent=4)
f.truncate()
f.close()
