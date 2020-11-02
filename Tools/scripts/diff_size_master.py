#!/usr/bin/env python
import os
import json
from argparse import ArgumentParser
import requests
from tabulate import tabulate


parser = ArgumentParser(description="Print binary size difference with master.")
parser.add_argument("-b", "--board", dest='board', type=str, help="Board type to use", required=True)

args = parser.parse_args()


def buildlogs_dirpath():
    return os.getenv("BUILDLOGS",
                     os.path.join(os.getcwd(), "..", "buildlogs"))


def get_board_json():
    print("Looking for board build result into the file ...")
    if args.board.lower() in master_json[0]:
        print("Find %s firmware sizes" % args.board)
        return [master_json[0].get(args.board.lower())]
    else:
        raise Exception("Cannot find %s firmware sizes" % args.board)


def sizes_for_file(filepath):
    cmd = "size %s" % (filepath,)
    stuff = os.popen(cmd).read()
    lines = stuff.splitlines()[1:]
    l = []
    for line in lines:
        row = line.strip().split()
        l.append(dict(
            text=int(row[0]),
            data=int(row[1]),
            bss=int(row[2]),
            total=int(row[3]),
        ))
    return l


def print_table(summary_data_list, summary_data_list_master):
    print_data = []
    print(summary_data_list)
    print(summary_data_list_master)
    for name in summary_data_list[0]:
        for master_name in summary_data_list_master[0]:
            if name == master_name:
                col_data = [name]
                for key in ["text", "data", "bss", "total"]:
                    bvalue = summary_data_list[0][name].get(key)
                    mvalue = summary_data_list_master[0][name].get(key)
                    if key == "total" and mvalue is None:
                        mvalue = summary_data_list_master[0][name].get("text") + summary_data_list_master[0][name].get("data") + summary_data_list_master[0][name].get("bss")
                    diff = (bvalue - mvalue) * 100.0 / mvalue
                    signum = "+" if diff > 0.0 else ""
                    print_diff = signum + "%0.4f%%" % diff
                    col_data.append(print_diff)
                print_data.append(col_data)
    print(tabulate(print_data, headers=["Binary", "text", "data", "bss", "total"]))


master_json = None
board_json = []

url = "https://autotest.ardupilot.org/firmware_sizes.json"
local_url = os.path.join(buildlogs_dirpath(), "firmware_sizes.json")

print("Try to get firmware_size.json from autotest.ardupilot.org...")
result = requests.get(url)
if result.status_code == requests.codes.ok:
    print("Success !")
    master_json = result.json()
    board_json = get_board_json()
else:
    print("Failed !")
    print("Try to get firmware_size.json from local buildlogs directory")
    if os.path.isfile(local_url):
        print("Success !")
        with open(local_url, "r+") as summary_json:
            master_json = json.load(summary_json)
        board_json = get_board_json()
    else:
        print("Failed !")
        print("No firmware_size.json present, diff is not possible")
        exit(2)

scripts_dir = os.path.dirname(os.path.realpath(__file__))
root_dir = os.path.realpath(os.path.join(scripts_dir, '../..'))
binary_basedir = "build/%s/bin/" % args.board
vehicle_binary = os.path.join(root_dir, binary_basedir)

# Try to look if that is a debug build
print("Looking for %s build directory ..." % args.board)
with open(vehicle_binary + "../compile_commands.json", "r") as searchfile:
    print("Success !")
    for line in searchfile:
        if "-DHAL_DEBUG_BUILD=1" in line:
            print("Cannot be used with debug build")
            exit(2)


print("Extracting %s build summary ..." % args.board)
binaries_list = []
for file in os.listdir(vehicle_binary):
    fileNoExt = os.path.splitext(file)[0]
    binaries_list.append(fileNoExt)
binaries_list = list(dict.fromkeys(binaries_list))

build_json = None
for binaries in binaries_list:
    path = vehicle_binary + binaries
    parsed = sizes_for_file(path)
    if build_json is None:
        build_json = [{binaries.lower(): parsed[0]}]
    else:
        build_json[0].update({binaries.lower(): parsed[0]})
print("Success !")

print_table(build_json, board_json)
