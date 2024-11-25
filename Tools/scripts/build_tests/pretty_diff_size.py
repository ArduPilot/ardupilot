#!/usr/bin/env python

'''
This script intend to provide a pretty size diff between two binaries.

AP_FLAKE8_CLEAN
'''

import os
from argparse import ArgumentParser
from tabulate import tabulate

parser = ArgumentParser(description="Print binary size difference with master.")
parser.add_argument("-m", "--master", dest='master', type=str, help="Master Binaries Path", required=True)
parser.add_argument("-s", "--second", dest='second', type=str, help="Second Binaries Path", required=True)

args = parser.parse_args()


def sizes_for_file(filepath):
    """Get binary size information with size."""
    print("Get binary size of %s" % filepath)
    cmd = "size %s" % (filepath,)
    stuff = os.popen(cmd).read()
    lines = stuff.splitlines()[1:]
    size_list = []
    for line in lines:
        row = line.strip().split()
        size_list.append(dict(
            text=int(row[0]),
            data=int(row[1]),
            bss=int(row[2]),
            total=int(row[3]),
        ))

    # Get the size of .crash_log to remove it from .bss reporting
    cmd = "size -A %s" % (filepath,)
    output = os.popen(cmd).read()
    lines = output.splitlines()[1:]
    for line in lines:
        if ".crash_log" in line:
            row = line.strip().split()
            size_list[0]["crash_log"] = int(row[1])
            break

    # check if crash_log wasn't found and set to 0B if not found
    # FIX ME : so it doesn't report Flash_Free 0B for non-ARM boards
    if size_list[0].get("crash_log") is None:
        size_list[0]["crash_log"] = 0

    return size_list


def print_table(summary_data_list_second, summary_data_list_master):
    """Print the binaries size diff on a table."""
    print_data = []
    print("")
    # print(summary_data_list_second)
    # print(summary_data_list_master)
    for name in summary_data_list_second[0]:
        for master_name in summary_data_list_master[0]:
            if name == master_name:
                col_data = [name]
                for key in ["text", "data", "bss", "total"]:
                    bvalue = summary_data_list_second[0][name].get(key)
                    mvalue = summary_data_list_master[0][name].get(key)

                    # BSS: remove the portion occupied by crash_log as the command `size binary.elf`
                    # reports BSS with crash_log included
                    if key == "bss":
                        mvalue = (summary_data_list_master[0][name].get("bss") -
                                  summary_data_list_master[0][name].get("crash_log"))
                        bvalue = (summary_data_list_second[0][name].get("bss") -
                                  summary_data_list_second[0][name].get("crash_log"))

                    # Total Flash Cost = Data + Text
                    if key == "total":
                        mvalue = (summary_data_list_master[0][name].get("text") +
                                  summary_data_list_master[0][name].get("data"))
                        bvalue = (summary_data_list_second[0][name].get("text") +
                                  summary_data_list_second[0][name].get("data"))
                    diff = (bvalue - mvalue) * 100.0 / mvalue
                    signum = "+" if diff > 0.0 else ""
                    print_diff = str(bvalue - mvalue)
                    print_diff += " (" + signum + "%0.4f%%" % diff + ")"
                    col_data.append(print_diff)

                # Append free flash space which is equivalent to crash_log's size
                col_data.append(str(summary_data_list_second[0][name].get("crash_log")))
                print_data.append(col_data)
    print(tabulate(print_data, headers=["Binary Name", "Text [B]", "Data [B]", "BSS (B)",
                                        "Total Flash Change [B] (%)", "Flash Free After PR (B)"]))


def extract_binaries_size(path):
    """Search and extract binary size for each binary in the given path."""
    print("Extracting binaries size on %s" % path)
    binaries_list = []
    for file in os.listdir(args.master):
        fileNoExt = os.path.splitext(file)[0]
        binaries_list.append(fileNoExt)
    binaries_list = list(dict.fromkeys(binaries_list))
    if len(binaries_list) == 0:
        print("Failed to get binaries")

    size_dict = None
    for binaries in binaries_list:
        binary_path = os.path.join(path, binaries)
        parsed = sizes_for_file(binary_path)
        if size_dict is None:
            size_dict = [{binaries.lower(): parsed[0]}]
        else:
            size_dict[0].update({binaries.lower(): parsed[0]})
    print("Success !")
    return size_dict


master_dict = extract_binaries_size(args.master)
second_dict = extract_binaries_size(args.second)

print_table(second_dict, master_dict)
