#!/usr/bin/env python3

'''
Reads lua-language-sever check

It generates a json report that this then parses to make it look nicer

AP_FLAKE8_CLEAN
'''

import argparse
import sys
import os
import pathlib
import shutil
import platform
import subprocess
import re
import logging

if __name__ == '__main__':

    parser = argparse.ArgumentParser(__file__)
    parser.add_argument('--debugLogging', action='store_true', help='turn on some extra logging to debug the auto installer')
    parser.add_argument('check_path', default="./", help='optional input file or directory name', nargs='?')

    args = parser.parse_args()

    check_path = pathlib.Path(args.check_path)

    if not os.path.exists(check_path):
        raise Exception("Path invalid: %s" % check_path)

    # This allows us to see debug logging from github_release_downloader which sometimes fails in CI
    if args.debugLogging:
        logging.basicConfig(level=logging.DEBUG)

    # Work out full path to config file
    ap_root = (pathlib.Path(__file__) / "../../../").resolve()
    setup = (ap_root / "libraries/AP_Scripting/tests/check.json").resolve()
    logs = (ap_root / "lualogs").resolve()
    docs = (ap_root / "libraries/AP_Scripting/docs/docs.lua").resolve()

    # Make sure the log directory is empty
    if os.path.exists(logs) and len(os.listdir(logs)) != 0:
        raise Exception("lualogs not empty")

    install_path = (ap_root / "lua-language-server").resolve()

    # See if there is a new version (only try on Linux)
    if platform.system() == "Linux":
        try:
            from github_release_downloader import check_and_download_updates, GitHubRepo
        except ImportError:
            print("Import github-release-downloader failed")
            print("Install with: python3 -m pip install github-release-downloader")
            sys.exit(0)

        asset_re = re.compile(r".*linux-x64\.tar\.gz")
        check_and_download_updates(
            GitHubRepo("LuaLS", "lua-language-server"),
            assets_mask=asset_re,
            downloads_dir=ap_root,
        )
        for filename in os.listdir(ap_root):
            if asset_re.match(filename):
                pack_path = (ap_root / filename).resolve()
                shutil.unpack_archive(pack_path, install_path)
                os.remove(pack_path)

    run_path = (install_path / "bin/lua-language-server")
    if not os.path.isfile(run_path):
        # Try and use version from path
        run_path = "lua-language-server"

    # If the target is a single script copy it to a new directory
    tmp_check_dir = None
    original_name = None
    if os.path.isfile(check_path):
        tmp_check_dir = (check_path / "../tmp_llc").resolve()
        os.mkdir(tmp_check_dir)
        shutil.copyfile(check_path, (tmp_check_dir / check_path.name).resolve())
        original_name = check_path
        check_path = tmp_check_dir

    # Can't get the lua-language-server to find docs outside of workspace, so just copy in and then delete
    docs_check_path = (pathlib.Path(os.getcwd()) / check_path).resolve()
    if os.path.isfile(docs_check_path):
        docs_check_path = (docs_check_path / "../").resolve()

    docs_copy = None
    if not docs.is_relative_to(docs_check_path):
        docs_copy = (docs_check_path / "docs.lua").resolve()

    if docs_copy is not None:
        # make copy of docs
        shutil.copyfile(docs, docs_copy)

    # Run check, print output in real time for user and capture so we can confirm it has found at least one file
    command = "%s --configpath %s --logpath %s --check %s" % (run_path, setup, logs, check_path)
    p = subprocess.Popen(command, shell=True, text=True, stdout=subprocess.PIPE)
    result = []
    while p.poll() is None:
        line = p.stdout.readline()
        result.append(line)
        print(line, end="")

    # Make sure we checked at least one file
    file_count_re = re.compile(r"^>*=* \d+/(\d+)")
    checked_files = None
    for line in result:
        match = file_count_re.search(line)
        if match is not None:
            count = int(match.group(1))
            if checked_files is None:
                checked_files = count
            elif checked_files != count:
                raise Exception("Checked files error expected: %i got: %i" % (checked_files, count))

    # Grab the summary line to so we can error out for errors
    summary = None
    summary_re = re.compile(r"^Diagnosis (?:complete|completed), (\d+|no) problems found")
    for line in result:
        match = summary_re.search(line)
        if match is not None:
            summary = match

    if summary is None:
        raise Exception("Could not complete Diagnosis")

    # Get number of errors
    errors = 0
    if summary.group(1) != "no":
        errors = int(summary.group(1))

    if tmp_check_dir is not None:
        # Remove test directory
        shutil.rmtree(tmp_check_dir)

    elif docs_copy is not None:
        # remove copy of docs
        os.remove(docs_copy)

    # Remove output
    shutil.rmtree(logs)

    # Rase error if detected
    if errors != 0:
        raise Exception("Detected %i errors" % errors)

    # Warn if no files were checked
    if (checked_files is None) or (checked_files < 1):
        raise Exception("No lua files found in: %s" % check_path)

    print("%i Files checked" % checked_files)
