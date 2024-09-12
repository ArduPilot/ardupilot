#!/usr/bin/env python3

'''
Reads lua-language-sever check

It generates a json report that this then parses to make it look nicer

AP_FLAKE8_CLEAN
'''

import optparse
import sys
import os
import pathlib
import json
import shutil
import platform
from urllib.parse import unquote
import subprocess
import re


def print_failures(file_name, fails, original_name):
    file_name = unquote(file_name)
    file_path = pathlib.Path(file_name[5:])

    if (original_name is not None) and (original_name.name == file_path.name):
        file_path = original_name

    for fail in fails:
        start = fail['range']['start']

        # These seem to be off by one, not sure why
        line = start['line'] + 1
        character = start['character'] + 1

        print("%s:%i:%i" % (file_path, line, character))

        print("\tCode: %s" % fail['code'])
        message = fail['message'].split("\n")
        for line in message:
            print("\t%s" % line)

    print()
    return len(fails)


if __name__ == '__main__':

    parser = optparse.OptionParser(__file__)

    opts, args = parser.parse_args()

    if len(args) > 1:
        print('Usage: %s "check path"' % parser.usage)
        sys.exit(0)

    check_path = "./"
    if len(args) > 0:
        check_path = args[0]
    check_path = pathlib.Path(check_path)

    if not os.path.exists(check_path):
        raise Exception("Path invalid: %s" % check_path)

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

    if tmp_check_dir is not None:
        # Remove test directory
        shutil.rmtree(tmp_check_dir)

    elif docs_copy is not None:
        # remove copy of docs
        os.remove(docs_copy)

    # Read output
    errors = 0
    result = (logs / "check.json").resolve()
    if os.path.exists(result):
        # File only created if there are errors
        f = open((logs / "check.json").resolve())
        data = json.load(f)
        f.close()

        if len(data) > 0:
            # Print output if there are any errors
            for key, value in data.items():
                errors += print_failures(key, value, original_name)

    # Remove output
    shutil.rmtree(logs)

    # Rase error if detected
    if errors != 0:
        raise Exception("Detected %i errors" % errors)

    # Warn if no files were checked
    if (checked_files is None) or (checked_files < 1):
        raise Exception("No lua files found in: %s" % check_path)

    print("%i Files checked" % checked_files)
