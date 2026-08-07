#!/usr/bin/env python3

# flake8: noqa

'''
Rsync apm.pdef.xml files for different versions of the ArduPilot firmware
For each version, it checks out the corresponding tag, generates parameter metadata,
and finally rsyncs the updated parameter metadata pdef.xml files.

SPDX-FileCopyrightText: 2024 Amilcar do Carmo Lucas <amilcar.lucas@iav.de>

SPDX-License-Identifier: GPL-3.0-or-later
'''

import os
import datetime
import shutil
import subprocess

VEHICLE_TYPES = ["Copter", "Plane", "Rover", "ArduSub", "Tracker"]  # Add future vehicle types here
RSYNC_USERNAME = 'amilcar'

# Store the current working directory
old_cwd = os.getcwd()

def get_vehicle_tags(vehicle_type):
    """
    Lists all tags in the ArduPilot repository that start with the given vehicle type followed by '-'.
    Returns a list of tag names.
    """
    try:
        # Change to the ArduPilot directory
        os.chdir('../ardupilot/')
        tags_output = subprocess.check_output(['git', 'tag', '--list', f'{vehicle_type}-[0-9]\\.[0-9]\\.[0-9]'], text=True)
        return tags_output.splitlines()
    except Exception as e: # pylint: disable=broad-exception-caught
        print(f"Error getting {vehicle_type} tags: {e}")
        return []

def generate_vehicle_versions():
    """
    Generates arrays for each vehicle type based on the tags fetched from the ArduPilot repository.
    """
    vehicle_versions = {}

    for vehicle_type in VEHICLE_TYPES:
        tags = get_vehicle_tags(vehicle_type)
        if tags:
            vehicle_versions[vehicle_type] = [tag.split('-')[1] for tag in tags]

    return vehicle_versions

def create_one_pdef_xml_file(vehicle_type: str, dst_dir: str, git_tag: str):
    os.chdir('../ardupilot')
    subprocess.run(['git', 'checkout', git_tag], check=True)
    # subprocess.run(['git', 'pull'], check=True)
    subprocess.run(['Tools/autotest/param_metadata/param_parse.py', '--vehicle', vehicle_type, '--format', 'xml'], check=True)
    # Return to the old working directory
    os.chdir(old_cwd)

    if not os.path.exists(dst_dir):
        os.makedirs(dst_dir)

    # Insert an XML comment on line 3 in the ../ardupilot/apm.pdef.xml file to indicate
    # the tag used to generate the file and the current date
    with open('../ardupilot/apm.pdef.xml', 'r', encoding='utf-8') as f:
        lines = f.readlines()
    lines.insert(2, f'<!-- Generated from git tag {git_tag} on {datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")} -->\n')
    with open('../ardupilot/apm.pdef.xml', 'w', encoding='utf-8') as f:
        f.writelines(lines)
    shutil.copy('../ardupilot/apm.pdef.xml', f'{dst_dir}/apm.pdef.xml')

# Function to sync files using rsync
def sync_to_remote(vehicle_dir: str) -> None:
    src_dir = f'{vehicle_dir}/'
    dst_user = RSYNC_USERNAME
    dst_host = 'firmware.ardupilot.org'
    dst_path = f'param_versioned/{vehicle_dir}/'

    # Construct the rsync command
    rsync_cmd = [
        'rsync',
        '-avz',
        '--progress',
        '--password-file=.rsync_pass',
        src_dir,
        f'{dst_user}@{dst_host}::{dst_path}'
    ]

    print(f'Synchronizing {src_dir} to {dst_path}...')
    print(rsync_cmd)
    subprocess.run(rsync_cmd, check=True)


def main():
    vehicle_versions = generate_vehicle_versions()

    # Iterate over the vehicle_versions list
    for vehicle_type, versions in vehicle_versions.items():

        vehicle_dir = vehicle_type
        if vehicle_type == 'ArduSub':
            vehicle_dir = 'Sub'

        for version in versions:
            if version[0] == '3' and vehicle_type != 'AP_Periph':
                continue # Skip ArduPilot 3.x versions, as param_parse.py does not support them out of the box
            if version[0] == '4' and version[2] == '0' and vehicle_type != 'ArduSub':
                continue # Skip ArduPilot 4.0.x versions, as param_parse.py does not support them out of the box
            create_one_pdef_xml_file(vehicle_type, f'{vehicle_dir}/stable-{version}', f'{vehicle_type}-{version}')

        sync_to_remote(vehicle_dir)


if __name__ == '__main__':
    main()
