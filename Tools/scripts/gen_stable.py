#!/usr/bin/env python3

# flake8: noqa

'''
create stable-x.y.z directories so we keep all past stable releases for users to download
'''

import os
import shutil

VEHICLES = ['AntennaTracker', 'Copter', 'Plane', 'Rover', 'Sub']

# beta directories that may contain stable builds
BETA_DIRS = []

def copy_metadata(old_dir, new_dir_parent):
    '''copy tag-level metadata into a new versioned release directory'''
    metadata_dir = os.path.join(old_dir, '__METADATA__')
    new_metadata_dir = os.path.join(new_dir_parent, '__METADATA__')
    if not os.path.isdir(metadata_dir) or os.path.exists(new_metadata_dir):
        return
    shutil.copytree(metadata_dir, new_metadata_dir)


def stable_versions(directory, official_only=False):
    '''return the set of firmware versions represented by a build directory'''
    versions = set()
    for board in os.listdir(directory):
        if board == '__METADATA__':
            continue
        vfile = os.path.join(directory, board, 'firmware-version.txt')
        if not os.path.isfile(vfile):
            continue
        version_parts = open(vfile).read().strip().split('-', 1)
        if len(version_parts) != 2:
            continue
        if official_only and version_parts[1] != 'FIRMWARE_VERSION_TYPE_OFFICIAL':
            continue
        versions.add(version_parts[0])
    return versions

def make_stable(basedir, vehicle):
    '''make stable version for a vehicle'''
    stable_dir = os.path.join(basedir, vehicle, 'stable')
    if not os.path.exists(stable_dir):
        print("Missing %s" % stable_dir)
        return
    versions = stable_versions(stable_dir)
    copy_metadata_for_release = len(versions) == 1
    for b in sorted(os.listdir(stable_dir)):
        if b == '__METADATA__':
            continue
        if not os.path.isdir(os.path.join(stable_dir, b)):
            continue
        vfile = os.path.join(stable_dir, b, "firmware-version.txt")
        if not os.path.exists(vfile):
            print("Missing %s" % vfile)
            continue
        vstr = open(vfile).read()
        version = vstr.split('-')[0]
        new_dir_parent = os.path.join(basedir, vehicle, 'stable-%s' % version)
        new_dir = os.path.join(new_dir_parent, b)
        os.makedirs(new_dir_parent, exist_ok=True)
        if copy_metadata_for_release:
            copy_metadata(stable_dir, new_dir_parent)
        if os.path.exists(new_dir):
            continue
        print('Creating %s' % new_dir)
        old_dir = os.path.join(stable_dir, b)
        shutil.copytree(old_dir, new_dir)

def make_stable_from_beta(basedir, vehicle, beta_dir):
    '''make stable version from a beta with OFFICIAL tag'''
    beta_dir = os.path.join(basedir, vehicle, beta_dir)
    if not os.path.exists(beta_dir):
        return
    versions = stable_versions(beta_dir, official_only=True)
    copy_metadata_for_release = len(versions) == 1
    for b in sorted(os.listdir(beta_dir)):
        if b == '__METADATA__':
            continue
        if not os.path.isdir(os.path.join(beta_dir, b)):
            continue
        vfile = os.path.join(beta_dir, b, "firmware-version.txt")
        if not os.path.exists(vfile):
            print("Missing %s" % vfile)
            continue
        vstr = open(vfile).read().strip()
        a = vstr.split('-')
        if len(a) != 2:
            continue
        version = a[0]
        vtype = a[1]
        #print(vfile, b, version, vtype)
        if vtype != 'FIRMWARE_VERSION_TYPE_OFFICIAL':
            # not a new stable
            continue
        new_dir_parent = os.path.join(basedir, vehicle, 'stable-%s' % version)
        new_dir = os.path.join(new_dir_parent, b)
        os.makedirs(new_dir_parent, exist_ok=True)
        if copy_metadata_for_release:
            copy_metadata(beta_dir, new_dir_parent)
        if os.path.exists(new_dir):
            continue
        print('Creating %s' % new_dir)
        old_dir = os.path.join(beta_dir, b)
        shutil.copytree(old_dir, new_dir)
        

def make_all_stable(basedir):
    '''make stable directory for all vehicles'''
    for v in VEHICLES:
        make_stable(basedir, v)
        for b in BETA_DIRS:
            make_stable_from_beta(basedir, v, b)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='gen_stable.py')
    parser.add_argument('basedir', type=str, default="-", help='base binaries directory')
    args = parser.parse_args()

    make_all_stable(args.basedir)
