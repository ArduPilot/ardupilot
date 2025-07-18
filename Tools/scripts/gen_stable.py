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

def make_stable(basedir, vehicle):
    '''make stable version for a vehicle'''
    stable_dir = os.path.join(basedir, vehicle, 'stable')
    if not os.path.exists(stable_dir):
        print("Missing %s" % stable_dir)
        return
    for b in sorted(os.listdir(stable_dir)):
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
        if os.path.exists(new_dir):
            continue
        if not os.path.exists(new_dir_parent):
            os.mkdir(new_dir_parent)
        print('Creating %s' % new_dir)
        old_dir = os.path.join(stable_dir, b)
        shutil.copytree(old_dir, new_dir)

def make_stable_from_beta(basedir, vehicle, beta_dir):
    '''make stable version from a beta with OFFICIAL tag'''
    beta_dir = os.path.join(basedir, vehicle, beta_dir)
    if not os.path.exists(beta_dir):
        return
    for b in sorted(os.listdir(beta_dir)):
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
        if os.path.exists(new_dir):
            continue
        if not os.path.exists(new_dir_parent):
            os.mkdir(new_dir_parent)
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
