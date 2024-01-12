#!/usr/bin/env python
'''
check firmware-version.txt in binaries directory
'''

import os

VEHICLES = ['AntennaTracker', 'Copter', 'Plane', 'Rover', 'Sub']

def parse_git_version(gfile):
    '''parse git-version.txt, producing a firmware-version.txt'''
    gv = open(gfile).readlines()
    vline = gv[-1]
    if not vline.startswith("APMVERSION:"):
        print("Bad version %s in %s" % (vline, gfile))
        return None
    vline = vline[11:]
    a = vline.split('V')
    if len(a) != 2:
        return None
    vers = a[1].strip()
    if vers[-1].isdigit():
        return vers+"-FIRMWARE_VERSION_TYPE_OFFICIAL"
    print("Bad vers %s in %s" % (vers, gfile))
    return None

def check_fw_version(version):
    try:
        (version_numbers, release_type) = version.split("-")
        (_, _, _) = version_numbers.split(".")
    except Exception:
        return False
    return True

def check_version(vehicle):
    '''check firmware-version.txt version for a vehicle'''
    for d in os.listdir(vehicle):
        if not d.startswith("stable"):
            continue
        stable_dir = '%s/%s' % (vehicle, d)
        for b in sorted(os.listdir(stable_dir)):
            if not os.path.isdir(os.path.join(stable_dir, b)):
                continue
            vfile = os.path.join(stable_dir, b, "firmware-version.txt")
            if os.path.exists(vfile):
                v = open(vfile).read()
                if check_fw_version(v):
                    continue
            gfile = os.path.join(stable_dir, b, "git-version.txt")
            if not os.path.exists(gfile):
                print("Missing %s" % gfile)
                continue
            v = parse_git_version(gfile)
            if v is not None:
                open(vfile, "w").write(v)
                print("Added %s" % vfile)
                continue
            print("Failed for %s" % gfile)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='check_firmware_version.py')

    args = parser.parse_args()

    for v in VEHICLES:
        check_version(v)
