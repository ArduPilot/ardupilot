#!/usr/bin/env python3

'''
create stable-x.y.z directories so we keep all past stable releases for users to download

AP_FLAKE8_CLEAN
'''

import os
import re
import shutil
import tempfile

VEHICLES = ['AntennaTracker', 'Copter', 'Plane', 'Rover', 'Sub']

# beta directories that may contain stable builds
BETA_DIRS = []


def metadata_release_version(old_dir):
    '''Return the release version identified by tag-level metadata.'''
    metadata_dir = os.path.join(old_dir, '__METADATA__')
    tag_path = os.path.join(metadata_dir, 'release-tag.txt')
    if os.path.isfile(tag_path):
        with open(tag_path, encoding='utf-8') as tag_file:
            tag = tag_file.read().strip()
        tag_match = re.match(r'[^-]+-(\d+\.\d+\.\d+)', tag)
        if tag_match is not None:
            return tag_match.group(1)

    pdef_path = os.path.join(metadata_dir, 'apm.pdef.xml')
    if not os.path.isfile(pdef_path):
        return None
    with open(pdef_path, encoding='utf-8') as pdef_file:
        contents = pdef_file.read()
    match = re.search(r'git_tag="[^"-]+-(\d+\.\d+\.\d+)"', contents)
    if match is None:
        match = re.search(r'Generated from git tag [^-]+-(\d+\.\d+\.\d+)', contents)
    return match.group(1) if match is not None else None


def copy_metadata(old_dir, new_dir_parent, version):
    '''Copy metadata only to the matching per-board release directory.'''
    if metadata_release_version(old_dir) != version:
        return
    metadata_dir = os.path.join(old_dir, '__METADATA__')
    new_metadata_dir = os.path.join(new_dir_parent, '__METADATA__')
    if not os.path.isdir(metadata_dir):
        return
    replace_directory(metadata_dir, new_metadata_dir)


def backup_directory(destination_dir):
    '''Return the hidden backup path for a directory replacement.'''
    destination_parent = os.path.dirname(destination_dir)
    return os.path.join(destination_parent,
                        f'.{os.path.basename(destination_dir)}.backup')


def recover_directory(destination_dir):
    '''Restore a directory left in the backup position by an interrupted replace.'''
    backup_dir = backup_directory(destination_dir)
    if os.path.exists(backup_dir):
        if os.path.exists(destination_dir):
            shutil.rmtree(backup_dir)
        else:
            os.replace(backup_dir, destination_dir)


def replace_directory(source_dir, destination_dir):
    '''Copy a directory into place and recover interrupted replacements.'''
    destination_parent = os.path.dirname(destination_dir)
    staging_parent = tempfile.mkdtemp(prefix=f'.copy-{os.path.basename(destination_dir)}-',
                                      dir=destination_parent)
    staging_dir = os.path.join(staging_parent, os.path.basename(destination_dir))
    backup_dir = backup_directory(destination_dir)
    try:
        recover_directory(destination_dir)
        shutil.copytree(source_dir, staging_dir)
        if os.path.exists(destination_dir):
            os.replace(destination_dir, backup_dir)
        os.replace(staging_dir, destination_dir)
    except OSError:
        recover_directory(destination_dir)
        raise
    else:
        if os.path.exists(backup_dir):
            shutil.rmtree(backup_dir)
    finally:
        shutil.rmtree(staging_parent, ignore_errors=True)


def make_stable(basedir, vehicle):
    '''make stable version for a vehicle'''
    stable_dir = os.path.join(basedir, vehicle, 'stable')
    if not os.path.exists(stable_dir):
        print("Missing %s" % stable_dir)
        return
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
        copy_metadata(stable_dir, new_dir_parent, version)
        old_dir = os.path.join(stable_dir, b)
        if os.path.exists(new_dir):
            print('Updating %s' % new_dir)
            replace_directory(old_dir, new_dir)
        else:
            print('Creating %s' % new_dir)
            replace_directory(old_dir, new_dir)


def make_stable_from_beta(basedir, vehicle, beta_dir):
    '''make stable version from a beta with OFFICIAL tag'''
    beta_dir = os.path.join(basedir, vehicle, beta_dir)
    if not os.path.exists(beta_dir):
        return
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
        # print(vfile, b, version, vtype)
        if vtype != 'FIRMWARE_VERSION_TYPE_OFFICIAL':
            # not a new stable
            continue
        new_dir_parent = os.path.join(basedir, vehicle, 'stable-%s' % version)
        new_dir = os.path.join(new_dir_parent, b)
        os.makedirs(new_dir_parent, exist_ok=True)
        copy_metadata(beta_dir, new_dir_parent, version)
        old_dir = os.path.join(beta_dir, b)
        if os.path.exists(new_dir):
            print('Updating %s' % new_dir)
            replace_directory(old_dir, new_dir)
        else:
            print('Creating %s' % new_dir)
            replace_directory(old_dir, new_dir)


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
