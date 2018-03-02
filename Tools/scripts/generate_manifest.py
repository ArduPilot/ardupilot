#!/usr/bin/python

from __future__ import print_function

import sys
import json
import os
import re

class Firmware():
    def __init__(self, date=None, platform=None, vehicletype=None, filepath=None, git_sha=None, frame=None):
        self.atts = dict()
        self.atts["date"] = date
        self.atts["platform"] = platform
        self.atts["vehicletype"] = vehicletype
        self.atts["filepath"] = filepath
        self.atts["git_sha"] = git_sha
        self.atts["frame"] = frame
        self.atts["release-type"] = None
        self.atts["firmware-version"] = None

    def __getitem__(self,what):
        return self.atts[what]

    def __setitem__(self,name,value):
        self.atts[name] = value

class ManifestGenerator():
    '''Return a JSON string describing "binary" directory contents under basedir'''

    def __init__(self,basedir,baseurl):
        self.basedir = basedir
        self.baseurl = baseurl

    def frame_map(self, frame):
        '''translate from ArduPilot frame type terminology into mavlink terminology'''
        frame_to_mavlink_dict = {
            "quad": "QUADROTOR",
            "hexa": "HEXAROTOR",
            "y6": "ARDUPILOT_Y6",
            "tri": "TRICOPTER",
            "octa": "OCTOROTOR",
            "octa-quad": "ARDUPILOT_OCTAQUAD",
            "heli": "HELICOPTER",
            "Plane": "FIXED_WING",
            "AntennaTracker": "ANTENNA_TRACKER",
            "Rover": "GROUND_ROVER",
            "Sub": "SUBMARINE"
        }
        if frame in frame_to_mavlink_dict:
            return frame_to_mavlink_dict[frame]

        return frame

    def releasetype_map(self, releasetype):
        '''translate from ArduPilot release type terminology into mavlink terminology'''
        if releasetype == 'stable': return 'OFFICIAL'
        return releasetype.upper()

    def looks_like_binaries_directory(self, dir):
        '''returns True if dir looks like it is a build_binaries.py output directory'''
        for entry in os.listdir(dir):
            if entry in {"AntennaTracker", "Copter", "Plane", "Rover", "Sub"}:
                return True
        return False

    def git_sha_from_git_version(self, filepath):
        '''parses get-version.txt (as emitted by build_binaries.py, returns git sha from it'''
        content = open(filepath).read()
        sha_regex = re.compile("commit (?P<sha>[0-9a-f]+)")
        m = sha_regex.search(content)
        if m is None:
            raise Exception("filepath (%s) does not appear to contain a git sha" % (filepath,))
        return m.group("sha")

    def add_firmware_data_from_dir(self, dir, firmware_data, vehicletype, releasetype="dev"):
        '''accumulate additional information about firmwares from a directory'''
        platform_frame_regex = re.compile("(?P<board>PX4|navio|pxf)(-(?P<frame>.+))?")
        variant_firmware_regex = re.compile("[^-]+-(?P<variant>v\d+)[.px4]")
        for platformdir in os.listdir(dir):
            some_dir = os.path.join(dir, platformdir)
            try:
                git_sha = self.git_sha_from_git_version(os.path.join(some_dir, "git-version.txt"))
            except Exception as e:
                continue
            firmware_version_file = os.path.join(some_dir, "firmware-version.txt")
            try:
                firmware_version = open(firmware_version_file).read()
                firmware_version = firmware_version.strip()
                (version_numbers,release_type) = firmware_version.split("-")
            except ValueError as e:
                # print("malformed firmware-version.txt at (%s)" % (firmware_version_file,), file=sys.stderr)
                firmware_version = None
            except Exception as e:
                # this exception is swallowed.... the current archive
                # is incomplete.
                firmware_version = None

            m = platform_frame_regex.match(platformdir)
            if m is not None:
                # the model type (quad/tri) is
                # encoded in the platform name
                # (e.g. navio-octa)
                platform = m.group("board") # e.g. navio
                frame = m.group("frame") # e.g. octa
                if frame is None:
                    frame = vehicletype
            else:
                frame = vehicletype # e.g. Plane
                platform = platformdir # e.g. apm2

            for file in os.listdir(some_dir):
                if file == "git-version.txt":
                    continue
                if file == "firmware-version.txt":
                    continue
                if file == "files.html":
                    continue

                m = variant_firmware_regex.match(file)
                if m:
                    # the platform variant is
                    # encoded in the firmware filename
                    # (e.g. the "v1" in
                    # ArduCopter-v1.px4)
                    variant = m.group("variant")
                    file_platform = "-".join([platform,variant])
                else:
                    file_platform = platform

                firmware_format = "".join(file.split(".")[-1:])

                if not vehicletype in firmware_data:
                    firmware_data[vehicletype] = dict()
                if not file_platform in firmware_data[vehicletype]:
                    firmware_data[vehicletype][file_platform] = dict()
                if not git_sha in firmware_data[vehicletype][file_platform]:
                    firmware_data[vehicletype][file_platform][git_sha] = dict()

                if not firmware_format in firmware_data[vehicletype][file_platform][git_sha]:
                    firmware_data[vehicletype][file_platform][git_sha][firmware_format] = dict()
                if not releasetype in firmware_data[vehicletype][file_platform][git_sha][firmware_format]:
                    firmware_data[vehicletype][file_platform][git_sha][firmware_format][releasetype] = dict()
                if not frame in firmware_data[vehicletype][file_platform][git_sha][firmware_format][releasetype]:
                    firmware_data[vehicletype][file_platform][git_sha][firmware_format][releasetype][frame] = Firmware()

                firmware = firmware_data[vehicletype][file_platform][git_sha][firmware_format][releasetype][frame]

                # translate from supplied "release type" into both a
                # "latest" flag andan actual release type.  Also sort
                # out which filepath we should use:
                firmware["latest" ] = 0
                if releasetype == "dev":
                    if firmware["filepath"] is None:
                        firmware["filepath"] = os.path.join(some_dir, file)
                    if firmware["release-type"] is None:
                        firmware["release-type"] = "dev"
                elif releasetype == "latest":
                    firmware["latest"] = 1
                    firmware["filepath"] = os.path.join(some_dir, file)
                    if firmware["release-type"] is None:
                        firmware["release-type"] = "dev"
                else:
                    if (not firmware["latest"]):
                        firmware["filepath"] = os.path.join(some_dir, file)
                    firmware["release-type"] = releasetype

                firmware["platform"] = file_platform
                firmware["vehicletype"] = vehicletype
                firmware["git_sha"] = git_sha
                firmware["frame"] = frame
                firmware["timestamp"] = os.path.getctime(firmware["filepath"])
                firmware["format"] = firmware_format
                firmware["firmware-version"] = firmware_version

    def xfirmwares_to_firmwares(self, xfirmwares):
        '''takes hash structure of firmwares, returns list of them'''
        if isinstance(xfirmwares, dict):
            ret = []
            for value in xfirmwares.values():
                o  = self.xfirmwares_to_firmwares(value)
                for oo in o:
                    ret.append(oo)
            return ret
        else:
            return [xfirmwares]

    known_release_types = {
        "beta" : 1,
        "latest" : 1,
        "stable" : 1
    }

    def parse_fw_version(self, version):
        (version_numbers,release_type) = version.split("-")
        (major,minor,patch) = version_numbers.split(".")
        return (major,minor,patch,version)

    def walk_directory(self, basedir):
        '''walks directory structure created by build_binaries, returns Python structure representing releases in that structure'''
        year_month_regex = re.compile("(?P<year>\d{4})-(?P<month>\d{2})")

        xfirmwares = dict()

        # used to listdir basedir here, but since this is also a web document root, there's a lot of other stuff accumulated...
        vehicletypes = [ 'AntennaTracker', 'Copter', 'Plane', 'Rover', 'Sub' ]
        for vehicletype in vehicletypes:
            try:
                vdir = os.listdir(os.path.join(basedir, vehicletype))
            except OSError as e:
                if e.errno == 2:
                    continue
            for firstlevel in vdir:
                if firstlevel == "files.html":
                    # generated file which should be ignored
                    continue
                # skip any non-directories (e.g. "files.html"):
                if year_month_regex.match(firstlevel):
                    # this is a dated directory e.g. binaries/Copter/2016-02
                    year_month_path = os.path.join(basedir, vehicletype, firstlevel)
                    for fulldate in os.listdir(year_month_path):
                        if fulldate == "files.html":
                            # generated file which should be ignored
                            continue
                        self.add_firmware_data_from_dir(os.path.join(year_month_path, fulldate), xfirmwares, vehicletype)
                else:
                    # assume this is a release directory such as
                    # "beta", or the "latest" directory (treated as a
                    # release and handled specially later)
                    tag = firstlevel
                    if tag not in self.known_release_types:
                        print("Unknown tag (%s) in directory (%s)" %
                              (tag, vdir), file=sys.stderr)
                    tag_path = os.path.join(basedir, vehicletype, tag)
                    self.add_firmware_data_from_dir(tag_path, xfirmwares, vehicletype, releasetype=tag)

        firmwares = self.xfirmwares_to_firmwares(xfirmwares)

        # convert from ardupilot-naming conventions to common JSON format:
        firmware_json = []
        for firmware in firmwares:
            filepath = firmware["filepath"]
            # replace the base directory with the base URL
            urlifier = re.compile("^" + re.escape(basedir))
            url = re.sub(urlifier, self.baseurl, filepath)
            some_json = dict({
                "mav-autopilot": "ARDUPILOTMEGA",
#                "vehicletype": firmware["vehicletype"],
                "platform": firmware["platform"],
                "git-sha": firmware["git_sha"],
                "url": url,
                "mav-type": self.frame_map(firmware["frame"]),
                "mav-firmware-version-type": self.releasetype_map(firmware["release-type"]),
                "latest": firmware["latest"],
                "format": firmware["format"],
            })
            if firmware["firmware-version"]:
                (major,minor,patch,release_type) = self.parse_fw_version(firmware["firmware-version"])
                some_json["mav-firmware-version"] = ".".join([major,minor,patch])
                some_json["mav-firmware-version-major"] = major
                some_json["mav-firmware-version-minor"] = minor
                some_json["mav-firmware-version-patch"] = patch

            firmware_json.append(some_json)

        ret = {
            "format-version": "1.0.0", # semantic versioning
            "firmware": firmware_json
        }

        return ret

    def json(self):
        '''walk directory supplied in constructor, return json string'''
        if not self.looks_like_binaries_directory(self.basedir):
            print("Warning: this does not look like a binaries directory", file=sys.stderr)

        structure = self.walk_directory(self.basedir)
        return json.dumps(structure, indent=4)

def usage():
    return '''Usage:
generate-manifest.py basedir baseurl'''

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(usage())
        sys.exit(1)

    generator = ManifestGenerator(sys.argv[1], sys.argv[2])
    print(generator.json())
