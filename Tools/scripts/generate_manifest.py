#!/usr/bin/python

from __future__ import print_function

import sys
import json
import os
import re
import fnmatch
import gen_stable
import subprocess

FIRMWARE_TYPES = ["AntennaTracker", "Copter", "Plane", "Rover", "Sub", "AP_Periph"]
RELEASE_TYPES = ["beta", "latest", "stable", "stable-*", "dirty"]

# mapping for board names to brand name and manufacturer
brand_map = {
    'Pixhawk4' : ('Pixhawk 4', 'Holybro'),
    'Pix32v5' :  ('Pix32 v5', 'Holybro'),
    'Durandal' : ('Durandal', 'Holybro'),
    'PH4-mini' : ('Pixhawk 4 Mini', 'Holybro'),
    'KakuteF4' : ('KakuteF4', 'Holybro'),
    'KakuteF7' : ('KakuteF7', 'Holybro'),
    'KakuteF7Mini' : ('KakuteF7Mini', 'Holybro'),
    'CubeBlack' : ('CubeBlack', 'Hex/ProfiCNC'),
    'CubeYellow' : ('CubeYellow', 'Hex/ProfiCNC'),
    'CubeOrange' : ('CubeOrange', 'Hex/ProfiCNC'),
    'CubePurple' : ('CubePurple', 'Hex/ProfiCNC'),
    'CubeSolo' : ('CubeSolo', '3DR'),
    'CubeGreen-solo' : ('CubeGreen Solo', 'Hex/ProfiCNC'),
    'CUAVv5' : ('CUAVv5', 'CUAV'),
    'CUAVv5Nano' : ('CUAVv5 Nano', 'CUAV'),
    'CUAV-Nora' : ('CUAV Nora', 'CUAV'),
    'CUAV-X7' : ('CUAV X7', 'CUAV'),
    'DrotekP3Pro' : ('Pixhawk 3 Pro', 'Drotek'),
    'MatekF405' : ('Matek F405', 'Matek'),
    'MatekF405-STD' : ('Matek F405 STD', 'Matek'),
    'MatekF405-Wing' : ('Matek F405 Wing', 'Matek'),
    'mini-pix' : ('MiniPix', 'Radiolink'),
    'Pixhawk1' : ('Pixhawk1', 'mRobotics'),
    'Pixracer' : ('PixRacer', 'mRobotics'),
    'mRoX21' : ('mRo X2.1', 'mRobotics'),
    'mRoX21-777' : ('mRo X2.1-777', 'mRobotics'),
    'mRoNexus' : ('mRo Nexus', 'mRobotics'),
    'TBS-Colibri-F7' : ('Colibri F7', 'TBS'),
    'sparky2' : ('Sparky2', 'TauLabs'),
    'mindpx-v2' : ('MindPX V2', 'AirMind'),
    'OMNIBUSF7V2' : ('Omnibus F7 V2', 'Airbot'),
    'omnibusf4pro' : ('Omnibus F4 Pro', 'Airbot'),
    'omnibusf4v6' : ('Omnibus F4 V6', 'Airbot'),
    'OmnibusNanoV6' : ('Omnibus Nano V6', 'Airbot'),
    'speedybeef4' : ('SpeedyBee F4', 'SpeedyBee'),
}

class Firmware():
    def __init__(self,
                 date=None,
                 platform=None,
                 vehicletype=None,
                 filepath=None,
                 git_sha=None,
                 frame=None):
        self.atts = dict()
        self.atts["date"] = date
        self.atts["platform"] = platform
        self.atts["vehicletype"] = vehicletype
        self.atts["filepath"] = filepath
        self.atts["git_sha"] = git_sha
        self.atts["frame"] = frame
        self.atts["release-type"] = None
        self.atts["firmware-version"] = None

    def __getitem__(self, what):
        return self.atts[what]

    def __setitem__(self, name, value):
        self.atts[name] = value


class ManifestGenerator():
    '''Return a JSON string describing "binary" directory contents under
    basedir'''

    def __init__(self, basedir, baseurl):
        self.basedir = basedir
        self.baseurl = baseurl

    def frame_map(self, frame):
        '''translate from ArduPilot frame type terminology into mavlink
        terminology'''
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
            "Sub": "SUBMARINE",
            "AP_Periph": "CAN_PERIPHERAL",
        }
        if frame in frame_to_mavlink_dict:
            return frame_to_mavlink_dict[frame]

        return frame

    def releasetype_map(self, releasetype):
        '''translate from ArduPilot release type terminology into mavlink
        terminology'''
        if releasetype == 'stable':
            return 'OFFICIAL'
        return releasetype.upper()

    def looks_like_binaries_directory(self, dir):
        '''returns True if dir looks like it is a build_binaries.py output
        directory'''
        for entry in os.listdir(dir):
            if entry in FIRMWARE_TYPES:
                return True
        return False

    def git_sha_from_git_version(self, filepath):
        '''parses get-version.txt (as emitted by build_binaries.py, returns
        git sha from it'''
        content = open(filepath).read()
        sha_regex = re.compile("commit (?P<sha>[0-9a-f]+)")
        m = sha_regex.search(content)
        if m is None:
            raise Exception(
                "filepath (%s) does not contain a git sha" % (filepath,))
        return m.group("sha")

    def add_USB_IDs_PX4(self, firmware):
        '''add USB IDs to a .px4 firmware'''
        url = firmware['url']
        suffix = url.split('-')[-1]
        if suffix == "v1.px4":
            firmware['USBID'] = ['0x26AC/0x0010']
            firmware['board_id'] = 5
            firmware['bootloader_str'] = ['PX4 BL FMU v1.x']
        elif suffix in ["v2.px4", "v3.px4"]:
            firmware['USBID'] = ['0x26AC/0x0011']
            firmware['board_id'] = 9
            firmware['bootloader_str'] = ['PX4 BL FMU v2.x']
        elif suffix == "v4.px4":
            firmware['USBID'] = ['0x26AC/0x0012']
            firmware['board_id'] = 11
            firmware['bootloader_str'] = ['PX4 BL FMU v4.x']
        elif suffix == "v4pro.px4":
            firmware['USBID'] = ['0x26AC/0x0013']
            firmware['board_id'] = 13
            firmware['bootloader_str'] = ['PX4 BL FMU v4.x PRO']

    def add_USB_IDs_ChibiOS(self, firmware):
        '''add USB IDs to a ChbiOS apj firmware'''
        url = firmware['url'][len(self.baseurl)+1:]
        apj_path = os.path.join(self.basedir, url)
        if not os.path.exists(apj_path):
            print("bad apj path %s" % apj_path, file=sys.stderr)
            return
        apj_json = json.load(open(apj_path, 'r'))
        if 'board_id' not in apj_json:
            print("no board_id in %s" % apj_path, file=sys.stderr)
            return
        if 'platform' not in firmware:
            print("no platform for %s" % apj_path, file=sys.stderr)
            return
        board_id = apj_json['board_id']
        platform = firmware['platform']

        # all ChibiOS builds can have platform as bootloader_str and board_id from
        # hwdef.dat
        firmware['board_id'] = board_id
        firmware['bootloader_str'] = [platform+"-BL"]

        # map of vendor specific USB IDs
        USBID_MAP = {
            'CubeBlack': ['0x2DAE/0x1011'],
            'CubeOrange': ['0x2DAE/0x1016'],
            'CubePurple': ['0x2DAE/0x1005'],
            'CubeYellow': ['0x2DAE/0x1002'],
            'Pixhawk4': ['0x3162/0x0047'],
            'PH4-mini': ['0x3162/0x0049'],
            'Durandal': ['0x3162/0x004B'],
            'VRBrain-v51': ['0x27AC/0x1151'],
            'VRBrain-v52': ['0x27AC/0x1152'],
            'VRBrain-v54': ['0x27AC/0x1154'],
            'VRCore-v10': ['0x27AC/0x1910'],
            'VRUBrain-v51': ['0x27AC/0x1351']
        }
        if 'USBID' in apj_json:
            # newer APJ files have USBID in the json data
            firmware['USBID'] = [apj_json['USBID']]
        elif platform in USBID_MAP:
            firmware['USBID'] = USBID_MAP[platform]
        else:
            # all others use a single USB VID/PID
            firmware['USBID'] = ['0x0483/0x5740']

        if board_id == 50:
            # special case for FMUv5, they always get the px4 bootloader IDs as an option
            firmware['bootloader_str'].append('PX4 BL FMU v5.x')
            firmware['USBID'].append('0x26AC/0x0032')

        if board_id == 9:
            # special case for FMUv3, they always get the px4 bootloader IDs as an option
            firmware['bootloader_str'].append('PX4 BL FMU v2.x')
            firmware['USBID'].append('0x26AC/0x0011')

        if board_id == 11:
            # special case for FMUv4, they always get the px4 bootloader IDs as an option
            firmware['bootloader_str'].append('PX4 BL FMU v4.x')
            firmware['USBID'].append('0x26AC/0x0012')

        if board_id == 13:
            # special case for FMUv4pro, they always get the px4 bootloader IDs as an option
            firmware['bootloader_str'].append('PX4 BL FMU v4.x PRO')
            firmware['USBID'].append('0x26AC/0x0013')

        if board_id == 88:
            # special case for MindPX-v2 boards
            firmware['bootloader_str'].append('MindPX BL FMU v2.x')
            firmware['USBID'].append('0x26AC/0x0030')

        if platform in brand_map:
            (brand_name, manufacturer) = brand_map[platform]
            firmware['brand_name'] = brand_name
            firmware['manufacturer'] = manufacturer
        # copy over some extra information if available
        extra_tags = [ 'image_size' ]
        for tag in extra_tags:
            if tag in apj_json:
                firmware[tag] = apj_json[tag]

    def add_USB_IDs(self, firmware):
        '''add USB IDs to a firmware'''
        fmt = firmware['format']
        if fmt == "px4":
            self.add_USB_IDs_PX4(firmware)
            return
        if fmt == "apj":
            self.add_USB_IDs_ChibiOS(firmware)
            return

    def firmware_format_for_filepath(self, filepath):
        filename = os.path.basename(filepath)
        if "." in filename:
            return "".join(filename.split(".")[-1:])
        # no extension; ensure this is an elf:
        text = subprocess.check_output(["file", "-b", filepath])
        if re.match("^ELF", text):
            return "ELF"
        print("Unknown file type (%s)" % filepath)
        print("Got: %s" % text)
        return "Unknown" # should raise an error somehow

    def add_firmware_data_from_dir(self,
                                   dir,
                                   firmware_data,
                                   vehicletype,
                                   releasetype="dev"):
        '''accumulate additional information about firmwares from directory'''
        variant_firmware_regex = re.compile("[^-]+-(?P<variant>v\d+)[.px4]")
        if not os.path.isdir(dir):
            return
        try:
            dlist = os.listdir(dir)
        except Exception:
            print("Error listing '%s'" % dir)
            return
        for platformdir in dlist:
            if platformdir.startswith("."):
                continue
            some_dir = os.path.join(dir, platformdir)
            if not os.path.isdir(some_dir):
                continue
            git_version_txt = os.path.join(some_dir, "git-version.txt")
            if not os.path.exists(git_version_txt):
                print("No file %s" % git_version_txt, file=sys.stderr)
                continue
            try:
                git_sha = self.git_sha_from_git_version(git_version_txt)
            except Exception as ex:
                print("Failed to parse %s" % git_version_txt, ex, file=sys.stderr)
                continue

            # we require a firmware-version.txt. These files have been added to
            # old builds that didn't have them
            firmware_version_file = os.path.join(some_dir,
                                                 "firmware-version.txt")
            if not os.path.exists(firmware_version_file):
                print("Missing %s" % firmware_version_file, file=sys.stderr)
                continue

            try:
                firmware_version = open(firmware_version_file).read()
                firmware_version = firmware_version.strip()
                (_, _) = firmware_version.split("-")
            except ValueError:
                print("malformed firmware-version.txt at (%s)" % (firmware_version_file,), file=sys.stderr)
                continue
            except Exception as ex:
                print("bad file %s" % firmware_version_file, file=sys.stderr)
                # this exception is swallowed.... the current archive
                # is incomplete.
                continue

            # Directory names for heli builds end in -heli
            platform_frame_regex = re.compile("(?P<board>.+)(-(?P<frame>heli)$)")
            m = platform_frame_regex.match(platformdir)
            if m is not None:
                # This is a heli build
                platform = m.group("board")  # e.g. navio
                frame = "heli"
            else:
                # Non-heli build
                frame = vehicletype  # e.g. Plane
                platform = platformdir  # e.g. apm2

            for filename in os.listdir(some_dir):
                if filename in ["git-version.txt", "firmware-version.txt", "files.html"]:
                    continue
                if filename.startswith("."):
                    continue

                m = variant_firmware_regex.match(filename)
                if m:
                    # the platform variant is
                    # encoded in the firmware filename
                    # (e.g. the "v1" in
                    # ArduCopter-v1.px4)
                    variant = m.group("variant")
                    file_platform = "-".join([platform, variant])
                else:
                    file_platform = platform

                filepath = os.path.join(some_dir, filename)
                firmware_format = self.firmware_format_for_filepath(filepath)
                if firmware_format not in [ "ELF", "abin", "apj", "hex", "px4", "bin" ]:
                    print("Unknown firmware format (%s)" % firmware_format)

                firmware = Firmware()

                # translate from supplied "release type" into both a
                # "latest" flag and an actual release type.  Also sort
                # out which filepath we should use:
                firmware["latest"] = 0
                if releasetype == "dev":
                    if firmware["filepath"] is None:
                        firmware["filepath"] = filepath
                    if firmware["release-type"] is None:
                        firmware["release-type"] = "dev"
                elif releasetype == "latest":
                    firmware["latest"] = 1
                    firmware["filepath"] = filepath
                    if firmware["release-type"] is None:
                        firmware["release-type"] = "dev"
                else:
                    if (not firmware["latest"]):
                        firmware["filepath"] = filepath
                    firmware["release-type"] = releasetype

                firmware["platform"] = file_platform
                firmware["vehicletype"] = vehicletype
                firmware["git_sha"] = git_sha
                firmware["frame"] = frame
                firmware["timestamp"] = os.path.getctime(firmware["filepath"])
                firmware["format"] = firmware_format
                firmware["firmware-version"] = firmware_version

                firmware_data.append(firmware)

    def valid_release_type(self, tag):
        '''check for valid release type'''
        for r in RELEASE_TYPES:
            if fnmatch.fnmatch(tag, r):
                return True
        return False

    def parse_fw_version(self, version):
        (version_numbers, release_type) = version.split("-")
        (major, minor, patch) = version_numbers.split(".")
        return (major, minor, patch, version)

    def walk_directory(self, basedir):
        '''walks directory structure created by build_binaries, returns Python
        structure representing releases in that structure'''
        year_month_regex = re.compile("(?P<year>\d{4})-(?P<month>\d{2})")

        firmwares = []

        # used to listdir basedir here, but since this is also a web
        # document root, there's a lot of other stuff accumulated...
        vehicletypes = FIRMWARE_TYPES
        for vehicletype in vehicletypes:
            try:
                # the sort means we prefer 'stable' to 'stable-x.y.z' when they
                # both contain the same contents
                vdir = sorted(os.listdir(os.path.join(basedir, vehicletype)), reverse=True)
            except OSError as e:
                if e.errno == 2:
                    continue
            for firstlevel in vdir:
                if firstlevel == "files.html" or firstlevel.startswith("."):
                    # generated file which should be ignored
                    continue
                # skip any non-directories (e.g. "files.html"):
                if year_month_regex.match(firstlevel):
                    # this is a dated directory e.g. binaries/Copter/2016-02
                    # we do not include dated directories in the manifest ATM:
                    continue

                # assume this is a release directory such as
                # "beta", or the "latest" directory (treated as a
                # release and handled specially later)
                tag = firstlevel
                if not self.valid_release_type(tag):
                    print("Unknown tag (%s) in directory (%s)" %
                          (tag, os.path.join(vdir)), file=sys.stderr)
                    continue
                tag_path = os.path.join(basedir, vehicletype, tag)
                if not os.path.isdir(tag_path):
                    continue
                self.add_firmware_data_from_dir(tag_path,
                                                firmwares,
                                                vehicletype,
                                                releasetype=tag)

        # convert from ardupilot-naming conventions to common JSON format:
        firmware_json = []
        for firmware in firmwares:
            filepath = firmware["filepath"]
            # replace the base directory with the base URL
            urlifier = re.compile("^" + re.escape(basedir))
            url = re.sub(urlifier, self.baseurl, filepath)
            version_type = self.releasetype_map(firmware["release-type"])
            some_json = dict({
                "mav-autopilot": "ARDUPILOTMEGA",
                "vehicletype": firmware["vehicletype"],
                "platform": firmware["platform"],
                "git-sha": firmware["git_sha"],
                "url": url,
                "mav-type": self.frame_map(firmware["frame"]),
                "mav-firmware-version-type": version_type,
                "latest": firmware["latest"],
                "format": firmware["format"],
            })
            if firmware["firmware-version"]:
                try:
                    (major, minor, patch, release_type) = self.parse_fw_version(
                        firmware["firmware-version"])
                except Exception:
                    print("Badly formed firmware-version.txt %s" % firmware["firmware-version"], file=sys.stderr)
                    continue
                some_json["mav-firmware-version"] = ".".join([major,
                                                              minor,
                                                              patch])
                some_json["mav-firmware-version-major"] = major
                some_json["mav-firmware-version-minor"] = minor
                some_json["mav-firmware-version-patch"] = patch

            self.add_USB_IDs(some_json)

            #print(some_json['url'])
            firmware_json.append(some_json)

        ret = {
            "format-version": "1.0.0",  # semantic versioning
            "firmware": firmware_json
        }

        return ret

    def json(self):
        '''walk directory supplied in constructor, return json string'''
        if not self.looks_like_binaries_directory(self.basedir):
            print("Warning: this does not look like a binaries directory",
                  file=sys.stderr)

        structure = self.walk_directory(self.basedir)
        return json.dumps(structure, indent=4, separators=(',', ': '))


def usage():
    return '''Usage:
generate-manifest.py basedir'''


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='generate manifest.json')

    parser.add_argument('--outfile', type=str, default=None, help='output file, default stdout')
    parser.add_argument('--baseurl', type=str, default="https://firmware.ardupilot.org", help='base binaries directory')
    parser.add_argument('basedir', type=str, default="-", help='base binaries directory')

    args = parser.parse_args()

    # ensure all stable directories are created
    gen_stable.make_all_stable(args.basedir)

    generator = ManifestGenerator(args.basedir, args.baseurl)
    if args.outfile is None:
        print(generator.json())
    else:
        f = open(args.outfile, "w")
        f.write(generator.json())
        f.close()
