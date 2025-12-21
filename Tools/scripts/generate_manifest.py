#!/usr/bin/env python3

'''
AP_FLAKE8_CLEAN
'''

import fnmatch
import gen_stable
import gzip
import json
import os
import re
import subprocess
import shutil
import sys

if sys.version_info[0] < 3:
    running_python3 = False
    running_python310 = False
elif sys.version_info[1] < 10:
    running_python3 = True
    running_python310 = False
else:
    running_python3 = True
    running_python310 = True

FIRMWARE_TYPES = ["AntennaTracker", "Copter", "Plane", "Rover", "Sub", "AP_Periph", "Blimp"]
RELEASE_TYPES = ["beta", "latest", "stable", "stable-*", "dirty"]

# mapping for board names to brand name and manufacturer
brand_map = {
    'Pixhawk4' : ('Pixhawk 4', 'Holybro'),
    'Pixhawk4-bdshot' : ('Pixhawk 4', 'Holybro'),
    'Pix32v5' :  ('Pix32 v5', 'Holybro'),
    'Durandal' : ('Durandal', 'Holybro'),
    'Durandal-bdshot' : ('Durandal', 'Holybro'),
    'PH4-mini' : ('Pixhawk 4 Mini', 'Holybro'),
    'KakuteF4' : ('KakuteF4', 'Holybro'),
    'KakuteF7' : ('KakuteF7', 'Holybro'),
    'KakuteF7Mini' : ('KakuteF7Mini', 'Holybro'),
    'KakuteF4Mini' : ('KakuteF4Mini', 'Holybro'),
    'KakuteH7Mini' : ('KakuteH7Mini', 'Holybro'),
    'KakuteH7Mini-Nand' : ('KakuteH7Mini-Nand', 'Holybro'),
    'KakuteH7' : ('KakuteH7', 'Holybro'),
    'KakuteH7-bdshot' : ('KakuteH7', 'Holybro'),
    'KakuteH7v2' : ('KakuteH7v2', 'Holybro'),
    'CubeBlack' : ('CubeBlack', 'Hex/ProfiCNC'),
    'CubeYellow' : ('CubeYellow', 'Hex/ProfiCNC'),
    'CubeOrange' : ('CubeOrange', 'Hex/ProfiCNC'),
    'CubeOrange-bdshot' : ('CubeOrange', 'Hex/ProfiCNC'),
    'CubePurple' : ('CubePurple', 'Hex/ProfiCNC'),
    'CubeSolo' : ('CubeSolo', '3DR'),
    'CubeGreen-solo' : ('CubeGreen Solo', 'Hex/ProfiCNC'),
    'CUAVv5' : ('CUAVv5', 'CUAV'),
    'CUAVv5-bdshot' : ('CUAVv5', 'CUAV'),
    'CUAVv5Nano' : ('CUAVv5 Nano', 'CUAV'),
    'CUAVv5Nano-bdshot' : ('CUAVv5 Nano', 'CUAV'),
    'CUAV-Nora' : ('CUAV Nora', 'CUAV'),
    'CUAV-Nora-bdshot' : ('CUAV Nora', 'CUAV'),
    'CUAV-X7' : ('CUAV X7', 'CUAV'),
    'CUAV-X7-bdshot' : ('CUAV X7', 'CUAV'),
    'DrotekP3Pro' : ('Pixhawk 3 Pro', 'Drotek'),
    'MambaF405v2' : ('Diatone Mamba F405 MK2', 'Diatone'),
    'MatekF405' : ('Matek F405', 'Matek'),
    'MatekF405-bdshot' : ('Matek F405', 'Matek'),
    'MatekF405-STD' : ('Matek F405 STD', 'Matek'),
    'MatekF405-Wing' : ('Matek F405 Wing', 'Matek'),
    'MatekF765-SE' : ('Matek F765 SE', 'Matek'),
    'MatekH743' : ('Matek H743', 'Matek'),
    'MatekH743-bdshot' : ('Matek H743', 'Matek'),
    'mini-pix' : ('MiniPix', 'Radiolink'),
    'Pixhawk1' : ('Pixhawk1', 'mRobotics'),
    'Pixracer' : ('PixRacer', 'mRobotics'),
    'Pixracer-bdshot' : ('PixRacer', 'mRobotics'),
    'mRoX21' : ('mRo X2.1', 'mRobotics'),
    'mRoX21-777' : ('mRo X2.1-777', 'mRobotics'),
    'mRoPixracerPro' : ('mRo PixracerPro', 'mRobotics'),
    'mRoPixracerPro-bdshot' : ('mRo PixracerPro', 'mRobotics'),
    'mRoControlZeroOEMH7' : ('mRo ControlZero OEM H7', 'mRobotics'),
    'mRoNexus' : ('mRo Nexus', 'mRobotics'),
    'TBS-Colibri-F7' : ('Colibri F7', 'TBS'),
    'sparky2' : ('Sparky2', 'TauLabs'),
    'mindpx-v2' : ('MindPX V2', 'AirMind'),
    'OMNIBUSF7V2' : ('Omnibus F7 V2', 'Airbot'),
    'omnibusf4pro' : ('Omnibus F4 Pro', 'Airbot'),
    'omnibusf4pro-bdshot' : ('Omnibus F4 Pro', 'Airbot'),
    'omnibusf4v6' : ('Omnibus F4 V6', 'Airbot'),
    'OmnibusNanoV6' : ('Omnibus Nano V6', 'Airbot'),
    'OmnibusNanoV6-bdshot' : ('Omnibus Nano V6', 'Airbot'),
    'speedybeef4' : ('SpeedyBee F4', 'SpeedyBee'),
    'speedybeef4v3' : ('SpeedyBee F4 v3', 'SpeedyBee'),
    'QioTekZealotF427' : ('ZealotF427', 'QioTek'),
    'BeastH7' : ('Beast H7 55A AIO', 'iFlight'),
    'BeastH7v2' : ('Beast H7 v2 55A AIO', 'iFlight'),
    'BeastF7' : ('Beast F7 45A AIO', 'iFlight'),
    'BeastF7v2' : ('Beast F7 v2 55A AIO', 'iFlight'),
    'MambaF405US-I2C' : ('Diatone Mamba Basic F405 MK3/MK3.5', 'Diatone'),
    'MambaF405-2022' : ('Diatone Mamba Basic F405 MK4', 'Diatone'),
    'MambaH743v4' : ('Diatone Mamba H743 MK4', 'Diatone'),
    "FlywooF745" : ('Flywoo Goku GN 745 AIO', 'Flywoo'),
    "FlywooF745Nano" : ('Flywoo Goku Hex F745', 'Flywoo'),
    "modalai_fc-v1" : ('ModalAI FlightCore v1', 'ModalAI'),
    'Pixhawk5X' : ('Pixhawk 5X', 'Holybro'),
    "AIRLink" : ("Sky-Drones Technologies", "AIRLink"),
    "SPRacingH7" : ("Seriously Pro Racing", "H7 Extreme"),
    "SkystarsH7HD" : ("Skystars", "H743 HD"),
    "SkystarsH7HD-bdshot" : ("Skystars", "H743 HD"),
    "MicoAir405v2" : ("MicoAir F405 v2.1", "MicoAir"),
    "MicoAir405Mini" : ("MicoAir F405 Mini", "MicoAir"),
    "MicoAir743" : ("MicoAir H743 v1.3", "MicoAir"),
    "MicoAir743-AIO" : ("MicoAir H743 AIO", "MicoAir"),
    "MicoAir743v2" : ("MicoAir H743 v2.0", "MicoAir"),
    "MicoAir743-Lite" : ("MicoAir H743 Lite v1.1", "MicoAir"),
    "GEPRCF745BTHD": ("TAKER F745 BT", "GEPRC"),
    "GEPRC_TAKER_H743": ("TAKER H743 BT", "GEPRC"),
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
        self.atts["firmware-version-str"] = ""
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
            "deca": "DECAROTOR",
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

    def fwversion_from_git_version(self, filepath):
        '''parses get-version.txt (as emitted by build_binaries.py, returns
        git sha from it'''
        content = open(filepath).read()
        sha_regex = re.compile(r"APMVERSION: \S+\s+(\S+)")
        m = sha_regex.search(content)
        if m is None:
            raise Exception(
                "filepath (%s) does not contain an APMVERSION" % (filepath,))
        return m.group(1)

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
            'CubeOrange': ['0x2DAE/0x1016', '0x2DAE/0x1017'],
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

        if board_id == 53:
            # special case for 6X, they always get the px4 bootloader IDs as an option
            firmware['bootloader_str'].append('PX4 BL FMU v6X.x')
            firmware['USBID'].append('0x3185/0x0035')

        if board_id == 56:
            # special case for 6C, they always get the px4 bootloader IDs as an option
            firmware['bootloader_str'].append('PX4 BL FMU v6C.x')
            firmware['USBID'].append('0x3185/0x0038')

        if platform in brand_map:
            (brand_name, manufacturer) = brand_map[platform]
            firmware['brand_name'] = brand_name
            firmware['manufacturer'] = manufacturer

        # copy over some extra information if available
        extra_tags = ['image_size', 'brand_name', 'manufacturer']
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
        if running_python3:
            text = text.decode('ascii')

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
        variant_firmware_regex = re.compile(r"[^-]+-(?P<variant>v\d+)[.px4]")
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
            try:
                fwversion_str = self.fwversion_from_git_version(git_version_txt)
            except Exception as ex:
                print("Failed to parse APMVERSION %s" % git_version_txt, ex, file=sys.stderr)
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
            except Exception:
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

            # also gather information from any features.txt files present:
            features_text = None
            features_filepath = os.path.join(some_dir, "features.txt")
            if os.path.exists(features_filepath):
                features_text = sorted(open(features_filepath).read().rstrip().split("\n"))

            for filename in os.listdir(some_dir):
                if filename in ["git-version.txt", "firmware-version.txt", "files.html", "features.txt"]:
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
                if firmware_format not in ["elf", "ELF", "abin", "apj", "hex", "px4", "bin"]:
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
                firmware["firmware-version-str"] = fwversion_str
                firmware["frame"] = frame
                firmware["timestamp"] = os.path.getctime(firmware["filepath"])
                firmware["format"] = firmware_format
                firmware["firmware-version"] = firmware_version

                firmware["features"] = features_text

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
        year_month_regex = re.compile(r"(?P<year>\d{4})-(?P<month>\d{2})")

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
                          (tag, os.path.join(*vdir)), file=sys.stderr)
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
        features_json = []  # a structure containing summarised features per firmware

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
                "mav-firmware-version-str": firmware["firmware-version-str"],
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

            firmware_json.append(some_json)

            # now the features the firmware supports...
            try:
                features = firmware["features"]
                # check apj here in case we're creating bin and apj etc:
                if (firmware["format"] == "apj" and
                        features is not None and
                        bool(firmware["latest"])):
                    x = dict({
                        "vehicletype": firmware["vehicletype"],
                        "platform": firmware["platform"],
                        "git-sha": firmware["git_sha"],
                        "latest": firmware["latest"],
                    })
                    x["features"] = features
                    features_json.append(x)

            except KeyError:
                pass

        ret = {
            "format-version": "1.0.0",  # semantic versioning
            "firmware": firmware_json
        }

        features_ret = {
            "format-version": "1.0.0",  # semantic versioning
            "features": features_json
        }

        return ret, features_ret

    def run(self):
        '''walk directory supplied in constructor, record results in self'''
        if not self.looks_like_binaries_directory(self.basedir):
            print("Warning: this does not look like a binaries directory",
                  file=sys.stderr)

        self.structure, self.features_structure = self.walk_directory(self.basedir)

    def json(self):
        '''returns JSON string for version information for all firmwares'''
        if getattr(self, 'structure', None) is None:
            self.run()
        return json.dumps(self.structure, indent=4, separators=(',', ': '))

    def json_features(self):
        '''returns JSON string for supported features for all firmwares.
        run() method must have been called already'''
        return json.dumps(self.features_structure, indent=4, separators=(',', ': '))

    def write_string_to_filepath(self, string, filepath):
        '''writes the entirety of string to filepath'''
        with open(filepath, "w") as x:
            x.write(string)

    def write_json(self, content, path):
        '''write content to path, also creating a compress .gz version'''
        new_json_filepath = path + ".new"
        self.write_string_to_filepath(content, new_json_filepath)
        # provide a pre-compressed version.  For reference, a 7M manifest
        # "gzip -9"s to 300k in 1 second, "xz -e"s to 80k in 26 seconds
        new_json_filepath_gz = path + ".gz.new"
        with gzip.open(new_json_filepath_gz, 'wb') as gf:
            if running_python3:
                content = bytes(content, 'ascii')
            gf.write(content)
            gf.close()
        shutil.move(new_json_filepath, path)
        shutil.move(new_json_filepath_gz, path + ".gz")

    def write_manifest_json(self, path):
        '''write generated JSON content to path'''
        self.write_json(self.json(), path)

    def write_features_json(self, path):
        '''write generated features JSON content to path'''
        self.write_json(self.json_features(), path)


def usage():
    return '''Usage:
generate-manifest.py basedir'''


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='generate manifest.json')

    parser.add_argument('--outfile', type=str, default=None, help='output file, default stdout')
    parser.add_argument('--outfile-features-json', type=str, default=None, help='output file for features json file')
    parser.add_argument('--baseurl', type=str, default="https://firmware.ardupilot.org", help='base binaries directory')
    parser.add_argument('basedir', type=str, default="-", help='base binaries directory')

    args = parser.parse_args()

    # ensure all stable directories are created
    gen_stable.make_all_stable(args.basedir)

    generator = ManifestGenerator(args.basedir, args.baseurl)
    generator.run()

    content = generator.json()
    if args.outfile is None:
        print(content)
    else:
        generator.write_manifest_json(args.outfile)

    if args.outfile_features_json is not None:
        generator.write_features_json(args.outfile_features_json)
