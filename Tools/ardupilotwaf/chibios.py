# encoding: utf-8

# flake8: noqa

"""
Waf tool for ChibiOS build
"""

from waflib import Errors, Logs, Task, Utils, Context
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys
import re
import pickle
import struct
import base64
import subprocess
import traceback

import hal_common

# sys.path already set up at the top of boards.py
import chibios_hwdef

_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/ChibiOS')
    include_dirs_node = bldnode.find_node('include_dirs')
    if include_dirs_node is None:
        _dynamic_env_data['include_dirs'] = []
        return
    tmp_str = include_dirs_node.read()
    tmp_str = tmp_str.replace(';\n','')
    tmp_str = tmp_str.replace('-I','')  #remove existing -I flags
    # split, coping with separator
    idirs = re.split('; ', tmp_str)

    # create unique list, coping with relative paths
    idirs2 = []
    for d in idirs:
        if d.startswith('../'):
            # relative paths from the make build are relative to BUILDROOT
            d = os.path.join(bld.env.BUILDROOT, d)
        d = os.path.normpath(d)
        if d not in idirs2:
            idirs2.append(d)
    _dynamic_env_data['include_dirs'] = idirs2

@feature('ch_ap_library', 'ch_ap_program')
@before_method('process_source')
def ch_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)
    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])


class upload_fw(Task.Task):
    color='BLUE'
    always_run = True
    def run(self):
        import platform
        upload_tools = self.env.get_flat('UPLOAD_TOOLS')
        upload_port = self.generator.bld.options.upload_port
        src = self.inputs[0]
        # Refer Tools/scripts/macos_remote_upload.sh for details
        if 'AP_OVERRIDE_UPLOAD_CMD' in os.environ:
            cmd = "{} '{}'".format(os.environ['AP_OVERRIDE_UPLOAD_CMD'], src.abspath())
        elif "microsoft-standard-WSL2" in platform.release():
            if not self.wsl2_prereq_checks():
                return
            print("If this takes takes too long here, try power-cycling your hardware\n")
            cmd = "{} -u '{}/uploader.py' '{}'".format('python.exe', upload_tools, src.abspath())
        else:
            cmd = "{} '{}/uploader.py' '{}'".format(self.env.get_flat('PYTHON'), upload_tools, src.abspath())
        if upload_port is not None:
            cmd += " '--port' '%s'" % upload_port
        if self.generator.bld.options.upload_force:
            cmd += " '--force'"
        return self.exec_command(cmd)

    def wsl2_prereq_checks(self):
        # As of July 2022 WSL2 does not support native USB support. The workaround from Microsoft
        #       using 'usbipd' does not work due to the following workflow:
        #
        # 1) connect USB device to Windows computer running WSL2
        # 2) device boots into app
        # 3) use 'usbipd' from Windows Cmd/PowerShell to determine busid, this is very hard to automate on Windows
        # 4) use 'usbipd' from Windows Cmd/PowerShell to attach, this is very hard to automate on Windows
        # -- device is now viewable via 'lsusb' but you need sudo to read from it.
        # either run 'chmod666 /dev/ttyACM*' or use udev to automate chmod on device connect
        # 5) uploader.py detects device, sends reboot command which disconnects the USB port and reboots into
        #       bootloader (different USB device)
        # 6) manually repeat steps 3 & 4
        # 7) doing steps 3 and 4 will most likely take several seconds and in many cases the bootloader has
        #        moved on into the app
        #
        # Solution: simply call "python.exe" instead of 'python' which magically calls it from the windows
        #   system using the same absolute path back into the WSL2's user's directory
        # Requirements: Windows must have Python 3.12.x installed and a few packages.
        import subprocess
        try:
            where_python = subprocess.check_output('where.exe python.exe', shell=True, text=True)
        except subprocess.CalledProcessError:
            #if where.exe can't find the file it returns a non-zero result which throws this exception
            where_python = ""
        if "python.exe" not in where_python:
            print(self.get_full_wsl2_error_msg("Windows python.exe not found"))
            return False
        return True

    def get_full_wsl2_error_msg(self, error_msg):
        return ("""
        ****************************************
        ****************************************
        WSL2 firmware uploads use the host's Windows Python.exe so it has access to the COM ports.

        %s
        Please download Windows Installer 3.12.x from https://www.python.org/downloads/
        and make sure to add it to your path during the installation. Once installed, run this
        command in Powershell or Command Prompt to install some packages:
        
        python.exe -m pip install empy==3.3.4 pyserial
        ****************************************
        ****************************************
        """ % error_msg)

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)

    def keyword(self):
        return "Uploading"

class set_default_parameters(Task.Task):
    '''copy the ELF and embed the default parameters file in the copy'''
    color='CYAN'
    vars = ['DEFAULT_PARAMETERS']
    def keyword(self):
        return "apj_tool"
    def run(self):
        rel_default_parameters = self.env.get_flat('DEFAULT_PARAMETERS').replace("'", "")
        abs_default_parameters = os.path.join(self.env.SRCROOT, rel_default_parameters)
        apj_tool = self.env.APJ_TOOL
        sys.path.append(os.path.dirname(apj_tool))
        from apj_tool import embedded_defaults
        shutil.copyfile(self.inputs[0].abspath(), self.outputs[0].abspath())
        defaults = embedded_defaults(self.outputs[0].abspath())
        if defaults.find():
            defaults.set_file(abs_default_parameters)
            defaults.save()


class generate_bin(Task.Task):
    color='CYAN'
    # run_str="${OBJCOPY} -O binary ${SRC} ${TGT}"
    vars = ['HAS_EXTERNAL_FLASH_SECTIONS']
    EXTF_MEMORY_START = 0x90000000
    EXTF_MEMORY_END  = 0x90FFFFFF
    INTF_MEMORY_START = 0x08000000
    INTF_MEMORY_END = 0x08FFFFFF
    def keyword(self):
        return "Generating"
    def run(self):
        if self.env.HAS_EXTERNAL_FLASH_SECTIONS:
            ret = self.split_sections()
            if (ret < 0):
                return ret
            return ret
        else:
            # use --gap-fill 0xFF so gaps match erased flash, avoiding CRC
            # mismatch when loading via GDB vs bootloader/DroneCAN
            cmd = [self.env.get_flat('OBJCOPY'), '-O', 'binary', '--gap-fill', '0xFF',
                   self.inputs[0].relpath(), self.outputs[0].relpath()]
            self.exec_command(cmd)

    '''list sections and split into two binaries based on section's location in internal, external or in ram'''
    def split_sections(self):
        # get a list of sections
        cmd = "'{}' -A -x {}".format(self.env.get_flat('SIZE'), self.inputs[0].relpath())
        out = self.generator.bld.cmd_and_log(cmd, quiet=Context.BOTH, cwd=self.env.get_flat('BUILDROOT'))
        extf_sections = []
        intf_sections = []
        is_text_in_extf = False
        found_text_section = False
        ramsections = []
        for line in out.splitlines():
            section_line = line.split()
            if (len(section_line) < 3):
                continue
            try:
                if int(section_line[2], 0) == 0:
                    continue
                else:
                    addr = int(section_line[2], 0)
            except ValueError:
                continue
            if (addr >= self.EXTF_MEMORY_START) and (addr <= self.EXTF_MEMORY_END):
                extf_sections.append("--only-section=%s" % section_line[0])
                if section_line[0] == '.text':
                    is_text_in_extf = True
                    found_text_section = True
            elif (addr >= self.INTF_MEMORY_START) and (addr <= self.INTF_MEMORY_END):
                intf_sections.append("--only-section=%s" % section_line[0])
                if section_line[0] == '.text':
                    is_text_in_extf = False
                    found_text_section = True
            else:   # most likely RAM data, we place it in the same bin as text
                ramsections.append(section_line[0])

        if found_text_section:
            for section in ramsections:
                if is_text_in_extf:
                    extf_sections.append("--only-section=%s" % section)
                else:
                    intf_sections.append("--only-section=%s" % section)
        else:
            Logs.error("Couldn't find .text section")
        # create intf binary
        # use --gap-fill 0xFF so gaps match erased flash, avoiding CRC
        # mismatch when loading via GDB vs bootloader/DroneCAN
        if len(intf_sections):
            cmd = "'{}' {} --gap-fill 0xFF -O binary {} {}".format(self.env.get_flat('OBJCOPY'),
                                                ' '.join(intf_sections), self.inputs[0].relpath(), self.outputs[0].relpath())
        else:
            cmd = "cp /dev/null {}".format(self.outputs[0].relpath())
        ret = self.exec_command(cmd)
        if (ret < 0):
            return ret
        # create extf binary
        cmd = "'{}' {} --gap-fill 0xFF -O binary {} {}".format(self.env.get_flat('OBJCOPY'),
                                                ' '.join(extf_sections), self.inputs[0].relpath(), self.outputs[1].relpath())
        return self.exec_command(cmd)

    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

def to_unsigned(i):
    '''convert a possibly signed integer to unsigned'''
    if i < 0:
        i += 2**32
    return i

def sign_firmware(image, private_keyfile):
    '''sign firmware with private key'''
    try:
        import monocypher
    except ImportError:
        Logs.error("Please install monocypher with: python3 -m pip install pymonocypher==3.1.3.2")
        return None

    if monocypher.__version__ != "3.1.3.2":
        Logs.error("must use monocypher 3.1.3.2, please run: python3 -m pip install pymonocypher==3.1.3.2")
        return None

    try:
        key = open(private_keyfile, 'r').read()
    except Exception as ex:
        Logs.error("Failed to open %s" % private_keyfile)
        return None
    keytype = "PRIVATE_KEYV1:"
    if not key.startswith(keytype):
        Logs.error("Bad private key file %s" % private_keyfile)
        return None
    key = base64.b64decode(key[len(keytype):])
    sig = monocypher.signature_sign(key, image)
    sig_len = len(sig)
    sig_version = 30437
    return struct.pack("<IQ64s", sig_len+8, sig_version, sig)


class set_app_descriptor(Task.Task):
    '''setup app descriptor in bin file'''
    color='BLUE'
    # the descriptor embeds these, so changing one patches again
    vars = ['AP_SIGNED_FIRMWARE', 'PRIVATE_KEY', 'APP_DESCRIPTOR_GITHASH']
    def keyword(self):
        return "app_descriptor"
    def run(self):
        if self.generator.bld.env.AP_SIGNED_FIRMWARE:
            descriptor = b'\x41\xa3\xe5\xf2\x65\x69\x92\x07'
        else:
            descriptor = b'\x40\xa2\xe4\xf1\x64\x68\x91\x06'

        elf_in, bin_in = self.inputs
        elf_out, bin_out = self.outputs

        img = bytearray(open(bin_in.abspath(), 'rb').read())
        offset = img.find(descriptor)
        if offset == -1:
            Logs.info("No APP_DESCRIPTOR found")
        else:
            offset += len(descriptor)
            # next 8 bytes is 64 bit CRC. We set first 4 bytes to
            # CRC32 of image before descriptor and 2nd 4 bytes
            # to CRC32 of image after descriptor. This is very efficient
            # for bootloader to calculate
            # after CRC comes image length and 32 bit git hash
            upload_tools = self.env.get_flat('UPLOAD_TOOLS')
            sys.path.append(upload_tools)
            from uploader import crc32
            if self.generator.bld.env.AP_SIGNED_FIRMWARE:
                desc_len = 92
            else:
                desc_len = 16
            img1 = bytearray(img[:offset])
            img2 = bytearray(img[offset+desc_len:])
            crc1 = to_unsigned(crc32(img1))
            crc2 = to_unsigned(crc32(img2))
            githash = to_unsigned(int('0x' + self.env.APP_DESCRIPTOR_GITHASH, 16))
            if self.generator.bld.env.AP_SIGNED_FIRMWARE:
                sig = bytearray([0 for i in range(76)])
                if self.generator.bld.env.PRIVATE_KEY:
                    sig_signed = sign_firmware(img1+img2, self.generator.bld.env.PRIVATE_KEY)
                    if sig_signed:
                        Logs.info("Signed firmware")
                        sig = sig_signed
                    else:
                        self.generator.bld.fatal("Signing failed")
                desc = struct.pack('<IIII76s', crc1, crc2, len(img), githash, sig)
            else:
                desc = struct.pack('<IIII', crc1, crc2, len(img), githash)
            img[offset:offset+desc_len] = desc
            Logs.info("Applying APP_DESCRIPTOR %08x%08x" % (crc1, crc2))
        open(bin_out.abspath(), 'wb').write(img)

        elf_img = bytearray(open(elf_in.abspath(), 'rb').read())
        if offset != -1:
            zero_descriptor = descriptor + struct.pack("<IIII",0,0,0,0)
            elf_ofs = elf_img.find(zero_descriptor)
            if elf_ofs == -1:
                Logs.info("No APP_DESCRIPTOR found in elf file")
            else:
                elf_ofs += len(descriptor)
                elf_img[elf_ofs:elf_ofs+desc_len] = desc
                Logs.info("Applying APP_DESCRIPTOR %08x%08x to elf" % (crc1, crc2))
        open(elf_out.abspath(), 'wb').write(elf_img)


class generate_apj(Task.Task):
    '''generate an apj firmware file'''
    color='CYAN'
    # everything from env that the apj embeds
    vars = ['APJ_BOARD_ID', 'APJ_BOARD_TYPE', 'BOARD', 'FLASH_TOTAL', 'EXT_FLASH_SIZE_MB',
            'USBID', 'MANUFACTURER', 'BRAND_NAME', 'build_dates', 'AP_SIGNED_FIRMWARE',
            'PRIVATE_KEY', 'HAS_EXTERNAL_FLASH_SECTIONS', 'GIT_HEAD_HASH']
    def keyword(self):
        return "apj_gen"
    def run(self):
        import json, time, base64, zlib
        intf_img = open(self.inputs[0].abspath(),'rb').read()
        if self.env.HAS_EXTERNAL_FLASH_SECTIONS:
            extf_img = open(self.inputs[1].abspath(),'rb').read()
        else:
            extf_img = b""
        d = {
            "board_id": int(self.env.APJ_BOARD_ID),
            "magic": "APJFWv1",
            "description": "Firmware for a %s board" % self.env.APJ_BOARD_TYPE,
            "image": base64.b64encode(zlib.compress(intf_img,9)).decode('utf-8'),
            "extf_image": base64.b64encode(zlib.compress(extf_img,9)).decode('utf-8'),
            "summary": self.env.BOARD,
            "version": "0.1",
            "image_size": len(intf_img),
            "extf_image_size": len(extf_img),
            "flash_total": int(self.env.FLASH_TOTAL),
            "image_maxsize": int(self.env.FLASH_TOTAL),
            "flash_free": int(self.env.FLASH_TOTAL) - len(intf_img),
            "extflash_total": int(self.env.EXT_FLASH_SIZE_MB * 1024 * 1024),
            "extflash_free": int(self.env.EXT_FLASH_SIZE_MB * 1024 * 1024) - len(extf_img),
            "git_identity": self.env.GIT_HEAD_HASH,
            "board_revision": 0,
            "USBID": self.env.USBID
        }
        if self.env.MANUFACTURER:
            d["manufacturer"] = self.env.MANUFACTURER
        if self.env.BRAND_NAME:
            d["brand_name"] = self.env.BRAND_NAME
        if self.env.build_dates:
            # we omit build_time when we don't have build_dates so that apj
            # file is identical for same git hash and compiler
            d["build_time"] = int(time.time())
        if self.env.AP_SIGNED_FIRMWARE and self.env.PRIVATE_KEY:
            # The firmware file was signed during the build process, so set the flag
            d['signed_firmware'] = True
        apj_file = self.outputs[0].abspath()
        f = open(apj_file, "w")
        f.write(json.dumps(d, indent=4))
        f.close()

class build_abin(Task.Task):
    '''build an abin file for skyviper firmware upload via web UI'''
    color='CYAN'
    run_str='${TOOLS_SCRIPTS}/make_abin.sh ${SRC} ${TGT}'
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_normalized_bins(Task.Task):
    '''write the bin: the internal flash bin, or the external flash bin if the internal one is empty'''
    color='CYAN'
    def run(self):
        intf_bin, extf_bin = self.inputs
        src = extf_bin if os.path.getsize(intf_bin.abspath()) == 0 else intf_bin
        shutil.copyfile(src.abspath(), self.outputs[0].abspath())

    def keyword(self):
        return "bin cleanup"

class build_intel_hex(Task.Task):
    '''build an intel hex file for upload with DFU'''
    color='CYAN'
    run_str='${TOOLS_SCRIPTS}/make_intel_hex.py ${SRC} ${FLASH_RESERVE_START_KB}'
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('ch_ap_program')
@after_method('process_source')
def chibios_firmware(self):
    # every step below writes its own files and never modifies its inputs,
    # so waf only reruns a step when what it reads has changed. waf orders
    # the steps from their input and output files.
    link_output = self.link_task.outputs[0]
    # bootloader builds don't get an app descriptor patched in at all
    do_patch = not self.bld.env.BOOTLOADER

    elf = link_output
    if do_patch or self.env.DEFAULT_PARAMETERS:
        # link to a private file, the steps below write the final ELF
        elf = self.bld.bldnode.find_or_declare('unpatched/' + link_output.name)
        self.link_task.outputs = [elf]
        # report the size of the final ELF, not the unpatched one
        self.build_summary['binary'] = link_output.path_from(self.bld.bldnode)

    if self.env.DEFAULT_PARAMETERS:
        defaults_elf = link_output
        if do_patch:
            defaults_elf = self.bld.bldnode.find_or_declare('unpatched/' + link_output.name + '_defaults')
        default_params_task = self.create_task('set_default_parameters', src=elf, tgt=defaults_elf)
        params = self.bld.root.find_node(os.path.join(self.env.SRCROOT, self.env.get_flat('DEFAULT_PARAMETERS').replace("'", "")))
        if params is not None:
            default_params_task.dep_nodes.append(params)
        # the step imports apj_tool, so an edit to it embeds the defaults again
        default_params_task.dep_nodes.append(self.bld.root.find_node(self.env.APJ_TOOL))
        elf = defaults_elf

    bin_name = link_output.change_ext('.bin').name
    final_bin = self.bld.bldnode.find_or_declare('bin/' + bin_name)
    extf_bin = []
    intf_bin = final_bin
    if self.bld.env.HAS_EXTERNAL_FLASH_SECTIONS:
        extf_bin = [self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('_extf.bin').name)]
        # the final bin is chosen from the internal and external ones below
        intf_bin = self.bld.bldnode.find_or_declare('patched/' + bin_name)
    raw_bin = intf_bin
    if do_patch:
        raw_bin = self.bld.bldnode.find_or_declare('unpatched/' + bin_name)
    apj_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.apj').name)

    self.env.GIT_HEAD_HASH = self.bld.git_head_hash(short=True)

    self.create_task('generate_bin', src=elf, tgt=[raw_bin] + extf_bin)

    # we need to setup the app descriptor so the bootloader can validate the firmware
    if do_patch:
        self.env.APP_DESCRIPTOR_GITHASH = os.environ.get('GIT_VERSION', self.env.GIT_HEAD_HASH)
        app_descriptor_task = self.create_task('set_app_descriptor',
                                                src=[elf, raw_bin],
                                                tgt=[link_output, intf_bin])
        # the step imports crc32 from uploader.py, so an edit to it patches again
        app_descriptor_task.dep_nodes.append(self.bld.srcnode.find_node('Tools/scripts/uploader.py'))
        if self.env.AP_SIGNED_FIRMWARE and self.env.PRIVATE_KEY:
            # sign again when the key file changes
            key = self.bld.root.find_node(os.path.abspath(self.env.PRIVATE_KEY))
            if key is not None:
                app_descriptor_task.dep_nodes.append(key)

    self.create_task('generate_apj', src=[intf_bin] + extf_bin, tgt=apj_target)

    if self.env.BUILD_ABIN:
        abin_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.abin').name)
        abin_task = self.create_task('build_abin', src=[intf_bin] + extf_bin, tgt=abin_target)
        abin_task.dep_nodes.append(self.bld.srcnode.find_node('Tools/scripts/make_abin.sh'))

    if extf_bin:
        self.create_task('build_normalized_bins', src=[intf_bin] + extf_bin, tgt=final_bin)

    bootloader_board = self.env.BOARD
    if self.bld.env.USE_BOOTLOADER_FROM_BOARD:
        bootloader_board = self.bld.env.USE_BOOTLOADER_FROM_BOARD
    bootloader_bin = self.bld.srcnode.make_node("Tools/bootloaders/%s_bl.bin" % bootloader_board)
    if self.bld.env.HAVE_INTEL_HEX:
        if os.path.exists(bootloader_bin.abspath()):
            if int(self.bld.env.FLASH_RESERVE_START_KB) > 0:
                hex_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('_with_bl.hex').name)
            else:
                hex_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.hex').name)
            hex_task = self.create_task('build_intel_hex', src=[final_bin, bootloader_bin], tgt=hex_target)
            hex_task.dep_nodes.append(self.bld.srcnode.find_node('Tools/scripts/make_intel_hex.py'))
        else:
            print("Not embedding bootloader; %s does not exist" % bootloader_bin)

    if self.bld.options.upload:
        self.create_task('upload_fw', src=apj_target)

    if self.bld.options.upload_blueos:
        # uploads the apj next to the ELF, so it waits for it
        blueos_task = self.create_task('upload_fw_blueos', src=link_output)
        blueos_task.dep_nodes.append(apj_target)

def setup_canmgr_build(cfg):
    '''enable CANManager build. By doing this here we can auto-enable CAN in
    the build based on the presence of CAN pins in hwdef.dat except for AP_Periph builds'''
    env = cfg.env
    env.AP_LIBRARIES += [
        'AP_DroneCAN',
        'modules/DroneCAN/libcanard/*.c',
        ]
    env.INCLUDES += [
        cfg.srcnode.find_dir('modules/DroneCAN/libcanard').abspath(),
        ]
    env.CFLAGS += ['-DHAL_CAN_IFACES=2']

    if not env.AP_PERIPH:
        env.DEFINES += [
            'DRONECAN_CXX_WRAPPERS=1',
            'USE_USER_HELPERS=1',
            'CANARD_ENABLE_DEADLINE=1',
            'CANARD_MULTI_IFACE=1',
            'CANARD_ALLOCATE_SEM=1'
            ]

    cfg.get_board().with_can = True

def setup_canperiph_build(cfg):
    '''enable CAN build for peripherals'''
    env = cfg.env
    env.DEFINES += [
        'CANARD_ENABLE_DEADLINE=1',
        ]

    cfg.get_board().with_can = True

def setup_optimization(env):
    '''setup optimization flags for build'''
    if env.DEBUG:
        OPTIMIZE = "-Og"
    elif env.OPTIMIZE:
        OPTIMIZE = env.OPTIMIZE
    else:
        OPTIMIZE = "-Os"
    env.CFLAGS += [ OPTIMIZE ]
    env.CXXFLAGS += [ OPTIMIZE ]
    env.CHIBIOS_BUILD_FLAGS += ' USE_COPT=%s' % OPTIMIZE

def configure(cfg):
    cfg.find_program('make', var='MAKE')
    #cfg.objcopy = cfg.find_program('%s-%s'%(cfg.env.TOOLCHAIN,'objcopy'), var='OBJCOPY', mandatory=True)
    cfg.find_program('arm-none-eabi-objcopy', var='OBJCOPY')
    env = cfg.env

    # Flash and FATFS use the same crash dump in a mutually exclusive way.
    # Selecting flash explicitly should override an SD-capable board's FATFS
    # default, while explicitly selecting both still reaches the compile-time
    # diagnostic.
    if (cfg.options.enable_CRASHDUMP_FLASH and
            not cfg.options.enable_CRASHDUMP_FATFS):
        cfg.options.disable_CRASHDUMP_FATFS = True

    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['ch_ap_program']

    # newlib's abort() ends in a silent infinite loop, so it must never be
    # linked, including through library assertion failures
    env.DISALLOWED_SYMBOLS = ['abort', '__assert_func', '__assert']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['ch_ap_library']

    env.CH_ROOT = srcpath('modules/ChibiOS')
    env.AP_HAL_ROOT = srcpath('libraries/AP_HAL_ChibiOS')
    env.BUILDDIR = bldpath('modules/ChibiOS')
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.PT_DIR = srcpath('Tools/ardupilotwaf/chibios/image')
    env.MKFW_TOOLS = srcpath('Tools/ardupilotwaf')
    env.UPLOAD_TOOLS = srcpath('Tools/scripts')
    env.CHIBIOS_SCRIPTS = srcpath('libraries/AP_HAL_ChibiOS/hwdef/scripts')
    env.TOOLS_SCRIPTS = srcpath('Tools/scripts')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')
    env.SERIAL_PORT = srcpath('/dev/serial/by-id/*_STLink*')

    # relative paths to pass to make, relative to directory that make is run from
    env.CH_ROOT_REL = os.path.relpath(env.CH_ROOT, env.BUILDROOT)
    env.AP_HAL_REL = os.path.relpath(env.AP_HAL_ROOT, env.BUILDROOT)
    env.BUILDDIR_REL = os.path.relpath(env.BUILDDIR, env.BUILDROOT)

    mk_custom = srcpath('libraries/AP_HAL_ChibiOS/hwdef/%s/chibios_board.mk' % env.BOARD)
    mk_common = srcpath('libraries/AP_HAL_ChibiOS/hwdef/common/chibios_board.mk')
    # see if there is a board specific make file
    if os.path.exists(mk_custom):
        env.BOARD_MK = mk_custom
    else:
        env.BOARD_MK = mk_common

    if cfg.options.default_parameters:
        cfg.msg('Default parameters', cfg.options.default_parameters, color='YELLOW')
        env.DEFAULT_PARAMETERS = cfg.options.default_parameters

    try:
        hwdef_obj = generate_hwdef_h(env)
    except Exception:
        traceback.print_exc()
        cfg.fatal("Failed to process hwdef.dat")
    hal_common.process_hwdef_results(cfg, hwdef_obj)

    if not env.SIM_ENABLED:
        # Only simulation's JSON loader may use these C++ container handlers.
        env.DISALLOWED_SYMBOLS += [
            '_ZSt17__throw_bad_allocv',
            '_ZSt19__throw_logic_errorPKc',
            '_ZSt20__throw_length_errorPKc',
            '_ZSt24__throw_out_of_range_fmtPKcz',
        ]

    if env.IOMCU_BOOTLOADER:
        # The F1 IOMCU ABI leaves only 4 KiB for ChibiOS and the protocol.
        env.CHIBIOS_BUILD_FLAGS += ' USE_LTO=yes USE_OPT="-fomit-frame-pointer -ffat-lto-objects"'
        env.CXXFLAGS += ['-flto', '-falign-functions=4']
        env.LINKFLAGS += ['-flto']

    crashdump_fatfs_enabled = env.ENABLE_CRASHDUMP_FATFS
    crashdump_flash_enabled = env.ENABLE_CRASHDUMP_FLASH
    if cfg.options.enable_CRASHDUMP_FATFS:
        crashdump_fatfs_enabled = env.CRASHDUMP_FATFS_SUPPORTED
    elif cfg.options.disable_CRASHDUMP_FATFS:
        crashdump_fatfs_enabled = False
    if cfg.options.enable_CRASHDUMP_FLASH:
        crashdump_flash_enabled = True
    elif cfg.options.disable_CRASHDUMP_FLASH:
        crashdump_flash_enabled = False
    if env.AP_PERIPH:
        crashdump_fatfs_enabled = False
        crashdump_flash_enabled = False
    env.ENABLE_CRASHDUMP_FATFS = crashdump_fatfs_enabled
    env.ENABLE_CRASHDUMP_FLASH = crashdump_flash_enabled
    env.ENABLE_CRASHDUMP = crashdump_fatfs_enabled or crashdump_flash_enabled

    if env.DEBUG or env.DEBUG_SYMBOLS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_DEBUG_SYMBOLS=yes'
    if env.ENABLE_ASSERTS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_ASSERTS=yes'
    if env.ENABLE_MALLOC_GUARD:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_MALLOC_GUARD=yes'
    if env.ENABLE_STATS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_STATS=yes'
    if env.ENABLE_DFU_BOOT and env.BOOTLOADER:
        env.CHIBIOS_BUILD_FLAGS += ' USE_ASXOPT=-DCRT0_ENTRY_HOOK=TRUE'
    if env.AP_BOARD_START_TIME:
        env.CHIBIOS_BUILD_FLAGS += ' AP_BOARD_START_TIME=0x%x' % env.AP_BOARD_START_TIME

    if env.HAL_NUM_CAN_IFACES and not env.AP_PERIPH:
        setup_canmgr_build(cfg)
    if env.HAL_NUM_CAN_IFACES and env.AP_PERIPH and not env.BOOTLOADER:
        setup_canperiph_build(cfg)
    if env.HAL_NUM_CAN_IFACES and env.AP_PERIPH and int(env.HAL_NUM_CAN_IFACES)>1 and not env.BOOTLOADER:
        env.DEFINES += [ 'CANARD_MULTI_IFACE=1' ]
    setup_optimization(cfg.env)

def get_build_option_value(env, name):
    '''return the value of a build option, or None if it was not specified'''
    enable_option = 'enable_' + name
    disable_option = 'disable_' + name
    if env.OPTIONS.get(enable_option, False) or env.OPTIONS.get(enable_option.lower(), False):
        return 1
    if env.OPTIONS.get(disable_option, False) or env.OPTIONS.get(disable_option.lower(), False):
        return 0
    return None


def generate_hwdef_h(env):
    '''run chibios_hwdef.py'''
    if env.BOOTLOADER:
        if len(env.HWDEF) == 0:
            env.HWDEF = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % env.BOARD)
        else:
            # update to using hwdef-bl.dat
            env.HWDEF = env.HWDEF.replace('hwdef.dat', 'hwdef-bl.dat')
        bootloader_flag = True
    else:
        if len(env.HWDEF) == 0:
            env.HWDEF = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % env.BOARD)
        bootloader_flag = False

    hwdef_script = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)

    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)

    hwdef_obj = chibios_hwdef.ChibiOSHWDef(
        outdir=hwdef_out,
        bootloader=bootloader_flag,
        signed_fw=bool(env.AP_SIGNED_FIRMWARE),
        mass_storage_option=get_build_option_value(env, 'MASS_STORAGE'),
        hwdef=hwdef,
        # stringify like old subprocess based invocation. note that no error is
        # generated if this path is missing!
        default_params_filepath=str(env.DEFAULT_PARAMETERS),
        quiet=False,
    )
    hwdef_obj.run()

    if get_build_option_value(env, 'USB_debug_startup_wait') and not get_build_option_value(env, 'USB_debug'):
        raise Errors.WafError('USB debug startup wait requires --enable-USB-debug')
    if get_build_option_value(env, 'USB_debug'):
        if (bootloader_flag or hwdef_obj.mcu_series != 'STM32H7xx' or
                not hwdef_obj.dual_USB_enabled or 'OTG1' not in hwdef_obj.bytype or
                hwdef_obj.get_mcu_config('STM32_OTG2_IS_OTG1', False)):
            raise Errors.WafError('USB debug requires an STM32H7 application with dual CDC on OTG1')
        if hwdef_obj.get_config('EXT_FLASH_SIZE_MB', default=0, type=int):
            raise Errors.WafError('USB debug does not support external-flash boards')
        if 'EXT_WDOG' in hwdef_obj.bylabel:
            raise Errors.WafError('USB debug does not support external-watchdog boards')
        with open(os.path.join(hwdef_out, 'hwdef.h'), 'a') as header:
            header.write('\n#define AP_USB_DEBUG_ENABLED 1\n')

    return hwdef_obj

def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    if bld.env.HAL_NUM_CAN_IFACES:
        bld.get_board().with_can = True
    if bld.env.WITH_LITTLEFS:
        bld.get_board().with_littlefs = True
    setup_optimization(bld.env)

def build(bld):

    # make ccache effective on ChibiOS builds
    os.environ['CCACHE_IGNOREOPTIONS'] = '--specs=nano.specs --specs=nosys.specs'
    
    bld(
        # create the file modules/ChibiOS/include_dirs
        rule="BUILDDIR=${BUILDDIR_REL} BUILDROOT=${BUILDROOT} CHIBIOS=${CH_ROOT_REL} AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${MAKE} pass -f '${BOARD_MK}'",
        group='dynamic_sources',
        target=bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')
    )

    bld(
        # create the file modules/ChibiOS/include_dirs
        rule="echo // BUILD_FLAGS: ${BUILDDIR_REL} ${BUILDROOT} ${CH_ROOT_REL} ${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} > chibios_flags.h",
        group='dynamic_sources',
        target=bld.bldnode.find_or_declare('chibios_flags.h')
    )
    
    common_src = [bld.bldnode.find_or_declare('hwdef.h'),
                  bld.bldnode.find_or_declare('hw.dat'),
                  bld.bldnode.find_or_declare('ldscript.ld'),
                  bld.bldnode.find_or_declare('common.ld'),
                  bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')]
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.[ch]')
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.mk')
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/CrashCatcher_armv7m_asm.S')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.mk')
    if bld.env.ROMFS_FILES:
        common_src += [bld.bldnode.find_or_declare('ap_romfs_embedded.h')]

    if bld.env.ENABLE_CRASHDUMP:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' BUILDROOT='${BUILDROOT}' ENABLE_CRASHDUMP=yes CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} '${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=[
                bld.bldnode.find_or_declare('modules/ChibiOS/libch.a'),
                bld.bldnode.find_or_declare('modules/ChibiOS/obj/CrashCatcher_armv7m_asm.o')
            ]
        )
    else:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' BUILDROOT='${BUILDROOT}' CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} '${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=bld.bldnode.find_or_declare('modules/ChibiOS/libch.a')
        )
    ch_task.name = "ChibiOS_lib"
    DSP_LIBS = {
        'cortex-m4' : 'libarm_cortexM4lf_math.a',
        'cortex-m7' : 'libarm_cortexM7lfdp_math.a',
    }
    if bld.env.CORTEX in DSP_LIBS:
        libname = DSP_LIBS[bld.env.CORTEX]
        # we need to copy the library on cygwin as it doesn't handle linking outside build tree
        shutil.copyfile(os.path.join(bld.env.SRCROOT,'libraries/AP_GyroFFT/CMSIS_5/lib',libname),
                        os.path.join(bld.env.BUILDROOT,'modules/ChibiOS/libDSP.a'))
        bld.env.LIB += ['DSP']
    bld.env.LIB += ['ch']
    bld.env.LIBPATH += ['modules/ChibiOS/']
    if bld.env.ENABLE_CRASHDUMP:
        # Link the handler directly so it overrides ChibiOS's weak fault handler.
        bld.env.LINKFLAGS += ['modules/ChibiOS/obj/CrashCatcher_armv7m_asm.o']
    # list of functions that will be wrapped to move them out of libc into our
    # own code
    wraplist = ['sscanf', 'fprintf', 'snprintf', 'vsnprintf', 'vasprintf', 'asprintf', 'vprintf', 'scanf', 'printf']

    # list of functions that we will give a link error for if they are
    # used. This is to prevent accidental use of these functions
    blacklist = ['_sbrk', '_sbrk_r', '_malloc_r', '_calloc_r', '_free_r', 'ftell', 'realloc',
                 'fopen', 'fflush', 'fwrite', 'fread', 'fputs', 'fgets',
                 'clearerr', 'fseek', 'ferror', 'fclose', 'tmpfile', 'getc', 'ungetc', 'feof',
                'ftell', 'freopen', 'remove', 'vfprintf', 'vfprintf_r', 'fscanf',
                '_gettimeofday', '_times', '_times_r', '_gettimeofday_r', 'time', 'clock',
                'setjmp']

    # these functions use global state that is not thread safe
    blacklist += ['gmtime']

    for w in wraplist + blacklist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]
