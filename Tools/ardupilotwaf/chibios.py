# encoding: utf-8

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

_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/ChibiOS')
    tmp_str = bldnode.find_node('include_dirs').read()
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
        if not d in idirs2:
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
    self.use += ' ch'
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
            cmd = "{} '{}/uploader.py' '{}'".format('python.exe', upload_tools, src.abspath())
        else:
            cmd = "{} '{}/uploader.py' '{}'".format(self.env.get_flat('PYTHON'), upload_tools, src.abspath())
        if upload_port is not None:
            cmd += " '--port' '%s'" % upload_port
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
        # Requirements: Windows must have Python3.9.x (NTO 3.10.x) installed and a few packages.
        import subprocess
        try:
            where_python = subprocess.check_output('where.exe python.exe', shell=True, text=True)
        except subprocess.CalledProcessError:
            #if where.exe can't find the file it returns a non-zero result which throws this exception
            where_python = ""
        if not where_python or not "\Python\Python" in where_python or "python.exe" not in where_python:
            print(self.get_full_wsl2_error_msg("Windows python.exe not found"))
            return False
        python_version = subprocess.check_output('python.exe --version', shell=True, text=True)
        if "3.10." in python_version:
            print(self.get_full_wsl2_error_msg("Your Windows %s version is not compatible" % python_version.strip()))
            return False
        return True

    def get_full_wsl2_error_msg(self, error_msg):
        return ("""
        ****************************************
        ****************************************
        WSL2 firmware uploads use the host's Windows Python.exe so it has access to the COM ports.

        %s
        Please download Windows Installer 3.9.x (not 3.10) from https://www.python.org/downloads/
        and make sure to add it to your path during the installation. Once installed, run this
        command in Powershell or Command Prompt to install some packages:
        
        pip.exe install empy pyserial
        ****************************************
        ****************************************
        """ % error_msg)

    def exec_command(self, cmd, **kw):
        kw['stdout'] = sys.stdout
        return super(upload_fw, self).exec_command(cmd, **kw)

    def keyword(self):
        return "Uploading"

class set_default_parameters(Task.Task):
    color='CYAN'
    always_run = True
    def keyword(self):
        return "apj_tool"
    def run(self):
        rel_default_parameters = self.env.get_flat('DEFAULT_PARAMETERS').replace("'", "")
        abs_default_parameters = os.path.join(self.env.SRCROOT, rel_default_parameters)
        apj_tool = self.env.APJ_TOOL
        sys.path.append(os.path.dirname(apj_tool))
        from apj_tool import embedded_defaults
        defaults = embedded_defaults(self.inputs[0].abspath())
        if defaults.find():
            defaults.set_file(abs_default_parameters)
            defaults.save()


class generate_bin(Task.Task):
    color='CYAN'
    # run_str="${OBJCOPY} -O binary ${SRC} ${TGT}"
    always_run = True
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
            cmd = [self.env.get_flat('OBJCOPY'), '-O', 'binary', self.inputs[0].relpath(),  self.outputs[0].relpath()]
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
        if len(intf_sections):
            cmd = "'{}' {} -O binary {} {}".format(self.env.get_flat('OBJCOPY'),
                                                ' '.join(intf_sections), self.inputs[0].relpath(), self.outputs[0].relpath())
        else:
            cmd = "cp /dev/null {}".format(self.outputs[0].relpath())
        ret = self.exec_command(cmd)
        if (ret < 0):
            return ret
        # create extf binary
        cmd = "'{}' {} -O binary {} {}".format(self.env.get_flat('OBJCOPY'),
                                                ' '.join(extf_sections), self.inputs[0].relpath(), self.outputs[1].relpath())
        return self.exec_command(cmd)

    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

def to_unsigned(i):
    '''convert a possibly signed integer to unsigned'''
    if i < 0:
        i += 2**32
    return i

class set_app_descriptor(Task.Task):
    '''setup app descriptor in bin file'''
    color='BLUE'
    always_run = True
    def keyword(self):
        return "app_descriptor"
    def run(self):
        descriptor = b'\x40\xa2\xe4\xf1\x64\x68\x91\x06'
        img = open(self.inputs[0].abspath(), 'rb').read()
        offset = img.find(descriptor)
        if offset == -1:
            Logs.info("No APP_DESCRIPTOR found")
            return
        offset += 8
        # next 8 bytes is 64 bit CRC. We set first 4 bytes to
        # CRC32 of image before descriptor and 2nd 4 bytes
        # to CRC32 of image after descriptor. This is very efficient
        # for bootloader to calculate
        # after CRC comes image length and 32 bit git hash
        upload_tools = self.env.get_flat('UPLOAD_TOOLS')
        sys.path.append(upload_tools)
        from uploader import crc32
        desc_len = 92
        crc1 = to_unsigned(crc32(bytearray(img[:offset])))
        crc2 = to_unsigned(crc32(bytearray(img[offset+desc_len:])))
        githash = to_unsigned(int('0x' + os.environ.get('GIT_VERSION', self.generator.bld.git_head_hash(short=True)),16))
        desc = struct.pack('<IIIII72s', crc1, crc2, len(img), githash, 0, bytes(bytearray([0 for i in range(72)])))
        img = img[:offset] + desc + img[offset+desc_len:]
        Logs.info("Applying APP_DESCRIPTOR %08x%08x" % (crc1, crc2))
        open(self.inputs[0].abspath(), 'wb').write(img)

class generate_apj(Task.Task):
    '''generate an apj firmware file'''
    color='CYAN'
    always_run = True
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
            "git_identity": self.generator.bld.git_head_hash(short=True),
            "board_revision": 0,
            "USBID": self.env.USBID
        }
        if self.env.MANUFACTURER:
            d["manufacturer"] = self.env.MANUFACTURER
        if self.env.BRAND_NAME:
            d["brand_name"] = self.env.BRAND_NAME
        if self.env.build_dates:
            # we omit build_time when we don't have build_dates so that apj
            # file is idential for same git hash and compiler
            d["build_time"] = int(time.time())
        apj_file = self.outputs[0].abspath()
        f = open(apj_file, "w")
        f.write(json.dumps(d, indent=4))
        f.close()

class build_abin(Task.Task):
    '''build an abin file for skyviper firmware upload via web UI'''
    color='CYAN'
    run_str='${TOOLS_SCRIPTS}/make_abin.sh ${SRC}.bin ${SRC}.abin'
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class build_normalized_bins(Task.Task):
    '''Move external flash binaries to regular location if regular bin is zero length'''
    color='CYAN'
    always_run = True
    def run(self):
        if self.env.HAS_EXTERNAL_FLASH_SECTIONS and os.path.getsize(self.inputs[0].abspath()) == 0:
                os.remove(self.inputs[0].abspath())
                shutil.move(self.inputs[1].abspath(), self.inputs[0].abspath())

    def keyword(self):
        return "bin cleanup"

class build_intel_hex(Task.Task):
    '''build an intel hex file for upload with DFU'''
    color='CYAN'
    run_str='${TOOLS_SCRIPTS}/make_intel_hex.py ${SRC} ${FLASH_RESERVE_START_KB}'
    always_run = True
    def keyword(self):
        return "Generating"
    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

@feature('ch_ap_program')
@after_method('process_source')
def chibios_firmware(self):
    self.link_task.always_run = True

    link_output = self.link_task.outputs[0]
    hex_task = None

    if self.bld.env.HAS_EXTERNAL_FLASH_SECTIONS:
        bin_target = [self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.bin').name),
                      self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('_extf.bin').name)]
    else:
        bin_target = [self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.bin').name)]
    apj_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.apj').name)

    generate_bin_task = self.create_task('generate_bin', src=link_output, tgt=bin_target)
    generate_bin_task.set_run_after(self.link_task)

    generate_apj_task = self.create_task('generate_apj', src=bin_target, tgt=apj_target)
    generate_apj_task.set_run_after(generate_bin_task)

    if self.env.BUILD_ABIN:
        abin_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.abin').name)
        abin_task = self.create_task('build_abin', src=link_output, tgt=abin_target)
        abin_task.set_run_after(generate_apj_task)

    cleanup_task = self.create_task('build_normalized_bins', src=bin_target)
    cleanup_task.set_run_after(generate_apj_task)

    bootloader_bin = self.bld.srcnode.make_node("Tools/bootloaders/%s_bl.bin" % self.env.BOARD)
    if self.bld.env.HAVE_INTEL_HEX:
        if os.path.exists(bootloader_bin.abspath()):
            hex_target = self.bld.bldnode.find_or_declare('bin/' + link_output.change_ext('.hex').name)
            hex_task = self.create_task('build_intel_hex', src=[bin_target[0], bootloader_bin], tgt=hex_target)
            hex_task.set_run_after(cleanup_task)
        else:
            print("Not embedding bootloader; %s does not exist" % bootloader_bin)

    if self.env.DEFAULT_PARAMETERS:
        default_params_task = self.create_task('set_default_parameters',
                                               src=link_output)
        default_params_task.set_run_after(self.link_task)
        generate_bin_task.set_run_after(default_params_task)

    # we need to setup the app descriptor so the bootloader can validate the firmware
    if not self.bld.env.BOOTLOADER:
        app_descriptor_task = self.create_task('set_app_descriptor', src=bin_target)
        app_descriptor_task.set_run_after(generate_bin_task)
        generate_apj_task.set_run_after(app_descriptor_task)
        if hex_task is not None:
            hex_task.set_run_after(app_descriptor_task)
    else:
        generate_apj_task.set_run_after(generate_bin_task)
        if hex_task is not None:
            hex_task.set_run_after(generate_bin_task)
        
    if self.bld.options.upload:
        _upload_task = self.create_task('upload_fw', src=apj_target)
        _upload_task.set_run_after(generate_apj_task)

def setup_canmgr_build(cfg):
    '''enable CANManager build. By doing this here we can auto-enable CAN in
    the build based on the presence of CAN pins in hwdef.dat except for AP_Periph builds'''
    env = cfg.env
    env.AP_LIBRARIES += [
        'AP_UAVCAN',
        'modules/uavcan/libuavcan/src/**/*.cpp',
        ]

    env.CFLAGS += ['-DHAL_CAN_IFACES=2']

    env.DEFINES += [
        'UAVCAN_CPP_VERSION=UAVCAN_CPP03',
        'UAVCAN_NO_ASSERTIONS=1',
        'UAVCAN_NULLPTR=nullptr'
        ]

    if cfg.env.HAL_CANFD_SUPPORTED:
        env.DEFINES += ['UAVCAN_SUPPORT_CANFD=1']
    else:
        env.DEFINES += ['UAVCAN_SUPPORT_CANFD=0']


    env.INCLUDES += [
        cfg.srcnode.find_dir('modules/uavcan/libuavcan/include').abspath(),
        ]
    cfg.get_board().with_can = True

def load_env_vars(env):
    '''optionally load extra environment variables from env.py in the build directory'''
    print("Checking for env.py")
    env_py = os.path.join(env.BUILDROOT, 'env.py')
    if not os.path.exists(env_py):
        print("No env.py found")
        return
    e = pickle.load(open(env_py, 'rb'))
    for k in e.keys():
        v = e[k]
        if k == 'ROMFS_FILES':
            env.ROMFS_FILES += v
            continue
        if k in env:
            if isinstance(env[k], dict):
                a = v.split('=')
                env[k][a[0]] = '='.join(a[1:])
                print("env updated %s=%s" % (k, v))
            elif isinstance(env[k], list):
                env[k].append(v)
                print("env appended %s=%s" % (k, v))
            else:
                env[k] = v
                print("env added %s=%s" % (k, v))
        else:
            env[k] = v
            print("env set %s=%s" % (k, v))
    if env.ENABLE_ASSERTS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_ASSERTS=yes'
    if env.ENABLE_MALLOC_GUARD:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_MALLOC_GUARD=yes'
    if env.ENABLE_STATS:
        env.CHIBIOS_BUILD_FLAGS += ' ENABLE_STATS=yes'


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
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['ch_ap_program']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['ch_ap_library']

    env.CH_ROOT = srcpath('modules/ChibiOS')
    env.CC_ROOT = srcpath('modules/CrashDebug/CrashCatcher')
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
    env.CC_ROOT_REL = os.path.relpath(env.CC_ROOT, env.BUILDROOT)
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
        ret = generate_hwdef_h(env)
    except Exception:
        cfg.fatal("Failed to process hwdef.dat")
    if ret != 0:
        cfg.fatal("Failed to process hwdef.dat ret=%d" % ret)
    load_env_vars(cfg.env)
    if env.HAL_NUM_CAN_IFACES and not env.AP_PERIPH:
        setup_canmgr_build(cfg)
    setup_optimization(cfg.env)

def generate_hwdef_h(env):
    '''run chibios_hwdef.py'''
    import subprocess
    if env.BOOTLOADER:
        if len(env.HWDEF) == 0:
            env.HWDEF = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef-bl.dat' % env.BOARD)
        else:
            # update to using hwdef-bl.dat
            env.HWDEF = env.HWDEF.replace('hwdef.dat', 'hwdef-bl.dat')
        if (env.SECURE_BL):
            env.BOOTLOADER_OPTION="--bootloader --secure-bl"
        else:
            env.BOOTLOADER_OPTION="--bootloader"
    else:
        if len(env.HWDEF) == 0:
            env.HWDEF = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ChibiOS/hwdef/%s/hwdef.dat' % env.BOARD)
        env.BOOTLOADER_OPTION=""

    hwdef_script = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    python = sys.executable
    cmd = "{0} '{1}' -D '{2}' --params '{3}' '{4}'".format(python, hwdef_script, hwdef_out, env.DEFAULT_PARAMETERS, env.HWDEF)
    if env.HWDEF_EXTRA:
        cmd += " '{0}'".format(env.HWDEF_EXTRA)
    if env.BOOTLOADER_OPTION:
        cmd += " " + env.BOOTLOADER_OPTION
    return subprocess.call(cmd, shell=True)

def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    load_env_vars(bld.env)
    if bld.env.HAL_NUM_CAN_IFACES:
        bld.get_board().with_can = True
    hwdef_h = os.path.join(bld.env.BUILDROOT, 'hwdef.h')
    if not os.path.exists(hwdef_h):
        print("Generating hwdef.h")
        try:
            ret = generate_hwdef_h(bld.env)
        except Exception:
            bld.fatal("Failed to process hwdef.dat")
        if ret != 0:
            bld.fatal("Failed to process hwdef.dat ret=%d" % ret)
    setup_optimization(bld.env)

def build(bld):


    hwdef_rule="%s '%s/hwdef/scripts/chibios_hwdef.py' -D '%s' --params '%s' '%s'" % (
            bld.env.get_flat('PYTHON'),
            bld.env.AP_HAL_ROOT,
            bld.env.BUILDROOT,
            bld.env.default_parameters,
            bld.env.HWDEF)
    if bld.env.HWDEF_EXTRA:
        hwdef_rule += " " + bld.env.HWDEF_EXTRA
    if bld.env.BOOTLOADER_OPTION:
        hwdef_rule += " " + bld.env.BOOTLOADER_OPTION
    bld(
        # build hwdef.h from hwdef.dat. This is needed after a waf clean
        source=bld.path.ant_glob(bld.env.HWDEF),
        rule=hwdef_rule,
        group='dynamic_sources',
        target=[bld.bldnode.find_or_declare('hwdef.h'),
                bld.bldnode.find_or_declare('ldscript.ld'),
                bld.bldnode.find_or_declare('hw.dat')]
    )
    
    bld(
        # create the file modules/ChibiOS/include_dirs
        rule="touch Makefile && BUILDDIR=${BUILDDIR_REL} CRASHCATCHER=${CC_ROOT_REL} CHIBIOS=${CH_ROOT_REL} AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${MAKE} pass -f '${BOARD_MK}'",
        group='dynamic_sources',
        target=bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')
    )

    common_src = [bld.bldnode.find_or_declare('hwdef.h'),
                  bld.bldnode.find_or_declare('hw.dat'),
                  bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')]
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.[ch]')
    common_src += bld.path.ant_glob('libraries/AP_HAL_ChibiOS/hwdef/common/*.mk')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.[ch]')
    common_src += bld.path.ant_glob('modules/ChibiOS/os/hal/**/*.mk')
    if bld.env.ROMFS_FILES:
        common_src += [bld.bldnode.find_or_declare('ap_romfs_embedded.h')]

    if bld.env.ENABLE_CRASHDUMP:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' CRASHCATCHER='${CC_ROOT_REL}' CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} '${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
            group='dynamic_sources',
            source=common_src,
            target=[bld.bldnode.find_or_declare('modules/ChibiOS/libch.a'), bld.bldnode.find_or_declare('modules/ChibiOS/libcc.a')]
        )
    else:
        ch_task = bld(
            # build libch.a from ChibiOS sources and hwdef.h
            rule="BUILDDIR='${BUILDDIR_REL}' CHIBIOS='${CH_ROOT_REL}' AP_HAL=${AP_HAL_REL} ${CHIBIOS_BUILD_FLAGS} ${CHIBIOS_BOARD_NAME} ${HAL_MAX_STACK_FRAME_SIZE} '${MAKE}' -j%u lib -f '${BOARD_MK}'" % bld.options.jobs,
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
        bld.env.LINKFLAGS += ['-Wl,-whole-archive', 'modules/ChibiOS/libcc.a', '-Wl,-no-whole-archive']
    # list of functions that will be wrapped to move them out of libc into our
    # own code note that we also include functions that we deliberately don't
    # implement anywhere (the FILE* functions). This allows us to get link
    # errors if we accidentially try to use one of those functions either
    # directly or via another libc call
    wraplist = ['sscanf', 'fprintf', 'snprintf', 'vsnprintf','vasprintf','asprintf','vprintf','scanf',
                'fiprintf','printf',
                'fopen', 'fflush', 'fwrite', 'fread', 'fputs', 'fgets',
                'clearerr', 'fseek', 'ferror', 'fclose', 'tmpfile', 'getc', 'ungetc', 'feof',
                'ftell', 'freopen', 'remove', 'vfprintf', 'fscanf' ]
    for w in wraplist:
        bld.env.LINKFLAGS += ['-Wl,--wrap,%s' % w]
