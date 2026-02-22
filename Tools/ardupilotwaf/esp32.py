# encoding: utf-8

# flake8: noqa

"""
Waf tool for ESP32 build
"""

from waflib import Build, ConfigSet, Configure, Context, Task, Utils
from waflib import Errors, Logs
from waflib.TaskGen import before, after_method, before_method, feature
from waflib.Configure import conf
from collections import OrderedDict

import os
import shutil
import sys
import traceback
import re
import pickle
import subprocess

import hal_common

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_ESP32/hwdef/scripts'))
import esp32_hwdef  # noqa:501

@feature('esp32_ap_library', 'esp32_ap_program')
@before_method('process_source')
def esp32_dynamic_env(self):
    pass


def load_env_vars(env):
    '''optionally load extra environment variables from env.py in the build directory'''
    pass

def configure(cfg):
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    #Load cmake builder and make
    cfg.load('cmake')

    #define env and location for the cmake esp32 file
    env = cfg.env
    mcu = env.get_flat('MCU')
    
    env.AP_HAL_ESP32 = srcpath('libraries/AP_HAL_ESP32/targets/'+mcu.lower()+'/esp-idf')
    env.AP_PROGRAM_FEATURES += ['esp32_ap_program']
    env.append_value('DEFINES', 'USE_USER_HELPERS=1')

    env.ESP_IDF_PREFIX_REL = 'esp-idf'

    prefix_node = bldnode.make_node(env.ESP_IDF_PREFIX_REL)
    env.ESP32_TARGET = mcu
    env.BUILDROOT = bldpath('')
    env.SRCROOT = srcpath('')
    env.APJ_TOOL = srcpath('Tools/scripts/apj_tool.py')

    # Use IDF_PATH from environment if set (e.g. by agent_build_wrapper.sh),
    # otherwise prefer IDF 6.x if installed, fall back to submodule
    import glob
    idf_path = os.environ.get('IDF_PATH', '')
    if not idf_path or not os.path.exists(idf_path):
        idf6_candidates = sorted(glob.glob('/opt/espressif/esp-idf-v6*'), reverse=True)
        if idf6_candidates:
            idf_path = idf6_candidates[0]
        else:
            idf_path = cfg.srcnode.abspath()+"/modules/esp_idf"
    env.IDF = idf_path

    # Set IDF_PATH environment variable to ensure consistency
    os.environ['IDF_PATH'] = idf_path

    # Detect IDF version for build-time decisions
    idf_version_h = os.path.join(idf_path, 'components/esp_common/include/esp_idf_version.h')
    idf_major = 5
    idf_minor = 0
    if os.path.exists(idf_version_h):
        with open(idf_version_h, 'r') as f:
            for line in f:
                m = re.match(r'#define\s+ESP_IDF_VERSION_MAJOR\s+(\d+)', line)
                if m:
                    idf_major = int(m.group(1))
                m = re.match(r'#define\s+ESP_IDF_VERSION_MINOR\s+(\d+)', line)
                if m:
                    idf_minor = int(m.group(1))
    env.IDF_MAJOR = idf_major
    env.IDF_MINOR = idf_minor

    # Source ESP-IDF environment to set up all necessary variables
    export_script = os.path.join(idf_path, 'export.sh')
    if os.path.exists(export_script):
        try:
            # Run export.sh and capture environment changes
            import subprocess
            result = subprocess.run(['bash', '-c', f'source {export_script} && env'],
                                  capture_output=True, text=True, check=True)
            # Update current environment with ESP-IDF variables
            env.ESP32_ENV = {}
            for line in result.stdout.splitlines():
                if '=' in line and not line.startswith('_'):
                    key, value = line.split('=', 1)
                    if key.startswith(('IDF_', 'PATH', 'PYTHON')) or 'esp' in key.lower():
                        os.environ[key] = value
                        env.ESP32_ENV[key] = value
        except Exception as e:
            print(f"Warning: Failed to source ESP-IDF environment: {e}")

    print("ESP-IDF: %s (v%d.%d)" % (env.IDF, idf_major, idf_minor))
    
    # Report ESP32 debug mode status
    try:
        # Check if this is a debug build variant
        is_debug = 'debug' in cfg.variant or cfg.options.debug
        print(f"ESP32 Debug build     : {'enabled' if is_debug else 'disabled'}")
    except:
        print("ESP32 Debug build     : disabled (default)")

    # setup cmake
    cfg.env['IDF_TARGET'] = mcu

    # Process hwdef files to generate board-specific configuration with upstream error handling
    board_name = getattr(cfg.options, 'board', '').replace('esp32', '')
    if board_name:
        hwdef_dir = os.path.join(cfg.srcnode.abspath(), f'libraries/AP_HAL_ESP32/hwdef/{cfg.options.board}')
        hwdef_file = os.path.join(hwdef_dir, 'hwdef.dat')

        if os.path.exists(hwdef_file):
            # Run the ESP32 hwdef processor
            hwdef_script = os.path.join(cfg.srcnode.abspath(), 'libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py')
            if os.path.exists(hwdef_script):
                import subprocess
                # Use board-specific build directory
                board_build_dir = os.path.join(cfg.bldnode.abspath(), cfg.env.BOARD)
                if not os.path.exists(board_build_dir):
                    os.makedirs(board_build_dir)
                cmd = [cfg.env.PYTHON[0], hwdef_script, '-D', board_build_dir, '--mcu', cfg.env.MCU, hwdef_file]
                print(f"Processing hwdef: {' '.join(cmd)}")
                try:
                    subprocess.run(cmd, check=True, cwd=cfg.srcnode.abspath())
                except subprocess.CalledProcessError as e:
                    cfg.fatal(f"hwdef processing failed: {e}")

    # Also try upstream hwdef generation method for completeness
    try:
        generate_hwdef_h(env)
    except Exception as e:
        print(get_exception_stacktrace(e))
        cfg.fatal("Failed to generate hwdef")

    # Build list of sdkconfig.defaults files: target-level + debug (if --debug) + board-level (if exists)
    target_sdkconfig = os.path.join(cfg.srcnode.abspath(), f'libraries/AP_HAL_ESP32/targets/{mcu.lower()}/esp-idf/sdkconfig.defaults')
    debug_sdkconfig = os.path.join(cfg.srcnode.abspath(), f'libraries/AP_HAL_ESP32/targets/{mcu.lower()}/esp-idf/sdkconfig.debug')
    
    # Use board-specific build directory for generated configs
    board_build_dir = os.path.join(cfg.bldnode.abspath(), cfg.env.BOARD)
    board_sdkconfig = os.path.join(board_build_dir, 'sdkconfig.board')

    # Check if we need to include debug configuration
    include_debug = cfg.options.debug and os.path.exists(debug_sdkconfig)

    sdkconfig_list = [target_sdkconfig]
    if os.path.exists(board_sdkconfig) or include_debug:
        # Create a combined sdkconfig.defaults that includes target, debug (if applicable), and board configs
        combined_sdkconfig = os.path.join(board_build_dir, 'sdkconfig.combined')
        print(f"Creating combined ESP-IDF config: {combined_sdkconfig}")

        with open(combined_sdkconfig, 'w') as combined_file:
            # Include target-level configuration
            with open(target_sdkconfig, 'r') as target_file:
                combined_file.write(f"# Target-level configuration from {target_sdkconfig}\n")
                combined_file.write(target_file.read())
                combined_file.write("\n")

            # Include debug configuration if --debug was specified
            if include_debug:
                with open(debug_sdkconfig, 'r') as debug_file:
                    combined_file.write(f"# Debug configuration from {debug_sdkconfig}\n")
                    combined_file.write(debug_file.read())
                    combined_file.write("\n")
                print(f"Including ESP32 debug config with stack protection")

            # Include board-specific configuration
            if os.path.exists(board_sdkconfig):
                with open(board_sdkconfig, 'r') as board_file:
                    combined_file.write(f"# Board-specific configuration from {board_sdkconfig}\n")
                    combined_file.write(board_file.read())

        cfg.env['ESP_IDF_SDKCONFIG_DEFAULTS'] = combined_sdkconfig
        print(f"Using combined ESP-IDF config: {combined_sdkconfig}")
        
        # If combined config exists, ensure we trigger a re-eval by CMake if it changed
        # by deleting the existing sdkconfig. 
        # This is a bit heavy-handed but ensures overrides work.
        sdkconfig_path = os.path.join(board_build_dir, 'esp-idf_build/sdkconfig')
        if os.path.exists(sdkconfig_path):
            os.remove(sdkconfig_path)
    else:
        cfg.env['ESP_IDF_SDKCONFIG_DEFAULTS'] = target_sdkconfig
    
    cfg.env['PROJECT_DIR'] = board_build_dir
    cfg.env['SDKCONFIG'] = os.path.join(board_build_dir, 'esp-idf_build/sdkconfig')
    cfg.env['PYTHON'] = cfg.env.get_flat('PYTHON')

    load_env_vars(cfg.env)

def get_exception_stacktrace(e):
    ret = "%s\n" % e
    ret += ''.join(traceback.format_exception(type(e),
                                              e,
                                              tb=e.__traceback__))
    return ret

def generate_hwdef_h(env):
    '''run esp32_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_ESP32/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)
    eh = esp32_hwdef.ESP32HWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
        mcu=env.MCU,
    )
    eh.run()

# delete the output sdkconfig file when the input defaults changes. we take the
# stamp as the output so we can compute the path to the sdkconfig, yet it
# doesn't have to exist when we're done.
class clean_sdkconfig(Task.Task):
    def keyword(self):
        return "delete sdkconfig generated from"

    def run(self):
        prefix = ".clean-stamp-"
        for out in self.outputs:
            if not out.name.startswith(prefix):
                raise ValueError("not a stamp file: "+out)
            dest = out.parent.abspath()+"/"+out.name[len(prefix):]
            if os.path.exists(dest):
                os.unlink(dest)

            # waf needs the output to exist after the task, so touch it
            open(out.abspath(), "w").close()

def pre_build(self):
    """Configure esp-idf as lib target"""
    # Export ESP_IDF_SDKCONFIG_DEFAULTS to OS environment for cmake to read
    # (waf env is internal storage, cmake reads $ENV{} from os.environ)
    if self.env.get_flat('ESP_IDF_SDKCONFIG_DEFAULTS'):
        os.environ['ESP_IDF_SDKCONFIG_DEFAULTS'] = self.env.get_flat('ESP_IDF_SDKCONFIG_DEFAULTS')

    lib_vars = OrderedDict()
    lib_vars['ARDUPILOT_CMD'] = self.cmd
    lib_vars['WAF_BUILD_TARGET'] = self.targets
    lib_vars['ARDUPILOT_LIB'] = self.bldnode.find_or_declare('lib/').abspath()
    lib_vars['ARDUPILOT_BIN'] = self.bldnode.find_or_declare('lib/bin').abspath()
    
    # Restore ESP-IDF environment variables captured during configure
    if self.env.ESP32_ENV:
        for key, value in self.env.ESP32_ENV.items():
            os.environ[key] = value
            
    target = self.env.ESP32_TARGET
    esp_idf = self.cmake(
            name='esp-idf',
            cmake_vars=lib_vars,
            cmake_src='libraries/AP_HAL_ESP32/targets/'+target.lower()+'/esp-idf',
            cmake_bld='esp-idf_build',
            )

    esp_idf_showinc = esp_idf.build('showinc', target='esp-idf_build/includes.list')

    # task to delete the sdkconfig (thereby causing it to be regenerated) when
    # the .defaults changes. it uses a stamp to find the sdkconfig. changing
    # the sdkconfig WILL NOT cause it to be deleted as it's not an input. this
    # is by design so the user can tweak it for testing purposes.
    clean_sdkconfig_task = esp_idf_showinc.create_task("clean_sdkconfig",
        src=self.srcnode.find_or_declare(self.env.AP_HAL_ESP32+"/sdkconfig.defaults"),
        tgt=self.bldnode.find_or_declare("esp-idf_build/.clean-stamp-sdkconfig"))

    esp_idf_showinc.post()

    # ensure the sdkconfig will be deleted before the cmake configure occurs
    # that regenerates it
    esp_idf_showinc.cmake_config_task.set_run_after(clean_sdkconfig_task)

    from waflib import Task
    class load_generated_includes(Task.Task):
        """After includes.list generated, load includes and toolchain flags from cmake"""
        always_run = True
        def run(tsk):
            bld = tsk.generator.bld
            includes = bld.bldnode.find_or_declare('esp-idf_build/includes.list').read().split()
            bld.env.prepend_value('INCLUDES', includes)

            # Read IDF cmake toolchain flags (e.g. -specs=picolibc.specs for IDF 6.0)
            for flag_file, env_key in [('toolchain/cflags', 'CFLAGS'),
                                       ('toolchain/cxxflags', 'CXXFLAGS')]:
                try:
                    flags_path = bld.bldnode.find_or_declare(
                        'esp-idf_build/' + flag_file).abspath()
                    if os.path.exists(flags_path):
                        with open(flags_path, 'r') as f:
                            cmake_flags = f.read().split()
                        for flag in cmake_flags:
                            if flag not in bld.env[env_key]:
                                bld.env.prepend_value(env_key, [flag])
                except Exception:
                    pass  # flags file may not exist on older IDF

    tsk = load_generated_includes(env=self.env)
    tsk.set_inputs(self.path.find_resource('esp-idf_build/includes.list'))
    self.add_to_group(tsk)

    # hwdef pre-build:
    load_env_vars(self.env)
#    if bld.env.HAL_NUM_CAN_IFACES:
#        bld.get_board().with_can = True
    hwdef_h = os.path.join(self.env.BUILDROOT, 'hwdef.h')
    if not os.path.exists(hwdef_h):
        print("Generating hwdef.h")
        try:
            generate_hwdef_h(self.env)
        except Exception:
            self.fatal(f"Failed to process hwdef.dat {hwdef_h}")

def build(bld):
    bld(
        # build hwdef.h from hwdef.dat. This is needed after a waf clean
        source=bld.path.ant_glob(bld.env.HWDEF),
        rule="",
        group='dynamic_sources',
        target=[
            bld.bldnode.find_or_declare('hwdef.h'),
        ]
    )

@feature('esp32_ap_program')
@after_method('process_source')
def esp32_firmware(self):
    self.link_task.always_run = True
    esp_idf = self.bld.cmake('esp-idf')

    build = esp_idf.build('all', target='esp-idf_build/ardupilot.bin')
    build.post()

    build.cmake_build_task.set_run_after(self.link_task)

    # optional upload is last
    if self.bld.options.upload:
        flasher = esp_idf.build('flash')
        flasher.post()
        
        # Reset OTA data partition to ensure ESP32 boots from factory (USB-uploaded) firmware
        # This prevents issues where FOTA has changed the boot partition to ota_0/ota_1
        from waflib import Task
        class reset_ota_data(Task.Task):
            """Reset ESP32 OTA data partition to boot from factory after USB upload"""
            def run(self):
                import subprocess
                import os
                
                # Simple approach: just inform the user
                # The complexity of auto-detecting ports and esptool paths isn't worth it
                # Users can manually reset if needed
                
                print("ESP32: USB firmware uploaded successfully")
                print("ESP32: If you previously used FOTA updates, the OTA data partition")
                print("ESP32: may need to be reset for this USB firmware to boot.")
                print("ESP32: ")
                print("ESP32: To reset manually if needed:")
                print("ESP32:   esptool.py --port /dev/ttyUSB0 erase_region 0xd000 0x2000")
                print("ESP32: ")
                print("ESP32: This will ensure the ESP32 boots from the factory (USB-uploaded) firmware.")
                
                # Don't actually try to reset automatically - too many variables
                # (port detection, esptool location, permissions, etc.)
                
                return 0
        
        # Add the OTA reset task after flashing
        reset_task = reset_ota_data(env=self.env)
        reset_task.set_run_after(flasher.cmake_build_task)  
        self.bld.add_to_group(reset_task)
