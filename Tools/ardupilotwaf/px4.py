#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for PX4 build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature

import os
import shutil
import sys

_dynamic_env_data = {}
def _load_dynamic_env_data(bld):
    bldnode = bld.bldnode.make_node('modules/PX4Firmware')
    for name in ('cxx_flags', 'include_dirs', 'definitions'):
        _dynamic_env_data[name] = bldnode.find_node(name).read().split(';')

@feature('px4_ap_library', 'px4_ap_program')
@before_method('process_source')
def px4_dynamic_env(self):
    # The generated files from configuration possibly don't exist if it's just
    # a list command (TODO: figure out a better way to address that).
    if self.bld.cmd == 'list':
        return

    if not _dynamic_env_data:
        _load_dynamic_env_data(self.bld)

    self.env.append_value('INCLUDES', _dynamic_env_data['include_dirs'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['cxx_flags'])
    self.env.prepend_value('CXXFLAGS', _dynamic_env_data['definitions'])

# Single static library
# NOTE: This only works only for local static libraries dependencies - fake
# libraries aren't supported yet
@feature('px4_ap_program')
@after_method('apply_link')
@before_method('process_use')
def px4_import_objects_from_use(self):
    queue = list(Utils.to_list(getattr(self, 'use', [])))
    names = set()

    while queue:
        name = queue.pop(0)
        if name in names:
            continue
        names.add(name)

        try:
            tg = self.bld.get_tgen_by_name(name)
        except Errors.WafError:
            continue

        tg.post()
        for t in getattr(tg, 'compiled_tasks', []):
            self.link_task.set_inputs(t.outputs)

        queue.extend(Utils.to_list(getattr(tg, 'use', [])))

class px4_copy(Task.Task):
    color = 'CYAN'

    def run(self):
        shutil.copy2(self.inputs[0].abspath(), self.outputs[0].abspath())

    def keyword(self):
        return "PX4: Copying %s to" % self.inputs[0].name

    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class create_abin(Task.Task):
    '''create an abin file - used for uploading to a skyviper'''
    color = 'CYAN'

    def run(self):
        from subprocess import call
        elf_name = self.inputs[0].abspath()
        bin_name = elf_name + ".bin"
        abin_name = self.outputs[0].abspath()
        call(['arm-none-eabi-objcopy', '-Obinary', elf_name, bin_name])
        call(['Tools/scripts/make_abin.sh', bin_name, abin_name])

    def keyword(self):
        return "Creating abin for %s to" % self.inputs[0].name

    def __str__(self):
        return self.outputs[0].path_from(self.generator.bld.bldnode)

class px4_add_git_hashes(Task.Task):
    run_str = '${PYTHON} ${PX4_ADD_GIT_HASHES} --ardupilot ${PX4_APM_ROOT} --px4 ${PX4_ROOT} --nuttx ${PX4_NUTTX_ROOT} ${SRC} ${TGT}'
    color = 'CYAN'

    def keyword(self):
        return "PX4: Copying firmware and adding git hashes"

    def __str__(self):
        return self.outputs[0].path_from(self.outputs[0].ctx.launch_node())

def _update_firmware_sig(fw_task, firmware, elf):
    original_post_run = fw_task.post_run
    def post_run():
        original_post_run()
        firmware.sig = firmware.cache_sig = Utils.h_file(firmware.abspath())
        elf.sig = elf.cache_sig = Utils.h_file(elf.abspath())
    fw_task.post_run = post_run

_cp_px4io = None
_firmware_semaphorish_tasks = []
_upload_task = []

@feature('px4_ap_program')
@after_method('process_source')
def px4_firmware(self):
    global _cp_px4io, _firmware_semaphorish_tasks, _upload_task
    board_name = self.env.get_flat('PX4_BOARD_NAME')

    px4 = self.bld.cmake('px4')
    px4.vars['APM_PROGRAM_LIB'] = self.link_task.outputs[0].abspath()

    if self.env.PX4_PX4IO_NAME and not _cp_px4io:
        px4io_task = self.create_cmake_build_task('px4', 'fw_io')
        px4io = px4io_task.cmake.bldnode.make_node(
            'src/modules/px4iofirmware/%s.bin' % self.env.PX4_PX4IO_NAME,
        )
        px4io_elf = px4.bldnode.make_node(
            'src/modules/px4iofirmware/%s' % self.env.PX4_PX4IO_NAME
        )
        px4io_task.set_outputs([px4io, px4io_elf])

        romfs = self.bld.bldnode.make_node(self.env.PX4_ROMFS_BLD)
        romfs_px4io = romfs.make_node('px4io/px4io.bin')
        romfs_px4io.parent.mkdir()
        _cp_px4io = self.create_task('px4_copy', px4io, romfs_px4io)
        _cp_px4io.keyword = lambda: 'PX4: Copying PX4IO to ROMFS'

        px4io_elf_dest = self.bld.bldnode.make_node(self.env.PX4IO_ELF_DEST)
        cp_px4io_elf = self.create_task('px4_copy', px4io_elf, px4io_elf_dest)

    fw_task = self.create_cmake_build_task(
        'px4',
        'build_firmware_%s' % board_name,
    )
    fw_task.set_run_after(self.link_task)

    # we need to synchronize in order to avoid the output expected by the
    # previous ap_program being overwritten before used
    for t in _firmware_semaphorish_tasks:
        fw_task.set_run_after(t)
    _firmware_semaphorish_tasks = []

    if self.env.PX4_PX4IO_NAME and _cp_px4io.generator is self:
        fw_task.set_run_after(_cp_px4io)

    firmware = px4.bldnode.make_node(
        'src/firmware/nuttx/nuttx-%s-apm.px4' % board_name,
    )
    fw_elf = px4.bldnode.make_node(
        'src/firmware/nuttx/firmware_nuttx',
    )
    _update_firmware_sig(fw_task, firmware, fw_elf)

    fw_dest = self.bld.bldnode.make_node(
        os.path.join(self.program_dir, '%s.px4' % self.program_name)
    )
    git_hashes = self.create_task('px4_add_git_hashes', firmware, fw_dest)
    git_hashes.set_run_after(fw_task)
    _firmware_semaphorish_tasks.append(git_hashes)

    fw_elf_dest = self.bld.bldnode.make_node(
        os.path.join(self.program_dir, self.program_name)
    )
    cp_elf = self.create_task('px4_copy', fw_elf, fw_elf_dest)
    cp_elf.set_run_after(fw_task)
    _firmware_semaphorish_tasks.append(cp_elf)

    if self.env.BUILD_ABIN:
        fw_abin_dest = self.bld.bldnode.make_node(os.path.join(self.program_dir, self.program_name + '.abin'))
        create_abin = self.create_task('create_abin', fw_elf, fw_abin_dest)
        create_abin.set_run_after(fw_task)

    self.build_summary = dict(
        target=self.name,
        binary=fw_elf_dest.path_from(self.bld.bldnode),
    )

    if self.bld.options.upload:
        if _upload_task:
            Logs.warn('PX4: upload for %s ignored' % self.name)
            return
        _upload_task = self.create_cmake_build_task('px4', 'upload')
        _upload_task.set_run_after(fw_task)
        _firmware_semaphorish_tasks.append(_upload_task)

def _px4_taskgen(bld, **kw):
    if 'cls_keyword' in kw and not callable(kw['cls_keyword']):
        cls_keyword = str(kw['cls_keyword'])
        kw['cls_keyword'] = lambda tsk: 'PX4: ' + cls_keyword

    if 'cls_str' in kw and not callable(kw['cls_str']):
        cls_str = str(kw['cls_str'])
        kw['cls_str'] = lambda tsk: cls_str

    kw['color'] = 'CYAN'

    return bld(**kw)

@feature('_px4_romfs')
def _process_romfs(self):
    bld = self.bld

    board_name = bld.env.get_flat('PX4_BOARD_NAME')

    file_list = [
        'init.d/rc.APM',
        'init.d/rc.error',
        (bld.env.PX4_RC_S_SCRIPT, 'init.d/rcS'),
        'tones/startup'
    ]
    if board_name != "px4fmu-v2":
        # we omit the bootloader on px4-v2 to save flash space
        file_list.append((bld.env.PX4_BOOTLOADER, 'bootloader/fmu_bl.bin'))

    if bld.env.PX4_BOARD_RC:
        board_rc = 'init.d/rc.%s' % bld.env.get_flat('PX4_BOARD_NAME')
        file_list.append((board_rc, 'init.d/rc.board'))

    if bld.env.PX4_PARAM_DEFAULTS:
        file_list.append((bld.env.PX4_PARAM_DEFAULTS, 'defaults.parm'))

    romfs_src = bld.srcnode.find_dir(bld.env.PX4_ROMFS_SRC)
    romfs_bld = bld.bldnode.make_node(bld.env.PX4_ROMFS_BLD)

    for item in file_list:
        if isinstance(item, str):
            src = romfs_src.make_node(item)
            dst = romfs_bld.make_node(item)
        else:
            src = romfs_src.make_node(item[0])
            dst = romfs_bld.make_node(item[1])

        bname = os.path.basename(str(src))
        if bname in bld.env.ROMFS_EXCLUDE:
            print("Excluding %s" % bname)
            continue

        dst.parent.mkdir()
        self.create_task('px4_copy', src, dst)

def configure(cfg):
    cfg.env.CMAKE_MIN_VERSION = '3.2'
    cfg.load('cmake')

    bldnode = cfg.bldnode.make_node(cfg.variant)
    env = cfg.env

    env.AP_PROGRAM_FEATURES += ['px4_ap_program']

    kw = env.AP_LIBRARIES_OBJECTS_KW
    kw['features'] = Utils.to_list(kw.get('features', [])) + ['px4_ap_library']

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()

    board_name = env.get_flat('PX4_BOARD_NAME')

    # TODO: we should move stuff from mk/PX4 to Tools/ardupilotwaf/px4 after
    # stop using the make-based build system
    env.PX4_ROMFS_SRC = 'mk/PX4/ROMFS'
    env.PX4_ROMFS_BLD = 'px4-extra-files/ROMFS'
    env.PX4_BOOTLOADER = '/../../../Tools/bootloaders/%s' % env.PX4_BOOTLOADER_NAME

    env.PX4_ADD_GIT_HASHES = srcpath('Tools/scripts/add_git_hashes.py')
    env.PX4_APM_ROOT = srcpath('')
    env.PX4_ROOT = srcpath('modules/PX4Firmware')
    env.PX4_NUTTX_ROOT = srcpath('modules/PX4NuttX')

    if env.PX4_PX4IO_NAME:
        env.PX4IO_ELF_DEST = 'px4-extra-files/px4io'

    nuttx_config='nuttx_%s_apm' % board_name
        
    env.PX4_CMAKE_VARS = dict(
        CONFIG=nuttx_config,
        CMAKE_MODULE_PATH=srcpath('Tools/ardupilotwaf/px4/cmake'),
        NUTTX_SRC=env.PX4_NUTTX_ROOT,
        PX4_NUTTX_ROMFS=bldpath(env.PX4_ROMFS_BLD),
        ARDUPILOT_BUILD='YES',
        EXTRA_CXX_FLAGS=' '.join((
            # NOTE: these "-Wno-error=*" flags should be removed as we update
            # the submodule
            '-Wno-error=double-promotion',
            '-Wno-error=reorder',
            # NOTE: *Temporarily* using this definition so that both
            # PX4Firmware build systems (cmake and legacy make-based) can live
            # together
            '-DCMAKE_BUILD',
            '-DARDUPILOT_BUILD',
            '-I%s' % bldpath('libraries/GCS_MAVLink'),
            '-I%s' % bldpath('libraries/GCS_MAVLink/include/mavlink'),
            '-Wl,--gc-sections',
        )),
        EXTRA_C_FLAGS=' '.join((
            # NOTE: *Temporarily* using this definition so that both
            # PX4Firmware build systems (cmake and legacy make-based) can live
            # together
            '-DCMAKE_BUILD',
        )),
    )

def build(bld):
    board_name = bld.env.get_flat('PX4_BOARD_NAME')
    px4 = bld.cmake(
        name='px4',
        cmake_src=bld.srcnode.find_dir('modules/PX4Firmware'),
        cmake_vars=bld.env.PX4_CMAKE_VARS,
        cmake_flags=['-Wno-deprecated'],
    )

    px4.build(
        'msg_gen',
        group='dynamic_sources',
        cmake_output_patterns='src/modules/uORB/topics/*.h',
    )
    px4.build(
        'prebuild_targets',
        group='dynamic_sources',
        cmake_output_patterns='%s/NuttX/nuttx-export/**/*.h' % board_name,
    )

    bld(
        name='px4_romfs_static_files',
        group='dynamic_sources',
        features='_px4_romfs',
    )

    bld.extra_build_summary = _extra_build_summary

def _extra_build_summary(bld, build_summary):
    build_summary.text('')
    build_summary.text('PX4')
    build_summary.text('', '''
The ELF files are pointed by the path in the "%s" column. The .px4 files are in
the same directory of their corresponding ELF files.
''' % build_summary.header_text['target'])

    if not bld.options.upload:
        build_summary.text('')
        build_summary.text('', '''
You can use the option --upload to upload the firmware to the PX4 board if you
have one connected.''')

    if bld.env.PX4_PX4IO_NAME:
        build_summary.text('')
        build_summary.text('PX4IO')
        summary_data_list = bld.size_summary([bld.env.PX4IO_ELF_DEST])
        header = bld.env.BUILD_SUMMARY_HEADER[:]
        try:
            header.remove('target')
        except ValueError:
            pass
        header.insert(0, 'binary_path')
        build_summary.print_table(summary_data_list, header)
