# encoding: utf-8

"""
Waf tool for Zephyr build orchestration.

AP_FLAKE8_CLEAN
"""

import base64
import json
import os
import re
import shutil
import subprocess
import sys
import zlib

from collections import OrderedDict

from waflib import Errors
from waflib import Logs
from waflib import Task
from waflib.TaskGen import after_method
from waflib.TaskGen import feature


def _resolve_zephyr_base(env):
    env_zephyr_base = os.environ.get('ZEPHYR_BASE')
    if env_zephyr_base:
        return env_zephyr_base

    # Stage 1 decision: prefer repository-managed Zephyr submodule.
    module_zephyr = os.path.join(env.SRCROOT, 'modules', 'zephyr')
    if os.path.isdir(module_zephyr):
        return module_zephyr

    return ''


def _resolve_zephyr_sdk_install_dir():
    env_zephyr_sdk = os.environ.get('ZEPHYR_SDK_INSTALL_DIR')
    if env_zephyr_sdk:
        return env_zephyr_sdk

    default_sdk = os.path.expanduser('~/zephyr-sdk-1.0.1')
    if os.path.isdir(default_sdk):
        return default_sdk

    return ''


def _walk_many(roots):
    """os.walk over several roots in sequence (module discovery helper)."""
    for r in roots:
        for entry in os.walk(r):
            yield entry


def _discover_zephyr_modules(zephyr_base):
    """Return (modules_str, cmake_vars) where modules_str is a semicolon-separated
    list of module paths and cmake_vars is a dict of ZEPHYR_<NAME>_{KCONFIG,CMAKE_DIR}
    vars for modules not auto-discovered by modules/modules.cmake (which only globs
    one level deep, missing modules nested under e.g. modules/hal/espressif/)."""
    import yaml
    modules_root = os.path.join(zephyr_base, 'modules')
    found = []
    cmake_vars = {}
    if not os.path.isdir(modules_root):
        return '', {}
    # bootloader/mcuboot lives outside modules/ but is a real Zephyr module,
    # needed for CONFIG_MCUBOOT_BOOTUTIL_LIB. Register EXACTLY its top-level
    # zephyr/module.yml: a deep walk also matches a sample whose hooks.c breaks it.
    walk_roots = [modules_root]
    mcuboot_zephyr = os.path.join(zephyr_base, 'bootloader', 'mcuboot', 'zephyr')
    if os.path.isfile(os.path.join(mcuboot_zephyr, 'module.yml')):
        walk_roots.append(os.path.dirname(mcuboot_zephyr))
    for root, dirs, files in _walk_many(walk_roots):
        # only the top-level module.yml of an explicitly-added extra root
        if root.startswith(os.path.join(zephyr_base, 'bootloader')) and \
                os.path.normpath(root) != os.path.normpath(mcuboot_zephyr):
            continue
        if 'module.yml' in files and os.path.basename(root) == 'zephyr':
            module_dir = os.path.dirname(root)
            found.append(module_dir)
            try:
                with open(os.path.join(root, 'module.yml'), 'r') as f:
                    meta = yaml.safe_load(f)
                build = meta.get('build', {}) if meta else {}
                name = (meta.get('name') or os.path.basename(module_dir)).upper().replace('-', '_')
                # kconfig-ext: the module's Kconfig sits where modules.cmake will not
                # find it. Always set the var even when the file is absent - unset
                # points Zephyr at a directory and osource then fails with EISDIR.
                if build.get('kconfig-ext'):
                    cmake_vars['ZEPHYR_{}_KCONFIG'.format(name)] = os.path.join(root, 'Kconfig')
                # cmake-ext: CMakeLists.txt lives in zephyr/ — set dir explicitly.
                # cmake: <rel>: CMakeLists.txt is at module_dir/<rel> — also
                #   needs an explicit CMAKE_DIR because modules/modules.cmake
                #   only globs one level deep and misses nested modules like
                #   modules/hal/stm32/, modules/hal/nxp/, etc.
                if build.get('cmake-ext'):
                    cmake_file = os.path.join(root, 'CMakeLists.txt')
                    if os.path.isfile(cmake_file):
                        cmake_vars['ZEPHYR_{}_CMAKE_DIR'.format(name)] = root
                elif 'cmake' in build:
                    cmake_rel = build['cmake']
                    cmake_dir = os.path.normpath(os.path.join(module_dir, cmake_rel))
                    if os.path.isfile(os.path.join(cmake_dir, 'CMakeLists.txt')):
                        cmake_vars['ZEPHYR_{}_CMAKE_DIR'.format(name)] = cmake_dir
            except Exception:  # noqa: BLE001
                pass
        # Don't recurse into test fixtures
        dirs[:] = [d for d in dirs if d not in ('tests',)]
    return ';'.join(found), cmake_vars


def _zephyr_build_dir_is_incomplete(build_dir):
    cache_path = os.path.join(build_dir, 'CMakeCache.txt')
    ninja_path = os.path.join(build_dir, 'build.ninja')

    if not os.path.exists(cache_path):
        return False

    if not os.path.exists(ninja_path):
        return True

    try:
        with open(cache_path, 'r', encoding='utf-8') as cache_file:
            return 'CMAKE_PROJECT_NAME:' not in cache_file.read()
    except OSError:
        return True


def _reset_incomplete_zephyr_build_dir(build_dir):
    if not _zephyr_build_dir_is_incomplete(build_dir):
        return

    Logs.warn('Zephyr: detected incomplete CMake state in {}. Forcing reconfigure.'.format(build_dir))
    shutil.rmtree(build_dir, ignore_errors=True)


def _discover_zephyr_conf_fragments(env):
    """Return ordered list of Kconfig fragment paths to merge.

    Four-layer discovery (each layer applied on top of the previous):
      1. prj.conf                — shared base for all Zephyr boards
      2. prj.<mfr>.conf         — manufacturer layer (e.g. prj.stm32.conf)
      3. prj.<soc>.conf         — SoC-family layer   (e.g. prj.esp32s3.conf)
      4. prj.<BOARD>.conf  or
         boards/<BOARD>.conf  or
         boards/<BOARD>_*.conf  — board-specific overlay
    """
    app_dir = env.ZEPHYR_APP_DIR
    fragments = []
    board_dir = os.path.join(app_dir, 'boards')

    def _add(candidate):
        if os.path.isfile(candidate) and candidate not in fragments:
            fragments.append(candidate)

    # Layer 1 — base
    _add(os.path.join(app_dir, 'prj.conf'))

    # Layer 2 — manufacturer (e.g. stm32, espressif, nxp)
    mfr = getattr(env, 'ZEPHYR_MFR', None)
    if mfr:
        _add(os.path.join(app_dir, 'prj.{}.conf'.format(mfr)))

    # Layer 3 — SoC family (e.g. stm32h7, esp32s3, imxrt11xx)
    soc = getattr(env, 'ZEPHYR_SOC', None)
    if soc:
        _add(os.path.join(app_dir, 'prj.{}.conf'.format(soc)))

    # Layer 4 — board-specific
    board_tokens = []
    for token in (env.BOARD, env.ZEPHYR_BOARD):
        if token and token not in board_tokens:
            board_tokens.append(token)

    for token in board_tokens:
        _add(os.path.join(app_dir, 'prj.{}.conf'.format(token)))
        _add(os.path.join(board_dir, '{}.conf'.format(token)))

        # Accept board variant fragments, e.g. native_sim_native_64.conf
        if os.path.isdir(board_dir):
            prefix = token + '_'
            for name in sorted(os.listdir(board_dir)):
                if not name.endswith('.conf'):
                    continue
                if not name.startswith(prefix):
                    continue
                _add(os.path.join(board_dir, name))

    # Layer 5 - bootloader overlay, merged last so it can override board-layer
    # defaults that suit the app but not a bootloader. 5a prj-bl.conf is
    # board-agnostic slimming; 5b prj.<board>-bl.conf is FCB/offset/SoC symbols.
    if getattr(env, 'BOOTLOADER', False):
        _add(os.path.join(app_dir, 'prj-bl.conf'))
        for token in board_tokens:
            _add(os.path.join(app_dir, 'prj.{}-bl.conf'.format(token)))

    # Layer 6 — opt-in feature overlays requested at configure time, e.g.
    # thread_stats.conf from `./waf configure --enable-stats`. Merged last so
    # an explicitly requested feature wins over any board default.
    for name in getattr(env, 'ZEPHYR_EXTRA_CONF_FRAGMENTS', []) or []:
        _add(os.path.join(app_dir, name))

    return fragments


def _write_if_changed(path, content):
    '''mtime-preserving write (see the ccache rationale in
    _write_autogen_prj_conf): only touch the file when the text differs.'''
    existing = None
    if os.path.exists(path):
        with open(path, 'r', encoding='utf-8') as fh:
            existing = fh.read()
    if content != existing:
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(content)


def _run_class_generator(env, fragments):
    '''Class generator (zephyr_class_generator.py): emit the hwdef-derived
    DTS overlay + Kconfig fragment into the build dir. INERT when the
    board's hwdef.dat has no PIN/DMA directives - the overlay is
    header-only and the conf fragment adds no symbols (merge-aware
    against the discovered fragments), keeping legacy boards
    byte-identical. Returns (overlay_path, conf_path).'''
    import zephyr_class_generator
    import zephyr_hwdef

    out_dir = os.path.join(env.BUILDROOT, 'zephyr_build')
    os.makedirs(out_dir, exist_ok=True)
    overlay_path = os.path.join(out_dir, 'hwdef_autogen.overlay')
    conf_path = os.path.join(out_dir, 'hwdef_autogen.conf')

    board = env.get_flat('BOARD')
    name = 'hwdef-bl.dat' if env.BOOTLOADER else 'hwdef.dat'
    hwdef_path = os.path.join(env.SRCROOT, 'libraries', 'AP_HAL_Zephyr',
                              'hwdef', board, name)
    if not os.path.isfile(hwdef_path):
        _write_if_changed(overlay_path, '/* no hwdef for %s */\n' % board)
        _write_if_changed(conf_path, '# no hwdef for %s\n' % board)
        return overlay_path, conf_path

    hwdef = zephyr_hwdef.ZephyrHWDef(hwdef_path, is_bootloader=bool(env.BOOTLOADER))
    gen = zephyr_class_generator.ZephyrClassGenerator(hwdef, env.ZEPHYR_BASE)

    # merge-awareness: symbols already assigned by any discovered fragment
    assigned = set()
    sym_re = re.compile(r'^(CONFIG_\w+)=', re.M)
    for fragment in fragments:
        with open(fragment, 'r', encoding='utf-8') as fh:
            assigned.update(sym_re.findall(fh.read()))

    _write_if_changed(overlay_path, gen.overlay_text())
    _write_if_changed(conf_path, gen.conf_text(assigned))

    if gen.errors:
        raise Errors.WafError('zephyr_class_generator: %s' % '; '.join(gen.errors))
    return overlay_path, conf_path


def _write_autogen_prj_conf(env):
    out_dir = os.path.join(env.BUILDROOT, 'zephyr_build')
    os.makedirs(out_dir, exist_ok=True)

    fragments = _discover_zephyr_conf_fragments(env)
    # class-generator conf fragment merges LAST (it is merge-aware, only
    # adding symbols nothing earlier set); its overlay path is re-derived
    # cheaply at the cmake step via env (same file, stable path).
    _gen_overlay, gen_conf = _run_class_generator(env, fragments)
    fragments = fragments + [gen_conf]
    output_path = os.path.join(out_dir, 'ardupilot_prj_autogen.conf')

    parts = ['# Auto-generated by Tools/ardupilotwaf/zephyr.py\n',
             '# Do not edit directly. Update source fragments instead.\n\n']
    for fragment in fragments:
        parts.append('# ---- begin {} ----\n'.format(os.path.relpath(fragment, env.SRCROOT)))
        with open(fragment, 'r', encoding='utf-8') as source:
            parts.append(source.read())
        parts.append('\n')
        parts.append('# ---- end {} ----\n\n'.format(os.path.relpath(fragment, env.SRCROOT)))
    output = ''.join(parts)

    # Only write when the content changed. An identical rewrite still bumps mtime,
    # and Zephyr's CMake/Kconfig layer keys off mtime (ninja does; waf hashes), so
    # it re-stamped generated headers and cost ccache its direct hits.
    existing = None
    if os.path.exists(output_path):
        with open(output_path, 'r', encoding='utf-8') as fh:
            existing = fh.read()
    if output != existing:
        with open(output_path, 'w', encoding='utf-8') as fh:
            fh.write(output)

    return output_path, fragments


def configure(cfg):
    env = cfg.env

    bldnode = cfg.bldnode.make_node(cfg.variant)
    env.SRCROOT = cfg.srcnode.make_node('').abspath()
    env.BUILDROOT = bldnode.make_node('').abspath()

    cfg.load('cmake')

    cfg.find_program('cmake', var='CMAKE', mandatory=False)
    cfg.find_program('ninja', var='NINJA', mandatory=False)

    env.AP_PROGRAM_FEATURES += ['zephyr_ap_program']

    env.ZEPHYR_APP_DIR = cfg.srcnode.make_node('libraries/AP_HAL_Zephyr/zephyr').abspath()
    env.ZEPHYR_BUILD_DIR = os.path.join(env.BUILDROOT, 'zephyr_build')
    env.ZEPHYR_LIB = os.path.join(env.ZEPHYR_BUILD_DIR, 'zephyr', 'libzephyr.a')
    env.ZEPHYR_BASE = _resolve_zephyr_base(env)
    env.ZEPHYR_BOARD = env.BOARD if env.BOARD else 'native_posix'

    cfg.msg('Zephyr cmake tool', env.get_flat('CMAKE') if env.CMAKE else 'not found',
            color='GREEN' if env.CMAKE else 'YELLOW')
    cfg.msg('Zephyr ninja tool', env.get_flat('NINJA') if env.NINJA else 'not found',
            color='GREEN' if env.NINJA else 'YELLOW')
    cfg.msg('Zephyr base', env.ZEPHYR_BASE if env.ZEPHYR_BASE else 'not found',
            color='GREEN' if env.ZEPHYR_BASE else 'YELLOW')


def pre_build(bld):
    # Zephyr cmake configure and include injection are now proper Waf tasks
    # created in build() under the dynamic_sources group — nothing to do here.
    pass


def _dtc_overlays(env, cmake_src):
    '''Devicetree overlays for this build, in the order cmake must see them.

    BOTH cmake configures have to pass the identical list. The early one
    (dynamic_sources group) produces the devicetree_generated.h that every
    ArduPilot object compiles against; the final-link one produces the
    devicetree the image is linked against. If only the second carries the
    overlay, the two devicetrees have different node ordinals and the link
    ends in undefined __device_dts_ord_NNN - one per node the overlay added.
    That stayed hidden while the generator emitted a single peripheral and
    became fatal at thirty-three.'''
    overlays = []
    # class-generator overlay: always present (header-only when the board has
    # no PIN/DMA directives - the inert default).
    gen_overlay = os.path.join(env.get_flat('BUILDROOT'),
                               'zephyr_build', 'hwdef_autogen.overlay')
    if os.path.isfile(gen_overlay):
        overlays.append(gen_overlay)
    if env.BOOTLOADER:
        cand = os.path.join(cmake_src, 'boards',
                            '%s-bl.overlay' % env.get_flat('BOARD'))
        if os.path.isfile(cand):
            overlays.append(cand)
    return overlays


def build(bld):
    if bld.env.BOARD_CLASS != 'Zephyr':
        return

    # Re-resolve at build time so a post-configure prerequisites install
    # (that creates modules/zephyr) is picked up without forcing a manual
    # reconfigure step.
    if not bld.env.ZEPHYR_BASE:
        bld.env.ZEPHYR_BASE = _resolve_zephyr_base(bld.env)

    if not bld.env.CMAKE:
        Logs.warn('Zephyr: cmake not available, skipping Zephyr side build')
        return

    if not bld.env.ZEPHYR_BASE:
        Logs.warn(
            'Zephyr: ZEPHYR_BASE/modules/zephyr not found, skipping Zephyr side build. '
            'Run ./Tools/scripts/zephyr_get_prerequisites.sh then ./waf configure --board <board>'
        )
        return

    cmakelists = os.path.join(bld.env.ZEPHYR_APP_DIR, 'CMakeLists.txt')
    if not os.path.isfile(cmakelists):
        Logs.warn('Zephyr: missing {} - skipping Zephyr side build'.format(cmakelists))
        return

    _reset_incomplete_zephyr_build_dir(bld.env.ZEPHYR_BUILD_DIR)

    conf_file, fragments = _write_autogen_prj_conf(bld.env)
    if fragments:
        Logs.info('Zephyr: using config fragments: {}'.format(', '.join(
            os.path.relpath(path, bld.env.SRCROOT) for path in fragments
        )))
    else:
        Logs.warn('Zephyr: no config fragments found; generated empty CONF_FILE')

    zephyr_vars = OrderedDict()
    zephyr_vars['ZEPHYR_BASE'] = bld.env.ZEPHYR_BASE
    zephyr_vars['BOARD'] = bld.env.ZEPHYR_BOARD
    zephyr_vars['CONF_FILE'] = conf_file
    # The early configure MUST see the same overlays as the final link, or the
    # two devicetrees disagree on node ordinals and every ArduPilot object is
    # compiled against the wrong one. See _dtc_overlays().
    # same expression the final-link configure uses, so the two cannot drift
    _cmake_src = bld.srcnode.make_node('libraries/AP_HAL_Zephyr/zephyr').abspath()
    zephyr_vars['EXTRA_DTC_OVERLAY_FILE'] = ';'.join(
        _dtc_overlays(bld.env, _cmake_src))
    zephyr_modules, kconfig_vars = _discover_zephyr_modules(bld.env.ZEPHYR_BASE)
    if zephyr_modules:
        zephyr_vars['ZEPHYR_MODULES'] = zephyr_modules
    zephyr_vars.update(kconfig_vars)

    if bld.env.ZEPHYR_BOARD.startswith('native_sim'):
        zephyr_vars['ZEPHYR_TOOLCHAIN_VARIANT'] = 'host'
    else:
        zephyr_sdk_dir = _resolve_zephyr_sdk_install_dir()
        if zephyr_sdk_dir:
            zephyr_vars['ZEPHYR_TOOLCHAIN_VARIANT'] = 'zephyr'
            zephyr_vars['ZEPHYR_SDK_INSTALL_DIR'] = zephyr_sdk_dir
            Logs.info('Zephyr: using Zephyr SDK at {}'.format(zephyr_sdk_dir))
        else:
            zephyr_vars['ZEPHYR_TOOLCHAIN_VARIANT'] = 'gnuarmemb'
            # Prefer binutils path to avoid ccache wrapper ambiguity on gcc.
            gnuarmemb_bin = shutil.which('arm-none-eabi-objcopy') or shutil.which('arm-none-eabi-gcc')
            if gnuarmemb_bin:
                gnuarmemb_bin = os.path.realpath(gnuarmemb_bin)
                # Zephyr expects GNUARMEMB_TOOLCHAIN_PATH as the toolchain root.
                zephyr_vars['GNUARMEMB_TOOLCHAIN_PATH'] = os.path.dirname(os.path.dirname(gnuarmemb_bin))

    # ARDUPILOT_LIB / BIN / CMD are intentionally NOT set here.  The initial
    # cmake configure (used for showinc and libzephyr.a stub) must not see the
    # AP lib paths — they don't exist yet.  _cmake_final_link reconfigures with
    # them after the AP static libraries are built by the waf link step.

    # dynamic_sources group: cmake configure + showinc + include injection. Runs
    # before 'build' so AP task generators are post()'ed with the Zephyr include
    # paths already in bld.env. Same pattern as Tools/ardupilotwaf/esp32.py.
    bld.set_group('dynamic_sources')

    zephyr = bld.cmake(
        name='zephyr',
        cmake_vars=zephyr_vars,
        cmake_src='libraries/AP_HAL_Zephyr/zephyr',
        cmake_bld='zephyr_build',
    )

    # showinc writes includes.list; cmake_build_task.always_run=True but Waf
    # only invalidates downstream tasks when the output file content changes.
    zephyr_showinc = zephyr.build('showinc', target='zephyr_build/includes.list')
    zephyr_showinc.post()

    # showdefs writes compile_defs.list (INTERFACE_COMPILE_DEFINITIONS from
    # zephyr_interface — includes e.g. -DSTM32H743xx on STM32H7 boards).
    zephyr_showdefs = zephyr.build('showdefs', target='zephyr_build/compile_defs.list')
    zephyr_showdefs.post()

    # Task: read includes.list and prepend paths to bld.env.INCLUDES so that
    # ArduPilot task generators post()'ed in the 'build' group inherit them.
    class load_zephyr_includes(Task.Task):
        color = 'CYAN'
        always_run = True

        def run(tsk):
            node = bld.bldnode.find_or_declare('zephyr_build/includes.list')
            if not node.exists():
                return 0
            raw = node.read().strip()
            if ';' in raw:
                paths = [p.strip() for p in raw.split(';') if p.strip() and os.path.isdir(p.strip())]
            else:
                paths = [p.strip() for p in raw.split() if p.strip() and os.path.isdir(p.strip())]
            if paths:
                Logs.info('Zephyr: injecting %d Zephyr include paths' % len(paths))
                bld.env.prepend_value('INCLUDES', paths)
            # Add hwdef.h build directory to includes so SPIDevice.cpp can #include "hwdef.h"
            hwdef_dir = bld.bldnode.abspath()
            if os.path.isdir(hwdef_dir):
                bld.env.prepend_value('INCLUDES', [hwdef_dir])
                Logs.info('Zephyr: added hwdef directory to includes: %s' % hwdef_dir)
            return 0

    lgi = load_zephyr_includes(env=bld.env)
    includes_node = bld.bldnode.find_or_declare('zephyr_build/includes.list')
    lgi.set_inputs([includes_node])
    lgi.set_run_after(zephyr_showinc.cmake_build_task)
    bld.add_to_group(lgi, 'dynamic_sources')

    # Task: read compile_defs.list and inject Zephyr interface defines into
    # ArduPilot compilation (e.g. -DSTM32H743xx on STM32H7, -DCORE_CM7, etc.)
    class load_zephyr_defines(Task.Task):
        color = 'CYAN'
        always_run = True

        def run(tsk):
            node = bld.bldnode.find_or_declare('zephyr_build/compile_defs.list')
            if not node.exists():
                return 0
            raw = node.read().strip()
            if not raw:
                return 0
            # showdefs writes space-separated tokens on one line. These prefixes are
            # Zephyr/POSIX/libc internal and must NOT be injected into the AP compile:
            # they change system-header behaviour (_POSIX_C_SOURCE makes errno TLS).
            _SKIP_PREFIXES = (
                'KERNEL', '__ZEPHYR__', '__LINUX', 'PICOLIBC', 'ZVFS',
                'K_HEAP', 'K_', '_POSIX', '__PROGRAM',
            )
            defs = []
            for item in raw.split():
                item = item.strip()
                if not item:
                    continue
                sym = item.lstrip('-D').split('=')[0]
                if any(sym.startswith(p) for p in _SKIP_PREFIXES):
                    continue
                # Normalise to -DSYMBOL or -DSYMBOL=VAL form
                if not item.startswith('-D'):
                    item = '-D' + item
                defs.append(item)
            if defs:
                Logs.info('Zephyr: injecting %d Zephyr compile definitions' % len(defs))
                bld.env.append_unique('CFLAGS', defs)
                bld.env.append_unique('CXXFLAGS', defs)
            # __ZEPHYR__ is needed so that AP_HAL_Zephyr driver code activates
            # its Zephyr-specific paths (guarded by #ifdef __ZEPHYR__).  It is
            # deliberately injected here rather than from the CMake showdefs
            # list because the CMake list also pulls in _POSIX_C_SOURCE and
            # KERNEL which change picolibc system-header behaviour.
            bld.env.append_unique('CFLAGS',   ['-D__ZEPHYR__'])
            bld.env.append_unique('CXXFLAGS', ['-D__ZEPHYR__'])
            return 0

    lgd = load_zephyr_defines(env=bld.env)
    defs_node = bld.bldnode.find_or_declare('zephyr_build/compile_defs.list')
    lgd.set_inputs([defs_node])
    lgd.set_run_after(zephyr_showdefs.cmake_build_task)
    bld.add_to_group(lgd, 'dynamic_sources')

    # ------------------------------------------------------------------ #
    # build group: zephyr_program @feature handles cmake build 'all' and  #
    # the AP link ordering via create_cmake_build_task.                    #
    # ------------------------------------------------------------------ #
    bld.set_group('build')


class upload_fw_zephyr(Task.Task):
    color = 'BLUE'
    always_run = True

    @staticmethod
    def _parse_hwdef_define(path, name):
        pattern = re.compile(r'^#define\s+{}\s+(.+)$'.format(re.escape(name)))
        try:
            with open(path, 'r', encoding='utf-8') as hwdef_file:
                for line in hwdef_file:
                    match = pattern.match(line.strip())
                    if match:
                        return match.group(1).strip()
        except OSError:
            return None
        return None

    @staticmethod
    def _parse_hwdef_token(path, name):
        pattern = re.compile(r'^\s*{}\s+(.+?)(?:\s+#.*)?$'.format(re.escape(name)))
        try:
            with open(path, 'r', encoding='utf-8') as hwdef_file:
                for line in hwdef_file:
                    stripped = line.strip()
                    if not stripped or stripped.startswith('#'):
                        continue
                    match = pattern.match(line)
                    if match:
                        return match.group(1).strip()
        except OSError:
            return None
        return None

    @staticmethod
    def _resolve_board_id_token(board_id_raw, srcroot):
        if board_id_raw is None:
            return None

        try:
            return int(board_id_raw, 0)
        except ValueError:
            pass

        board_types = os.path.join(srcroot, 'Tools', 'AP_Bootloader', 'board_types.txt')
        try:
            with open(board_types, 'r', encoding='utf-8') as board_types_file:
                for line in board_types_file:
                    line = line.split('#', 1)[0].strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) < 2:
                        continue
                    if parts[0] == board_id_raw:
                        return int(parts[1], 0)
        except OSError:
            return None
        return None

    def _collect_apj_metadata(self):
        buildroot = self.env.get_flat('BUILDROOT')
        srcroot = self.env.get_flat('SRCROOT')
        board = self.env.get_flat('BOARD')

        hwdef_h = os.path.join(buildroot, 'hwdef.h')
        zephyr_hwdef_dir = os.path.join(srcroot, 'libraries', 'AP_HAL_Zephyr', 'hwdef', board)
        hwdef_candidates = [
            os.path.join(zephyr_hwdef_dir, 'hwdef.inc'),
            os.path.join(zephyr_hwdef_dir, 'hwdef.dat'),
            hwdef_h,
        ]

        board_id_raw = None
        board_type = None
        usb_vendor = None
        usb_product = None

        for candidate in hwdef_candidates:
            if board_id_raw is None:
                board_id_raw = self._parse_hwdef_token(candidate, 'APJ_BOARD_ID')
            if board_type is None:
                board_type = self._parse_hwdef_token(candidate, 'APJ_BOARD_TYPE')
            if usb_vendor is None:
                usb_vendor = self._parse_hwdef_token(candidate, 'USB_VENDOR')
            if usb_product is None:
                usb_product = self._parse_hwdef_token(candidate, 'USB_PRODUCT')

        if board_id_raw is None:
            board_id_raw = self._parse_hwdef_define(hwdef_h, 'APJ_BOARD_ID')
        if board_type is None:
            board_type = self._parse_hwdef_define(hwdef_h, 'APJ_BOARD_TYPE')
        if board_type is None:
            board_type = self._parse_hwdef_token(os.path.join(zephyr_hwdef_dir, 'hwdef.inc'), 'BOARD_NAME')

        board_id = self._resolve_board_id_token(board_id_raw, srcroot)
        if board_id is None:
            raise RuntimeError(
                'Zephyr upload requires APJ_BOARD_ID in hwdef and a matching entry in Tools/AP_Bootloader/board_types.txt'
            )

        if board_type is None:
            board_type = self._parse_hwdef_define(hwdef_h, 'BOARD_NAME')
        if board_type is None:
            board_type = board

        usbid = self._parse_hwdef_define(hwdef_h, 'HAL_USB_VENDOR_ID')
        if usbid is None and usb_vendor and usb_product:
            usbid = '{}/{}'.format(usb_vendor, usb_product)
        if usbid is None:
            usbid = '0x0000/0x0000'

        return {
            'board_id': board_id,
            'board_type': board_type.strip('"'),
            'usbid': usbid,
        }

    def _build_apj_from_bin(self, bin_path, apj_path):
        metadata = self._collect_apj_metadata()

        image = open(bin_path, 'rb').read()
        summary = self.env.get_flat('BOARD')
        flash_total = max(len(image), 1024 * 1024)

        desc = {
            'board_id': metadata['board_id'],
            'magic': 'APJFWv1',
            'description': 'Firmware for a {} board'.format(metadata['board_type']),
            'image': base64.b64encode(zlib.compress(image, 9)).decode('utf-8'),
            'summary': summary,
            'version': '0.1',
            'image_size': len(image),
            'flash_total': flash_total,
            'image_maxsize': flash_total,
            'flash_free': max(0, flash_total - len(image)),
            'extflash_total': 0,
            'extflash_free': 0,
            'git_identity': self.generator.bld.git_head_hash(short=True),
            'board_revision': 0,
            'USBID': metadata['usbid'],
        }

        with open(apj_path, 'w', encoding='utf-8') as apj_file:
            apj_file.write(json.dumps(desc, indent=4))

    def _resolve_upload_firmware(self):
        src = self.inputs[0].abspath()
        if src.endswith('.apj'):
            return src

        def _is_arm_elf(path):
            try:
                with open(path, 'rb') as elf:
                    hdr = elf.read(20)
            except OSError:
                return False

            if len(hdr) < 20 or hdr[0:4] != b'\x7fELF':
                return False

            # e_machine at bytes 18..19, endianness from EI_DATA byte 5.
            byteorder = 'big' if hdr[5] == 2 else 'little'
            return int.from_bytes(hdr[18:20], byteorder=byteorder) == 40  # EM_ARM

        bin_source = os.path.join(self.env.get_flat('BUILDROOT'), 'zephyr_upload.bin')
        objcopy = self.env.get_flat('OBJCOPY') or shutil.which('arm-none-eabi-objcopy')
        if not objcopy:
            raise RuntimeError('Zephyr upload requires objcopy to convert ELF to binary')

        # waf link output for tool targets can be a host executable. Only try
        # objcopy on the link output if it is an ARM ELF.
        candidates = []
        if _is_arm_elf(src):
            candidates.append(src)

        zephyr_bin = os.path.join(
            self.env.get_flat('BUILDROOT'),
            'zephyr_build',
            'zephyr',
            'zephyr.bin',
        )

        zephyr_elf = os.path.join(
            self.env.get_flat('BUILDROOT'),
            'zephyr_build',
            'zephyr',
            'zephyr.elf',
        )
        if os.path.exists(zephyr_elf):
            candidates.append(zephyr_elf)
        if os.path.exists(zephyr_bin):
            candidates.append(zephyr_bin)

        converted = False
        for candidate in candidates:
            if candidate.endswith('.bin'):
                shutil.copyfile(candidate, bin_source)
                converted = True
                break

            ret = subprocess.run([
                objcopy,
                '-O', 'binary',
                '--gap-fill', '0xFF',
                candidate,
                bin_source,
            ], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode
            if ret == 0:
                converted = True
                break

        if not converted:
            raise RuntimeError('Failed to convert upload image; no usable Zephyr firmware artifact found')

        self._pad_bin_for_flash_load_offset(bin_source)

        apj_out = os.path.join(self.env.get_flat('BUILDROOT'), 'zephyr_upload.apj')
        self._build_apj_from_bin(bin_source, apj_out)
        return apj_out

    # Bytes into the .bin where the app's vector table must sit so a
    # bootloader-mediated upload lands it at APP_LOAD_ADDRESS+APP_VECTOR_OFFSET.
    # NOT CONFIG_FLASH_LOAD_OFFSET: that double-counts the bootloader region.
    _BOOTLOADER_UPLOAD_PAD_BYTES = {
        'mr_vmu_rt1176': 0x2000,
    }

    def _pad_bin_for_flash_load_offset(self, bin_source):
        """See _BOOTLOADER_UPLOAD_PAD_BYTES for why this pad amount is a
        per-board bootloader constant, not derived from Kconfig."""
        # Never pad a bootloader image. It is booted directly by the BootROM from
        # the flash base and carries its own FCB, so it must start at offset 0 as
        # linked; its first word is the FCB, not a vector table.
        if getattr(self.env, 'BOOTLOADER', False):
            return

        # Same for a direct-flash app: with CONFIG_NXP_IMXRT_BOOT_HEADER=y the
        # image carries its own FCB/IVT and boots from the flash base, so it must
        # not be shifted. Padding applies only to the bootloader-relative case.
        autoconf = os.path.join(
            self.env.get_flat('BUILDROOT'), 'zephyr_build', 'zephyr',
            'include', 'generated', 'zephyr', 'autoconf.h')
        try:
            with open(autoconf, 'r', encoding='utf-8') as f:
                for line in f:
                    if re.match(r'#define\s+CONFIG_NXP_IMXRT_BOOT_HEADER\s+1', line):
                        return
        except OSError:
            pass

        board = self.env.get_flat('BOARD')
        offset = self._BOOTLOADER_UPLOAD_PAD_BYTES.get(board, 0)
        if offset <= 0:
            return

        with open(bin_source, 'rb') as f:
            image = f.read()

        if len(image) < 8:
            raise RuntimeError(
                'Zephyr upload .bin is too small (%d bytes) to contain a '
                'vector table - refusing to pad' % len(image))

        # The unpadded .bin's first word is the app's real initial SP (must
        # point into RAM, i.e. word & 0x20000000 set per ARM Cortex-M SRAM
        # convention on this SoC family) - sanity-check we are padding an
        # image that actually starts with a vector table, not something
        # objcopy produced from an unexpected LOAD segment layout.
        initial_sp = int.from_bytes(image[0:4], byteorder='little')
        if (initial_sp & 0x20000000) == 0:
            raise RuntimeError(
                'Zephyr upload .bin does not start with a plausible vector '
                'table (first word 0x%08x is not a RAM address) - refusing '
                'to pad %d bytes for board %s; the objcopy output layout '
                'may have changed, verify manually before flashing via a '
                'bootloader-mediated path' % (initial_sp, offset, board))

        padded = b'\xFF' * offset + image
        with open(bin_source, 'wb') as f:
            f.write(padded)

        # Verify the write landed exactly where expected: the vector table's
        # initial SP word must now sit at file-offset `offset`.
        with open(bin_source, 'rb') as f:
            check = f.read()
        if len(check) != len(image) + offset:
            raise RuntimeError(
                'Zephyr upload .bin padding size mismatch after write: '
                'expected %d bytes, got %d' % (len(image) + offset, len(check)))
        if check[offset:offset + 4] != image[0:4]:
            raise RuntimeError(
                'Zephyr upload .bin padding verification failed: vector '
                'table not found at file offset 0x%x after padding by '
                '0x%x bytes - upload would silently flash a broken image'
                % (offset, offset))
        if check[0:offset] != b'\xFF' * offset:
            raise RuntimeError(
                'Zephyr upload .bin padding verification failed: leading '
                '%d bytes are not all 0xFF' % offset)

        Logs.info('Zephyr: padded zephyr_upload.bin by 0x%x bytes '
                  '(bootloader APP_VECTOR_OFFSET for board %s) for '
                  'bootloader-mediated upload; vector table verified at '
                  'file offset 0x%x' % (offset, board, offset))

    def _run_ap_final_link(self):
        """Re-run the Zephyr CMake build with ARDUPILOT_LIB set, producing the
        final zephyr.elf with AP code linked in.

        Must run before _resolve_upload_firmware() looks for that artifact.
        This used to be a bld.add_post_fun() callback, but post_funs only run
        after every WAF task succeeds - since _resolve_upload_firmware() (called
        from run(), a normal task) needs the very artifact this step produces,
        a clean build (no stale zephyr.elf left over from an earlier run) would
        deadlock: run() raises before the post_fun ever gets a chance to run.
        Calling it here instead - this task already runs after link_task
        (set_run_after), so the AP archive is guaranteed fresh - makes the
        artifact unconditionally available before it's needed, regardless of
        whether anything produced it before.
        """
        bld = self.generator.bld
        link_task = self.generator.link_task

        cmake_bld = bld.bldnode.make_node('zephyr_build').abspath()
        cmake_src = bld.srcnode.make_node('libraries/AP_HAL_Zephyr/zephyr').abspath()
        cmake_bin = bld.env.get_flat('CMAKE')
        lib_output = link_task.outputs[0].abspath()

        # Determine ARDUPILOT_LIB / ARDUPILOT_BIN / ARDUPILOT_CMD
        # ARDUPILOT_BIN: directory holding the program-specific archive (libarducopter.a etc.)
        # ARDUPILOT_LIB: parent of BIN — holds the shared _libs.a archive (libArduCopter_libs.a etc.)
        ardupilot_bin = os.path.dirname(lib_output)
        ardupilot_lib = os.path.dirname(ardupilot_bin)
        program_name = getattr(bld.env, 'ARDUPILOT_CMD', '') or os.path.basename(lib_output)
        # Strip lib prefix and .a suffix to get the command name
        if program_name.startswith('lib') and program_name.endswith('.a'):
            program_name = program_name[3:-2]  # e.g. libarducopter.a → arducopter

        # Re-run cmake configure with ARDUPILOT_LIB so CMakeLists.txt activates
        # the link section, then build all.

        # IRAM placement: generate ap_iram_sections.ld from the PER BOARD registry.
        # This was hardcoded to ESP32S3Zephyr, so the Xtensa fragment was injected
        # into ARM links, silently placing nothing. NXP RT uses itcm_hot_code.ld.
        iram_registry = os.path.join(
            bld.srcnode.abspath(),
            'libraries', 'AP_HAL_Zephyr', 'hwdef', bld.env.get_flat('BOARD'),
            'ap_iram_registry.txt')
        iram_script = os.path.join(
            bld.srcnode.abspath(),
            'libraries', 'AP_HAL_Zephyr', 'hwdef', 'common',
            'ap_iram_sections.sh')
        iram_ld = os.path.join(ardupilot_bin, '..', 'ap_iram_sections.ld')
        iram_ld = os.path.normpath(iram_ld)
        if os.path.exists(iram_registry) and os.path.exists(iram_script):
            ret = subprocess.call(['/bin/bash', iram_script,
                                   os.path.dirname(ardupilot_bin),
                                   iram_registry, iram_ld])
            if ret != 0:
                print('Warning: ap_iram_sections.sh failed (exit %d) — '
                      'continuing without IRAM placement' % ret)
                iram_ld = ''
        else:
            iram_ld = ''

        configure_cmd = [
            cmake_bin,
            '-S', cmake_src,
            '-B', cmake_bld,
            '-DARDUPILOT_LIB=' + ardupilot_lib,
            '-DARDUPILOT_BIN=' + ardupilot_bin,
            '-DARDUPILOT_CMD=' + program_name,
            # so the Zephyr-compiled shim can #include the generated hwdef.h
            # and use HAL_USB_STRING_PRODUCT etc, the same header the
            # WAF-compiled HAL sources already consume
            '-DARDUPILOT_HWDEF_DIR=' + bld.env.get_flat('BUILDROOT'),
        ]
        # Pass this ALWAYS, empty when unused. Omitting it does not clear it:
        # CMake caches the variable, so a value set by an earlier configure
        # (e.g. when the registry lookup was hardcoded to ESP32S3Zephyr) stays
        # in CMakeCache.txt forever and CMakeLists.txt's if(DEFINED ...) keeps
        # firing. Passing an empty string makes the guard fail as intended.
        configure_cmd.append('-DAP_IRAM_SECTIONS_LD=' + iram_ld)

        # Bootloader-only devicetree overlay, the DTS analogue of
        # prj.<board>-bl.conf: some nodes are app-only and are dead weight in a
        # bootloader. A change flips the configure hash so app<->bl re-configures.
        overlays = _dtc_overlays(bld.env, cmake_src)
        configure_cmd.append('-DEXTRA_DTC_OVERLAY_FILE=' + ';'.join(overlays))

        # Only re-configure when a value we pass changed. A cmake configure
        # regenerates devicetree_generated.h with a fresh mtime even if identical,
        # and ninja is mtime-based, so nearly every TU recompiled on every build.
        want = {
            'ARDUPILOT_LIB': ardupilot_lib,
            'ARDUPILOT_BIN': ardupilot_bin,
            'ARDUPILOT_CMD': program_name,
            'ARDUPILOT_HWDEF_DIR': bld.env.get_flat('BUILDROOT'),
            'AP_IRAM_SECTIONS_LD': iram_ld,
            # A new configure_cmd arg MUST be listed here or it never takes
            # effect: the cache keeps its old value and cmake is never re-run.
            # Covers LIST changes; overlay CONTENT is ninja-tracked via zephyr.dts.d.
            'EXTRA_DTC_OVERLAY_FILE': ';'.join(overlays),
        }
        cache_path = os.path.join(cmake_bld, 'CMakeCache.txt')
        need_configure = True
        if os.path.exists(cache_path):
            cached = {}
            with open(cache_path, 'r', encoding='utf-8') as fh:
                for line in fh:
                    if ':' in line and '=' in line:
                        key = line.split(':', 1)[0]
                        if key in want:
                            cached[key] = line.split('=', 1)[1].rstrip('\n')
            need_configure = any(cached.get(k) != v for k, v in want.items())

        if need_configure:
            ret = subprocess.call(configure_cmd)
            if ret != 0:
                bld.fatal('cmake re-configure for AP link failed (exit %d)' % ret)
        else:
            Logs.info('Zephyr: cmake cache already current, skipping re-configure '
                      '(ninja re-runs cmake itself if the DTS or CMakeLists change)')

        ret = subprocess.call([cmake_bin, '--build', cmake_bld, '--target', 'all'])
        if ret != 0:
            bld.fatal('cmake final link failed (exit %d)' % ret)

        # For ESP32/ESP32-S3 Zephyr boards, run esptool directly here because
        # zephyr.bin is produced by the cmake build above, which otherwise
        # wouldn't exist yet when the generic uploader.py path below runs.
        if bld.options.upload:
            zephyr_board = bld.env.get_flat('ZEPHYR_BOARD') or ''
            if 'esp32' in zephyr_board.lower():
                esptool = shutil.which('esptool') or shutil.which('esptool.py')
                if not esptool:
                    bld.fatal('--upload on ESP32 Zephyr board requires esptool on PATH')
                zephyr_bin = os.path.join(cmake_bld, 'zephyr', 'zephyr.bin')
                chip = 'esp32s3' if 'esp32s3' in zephyr_board.lower() else 'esp32'
                port = getattr(bld.options, 'upload_port', None) or '/dev/ttyACM0'
                flash_cmd = [esptool,
                             '--chip', chip,
                             '--port', port,
                             '--baud', '921600',
                             'write-flash', '0x0',
                             zephyr_bin]
                print('esptool: flashing {} ({}) → {}'.format(
                    os.path.basename(zephyr_bin), chip, port))
                ret = subprocess.call(flash_cmd)
                if ret != 0:
                    bld.fatal('esptool flash failed (exit %d)' % ret)
                print('NOTE: On native-USB ESP32 boards the RTS hard-reset has no effect.')
                print('      Physically unplug and replug the USB cable (or press RESET) to boot the app.')

    def _emit_bootloader_artifacts(self, apj_path):
        """Dual-format bootloader delivery (2026-08-13): distinctly-named
        copies so an app build in the same dir cannot clobber them, plus an
        imgtool-signed MCUBoot image so mcumgr tooling can push the
        bootloader. libraries/AP_HAL_Zephyr/BOOTLOADER_SECURITY.md covers
        what that image is and is not checked against.

        Signing key: AP_BL_IMGTOOL_KEY env var if set, else unsigned-hash
        image (imgtool without -k still emits a valid MCUBoot image with
        SHA256 TLV; verification-required bootloaders will reject it, which
        is the honest default until keys are provisioned)."""
        buildroot = self.env.get_flat('BUILDROOT')
        board = self.env.get_flat('BOARD') or 'zephyr'
        bin_src = os.path.join(buildroot, 'zephyr_upload.bin')
        base = os.path.join(buildroot, 'ap_bootloader_%s' % board)
        try:
            shutil.copyfile(bin_src, base + '.bin')
            shutil.copyfile(apj_path, base + '.apj')
        except OSError as e:
            Logs.warn('Zephyr: bootloader artifact copy failed: %s' % e)
            return
        # imgtool ships as a console-script, not a runnable module
        # (`python3 -m imgtool` fails: it has no __main__). Prefer the CLI on
        # PATH; the .bin/.apj are already written, so a missing imgtool only
        # skips the MCUBoot .img.
        imgtool = shutil.which('imgtool')
        if imgtool is None:
            print('Zephyr: bootloader artifacts: %s.{bin,apj}  '
                  '(.img skipped - imgtool not on PATH: pip install imgtool)' % base)
            return
        img_cmd = [imgtool, 'sign',
                   '--align', '4', '--version', '1.0.0',
                   '--header-size', '0x200', '--pad-header',
                   '--slot-size', '0x100000',
                   bin_src, base + '.img']
        key = os.environ.get('AP_BL_IMGTOOL_KEY')
        if key:
            img_cmd[2:2] = ['-k', key]
        ret = subprocess.run(img_cmd, capture_output=True, text=True, check=False)
        if ret.returncode == 0:
            print('Zephyr: bootloader artifacts: %s.{bin,apj,img}%s' %
                  (base, '' if key else '  (.img UNSIGNED - set AP_BL_IMGTOOL_KEY)'))
        else:
            Logs.warn('Zephyr: imgtool sign failed (%s); .bin/.apj still emitted'
                      % (ret.stderr.strip().splitlines()[-1:] or ['?']))

    def _emit_app_mcuboot_image(self, apj_path):
        """MCUBoot-format APP image for the bootloader's A/B slot 1
        (Tools/AP_Bootloader/mcuboot_ab.cpp): ap_firmware_<board>.img.

        Layout requirement: mcuboot_ab_update() copies slot 1 verbatim over
        slot 0, and the bootloader jumps to slot0+APP_VECTOR_OFFSET (0x2000).
        zephyr_upload.bin already carries a 0x2000 pad in that region (the
        FCB/IVT area - see _pad_bin_for_flash_load_offset), so the .img is
        built from the UNPADDED app with --header-size 0x2000: imgtool's
        padded header occupies exactly the region the pad did, and the vector
        table lands at +0x2000 after the copy. --slot-size matches
        AP_MCUBOOT_SLOT_SIZE (2 MB). Same key handling as the bootloader
        artifact: AP_BL_IMGTOOL_KEY env var, else SHA256-only image (which the
        default bootloader build accepts - Ed25519 enforcement is a
        compile-time option there)."""
        board = self.env.get_flat('BOARD') or 'zephyr'
        pad = self._BOOTLOADER_UPLOAD_PAD_BYTES.get(board, 0)
        if pad <= 0:
            return   # no bootloader-relative app layout on this board
        buildroot = self.env.get_flat('BUILDROOT')
        bin_src = os.path.join(buildroot, 'zephyr_upload.bin')
        imgtool = shutil.which('imgtool')
        if imgtool is None:
            return   # .bin/.apj unaffected; A/B staging just unavailable
        try:
            with open(bin_src, 'rb') as f:
                image = f.read()
        except OSError:
            return
        if len(image) <= pad:
            return
        unpadded = os.path.join(buildroot, 'ap_firmware_%s_unpadded.tmp' % board)
        out_img = os.path.join(buildroot, 'ap_firmware_%s.img' % board)
        with open(unpadded, 'wb') as f:
            f.write(image[pad:])
        # --slot-size matches the DTS slot0/slot1_partition size (3 MB) and
        # mcuboot_ab.cpp's AP_MCUBOOT_SLOT_SIZE - keep the three in sync.
        img_cmd = [imgtool, 'sign',
                   '--align', '4', '--version', '1.0.0',
                   '--header-size', '0x%x' % pad, '--pad-header',
                   '--slot-size', '0x300000',
                   unpadded, out_img]
        key = os.environ.get('AP_BL_IMGTOOL_KEY')
        if key:
            img_cmd[2:2] = ['-k', key]
        ret = subprocess.run(img_cmd, capture_output=True, text=True, check=False)
        try:
            os.unlink(unpadded)
        except OSError:
            pass
        if ret.returncode == 0:
            print('Zephyr: app MCUBoot image: %s%s' %
                  (out_img, '' if key else '  (SHA256-only - set AP_BL_IMGTOOL_KEY to sign)'))
        else:
            Logs.warn('Zephyr: app imgtool sign failed (%s)'
                      % (ret.stderr.strip().splitlines()[-1:] or ['?']))

    def run(self):
        self._run_ap_final_link()

        # native_sim produces a host executable (bin/arducopter), not
        # firmware: there is nothing to objcopy into an APJ and nothing to
        # upload.
        if (self.env.get_flat('ZEPHYR_BOARD') or '').startswith('native_sim'):
            return 0

        upload_tools = os.path.join(self.env.get_flat('SRCROOT'), 'Tools', 'scripts')
        upload_port = self.generator.bld.options.upload_port
        fw_path = self._resolve_upload_firmware()

        # Dual-format bootloader artifacts are a BUILD product, not an upload
        # step - emit them here, before the no-upload early return, so a plain
        # `waf bootloader` (the documented flow, no --upload) produces them.
        if self.env.BOOTLOADER:
            self._emit_bootloader_artifacts(fw_path)
        else:
            # App builds likewise emit their MCUBoot A/B slot-1 image here.
            self._emit_app_mcuboot_image(fw_path)

        if not self.generator.bld.options.upload:
            # APJ generated; skip actual upload (no --upload flag).
            return 0

        # ESP32/ESP32-S3 Zephyr boards are flashed via esptool inside
        # _cmake_final_link (post_fun) because zephyr.bin doesn't exist yet
        # when this waf task runs.  Nothing to do here for those boards.
        zephyr_board = self.env.get_flat('ZEPHYR_BOARD') or ''
        if 'esp32' in zephyr_board.lower():
            return 0

        # A bootloader build has no bootloader to upload through - it needs the
        # SWD/LinkServer path, which requires BOOT0 held: physical and
        # interactive. Print the command rather than running it.
        if self.env.BOOTLOADER:
            print('')
            print('Bootloader build complete - this is NOT flashed via uploader.py.')
            print('Power the board on with BOOT0 held, then run manually:')
            print('    python3 Tools/scripts/rt1176_linkserver_flash.py')
            print('')
            return 0

        board_name = self._collect_apj_metadata()['board_type']

        cmd = "{} '{}/uploader.py' '{}'".format(
            self.env.get_flat('PYTHON'),
            upload_tools,
            fw_path,
        )
        if upload_port is not None:
            cmd += " '--port' '{}'".format(upload_port)
        if board_name:
            cmd += " '--boardname' '{}'".format(board_name)
        if self.generator.bld.options.upload_force:
            cmd += " '--force'"
        return self.exec_command(cmd)

    def exec_command(self, cmd, **kw):
        # Keep uploader.py fully interactive when called under waf --upload.
        kw['stdout'] = sys.stdout
        kw['stderr'] = sys.stderr
        kw['stdin'] = sys.stdin
        return super(upload_fw_zephyr, self).exec_command(cmd, **kw)

    def keyword(self):
        return 'Uploading'


@feature('zephyr_ap_program')
@after_method('process_source')
def zephyr_program(self):
    if self.env.BOARD_CLASS != 'Zephyr':
        return

    if not self.env.ZEPHYR_BASE:
        self.env.ZEPHYR_BASE = _resolve_zephyr_base(self.env)

    if not self.env.ZEPHYR_BASE:
        return

    if not hasattr(self, 'link_task'):
        return

    try:
        zephyr = self.bld.cmake('zephyr')
    except Exception:  # noqa: BLE001
        return

        # Build ONLY the zephyr library target here, never 'all'. The final
        # link persists ARDUPILOT_LIB in the cmake cache, so 'all' would link
        # zephyr.elf against the PREVIOUS AP archive before link_task refreshes it.
    build = zephyr.build('zephyr', target='zephyr_build/zephyr/libzephyr.a')
    build.post()
    self.link_task.set_run_after(build.cmake_build_task)

    # The final zephyr.elf is produced by upload_fw_zephyr._run_ap_final_link(),
    # not here. As a bld.add_post_fun() it deadlocked: post_funs run only after
    # every task succeeds, but upload_fw_zephyr.run() needs that artifact first.

    # Always generate the APJ so it can be flashed separately even without
    # --upload.  The task's run() checks bld.options.upload before calling
    # the uploader, so no interactive prompt happens on a plain build.
    link_output = self.link_task.outputs[0]
    upload_task = self.create_task('upload_fw_zephyr', src=link_output)
    upload_task.set_run_after(self.link_task)
