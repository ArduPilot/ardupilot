# encoding: utf-8
# AP_FLAKE8_CLEAN

"""
Build the pinned Picolibc sources directly with Waf.
"""

import os
import subprocess

from waflib import Errors
from waflib import Utils
from waflib.TaskGen import after_method
from waflib.TaskGen import feature

# Sources available to all ChibiOS firmware. This intentionally excludes
# process termination, signals and direct stdio output so accidental use in
# flight firmware fails at link time. Use the AP_HAL interfaces instead.
PICOLIBC_CHIBIOS_BASE_SOURCES = (
    # libc/ctype
    'libc/ctype/ctype_.c',
    'libc/ctype/isalnum.c',
    'libc/ctype/isalpha.c',
    'libc/ctype/isprint.c',
    'libc/ctype/isspace.c',
    'libc/ctype/tolower.c',
    'libc/ctype/toupper.c',
    # libc/locale
    'libc/locale/localeconv.c',
    # libc/machine/arm
    'libc/machine/arm/memchr.S',
    'libc/machine/arm/memchr.c',
    'libc/machine/arm/memcmp.c',
    'libc/machine/arm/memcpy.c',
    'libc/machine/arm/memmove.c',
    'libc/machine/arm/memset.c',
    'libc/machine/arm/setjmp.S',
    'libc/machine/arm/strcmp.S',
    'libc/machine/arm/strcpy.S',
    'libc/machine/arm/strlen.S',
    # libc/misc
    'libc/misc/dso_handle.c',
    # libc/stdio
    'libc/stdio/atod_ryu.c',
    'libc/stdio/atof_ryu.c',
    'libc/stdio/ryu_divpow2.c',
    'libc/stdio/ryu_log2pow5.c',
    'libc/stdio/ryu_pow5bits.c',
    'libc/stdio/ryu_table.c',
    'libc/stdio/ryu_umul128.c',
    'libc/stdio/strtod.c',
    'libc/stdio/strtof.c',
    'libc/stdio/strtol.c',
    'libc/stdio/strtoul.c',
    # libc/stdlib
    'libc/stdlib/_findenv.c',
    'libc/stdlib/atof.c',
    'libc/stdlib/atoi.c',
    'libc/stdlib/atol.c',
    'libc/stdlib/cxa-atexit.c',
    'libc/stdlib/environ.c',
    'libc/stdlib/exitprocs.c',
    'libc/stdlib/getenv.c',
    'libc/stdlib/rand.c',
    'libc/stdlib/random.c',
    'libc/stdlib/srand.c',
    'libc/stdlib/srandom.c',
    # libc/string
    'libc/string/memmem.c',
    'libc/string/strcasecmp.c',
    'libc/string/strchr.c',
    'libc/string/strcoll.c',
    'libc/string/strcspn.c',
    'libc/string/strdup.c',
    'libc/string/strerror.c',
    'libc/string/strncasecmp.c',
    'libc/string/strncat.c',
    'libc/string/strncmp.c',
    'libc/string/strncpy.c',
    'libc/string/strndup.c',
    'libc/string/strnlen.c',
    'libc/string/strpbrk.c',
    'libc/string/strrchr.c',
    'libc/string/strspn.c',
    'libc/string/strstr.c',
    'libc/string/strtok.c',
    'libc/string/strtok_r.c',
    'libc/string/swab.c',
    # libc/time
    'libc/time/gmtime_r.c',
    # libm/common
    'libm/common/math_err_divzero.c',
    'libm/common/math_err_invalid.c',
    'libm/common/math_err_oflow.c',
    'libm/common/math_err_uflow.c',
    'libm/common/math_errf_divzerof.c',
    'libm/common/math_errf_invalidf.c',
    'libm/common/math_errf_oflowf.c',
    'libm/common/math_errf_uflowf.c',
    'libm/common/math_inexact.c',
    'libm/common/math_inexactf.c',
    'libm/common/s_fmax.c',
    'libm/common/s_fmin.c',
    'libm/common/s_fpclassify.c',
    'libm/common/s_isinf.c',
    'libm/common/s_isnan.c',
    'libm/common/s_issignaling.c',
    'libm/common/s_modf.c',
    'libm/common/s_scalbn.c',
    'libm/common/sf_fmax.c',
    'libm/common/sf_fmin.c',
    'libm/common/sf_fpclassify.c',
    'libm/common/sf_isinf.c',
    'libm/common/sf_isnan.c',
    'libm/common/sf_issignaling.c',
    'libm/common/sf_lrint.c',
    'libm/common/sf_modf.c',
    'libm/common/sf_nan.c',
    'libm/common/sf_scalbn.c',
    # libm/machine/arm
    'libm/machine/arm/s_ceil.c',
    'libm/machine/arm/s_floor.c',
    'libm/machine/arm/s_sqrt.c',
    'libm/machine/arm/sf_ceil.c',
    'libm/machine/arm/sf_floor.c',
    'libm/machine/arm/sf_round.c',
    'libm/machine/arm/sf_sqrt.c',
    # libm/math
    'libm/math/k_cos.c',
    'libm/math/k_rem_pio2.c',
    'libm/math/k_sin.c',
    'libm/math/kf_cos.c',
    'libm/math/kf_sin.c',
    'libm/math/kf_tan.c',
    'libm/math/s_acos.c',
    'libm/math/s_asin.c',
    'libm/math/s_atan.c',
    'libm/math/s_atan2.c',
    'libm/math/s_cos.c',
    'libm/math/s_fmod.c',
    'libm/math/s_log.c',
    'libm/math/s_pow.c',
    'libm/math/s_rem_pio2.c',
    'libm/math/s_sin.c',
    'libm/math/sf_acos.c',
    'libm/math/sf_asin.c',
    'libm/math/sf_atan.c',
    'libm/math/sf_atan2.c',
    'libm/math/sf_cos.c',
    'libm/math/sf_exp.c',
    'libm/math/sf_fmod.c',
    'libm/math/sf_frexp.c',
    'libm/math/sf_log.c',
    'libm/math/sf_log10.c',
    'libm/math/sf_log2.c',
    'libm/math/sf_pow.c',
    'libm/math/sf_rem_pio2.c',
    'libm/math/sf_sin.c',
    'libm/math/sf_tan.c',
)

# Additional sources used by simulation-on-hardware, Replay, tools, examples
# and tests. This includes the complete Picolibc dependency closure for those paths.
PICOLIBC_CHIBIOS_NONFLIGHT_SOURCES = (
    # libc/locale
    'libc/locale/locale_names.c',
    'libc/locale/localedata.c',
    'libc/locale/setlocale.c',
    'libc/locale/timedata.c',
    # libc/misc
    'libc/misc/fini.c',
    # libc/stdio
    'libc/stdio/asprintf.c',
    'libc/stdio/dtoa_ryu.c',
    'libc/stdio/dtox_engine.c',
    'libc/stdio/fgetc.c',
    'libc/stdio/filestrget.c',
    'libc/stdio/filestrput.c',
    'libc/stdio/filestrputalloc.c',
    'libc/stdio/printf.c',
    'libc/stdio/ryu_log10.c',
    'libc/stdio/snprintf.c',
    'libc/stdio/snprintfd.c',
    'libc/stdio/sprintf.c',
    'libc/stdio/sscanf.c',
    'libc/stdio/ungetc.c',
    'libc/stdio/vasprintf.c',
    'libc/stdio/vfprintf.c',
    'libc/stdio/vfscanf.c',
    'libc/stdio/vprintf.c',
    'libc/stdio/vsnprintf.c',
    # libc/stdlib
    'libc/stdlib/abort.c',
    'libc/stdlib/div.c',
    'libc/stdlib/exit.c',
    'libc/stdlib/system.c',
    # libc/time
    'libc/time/clock.c',
    'libc/time/difftime.c',
    'libc/time/lcltime_r.c',
    'libc/time/mktime.c',
    'libc/time/month_lengths.c',
    'libc/time/strftime.c',
    'libc/time/time.c',
    'libc/time/tzcalc_limits.c',
    'libc/time/tzset.c',
    'libc/time/tzvars.c',
    # libos/dummyhost
    'libos/dummyhost/_exit.c',
    # libos/fallback
    'libos/fallback/raise.c',
    'libos/fallback/signal.c',
)

PICOLIBC_CHIBIOS_NONFLIGHT_COMMANDS = frozenset((
    'all',
    'benchmarks',
    'check',
    'check-all',
    'examples',
    'replay',
    'tests',
    'tool',
))

PICOLIBC_CHIBIOS_NONFLIGHT_GROUPS = frozenset((
    'all',
    'benchmarks',
    'examples',
    'replay',
    'tests',
    'tool',
))

PICOLIBC_SITL_SOURCES = (
    'libc/stdio/atod_ryu.c',
    'libc/stdio/atof_ryu.c',
    'libc/stdio/ryu_divpow2.c',
    'libc/stdio/ryu_log2pow5.c',
    'libc/stdio/ryu_pow5bits.c',
    'libc/stdio/ryu_table.c',
    'libc/stdio/strtod.c',
    'libc/stdio/strtof.c',
    'libc/stdlib/atof.c',
)

# Namespace every global defined by the SITL source closure, along with the
# four classification helpers supplied by AP_Common. This works with ELF,
# Mach-O and PE/COFF and avoids requiring a host objcopy.
PICOLIBC_SITL_RENAMES = (
    '__atod_engine',
    '__atof_engine',
    '__ceil_log2pow5',
    '__double_computeInvPow5',
    '__double_computePow5',
    '__fpclassifyd',
    '__fpclassifyf',
    '__isnand',
    '__isnanf',
    '__log2pow5',
    '__mulPow5InvDivPow2',
    '__mulPow5divPow2',
    '__pow5Factor',
    '__pow5bits',
    'atof',
    'strtod',
    'strtof',
    'strtold',
)

PICOLIBC_CONFIG = """/* Generated by the ArduPilot Waf build. */
#pragma once

#define _NEWLIB_VERSION "4.3.0"
#undef __ARM_SEMIHOST
#undef __ARM_SEMIHOST_USE_HLT
#undef __ASSERT_VERBOSE
#define __ATOMIC_UNGETC
#undef __FAST_BUFIO
#define __FAST_STRCMP
#undef __FSTAT_BUFSIZ
#undef __GLOBAL_ERRNO
#define __HAVE_BITFIELDS_IN_PACKED_STRUCTS
#define __HAVE_BUILTIN_COMPLEX 1
#define __HAVE_CC_INHIBIT_LOOP_TO_LIBCALL
#define __HAVE_COMPLEX
#undef __HAVE_FCNTL
#undef __IEEEFP_FUNCS
#define __IEEE_LIBM
@IEEE_ENDIAN@
@INIT_FINI_ARRAY@
#undef __INIT_FINI_FUNCS
#define __IO_C99_FORMATS
#define __IO_DEFAULT 'd'
#define __IO_FLOAT_EXACT
#define __IO_LENGTH_W
#undef __IO_LONG_DOUBLE
#define __IO_LONG_LONG
#undef __IO_MINIMAL_LONG_LONG
#define __IO_PERCENT_B
#undef __IO_PERCENT_N
#undef __IO_POS_ARGS
#define __IO_SMALL_ULTOA
#undef __IO_WCHAR
#undef __MALLOC_CLEAR_FREED
#undef __MALLOC_ERROR_ABORT
#define __MALLOC_SMALL_BUCKET 0
#undef __MATH_ERRNO
#undef __MB_CAPABLE
#define __NEWLIB_MINOR__ 3
#define __NEWLIB_PATCHLEVEL__ 0
#define __NEWLIB_VERSION__ "4.3.0"
#define __NEWLIB__ 4
#undef __OBSOLETE_MATH
#undef __OBSOLETE_MATH_DOUBLE
#undef __OBSOLETE_MATH_FLOAT
#define __PICOLIBC_ERRNO_FUNCTION @ERRNO_FUNCTION@
#define __PICOLIBC_EXPORT
#define __PICOLIBC_MINOR__ 8
#define __PICOLIBC_PATCHLEVEL__ 12
#define __PICOLIBC_VERSION__ "1.8.12"
#define __PICOLIBC__ 1
#define __PREFER_SIZE_OVER_SPEED
#undef __SEMIHOST
#undef __SINGLE_THREAD
#undef __STDIO_EXIT_FLUSH
#define __STDIO_LOCKING
#undef __STRCMP_PAIR
@THREAD_LOCAL_STORAGE@
#undef __THREAD_LOCAL_STORAGE_API
#undef __THREAD_LOCAL_STORAGE_RP2040
#undef __THREAD_LOCAL_STORAGE_STACK_GUARD
#define __TINY_STDIO
@USE_SYSTEM_LIBC@
"""

PICOLIBC_DARWIN_COMPAT = """/* Generated by the ArduPilot Waf build. */
#pragma once
#include <sys/cdefs.h>

/* Apple Clang advertises aliases that Mach-O cannot emit. */
#undef __strong_reference
#undef __strong_reference_dup
"""


def _program(env, name):
    value = Utils.to_list(env[name])
    if not value:
        raise Errors.WafError('Picolibc requires %s to be configured' % name)
    return value


def _abi_flags(env):
    flags = []
    prefixes = ('-mcpu=', '-march=', '-mfpu=', '-mfloat-abi=')
    for flag in env.CPU_FLAGS:
        if flag == '-mthumb' or flag.startswith(prefixes):
            flags.append(flag)
    if '-mthumb' not in flags:
        flags.append('-mthumb')
    return flags


def _sitl_abi_flags(env):
    return [flag for flag in env.CFLAGS if flag in ('-m32', '-m64')]


def _compiler_include_dirs(program, abi_flags, language):
    command = program + abi_flags + ['-E', '-v', '-x', language, '-']
    result = subprocess.run(command, input='', text=True, capture_output=True, check=True)
    includes = []
    in_search_list = False
    for line in result.stderr.splitlines():
        if '#include <...> search starts here:' in line:
            in_search_list = True
            continue
        if in_search_list and line.strip() == 'End of search list.':
            break
        if in_search_list:
            path = line.strip()
            if path and not path.startswith('('):
                includes.append(os.path.normpath(path))
    if not includes:
        raise Errors.WafError('Unable to determine %s system include paths' % language)
    return includes


def _without_toolchain_libc(includes):
    return [path for path in includes
            if not (path.endswith('/arm-none-eabi/include') or
                    path.endswith('/arm-none-eabi/usr/include'))]


def _write_if_changed(path, contents):
    try:
        with open(path, 'r', encoding='utf-8') as existing:
            if existing.read() == contents:
                return
    except FileNotFoundError:
        pass
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as output:
        output.write(contents)


def _config_header(mode, dest_os, toolchain):
    errno_functions = {
        'cygwin': '__errno',
        'darwin': '__error',
        'linux': '__errno_location',
    }
    if mode == 'chibios':
        errno_function = 'ap_picolibc_errno'
    elif toolchain == 'emscripten':
        errno_function = '__errno_location'
    else:
        try:
            errno_function = errno_functions[dest_os]
        except KeyError as error:
            raise Errors.WafError('Unsupported SITL errno ABI: %s' % dest_os) from error
    values = {
        '@ERRNO_FUNCTION@': errno_function,
        '@IEEE_ENDIAN@': '#define __IEEE_LITTLE_ENDIAN' if toolchain == 'emscripten' else '',
        '@INIT_FINI_ARRAY@': '#define __INIT_FINI_ARRAY' if mode == 'chibios' else '#undef __INIT_FINI_ARRAY',
        '@THREAD_LOCAL_STORAGE@': '#undef __THREAD_LOCAL_STORAGE' if mode == 'chibios' else '#define __THREAD_LOCAL_STORAGE',
        '@USE_SYSTEM_LIBC@': '' if mode == 'chibios' else '#define __USE_SYSTEM_LIBC',
    }
    header = PICOLIBC_CONFIG
    for token, value in values.items():
        header = header.replace(token, value)
    return header


def _write_config_headers(config_dir, mode, dest_os, toolchain):
    _write_if_changed(os.path.join(config_dir, 'picolibc.h'),
                      _config_header(mode, dest_os, toolchain))
    if dest_os == 'darwin':
        _write_if_changed(os.path.join(config_dir, 'picolibc-darwin-compat.h'),
                          PICOLIBC_DARWIN_COMPAT)


def configure(cfg):
    board_class = cfg.env.BOARD_CLASS
    if board_class not in ('ChibiOS', 'SITL'):
        cfg.fatal('Picolibc supports ChibiOS and SITL boards only')
    if (board_class == 'SITL' and
            cfg.env.DEST_OS not in ('cygwin', 'darwin', 'linux') and
            cfg.env.TOOLCHAIN != 'emscripten'):
        cfg.fatal('Picolibc does not support the %s SITL errno ABI' % cfg.env.DEST_OS)

    cfg.load('asm')
    cfg.env.AS = cfg.env.CC
    cfg.env.AS_TGT_F = ['-c', '-o']
    cfg.env.ASM_NAME = 'gas'
    source = cfg.srcnode.make_node('modules/picolibc').abspath()
    if not os.path.isfile(os.path.join(source, 'libc', 'include', 'stdlib.h')):
        cfg.fatal('Picolibc submodule is missing; run git submodule update --init modules/picolibc')

    gitlink = subprocess.check_output(
        ['git', '-C', cfg.srcnode.abspath(), 'ls-files', '-s', '--', 'modules/picolibc'],
        text=True).strip()
    revision = gitlink.split()[1]
    board_build = cfg.bldnode.make_node(cfg.variant)
    if board_class == 'ChibiOS':
        # Keep the identical generated header at a board-independent path so
        # ccache can reuse Picolibc and application objects between boards.
        config_dir = cfg.bldnode.make_node('picolibc-config/chibios').abspath()
    else:
        config_dir = board_build.make_node('modules/picolibc-config').abspath()
    mode = board_class.lower()
    _write_config_headers(config_dir, mode, cfg.env.DEST_OS, cfg.env.TOOLCHAIN)

    cfg.env.PICOLIBC = True
    cfg.env.PICOLIBC_CONFIG_DIR = config_dir
    cfg.env.PICOLIBC_MODE = mode
    cfg.env.PICOLIBC_REVISION = revision
    cfg.env.GIT_SUBMODULES += ['picolibc']
    if board_class == 'ChibiOS':
        cfg.env.PICOLIBC_LIB = board_build.make_node('modules/picolibc/libc.a').abspath()
        abi_flags = _abi_flags(cfg.env)
        public_include_dirs = [
            config_dir,
            os.path.join(source, 'libc', 'machine', 'arm'),
            os.path.join(source, 'libc', 'include'),
        ]
        c_system_dirs = _without_toolchain_libc(
            _compiler_include_dirs(_program(cfg.env, 'CC'), abi_flags, 'c')) + public_include_dirs
        cxx_system_dirs = _without_toolchain_libc(
            _compiler_include_dirs(_program(cfg.env, 'CXX'), abi_flags, 'c++')) + public_include_dirs
        c_system_flags = ['-nostdinc'] + ['-isystem%s' % path for path in c_system_dirs]
        cxx_system_flags = ['-nostdinc'] + ['-isystem%s' % path for path in cxx_system_dirs]
        cfg.env.prepend_value('CFLAGS', c_system_flags)
        cfg.env.prepend_value('CXXFLAGS', cxx_system_flags)
        cfg.env.PICOLIBC_C_SYSTEM_FLAGS = c_system_flags
        cfg.env.PICOLIBC_INCLUDE_DIRS = public_include_dirs
        cfg.env.CHIBIOS_BUILD_FLAGS = (
            cfg.env.get_flat('CHIBIOS_BUILD_FLAGS') +
            ' PICOLIBC_C_INCLUDE_FLAGS="%s"' %
            ' '.join(c_system_flags) +
            ' PICOLIBC_CXX_INCLUDE_FLAGS="%s"' % ' '.join(cxx_system_flags)
        )
    else:
        cfg.env.PICOLIBC_LIB = board_build.make_node('modules/picolibc-sitl/libpicolibc.a').abspath()
        cfg.env.append_value('CFLAGS', '-DAP_PICOLIBC_SITL_ENABLED=1')
        cfg.define('AP_PICOLIBC_SITL_ENABLED', 1)
        if cfg.env.TOOLCHAIN == 'emscripten':
            cfg.env.append_value('LINKFLAGS', [
                '-Wl,--wrap=atof',
                '-Wl,--wrap=strtod',
                '-Wl,--wrap=strtof',
            ])

    cfg.msg('Picolibc source build', revision[:12])


def _chibios_nonflight_build(bld):
    if bld.env.SIM_ENABLED or bld.cmd in PICOLIBC_CHIBIOS_NONFLIGHT_COMMANDS:
        return True
    groups = set(Utils.to_list(bld.options.program_group))
    if groups & PICOLIBC_CHIBIOS_NONFLIGHT_GROUPS:
        return True
    targets = [target.strip().lower() for target in (bld.targets or '').split(',')]
    prefixes = ('benchmarks/', 'examples/', 'tests/', 'tool/')
    return any(target.startswith(prefixes) or target == 'replay'
               for target in targets)


def _source_nodes(bld, source):
    mode = bld.env.PICOLIBC_MODE
    if mode == 'chibios':
        source_paths = PICOLIBC_CHIBIOS_BASE_SOURCES
        if _chibios_nonflight_build(bld):
            source_paths += PICOLIBC_CHIBIOS_NONFLIGHT_SOURCES
        source_paths = sorted(source_paths)
    else:
        source_paths = PICOLIBC_SITL_SOURCES
    nodes = [source.find_node(path) for path in source_paths]
    missing = [path for path, node in zip(source_paths, nodes) if node is None]
    if missing:
        bld.fatal('Missing Picolibc %s sources: %s' %
                  (mode, ', '.join(missing)))
    return nodes


def _compile_flags(env):
    flags = [
        '-std=c18',
        '-Os',
        '-ffunction-sections',
        '-fdata-sections',
        '-fno-common',
        '-MMD',
        '-frounding-math',
        '-ftrapping-math',
        '-fsignaling-nans',
        '-fno-builtin-copysignl',
        '-fno-stack-protector',
        '-D_FILE_OFFSET_BITS=64',
        '-D_LIBC',
        '-U_FORTIFY_SOURCE',
    ]
    if env.PICOLIBC_MODE == 'chibios':
        return _abi_flags(env) + Utils.to_list(env.PICOLIBC_C_SYSTEM_FLAGS) + flags
    if env.DEST_OS == 'darwin':
        flags.remove('-fsignaling-nans')
        flags += [
            '-include', os.path.join(env.PICOLIBC_CONFIG_DIR,
                                     'picolibc-darwin-compat.h'),
            '-Wno-macro-redefined',
        ]
    renames = ['-D%s=ap_picolibc_%s' % (symbol, symbol)
               for symbol in PICOLIBC_SITL_RENAMES]
    return _sitl_abi_flags(env) + renames + flags


@feature('picolibc_archive')
@after_method('apply_link')
def make_picolibc_archive(self):
    self.picolibc_task = self.link_task


def build(bld):
    source = bld.srcnode.find_dir('modules/picolibc')
    # `waf clean` removes generated files below the board build directory
    # without rerunning configure. Recreate the configuration headers before
    # declaring compile tasks so configure-clean-build works.
    _write_config_headers(bld.env.PICOLIBC_CONFIG_DIR,
                          bld.env.PICOLIBC_MODE, bld.env.DEST_OS,
                          bld.env.TOOLCHAIN)
    config_dir = bld.root.find_dir(bld.env.PICOLIBC_CONFIG_DIR)
    include_dirs = [
        config_dir,
        source,
        source.find_dir('libm/common'),
        source.find_dir('libm/math_two'),
        source.find_dir('libc/stdio'),
        source.find_dir('libc/locale'),
        source.find_dir('libc/include'),
    ]
    if bld.env.PICOLIBC_MODE == 'chibios':
        include_dirs.append(source.find_dir('libc/machine/arm'))
    compile_env = bld.env.derive()
    compile_env.CFLAGS = _compile_flags(bld.env)
    compile_env.ASFLAGS = [flag for flag in compile_env.CFLAGS if flag != '-std=c18']
    compile_env.DEFINES = []
    compile_env.INCLUDES = []
    compile_env.append_value('ASFLAGS', ['-x', 'assembler-with-cpp'])
    target = ('modules/picolibc/c' if bld.env.PICOLIBC_MODE == 'chibios'
              else 'modules/picolibc-sitl/picolibc')
    task_gen = bld.stlib(
        name='picolibc_build',
        features='c cstlib asm picolibc_archive',
        source=_source_nodes(bld, source),
        target=target,
        includes=include_dirs,
        env=compile_env,
        group='dynamic_sources',
    )
    if bld.env.PICOLIBC_MODE == 'sitl':
        bld.env.LIBPATH += [os.path.dirname(bld.env.PICOLIBC_LIB)]
        bld.env.LIB += ['picolibc']
    else:
        bld.env.LIBPATH += [os.path.dirname(bld.env.PICOLIBC_LIB)]
    return task_gen


@feature('cprogram', 'cxxprogram')
@after_method('apply_link')
def wait_for_picolibc(self):
    if not self.env.PICOLIBC:
        return
    task_gen = self.bld.get_tgen_by_name('picolibc_build')
    task_gen.post()
    self.link_task.set_run_after(task_gen.picolibc_task)
    self.link_task.dep_nodes.extend(task_gen.picolibc_task.outputs)
