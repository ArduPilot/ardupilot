# flake8: noqa

"""
WAF Tool to select the correct toolchain based on the target architecture.

This tool loads compiler_c and compiler_cxx, so you don't need to load them
(and you must not load them before this tool). Use the environment variable
TOOLCHAIN to define the toolchain.

Example::

    def configure(cfg):
        cfg.env.TOOLCHAIN = 'arm-linux-gnueabihf'
        cfg.load('toolchain')
"""

from waflib import Errors, Context, Utils
from waflib.Configure import conf
from waflib.Tools import compiler_c, compiler_cxx
from waflib.Tools import clang, clangxx, gcc, gxx
from waflib.Tools import c_config
from waflib import Logs

import glob
import os
import re
import sys

@conf
def find_gxx(conf):
    names = ['g++', 'c++']
    if conf.env.TOOLCHAIN != 'native':
        names = ['%s-%s' % (conf.env.TOOLCHAIN, n) for n in names]
    cxx = conf.find_program(names, var='CXX')
    conf.get_cc_version(cxx, gcc=True)
    conf.env.CXX_NAME = 'gcc'

@conf
def find_gcc(conf):
    names = ['gcc', 'cc']
    if conf.env.TOOLCHAIN != 'native':
        names = ['%s-%s' % (conf.env.TOOLCHAIN, n) for n in names]
    cc = conf.find_program(names, var='CC')
    conf.get_cc_version(cc, gcc=True)
    conf.env.CC_NAME = 'gcc'

def _clang_cross_support(cfg):
    if _clang_cross_support.called:
        return

    prefix = cfg.env.TOOLCHAIN + '-'

    try:
        cfg.find_program(prefix + 'gcc', var='CROSS_GCC')
    except Errors.ConfigurationError as e:
        cfg.fatal("toolchain: clang: couldn't find cross GCC", ex=e)

    environ = dict(os.environ)
    if 'TOOLCHAIN_CROSS_AR' in environ:
        # avoid OS's environment to mess up toolchain path finding
        del environ['TOOLCHAIN_CROSS_AR']
    try:
        cfg.find_program(
            prefix + 'ar',
            var='TOOLCHAIN_CROSS_AR',
            environ=environ,
        )
    except Errors.ConfigurationError as e:
        cfg.fatal("toolchain: clang: couldn't find toolchain path", ex=e)

    toolchain_path = os.path.join(cfg.env.TOOLCHAIN_CROSS_AR[0], '..', '..')
    toolchain_path = os.path.abspath(toolchain_path)
    cfg.msg('Using toolchain path for clang', toolchain_path)

    sysroot = cfg.cmd_and_log(
        [cfg.env.CROSS_GCC[0], '--print-sysroot'],
        quiet=Context.BOTH,
    ).strip()

    cfg.env.CLANG_FLAGS = [
        '--target=' + cfg.env.TOOLCHAIN,
        '--gcc-toolchain=' + toolchain_path,
        '--sysroot=' + sysroot,
        '-B' + os.path.join(toolchain_path, 'bin')
    ]

_clang_cross_support.called = False

def _set_clang_crosscompilation_wrapper(tool_module):
    original_configure = tool_module.configure
    def new_configure(cfg):
        if cfg.env.TOOLCHAIN == 'native':
            original_configure(cfg)
            return

        cfg.env.stash()
        try:
            _clang_cross_support(cfg)
            original_configure(cfg)
        except Errors.ConfigurationError as e:
            cfg.env.revert()
            raise
        else:
            cfg.env.commit()
    tool_module.configure = new_configure

_set_clang_crosscompilation_wrapper(clang)
_set_clang_crosscompilation_wrapper(clangxx)

def _filter_supported_c_compilers(*compilers):
    for k in compiler_c.c_compiler:
        l = compiler_c.c_compiler[k]
        compiler_c.c_compiler[k] = [c for c in compilers if c in l]

def _filter_supported_cxx_compilers(*compilers):
    for k in compiler_cxx.cxx_compiler:
        l = compiler_cxx.cxx_compiler[k]
        compiler_cxx.cxx_compiler[k] = [c for c in compilers if c in l]

def _set_pkgconfig_crosscompilation_wrapper(cfg):
    original_validatecfg = cfg.validate_cfg

    @conf
    def new_validate_cfg(kw):
        if 'path' not in kw:
            if not cfg.env.PKGCONFIG:
                cfg.find_program('%s-pkg-config' % cfg.env.TOOLCHAIN, var='PKGCONFIG')
            kw['path'] = cfg.env.PKGCONFIG

        original_validatecfg(kw)

    cfg.validate_cfg = new_validate_cfg

# Directories where ArduPilot's own setup scripts (Tools/environment_install/
# install-prereqs-*.sh) install the cross toolchains, keyed by toolchain triple.
ARDUPILOT_TOOLCHAIN_DIRS = {
    'arm-none-eabi': ['/opt/gcc-arm-none-eabi-10-2020-q4-major/bin'],
}

def _prefer_installed_toolchain(cfg):
    """Bias program lookup toward ArduPilot's own installed toolchain.

    Prepends the first matching install dir to the PATH that waf's find_program
    consults (cfg.environ), so gcc/g++/ar/nm/objcopy/size all resolve to the
    same toolchain, with a normal PATH fallback when it is not installed there.
    This lets the build find its supported compiler even when a *different*
    arm-none-eabi-gcc is the system default on PATH (e.g. for another project),
    so ArduPilot need not prepend its toolchain to the global PATH.

    A user-supplied --toolchain (which sets TOOLCHAIN to an explicit value rather
    than a plain triple) takes precedence and is left untouched.
    """
    dirs = list(ARDUPILOT_TOOLCHAIN_DIRS.get(cfg.env.TOOLCHAIN, []))
    if cfg.env.TOOLCHAIN == 'arm-none-eabi':
        # accept any versioned install too, preferring the highest version
        dirs += sorted(glob.glob('/opt/gcc-arm-none-eabi-*/bin'), reverse=True)
    for d in dirs:
        if not os.path.exists(os.path.join(d, '%s-gcc' % cfg.env.TOOLCHAIN)):
            continue
        # copy os.environ the first time so we never mutate the real process env
        if getattr(cfg, 'environ', os.environ) is os.environ:
            cfg.environ = dict(os.environ)
        path = cfg.environ.get('PATH', '')
        if d not in path.split(os.pathsep):
            cfg.environ['PATH'] = d + os.pathsep + path
        cfg.msg('Using toolchain dir', d)
        return

def configure(cfg):
    _filter_supported_c_compilers('gcc', 'clang')
    _filter_supported_cxx_compilers('g++', 'clang++')

    cfg.msg('Using toolchain', cfg.env.TOOLCHAIN)
    if cfg.env.TOOLCHAIN == "custom":
        return

    if cfg.env.TOOLCHAIN == 'native':
        cfg.load('compiler_cxx compiler_c gccdeps')

        return

    _prefer_installed_toolchain(cfg)

    _set_pkgconfig_crosscompilation_wrapper(cfg)
    if sys.platform.startswith("cygwin"):
        # on cygwin arm-none-eabi-ar doesn't support the @FILE syntax for splitting long lines
        cfg.find_program('ar', var='AR', quiet=True)
    else:
        cfg.find_program('%s-ar' % cfg.env.TOOLCHAIN, var='AR', quiet=True)
    cfg.load('compiler_cxx compiler_c gccdeps')

    if sys.platform.startswith("cygwin"):
        cfg.find_program('nm', var='NM')
    else:
        cfg.find_program('%s-nm' % cfg.env.TOOLCHAIN, var='NM')

    if cfg.env.COMPILER_CC == 'clang':
        cfg.env.CFLAGS += cfg.env.CLANG_FLAGS
        cfg.env.LINKFLAGS_cprogram += cfg.env.CLANG_FLAGS

    if cfg.env.COMPILER_CXX == 'clang++':
        cfg.env.CXXFLAGS += cfg.env.CLANG_FLAGS
        cfg.env.LINKFLAGS_cxxprogram += cfg.env.CLANG_FLAGS
