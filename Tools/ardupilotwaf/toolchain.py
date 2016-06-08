"""
WAF Tool to select the correct toolchain based on the target archtecture.

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

import os
import re

def _set_toolchain_prefix_wrapper(tool_module, var, compiler_names):
    original_configure = tool_module.configure
    def new_configure(cfg):
        if cfg.env.TOOLCHAIN == 'native':
            original_configure(cfg)
            return

        last_exception = None
        for name in compiler_names:
            cfg.env.stash()
            try:
                cfg.env[var] = '%s-%s' % (cfg.env.TOOLCHAIN, name)
                original_configure(cfg)
            except Errors.ConfigurationError as e:
                cfg.env.revert()
                last_exception = e
            else:
                cfg.env.commit()
                return
        raise last_exception
    tool_module.configure = new_configure

_set_toolchain_prefix_wrapper(gxx, 'CXX', ('g++', 'c++'))
_set_toolchain_prefix_wrapper(gcc, 'CC', ('gcc', 'cc'))

def _clang_cross_support(cfg):
    if _clang_cross_support.called:
        return

    prefix = cfg.env.TOOLCHAIN + '-'

    try:
        cfg.find_program(prefix + 'gcc', var='CROSS_GCC')
    except Errors.ConfigurationError as e:
        cfg.fatal('toolchain: clang: couldn\'t find cross GCC', ex=e)

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
        cfg.fatal('toolchain: clang: couldn\'t find toolchain path', ex=e)

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
        compiler_c.c_compiler[k] = [c for c in l if c in compilers]

def _filter_supported_cxx_compilers(*compilers):
    for k in compiler_cxx.cxx_compiler:
        l = compiler_cxx.cxx_compiler[k]
        compiler_cxx.cxx_compiler[k] = [c for c in l if c in compilers]

@conf
def find_toolchain_program(cfg, filename, **kw):
    filename = Utils.to_list(filename)

    if not kw.get('var', ''):
        # just copy from the original implementation
        kw['var'] = re.sub(r'[-.]', '_', filename[0].upper())

    if cfg.env.TOOLCHAIN != 'native':
        for i, name in enumerate(filename):
            filename[i] = '%s-%s' % (cfg.env.TOOLCHAIN, name)

    return cfg.find_program(filename, **kw)

def configure(cfg):
    if cfg.env.TOOLCHAIN == 'native':
        cfg.load('compiler_cxx compiler_c')
        return

    _filter_supported_c_compilers('gcc', 'clang')
    _filter_supported_cxx_compilers('g++', 'clang++')

    cfg.env.AR = cfg.env.TOOLCHAIN + '-ar'
    cfg.env.PKGCONFIG = cfg.env.TOOLCHAIN + '-pkg-config'
    cfg.msg('Using toolchain', cfg.env.TOOLCHAIN)
    cfg.load('compiler_cxx compiler_c')

    if cfg.env.COMPILER_CC == 'clang':
        cfg.env.CFLAGS += cfg.env.CLANG_FLAGS
        cfg.env.LINKFLAGS_cprogram += cfg.env.CLANG_FLAGS

    if cfg.env.COMPILER_CXX == 'clang++':
        cfg.env.CXXFLAGS += cfg.env.CLANG_FLAGS
        cfg.env.LINKFLAGS_cxxprogram += cfg.env.CLANG_FLAGS
