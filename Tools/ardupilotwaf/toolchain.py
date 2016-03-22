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
from waflib.Tools import gxx, gcc

import os

def find_realexec_path(cfg, filename, path_list=[]):
    if not filename:
        return ''

    if not path_list:
        path_list = cfg.environ.get('PATH','').split(os.pathsep)

    for dir in path_list:
        path = os.path.abspath(os.path.expanduser(os.path.join(dir, filename)))

        if os.path.isfile(path):
            if os.path.islink(path):
                realpath = os.path.realpath(path)

                if filename not in os.path.basename(realpath):
                    continue
                else:
                    return os.path.dirname(realpath)

            else:
                return os.path.dirname(path)

    cfg.fatal('Could not find real exec path to %s in path_list %s:' % (filename, path_list))

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

def configure(cfg):
    toolchain = cfg.env.TOOLCHAIN

    if toolchain == 'native':
        cfg.load('compiler_cxx compiler_c')
        return

    cfg.msg('Using toolchain', toolchain)
    prefix = toolchain + '-'

    c_compiler = cfg.options.check_c_compiler or 'gcc'
    cxx_compiler = cfg.options.check_cxx_compiler or 'g++'

    cfg.env['AR'] = prefix + 'ar'

    if 'clang' == c_compiler or 'clang++' == cxx_compiler:
        toolchain_path = os.path.join(find_realexec_path(cfg, prefix + 'ar'),
                                      '..')
        toolchain_path = os.path.abspath(toolchain_path)
        cfg.msg('Using toolchain path', toolchain_path)

        sysroot = cfg.cmd_and_log(
            [prefix + 'gcc', '--print-sysroot'],
            quiet=Context.BOTH,
        )[:-1]
        clang_flags = [
            '--target=' + toolchain,
            '--gcc-toolchain=' + toolchain_path,
            '--sysroot=' + sysroot,
            '-B' + os.path.join(toolchain_path, 'bin')
        ]
        cfg.env.LINKFLAGS += clang_flags

    if 'clang' == c_compiler:
        cfg.env['CC'] = 'clang'
        cfg.env.CFLAGS += clang_flags

    if 'clang++' == cxx_compiler:
        cfg.env['CXX'] = 'clang++'
        cfg.env.CXXFLAGS += clang_flags

    cfg.load('compiler_cxx compiler_c')
