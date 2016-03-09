"""
WAF Tool to select the correct toolchain based on the target archtecture.

This tool must be loaded before compiler tools. Use the environment variable
TOOLCHAIN to define the toolchain prefix.

Example::

    def configure(cfg):
        cfg.env.TOOLCHAIN = 'arm-linux-gnueabihf'
        cfg.load('toolchain')
        cfg.load('cxx_compiler')
"""

from waflib import Utils, Context

import os

suffixes = dict(
    AS='gcc',
    AR='ar',
    LD='g++',
    GDB='gdb',
    OBJCOPY='objcopy',
)

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

def configure(cfg):
    toolchain = cfg.env.TOOLCHAIN

    if toolchain != 'native':
        cfg.msg('Using toolchain prefix', toolchain)
        prefix = toolchain + '-'

        c_compiler = cfg.options.check_c_compiler or 'gcc'
        cxx_compiler = cfg.options.check_cxx_compiler or 'g++'

        if 'gcc' == c_compiler or 'g++' == cxx_compiler or 'clang' == c_compiler or 'clang++' == cxx_compiler:
            toolchain_path = os.path.abspath(os.path.join(find_realexec_path(cfg, prefix + suffixes['AR']), '..'))
            cfg.msg('Using toolchain path', toolchain_path)

            if 'gcc' == c_compiler or 'g++' == cxx_compiler:
                for k in suffixes:
                    cfg.env[k] = [prefix + suffixes[k]]

            if 'clang' == c_compiler or 'clang++' == cxx_compiler:
                sysroot = cfg.cmd_and_log([prefix + 'gcc', '--print-sysroot'], quiet=Context.BOTH)[:-1]
                clang_flags = [
                    '--target=' + toolchain,
                    '--gcc-toolchain=' + toolchain_path,
                    '--sysroot=' + sysroot,
                    '-B' + os.path.join(toolchain_path, 'bin')
                ]
                cfg.env.LINKFLAGS += clang_flags

        if 'gcc' == c_compiler:
            cfg.env['CC'] = [prefix + 'gcc']

        elif 'clang' == c_compiler:
            cfg.env['CC'] = [c_compiler]
            cfg.env['AR'] = [prefix + suffixes['AR']]
            cfg.env.CFLAGS += clang_flags

        if 'g++' == cxx_compiler:
            cfg.env['CXX'] = [prefix + 'g++']

        elif 'clang++' == cxx_compiler:
            cfg.env['CXX'] = [cxx_compiler]
            cfg.env['AR'] = [prefix + suffixes['AR']]
            cfg.env.CXXFLAGS += clang_flags