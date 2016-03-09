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

from waflib import Utils
import os

suffixes = dict(
    CXX='g++',
    CC='gcc',
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
    if not cfg.env.TOOLCHAIN:
        cfg.env.TOOLCHAIN = 'native'
        prefix = ''
    else:
        cfg.env.TOOLCHAIN = Utils.to_list(cfg.env.TOOLCHAIN)[0]
        cfg.msg('Using toolchain prefix', cfg.env.TOOLCHAIN)
        prefix = cfg.env.TOOLCHAIN + '-'

    if cfg.env.TOOLCHAIN != 'native':
        for k in suffixes:
            cfg.env.append_value(k, prefix + suffixes[k])

        c_compiler = cfg.options.check_c_compiler or cfg.environ.get('CC', '')
        cxx_compiler = cfg.options.check_cxx_compiler or cfg.environ.get('CXX', '')
        cfg.environ.pop('CC', None)
        cfg.environ.pop('CXX', None)

        if 'clang' in c_compiler:
            toolchain_path = os.path.abspath(os.path.join(find_realexec_path(cfg, cfg.env['CC'][0]), '..'))

            cfg.env['CC'] = [ c_compiler ]
            cfg.env.CFLAGS += [
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]
            cfg.env.LINKFLAGS += [
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]

        if 'clang++' in cxx_compiler:
            toolchain_path = os.path.abspath(os.path.join(find_realexec_path(cfg, cfg.env['CXX'][0]), '..'))

            cfg.env['CXX'] = [ cxx_compiler ]
            cfg.env.CXXFLAGS += [
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]
            cfg.env.LINKFLAGS += [
                '--target=' + cfg.env.TOOLCHAIN,
                '--gcc-toolchain=' + toolchain_path,
                '--sysroot=' + os.path.join(toolchain_path, cfg.env.TOOLCHAIN, 'libc'),
                '-B' + os.path.join(toolchain_path, 'bin')
            ]