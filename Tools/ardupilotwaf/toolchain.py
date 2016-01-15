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

import sys

from waflib import Utils

suffixes = dict(
    CXX='g++',
    CC='gcc',
    AS='gcc',
    AR='ar',
    LD='g++',
    GDB='gdb',
    OBJCOPY='objcopy',
)

def configure(cfg):
    if not cfg.env.TOOLCHAIN:
        cfg.env.TOOLCHAIN = 'native'
        prefix = ''
    else:
        cfg.env.TOOLCHAIN = Utils.to_list(cfg.env.TOOLCHAIN)[0]
        if sys.platform == 'win32':
            if cfg.env.TOOLCHAIN == 'arm-linux-gnueabihf':
                cfg.env.TOOLCHAIN = 'arm-none-eabi'
        cfg.msg('Using toolchain prefix', cfg.env.TOOLCHAIN)
        prefix = cfg.env.TOOLCHAIN + '-'

    if sys.platform == 'win32':
        """
        Removes msvc check.
        """
        from waflib.Tools.compiler_cxx import cxx_compiler
        from waflib.Tools.compiler_c import c_compiler
        cxx_compiler['win32'] = ['g++', 'clang++']
        c_compiler['win32'] = ['gcc', 'clang']
        exe = '.exe'
    else:
        exe = ''
    
    for k in suffixes:
        cfg.env.append_value(k, prefix + suffixes[k] + exe)
