#!/usr/bin/env python
# encoding: utf-8

"""
gtest is a Waf tool for test builds in Ardupilot
"""

from waflib import Utils
from waflib.Configure import conf

import boards

def configure(cfg):
    cfg.env.HAS_GTEST = False
    if cfg.options.disable_tests:
        return

    board = cfg.get_board()
    if isinstance(board, boards.px4) or isinstance(board, boards.chibios):
        # toolchain is currently broken for gtest
        cfg.msg(
            'Gtest',
            'STM32 boards currently don\'t support compiling gtest',
            color='YELLOW',
        )
        return

    if cfg.env.STATIC_LINKING:
        # gtest uses a function (getaddrinfo) that is supposed to be linked
        # dynamically
        cfg.msg(
            'Gtest',
            'statically linked tests not supported',
            color='YELLOW',
        )
        return

    cfg.env.append_value('GIT_SUBMODULES', 'gtest')
    cfg.env.HAS_GTEST = True

@conf
def libgtest(bld, **kw):
    kw['cxxflags'] = Utils.to_list(kw.get('cxxflags', [])) + ['-Wno-undef']
    kw.update(
        source='modules/gtest/src/gtest-all.cc',
        target='gtest/gtest',
        includes='modules/gtest/ modules/gtest/include',
        export_includes='modules/gtest/include',
        name='GTEST',
    )
    return bld.stlib(**kw)
