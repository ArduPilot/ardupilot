#!/usr/bin/env python
# encoding: utf-8

"""
gtest is a Waf tool for test builds in Ardupilot
"""

from waflib.Configure import conf

def configure(cfg):
    cfg.env.HAS_GTEST = False

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
    kw.update(
        source='modules/gtest/src/gtest-all.cc',
        target='gtest/gtest',
        includes='modules/gtest/ modules/gtest/include',
        export_includes='modules/gtest/include',
        name='GTEST',
    )
    return bld.stlib(**kw)
