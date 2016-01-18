#!/usr/bin/env python
# encoding: utf-8

"""
gtest is a Waf tool for test builds in Ardupilot
"""

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

    cfg.start_msg('Checking for gtest submodule')
    readme = cfg.srcnode.find_resource('modules/gtest/README')
    if not readme:
        cfg.end_msg('not initialized', color='YELLOW')
        return
    cfg.end_msg('yes')

    cfg.env.HAS_GTEST = True

def build(bld):
    bld.stlib(
        source='modules/gtest/src/gtest-all.cc',
        target='gtest/gtest',
        includes='modules/gtest/ modules/gtest/include',
        export_includes='modules/gtest/include',
        name='GTEST',
    )
