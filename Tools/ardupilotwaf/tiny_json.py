# encoding: utf-8


"""
Adds support for building tiny-json as part of a Waf build
"""

from waflib.Configure import conf


def configure(cfg):
    cfg.env.GIT_SUBMODULES += ['tiny-json']
    cfg.env.INCLUDES += [
        cfg.srcnode.find_dir('modules/tiny-json').abspath()
    ]


@conf
def tiny_json(bld, **kw):
    kw.update(
        name='tiny-json',
        source=[bld.srcnode.make_node('modules/tiny-json/tiny-json.c')],
        target='tiny-json',
        # json_containerOf macro casts to a stricter-aligned pointer
        cflags=['-Wno-cast-align'],
    )
    return bld.stlib(**kw)
