# encoding: utf-8

"""
Adds support for building littlefs as part of a Waf build
"""

from waflib.Configure import conf

def configure(cfg):
    cfg.env.append_value('GIT_SUBMODULES', 'littlefs')
    cfg.env.prepend_value('INCLUDES', [
        cfg.srcnode.abspath() + '/modules/littlefs/',
    ])


@conf
def littlefs(bld, **kw):
    kw.update(
        name='littlefs',
        source=['modules/littlefs/lfs.c', 'modules/littlefs/lfs_util.c'],
        target='littlefs',
        defines=['LFS_NO_DEBUG'],
        cflags=['-Wno-format', '-Wno-format-extra-args', '-Wno-shadow', '-Wno-unused-function', '-Wno-missing-declarations']
    )
    return bld.stlib(**kw)
