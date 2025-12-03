# encoding: utf-8

"""
Waf tool for Linux build

AP_FLAKE8_CLEAN
"""

from waflib.TaskGen import after_method, feature

import os
import sys
import traceback

import hal_common

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_Linux/hwdef/scripts'))
import linux_hwdef  # noqa:501


@feature('linux_ap_program')
@after_method('process_source')
def linux_firmware(self):
    pass


def configure(cfg):

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()

    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    env.SRCROOT = srcpath('')
    env.BUILDROOT = bldpath('')

    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.AP_PROGRAM_FEATURES += ['linux_ap_program']

    try:
        hwdef_obj = generate_hwdef_h(env)
    except Exception:
        traceback.print_exc()
        cfg.fatal("Failed to process hwdef.dat")
    hal_common.process_hwdef_results(cfg, hwdef_obj)


def generate_hwdef_h(env):
    '''run linux_hwdef.py'''
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_Linux/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)

    hwdef_obj = linux_hwdef.LinuxHWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
    )
    hwdef_obj.run()

    return hwdef_obj


def pre_build(bld):
    '''pre-build hook to change dynamic sources'''
    if bld.env.HAL_NUM_CAN_IFACES:
        bld.get_board().with_can = True
    if bld.env.WITH_LITTLEFS:
        bld.get_board().with_littlefs = True
