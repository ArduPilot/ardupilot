# encoding: utf-8

# flake8: noqa

"""
Waf tool for QURT build
"""

import os
import sys
import traceback

import hal_common

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../libraries/AP_HAL_QURT/hwdef/scripts'))
import qurt_hwdef  # noqa: E402

def generate_hwdef_h(env):
    '''run qurt_hwdef.py'''
    print(f"{env.SRCROOT=} {type(env.SRCROOT)=}")
    hwdef_dir = os.path.join(env.SRCROOT, 'libraries/AP_HAL_QURT/hwdef')

    if len(env.HWDEF) == 0:
        env.HWDEF = os.path.join(hwdef_dir, env.BOARD, 'hwdef.dat')
    hwdef_out = env.BUILDROOT
    if not os.path.exists(hwdef_out):
        os.mkdir(hwdef_out)
    hwdef = [env.HWDEF]
    if env.HWDEF_EXTRA:
        hwdef.append(env.HWDEF_EXTRA)

    hwdef_obj = qurt_hwdef.QURTHWDef(
        outdir=hwdef_out,
        hwdef=hwdef,
        quiet=False,
    )
    hwdef_obj.run()

    return hwdef_obj

def configure(cfg):
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()
    def bldpath(path):
        return bldnode.make_node(path).abspath()

    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    env.SRCROOT = srcpath('')
    env.BUILDROOT = bldpath('')

    try:
        hwdef_obj = generate_hwdef_h(env)
    except Exception:  # noqa: BLE001
        traceback.print_exc()
        cfg.fatal("Failed to process hwdef.dat")
    hal_common.process_hwdef_results(cfg, hwdef_obj)

def build(bld):
    AARCH64_SDK_DIR = "/opt/aarch64-sdk/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu"
    AARCH64_CC = AARCH64_SDK_DIR + "/bin/aarch64-linux-gnu-gcc"
    AARCH64_CXX = AARCH64_SDK_DIR + "/bin/aarch64-linux-gnu-g++"

    STUB_C = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/libslpi-link-api/src/slpi_link_stub.c')
    STUB_INC = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/libslpi-link-api/inc')
    STUB_SO = bld.bldnode.find_or_declare('slpi_link_stub.so')
    MAIN_CPP = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/src/main.cpp')
    IFADDR_CPP = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/src/getifaddrs.cpp')
    UART_CPP = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/src/RemoteUARTDriver.cpp')
    AP_HOST = bld.bldnode.find_or_declare('ardupilot')

    bld(
        # build slpi stub library
        source=[STUB_C],
        rule="%s -fPIC -o %s -shared -Wl,-soname,libslpi_link.so %s" % (AARCH64_CC, STUB_SO.abspath(), STUB_C.abspath()),
        target=[STUB_SO],
        group='dynamic_sources'
    )

    bld(
        # build ap_host
        source=[STUB_SO, MAIN_CPP, UART_CPP],
        rule="%s -I%s %s %s %s %s -lpthread -o %s" % (AARCH64_CXX, STUB_INC.abspath(),
                                          MAIN_CPP.abspath(), IFADDR_CPP.abspath(), UART_CPP.abspath(), STUB_SO.abspath(), AP_HOST.abspath()),
        target=[AP_HOST],
        group='dynamic_sources'
    )
    
