# encoding: utf-8

# flake8: noqa

"""
Waf tool for QURT build
"""

def build(bld):
    AARCH64_SDK_DIR = "/opt/aarch64-sdk/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu"
    AARCH64_CC = AARCH64_SDK_DIR + "/bin/aarch64-linux-gnu-gcc"
    AARCH64_CXX = AARCH64_SDK_DIR + "/bin/aarch64-linux-gnu-g++"

    STUB_C = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/libslpi-link-api/src/slpi_link_stub.c')
    STUB_INC = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/libslpi-link-api/inc')
    STUB_SO = bld.bldnode.find_or_declare('slpi_link_stub.so')
    MAIN_CPP = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/src/main.cpp')
    IFADDR_CPP = bld.srcnode.make_node('libraries/AP_HAL_QURT/ap_host/src/getifaddrs.cpp')
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
        source=[STUB_SO, MAIN_CPP],
        rule="%s -I%s %s %s %s -lpthread -o %s" % (AARCH64_CXX, STUB_INC.abspath(),
                                          MAIN_CPP.abspath(), IFADDR_CPP.abspath(), STUB_SO.abspath(), AP_HOST.abspath()),
        target=[AP_HOST],
        group='dynamic_sources'
    )
    
