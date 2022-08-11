#!/usr/bin/env python
# encoding: utf-8

"""
Waf tool for WolfSSL build
"""

from waflib import Errors, Logs, Task, Utils
from waflib.TaskGen import after_method, before_method, feature, after

import os
import shutil
import sys
import re
import pickle

def configure(cfg):
    cfg.find_program('autoconf')
    env = cfg.env
    bldnode = cfg.bldnode.make_node(cfg.variant)
    def srcpath(path):
        return cfg.srcnode.make_node(path).abspath()

    def bldpath(path):
        return bldnode.make_node(path).abspath()
    env.WOLFSSL_ROOT =   srcpath('modules/wolfssl')
    env.WOLFSSL_BUILDDIR = bldpath('.')
    if env.TOOLCHAIN == 'native':
        env.WOLFSSL_USER_HEADERS = cfg.path.find_node('libraries/AP_HAL_SITL/wolfssl/').abspath()
    else:
        env.WOLFSSL_USER_HEADERS = ""
    env.EXTRA_CFLAGS = cfg.env.CPU_FLAGS + ['-fno-builtin']
    if env.STM32_HASH_AVAILABLE:
        env.STM32_HASH = "-DSTM32_HASH"
    else:
        env.STM32_HASH = ""

    if cfg.env.DEBUG:
        env.EXTRA_CFLAGS += ['-g']
    try:
        cmd = "cd {0} && ./autogen.sh".format(env.WOLFSSL_ROOT)
        import subprocess
        ret = subprocess.call(cmd, shell=True)
    except Exception:
        cfg.fatal("Failed to setup autoconf in wolfssl")
    if ret != 0:
        cfg.fatal("Failed to setup autoconf in wolfssl ret=%d" % ret)

def build(bld):
    bld.add_group('wolfssl')
    if bld.env.TOOLCHAIN == 'native':
        wolfssl_source = [bld.path.find_node('libraries/AP_HAL_SITL/wolfssl/user_settings.h')]
        wolfssl_task = bld(
            #build libwolfssl.a from WolfSSL sources
            rule="cd ${WOLFSSL_ROOT} && ./configure --prefix=${WOLFSSL_BUILDDIR} CFLAGS=\"${EXTRA_CFLAGS} -DWOLFSSL_USER_SETTINGS ${STM32_HASH} -I${WOLFSSL_USER_HEADERS} -Wno-error=unused-parameter\" --disable-filesystem --disable-shared --enable-sha --enable-keygen --enable-rsa --disable-oldnames && make clean && make && make install",
            group='dynamic_sources',
            source=wolfssl_source,
            target=bld.bldnode.find_or_declare('lib/libwolfssl.a')
        )
    elif bld.env.TOOLCHAIN == 'arm-none-eabi':
        wolfssl_source = [bld.path.find_node('libraries/AP_HAL_ChibiOS/hwdef/common/user_settings.h'),
                          bld.bldnode.find_or_declare('modules/ChibiOS/libch.a'),
                          bld.bldnode.find_or_declare('modules/ChibiOS/include_dirs')]
        wolfssl_task = bld(
            #build libwolfssl.a from WolfSSL sources
            rule="cd ${WOLFSSL_ROOT} && ./configure --enable-static --disable-shared --host=arm-none-eabi CC=arm-none-eabi-gcc AR=arm-none-eabi-ar STRIP=arm-none-eabi-strip RANLIB=arm-none-eabi-ranlib --prefix=${WOLFSSL_BUILDDIR} CFLAGS=\"--specs=nosys.specs -mthumb ${EXTRA_CFLAGS} -DWOLFSSL_USER_SETTINGS -I${BUILDROOT} -I${SRCROOT}/libraries/AP_HAL_ChibiOS/hwdef/common/ ${STM32_HASH} -Wno-error=unused-parameter -Wno-error=strict-prototypes\" --disable-filesystem --enable-sha --enable-keygen --enable-rsa --disable-oldnames && make clean && make && make install",
            group='dynamic_sources',
            source=wolfssl_source,
            target=bld.bldnode.find_or_declare('lib/libwolfssl.a')
        )
    wolfssl_task.name = "WolfSSL_lib"
    bld.env.LIB += ['wolfssl']
    bld.env.INCLUDES += ['include']
    bld.env.INCLUDES += [bld.env.WOLFSSL_USER_HEADERS]
