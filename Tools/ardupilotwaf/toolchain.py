"""
WAF Tool to select the correct toolchain based on the target archtecture.
"""

from waflib import Utils

default = dict(
    CXX='g++',
    CC='gcc',
    AS='gcc',
    AR='ar',
    LD='g++',
    GDB='gdb',
    OBJCOPY='objcopy',
)

toolchains = {}

def toolchain(name, **kw):
    chain = dict(default)
    chain.update(kw)
    toolchains[name] = chain

toolchain('native')

toolchain('arm',
    CXX='arm-none-eabi-g++',
    CC='arm-none-eabi-gcc',
    AS='arm-none-eabi-gcc',
    AR='arm-none-eabi-ar',
    LD='arm-none-eabi-g++',
    GDB='arm-none-eabi-gdb',
    OBJCOPY='arm-none-eabi-objcopy',
)

toolchain('bbone',
    CXX='arm-linux-gnueabihf-g++',
    CC='arm-linux-gnueabihf-gcc',
    AS='arm-linux-gnueabihf-gcc',
    LD='arm-linux-gnueabihf-g++',
)

toolchain('rpi',
    CXX='arm-linux-gnueabihf-g++',
    CC='arm-linux-gnueabihf-gcc',
    AS='arm-linux-gnueabihf-gcc',
    AR='arm-linux-gnueabihf-ar',
    LD='arm-linux-gnueabihf-g++',
    GDB='arm-linux-gnueabihf-gdb',
    OBJCOPY='arm-linux-gnueabihf-obj',
)

toolchain('zynq',
    CXX='arm-xilinx-linux-gnueabi-g++',
    CC='arm-xilinx-linux-gnueabi-gcc',
    AS='arm-xilinx-linux-gnueabi-gcc',
    AR='arm-xilinx-linux-gnueabi-ar',
    LD='arm-xilinx-linux-gnueabi-g++',
    GDB='arm-xilinx-linux-gnueabi-gdb',
    OBJCOPY='arm-xilinx-linux-gnueabi-objcopy',
)

def configure(cfg):
    if not cfg.env.TOOLCHAIN:
        cfg.env.TOOLCHAIN = 'native'
    else:
        cfg.env.TOOLCHAIN = Utils.to_list(cfg.env.TOOLCHAIN)[0]

    name = cfg.env.TOOLCHAIN
    cfg.msg('Using toolchain', name)

    if name not in toolchains:
        cfg.fatal('Toolchain %s not found' % name)

    chain = toolchains[name]

    for k in default:
        cfg.env.append_value(k, chain[k])
