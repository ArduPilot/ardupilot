#!/usr/bin/env python
'''
these tables are generated from the STM32 datasheets for the
STM32F42x
'''
from utils_dict import *
from utils_dict2 import *

# additional build information for ChibiOS
build = {
    "CHIBIOS_STARTUP_MK"  : "os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk",
    "CHIBIOS_PLATFORM_MK" : "os/hal/ports/STM32/STM32F4xx/platform.mk"
    }

# MCU parameters
mcu = {
    # location of MCU serial number
    'UDID_START' : 0x1FFF7A10,

    # ram map, as list of (address, size-kb, flags)
    # flags of 1 means DMA-capable
    # flags of 2 means faster memory for CPU intensive work
    'RAM_MAP' : [
        (0x20000000, 192, 1), # main memory, DMA safe
        (0x10000000,  64, 2), # CCM memory, faster, but not DMA safe
    ]
}



