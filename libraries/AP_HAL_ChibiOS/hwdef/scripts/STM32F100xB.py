#!/usr/bin/env python
'''
these tables are generated from the STM32 datasheets for the
STM32F100xB
'''

# additional build information for ChibiOS
build = {
    "CHIBIOS_STARTUP_MK"  : "os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f1xx.mk",
    "CHIBIOS_PLATFORM_MK" : "os/hal/ports/STM32/STM32F1xx/platform.mk",
    "CHPRINTF_USE_FLOAT" : 'no',
    "USE_FPU" : 'no'
    }

pincount = {
    'A': 16,
    'B': 16,
    'C': 16,
    'D': 16,
    'E': 16
}

# MCU parameters
mcu = {
    # location of MCU serial number
    'UDID_START' : 0x1FFFF7E8,

    # base address of main memory
    'RAM_BASE_ADDRESS' : 0x20000000,

    # size of main memory
    'RAM_SIZE_KB' : 8, 
    
}

ADC1_map = {
	# format is PIN : ADC1_CHAN
    "PA0"	:	0,
	"PA1"	:	1,
	"PA2"	:	2,
	"PA3"	:	3,
	"PA4"	:	4,
	"PA5"	:	5,
	"PA6"	:	6,
	"PA7"	:	7,
	"PB0"	:	8,
	"PB1"	:	9,
	"PC0"	:	10,
	"PC1"	:	11,
	"PC2"	:	12,
	"PC3"	:	13,
	"PC4"	:	14,
	"PC5"	:	15,
}
    
