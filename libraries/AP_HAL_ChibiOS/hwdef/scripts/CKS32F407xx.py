#!/usr/bin/env python

'''
CKS32F407 is based on STM32F407
'''

import STM32F407xx as ST

build = ST.build
mcu = ST.mcu
DMA_Map = ST.DMA_Map
AltFunction_map = ST.AltFunction_map
ADC1_map = ST.ADC1_map

# add defines to allow for F407 ChibiOS build
mcu['DEFINES'].update({
        'STM32F4xx' : '1',
        'STM32F4xx_MCUCONF' : '1',
        'STM32F407xx' : '1',
        'STM32F407_MCUCONF' : '1',
        })

# different RAM layout from STM32F407
mcu['RAM_MAP'] = [
        (0x20000000, 128, 1), # main memory - DMA safe
        (0x20020000, 128, 2)  # CCM memory, fast but not DMA safe
    ]

