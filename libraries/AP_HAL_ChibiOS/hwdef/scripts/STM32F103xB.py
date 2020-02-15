#!/usr/bin/env python
'''
these tables are generated from the STM32 datasheets for the
STM32F103x8
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

    'RAM_MAP' : [
        (0x20000000, 20, 1), # main memory, DMA safe
    ]
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
    

DMA_Map = {
    # format is (DMA_TABLE, StreamNum, Channel)
    "ADC1"          :   [(1,1,0)],
    "TIM1_CH1"      :   [(1,2,0)],
    "TIM1_CH3"      :   [(1,6,0)],
    "TIM1_CH4"      :   [(1,4,0)],
    "TIM1_UP"       :   [(1,5,0)],
    "TIM2_CH1"      :   [(1,5,0)],
    "TIM2_CH2"      :   [(1,7,0)],
    "TIM2_CH3"      :   [(1,1,0)],
    "TIM2_CH4"      :   [(1,7,0)],
    "TIM2_UP"       :   [(1,2,0)],
    "TIM3_CH1"      :   [(1,6,0)],
    "TIM3_CH3"      :   [(1,2,0)],
    "TIM3_CH4"      :   [(1,3,0)],
    "TIM3_UP"       :   [(1,3,0)],
    "TIM4_CH1"      :   [(1,1,0)],
    "TIM4_CH2"      :   [(1,4,0)],
    "TIM4_CH3"      :   [(1,5,0)],
    "TIM4_UP"       :   [(1,7,0)],
    "TIM5_CH1"      :   [(2,5,0)],
    "TIM5_CH2"      :   [(2,4,0)],
    "TIM5_CH3"      :   [(2,2,0)],
    "TIM5_CH4"      :   [(2,1,0)],
    "TIM5_UP"       :   [(2,2,0)],
    "TIM8_CH1"      :   [(2,3,0)],
    "TIM8_CH2"      :   [(2,5,0)],
    "TIM8_CH3"      :   [(2,1,0)],
    "TIM8_CH4"      :   [(2,2,0)],
    "TIM8_UP"       :   [(2,1,0)],
    "TIM6_UP"       :   [(2,3,0)],
    "TIM7_UP"       :   [(2,4,0)],
    "I2C1_RX"    	:	[(1,7,0)],
    "I2C1_TX"    	:	[(1,6,0)],
    "I2C2_RX"    	:	[(1,5,0)],
    "I2C2_TX"    	:	[(1,4,0)],
    "SPI1_RX"    	:	[(1,2,0)],
    "SPI1_TX"    	:	[(1,3,0)],
    "SPI2_RX"    	:	[(1,4,0)],
    "SPI2_TX"    	:	[(1,5,0)],
    "SPI3_RX"    	:	[(2,1,0)],
    "SPI3_TX"    	:	[(2,2,0)],
    "UART4_RX"    	:	[(2,3,0)],
    "UART4_TX"    	:	[(2,5,0)],
    "USART1_RX"    	:	[(1,5,0)],
    "USART1_TX"    	:	[(1,4,0)],
    "USART2_RX"    	:	[(1,6,0)],
    "USART2_TX"    	:	[(1,7,0)],
    "USART3_RX"    	:	[(1,3,0)],
    "USART3_TX"    	:	[(1,2,0)],
}

