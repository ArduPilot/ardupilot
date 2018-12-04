#!/usr/bin/env python
'''
these tables are generated from the STM32 datasheets for the
STM32F103T8
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
    'B': 8,
    'C': 0,
    'D': 2,
    'E': 0
}

# MCU parameters
mcu = {
    # location of MCU serial number
    'UDID_START' : 0x1FFFF7E8,

    # base address of main memory
    'RAM_BASE_ADDRESS' : 0x20000000,

    # size of main memory
    'RAM_SIZE_KB' : 64, 
    
}


# AFNUM List:
#  0 = System pins: SWDIO, SWCLK, MCO*, NJTRST, TRACE*, JTD*, RTC*
#  1 = TIM1* / TIM2*
#  2 = TIM3* / TIM4* / TIM5*
#  3 = TIM8* / TIM9* / TIM10* / TIM11*
#  4 = I2C1* / I2C2* / I2C3*
#  5 = SPI1* / SPI2*
#  6 = SPI3*
#  7 = USART1* / USART2* / USART3* / (I2C3EXT_SD)
#  8 = UART4* / UART5* / USART6* 
#  9 = TIM12* / TIM13* / TIM14* / CAN1* / CAN2*
# 10 = OTG_FS* / OTG_HS_ULPI*
# 11 = ETH*
# 12 = SDIO* / FSMC*
# 13 = DCMI* 
# 14 = n/a
# 15 = EVENTOUT


AltFunction_map = {
	# format is PIN:FUNCTION : AFNUM
	# made manually from STM32F103T8 datasheet
	"PA0:TIM2_CH1"					:	1,
	"PA0:TIM2_CH1"					:	1,
	"PA0:USART2_CTS"				:	7,
	"PA1:TIM2_CH2"					:	1,
	"PA1:USART2_RTS"				:	7,
	"PA2:TIM2_CH3"					:	1,
	"PA2:USART2_TX"					:	7,
	"PA3:TIM2_CH4"					:	1,
	"PA3:USART2_RX"					:	7,
	"PA4:SPI1_NSS"					:	5,
	"PA4:USART2_CK"					:	1,
	"PA5:SPI1_SCK"					:	5,
	"PA6:SPI1_MISO"					:	5,
	"PA6:TIM3_CH1"					:	2,
	"PA7:SPI1_MOSI"					:	5,
	"PA7:TIM3_CH2"					:	2,
	"PA8:USART1_CK"					:	7,
	"PA8:TIM1_CH1"					:	1,
	"PA8:MCO"						:	0,
	"PA9:USART1_TX"					:	7,
	"PA9:TIM1_CH2"					:	1,
	"PA10:USART1_RX"				:	1,
	"PA10:TIM1_CH3"					:	1,
	"PA11:USART1_CTS"				:	1,
	"PA11:CAN1_RX"					:	9,
	"PA11:OTG_FS_DM"				:	10,
	"PA11:TIM1_CH4"					:	1,
	"PA12:USART1_RTS"				:	1,
	"PA12:CAN1_TX"					:	9,
	"PA12:OTG_FS_DP"				:	10,
	"PA12:TIM1_ETR"					:	1,
	"PA13:JTMS-SWDIO"				:	0,
	"PA14:JTMS-SWCLK"				:	0,
	"PA15:JTDI"						:	0,
#	"PD0:"

#	"PC4"
# 	"PC5"
	"PB0:TIM3_CH3"					:	2,
	"PB1:TIM3_CH4"					:	2,
	

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
    
