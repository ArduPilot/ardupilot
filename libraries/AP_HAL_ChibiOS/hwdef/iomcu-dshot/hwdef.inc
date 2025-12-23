# hw definition file for processing by chibios_pins.py

include ../iomcu/hwdef.inc

undef AP_FASTBOOT_ENABLED AP_BOOTLOADER_FLASHING_ENABLED 
undef HAL_DSHOT_ENABLED

# PPM uses a DMA channel that is required for TIM2, and no remapping of PA8 is possible
undef PA8
undef STM32_ICU_USE_TIM1 RCIN_ICU_TIMER STM32_RCIN_DMA_STREAM HAL_USE_ICU
undef PORT_INT_REQUIRED_STACK MAIN_STACK
undef CH_CFG_ST_TIMEDELTA CH_CFG_ST_FREQUENCY
undef AP_HAL_SHARED_DMA_ENABLED

# TIM2_UP required for PWM1/2
define STM32_TIM_TIM2_UP_DMA_STREAM STM32_DMA_STREAM_ID(1, 2)
define STM32_TIM_TIM2_UP_DMA_CHAN 1

# TIM4_UP (PWM 3/4) cannot be used without sharing as channels used by high speed USART2 RX/TX
define AP_HAL_SHARED_DMA_ENABLED 1
define STM32_TIM_TIM4_UP_DMA_STREAM STM32_DMA_STREAM_ID(1, 7)
define STM32_TIM_TIM4_UP_DMA_CHAN 1
define SHARED_DMA_MASK (1U<<STM32_TIM_TIM4_UP_DMA_STREAM)

# TIM3_UP required for PWM5-8
define STM32_TIM_TIM3_UP_DMA_STREAM STM32_DMA_STREAM_ID(1, 3)
define STM32_TIM_TIM3_UP_DMA_CHAN 1

define HAL_WITH_DSP FALSE
define HAL_DSHOT_ENABLED TRUE
define HAL_SERIALLED_ENABLED FALSE

# Tickless mode is required in order for the virtual timers used by dshot to work correctly
STM32_ST_USE_TIMER 15
define CH_CFG_ST_RESOLUTION 16
define CH_CFG_ST_TIMEDELTA 2
define CH_CFG_ST_FREQUENCY 1000000

# ChibiOS already correctly sets stack sizes and considers 64 extra as conservative
define PORT_INT_REQUIRED_STACK 64
# "main" stack is only used for exceptions and ISR, but they can be nested
MAIN_STACK 0x180

# defines to allow the loop timing to be observed with a Saleae
#define IOMCU_LOOP_TIMING_DEBUG 1
#undef PB0 PB1
#PB0 PINIO1 OUTPUT GPIO(107) LOW
#PB1 PINIO2 OUTPUT GPIO(108) LOW
