# hw definition file for processing by chibios_pins.py

include ../iomcu-dshot/hwdef.inc

undef STM32_ST_USE_TIMER

# directly define DMA channels
DMA_NOMAP 1

# only four timers on F103xB so use TIM1 for system timer
STM32_ST_USE_TIMER 1

define HAL_WITH_ESC_TELEM 1

undef PA0 PA1 PB8 PB9

# the order is important here as it determines the order that timers are used to sending dshot
# TIM4 needs to go first so that TIM4_UP can be freed up to be used by input capture for TIM2
PB8 TIM4_CH3 TIM4 PWM(3) GPIO(103) BIDIR UP_SHARED
PB9 TIM4_CH4 TIM4 PWM(4) GPIO(104)
PA0 TIM2_CH1 TIM2 PWM(1) GPIO(101)
PA1 TIM2_CH2 TIM2 PWM(2) GPIO(102) BIDIR # DMA channel 7, shared with TIM4_UP and USART2_TX

# currently no support for having mixed outputs on the same timer
# PB1 TIM3_CH4 TIM3 PWM(8) GPIO(108) BIDIR # DMA channel 3, shared with TIM3_UP

# TIM2_UP - (1,2)
# TIM4_UP - (1,7)
# TIM3_UP - (1,3)

# TIM2_CH2 (PWM 1/2)
define STM32_TIM_TIM2_CH2_DMA_STREAM STM32_DMA_STREAM_ID(1, 7)
define STM32_TIM_TIM2_CH2_DMA_CHAN 1

# TIM4_CH3 (PWM 3/4)
define STM32_TIM_TIM4_CH3_DMA_STREAM STM32_DMA_STREAM_ID(1, 5)
define STM32_TIM_TIM4_CH3_DMA_CHAN 1

# TIM3_CH4 (PWM 7-8)
define STM32_TIM_TIM3_CH4_DMA_STREAM STM32_DMA_STREAM_ID(1, 3)
define STM32_TIM_TIM3_CH4_DMA_CHAN 1

# STM32_UART_USART2_TX_DMA_STREAM defined as (1, 7) by ChibiOS

undef SHARED_DMA_MASK
define SHARED_DMA_MASK (1U<<STM32_TIM_TIM4_UP_DMA_STREAM | 1U<<STM32_TIM_TIM2_CH2_DMA_STREAM | 1U<<STM32_UART_USART2_TX_DMA_STREAM)

undef MAIN_STACK
MAIN_STACK 0x400
