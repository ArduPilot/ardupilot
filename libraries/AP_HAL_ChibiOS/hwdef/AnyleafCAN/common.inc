# See AeroFox-PMU's config for an example

# board ID for firmware load
APJ_BOARD_ID 1125

# MCU class and specific type
MCU STM32G4xx STM32G431xx

FLASH_SIZE_KB 128

# crystal frequency, setup to use external oscillator
OSCILLATOR_HZ 16000000

# setup build for a peripheral firmware
env AP_PERIPH 1

# CAN
PA11 CAN1_RX CAN1
PB9 CAN1_TX CAN1

CAN_ORDER 1

# SWD debugging
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

define HAL_NO_GPIO_IRQ
define DMA_RESERVE_SIZE 0
define HAL_DISABLE_LOOP_DELAY
define HAL_USE_SERIAL FALSE