# HC Robotics - HCR-5xx, base Peripheral for multiple distict boards
# STM32G491KEU6

# setup build for a peripheral firmware
env AP_PERIPH 1

# MCU class and specific type
MCU STM32G4xx STM32G491xx

# Crystal Frequency
OSCILLATOR_HZ 8000000

# MCU Flash Size
FLASH_SIZE_KB 512

FLASH_RESERVE_START_KB 0
APP_START_OFFSET_KB 4
FLASH_BOOTLOADER_LOAD_KB 32

# Status LED
PA15 LED_BOOTLOADER OUTPUT HIGH # blue
define HAL_LED_ON 1

# SWD debugging
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# CAN interface
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1

# Other required defines
define HAL_USE_SERIAL 0
define DMA_RESERVE_SIZE 0

# use a smaller bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 1000
