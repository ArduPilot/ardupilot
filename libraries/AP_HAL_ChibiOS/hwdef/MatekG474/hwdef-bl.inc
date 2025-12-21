# hw definition file Matek G474 CAN node

# MCU class and specific type
MCU STM32G474 STM32G474xx

# crystal frequency
OSCILLATOR_HZ 8000000

# board ID. See Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_MatekG474

# setup build for a peripheral firmware
env AP_PERIPH 1

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 32
FLASH_SIZE_KB 512

# reserve some space for params
APP_START_OFFSET_KB 4

# ---------------------------------------------
PC13 LED_BOOTLOADER OUTPUT LOW  # blue
define HAL_LED_ON 0

# SWD debugging
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# enable CAN support
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1

PB5  CAN2_RX CAN2
PB6  CAN2_TX CAN2


# ---------------------------------------------

# make bl baudrate match debug baudrate for easier debugging
define BOOTLOADER_BAUDRATE 57600

# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 2500

define HAL_USE_EMPTY_IO TRUE
define DMA_RESERVE_SIZE 0

define HAL_USE_SERIAL FALSE

# Add CS pins to ensure they are high in bootloader
PB12 MAG_CS CS
PC14 SPARE_CS CS
