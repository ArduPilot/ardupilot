# hw definition file Vimdrones L431 CAN node

# MCU class and specific type
MCU STM32L431 STM32L431xx

# crystal frequency
OSCILLATOR_HZ 24000000
define STM32_HSE_BYPASS

# board ID for firmware load
APJ_BOARD_ID AP_HW_VIMDRONES_PERIPH

# setup build for a peripheral firmware
env AP_PERIPH 1

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 36
FLASH_SIZE_KB 256

# reserve some space for params
APP_START_OFFSET_KB 4

# enable CAN support
define CAN_APP_NODE_NAME "com.vimdrones.periph_bl"
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1
# PC13 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

CAN_ORDER 1

# ---------------------------------------------

# make bl baudrate match debug baudrate for easier debugging
define BOOTLOADER_BAUDRATE 57600

# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 1000

define HAL_USE_SERIAL FALSE

MAIN_STACK 0x800
PROCESS_STACK 0x800

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD
