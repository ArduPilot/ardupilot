# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32F103 STM32F103xB

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 23

# reserve some space for params
APP_START_OFFSET_KB 2

# board ID. See Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_F103_PERIPH

# setup build for a peripheral firmware
env AP_PERIPH 1

# crystal frequency
OSCILLATOR_HZ 8000000

define CH_CFG_ST_FREQUENCY 1000

# assume 128k flash part
FLASH_SIZE_KB 128

define HAL_USE_SERIAL FALSE

PA4 LED_BOOTLOADER OUTPUT LOW
define HAL_LED_ON 1

define CH_CFG_ST_TIMEDELTA 0
#define CH_CFG_USE_DYNAMIC FALSE
define HAL_USE_EMPTY_IO TRUE
define PORT_INT_REQUIRED_STACK 64

#defined to turn off undef warnings
define __FPU_PRESENT 0


define DMA_RESERVE_SIZE 0

MAIN_STACK 0x800
PROCESS_STACK 0x800

# enable CAN support
PA11 CAN_RX CAN
PA12 CAN_TX CAN
define HAL_USE_CAN TRUE
define STM32_CAN_USE_CAN1 TRUE

# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 1000

# use PB6 (normally I2C1_SCL) as "hold in bootloader" pin
# this has a hw pullup, so if we set it as input floating
# and look for it low then we know user has pulled it down and
# want to stay in the bootloader
PB6 STAY_IN_BOOTLOADER INPUT FLOATING


# Add CS pins to ensure they are high in bootloader
PB0 MAG_CS CS
