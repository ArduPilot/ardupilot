# HW definition file for 3DR CAN Nodes (STM32L431)

# MCU class and specific type
MCU STM32L431 STM32L431xx

# use LSE clock source
OSCILLATOR_HZ 32768

# Board ID: Inheriting boards will set their own ID. 

# setup build for a peripheral firmware
env AP_PERIPH 1

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 48
FLASH_SIZE_KB 256

# reserve some space for params
APP_START_OFFSET_KB 16

STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600

# order of UARTs
SERIAL_ORDER USART1

# USART1
PB6 USART1_TX USART1 SPEED_HIGH NODMA
PB7 USART1_RX USART1 SPEED_HIGH NODMA
define HAL_USE_SERIAL TRUE
define STM32_SERIAL_USE_USART1 TRUE
define HAL_USE_EMPTY_IO TRUE

define DMA_RESERVE_SIZE 0

# a fault LED
PB1 LED_BOOTLOADER OUTPUT LOW
define HAL_LED_ON 1

# CAN
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1
CAN_ORDER 1

MAIN_STACK 0x800
PROCESS_STACK 0x800

# CS Pins need to be HIGH in bootloader
PA4 MCU_CS CS

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# make bl baudrate match debug baudrate for easier debugging
define BOOTLOADER_BAUDRATE 57600

# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 1000
