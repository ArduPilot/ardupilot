# hw definition file for Skyronin L431 CAN node

# MCU class and specific type
MCU STM32L431 STM32L431xx

# bootloader starts firmware at 36k + 4k (STORAGE_FLASH)
FLASH_RESERVE_START_KB 40
FLASH_SIZE_KB 256

# store parameters in pages 18 and 19
STORAGE_FLASH_PAGE 18
define HAL_STORAGE_SIZE 800

# ChibiOS system timer
STM32_ST_USE_TIMER 15
define CH_CFG_ST_RESOLUTION 16

# board ID. See Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_SkyroninL431

# crystal frequency
OSCILLATOR_HZ 8000000

env AP_PERIPH 1

define SERIAL_BUFFERS_SIZE 512
define PORT_INT_REQUIRED_STACK 64

define DMA_RESERVE_SIZE 0

# MAIN_STACK is stack for ISR handlers
MAIN_STACK 0x300

# PROCESS_STACK controls stack for main thread
PROCESS_STACK 0xA00

# we setup a small defaults.parm
define AP_PARAM_MAX_EMBEDDED_PARAM 512

# blue LED0 marked as ACT
PB3 LED OUTPUT HIGH
define HAL_LED_ON 1

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# --------------------- SPI1 -----------------------
PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1
PA4 MAG_CS CS
PB2 SPARE_CS CS

# ---------------------- CAN bus -------------------------
CAN_ORDER 1

PB8 CAN1_RX CAN1
PB9 CAN1_TX CAN1
# PC13 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW

define HAL_CAN_POOL_SIZE 6000

# ---------------------- UARTs ---------------------------
#           | sr0  | MSP | GPS  |
SERIAL_ORDER USART1 EMPTY EMPTY

# USART1 for debug (not broken out on production boards)
PB6  USART1_TX USART1 SPEED_HIGH
PB7  USART1_RX USART1 SPEED_HIGH

STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600

# allow for reboot command for faster development
define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0

# keep ROMFS uncompressed as we don't have enough RAM
# to uncompress the bootloader at runtime
env ROMFS_UNCOMPRESSED True

# no ADC pins
define HAL_USE_ADC FALSE
