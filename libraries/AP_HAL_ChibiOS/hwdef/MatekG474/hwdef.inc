# hw definition file for Matek G474 CAN node

# MCU class and specific type
MCU STM32G474 STM32G474xx

# bootloader starts firmware at 32k + 4k (STORAGE_FLASH)
FLASH_RESERVE_START_KB 36
FLASH_SIZE_KB 512

# store parameters in pages
STORAGE_FLASH_PAGE 16
define HAL_STORAGE_SIZE 1800

# ChibiOS system timer
STM32_ST_USE_TIMER 15
define CH_CFG_ST_RESOLUTION 16

# board ID. See Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_MatekG474

# crystal frequency
OSCILLATOR_HZ 8000000


env AP_PERIPH 1

define SERIAL_BUFFERS_SIZE 512
# stack for fast interrupts
define PORT_INT_REQUIRED_STACK 64

define DMA_RESERVE_SIZE 2048

# don't share any DMA channels (there are enough for everyone)
DMA_NOSHARE *


# keep ROMFS uncompressed as we don't have enough RAM
# to uncompress the bootloader at runtime
env ROMFS_UNCOMPRESSED True

define AP_PARAM_MAX_EMBEDDED_PARAM 512

# allow for reboot command for faster development
# define HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT 0


# blue LED0 marked as ACT
PC13 LED OUTPUT HIGH
define HAL_LED_ON 1

# --------------------- SPI1 RM3100 -----------------------
PB13 SPI2_SCK SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2
PB12 MAG_CS CS
PC14 SPARE_CS CS

# ---------------------- I2C bus ------------------------
I2C_ORDER I2C1 I2C2

# SWD debugging, disabled for I2C1
# PA13 JTMS-SWDIO SWD
# PA14 JTCK-SWCLK SWD

PA13 I2C1_SCL I2C1
PA14 I2C1_SDA I2C1

PC4 I2C2_SCL I2C2
PA8 I2C2_SDA I2C2

define HAL_I2C_CLEAR_ON_TIMEOUT 0
define HAL_I2C_INTERNAL_MASK 0

define HAL_USE_I2C TRUE

# ---------------------- CAN bus -------------------------
PA11 CAN1_RX CAN1
PA12 CAN1_TX CAN1
PB5  CAN2_RX CAN2
PB6  CAN2_TX CAN2

define HAL_CAN_POOL_SIZE 6000

# ---------------------- UARTs ---------------------------
# make the UARTn numbers in parameters match the silkscreen
#           | sr0  | sr1 | sr2  | sr3  | sr4  |
SERIAL_ORDER EMPTY  USART1 USART2 USART3 UART4

PA9  USART1_TX USART1 SPEED_HIGH
PA10 USART1_RX USART1 SPEED_HIGH

PB3 USART2_TX USART2 SPEED_HIGH
PB4 USART2_RX USART2 SPEED_HIGH

PB10  USART3_TX USART3 SPEED_HIGH NODMA
PB11  USART3_RX USART3 SPEED_HIGH NODMA

PC10  UART4_TX UART4 SPEED_HIGH 
PC11  UART4_RX UART4 SPEED_HIGH

STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600

# ----------- ADC
define HAL_USE_ADC FALSE
define STM32_ADC_USE_ADC1 FALSE
define HAL_DISABLE_ADC_DRIVER TRUE
