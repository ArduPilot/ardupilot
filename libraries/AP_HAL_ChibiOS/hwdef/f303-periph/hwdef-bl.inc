# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32F303 STM32F303xC

FLASH_RESERVE_START_KB 0
FLASH_BOOTLOADER_LOAD_KB 22

# reserve some space for params
APP_START_OFFSET_KB 4

# board ID for firmware load
APJ_BOARD_ID 1004

# setup build for a peripheral firmware
env AP_PERIPH 1

# crystal frequency
OSCILLATOR_HZ 8000000

define CH_CFG_ST_FREQUENCY 1000

# assume 256k flash part
FLASH_SIZE_KB 256

STDOUT_SERIAL SD1
STDOUT_BAUDRATE 57600


# order of UARTs
SERIAL_ORDER
define HAL_USE_UART FALSE

PA4 LED_BOOTLOADER OUTPUT LOW
define HAL_LED_ON 1

# USART1
PA9  USART1_TX USART1 SPEED_HIGH NODMA
PA10 USART1_RX USART1 SPEED_HIGH NODMA

# USART2
PA2 USART2_TX USART2 SPEED_HIGH NODMA
PA3 USART2_RX USART2 SPEED_HIGH NODMA

define HAL_USE_SERIAL TRUE

define STM32_SERIAL_USE_USART1 TRUE
define STM32_SERIAL_USE_USART2 TRUE
define STM32_SERIAL_USE_USART3 FALSE

define HAL_NO_GPIO_IRQ
define CH_CFG_ST_TIMEDELTA 0
#define CH_CFG_USE_DYNAMIC FALSE
define SERIAL_BUFFERS_SIZE 32
define HAL_USE_EMPTY_IO TRUE
define PORT_INT_REQUIRED_STACK 64

define DMA_RESERVE_SIZE 0

MAIN_STACK 0x800
PROCESS_STACK 0x800
define HAL_DISABLE_LOOP_DELAY

# enable CAN support
PA11 CAN_RX CAN
PA12 CAN_TX CAN

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# make bl baudrate match debug baudrate for easier debugging
define BOOTLOADER_BAUDRATE 57600

# use a small bootloader timeout
define HAL_BOOTLOADER_TIMEOUT 1000

# use PB6 (normally I2C1_SCL) as "hold in bootloader" pin
# this has a hw pullup, so if we set it as input floating
# and look for it low then we know user has pulled it down and
# want to stay in the bootloader
PB6 STAY_IN_BOOTLOADER INPUT FLOATING


# Add CS pins to ensure they are high in bootloader
PB0 MAG_CS CS
