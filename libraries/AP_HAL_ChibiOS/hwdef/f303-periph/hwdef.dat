# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32F303 STM32F303xC

# bootloader starts firmware at 22k + 4k (STORAGE_FLASH)
FLASH_RESERVE_START_KB 26

# store parameters in pages 11 and 12
define STORAGE_FLASH_PAGE 11
define HAL_STORAGE_SIZE 800

# board ID for firmware load
APJ_BOARD_ID 1004

# setup build for a peripheral firmware
env AP_PERIPH 1

# enable watchdog
define HAL_WATCHDOG_ENABLED_DEFAULT true

# crystal frequency
OSCILLATOR_HZ 8000000

define CH_CFG_ST_FREQUENCY 1000
define CH_CFG_ST_TIMEDELTA 0

# assume the 256k flash part for now
FLASH_SIZE_KB 256


# order of UARTs
SERIAL_ORDER USART2 EMPTY EMPTY USART1

# a LED to flash
PA4 LED OUTPUT LOW

define HAL_CAN_POOL_SIZE 6000

# USART1, connected to GPS
PA9  USART1_TX USART1 SPEED_HIGH
PA10 USART1_RX USART1 SPEED_HIGH

# USART2 for debug
PA2 USART2_TX USART2 SPEED_HIGH NODMA
PA3 USART2_RX USART2 SPEED_HIGH NODMA

# only one I2C bus in normal config
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1

PB0 MAG_CS CS

# spi bus for compass
PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PA7 SPI1_MOSI SPI1

# analog input
# PA5 VIN5 ADC1
define HAL_USE_ADC FALSE
define STM32_ADC_USE_ADC1 FALSE
define HAL_DISABLE_ADC_DRIVER TRUE

define HAL_NO_GPIO_IRQ
define SERIAL_BUFFERS_SIZE 512
define PORT_INT_REQUIRED_STACK 64

# avoid timer and RCIN threads to save memory
define HAL_NO_RCIN_THREAD
define HAL_NO_RCOUT_THREAD

define HAL_USE_RTC FALSE
define DISABLE_SERIAL_ESC_COMM TRUE
define NO_DATAFLASH TRUE

define DMA_RESERVE_SIZE 0

define PERIPH_FW TRUE

# MAIN_STACK is stack of initial thread and of ISRs
MAIN_STACK 0x300

# PROCESS_STACK controls stack for main thread
PROCESS_STACK 0x800
define HAL_DISABLE_LOOP_DELAY

# enable CAN support
PA11 CAN_RX CAN
PA12 CAN_TX CAN

# debugger support
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

define HAL_USE_I2C TRUE
define STM32_I2C_USE_I2C1 TRUE

define HAL_UART_MIN_TX_SIZE 256
define HAL_UART_MIN_RX_SIZE 128

define HAL_UART_STACK_SIZE 0x300

define STORAGE_THD_WA_SIZE 512
define IO_THD_WA_SIZE      512

define HAL_NO_GCS

define HAL_MINIMIZE_FEATURES 0

define HAL_BUILD_AP_PERIPH

# only one I2C bus
I2C_ORDER I2C1

define HAL_I2C_CLEAR_ON_TIMEOUT 0

define HAL_DEVICE_THREAD_STACK 0x200

define AP_PARAM_MAX_EMBEDDED_PARAM 512

define HAL_I2C_INTERNAL_MASK 0

# disable dual GPS and GPS blending to save flash space
define GPS_MAX_RECEIVERS 1
define GPS_MAX_INSTANCES 1
define HAL_COMPASS_MAX_SENSORS 1
define RANGEFINDER_MAX_INSTANCES 1

# use the app descriptor needed by MissionPlanner for CAN upload
env APP_DESCRIPTOR MissionPlanner

# reserve 256 bytes for comms between app and bootloader
RAM_RESERVE_START 256

# keep ROMFS uncompressed as we don't have enough RAM
# to uncompress the bootloader at runtime
env ROMFS_UNCOMPRESSED True

# reduce the number of CAN RX Buffer
define HAL_CAN_RX_QUEUE_SIZE 64
