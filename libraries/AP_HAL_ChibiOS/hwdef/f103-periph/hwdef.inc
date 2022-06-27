# hw definition file for processing by chibios_pins.py

# MCU class and specific type
MCU STM32F103 STM32F103xB

# bootloader starts firmware at 23k + 2k (STORAGE_FLASH_PAGES)
FLASH_RESERVE_START_KB 25

# store parameters in pages 23 and 24
STORAGE_FLASH_PAGE 23
define HAL_STORAGE_SIZE 800

# board ID for firmware load
APJ_BOARD_ID 1000

# setup build for a peripheral firmware
env AP_PERIPH 1

# enable watchdog

# crystal frequency
OSCILLATOR_HZ 8000000

define CH_CFG_ST_FREQUENCY 1000

# assume the 128k flash part for now
FLASH_SIZE_KB 128


# order of UARTs
SERIAL_ORDER EMPTY EMPTY EMPTY USART1

# a LED to flash
PA4 LED OUTPUT LOW

# USART1, connected to GPS
PA9  USART1_TX USART1 SPEED_HIGH NODMA
PA10 USART1_RX USART1 SPEED_HIGH NODMA

# USART2 for debug (disabled)
# PA2 USART2_TX USART2 SPEED_HIGH NODMA
# PA3 USART2_RX USART2 SPEED_HIGH NODMA

define HAL_UART_NODMA

# only one I2C bus in normal config
PB6 I2C1_SCL I2C1
PB7 I2C1_SDA I2C1

PB0 MAG_CS CS

# analog input
# PA5 VIN5 ADC1
define HAL_USE_ADC TRUE
define STM32_ADC_USE_ADC1 TRUE
define HAL_DISABLE_ADC_DRIVER TRUE

define HAL_NO_GPIO_IRQ
define CH_CFG_ST_TIMEDELTA 0
#define CH_CFG_USE_DYNAMIC FALSE
define SERIAL_BUFFERS_SIZE 512
define PORT_INT_REQUIRED_STACK 64

# avoid timer and RCIN threads to save memory
define HAL_NO_TIMER_THREAD
define HAL_NO_RCOUT_THREAD
define HAL_NO_RCIN_THREAD

#defined to turn off undef warnings
define __FPU_PRESENT 0

define HAL_USE_RTC FALSE
define DISABLE_SERIAL_ESC_COMM TRUE

define DMA_RESERVE_SIZE 0


# MAIN_STACK is stack of initial thread and ISRs
MAIN_STACK 0x200

# PROCESS_STACK controls stack for main thread
PROCESS_STACK 0xA00
define HAL_DISABLE_LOOP_DELAY

# enable CAN support
PA11 CAN_RX CAN
PA12 CAN_TX CAN



# reduce memory overheads
define HAL_CAN_POOL_SIZE 2500

define HAL_USE_I2C TRUE
define STM32_I2C_USE_I2C1 TRUE

define HAL_UART_MIN_TX_SIZE 128
define HAL_UART_MIN_RX_SIZE 128

define HAL_UART_STACK_SIZE 256
define STORAGE_THD_WA_SIZE 300
define IO_THD_WA_SIZE      300

define HAL_NO_MONITOR_THREAD


# only one I2C bus
I2C_ORDER I2C1

define HAL_I2C_CLEAR_ON_TIMEOUT 0

define HAL_DEVICE_THREAD_STACK 256

define AP_PARAM_MAX_EMBEDDED_PARAM 0

define HAL_I2C_INTERNAL_MASK 0

# disable dual GPS and GPS blending to save flash space
define GPS_MAX_RECEIVERS 1
define GPS_MAX_INSTANCES 1
define HAL_COMPASS_MAX_SENSORS 1
define HAL_WITH_DSP FALSE



# keep ROMFS uncompressed as we don't have enough RAM
# to uncompress the bootloader at runtime
env ROMFS_UNCOMPRESSED True

# reduce the number of CAN RX Buffer
define HAL_CAN_RX_QUEUE_SIZE 32

# disable stack checking to reduce flash cost
define CH_DBG_ENABLE_STACK_CHECK FALSE
