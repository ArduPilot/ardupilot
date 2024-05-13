# crystal frequency
OSCILLATOR_HZ 24000000

FLASH_SIZE_KB 2048

# bootloader loads at start of flash
FLASH_RESERVE_START_KB 0

# the location where the bootloader will put the firmware
# the H743 has 128k sectors
FLASH_BOOTLOADER_LOAD_KB 128

define HAL_LED_ON 0

# uarts and USB to run bootloader protocol on
SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7 USART1

# these are the pins for SWD debugging with a STlinkv2 or black-magic probe
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# now we define the pins that USB is connected on
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# this is the pin that senses USB being connected. It is an input pin
# setup as OPENDRAIN
PA9 VBUS INPUT OPENDRAIN

include serial.inc

# Add CS pins to ensure they are high in bootloader
PC13 GYRO_CS CS
PD7 BARO_CS CS
PD10 FRAM_CS CS SPEED_VERYLOW
PE4 MPU_EXT_CS CS

PB5 LED_BOOTLOADER OUTPUT HIGH OPENDRAIN

# enable DFU by default
ENABLE_DFU_BOOT 1

include sdcard.inc

# enable FAT filesystem support (needs a microSD defined via SDMMC)
define HAL_OS_FATFS_IO 1
define AP_BOOTLOADER_FLASH_FROM_SD_ENABLED 1

# FATFS support:
define CH_CFG_USE_MEMCORE 1
define CH_CFG_USE_HEAP 1
define CH_CFG_USE_SEMAPHORES 0
define CH_CFG_USE_MUTEXES 1
define CH_CFG_USE_DYNAMIC 1
define CH_CFG_USE_WAITEXIT 1
define CH_CFG_USE_REGISTRY 1
