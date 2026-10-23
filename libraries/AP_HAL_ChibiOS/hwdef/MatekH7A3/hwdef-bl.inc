# hw definition file for processing by chibios_pins.py
# for Matek H7A3 bootloader

# MCU class and specific type
MCU STM32H7xx STM32H7A3xx

# crystal frequency, setup to use external oscillator
OSCILLATOR_HZ 16000000

FLASH_SIZE_KB 2048

# bootloader starts at zero offset
FLASH_RESERVE_START_KB 0

# the location where the bootloader will put the firmware
# the H7A3 has 8k sectors
FLASH_BOOTLOADER_LOAD_KB 32


# order of UARTs (and USB). Allow bootloading on USB and telem1
SERIAL_ORDER OTG1 USART1

# USART1 (telem1)
PB15 USART1_RX USART1 NODMA
PB14 USART1_TX USART1 NODMA

# PA10 IO-debug-console
PA11 OTG_HS_DM OTG1
PA12 OTG_HS_DP OTG1

# PA13 JTMS-SWDIO SWD
# PA14 JTCK-SWCLK SWD

# make sure Vsw is on during bootloader
PC13  PINIO1 OUTPUT LOW

PA14 LED_BOOTLOADER OUTPUT LOW
define HAL_LED_ON 0

# Add CS pins to ensure they are high in bootloader
PC4  IMU_CS CS
PB2  MAX7456_CS CS
PC15 FLASH_CS CS
PC14 SD_CS CS
