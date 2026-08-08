# hw definition file for processing by chibios_hwdef.py

# MCU class and specific type
MCU STM32H7xx STM32H757xx
define CORE_CM7
define SMPS_PWR

# board ID. See Tools/AP_Bootloader/board_types.txt
APJ_BOARD_ID AP_HW_CUBENODE

FLASH_SIZE_KB 2048

# bootloader is installed at zero offset
FLASH_RESERVE_START_KB 0

# the location where the bootloader will put the firmware
# the H757 has 128k sectors, assign 2 sectors for BL
FLASH_BOOTLOADER_LOAD_KB 256


# crystal frequency
OSCILLATOR_HZ 24000000

# USB
PA11    OTG_FS_DM    OTG1
PA12    OTG_FS_DP    OTG1

# SERIAL
PE7 UART7_RX UART7
PE8 UART7_TX UART7

SERIAL_ORDER OTG1 UART7

# CAN Common
PG1 SLEEPCAN OUTPUT PUSHPULL SPEED_LOW LOW 
PG0 SHUTDOWNCAN OUTPUT PUSHPULL SPEED_LOW LOW

# These are the pins for SWD debugging with a STlinkv2 or black-magic probe.
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD
