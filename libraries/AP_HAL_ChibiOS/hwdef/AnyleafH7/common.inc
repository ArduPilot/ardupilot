# MCU class and specific type
MCU STM32H7xx STM32H743xx

# board ID for firmware load
APJ_BOARD_ID AP_HW_AnyleafH7

# crystal frequency, setup to use external oscillator
OSCILLATOR_HZ 16000000

FLASH_SIZE_KB 2048

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

define AP_CAN_SCLAN_ENABLED 1
define HAL_OTG2_PROTOCOL SerialProtocol_SLCAN