# MCU class and specific type
MCU STM32H7xx STM32H743xx

# board ID for firmware load
APJ_BOARD_ID 59

# crystal frequency
OSCILLATOR_HZ 16000000

# 480Hz is only value support on H7
# MCU_CLOCKRATE_MHZ 320

# flash size
FLASH_SIZE_KB 2048

# enable DFU reboot for installing bootloader
# note that if firmware is build with --secure-bl then DFU is
# disabled
ENABLE_DFU_BOOT 1

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA9 VBUS INPUT OPENDRAIN

# SWD
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# debug uart
PD8 USART3_TX USART3
PD9 USART3_RX USART3
