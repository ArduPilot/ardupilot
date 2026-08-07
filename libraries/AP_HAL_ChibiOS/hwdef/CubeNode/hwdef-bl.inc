# CubeNode module bootloader hwdef - shared module-fixed pins
# Used by bootloader hwdefs targeting the CubePilot CubeNode H757 module.

# MCU class and specific type
MCU STM32H7xx STM32H757xx

define CORE_CM7
define SMPS_PWR

FLASH_SIZE_KB 2048

# crystal frequency
OSCILLATOR_HZ 24000000

# CAN1
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1

# CAN2
#PB5    CAN2_RX    CAN2
#PB6    CAN2_TX    CAN2

# USB
PA11    OTG_FS_DM    OTG1
PA12    OTG_FS_DP    OTG1

# CAN Common
PG1 SLEEPCAN OUTPUT PUSHPULL SPEED_LOW LOW
PG0 SHUTDOWNCAN OUTPUT PUSHPULL SPEED_LOW LOW

# These are the pins for SWD debugging with a STlinkv2 or black-magic probe.
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD
