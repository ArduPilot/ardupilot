# CubeNode module hwdef - shared module-fixed pins
# Used by hwdefs targeting the CubePilot CubeNode H757 module.
# CubeNode pin descriptions can be found at https://docs.cubepilot.org/user-guides/cubenode/pin-descriptions

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

# CAN Common (sleep/shutdown lines on the module)
PG1 SLEEPCAN OUTPUT PUSHPULL SPEED_LOW LOW
PG0 SHUTDOWNCAN OUTPUT PUSHPULL SPEED_LOW LOW

# These are the pins for SWD debugging with a STlinkv2 or black-magic probe.
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# Ethernet PHY (LAN8720 RMII on the module)
PC1  ETH_MDC            ETH1
PA2  ETH_MDIO           ETH1
PC4  ETH_RMII_RXD0      ETH1
PC5  ETH_RMII_RXD1      ETH1
PB12 ETH_RMII_TXD0      ETH1
PB13 ETH_RMII_TXD1      ETH1
PB11 ETH_RMII_TX_EN     ETH1
PA7  ETH_RMII_CRS_DV    ETH1
PA1  ETH_RMII_REF_CLK   ETH1

define BOARD_PHY_ID  MII_LAN8720_ID
define BOARD_PHY_RMII

# IMU on module (ICM-45686 on SPI3)
PC10 SPI3_SCK SPI3
PC11 SPI3_MISO SPI3
PC12 SPI3_MOSI SPI3
PG15 IMU_CS CS

SPIDEV icm45686  SPI3  DEVID4  IMU_CS   MODE0  24*MHZ  24*MHZ
