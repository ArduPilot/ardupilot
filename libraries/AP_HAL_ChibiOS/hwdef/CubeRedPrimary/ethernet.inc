# Ethernet
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

# Refer to https://maclookup.app/vendors/cubepilot-pty-ltd
# Note, lower 3 bytes (ADDR3,4,5) will be replaced with the platform UUID
define AP_NETWORKING_DEFAULT_MAC_ADDR "A8:B0:28:00:00:00"
