
# on boards with a DFU button that are low on flash we can omit the
# bootloader from ROMFS, saving about 11k of flash. The user can still
# update using the apj and mission planner, but if they want to update
# the bootloader they need to use DFU with the 'xxx_with_bl.hex'
# firmware image
define AP_BOOTLOADER_FLASHING_ENABLED 0
