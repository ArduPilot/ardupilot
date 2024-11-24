# F4BY Flight Controller MCU upgrade

The F4BY flight controller shop: https://f4by.com/en/?order/our_product

The instructions are available here: https://f4by.com/en/?doc/fc_f4by_v2.1.5

## Howto
for self upgrage old fc: 
replace old MCU STM32F407VGT (1MB Flash) with STM32F427VET rev3 or above (2MB Flash)


## Features

 - Full Ardupilot features support (exclude LUA Script)


## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
boot solder pads connected. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

