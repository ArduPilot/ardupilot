# Mamba F405 MK2 Flight Controller

https://www.diatone.us/products/mamba-f405-flight-controller-mk2

The Mamba F405 MK2 is a flight controller produced by [Diatone](https://www.diatone.us).

## Features

 - STM32F405RGT6 microcontroller
 - MPU6000 IMU
 - AT7456E OSD
 - 4 UARTs
 - 4 PWM outputs

## Pinout

![Mamba F405 MK2 Board](mamba_f405mk2_pinout.png "Mamba F405 MK2")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|PPM/RX1/SBUS/TX1|UART1 (RC Input)|
|SERIAL3|TX3/RX3|UART3 (Telem1)|
|SERIAL6|TX6/RX6|UART6 (GPS)|


## RC Input
 
RC input is configured on the PPM (UART1_RX) pin. It supports all RC protocols.
The S.BUS pad provides an inverted input for S.BUS receivers.
  
## OSD Support

The Mamba F405 MK2 supports OSD using OSD_TYPE 1 (MAX7456 driver).

## PWM Output

The Mamba F405 MK2 supports up to 4 PWM outputs. The pads for motor output ESC1 to ESC4 on the above diagram are for the 4 outputs. All 4 outputs support DShot as well as all PWM types.

The PWM are in one group.

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

## Battery Monitoring

The board has a built-in voltage sensor. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 3
 - BATT_VOLT_PIN 11
 - BATT_VOLT_MULT around 12.0
 - BATT_CURR_PIN 13
 - BATT_CURR_MULT around 39 with the included 40A ESC

## Compass

The Mamba F405 MK2 does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Alternate settings

It is possible to set alternate configurations with the BRD_ALT_CONFIG parameter.

|BRD_ALT_CONFIG|RX1 function|
|:----|:----|
|ALT 0(default) |RCININT|
|ALT 1|RX1/TX1|

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
