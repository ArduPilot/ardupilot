# iFlight Borg H7 Flight Controller

The Borg H7 is a flight controller produced by [iFlight](https://shop.iflight-rc.com/).

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - Gyro: ICM42688
 - 32Gb SDCard for logging
 - BEC output: 5V 2.5A,  switch controlled 12v 2A
 - Barometer: DPS310 / SPL06
 - OSD: AT7456E
 - 8x UARTs
 - 13x PWM Outputs (12 Motor Output, 1 LED)
 - Battery input voltage: 2S-8S
 - 2x I2C for external compass, airspeed, etc.
 - CAN port

## Pinout

![Borg H7 Board](blitz_h7_pro.png "Borg H7")

back side pinout image pending from iFlight

The expansion connector provides access to the following pins:
 - CAN+/CAN-
 - M9 through M12
 - TX7/RX7
 - SCL2/SDA2
 - RSSI
 - 5V/12V/GND

## UART Mapping

The UARTs are marked Rn and Tn in the pinouts. The Rn pin is the receive pin for UARTn. The Tn pin is the transmit pin for UARTn. Note that the ArduPilot `SERIALx` port numbers may not match the MCU's `UARTn` numbers.
|Name|Pin|Function|
|:-|:-|:-|
|SERIAL0|COMPUTER|USB|
|SERIAL1|RX1/TX1|UART1 (DJI connector, DMA-enabled)|
|SERIAL2|TX2/RX2|UART2 (RX, DMA-enabled)|
|SERIAL3|TX3/RX3|UART3 (DMA-enabled)|
|SERIAL4|TX4/RX4|UART4 (GPS, DMA-enabled)|
|SERIAL5|RX5 (ESC Connector)|UART6 (ESC Telemetry)|
|SERIAL6|TX7/RX7|UART7|
|SERIAL7|TX8/RX8|UART8|


## RC Input

RC input is configured on the `UART2_RX` pin (marked RX2) which forms part of the DJI connector. It supports all serial RC protocols.

## OSD Support

The Borg H7 supports OSD using OSD_TYPE 1 (AT7456E driver).

## PWM Output

The Borg H7 has 13 PWM outputs. The pads for motor output M1-M4 are in one ESC connector and M5-M8 in the second ESC connector. The remaining outputs (M9-M12) are available on pads or via the expansion connector. The first 8 outputs support bi-directional DShot, as well as all PWM types. Outputs 9-10 support DShot, as well as all PWM types and outputs 11-12 only support PWM.

The PWM outputs are in five groups:

 - PWM 1-2 in group1
 - PWM 3-6 in group2
 - PWM 7-10 in group3
 - PWM 11-12 in group4
 - PWM 13 in group5

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## Video Power Control

The 12V video power can be turned on/off using GPIO 81 which is already assigned by default to `RELAY2`. This relay can be controlled either from the GCS or using a transmitter channel.

## Camera Switch

The camera output can be switched using GPIO 82 which is already assigned by default to `RELAY3`. This relay can be controlled either from the GCS or using a transmitter channel.

## RSSI

The analog RSSI input pin is `8`.

## Battery Monitoring

The board has a builtin voltage sensor and a current sensor input tied to its 4-in-1 ESC current sensor. The voltage sensor can handle up to 8S LiPo batteries.

The correct battery setting parameters are:

 - `BATT_MONITOR` = 4
 - `BATT_VOLT_PIN` = 11
 - `BATT_VOLT_MULT` = 11
 - `BATT_CURR_PIN` = 10
 - `BATT_CURR_MULT` = 50

These are set by default in the firmware and shouldn't need to be adjusted.

## Compass

The Borg H7 does not have a builtin compass, but you can attach an external compass to an I2C port.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the *.apj firmware files.