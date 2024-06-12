# SDMODEL SDH7 V2 Flight Controller

## Features

 - STM32H743 microcontroller
 - MPU6000 IMU
 - BMP280 barometer
 - IST8310 Compass
 - microSD card slot
 - AT7456E OSD
 - 6 UARTs
 - 9 PWM outputs

## Pinout

![SDH7V2](SDMODEL_H7V2.png)

![SDH7V2](H7V1_0502.png)

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (Telem1) (MSP DisplayPort)(DMA Capable)
 - SERIAL2 -> UART2 (Telem2) (connected to internal BT module, not useable by ArduPilot)
 - SERIAL3 -> UART3 (GPS)(DMA Capable)
 - SERIAL4 -> UART4 (GPS)
 - SERIAL5 -> not available
 - SERIAL6 -> UART6 (RX6 in RCinput, ALT config to use as UART input)
 - SERIAL7 -> UART7 RX pin only, ESC telem)(DMA Capable)

## RC Input

RC input is configured on the R6 (UART6_RX) pin. It supports all single wire unidirectional RC 
protocols. For protocols requiring half-duplex  or full duplex serial for operation
select another UART with DMA and set its protocol to "23". To use this UART for other uses, set
:ref:`BRD_ALT_CONFIG<BRD_ALT_CONFIG>` to "1"
 
## FrSky Telemetry
 
FrSky Telemetry is supported using the Tx pin of any UART including SERIAL6/UART6 . You need to set the following parameters to enable support for FrSky S.PORT (example shows SERIAL6). Note this assumes the RC input is using default (ALT_BRD_CONFIG =0). Obviously, if using ALT_BRD_CONFIG = 1 for full duplex RC prtocols, you must a different UART for FrSky Telemetry.
 
  - SERIAL6_PROTOCOL 10
  - SERIAL6_OPTIONS 7
  
## OSD Support

The SDMODEL SDH7 V2 supports OSD using OSD_TYPE 1 (MAX7456 driver).The defaults are also setup to allow DJI Goggle OSD support on UART1. Both OSDs can operate simultaneously.

## VTX Support

The JST-GH-6P connector supports a standard DJI HD VTX connection. Pin 1 of the connector is 9v so be careful not to connect
this to a peripheral requiring 5v. The 9v supply is controlled by RELAY2_PIN set to GPIO 81 and is on by default. It can be configured to be operated by an RC switch by selecting the function RELAY2.

## Camera Control

The Cam pin is GPIO82 and is set to be controlled by RELAY4 by default. Relay pins can be controlled either by an RC switch or GCS command. See :ref:`common-relay` for more information.

## PWM Output

The SDMODEL SDH7 V2 supports up to 9 PWM or DShot outputs.  Outputs 1-4 support BDShot. The pads for motor output
M1 to M8 on the two motor connectors, plus M9 preconfigured for LED strip or can be used as another
PWM output.

The PWM is in 5 groups:

 - PWM 1, 2 in group1
 - PWM 3, 4 in group2
 - PWM 5, 6 in group3
 - PWM 7, 8 in group4
 - PWM 9 in group5

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot.

.. note:: for users migrating from BetaflightX quads, the first four outputs M1-M4 have been configured for use with existing motor wiring using these default parameters:

- :ref:`FRAME_CLASS<FRAME_CLASS>` = 1 (Quad)
- :ref:`FRAME_TYPE<FRAME_TYPE>` = 12 (BetaFlightX) 

## Battery Monitoring

The board has a builting voltage and current sensor. The current
sensor can read up to 130 Amps. The voltage sensor can handle up to 6S
LiPo batteries.

The correct battery setting parameters are:

 - BATT_MONITOR 4
 - BATT_VOLT_PIN 10
 - BATT_CURR_PIN 11
 - BATT_VOLT_MULT 11
 - BATT_AMP_PERVLT 59.5

## Compass

SDMODEL SDH7 V2 has a built-in compass IST8310, but you can add an external compass 2nd using the I2C connections on the SDA and SCL pads.

## Firmware

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`_ in  sub-folders labeled "SDMODELH7V2".


## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.

