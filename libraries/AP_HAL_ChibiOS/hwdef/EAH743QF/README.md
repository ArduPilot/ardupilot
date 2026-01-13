# Ewing Aerospace H743QF Flight Controller

The Ewing Aerospace H743QF is a flight controller produced by [Ewing Aerospace](https://ewingaerospace.com/).

## Features

 - STM32H743 microcontroller
 - ICM42688 IMU
 - BMP390 barometer
 - MAX7456 OSD
 - 6 UARTs
 - 6 PWM outputs

## Pinout

![Ewing Aerospace H743QF v1](Ewing_Aeropsace_H743QF_Board_Top.png "Ewing Aerospace H743QF v1")
![Ewing Aerospace H743QF v1](Ewing_Aeropsace_H743QF_Board_Bottom.png "Ewing Aerospace H743QF v1")

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB
 - SERIAL1 -> UART1 (GPS)
 - SERIAL2 -> UART2 (HD OSD) 
 - SERIAL3 -> UART3 (ANALOG OSD)
 - SERIAL4 -> UART4 (SIK Radio GCSD)
 - SERIAL5 -> UART5 (Open use (NODMA))
 - SERIAL6 -> UART6 (RX-only from HD VTX)

## RC Input

RC input is configured on the R4 (UART4_RX) pin for most RC unidirectional protocols. Bidirectional protocols, like CRSF/ELRS will use T4 also, and: 

- PPM is not supported.
- SBUS/DSM/SRXL connects to the R4 pin.
- FPort requires connection to T4. Set :ref:`SERIAL4_OPTIONS<SERIAL4_OPTIONS>` = 7.  See :ref:`common-FPort-receivers`.
- CRSF also requires a T4 connection, in addition to 4, and automatically provides telemetry. Set :ref:`RSSI_TYPE<RSSI_TYPE>` = 3.
-  SRXL2 requires a connection to T4 and automatically provides telemetry. Set :ref:`SERIAL2_OPTIONS<SERIAL2_OPTIONS>` to “4”. Set :ref:`RSSI_TYPE<RSSI_TYPE>` = 3.
.. note:: the RC input on the HD VTX connector (SBUS from DJI air units) is directly connected to R6. If this feature is used, :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` should be set to "23" and :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` changed from "23" to something else.
Any UART can be used for RC system connections in ArduPilot. See [Radio Control Systems](https://ardupilot.org/plane/docs/common-rc-systems.html#common-rc-systems) for details.

 
## OSD Support

The Ewing Aerospace H743QF supports OSD using OSD_TYPE 1 (MAX7456 driver). Simultaneous DisplayPort OSD is also possible and is configured by default.

## VTX Support

The JST-GH-6P connector supports a standard DJI HD VTX connection. Pin 1 of the connector is 9v so be careful not to connect
this to a peripheral that cannot tolerate 10v.

## PWM Output

The Ewing Aerospace H743QF supports up to 6 PWM outputs.

The PWM is in 3 groups:

 - PWM 1-2 in group1
 - PWM 3-4 in group2
 - PWM 5-6 in group3

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-4 support bi-directional DShot.

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S.
LiPo batteries.

The default battery parameters are:

 - :ref:BATT_MONITOR<BATT_MONITOR> = 4
 - :ref:BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog> = 10
 - :ref:BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog> = 11 (CURR pin)
 - :ref:BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog> = 11.0
 - :ref:BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog> = 100 (will need to be adjusted for whichever current sensor is attached)

## Compass

The Ewing Aerospace H743QF does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*\.apj firmware files.

