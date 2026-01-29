# Brahma H7 Flight Controller

The Brahma H7 is a high-performance flight controller produced by [Manufacturer](URL), featuring an STM32H743 processor, dual ICM42688 IMUs for redundancy, and a full suite of interfaces including 7 UARTs, CAN bus, and analog OSD.

## Features

 - MCU - STM32H743VIH6 32-bit processor running at 480 MHz
 - Two ICM42688-P IMUs
 - DPS310 barometer
 - OSD - AT7456E
 - microSD card slot
 - 7x UARTs
 - CAN support
 - 13x PWM Outputs (12 Motor Output, 1 LED)
 - Battery input voltage: 2S-6S
 - BEC 5V 3A for peripherals
 - BEC 10V 3A for video (user switchable via GPIO)

## Pinout

*Pinout images not yet available.*

## UART Mapping

The UARTs are marked Rn and Tn in the above pinouts. The Rn pin is the
receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB (MAVLink2)
 - SERIAL1 -> USART1 (MAVLink2, DMA-enabled)
 - SERIAL2 -> USART2 (GPS, DMA-enabled)
 - SERIAL3 -> USART3 (GPS, DMA-enabled)
 - SERIAL4 -> UART4 (ESC Telemetry)
 - SERIAL6 -> USART6 (RC Input, DMA-enabled)
 - SERIAL7 -> UART7 (MAVLink2, DMA and flow-control enabled)
 - SERIAL8 -> UART8 (Spare, DMA-enabled)

## RC Input

The default RC input is configured on USART6. RC could be applied instead to a different UART port such as UART8 and set
the protocol to receive RC data :ref:`SERIALn_PROTOCOL<SERIALn_PROTOCOL>` = 23 and change :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>`
to something other than '23'. For RC protocols other than unidirectional, the USART6_TX pin will need to be used:

 - :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` should be set to "23".
 - FPort would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "15".
 - CRSF would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "0".
 - SRXL2 would require :ref:`SERIAL6_OPTIONS<SERIAL6_OPTIONS>` be set to "4" and connects only the TX pin.

## FrSky Telemetry

FrSky Telemetry is supported using an unused UART, such as the Tx pin of UART8.
You need to set the following parameters to enable support for FrSky S.PORT:

 - :ref:`SERIAL8_PROTOCOL<SERIAL8_PROTOCOL>` 10
 - :ref:`SERIAL8_OPTIONS<SERIAL8_OPTIONS>` 7

## OSD Support

The Brahma H7 supports OSD using OSD_TYPE 1 (MAX7456 driver). External MSP DisplayPort OSDs (like DJI or Walksnail) are supported on any spare UART.

## PWM Output

The Brahma H7 supports up to 13 PWM or DShot outputs. The pads for motor output
M1 to M12 are provided on both the motor connectors and on separate pads, plus
separate pads for LED strip and other PWM outputs.

The PWM is in 5 groups:

 - PWM 1-2   in group1 (TIM3)
 - PWM 3-6   in group2 (TIM5)
 - PWM 7-10  in group3 (TIM4)
 - PWM 11-12 in group4 (TIM15)
 - PWM 13    in group5 (TIM1, LED)

Channels within the same group need to use the same output rate. If
any channel in a group uses DShot then all channels in the group need
to use DShot. Channels 1-10 support bi-directional dshot.

## Battery Monitoring

The board has a internal voltage sensor and connections on the ESC connector for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.0
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 78.4

Pads for a second analog battery monitor are provided. To use:

 - :ref:`BATT2_MONITOR<BATT2_MONITOR>` 4
 - :ref:`BATT2_VOLT_PIN<BATT2_VOLT_PIN__AP_BattMonitor_Analog>` 18
 - :ref:`BATT2_CURR_PIN<BATT2_CURR_PIN__AP_BattMonitor_Analog>` 7
 - :ref:`BATT2_VOLT_MULT<BATT2_VOLT_MULT__AP_BattMonitor_Analog>` 11.0
 - :ref:`BATT2_AMP_PERVLT<BATT2_AMP_PERVLT__AP_BattMonitor_Analog>` as required

## Analog RSSI input

Analog RSSI uses :ref:`RSSI_PIN<RSSI_PIN>` 8

## Analog AIRSPEED inputs

Analog Airspeed sensor would use ARSPD_PIN 4

## CAN

The Brahma H7 has a CAN port for DroneCAN peripherals such as GPS, compass, airspeed, and rangefinder.

## Compass

The Brahma H7 does not have a builtin compass, but you can attach an external compass using I2C on the SDA and SCL pads.

## VTX power control

GPIO 81 controls the VTX BEC output to pins marked "10V". Setting this GPIO low removes
voltage supply to this pin/pad. By default RELAY1 is configured to control this pin and sets the GPIO high.

## Loading Firmware

Firmware for these boards can be found `here <https://firmware.ardupilot.org>`__ in sub-folders labeled "BrahmaH7".

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button pressed. Then you should load the "with_bl.hex"
firmware, using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using
any ArduPilot ground station software. Updates should be done with the
*.apj firmware files.
