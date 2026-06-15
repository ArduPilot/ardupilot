# HAKRC H743 Flight Controller

The HAKRC H743 produced by [HAKRC](https://www.hakrc.com/) features up to 8S or 12S input voltage (depending on variant), an STM32H743 processor, dual ICM42688 IMUs for redundancy, and a full suite of interfaces including 8 UARTs, CAN bus, analog OSD and microSD logging.

## Features

 - MCU - STM32H743 32-bit processor running at 480 MHz
 - 2MB internal flash
 - Two ICM42688 IMUs
 - SPL06 barometer
 - AT7456E analog OSD
 - microSD card slot
 - 8x UARTs
 - 1x CAN bus
 - 13x PWM Outputs (8 motor outputs with bi-directional DShot, 2 servo outputs, 1 LED, 2 spare DShot)
 - Dual analog battery monitors (supports up to 12S LiPo)
 - Analog RSSI input
 - Analog airspeed input
 - Switchable VTX / camera GPIOs

## Pinout

*Pinout images not yet available - refer to the manufacturer's product page.*

## UART Mapping

The UARTs are marked Rn and Tn in the board silkscreen. The Rn pin is
the receive pin for UARTn. The Tn pin is the transmit pin for UARTn.

 - SERIAL0 -> USB (MAVLink2)
 - SERIAL1 -> USART1
 - SERIAL2 -> USART2
 - SERIAL3 -> USART3
 - SERIAL4 -> UART4
 - SERIAL5 -> UART5 (RC Input by default)
 - SERIAL6 -> USART6
 - SERIAL7 -> UART7
 - SERIAL8 -> UART8

All UARTs are DMA-capable.

## RC Input

The default RC input is configured on UART5. It supports all unidirectional ArduPilot-compatible protocols, except PPM. Receivers using bi-directional protocols such as CRSF/ELRS should be connected to both TX5 and RX5.

 - FPort requires connection to TX5. See [FPort Receivers](https://ardupilot.org/copter/docs/common-FPort-receivers.html).
 - CRSF/ELRS also requires a TX5 connection and automatically provides telemetry.
 - SRXL2 requires a connection to TX5 and automatically provides telemetry. Set :ref:`SERIAL5_OPTIONS<SERIAL5_OPTIONS>` to "4".

RC input can be moved to any other UART by setting :ref:`SERIALn_PROTOCOL<SERIALn_PROTOCOL>` to 23 on the desired port and removing the protocol from SERIAL5.

## FrSky Telemetry

FrSky Telemetry can be supported on any spare UART. For example on USART1 set:

 - :ref:`SERIAL1_PROTOCOL<SERIAL1_PROTOCOL>` 10
 - :ref:`SERIAL1_OPTIONS<SERIAL1_OPTIONS>` 7

## OSD Support

The HAKRC H743-12S supports analog OSD using its built-in AT7456E (MAX7456-compatible) chip, set with `OSD_TYPE` 1. DisplayPort (MSP) HD goggle OSD can be used simultaneously on any spare UART.

## PWM Output

The HAKRC H743-12S supports up to 13 PWM / DShot outputs.

The PWM outputs are in 5 groups:

 - PWM 1-4   in group1 (TIM2)
 - PWM 5-8   in group2 (TIM4)
 - PWM 9-10  in group3 (TIM3)
 - PWM 11-12 in group4 (TIM15) - servo outputs
 - PWM 13    in group5 (TIM1) - serial LED strip

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

 - Channels 1-8 support bi-directional DShot.
 - Channels 9-10 support DShot.
 - Channels 11-12 are standard PWM only (TIM15 does not support DShot).
 - Channel 13 is defaulted for serial LED strip but may be used as a PWM output.

## Battery Monitoring

The board provides dual analog battery monitors. The primary voltage divider is sized for up to 12S LiPo batteries.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.0
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 40.0

Pads for a second analog battery monitor are provided. To use:

 - :ref:`BATT2_MONITOR<BATT2_MONITOR>` 4
 - :ref:`BATT2_VOLT_PIN<BATT2_VOLT_PIN__AP_BattMonitor_Analog>` 18
 - :ref:`BATT2_CURR_PIN<BATT2_CURR_PIN__AP_BattMonitor_Analog>` 7
 - :ref:`BATT2_VOLT_MULT<BATT2_VOLT_MULT__AP_BattMonitor_Analog>` 11.0
 - :ref:`BATT2_AMP_PERVLT<BATT2_AMP_PERVLT__AP_BattMonitor_Analog>` as required

## Analog RSSI and AIRSPEED inputs

Analog RSSI uses :ref:`RSSI_PIN<RSSI_PIN>` 8

Analog Airspeed sensor would use ARSPD_PIN 4

## CAN

The HAKRC H743-12S has a CAN port for DroneCAN peripherals such as GPS, compass, airspeed, and rangefinder. GPIO 70 controls the CAN1 silent line; it is driven low at boot so CAN1 is active by default.

## Compass

The HAKRC H743-12S does not have a built-in compass, but you can attach an external compass using I2C on the SDA and SCL pads (I2C2). All common ArduPilot-supported compass types are auto-detected.

## VTX / Peripheral Power Control

GPIO 81 (PINIO1) and GPIO 82 (PINIO2) are provided for switching VTX or other peripheral power. Both pins are driven HIGH at boot. They can be controlled at runtime by assigning them to `RELAYn_PIN` parameters (e.g. `RELAY2_PIN 81`, `RELAY3_PIN 82`).

## Loading Firmware

Firmware for this board can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled "HAKRCH743".

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then load the "with_bl.hex" firmware using your favourite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the \*.apj firmware files.

The bootloader also supports loading firmware directly from a microSD card - place the appropriate `ardupilot.apj` file on the SD card root and power-cycle with the bootloader active.