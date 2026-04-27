# NWBLUE_PROH757 Flight Controller

The NWBLUE_PROH757 is a 30 × 30 mm multirotor flight controller designed by
Metzler LLC and built around the
[CubePilot CubeNode H7 module](https://docs.cubepilot.org/user-guides/cubenode/pin-descriptions).

## Features

 - MCU - STM32H757 dual-core (Cortex-M7 / M4) running the M7 core at 480 MHz, 2 MB flash
 - On-module ICM-45686 IMU on SPI3 (mounted upside down on this carrier — `ROTATION_ROLL_180`)
 - On-board DPS368 barometer on SPI3
 - On-board IIS2MDC magnetometer on internal I2C3
 - microSD card slot on SDMMC2
 - 6 UARTs plus USB
 - 9 PWM / DShot outputs (4 motor + 5 AUX)
 - MSP DisplayPort OSD support for HD VTX (no analog / MAX7456 OSD chip on board)
 - CAN1 for DroneCAN peripherals
 - USB-C
 - RGB notify LED + amber status LED
 - Buzzer driver
 - Voltage and current monitoring on the battery rail; rail-health sensing on 3V3, 5V, and the 10 V servo rail
 - 30 × 30 mm mounting

The CubeNode module also provides Ethernet and a second CAN bus, but the
NWBLUE_PROH757 carrier does not expose those signals.

## UART Mapping

`SERIALn` maps 1:1 to `USARTn` / `UARTn`, with `EMPTY` placeholders for the
UARTs not broken out on the carrier (USART3, UART5):

| SERIAL | UART | Connector | Default protocol |
| --- | --- | --- | --- |
| SERIAL0 | OTG1 | USB | MAVLink2 |
| SERIAL1 | USART1 | ESC J10 pin 3 (RX only) | ESC Telemetry |
| SERIAL2 | USART2 | VTX J9 pin 5 (RX only) | None — user picks (e.g. RunCam, MSP) |
| SERIAL3 | — | (USART3 not exposed) | — |
| SERIAL4 | UART4 | RC J11 | RC Input |
| SERIAL5 | — | (UART5 not exposed) | — |
| SERIAL6 | USART6 | VTX J9 pins 3–4 (TX/RX) | MSP DisplayPort |
| SERIAL7 | UART7 | TELEM1 J4 | MAVLink2 |
| SERIAL8 | UART8 | GPS J8 | GPS |

The TELEM1 connector exposes RTS/CTS on PE6/PE5 for cable compatibility, but
those pins do not have UART7 hardware-flow-control alternate functions on the
STM32H757; the firmware uses TELEM1 in 8N1 without HW flow control.

`SERIAL6_PROTOCOL` defaults to MSP DisplayPort assuming an HD VTX (DJI / Walksnail / HDZero) is plugged into J9. For an analog VTX, change it to:

 - 37 (`SerialProtocol_SmartAudio`) for SmartAudio VTXs
 - 44 (`SerialProtocol_Tramp`) for Tramp VTXs

`SERIAL2_PROTOCOL` is left at None because J9 pin 5 (USART2 RX) can be used for several different camera/VTX backchannels — set it to match the device you connect.

## Connectors

All signal connectors are JST-GH 1.25 mm pitch.

### TELEM1 (J4, 6-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | VCC | +5 V |
| 2 | UART7 TX (out) | +3.3 V |
| 3 | UART7 RX (in) | +3.3 V |
| 4 | UART7 CTS (software-only — see UART Mapping note) | +3.3 V |
| 5 | UART7 RTS (software-only) | +3.3 V |
| 6 | GND | GND |

### GPS (J8, 6-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | VCC | +5 V |
| 2 | GND | GND |
| 3 | UART8 TX (out) | +3.3 V |
| 4 | UART8 RX (in) | +3.3 V |
| 5 | I2C4 SCL | +3.3 V |
| 6 | I2C4 SDA | +3.3 V |

### RC (J11, 4-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | VCC | +5 V |
| 2 | GND | GND |
| 3 | UART4 TX (out) | +3.3 V |
| 4 | UART4 RX (in) | +3.3 V |

### VTX (J9, 6-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | VCC | +10 V |
| 2 | GND | GND |
| 3 | USART6 TX (out) | +3.3 V |
| 4 | USART6 RX (in) | +3.3 V |
| 5 | USART2 RX (in) | +3.3 V |
| 6 | GND | GND |

### CAN1 (J3, 4-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | VCC | +5 V |
| 2 | CAN_H | — |
| 3 | CAN_L | — |
| 4 | GND | GND |

A 120 Ω termination resistor is fitted on the carrier between CAN_H and CAN_L.

### ESC (J10, 8-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | VBAT (VSYS, battery direct) | battery |
| 2 | Current sense input | analog |
| 3 | USART1 RX (ESC telemetry) | +3.3 V |
| 4 | M1 (FMU_CH1) | +3.3 V |
| 5 | M2 (FMU_CH2) | +3.3 V |
| 6 | M3 (FMU_CH3) | +3.3 V |
| 7 | M4 (FMU_CH4) | +3.3 V |
| 8 | GND | GND |

### PWM AUX (J7, 6-pin)

| Pin | Signal | Voltage |
| --- | --- | --- |
| 1 | M5 (FMU_CH5) | +3.3 V |
| 2 | M6 (FMU_CH6) | +3.3 V |
| 3 | M7 (FMU_CH7) | +3.3 V |
| 4 | M8 (FMU_CH8) | +3.3 V |
| 5 | M9 (FMU_CH9) | +3.3 V |
| 6 | GND | GND |

### SWD (J5, 6-pin)

For programming and debug. Carries SWDIO, SWCLK, +3.3 V, GND with on-board ESD protection.

### PWR Output (J6, 3-pin)

Provides VSYS (battery) and +10 V outputs to peripherals (e.g. powered VTX).
The +10 V rail comes from the on-board buck converter (2.5 A, eFuse-protected).

## RC Input

The default RC input is on UART4 (SERIAL4, J11). It supports all serial RC
protocols (SBUS, CRSF/ELRS, DSM, FPort, SRXL2). For half-duplex / single-wire
protocols the UART4 TX pin is also brought out on J11 pin 3.

 - :ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` defaults to 23 (RC Input)
 - :ref:`SERIAL4_OPTIONS<SERIAL4_OPTIONS>` = 0 for CRSF/ELRS (default), 4 for SRXL2, 15 for FPort

## OSD Support

The NWBLUE_PROH757 has no analog (MAX7456) OSD chip. OSD is provided over MSP
DisplayPort to an HD VTX (DJI O3, Walksnail, HDZero, etc.) on the J9 VTX
connector. Defaults:

 - :ref:`OSD_TYPE<OSD_TYPE>` = 5 (MSP DisplayPort)
 - :ref:`SERIAL6_PROTOCOL<SERIAL6_PROTOCOL>` = 42 (MSP DisplayPort)

If you are connecting an analog VTX instead, change `SERIAL6_PROTOCOL` to
`SerialProtocol_SmartAudio` (37) or `SerialProtocol_Tramp` (44) and either
disable the OSD (`OSD_TYPE = 0`) or use a separate analog OSD module.

## PWM Output

The board has 9 PWM / DShot outputs grouped by timer:

 - PWM 1–4 in group 1 (TIM1) — motor outputs on the ESC connector
 - PWM 5–6 in group 2 (TIM4) — AUX on PWM AUX connector
 - PWM 7–8 in group 3 (TIM3) — AUX on PWM AUX connector
 - PWM 9   in group 4 (TIM2) — AUX on PWM AUX connector

Channels within the same group must use the same output rate. If any channel
in a group uses DShot, all channels in that group must use DShot. PWM 1–8
(motors and the PWM AUX connector) support bi-directional DShot. PWM 9 is on
TIM2 and runs DShot without bi-directional telemetry.

## Battery Monitoring

The board has an on-board voltage divider for the battery rail. The current
sense signal is brought in via the ESC connector (J10 pin 2) so that it can be
fed by an in-line current sensor or by the ESC's own current output.

Default battery monitor parameters:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4 (Analog Voltage and Current)
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 5 (PF3, ADC3 ch 5)
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 12 (PC2, ADC1 ch 12)
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 18.0
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 24.0

The current sense input is fed from the ESC connector (J10 pin 2).
`BATT_VOLT_MULT` and `BATT_AMP_PERVLT` should be calibrated against the actual
divider resistor values and the current sensor attached to the ESC connector.

## Compass

The NWBLUE_PROH757 has a built-in IIS2MDC magnetometer on the internal I2C3
bus. External compasses can be added on the GPS connector (I2C4 bus) — for
example the magnetometer in a u-blox / Holybro / Matek GPS module.

## CAN

CAN1 is exposed on connector J3 for DroneCAN peripherals (GPS, compass,
ESCs, airspeed, range finders, etc.). The CubeNode module's on-module CAN
transceiver is enabled by default. CAN2 is not broken out on this carrier.

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the
bootloader button held. Then load the `with_bl.hex` firmware using your
favourite DFU tool.

Once the initial firmware is loaded, you can update the firmware using any
ArduPilot ground station software. Updates should be done with the `*.apj`
firmware files.
