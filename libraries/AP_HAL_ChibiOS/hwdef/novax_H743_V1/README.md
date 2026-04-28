# novax_H743_V1 Flight Controller

The novax_H743_V1 is a flight controller produced by [novaX](https://www.novaxfly.com/).

## Features

- MCU: STM32H743VIH6, 480MHz
- Gyro1: ICM-42688-P (SPI1)
- Gyro2: ICM-42688-P (SPI4)
- Barometer: DPS310
- Magnetometer: IST8310
- OSD: AT7456E (MAX7456 compatible)
- MicroSD card logging (SDIO 4-bit)
- CAN bus with TJA1051TK/3 transceiver
- 7 UARTs
- 10 PWM outputs (4 bidirectional DShot capable)
- 4 user-configurable GPIOs (PINIO)
- 5V BEC, 10V BEC
- USB Type-C

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> USART1 (VTX SmartAudio/IRC Tramp)
- SERIAL2 -> USART2 (DJI O3 SBUS)
- SERIAL3 -> USART3 (ELRS Receiver, DMA-enabled)
- SERIAL4 -> UART4 (DJI O3 MSP / CAN connector)
- SERIAL6 -> USART6 (GPS)
- SERIAL7 -> UART7 (Spare)
- SERIAL8 -> UART8 (Spare)

## RC Input

RC input is configured on SERIAL3 (USART3_RX, PD9) for CRSF/ELRS by default. All ArduPilot compatible RC protocols are supported.

- CRSF/ELRS requires both TX3 and RX3 connections and automatically provides telemetry.
- FPort requires connection to TX3. See :ref:`FPort Receivers<common-Fport-receivers>`.
- SRXL2 requires a connection to TX3 and automatically provides telemetry. Set :ref:`SERIAL3_OPTIONS<SERIAL3_OPTIONS>` to "4".

## OSD Support

The novax_H743_V1 supports analog OSD using its built-in AT7456E chip (MAX7456 compatible). MSP DisplayPort OSD is pre-configured on SERIAL4 for DJI O3 systems. See :ref:`common-msp-osd-overview-4.2` for more info.

## PWM Output

The novax_H743_V1 supports up to 10 PWM outputs plus a LED strip output.

Channels 1-4 support bidirectional DShot. All motor outputs support DShot.

PWM outputs are grouped and every group must use the same output protocol:

- PWM 1-2   in group1 (TIM3)
- PWM 3-6   in group2 (TIM5)
- PWM 7-10  in group3 (TIM4)
- PWM 11    in group4 (LED strip, TIM1)

## Battery Monitoring

The board has built-in voltage and current sensing. A second battery monitor input is also available.

The default battery parameters are:

- :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
- :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 10
- :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 11
- :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.0
- :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 40.0

A second battery monitor is also available:

- :ref:`BATT2_MONITOR<BATT2_MONITOR>` = 4
- :ref:`BATT2_VOLT_PIN<BATT2_VOLT_PIN__AP_BattMonitor_Analog>` = 18
- :ref:`BATT2_CURR_PIN<BATT2_CURR_PIN__AP_BattMonitor_Analog>` = 7

## RSSI

Analog RSSI input is available on ADC pin 8.

## Compass

The novax_H743_V1 has a built-in IST8310 compass on internal I2C bus. An external compass can also be connected via the I2C1 connector (GPS/Compass port).

## CAN

A single CAN bus interface is available using an onboard TJA1051TK/3 transceiver, configured for DroneCAN by default.

## GPIOs

4 user-configurable GPIO outputs (PINIO1-4) are available for controlling peripherals such as VTX power, camera switching, etc.

- PINIO1: GPIO 81
- PINIO2: GPIO 82
- PINIO3: GPIO 83
- PINIO4: GPIO 84

## Loading Firmware

Initial firmware load can be done with DFU by plugging in USB with the bootloader button pressed. Then you should load the "with_bl.hex" firmware, using your favorite DFU loading tool.

Once the initial firmware is loaded you can update the firmware using any ArduPilot ground station software. Updates should be done with the "*.apj" firmware files.

Firmware for this board can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labeled "novax_H743_V1".
