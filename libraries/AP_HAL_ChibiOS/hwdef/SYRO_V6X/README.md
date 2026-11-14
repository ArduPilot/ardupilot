# SYRO_V6X Flight Controller

![SYRO_V6X.png](SYRO_V6X.png)

The SYRO_V6X flight controller produced by [SYRO](need link).
The SYRO_V6X is based on the Pixhawk® Autopilot FMUv6X Standard, Autopilot Bus Standard, and connector Standard.

## Where to Buy

---Todo---

## Pinout

![SYRO_V6X-pinout.png](SYRO_V6X-pinout.png)

## Features

- STM32H753 microcontroller
- 3 IMUs: 2x ICM45686  and BMI088
- builtin BMM350 magnetometer
- Barometers: BMP581
- microSD card slot
- USB-TypeC port
- 1 ETH network interface
- 7 UARTs plus USB
- 16 PWM outputs
- 3 external I2C ports
- 2 CAN ports (two independent CAN buses)

## UART Mapping

Default mapping and protocols are shown below. All UARTs have full DMA capability.

- SERIAL0 -> USB
- SERIAL1 -> UART7 (TELEM1, MAVLink2)
- SERIAL2 -> UART5 (TELEM2, MAVLink2)
- SERIAL3 -> USART1 (GPS&SAFETY)
- SERIAL4 -> UART8 (GPS2)
- SERIAL5 -> USART2 (TELEM3, MAVLink2)
- SERIAL6 -> UART4 (UART4, None)
- SERIAL7 -> USART3 (DEBUG, None)
- SERIAL8 -> USB OTG2 (MAVLink2)

The TELEM1, TELEM2 and TELEM3 ports have RTS/CTS pins, the other UARTs do not
have RTS/CTS.

The USART3 connector is labelled debug, but is available as a general purpose UART with ArduPilot.

## RC Input

RC input is configured on the port marked DSM/SBUS RC or PPM IN for all unidirectional protocols.

To allow CRSF/ELRS and the embedded telemetry available in FPort, CRSF, and SRXL2 receivers, a full UART, such as SERIAL6 (UART4), would need to be used for receiver connections. CRSF/ELRS in particular requires a UART with DMA capability for reliable operation. Below are example settings for the SERIALn parameters of the UART used.

`SERIALn_PROTOCOL` should be set to `23` (RCIN).

- FPort requires `SERIALn_OPTIONS` be set to `15`.
- CRSF requires `SERIALn_OPTIONS` be set to `0`.
- SRXL2 requires `SERIALn_OPTIONS` be set to `4` and connects only the TX pin.

See [RC Systems](https://ardupilot.org/plane/docs/common-rc-systems.html) for details for each protocol.

## PWM Output

The SYRO_V6X supports up to 16 PWM outputs. First 8 outputs (labelled M1 to M8) are the "main" outputs. It is controlled by a dedicated STM32F103 IO controller. M1-4 support Bi-Directional DShot. DShot on M1-M8 needs BRD_IO_DSHOT=1 and a reboot.

The remaining 8 outputs (labelled A1 to A8) are the "auxiliary" outputs. These are directly attached to the STM32H753 and support all PWM protocols, except A7 and A8 which only support PWM protocols. The first 6 of the auxiliary PWM outputs support BDShot.

The 8 main PWM outputs are in 3 groups:

- M1 and M2 in group1 (TIM2)
- M3 and M4 in group2 (TIM4)
- M5 - M8 in group3 (TIM3)

The 8 auxiliary PWM outputs are in 3 groups:

- AUX 1, 2, 3 and 4 in group1 (TIM5)
- AUX 5 and 6 in group2 (TIM4)
- AUX 7 and 8 in group3 (TIM12)

Channels within the same group need to use the same output rate. If any channel in a group uses DShot then all channels in the group need to use DShot.

## Battery Monitoring

The board has two dedicated I2C power monitor ports on 6 pin connectors.

The default battery parameters are:

- `BATT_MONITOR` = 21 (INA2xx)
- I2C bus 1, address 0

## Compass

The SYRO_V6X has a BMM350 builtin compass. Users usually disable this and use an external compass to minimize power system interference

## GPIOs

The auxiliary PWM ports and some pins can be used as GPIOs (relays, buttons, RPM etc).

The numbering of the GPIOs for PIN variables in ArduPilot is:

- AUX1 50
- AUX2 51
- AUX3 52
- AUX4 53
- AUX5 54
- AUX6 55
- AUX7 56
- AUX8 57
- FMU_CAP1 58
- NFC_GPIO 60
- MAIN(1) 101
- MAIN(2) 102
- MAIN(3) 103
- MAIN(4) 104
- MAIN(5) 105
- MAIN(6) 106
- MAIN(7) 107
- MAIN(8) 108

## Analog inputs

The SYRO_V6X has 2 analog inputs

- ADC Pin13 -> ADC 3.3V Sense
- ADC Pin12 -> ADC 6.6V Sense

## CAN

The SYRO_V6X has two independent CAN buses.

## Loading Firmware

Firmware for these boards can be found at [ArduPilot Firmware Server](https://firmware.ardupilot.org) in sub-folders labeled "SYRO_V6X".

The board comes pre-installed with an ArduPilot compatible bootloader, allowing the loading of \*.apj firmware files with any ArduPilot compatible ground station.
