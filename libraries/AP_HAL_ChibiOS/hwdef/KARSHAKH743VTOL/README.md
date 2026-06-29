# KARSHAKH743VTOL Flight Controller

The KARSHAKH743VTOL is a hardware product produced by [Karshak Drones Pvt. Ltd.](https://www.karshakdrones.com/).

## Features

- Processor
  - STM32H743 microcontroller
- Sensors
  - Single BMI088 IMU (+1 External IMU; Optional Addon)
  - SPL06 barometer
- Power
  - 12V 2.5A BEC; 5V 2.5A BEC
- Interfaces
  - 10x PWM outputs DShot capable (direct use)
  - 1x RC input
  - 7x UARTs/Serial for GPS and other peripherals plus USB
  - microSD card slot for logging, etc.
  - USB-TypeC port
  - Built-in RGB LED
  - External/built-in Buzzer
  - Voltage and Current monitoring for VCC.

## UART Mapping

- SERIAL0 -> USB
- SERIAL1 -> UART1 (GPS, DMA-enabled)
- SERIAL2 -> UART2 (MAVLink2, DMA-enabled)
- SERIAL3 -> UART3 (VTX-HD, DMA-enabled)
- SERIAL4 -> UART4 (RX tied to inverted SBUS RC input, but can be used as normal UART if [BRD_ALT_CONFIG](https://ardupilot.org/copter/docs/parameters.html#brd-alt-config-alternative-hw-config) =1), Shared port)
- SERIAL5 -> UART5 (MAVLink2, DMA-enabled)
- SERIAL7 -> UART7 (ESC Telemetry, DMA-enabled)
- SERIAL8 -> UART8 (Serial RC Input, DMA-enabled)

## RC Input

RC input is configured on the RCIN pin, located at one end of the servo rail. All ArduPilot-supported unidirectional RC protocols can be connected directly to this pin, including PPM. For bi-directional or half-duplex protocols, such as CRSF, ELRS, or SBUS, a full UART is required. In this case, set the parameter [BRD_ALT_CONFIG](https://ardupilot.org/copter/docs/parameters.html#brd-alt-config-alternative-hw-config) =1) to enable the alternative hardware configuration.

## VTX Support

The generic 2.54 mm pin header located at the top-right corner provides connectivity for a DJI Air Unit or HD VTX. The default communication protocol is DisplayPort. Note that pin 1 (_the rightmost through-hole pad_) supplies 9 V. Exercise caution to avoid connecting peripherals that require only 5 V or 3.3 V.

## PWM Output

The KARSHAKH743VTOL supports up to 10 PWM outputs. All channels support bi-directional DShot.

10 PWM outputs are grouped into 3 groups:

- PWM 1, 2, 3, 4 are in Group 1 (TIM1);
- PWM 5 and 6 are in Group 2 (TIM2);
- PWM 7, 8, 9, 10 are in Group 3 (TIM3);

Channels in the same group must use the same output protocol and/or output rate. If any channel in a group uses DShot, all channels in the group need to use DShot.

## Battery Monitoring

The board has an internal voltage sensor (VBAT pin) and connections (CURR pin) for an external current sensor input.
The voltage sensor can handle up to 6S LiPo batteries.

The default battery parameters are:

- `BATT_MONITOR 4`
- `BATT_VOLT_PIN 4`
- `BATT_CURR_PIN 8`
- `BATT_VOLT_MULT 10.2`
- `BATT_AMP_PERVLT 20.4`

## Compass

The KARSHAKH743VTOL does not have a built-in compass, but you can attach an external compass via the I2C connector using the SDA and SCL pins.

## RSSI

Analog RSSI can be input on the RSSI pin. Set `RSSI_TYPE = 1`, `RSSI_ANA_PIN = 11`.

## GPIOs

The 2 pins labelled AX1 and AX3 are GPIO outputs.

The numbering of the GPIOs for PIN variables in ArduPilot is:

- `AX1 - 81`
- `AX3 - 83`

## CAN

The KARSHAKH743VTOL has ONE independent CAN bus, with the following pinouts.

### CAN1

   | Pin | Signal | Volt |
| --- | --- | --- |
| 1 (red) | VCC | +5V |
| 2 (blk) | CAN_H | +12V |
| 3 (blk) | CAN_L | +12V |
| 4 (blk) | GND | GND |

## Physical

- Mounting: `30.5 x 30.5mm`, `Φ2mm`
- Dimensions: `60 x 37mm`
- Weight: 16.5g (with onboard buzzer, external IMU connector and USB Type-C port)

## Loading Firmware

Firmware for these boards can be found at the [ArduPilot firmware server](https://firmware.ardupilot.org) in sub-folders labelled `"KARSHAKH743VTOL"`.

The board comes pre-installed with an ArduPilot-compatible bootloader, enabling the loading of `*.apj` firmware files through any ArduPilot-compatible ground station.
