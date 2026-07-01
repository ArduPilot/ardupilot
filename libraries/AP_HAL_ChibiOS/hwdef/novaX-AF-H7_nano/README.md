# novaX AF-H7 nano Flight Controller

The novaX AF-H7 nano is a STM32H743-based flight controller by novaX.

![novaX AF-H7 nano pinout](novaX-AF-H7_nano-pinout.png)

## Specifications

- **MCU:** STM32H743VIH6 (480MHz, 2MB Flash, TFBGA-100)
- **IMU:** Dual ICM-42688-P on SPI1 + SPI4
- **Barometer:** DPS310 or SPA06-003 on I2C2 (0x76)
- **Compass:** IST8310 on I2C2 (0x0E), onboard
- **OSD:** AT7456E (MAX7456 compatible) on SPI2
- **CAN:** FDCAN1 with TJA1051TK/3 transceiver
- **SD Card:** SDMMC1 (4-bit)
- **USB:** Type-C (OTG_FS)
- **Motor Outputs:** 10 PWM (M1-M4 bidirectional DShot capable)
- **UARTs:** 7 (USART1/2/3/6, UART4/7/8)
- **ADC:** Dual battery voltage/current, RSSI, airspeed
- **LED Strip:** WS2812 on PA8

## UART Mapping

| Serial | Port   | Pins        | Default use                       |
|--------|--------|-------------|-----------------------------------|
| SERIAL0 | USB    | -           | MAVLink (USB)                     |
| SERIAL1 | USART1 | PA9/PA10    | VTX (SmartAudio/IRC Tramp)        |
| SERIAL2 | USART2 | PD5/PD6     | DJI O3 RC                         |
| SERIAL3 | USART3 | PD8/PD9     | ELRS receiver                     |
| SERIAL4 | UART4  | PB9/PB8     | DJI O3 MSP DisplayPort            |
| SERIAL6 | USART6 | PC6/PC7     | GPS                               |
| SERIAL7 | UART7  | PE8/PE7     | Spare                             |
| SERIAL8 | UART8  | PE1/PE0     | Spare                             |

## RC Input

RC input is configured by default on SERIAL3 (USART3) for ELRS. CRSF/ELRS,
SBUS and other protocols are supported on any UART by setting the matching
`SERIALn_PROTOCOL`.

SERIAL2 (USART2) is broken out for the DJI O3 Air Unit and defaults to MAVLink;
for DJI SBUS RC on that port set `SERIAL2_PROTOCOL 23` (RCIN).

## Battery Monitoring

Battery monitor 1 is enabled by default with voltage and current sensing
(divider 11.0, current scale 40.0 A/V). A second battery input (BATT2 voltage
and current pins) is wired but left disabled; set `BATT2_MONITOR` to enable it.
Adjust `BATT_VOLT_MULT` / `BATT_AMP_PERVLT` to match the connected power module.

## Motor Outputs

Outputs 1-10 are available. Outputs 1-4 (TIM3/TIM5) support bidirectional
DShot. Output 11 (PA8, TIM1) drives the WS2812 LED strip.

Output group timers: M1-M2 on TIM3, M3-M6 on TIM5, M7-M10 on TIM4, LED on TIM1.
DShot on M7-M10 (TIM4) shares a DMA stream with the OSD SPI.

## Additional I/O

- Analog RSSI input on ADC pin 8 (`BOARD_RSSI_ANA_PIN`); set `RSSI_TYPE 1` to use it.
- Analog airspeed input on ADC pin 4.
- Four user GPIOs, `PINIO1`-`PINIO4`, usable as relays or servo outputs.
- Buzzer output, and a GPIO to control the CAN transceiver silent pin.

## Building

```bash
./waf configure --board novaX-AF-H7_nano
./waf copter
```

## Loading Firmware

The board ships with the ArduPilot bootloader. Firmware can be loaded over USB
with a ground station (Mission Planner / QGroundControl) or with:

```bash
./waf copter --upload
```
