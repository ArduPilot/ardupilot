# CapstoneH753 Flight Controller

A custom 4-layer flight controller built around an STM32H753VI (LQFP100), with a 25 MHz
crystal. ArduPilot treats it as an STM32H743.

## Features

- MCU: STM32H753VI, 480 MHz, 2 MB flash
- IMU: U3 on SPI4, reporting WHO_AM_I 0x73 (LSM6DSV320X / LSM6DSV80X / ISM6HG256X),
  handled by the LSM6DSV driver
- Barometer: MS5611 on I2C2 (address 0x77)
- microSD card on SDMMC2 for logging
- 3 UARTs plus USB
- 8 PWM outputs (4 motors, 4 servos)
- Battery current input

## Pinout

| Connector | Pin 1 | Pin 2 | Pin 3 | Pin 4 |
|---|---|---|---|---|
| J7 power | GND | VBAT | VBAT | BATT_CURR |
| J9 GPS (SERIAL1) | 5V | RX | TX | GND |
| J11 ELRS (SERIAL2) | 5V | TX | RX | GND |
| J6 MAVLink (SERIAL3) | **GND** | RX | TX | **5V** |
| J2 I2C (external) | 5V | SDA | SCL | GND |
| J8 motors | M1 | M2 | M3 | M4 |
| J5 servos | S1 | S2 | S3 | S4 |

J6 is reversed compared with the other connectors, so its cables are not interchangeable.
J9 puts the FC's RX on pin 2, unlike the Pixhawk GPS convention.

## UART Mapping

| Port | UART | Pins (TX/RX) | Default protocol |
|---|---|---|---|
| SERIAL0 | USB | PA12/PA11 | MAVLink2 |
| SERIAL1 | USART1 | PA9/PA10 | GPS |
| SERIAL2 | UART5 | PB13/PB12 | RC input (CRSF/ELRS and others) |
| SERIAL3 | UART8 | PE1/PE0 | MAVLink2, 57600 |

## PWM Output

| Output | Pin | Timer | Use |
|---|---|---|---|
| 1-4 | PA0-PA3 | TIM2 | Motors M1-M4, DShot and bidirectional DShot capable |
| 5-8 | PD12-PD15 | TIM4 | Servos S1-S4 |

Outputs within the same timer must use the same protocol.

## Battery Monitoring

Only current can be measured, on PC0 (`BATT_CURR_PIN` 10). The battery voltage divider is
wired to PC13, which has no ADC channel, so no battery monitor is enabled by default. Rerouting
the divider to PC1, PC2_C, PC3_C or PA8 would fix this. VDDA is about 3.22 V.

## Compass

There is no onboard compass. External compasses on J2 (I2C1) are probed at boot. J2 has no
pull-up resistors fitted, so only the weak internal pull-ups are active; add 2.2k-4.7k
resistors or use a GPS/compass module that has its own.

## Known Hardware Issues

- BOOT0 is sampled high at reset, so the MCU starts the ST ROM bootloader instead of flash.
  Check the TP1-TP2 bridge, the R23 pull-down and the BOOTS net. Until this is fixed, the
  firmware has to be started over SWD.
- IMU U11 on SPI3 (CS PA15) does not respond and is not used.
- No USB host activity has been seen yet; check the cable and the D+/D- path.
- The M95M04 SPI EEPROM on SPI1 is not supported by ArduPilot, so parameters are stored in
  internal flash.
- No LED or buzzer is connected to a GPIO.

## Building

Build in WSL/Linux from a checkout with LF line endings:

```
Tools/scripts/build_bootloaders.py CapstoneH753
./waf configure --board CapstoneH753
./waf copter
```

This produces `build/CapstoneH753/bin/arducopter_with_bl.hex` (bootloader and firmware
together; needs the `intelhex` Python package) and `arducopter.apj`.

## Loading Firmware

Flash `arducopter_with_bl.hex` over SWD (J1) with STM32CubeProgrammer:

```
STM32_Programmer_CLI -c port=SWD mode=NORMAL -w arducopter_with_bl.hex -v
```

Later updates can be loaded as `arducopter.apj` through Mission Planner once the bootloader is
running and USB or SERIAL3 is connected.
