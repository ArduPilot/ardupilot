# AD-TALON V6

AD-TALON V6 is an ArduPilot-compatible flight controller developed by AD SYSTEM. It uses an STM32H753 FMU and a separate STM32F103 IOMCU for RC input, safety handling, and eight I/O outputs.

The hardware definition in this directory reflects a configuration that has been bench-tested and flight-tested by the developer. Inclusion in the upstream ArduPilot repository is subject to maintainer review and merge.

## Hardware

| Component | Hardware / configuration |
| --- | --- |
| FMU | STM32H753IIK6 (STM32H743xx build target), 16 MHz HSE, 2 MB flash |
| IOMCU | STM32F103C8T6, 12 MHz HSE, 64 KB flash |
| ArduPilot board ID | `6500` (`AP_HW_AD_TALON_V6`), pending upstream allocation |
| IMUs | ICM-20649 (SPI1), ICM-42688-P (SPI2), BMI088 (SPI3) |
| Barometers | Two ICP-20100 sensors: I2C2 address `0x63` and I2C4 address `0x64` |
| Magnetometer | RM3100 on internal I2C4, address `0x20` |
| Nonvolatile storage | SPI5 FRAM; 32 KB parameter storage configured |
| Removable storage | microSD connected to SDMMC2 |
| CAN | CAN1 and CAN2 |
| Ethernet | LAN8742A RMII hardware signals defined; network services require separate configuration |
| PWM outputs | Eight IOMCU outputs and eight local FMU outputs |
| USB | FMU USB FS; USB FS is also defined for the FMU bootloader |

## Serial port mapping

The following mapping describes the physical interfaces and labels in `hwdef.dat`. Serial protocols and baud rates are configured through ArduPilot parameters.

| ArduPilot serial port | MCU interface | Hardware label |
| --- | --- | --- |
| SERIAL0 | OTG1 | USB |
| SERIAL1 | UART7 | TELEM1 |
| SERIAL2 | UART5 | TELEM2 |
| SERIAL3 | USART1 | GPS1 |
| SERIAL4 | UART8 | GPS2 |
| SERIAL5 | USART2 | TELEM3 |
| SERIAL6 | UART4 | UART4 |
| SERIAL7 | USART3 | DEBUG |

USART6 is dedicated to the FMU-to-IOMCU link and does not appear in the standard `SERIAL_ORDER` list.

## FMU and IOMCU configuration

The FMU hardware definition is `AD-TALON-V6/hwdef.dat` and the USB bootloader definition is `AD-TALON-V6/hwdef-bl.dat`.

The separate IOMCU hardware definition is `AD-TALON-V6-BASE-IO/hwdef.dat`. It uses a 12 MHz external oscillator and the corresponding STM32F1 clock configuration in `hwdef/common/stm32f1_mcuconf.h`.

The FMU embeds the IOMCU firmware from `Tools/IO_Firmware/iofirmware_adtalon_12mhz.bin` as `io_firmware.bin` using the `ROMFS` directive. The firmware used for the initial hardware integration is included in this branch; reproducible IOMCU builds and update behavior should be checked during upstream review.

## Build

From a configured ArduPilot source tree, the FMU firmware can be built with:

```sh
./waf configure --board AD-TALON-V6
./waf copter
```

The IOMCU definition is maintained separately for IOMCU firmware builds:

```sh
./waf configure --board AD-TALON-V6-BASE-IO
./waf iofirmware
```

Commands describe the intended build targets. A clean build and generated firmware artifact verification must be completed before upstream submission.

## Notes

- The FMU board ID is requested in ArduPilot PR #34979 and is not considered officially allocated until that PR is merged.
- Flight-test completion is reported by the hardware developer; this README does not constitute product certification or independent validation.
