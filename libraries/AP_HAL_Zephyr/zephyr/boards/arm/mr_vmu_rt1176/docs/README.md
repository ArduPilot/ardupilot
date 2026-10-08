# MR-VMU-RT1176 — reference docs

Local reference material for bringing up the NXP MR-VMU-RT1176 (i.MX RT1176,
Cortex-M7) so sensor/bus/connector wiring is derived from documentation, not
guessed.

The board is NXP's open-source reference design implementing the **Pixhawk
FMUv6X-RT** open standard: an FMU module (same as Pixhawk 6X-RT) on an
NXP carrier board. It is fully supported by PX4, whose board config is the most
precise machine-readable wiring source.

## Reference documents — fetch locally, not committed

These are NXP copyright and freely downloadable, so they are **git-ignored, not
stored here** (they total ~56 MB). Download them into this folder when you need
them; `.gitignore` already covers the filenames below.

| File                                                  | What                                                                                             | Size   | Source                                                                                                      |
| ----------------------------------------------------- | ------------------------------------------------------------------------------------------------ | ------ | ----------------------------------------------------------------------------------------------------------- |
| `IMXRT1170RM_reference_manual_rev3.pdf`               | **Register reference manual** (memory map, every peripheral register), IMXRT1170RM Rev 3         | 34 MB  | [direct](https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6465/IMXRT1170RM%20manual%20REV3.pdf) |
| `fmu_schematic_fmum-rt117x-01_20230123.pdf`           | **FMU module schematic**, the MCU + sensor domain (IMUs, baro, mag, FRAM, SPI/I2C wiring)        | 2.4 MB | [NXP-FMUMRT](https://github.com/NXP-Robotics/NXP-FMUMRT)                                                    |
| `carrier_schematic_fmuvxX-rt117x-baseT1_20240404.pdf` | **Carrier board schematic** (RC02, 2024), connectors, power, servo/PWM, IOMCU, CAN, ethernet, RC | 0.9 MB | [NXP-FMUMRT](https://github.com/NXP-Robotics/NXP-FMUMRT)                                                    |
| `imu_board_SPF-93959_20250128.pdf`                    | **IMU board schematic**                                                                          | 0.1 MB | [NXP-FMUMRT](https://github.com/NXP-Robotics/NXP-FMUMRT)                                                    |

The grep recipes in `libraries/AP_HAL_Zephyr/DEBUGGING.md` search a text dump of the
reference manual. Produce it locally after downloading the PDF:

```sh
pdftotext -layout IMXRT1170RM_reference_manual_rev3.pdf \
  IMXRT1170RM_reference_manual_rev3.no-images.txt
```

**Datasheet** (electrical / package pinout, IMXRT1170AEC/CEC) is gated behind
NXP's doc portal login. Get it from the
[NXP RT1170 product page](https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt1170-1-ghz-crossover-mcu-with-arm-cortex-cores:i.MX-RT1170).

## Online sources

- Reference-manual latest-rev notice: <https://www.nxp.com/pcn/202505026I>
- Board product page + design files: <https://www.nxp.com/part/MR-VMU-RT1176>
- Full hardware design repo (schematics, Altium, dimensions, IMU board):
  <https://github.com/NXP-Robotics/NXP-FMUMRT>
- Carrier-board connector pinouts (gitbook): <https://nxp.gitbook.io/vmu-rt1176/production-v1-carrier-board-connectors>
- PX4 flight-controller guide (sensors, ports): <https://docs.px4.io/main/en/flight_controller/nxp_mr_vmu_rt1176>
- PX4 board config (exact SPI bus/CS/driver): `boards/nxp/fmurt1176/` in
  <https://github.com/PX4/PX4-Autopilot>
- Pixhawk FMUv6X-RT open standard: <https://dronecode.org/announcing-the-pixhawk-fmuv6x-rt/>

## Wiring

**`WIRING.md` in this folder is the current, authoritative wiring reference**,
derived from the schematics above. `../../../../hwdef/mr_vmu_rt1176/hwdef.dat`
is the machine-readable source of truth for what actually gets probed.

Sensor bring-up completed 2026-07-20..30. The parts on the board are not what
the first guessed hwdef assumed, so hwdef.dat now declares a **probe policy**:
each SPI chip-select is offered to more than one driver and detection decides.

|         | On the board                                            | hwdef.dat today                                                   |
| ------- | ------------------------------------------------------- | ----------------------------------------------------------------- |
| IMU x3  | ICM-20649 / BMI088, ICM-42688-P, ICM-42670-P (see note) | `IMU Invensensev3` + `IMU Invensense` on each of `imu_sensor1..3` |
| Baro x2 | 2x BMP388                                               | `BARO BMP388 I2C:1:0x76`, `I2C:2:0x77`                            |
| Compass | BMM150                                                  | `COMPASS BMM150 I2C:2:0x10 true`                                  |

> Note: the device on `lpspi1` reports WHOAMI `0x44`, which is an **ICM-42686-P**.
> That part is not in AP's Invensensev3 supported list. See the memory note
> `../../../../../COMPARED_TO_CHIBIOS.md` for status.

IOMCU is **optional and not fitted** on this board (confirmed in `WIRING.md`),
so all 12 outputs are FMU-direct and the UART6 PX4IO path is unused. That
removes the co-processor output limitation that CubeOrange has.

### Serial map

`SERIAL_ORDER` in hwdef.dat is authoritative. As configured:

```text
SERIAL_ORDER OTG1 USART4 USART8 USART3 USART5 USART10 USART11 USART6 LPUART1
```

| UART            | Function                           |
| --------------- | ---------------------------------- |
| OTG1            | USB CDC console + MAVLink          |
| UART1 (LPUART1) | Debug console                      |
| UART3           | GPS1                               |
| UART4           | TELEM1                             |
| UART5           | GPS2                               |
| UART6           | PX4IO (IOMCU) - not fitted, unused |
| UART8           | TELEM2                             |
| UART10          | TELEM3                             |
| UART11          | External                           |

Other: **3 CAN** buses (2 wired for use), **12 PWM** outputs (8 DShot-capable),
100Base-T1 ethernet, I2C port, 2 power inputs (SMBus).
