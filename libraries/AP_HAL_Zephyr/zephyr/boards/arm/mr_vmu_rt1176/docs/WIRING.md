# MR-VMU-RT1176 — sensor & bus wiring (extracted from schematics)

Derived from the NXP schematics in this folder (not guessed). **Read the
revision caveats** — the sensor complement is highly revision-dependent, so
the ArduPilot hwdef should probe candidates rather than hard-assume one part.

Sources: `fmu_schematic_fmum-rt117x-01_20230123.pdf` (FMU module, page 7
SENSORS / page 9 MEMORY) and `imu_board_SPF-93959_20250128.pdf` (production
shock-mounted IMU daughterboard, Rev A, page 5).

## FMU module (onboard — `fmum-rt117x-01`)

| Sensor         | Part                           | Bus                         | Addr/CS             | AP driver    | Notes                                                                   |
| -------------- | ------------------------------ | --------------------------- | ------------------- | ------------ | ----------------------------------------------------------------------- |
| IMU (onboard)  | **ICM-20602**                  | SPI1 (SENSOR1)              | nCS0, DRDY1, FSYNC  | Invensense ✓ | U8                                                                      |
| Baro (onboard) | **BMP388**                     | I2C2 (BASE/GPS2/MAG/LED/PM) | 0x76 (chip id 0x50) | BMP388 ✓     | U9, DRDY                                                                |
| Secure element | SE050                          | I2C                         | 0x48                | —            | not an AP sensor                                                        |
| FRAM           | **MB85RS1MT** 1 Mbit / 256 KiB | **FlexSPI2**                | nCS0                | ⚠ special    | NOT a normal LPSPI — Zephyr FRAM/SPI access via FlexSPI2 is non-trivial |
| Boot flash     | MX25UM51345G, 64 MB octal NOR  | FlexSPI1                    | -                   | -            | app+bootloader @ 0x30000000                                             |
| SD card        | MicroSD                        | USDHC1                      | —                   | —            | AP_Logger flight logs                                                   |

> The schematic reads as S26KS512 HyperFlash here. The devicetree declares
> `nxp,imx-flexspi-mx25um51345g`, jedec-id `c2 81 3a`, which is Macronix octal
> NOR, and that is the value used above. The board boots from a FlexSPI
> configuration block built for the octal-SPI part, which a HyperFlash device
> would not answer, so the devicetree matches the silicon. The DTS flash node
> was inherited from NXP's `vmu_rt1170` reference design, which this board is
> based on. Not reconciled against the schematic.

## Shock-mounted IMU daughterboard (via FPC — `SPF-93959` Rev A, 2025 production)

| Sensor          | Part                                | Bus            | Addr/CS                       | AP driver                  | Notes                                                 |
| --------------- | ----------------------------------- | -------------- | ----------------------------- | -------------------------- | ----------------------------------------------------- |
| IMU4 (U8)       | **ICM-45686**                       | SPI2 (SENSOR2) | nCS, DRDY1                    | ⚠ likely **unsupported**   | ICM-456xx family, not the Invensensev3 (426xx) family |
| IMU2 (U1)       | ICM-4xxxx                           | SPI3 (SENSOR3) | nCS0/nCS1, DRDY2              | ⚠ verify                   | second offboard IMU                                   |
| Baro (offboard) | **BMP390**                          | I2C3 (FMU)     | 0x77 (chip id 0x60)           | BMP388 ✓ (handles 388/390) |                                                       |
| Mag (offboard)  | **BMM150** default / **BMM350** alt | I2C4 (SENSOR4) | 0x10 (BMM150) / 0x14 (BMM350) | BMM150 ✓ / BMM350 ⚠        | "BMM350 is an alternative for BMM150 (default)"       |
| Sensor EEPROM   | —                                   | I2C3           | 0x50                          | —                          | factory cal, not an AP sensor                         |

## Revision variance (why probe lists, not hard assumes)

The same logical slots hold different parts across revisions:

- **2023 FMU draft** connector labels: shock IMUs were **BMI088** (SPI3) +
  **ISM330DHCX** (SPI2); onboard **ICM-20602** (SPI1); onboard baro **BMP388**.
- **2025 production IMU board** (SPF-93959 Rev A): shock IMUs are
  **ICM-45686** (SPI2) + a second ICM (SPI3); offboard baro **BMP390**; mag
  **BMM150**→**BMM350** option. Rev history: "Replace ICM42688 with ICM45686",
  "Remove BMM150 → BMM350".
- **PX4 doc** lists yet another set (ICM-20649/BMI088, ICM-42688-P,
  ICM-42670-P, 2× BMP388, BMM150).

**Conclusion:** the physical board must be probed to know for sure. hwdef
should list every plausible candidate so ArduPilot detects whatever is fitted.

### AP driver-support gaps — RESOLVED 2026-07-29

This section previously listed ICM-45686, BMM350 and BMI088 as unsupported and
called ICM-45686 "a real bring-up blocker". **That was wrong** — all three have
ArduPilot drivers in this tree:

- **ICM-45686** — supported by `AP_InertialSensor_Invensensev3`
  (`INV3_ID_ICM45686 0xE9`, `DEVTYPE_INS_ICM45686 = 0x3B`). The driver covers
  the 456xx family, not just 426xx.
- **BMM350** — `AP_Compass_BMM350.cpp`.
- **BMI088** — `AP_InertialSensor_BMI088.cpp`. Still awkward here for a
  different reason: it presents separate accel and gyro chip selects, and its
  `probe()` takes two devices, which the Zephyr hwdef parser's
  `IMU <driver> SPI:<device>` form cannot express. A parser change is needed
  before BMI088 can be declared, but the driver itself exists.

Because `Invensensev3::probe()` auto-detects across all nine supported WHOAMI
values, a single `IMU Invensensev3 SPI:<bus device>` line per sensor bus covers
ICM-42688/42670/42605/40605/40609/45686 and IIM-42652/42653 at once. That is
why hwdef.dat declares one probe per bus rather than one per candidate part.

The one part still genuinely unsupported is **ISM330DHCX** (2023 FMU draft
only; the 2025 production IMU board does not fit it).

## Serial / UART map

The same map twice. Sorted by **UART number** — use this when you have a net or
a pad and want to know which ArduPilot port it becomes:

| UART   | Function                     | ArduPilot                                         |
| ------ | ---------------------------- | ------------------------------------------------- |
| (USB)  | USB CDC ACM (`usb_cdc_acm0`) | SERIAL0 = MAVLink2                                |
| UART1  | Debug console                | SERIAL8 (also Zephyr `zephyr,console`)            |
| UART3  | GPS1                         | SERIAL3 = GPS                                     |
| UART4  | TELEM1                       | SERIAL1                                           |
| UART5  | GPS2                         | SERIAL4 = GPS2                                    |
| UART6  | RC-IN (SBUS, single-wire)    | SERIAL7 — `RCInput.cpp` hardcodes `hal.serial(7)` |
| UART8  | TELEM2                       | SERIAL2                                           |
| UART10 | TELEM3                       | SERIAL5                                           |
| UART11 | External                     | SERIAL6                                           |

Sorted by **ArduPilot SERIAL slot**, i.e. `SERIAL_ORDER` order — use this when
reading `hwdef.dat` or setting `SERIALn_` parameters:

| UART   | Function                  | ArduPilot                                         |
| ------ | ------------------------- | ------------------------------------------------- |
| (USB)  | USB CDC                   | SERIAL0 = MAVLink2                                |
| UART4  | TELEM1                    | SERIAL1                                           |
| UART8  | TELEM2                    | SERIAL2                                           |
| UART3  | GPS1                      | SERIAL3 = GPS                                     |
| UART5  | GPS2                      | SERIAL4 = GPS2                                    |
| UART10 | TELEM3                    | SERIAL5                                           |
| UART11 | External                  | SERIAL6                                           |
| UART6  | RC-IN (SBUS, single-wire) | SERIAL7 — `RCInput.cpp` hardcodes `hal.serial(7)` |
| UART1  | Debug console             | SERIAL8 (also Zephyr `zephyr,console`)            |

**IOMCU: optional on this schematic, NOT fitted on the hardware we have.** The
PX4IO co-processor is a build option that would sit on UART6; the boards in use
here do not populate it, so UART6 carries the single-wire SBUS RC input instead
and all 12 PWM outputs are driven directly by the RT1176. An earlier revision of
this table listed UART6 as "PX4IO (IOMCU), unsupported" with no mention that the
part is optional — that is why this doc and `hwdef.dat` appeared to contradict
each other. Both were half-right.

## RC input — TWO paths, not one (corrected 2026-08-13)

The RC-IN jack feeds **two separate MCU pads simultaneously** — the classic
Pixhawk dual-path RC input. Earlier revisions of this doc listed only the
UART6 path, and that omission propagated as "no spare
pin exists in this board's documented wiring", which drove the misdirected
pad-sharing work documented in
`libraries/AP_HAL_Zephyr/hwdef/mr_vmu_rt1176/README.md`. Both schematic PDFs are NXP copyright and are
git-ignored rather than committed; download them into this directory (see
`README.md` for URLs), then `pdftotext -layout` + grep is enough to re-verify
everything below.

| Path                   | Carrier net           | FMU connector pin | FMU net                    | RT1176 ball | Pad              | Used as                                                                                                               |
| ---------------------- | --------------------- | ----------------- | -------------------------- | ----------- | ---------------- | --------------------------------------------------------------------------------------------------------------------- |
| Serial (SBUS/CRSF/...) | `USART6_TX__RC_INPUT` | 72                | `UART6_TX_TO_IO__RC_INPUT` | K1          | `GPIO_EMC_B1_40` | LPUART6 single-wire (SERIAL7, `AP_RCProtocol` UART autodetect)                                                        |
| Pulse (PPM-SUM/CPPM)   | `FMU_PPM_INPUT`       | 39                | `FMU_PPM_INPUT`            | M2          | `GPIO_EMC_B2_12` | Pad exists and is real, but **is not fed by the RCIN position on this carrier** - see the correction below. Not used. |

> **CORRECTION, hardware-measured 2026-08-13.** An earlier revision of this
> table said PPM capture ran on `gpio2` pin 22 (`GPIO_EMC_B2_12`). It does not.
> With the receiver seated in the carrier's RCIN 3-pin servo-rail position, a
> live SWD poll saw the CPPM pulse train on **`EMC_B1_40`** (~470 transitions/s)
> and **nothing** on `EMC_B2_12`: this carrier's RCIN position feeds only the
> UART6 net. So `rcin-gpios` in the board DTS is `<&gpio2 8>` (`EMC_B1_40`,
> the LPUART6 single-wire pad), used **with** the runtime MUX arbiter, not the
> dedicated PPM pad. The `FMU_PPM_INPUT`/`EMC_B2_12` pad is still real and
> still wired to FMU connector pin 39; it is simply not reachable from the RCIN
> position on this carrier, so pin 22 is not usable here. The board DTS
> `zephyr,user` node comment is authoritative and carries the full measurement.

Notes:

- The FMU's UART6 **RX** pad (`EMC_B1_41`, net `UART6_RX_FROM_IO__NC`) is
  genuinely not connected on this carrier; single-wire mode on the TX pad is
  correct for the serial path.
- Per the Pixhawk FMUv6X standard pin function column in the FMU schematic,
  RC input is nominally on the FMU connector's USART6_RX pin (70); this
  carrier instead wires it to pin 72 (the TX net) for the serial path — a
  deliberate mRo design choice, both columns visible in the connector table
  on the schematic.
- Both RC paths share the one `EMC_B1_40` pad and are arbitrated at runtime
  (`RCIN_PULSE_GPIO_SHARES_UART_PAD 1`): the LPUART6 protocol scan and the GPIO
  edge capture take turns on it. The no-pad-sharing design that the earlier
  `EMC_B2_12` reading implied was never reachable on this carrier.

Connector silkscreen matches the ArduPilot slot number for TELEM1, TELEM2, GPS1
and GPS2. TELEM3 is the exception, at SERIAL5 — which is where Pixhawk6X and the
other Pixhawk-standard boards also put TELEM3. SERIAL3/SERIAL4 are not available
for it: `SerialProtocol_GPS` is the built-in default for both in
`AP_SerialManager.cpp` (project-wide, not per-board), so moving TELEM3 there
would break the GPS1 connector's match to fix TELEM3's.

## Other interfaces

- **CAN**: 3 buses (2 wired for use) — FlexCAN.
- **PWM**: 12 outputs, 8 DShot-capable. The IOMCU that would own the MAIN rail is
  optional and is not fitted on our hardware, so all 12 outputs are FMU-direct.
- **Power**: 2 power inputs with SMBus (POWER1/POWER2).
- **Ethernet**: 100Base-T1, not implemented.
