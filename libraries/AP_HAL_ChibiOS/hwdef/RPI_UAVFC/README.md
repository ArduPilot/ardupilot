# Raspberry Pi UAV Flight Controller

The Raspberry Pi UAV Flight Controller (`RPI_UAVFC`) is an RP2350B flight
controller board. It is a revision of the original Laurel carrier with a
substantially different pinout: the IMU, the microSD socket, the motor outputs
and the barometer bus have all moved, and the secondary blackbox flash has been
removed.

Pin assignments in this target were taken from the R2 Rev C schematic, which is
the authority where it disagrees with the vendor GPIO assignment sheet - it does
so on the ESC channel order and on both regulator enables. Do not
cross-reference the original `Laurel` hwdef for pin numbers; only the clock,
QMI flash timing and SMP configuration are shared between the two.

## Features

 - MCU - RP2350B dual-core Cortex-M33 running at 225 MHz
 - 520 KB SRAM
 - 4 MB boot/XIP flash (QSPI, dedicated `QSPI_SS` chip-select)
 - TDK ICM-56686 IMU on SPI0 (same silicon as the ICM-45686)
 - DPS368 barometer on the internal I2C0 bus (address 0x76)
 - microSD card slot (SPI mode) for logging
 - 2 hardware UARTs plus 2 PIO UARTs
 - 2 I2C buses (internal barometer, external GPS/compass)
 - 4 PWM motor outputs
 - USB CDC serial

## UART Mapping

The board sheet names its ports UART0 through UART3. RP2350 provides only two
hardware UARTs, so the sheet's UART2 and UART3 are served by PIO UARTs.

 - SERIAL0 -> USB (MAVLink2)
 - SERIAL1 -> UART0, GPIO44/45, DVTX connector (DisplayPort, DMA-enabled)
 - SERIAL2 -> UART1, GPIO36/37, GPS connector (GPS, DMA-enabled)
 - SERIAL3 -> PIOUART0, GPIO42/43, RADIO connector (RC Input)
 - SERIAL4 -> PIOUART1, GPIO16/17, VID connector (Spare)

SERIAL4 defaults to `SerialProtocol_None`. A floating RX input on an
unconnected header can generate enough IRQ traffic to stall startup, so enable
it only once wiring is confirmed: set
:ref:`SERIAL4_PROTOCOL<SERIAL4_PROTOCOL>` = 2 for a MAVLink telemetry link.

## RC Input

RC input is on SERIAL3 (the RADIO connector, GPIO42/43), which defaults to
:ref:`SERIAL3_PROTOCOL<SERIAL3_PROTOCOL>` = 23.

 - CRSF requires :ref:`SERIAL3_OPTIONS<SERIAL3_OPTIONS>` = 0
 - FPort requires :ref:`SERIAL3_OPTIONS<SERIAL3_OPTIONS>` = 15
 - SRXL2 requires :ref:`SERIAL3_OPTIONS<SERIAL3_OPTIONS>` = 4, TX pin only

The board also routes an SBUS pad on GPIO41, but SBUS needs an inverted UART
rather than the GPIO edge capture this port provides, so that pad is not
currently supported.

## PWM Output

Four PWM outputs on GPIO6-9, in two groups:

 - PWM 1-2 in group1 (PWM slice 4, GPIO9 and GPIO8)
 - PWM 3-4 in group2 (PWM slice 3, GPIO7 and GPIO6)

The ESC connector is wired in descending order - DSHOT1 is GPIO9 and DSHOT4 is
GPIO6 - so the ArduPilot channel numbers run opposite to the GPIO numbers.

Channels within a group share an output rate. The board sheet labels these
DSHOT1-4, but bidirectional DShot is not yet supported by the RP2350 port and
the outputs run as plain PWM.

## Battery Monitoring

The board has internal voltage and current sense on the RP2350B ADC pins.

The default battery parameters are:

 - :ref:`BATT_MONITOR<BATT_MONITOR>` = 4
 - :ref:`BATT_VOLT_PIN<BATT_VOLT_PIN__AP_BattMonitor_Analog>` = 6 (GPIO46)
 - :ref:`BATT_CURR_PIN<BATT_CURR_PIN__AP_BattMonitor_Analog>` = 7 (GPIO47)
 - :ref:`BATT_VOLT_MULT<BATT_VOLT_MULT__AP_BattMonitor_Analog>` = 11.1
 - :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT__AP_BattMonitor_Analog>` = 1.0

The voltage multiplier was checked against a bench supply and matches the usual
11.1 divider ratio. The current scale is still the placeholder carried over from
the original Laurel; measure it against a known load before relying on it.

## Analog RSSI input

Analog RSSI uses :ref:`RSSI_PIN<RSSI_PIN>` 0, wired to the AN1 spare pad on
GPIO40.

## Compass

There is no builtin compass. An external compass can be attached to the I2C1
bus on the GPS connector (GPIO38 SDA, GPIO39 SCL).

## OSD Support

There is no SPI OSD device on this revision. The board routes discrete
`OSD_W`, `OSD_EN` and `OSD_SYNC` lines on GPIO21-23, but with no accompanying
SPI bus, so no hardware OSD is declared.

MSP DisplayPort is available on SERIAL1 (the DVTX connector) and is the
compiled-in default for that port.

## VTX power control

The 9 V rail feeding the VID connector is switched by GPIO18, and the 5 V
peripheral rail by GPIO19. Both drive an MP4334 EN pin with a pull-down, so
they are active HIGH. The 5 V rail is enabled at boot; the 9 V rail is held off
until the power tree has been validated on this hardware.

## Logging

Logs are written to the microSD card on SPI1 (`LOG_BACKEND_TYPE` = 1, set
`@READONLY` so a stale stored value cannot silently disable logging). There is
no blackbox flash on this board, so the card is the only logging medium.

Parameter storage uses a 32 KB region of the boot XIP flash at 0x10008000,
inside the 64 KB reserve below the application.

## Loading Firmware

Initial firmware load is done over SWD with an external probe, or via the
RP2350 ROM BOOTSEL path. Once the bootloader is in place, updates can be done
with any ArduPilot ground station using the `*.apj` firmware files.

## Known Limitations

| Feature | Status |
|---------|--------|
| DShot / BLHeli / SerialLED | Not supported on current RP2350 target |
| CAN / DroneCAN | Not supported by RP2350 hardware |
| Hardware OSD | No SPI OSD device on this revision; use MSP DisplayPort |
| SBUS pad (GPIO41) | Needs an inverted UART; not currently supported |
| ESC telemetry (GPIO5) | Can only reach UART1 RX, which the GPS owns |
| RGB LED (GPIO2) | Serial LED output not supported on RP2350 |
| Battery current scaling | Placeholder factor carried from Laurel v1 |
| IMU rotation | `ROTATION_NONE`; set `AHRS_ORIENTATION` to match your mounting |
