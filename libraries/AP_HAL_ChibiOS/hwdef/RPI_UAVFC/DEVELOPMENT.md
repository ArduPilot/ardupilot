# RPI_UAVFC development notes

Working notes for the RPI_UAVFC bring-up. `README.md` describes the board as
users see it; this file is for whoever is working on the port. See
`PROFILING.md` for the performance instrumentation.

## Where things stand

The hwdef is complete and builds, and the pinout is verified against the
schematic. The board has not yet flown. The IMU does not currently probe, and
that is the one thing blocking everything else.

| Area | State |
|-----------------------|--------------------------------------------------|
| Pinout | Verified against R2 Rev C schematic |
| Build | `./waf configure --board RPI_UAVFC && ./waf copter` |
| Bootloader | Built, board ID 1215 |
| IMU | BLOCKED - ICM-56686 not supported by the driver |
| Barometer | DPS368 on I2C0 at 0x76, untested on hardware |
| microSD logging | Untested on this revision |
| Serial ports | Wiring verified, protocols untested |
| Battery monitoring | Scale factors are placeholders from v1 |

## The IMU blocker

The fitted part is a TDK ICM-56686. `AP_InertialSensor_Invensensev3::
check_whoami()` recognises ICM40605, ICM40609, ICM42605, ICM42688 P and V,
IIM42652, IIM42653, ICM42670 and ICM45686. The ICM-56686 is not among them,
and the ICM-456xy path accepts only `0xE9` at register `0x72`.

The part is very likely register-compatible with the ICM-45686, but its WHOAMI
value has not been confirmed. Do not guess it. Either read WHOAMI from the
hardware over SWD, or add a temporary print of both WHOAMI registers
(`0x75` and `0x72`) in `check_whoami()` and read it from the boot messages.

Note the failure is now loud: `HAL_INS_ALLOW_NO_SENSORS` is deliberately NOT
set for this board. On Laurel v1 it was, and a failed probe silently
substituted `AP_InertialSensor_NONE`, which synthesises a flat 0.01 on each
axis with no gravity term. That produced plausible-looking but wrong IMU data
and cost a day of chasing a phantom performance problem. Leave the flag off.

## Pin data provenance

Two sources exist and they disagree. The R2 Rev C schematic in the vendor
support pack is authoritative; the GPIO assignment spreadsheet is wrong in
three places:

- ESC channel order. The connector is wired descending: DSHOT1 is GPIO9 and
  DSHOT4 is GPIO6. The sheet lists them ascending.
- Regulator enables. GPIO18 is the 9V rail and GPIO19 the 5V rail, the
  opposite of the sheet, and both are active HIGH. The sheet's "ENn" naming
  implied active low. Each drives an MP4334 EN pin with a pull-down.
- Sensor part numbers. The IMU is an ICM-56686 and the baro a DPS368, not the
  ICM42688P and DPS310 the sheet names.

Everything else in the sheet is confirmed: SPI0, SPI1, both I2C buses, all
four serial ports, ADC channels, LED polarity and the buzzer drive.

Unlike Laurel v1, SWCLK/SWDIO/RUN are dedicated package pins, so there is no
collision between SWD and the IMU chip-select. The v1 hwdef comment warning
about that does not apply here.

## Performance architecture, inherited from v1

The configuration is carried over unchanged from the v1 baseline, which was
validated on hardware. See `../Laurel/BASELINE.md` for how it was derived and
`../Laurel/xip-cache-and-pgo.md` for the XIP cache analysis behind it.

Core clock is 225 MHz at 1.15 V, down from an original 375 MHz / 1.30 V
overclock. The overclock existed only to hide XIP flash latency: both cores
fetch through a shared 16 KB XIP cache, and core0's EKF plus core1's rate loop
were thrashing it. Moving core1's hot path into SRAM freed the cache for core0
and let the clock come back down.

Work is split as main loop 200 Hz on core0 (nav, EKF inline, GCS, logging) and
a 2 kHz rate thread pinned to core1, fed by a 4 kHz gyro backend. Core affinity
is set in `hwdef/common/rp2350_core_affinity.h`: the SPI buses are on core1,
I2C on core0. Only the rate thread (`ArduCopter/Copter.cpp`, via
`thread_create_pinned_to_core`) and the SPI bus threads run on core1;
everything else, including logging, is on core0.

The EKF runs inline in the main loop, not as a thread. Earlier threaded-EKF
scaffolding was removed because it added a one-tick lag and shared-state
locking for no benefit at these loop rates. Do not reintroduce it.

## The XIP-off / core1 park problem

This is the biggest remaining performance issue and it is worth understanding
before touching anything flash-related.

Setting `DIRECT_CSR_EN=1` for a flash write disables instruction fetch for
BOTH cores. To stop core1 faulting, `rpEflBeforeXipOff()` in
`hwdef/common/board_rp2350.c` rings core1's SIO doorbell (IRQ26) and spin-waits
until core1 parks itself in an SRAM-resident handler. Core1 is frozen for the
whole operation.

In the ChibiOS EFL driver (`modules/ChibiOS/os/hal/ports/RP/LLD/EFLv1/
rp_efl_lld.c`) the park brackets the entire multi-page program loop, not each
page, so an 8 KB parameter write freezes core1 for the full duration. Erase is
bracketed the same way.

The intended fix is to make the park unnecessary rather than shorter: once
every instruction and every constant core1 can reach during the window is
SRAM-resident, `rpEflBeforeXipOff()` can become a no-op and core1 runs straight
through. That is what the SRAM relocation work has been building toward.

Two cautions before removing the park. Instruction relocation alone is not
sufficient - const data and compiler-generated literal pools reachable from
core1 must also be in SRAM. A boot crash from exactly that cause was seen
during the QMI flash work on a sibling branch. And every ISR that can fire on
core1, plus the ChibiOS context-switch path, has to be covered too.

`rp2350_xip_park_stats()` reports park count and worst-case duration; it is
emitted as the `XIPpark:` line by `perf_report`. If the park maximum tracks the
`RTlat` glat maximum, the lockout is the jitter source.

## Build and flash

```
./waf configure --board RPI_UAVFC
./waf copter
python3 Tools/scripts/build_bootloaders.py RPI_UAVFC   # only if hwdef-bl changes
```

The board build path is gated on the board name: `board_uses_rp2350_bootsel()`
in `Tools/ardupilotwaf/chibios.py` matches names starting with `laurel` or
containing `pico2`. A board that does not match silently skips the RP2350
linker script generation and fails at link on a missing scratch section file.

`chibios_board.mk` in this directory is a standalone RP2350 makefile, not the
common one, and it hardcodes the path to `c1_main.c`. Both files are per-board
copies; if you create another revision, copy and fix the path.

For flashing and SWD debugging use the `flash-debug-hardware` skill rather than
hand-rolling OpenOCD invocations.

## Gotchas worth knowing

Anything relocated to SRAM must appear in exactly ONE registry. The linker
claims `.text` sections first-come-first-served, so a symbol listed in two
registries is silently dropped from one. See `PROFILING.md`.

`memcpy` and `memset` are relocated into `.ramtext` because they are the top
flash-resident functions on the core1 rate/IMU path. The relocation has a
boot-order gotcha involving a volatile copy loop; see the memcpy/memset section
of `../Laurel/BASELINE.md` before touching it.

Parameter storage sits at 0x10008000 in the 64 KB reserve below the app, on the
4 MB boot flash. There is no blackbox flash on this board. A sibling branch
implemented a QMI M1 driver for a second flash part; it does not apply here.

`defaults.parm` deliberately does not set `AHRS_ORIENTATION`, `FRAME_CLASS`,
`FRAME_TYPE`, `COMPASS_ENABLE` or the harmonic notch parameters. Those are
mounting and airframe choices, not board properties. Laurel v1 baked in
`AHRS_ORIENTATION 8 @READONLY`, which could not be corrected from a GCS and
made a wrong-orientation fault very hard to diagnose.

## Next steps

1. Identify the ICM-56686 WHOAMI and add driver support. Nothing else can be
   tested on hardware until the IMU probes.
2. Confirm the DPS368 responds at 0x76.
3. Verify microSD mount and logging.
4. Measure the battery voltage and current dividers and replace the v1
   placeholders in `hwdef.dat`.
5. Determine the mounting orientation and set `AHRS_ORIENTATION` as a user
   parameter, not a board default.
6. Re-check the QMI flash timing if this revision fits a different flash part.
   `RP_QMI_CLKDIV 3` / `RP_QMI_RXDELAY 2` were characterised on the v1 part.
7. Establish a fresh profiling baseline and compare against v1's numbers.
