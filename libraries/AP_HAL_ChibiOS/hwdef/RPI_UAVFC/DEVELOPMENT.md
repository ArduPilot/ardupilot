# RPI_UAVFC development notes

Working notes for the RPI_UAVFC bring-up. `README.md` describes the board as
users see it; this file is for whoever is working on the port. See
`PROFILING.md` for the performance instrumentation.

## Where things stand

The hwdef is complete and builds, and the pinout is verified against the
schematic. Both sensors, parameter storage and microSD logging are confirmed
on hardware, as is an armed run with CRSF RC and motors turning. The items
still marked untested below are exactly that.

| Area | State |
|-----------------------|--------------------------------------------------|
| Pinout | Verified against R2 Rev C schematic |
| Build | `./waf configure --board RPI_UAVFC && ./waf copter` |
| Bootloader | Built, board ID 1215 |
| IMU | Working on hardware; fitted part varies, see below |
| Barometer | DPS368 detected on I2C0 at 0x76 |
| microSD logging | Working on hardware |
| Parameter storage | Working, accel cal persists across reboot |
| RC input | CRSF on SERIAL3 working |
| Serial ports | SERIAL3 confirmed; SERIAL1/2/4 protocols untested |
| Battery voltage | Multiplier measured, 11.1 |
| Battery current | Scale is still the v1 placeholder |
| Motor outputs | Armed and spun on the bench; channel order not yet confirmed against the frame |

## The IMU

The board line does not carry a single part. The R2 Rev C schematic shows a TDK
ICM-56686; earlier samples carry an ICM42688P. The hwdef therefore names the
SPI device `imu1` rather than after any part, following the convention most
boards already use.

This needs no hwdef change to switch, because Invensensev3 probes both WHOAMI
registers - `0x75` for the ICM-426xx family, `0x72` for ICM-456xy - and
configures whichever it finds.

An actual ICM-56686 is a different chip, not a rebadged ICM-45686, and would
need driver work rather than a hwdef edit. Its register map diverges from the
ICM-45686 that ArduPilot's ICM-456xy path assumes: `PWR_MGMT0` at `0x14`
rather than `0x10`, `ACCEL_CONFIG0` at `0x1F` rather than `0x1B`,
`GYRO_CONFIG0` at `0x20` rather than `0x1C`. Note ArduPilot reads `FIFO_DATA`
at `0x14`, which is `PWR_MGMT0` on the ICM-56686, so the maps cannot be used
interchangeably. There is a Betaflight driver on the `accgyro_spi_icm56686`
branch of `mjs1441/betaflight` to model any such work on.

To confirm which part is fitted, read `INS_ACC1_ID` and take the top byte
(divide by 65536): `0x34` (52) is ICM42688, `0x3B` (59) is ICM45686.

### ICM-56686 notes for whoever writes the driver

Deferred until a datasheet is available. Everything below is second-hand, read
off the Betaflight driver rather than a TDK document, so treat the datasheet as
authoritative where it disagrees.

| | ICM-45686 (ArduPilot) | ICM-56686 (Betaflight) |
|-------------------|-----------------------|------------------------|
| `WHO_AM_I` | `0x72` | `0x72` |
| `PWR_MGMT0` | `0x10` | `0x14` |
| `ACCEL_CONFIG0` | `0x1B` | `0x1F` |
| `GYRO_CONFIG0` | `0x1C` | `0x20` |
| IREG addr/data | `0x7C`/`0x7D`/`0x7E` | same |
| `REG_MISC2` | `0x7F` | same |

`FIFO_DATA` is `0x14` in ArduPilot's ICM-456xy path, which is `PWR_MGMT0` on
the ICM-56686. The maps are not interchangeable.

Two traps worth knowing before writing any of it:

- Gyro full-scale encoding differs. On the ICM-56686, `GYRO_FS_SEL` `0x0 << 4`
  selects 4000 dps and `0x1 << 4` selects 2000 dps. ArduPilot's ICM-456xy setup
  writes `0x0 << 4` intending 2000 dps, so carrying that across unchanged would
  give a silent 2x gyro scale error.
- The chip powers up in 20-bit big-endian sample format. Betaflight explicitly
  switches it to 16-bit little-endian via `SREG_CTRL` at `IPREG_TOP1_BASE +
  0x60` (`SIFS_20BITS_EN` bit 3, `DATA_ENDIAN_SEL_BIG` bit 1).

Other facts collected: IREG bases `IPREG_SYS1 0xA400`, `IPREG_SYS2 0xA500`,
`IPREG_TOP1 0xA200`; IREG access is an auto-incrementing burst of addr-high,
addr-low, data with CS held low, polling `REG_MISC2` bit 0 for done with a 4 us
minimum gap. Data registers are `ACCEL_DATA_X1 0x00`, `GYRO_DATA_X1 0x06`,
`TEMP_DATA0 0x0C`. Accel FS codes are 32G/16G/8G/4G/2G at `0x00`-`0x04 << 4`;
ODR codes are 6.4k `0x03`, 3.2k `0x04`, 1.6k `0x05`, 800 `0x06`. Betaflight
reads samples directly rather than through a FIFO, so it is a starting point
for register setup but not for ArduPilot's FIFO-based sampling.

Source: `mjs1441/betaflight`, branch `accgyro_spi_icm56686`,
`src/main/drivers/accgyro/accgyro_spi_icm56686.c`.

The quickest health check is the `ICM dbg:` line. Gravity magnitude should come
to about 9.81 across the three accel axes, the gyro should sit near zero at
rest, and `t=` should read a plausible room temperature - temperature is
decoded from the same FIFO packet as accel and gyro, so a sane value confirms
the packet layout and register map are right.

Failure is loud here: `HAL_INS_ALLOW_NO_SENSORS` is deliberately not set. On
Laurel v1 it was, and a failed probe silently
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

Worth understanding before touching anything flash-related, but no longer the
top performance item - see the measurement at the end of this section.

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
emitted as the `XIPpark:` line by `perf_report`.

That test has now been run and the park came out innocent. Across twelve
`perf_report` intervals on hardware, park maxima of 2331 and 3061 us sat
alongside `RTlat` glat maxima of 475 and 729 us, while park-free intervals ran
732 to 1143 us. Only one interval of the twelve was high on both. The rate loop
therefore sees 700-1100 us worst-case latency whether or not a flash write
happened, so the lockout is not the jitter source and removing the park will
not fix it. Chase the phase/batching effect below first.

The park is still a correctness hazard and the SRAM relocation still has to be
complete before it can go, so nothing above is wrong - it is just not where the
jitter is coming from.

## Gyro-to-attitude latency is phase-dependent

The `RTlat` glat average is bimodal: it sits at 197-210 us in some ten-second
windows and 34-116 us in others, with nothing in between. Rate-controller
compute (`rtc`) is a flat 13 us in every window, so this is not load.

The cause looks like a beat between the IMU FIFO read cadence and the rate
loop. The `ICM dbg` counters give roughly 3040 FIFO reads/s delivering 4055
samples/s against a 2026 Hz rate loop, so a read yields 1.3 samples on average
and the rate thread sometimes drains a batch - the last sample out of a batch
is old by the time the controller finishes with it. Which regime you land in
appears to be set by the last disturbance: in eleven of twelve windows, the low
average coincided with a window containing a flash write, which stalls core1
briefly and re-phases the two rates.

There is roughly 165 us of gyro-to-attitude latency available here, which is
more than anything left in the relocation work. Locking the rate loop to sample
arrival rather than letting it free-run against the backend is the thing to try.

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

Storage writes are deferred entirely while armed via
`AP_STORAGE_NO_WRITE_WHILE_ARMED`. A boot-flash write parks core1 for the whole
operation, which the 2 kHz rate loop cannot absorb, and stock ArduPilot only
guards the full sector erase. Nothing is lost: `_timer_tick()` clears a dirty
line only after a successful write, so pending data sits in the RAM buffer and
flushes on disarm.

Editing `hwdef.dat` requires a reconfigure. `./waf copter` on its own will not
regenerate `hwdef.h`, and the build will silently succeed without the change.
Check the define landed in `build/RPI_UAVFC/hwdef.h` if a hwdef edit appears to
have no effect.

`defaults.parm` deliberately does not set `AHRS_ORIENTATION`, `FRAME_CLASS`,
`FRAME_TYPE`, `COMPASS_ENABLE` or the harmonic notch parameters. Those are
mounting and airframe choices, not board properties. Laurel v1 baked in
`AHRS_ORIENTATION 8 @READONLY`, which could not be corrected from a GCS and
made a wrong-orientation fault very hard to diagnose.

## Next steps

1. Chase the glat phase effect above. It is worth more than anything left in
   the relocation work.
2. Attack core0's flash share, starting with the veneers - see the veneer
   section in `PROFILING.md`. Core1 is done and needs nothing further.
3. Bring up the remaining serial ports: GPS on SERIAL2, then SERIAL1 and
   SERIAL4. SERIAL3 is confirmed working with CRSF.
4. Verify motor output channel order against the frame before fitting props.
   The outputs spin, but the schematic reversed the order relative to the
   vendor sheet and that has not been checked end to end.
5. Measure the current shunt against a known load and replace the v1
   placeholder `HAL_BATT_CURR_SCALE`. Voltage is done.
6. Determine the mounting orientation and set `AHRS_ORIENTATION` as a user
   parameter, not a board default.
7. Re-check the QMI flash timing if this revision fits a different flash part.
   `RP_QMI_CLKDIV 3` / `RP_QMI_RXDELAY 2` were characterised on the v1 part.
