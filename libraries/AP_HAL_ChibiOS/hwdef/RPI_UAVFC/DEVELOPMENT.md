# RPI_UAVFC development notes

Working notes for the RPI_UAVFC bring-up. `README.md` describes the board as
users see it; this file is for whoever is working on the port. See
`PROFILING.md` for the performance instrumentation.

## Where things stand

The board flies. Six flights so far, all in Stabilize, longest about 40 s.
The hwdef is complete, the pinout is verified against the schematic, and the
timing architecture holds up in the air: the rate thread sits on 2 kHz with a
worst-case dt of 0.8 ms in flight, and the 4 ms XIP parks only ever appear
while disarmed.

| Area | State |
|-----------------------|--------------------------------------------------|
| Pinout | Verified against R2 Rev C schematic |
| Build | `./waf configure --board RPI_UAVFC && ./waf copter` |
| Bootloader | Built, board ID 1215 |
| IMU | Working; fitted part is ICM42688P, see below |
| Barometer | DPS368 detected on I2C0 at 0x76 |
| microSD logging | Working; 19 MB logs at 1 kHz RATE without drops |
| Parameter storage | Working; needs both the sector-bound and write-verify fixes |
| RC input | CRSF/ELRS on SERIAL3, 333 Hz link, 199 Hz telemetry |
| GPS | Detected (ublox at 230400) but has never got a fix |
| Serial ports | SERIAL2/3 confirmed on hardware; SERIAL1/4 untested |
| Battery voltage | Multiplier measured, 11.1 |
| Battery current | Scale 50 (20 mV/A ESC); offset and load check outstanding |
| Motor outputs | 4x DShot600 via PIO; has also flown on PWM at 490 Hz |
| DShot | Bidirectional DShot600 running, eRPM telemetry decoding |
| Compass | None fitted; EKF runs in constant-position mode |
| Rate loop in flight | 2 kHz held, dtMax 0.8 ms, no scheduler overruns |
| Tune | Flyable starting tune, see below; not autotuned yet |

One thing is known-wrong and it is not a board fault:

The GPS is detected and talks (the driver reads its firmware version) but has
never reported a satellite: `NSats` 0 and `HDop` 99.99 in every flight log so
far. Note ArduPilot still marks a no-fix GPS *healthy* - `min_status_for_gps_
healthy()` returns `NONE`, which only excludes "no GPS at all" - so this does
not show up as a failing sensor.

The fitted IMU is an ICM42688P, not the ICM-56686 the schematic shows:
`INS_ACC_ID` 3408130 has top byte `0x34` = 52. So the standard Invensensev3
path is in use and the driver notes below are not needed for this unit.

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

That test has been run twice and the answer is narrower than it first looked.
The park is not the source of the routine jitter, but it does produce the worst
outliers.

The first run suggested innocence: park maxima of 2331 and 3061 us sat next to
`RTlat` glat maxima of 475 and 729 us, while park-free windows ran 732 to
1143 us. A later run contradicts that in one window - a 4097 us park alongside a
4318 us glat maximum, tracking within 221 us, which is a direct hit.

Read together: the baseline 700-1400 us worst case happens with `park n=0`, so
removing the park will not fix the routine jitter and something else is behind
it. But a long park lands on top of that, and a 4 ms freeze is eight missed
iterations at 2 kHz. Storage writes are deferred while armed
(`AP_STORAGE_NO_WRITE_WHILE_ARMED`) and armed windows do show `n=0`, so it
should not reach flight - that guard is the only thing preventing it, and it is
worth confirming nothing else writes flash while armed before trusting it.

## Gyro-to-attitude latency

Retracted: an earlier version of this section claimed the glat average was
bimodal, correlated with flash writes in eleven of twelve windows, and that
~165 us was available by re-phasing the rate loop against the IMU FIFO reads.

That did not reproduce. A later run is flat at 183-198 us across every window,
with the only low value (26 us) in the boot window before the rate loop was
running. One window had 12 parks and glat stayed at 197. The likely explanation
is that the original measurement predates the storage sector-bound fix, when
`AP_FlashStorage` was erasing and rewriting constantly - exactly the kind of
churn that would perturb the phase.

What does hold across both runs: glat averages about 190-200 us and `rtc` is
flat at 12-13 us, so the latency is not rate-controller compute. If you want to
chase it, re-measure first rather than trusting the numbers above.

## Build and flash

```
./waf configure --board RPI_UAVFC
./waf copter
python3 Tools/scripts/build_bootloaders.py RPI_UAVFC   # only if hwdef-bl changes
```

`board_uses_rp2350_bootsel()` in `Tools/ardupilotwaf/chibios.py` decides whether
a board gets the SRAM relocation linker scripts and the BOOTSEL upload path. It
used to match on board name (`laurel*`, `*pico2*`), which meant a rename
silently skipped both and failed at link on a missing scratch section file. It
now reads `env.RP_MCU`, set at configure from the hwdef, so the name no longer
matters.

`chibios_board.mk` in this directory is a standalone RP2350 makefile, not the
common one, and it hardcodes the path to `c1_main.c`. Both files are per-board
copies; if you create another revision, copy and fix the path.

For flashing and SWD debugging see `FLASHING.md`, which has the working
OpenOCD invocation and the flash layout. Note OpenOCD here is a native Windows
binary run from WSL, so it cannot see WSL paths - stage images under `/mnt/c`.

### Diagnostics that cost real time

Three switches, all off in this hwdef. Turn them on to measure, off to fly.

| Define | Cost when on |
|-----------------------------------|-------------------------------------|
| `HAL_ENABLE_THREAD_STATISTICS` | 13.6% of core1, 10.2% of core0 non-idle |
| `AP_RP2350_PC_SAMPLER_ENABLED` | ~5.1 kHz ISR per core, 24 KB BSS |
| `AP_RP2350_DEBUG_REPORT_ENABLED` | negligible CPU; clutters the GCS pane |

Statistics instrument every critical section and context switch. Turning them
off also removes `core1load` from the `Perf` line - `Scheduler::get_core1_load_pct()`
reads `ch1.idlethread.stats.cumulative`, which only exists with
`CH_DBG_STATISTICS`. There is no way to keep the core1 load figure without
paying for the statistics.

Two traps here, both of which cost an afternoon:

- `chibios_board.mk` had `-DHAL_ENABLE_THREAD_STATISTICS` hardcoded in the base
  `UDEFS`. A command-line `-D` with no value is 1 and beats the hwdef, so the
  define looked off in `hwdef.h` while `CH_DBG_STATISTICS` stayed TRUE. If a
  hwdef define appears to have no effect, grep this makefile before anything else.
- The PC sampler guards were `#if defined(...)`, so setting the flag to 0 left
  it compiled in. All the RP2350 flags are value-tested now; keep them that way.

### The ChibiOS library does not rebuild on source edits

ChibiOS is built by a single waf task that shells out to `make`, and its
signature comes from its declared inputs (`hwdef.h`, `ldscript.ld` and so on),
not from anything under `modules/ChibiOS`. Touching a ChibiOS source does
nothing. Delete `build/<board>/modules/ChibiOS` to force it.

## Upstream bugs found during bring-up

Four of these are not board-specific. This port just exercises paths that most
vehicles do not, so they surfaced here first. All are PR candidates.

**The gyro calibration stripped the board rotation from the accel.**
`_init_gyro()` zeroed `_board_orientation` for the duration so its gyro samples
came out in board frame - but that is a single global field, so it stripped the
rotation from the accel too. The last accel published in that window stays in
`_accel[0]`, and `AP_AHRS_DCM::reset()` reads it a few lines later during
`init_ardupilot()`, gating only on the vector *magnitude*. A board-frame 9.81
passes, so DCM aligned to it. On a board mounted inverted that is 180 degrees
out, and DCM's drift correction crawls back at 0.5 deg/s per minute, failing
the attitude pre-arm for the whole of that time. Fixed by skipping the rotation
in the gyro backend while `_calibrating_gyro` is set, alongside the offset
subtraction it already gates. Requires an `AHRS_ORIENTATION` that flips Z to be
visible at all, which is why it has gone unnoticed.

**Sensor health flags had a cross-thread race.** `AP_InertialSensor::update()`
cleared `_gyro_healthy`/`_accel_healthy` and relied on the backends to set them
true again microseconds later. Those flags are read from other threads -
`AP_RCTelemetry::check_sensor_status_flags()` runs from the CRSF frame handler
on the RC input thread - so every main loop left a window in which a healthy
sensor read unhealthy. At 199 Hz telemetry that produced a continuous "Bad Gyro
Health" on a vehicle whose gyro never missed a sample. It is invisible in a log
by construction: `IMU.GH` is written from the main loop, which cannot be inside
`update()` at the same time, so it reads 1 even logged at loop rate. Fixed by
assigning the flag exactly once per cycle in `update_gyro()`/`update_accel()`.

**RP2350 SPI ran at the wrong clock.** `SPIDevice.cpp` hardcoded a 150 MHz
source, but the PL022 is fed by `clk_peri`, which `rp_clocks.c` ties to
`CLK_SYS` with DIV=1 - so it follows the board's PLL. At 225 MHz every
requested speed came out 1.5x: the IMU ran at 11.25 MHz where 8 was asked for,
and the microSD at 28.1 MHz against a 25 MHz SPI-mode limit with its 400 kHz
init clock at 598 kHz. Now derived from `RP_CLK_PERI_FREQ`. Note the SCR clamp
at 255 still bounds the minimum to `clk_peri`/512 = 439 kHz at 225 MHz, so the
400 kHz SD init requirement is still not quite met; that needs an `SSPCPSR`
above the fixed 2.

**`ESC_CALIBRATION` 2 and 3 are a one-way door.** Both block in `while(1)`
unconditionally, and the `set_and_save(ESCCAL_NONE)` they issue first never
reaches flash: `AP_Param::save()` only queues, and both the IO thread that
drains the queue and the storage thread that writes it wait on
`_hal_initialized`, which is set *after* `setup()` returns. Since the
calibration never returns, the clear is never persisted and the board boots
straight back into calibration forever, with no MAVLink up to fix the
parameter. Recovery is a reflash. Mode 1 escapes only by accident of its
throttle check, which lets a later boot fall through and return normally.
**Use the throttle-high procedure (mode 1); never set 3 on this board.**

Note this was initially misdiagnosed as `AP_STORAGE_NO_WRITE_WHILE_ARMED`
swallowing the write, since ESC calibration arms and never disarms. That guard
is a real and separate issue - fixed by draining what was queued before the arm
transition - but it is not what caused the boot loop, because the write never
reaches the storage layer at all.

## Gotchas worth knowing

Anything relocated to SRAM must appear in exactly ONE registry. The linker
claims `.text` sections first-come-first-served, so a symbol listed in two
registries is silently dropped from one. See `PROFILING.md`.

`memcpy` and `memset` are relocated into `.ramtext` because they are the top
flash-resident functions on the core1 rate/IMU path. The relocation has a
boot-order gotcha involving a volatile copy loop; see the memcpy/memset section
of `../Laurel/BASELINE.md` before touching it.

Flash is laid out one region per 64 KB erase block: bootloader in block 0,
parameter storage in block 1 (pages 16-23, using the first 32 KB of it), app
from block 2. Storage and the bootloader used to share block 0, which meant a
64 KB block erase aimed at the bootloader took the parameters with it - and the
bootloader's own `flash_func_erase_apparea_fast()` uses exactly that erase.
Keep `FLASH_RESERVE_START_KB` and `STORAGE_FLASH_PAGE` in `hwdef.dat` in step
with `APP_START_OFFSET_KB` and `APP_START_ADDRESS` in `hwdef-bl.dat`; the app
and the bootloader compute the app base independently and nothing checks that
they agree.

`FLASH_RESERVE_END_KB 0` in `hwdef-bl.dat` is load-bearing. Left unset,
`chibios_hwdef.py` sees a storage page above the bootloader, assumes storage
must therefore be at the top of flash, and reserves everything from it to the
end - which reserved 4032 KB of a 4096 KB part and left no app area at all.

There is no blackbox flash on this board. A sibling branch implemented a QMI M1
driver for a second flash part; it does not apply here.

The DMA channel numbers in `hwdef.h` are advisory. They are generated
STM32-style, but `rp2350_mcuconf.h` gives every SPI channel
`RP_DMA_CHANNEL_ID_ANY`, so the ChibiOS RP drivers take the lowest free channel
and the SPI buses start before the serial ports. `dmaChannelAllocI()` with a
specific id has no fallback and returns NULL when that channel is gone. This is
what kept the GPS off the air: SPI1 had taken channel 2, UART1 RX got NULL, and
`RXDMAE` was still set because `rx_dma_enabled` is a config flag rather than an
allocation result - so the UART raised DMA requests nothing serviced and the
FIFO overran in silence. UART0 only worked because channel 4 happened to be
free. `UARTDriver` now falls back to `RP_DMA_CHANNEL_ID_ANY`; the channel number
carries no meaning because TREQ selects the peripheral. Anything new that wants
DMA on this chip should do the same rather than trust the hwdef number.

Storage sector geometry is set in two places and they must agree.
`AP_FlashStorage` is constructed with `pagesize * AP_FLASH_STORAGE_PAGES_PER_SECTOR`,
and `Storage::_flash_read_data()` bounds the read against the same figure. It
used to bound against a single page while the sector was four, so every read
past the first 4 KB was rejected, `load_sector()` failed and `init()` fell
through to `erase_all()`. The symptom was parameters surviving a few reboots
and then vanishing once enough of them had accumulated to cross 4 KB.

A refused page program used to be invisible. `efl_lld_program()` in the
ChibiOS RP EFL driver ends with an unconditional `return FLASH_NO_ERROR`, and
the `rp_flash_wait_ready()` under it polls only the BUSY status bit, which
never sets if the chip declines the write. Nothing reads WEL or the error bits.
That false success runs all the way up to `AP_FlashStorage`, which clears its
dirty mask and drops the data, so every parameter silently read back whatever
flash already held. `stm32_flash_write()` now reads the range back through XIP
and compares before reporting success, which leaves the dirty bit set so
`_timer_tick()` retries.

Worth knowing when reading that code: the symptom is not "the value you just
set is wrong". Everything reverts, but only the parameter you changed looks
wrong, because the rest were already correct in flash. Diagnosing it means
watching the write frontier in the active sector rather than trusting a
readback of the value itself.

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

## Orientation

The board is mounted inverted in the airframe and the IMU is flipped relative
to the board, so **both** rotations are real and both are needed:

 - `AHRS_ORIENTATION` = 8 (`ROTATION_ROLL_180`) - board to vehicle
 - hwdef `IMU ... ROTATION_PITCH_180` - chip to board

They each flip Z, so the net is `ROTATION_YAW_180` and a level vehicle reads
level. Do not "simplify" this to one rotation without checking how the board
and the chip are actually mounted; the composition is correct, not redundant.

Verify orientation from the gyro, not the compass (there isn't one): nose up
gives positive pitch, right side down positive roll, and yaw clockwise seen
from above positive.

## The tune

Starting gains, arrived at from flight data rather than autotune. The airframe
is roughly 8:1 thrust-to-weight (`MOT_THST_HOVER` learned to 0.125), so stock
ArduPilot defaults - which assume something much heavier and slower - are far
too hot and produce a violent limit cycle before it will even leave the ground.

```
ATC_RAT_RLL_P 0.060   ATC_RAT_RLL_I 0.060   ATC_RAT_RLL_D 0.0008
ATC_RAT_PIT_P 0.060   ATC_RAT_PIT_I 0.060   ATC_RAT_PIT_D 0.0008
```

Halving P and I from the defaults got it flying. What then remained was a
narrow peak at 14 Hz on roll only, carrying about 20% of roll power, with the
roll loop D-dominated (D output 2.2x P output). Halving roll D removed it;
halving pitch D removed the matching 13 Hz peak on pitch. Tracking went from
3.6x demand to about 1.15x.

There is still a residual around 11.4 Hz at roughly 3 deg/s rms. It is stable
across flights and is only about 7% of where this started.

**Do not read the actual/demand ratio when the stick input varies.** A gentle
flight and an aggressive one gave 1.63x and 1.12x with an *identical* 2.95
deg/s residual - the ratio moved entirely because the denominator did. Compare
the absolute amplitude at the peak frequency instead.

The harmonic notch is deliberately still off. The motor fundamental is around
180-190 Hz and appears in the accel, but the gyro is clean there - the
oscillations chased so far were all sub-15 Hz control modes, which a notch
cannot touch. There is no ESC telemetry on this board, so if a notch is ever
needed it has to be throttle-based (`INS_HNTCH_MODE` 1).

## DShot

**Working.** Motors arm and spin on bidirectional DShot600 and the eRPM
telemetry decodes. Bring-up took six separate fixes, listed at the end of this
section; each one masked the next, so the failure never presented the same way
twice.

DShot600 comes out of the PIO, not a timer and DMAR burst, so almost none of
ArduPilot's DShot path applies. `RCOutput_pico.cpp` holds the driver;
`set_group_mode()` and `dshot_send()` branch to it, and `setup_group_DMA()`
and `timer_info()` refuse on this chip because both are built around a timer
clock that does not exist here.

 - `MOT_PWM_TYPE` 6. Any other DShot rate raises a config error at boot rather
   than falling back - the PIO programs are written for DShot600 timing.
 - `SERVO_BLH_BDMASK` selects bidirectional channels, `SERVO_BLH_POLES` scales
   the eRPM.
 - `HAL_DSHOT_ENABLED 1` in the hwdef is the only build-time switch.
   `HAL_WITH_BIDIR_DSHOT` is emitted for every RP2350 board by
   chibios_hwdef.py, deliberately not gated on the BIDIR pin tag: that encodes
   an STM32 timer-pair constraint with no equivalent when each state machine
   turns its own line around.

The programs are assembled from Betaflight's `src/platform/PICO/dshot.pio`,
committed here as `dshot.pio` so the embedded words can be checked. ArduPilot
has no pioasm and requiring one for a single board is not worth the ~45 lines
of table it would save - the state machine setup is register writes either way.

Things that constrain any change here:

 - **PIO2, GPIOBASE 0.** PIOUART owns PIO0 and PIO1 and sets GPIOBASE 16 on
   them to reach GPIO16-47, which would put the motor pins at GPIO6-9 out of
   range. A separate block sidesteps that entirely.
 - **Only one program fits.** 13 and 29 instructions against 32 per block, so
   the block is reloaded when the direction changes. All channels share a
   direction, so this only happens at mode-set.
 - **The bidirectional decode assumes a 75MHz PIO.** It converts sample counts
   to bit times against that constant, so a fractional divider would put the
   decode on the wrong scale rather than merely adding jitter. There is a
   static_assert that the system clock is a multiple of 75MHz; 225 gives 3.
   The non-bidirectional program has no such constraint and takes 9.375.
 - **No DMA.** FIFOs are read and written directly, which avoids the
   allocation trap that silently killed the GPS (see the DMA note below).

The GCR decode is shared with the timer path (`bdshot_decode_gcr()`); only the
recovery of run lengths differs, because input capture measures edge times
while the PIO oversamples the line at 5.56 samples a bit.

Expect a poor telemetry error rate. Betaflight's own note on this code says
5-8% of frames fail to decode with motors spinning, against under 1% at rest,
and that feeds the harmonic notch here. Telemetry decoding now, but the rate
has not been measured - `_bdshot.erpm_clean_frames[]` against
`_bdshot.erpm_errors[]`, both reset every 5 s, is the figure to take before
letting it drive a notch.

### What it took to get here

In order found. The first four are RP2350 hardware details, the last two are
places where shared ArduPilot code assumed an STM32.

1. **`set_output_mode()` downgraded to PWM before the PIO path ran.** The
   generic code checks `mode_requires_dma()` against `have_up_dma` and falls
   back to `MODE_PWM_NORMAL`. There is no UP DMA here and none needed, so
   RP2350 is exempted from that check.
2. **FUNCSEL 11 routed the pads to the aux UART.** PIO2 is FUNCSEL 8 on
   RP2350 (PIO0 is 6, PIO1 is 7). The state machines ran and nothing reached
   the pin.
3. **The frame went into the high half of the FIFO word.** Both programs open
   with `out y, 16` to discard the top half, so the frame belongs in the low
   half. In the high half the discard eats the frame itself and sixteen zeros
   go out - a well formed packet meaning throttle zero, which an ESC accepts
   and sits on, so the output looked alive.
4. **`dshot_state` stuck at `SEND_COMPLETE`.** Nothing returns it to `IDLE`
   without a DMA completion interrupt, so exactly one frame left the board at
   boot. `send_pulses_DMAR()` now sets `IDLE` directly on this chip.
5. **Parameters never reached flash**, so `SERVO_BLH_BDMASK` reverted to 0 on
   every reboot and the bidirectional path was never entered at all. See the
   storage section - the QSPI driver cannot report a refused page program.
6. **The checksum was not inverted for bidirectional.** `create_dshot_packet()`
   took its direction from `group.bdshot.enabled`, which is only set once a
   timer input capture DMA handle is held. There is no input capture here, so
   it was always false while the PIO ran the inverted program selected from
   `SERVO_BLH_BDMASK`. Inverted waveform, plain checksum, every frame rejected
   - so enabling `BDMASK` stopped the motors arming rather than merely failing
   to produce telemetry. It now reads `is_bidir_dshot_enabled()`, the same
   source the PIO program selection and the telemetry read path use.

## Logging setup for tuning work

The stock `LOG_BITMASK` logs `RATE` at **10 Hz**, which aliases anything
interesting into nonsense - a 14 Hz oscillation reads as a random walk of
+/-250 deg/s. `ArduCopter/rate_thread.cpp` picks between `fast_logging_rate`
(1 kHz) and `medium_logging_rate` (10 Hz) purely on one bit.

```
LOG_BITMASK 442367     # 180222 + bit 0 (ATTITUDE_FAST) + bit 18 (IMU_FAST)
LOG_DISARMED 2
```

Bit 0 gives `RATE`/`PID` at 1 kHz; bit 18 moves `IMU` from ~7 Hz to loop rate.
Keep `INS_LOG_BAT_MASK` 3 and `INS_LOG_BAT_OPT` 4 - the pre/post-filter batch
samples are the only way to see the spectrum above the loop rate. Expect
around 19 MB for a 40 s flight; the card keeps up.

## Battery failsafe

`BATT_LOW_VOLT` shipped at 21.6 V, which on 6S is 3.60 V/cell and fires a Land
in mid-discharge on a healthy pack. It did exactly that on flight 17. For 6S
use 21.0 (3.5 V/cell) low and 19.8 (3.3 V/cell) critical. Sag compensation
(`BATT_FS_VOLTSRC` 1) is the better answer but is useless until the current
scale above is fixed.

## Next steps

1. Set `BATT_AMP_OFFSET` and verify the current scale against a known load -
   compare logged mAh with what the charger puts back. `HAL_BATT_CURR_SCALE` is
   now 50, but re-read the idle voltage on `BATT_CURR_PIN` first: the 0.61 V
   measured before was taken through a pad whose input buffer and pulls were
   never configured for analog use, so it may not be the sensor's real
   quiescent point.
2. Fix the battery failsafe thresholds, per above.
3. Measure the bidirectional telemetry error rate before letting it drive the
   harmonic notch, and fly DShot600 - the flights so far were on PWM. With
   eRPM available `INS_HNTCH_MODE` 3 becomes possible, which the board could
   not do at all before.
4. Establish whether flash page programs now succeed first time or only on the
   retry after a failed verify. Harmless either way for correctness, but a
   retry every time means the underlying QSPI defect is still there and double
   the flash wear. One counter on the `memcmp` mismatch answers it.
5. Run AUTOTUNE, one axis at a time. The tune is stable enough to start from
   and this is the point at which the 8:1 thrust-to-weight gets measured
   instead of guessed at.
6. Find out why the GPS never gets a fix. It is detected and communicating, so
   this is antenna, siting or configuration rather than the port.
7. Attack core0's flash share, starting with the veneers - see the veneer
   section in `PROFILING.md`. Core1 is done and needs nothing further.
8. Bring up SERIAL1 and SERIAL4. SERIAL2 and SERIAL3 are confirmed.
9. Re-check the QMI flash timing if this revision fits a different flash part.
   `RP_QMI_CLKDIV 3` / `RP_QMI_RXDELAY 2` were characterised on the v1 part.
10. Re-measure glat before acting on it, per the retraction above.
