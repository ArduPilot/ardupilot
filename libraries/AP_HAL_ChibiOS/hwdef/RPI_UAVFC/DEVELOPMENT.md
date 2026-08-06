# RPI_UAVFC development notes

Working notes for the RPI_UAVFC bring-up. `README.md` describes the board as
users see it; this file is for whoever is working on the port. See
`PROFILING.md` for the performance instrumentation.

## Where things stand

The board flies, and now flies acro. Six flights on PWM, three on bdshot in
Stabilize (logs 51-53, longest about 35 s), Loiter plus a partial AUTOTUNE in
log62, and now log69 - 153 s of ACRO at the full `ACRO_RP_RATE` 800 deg/s,
inverted, followed by 28 s of Loiter. log70 is a 119 s Loiter flight from the
same session.

The hwdef is complete, the pinout is verified against the schematic, and the
timing architecture is no longer the limiting factor in anything. Across the
whole 280 s of log69, including 807 deg/s rolls, +/-177 deg roll attitude and
2.75 g, **no `RTDT` record exceeds 1.4 ms** against a 500 us period, and the
main loop held `MaxT` 5042-5086 us against a 5000 us budget with `NLon` 0. No
accel clipping in either flight despite `VIBE.VibeY` peaking at 33.9 m/s/s.

What the two flights did surface is that the SD logging ceiling is CPU
starvation on core0, not the card. See the SD section below - the previous
"~100 KB/s card ceiling" conclusion is retracted.

| Area | State |
|-----------------------|--------------------------------------------------|
| Pinout | Verified against R2 Rev C schematic |
| Build | `./waf configure --board RPI_UAVFC && ./waf copter` |
| Bootloader | Built, board ID 1215 |
| ChibiOS | ArduPilot fork, kernel RT 7.0.6 - see below, do not bump it |
| IMU | Working; fitted part is ICM42688P, see below |
| Barometer | DPS368 detected on I2C0 at 0x76 |
| microSD logging | Throughput is core0-CPU-bound, 18-91 KB/s - see below |
| Parameter storage | Working; needs both the sector-bound and write-verify fixes |
| RC input | CRSF/ELRS on SERIAL3, 333 Hz link, 199 Hz telemetry |
| GPS | log69 8-13 sats HDop 1.1-2.2; log70 11-16 sats HDop 0.8-1.3 |
| Serial ports | SERIAL2/3 confirmed on hardware; SERIAL1/4 untested |
| Battery voltage | Multiplier measured, 11.1 |
| Battery current | Broken - temperature-dependent and does not track load; see below |
| Motor outputs | 4x DShot600 via PIO; has also flown on PWM at 490 Hz |
| DShot | Bidirectional DShot600 at 2 kHz, flown; eRPM scale verified |
| DShot params | `SERVO_DSHOT_RATE` 1, `FSTRATE_DIV` 2, `SERVO_DSHOT_ESC` 0 - see below |
| eRPM error rate | Not measurable: `ESC.Err` is hardcoded 0 here, see below |
| Harmonic notch | Per-motor, -34 dB; tracked 72-353 Hz through acro |
| Compass | External on I2C1, `COMPASS_EXTERNAL` 1, `ORIENT` 101 |
| Position modes | Loiter flown, 119 s in log70; EKF fusing GPS |
| Acro | log69, 153 s at 800 deg/s, inverted, EKF `FS` 0 throughout |
| Rate loop in flight | 2 kHz held; dtMax never above 1.4 ms in any flight |
| Yaw trim | 17% diagonal RPM split; explained, not a fault - see below |
| DCM backup AHRS | Drifts 25-65 deg in flight; blocks re-arm - see below |
| Tune | Hand tune below; AUTOTUNE started, roll only, unsaved |
| Serial LED (J2) | Mode correct, LED not yet lit; see below |
| 9V rail (VID) | Stuck on; relay does not switch it, see below |

Retracted: this section used to record that the GPS was detected but had never
reported a satellite, `NSats` 0 and `HDop` 99.99 in every log. That is no
longer true and was never a port fault. log62 has 7-9 satellites, HDop
1.07-1.31 and `GPA.HAcc` 0.69 m, with `EKF3 IMU0 is using GPS` at 38.9 s and
the origin set at 17.3 s. Antenna or siting, as suspected.

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
  opposite of the sheet. Each directly drives an MP4334 EN pin with a 27k
  pull-down, so the schematic makes both active HIGH. An early software test
  appeared to leave the 9V rail on while commanding the GPIO low, but neither
  the GPIO nor the regulator EN pin was metered. Treat runtime control as
  unverified rather than inferring the opposite polarity. See the RP2350
  initial-level section below.
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
overclock. It remains above the RP2350 datasheet's 150 MHz clk_sys/clk_peri
limit and is validated only by tests on the bring-up sample, not across process,
voltage and temperature. The original overclock existed only to hide XIP flash
latency: both cores fetch through a shared 16 KB XIP cache, and core0's EKF plus
core1's rate loop were thrashing it. Moving core1's hot path into SRAM freed the
cache for core0 and let the clock come back down.

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
reads the cumulative time of core1's idle thread, which only exists with
`CH_DBG_STATISTICS`. There is no way to keep the core1 load figure without
paying for the statistics.

RT 7 has no `os_instance_t::idlethread`, so `Scheduler::core1_idle_cumulative()`
finds that thread once by walking the registry for the sole `IDLEPRIO` thread
owned by `ch1`, and caches it. If core1 load ever reads a flat zero, that lookup
found nothing - check the registry is enabled before suspecting the statistics.

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

Two more traps in the same area:

- `waf configure` runs `git submodule update`, so it will quietly move
  `modules/ChibiOS` back to the recorded gitlink if that is a fast-forward from
  where you left it. A measurement taken straight after a configure may not be
  measuring the tree you think. Pass `--no-submodule-update` when comparing
  submodule states, and check `git -C modules/ChibiOS log --oneline -1`
  afterwards.
- The build dir remembers which board it was configured for. Deleting
  `build/<board>` while another board is configured gives "Missing
  configuration file .../common.ld, reconfigure the project!" - reconfigure,
  the tree is fine.

### The ChibiOS branch stays on RT 7.0.6

`modules/ChibiOS` tracks the ArduPilot fork of stable_21.11.x, kernel RT 7.0.6,
and the RP2350 work sits on top of that. Upstream's RP2350 support was written
against RT 8.0.0, so the obvious way to bring it in - take upstream's tree - also
bumps the kernel under every ArduPilot board in the world. That was tried and
undone. Do not redo it without reading this.

Nothing in the RP HAL or the ARMv8-M-ML-ALT core port needs RT 8. Most of the
8.0.0 delta is renaming (`stkalign_t` to `stkline_t`, `THD_WORKING_AREA` to
`THD_STACK`, `F_LOCK` to `FACTORY_LOCK`), and the new
`os/common/ports/ARM-common/include/chtypes.h` is the old file with one typedef
renamed. What the port actually needs from RT is three things, all added to
files the port owns rather than to shared code:

- `PORT_WORKING_AREA` in the ALT port's `chcore.h`. RT 7 asks the port for it;
  RT 8 builds the working area itself.
- `PORT_CORE0_BSS_SECTION` / `PORT_CORE1_BSS_SECTION` in the ALT SMP header,
  aliased to the `PORT_MEM_LOCAL_COHERENT_BSSn` names. RT 8 spells these
  differently, and without the aliases `ch0`/`ch1` and both idle stacks fall
  silently out of the scratch banks into main SRAM. It still builds and still
  boots, so check the symbols, not the build:
  `arm-none-eabi-objdump -t build/Pico2/bin/arducopter | grep -E '\bch0$|\bch1$'`
  must show `.ram4_clear.core0` and `.ram5_clear.core1`.
- `mpu_v8m.h`, placed beside the existing `mpu_v7m.h` in
  `os/hal/ports/common/ARMCMx/` rather than in a parallel include tree.

Costs of the bump, measured: every STM32 board grew about 3.6 KB of flash and
moved about 3.3 KB from `.data` to `.bss`. With the kernel back at 7.0.6,
SPRacingH7 is byte-identical to the pre-rebase build and MatekH743, CubeOrange
and MatekF405 are symbol-identical.

The ArduPilot side pays for this in four places, all in the RP2350 paths:
`thread_descriptor_t` is filled in by hand rather than through
`__THD_DECL_DATA`, `chCoreGetStatusX()` keeps its RT 7 signature, the
`stkalign_t`/`stkline_t` shim in `stm32_util.h` is gone, and core1 load
reporting looks up the idle thread through the registry.

Two shared files are still touched, and both are deliberate:
`os/rt/src/chinstances.c` carries the RP2350 per-core idle-loop hook under
`#if defined(RP2350) && (CH_CFG_SMP_MODE == TRUE)` - its `#else` branch is
byte-for-byte the fork base, so STM32 codegen is untouched - and
`os/hal/include/hal_usb.h` gains one config default that the RP USB LLD tests.

If you revert a shared ChibiOS file, audit the fork commits that touched it
rather than trusting the build. Reverting `chinstances.c` compiled and linked
cleanly while leaving `rp2350_idle_loops.S` built but never called.

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

RP2350 code in shared files needs an `#if defined(RP2350)` that covers all of
it, and only an STM32 build will tell you it does not. Two cases had gone
unnoticed in `Tools/AP_Bootloader/bl_protocol.cpp` until an STM32 bootloader
was built: `__set_MSPLIM()`/`__set_PSPLIM()` sat one line below the closing
`#endif`, and the `WATCHDOG->SCRATCH[]` reset handshake guarded only its inner
`SCRATCH[3]` writes. Stack limit registers are ARMv8-M and `WATCHDOG` is an
RP2350 block, so no STM32 bootloader would build at all. Build one STM32 board
and one STM32 bootloader before pushing anything that touches shared code.

A related one on the RP side: `RCOutput_pico.cpp` guarded on `defined(RP2350)`
while its own header also required `HAL_DSHOT_ENABLED || HAL_SERIALLED_ENABLED`,
so the body compiled where the class was never declared. It also borrows its PIO
register bit-field constants from `PIOUART.h`, and those live behind
`HAL_HAVE_PIO_UARTS`, which no bootloader hwdef sets. Both guards now carry
`HAL_USE_PWM` as well. If a file borrows constants from another module's header,
check what that header is gated on.

Anything relocated to SRAM must appear in exactly ONE registry. The linker
claims `.text` sections first-come-first-served, so a symbol listed in two
registries is silently dropped from one. See `PROFILING.md`.

`memcpy` and `memset` are relocated into `.ramtext` because they are the top
flash-resident functions on the core1 rate/IMU path. The relocation has a
boot-order gotcha involving a volatile copy loop; see the memcpy/memset section
of `../Laurel/BASELINE.md` before touching it.

They are relocated by `common_rp2350_smp.ld` picking the newlib archive members
directly, *not* through a registry, and that is the only mechanism that can
work for them: newlib has no per-function sections, and its `libc.a` sits in
the toolchain rather than under the build root that the symbol map is built
from. Registry entries for them existed for a while and did nothing except
print `no symbol match` at link time, which reads exactly like the relocation
having failed - it had not, `nm` put both in SRAM throughout. Verify placement
with `arm-none-eabi-nm` on the ELF rather than trusting either the registry or
the warning. A `no symbol match` line for anything else is a real miss; the
`__stats_*` ones are expected on any build with statistics off.

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

## The hwdef OUTPUT HIGH/LOW initial level was ignored on RP2350 (fixed)

`board_rp2350.c` used to walk every `HAL_GPIO_PINS` entry and call only
`palSetLineMode(line, PAL_MODE_OUTPUT_PUSHPULL)`. Nothing applied the
`HIGH`/`LOW` qualifier from `hwdef.dat`, and the qualifier did not even
survive generation - `hwdef.h` rendered the entry as
`/* PA18 BEC_9V_EN OUTPUT */` with the level dropped. Contrast the chip-select
pins further down, which each do an explicit `palSetLine()` *before*
`palSetLineMode()` precisely because CS has to idle high; the generic GPIO loop
had no equivalent.

So `PA19 BEC_5V_EN OUTPUT HIGH` did not come up high. Both regulator enables
came up at the SIO `GPIO_OUT` reset value, which is 0, and `AP_Relay::init()`
then drove both low again because `RELAY2_DEFAULT` and `RELAY3_DEFAULT` are
both 0. Every software path was holding these pins low from boot onwards.

Fixed by emitting a `HAL_GPIO_INIT_LEVELS` table from `chibios_hwdef.py` for
RP MCUs and applying it in `board_rp2350.c` before the mode loop, matching the
chip-select ordering so the pad never briefly drives the wrong way. It is a
separate macro rather than a fifth field on `HAL_GPIO_PINS` because that macro
initialises `gpio_entry`, whose next member is the IRQ handler. Levels come
from the existing `get_ODR_value()`, so a pin with no explicit qualifier now
takes the STM32 default of HIGH - every non-PWM GPIO pin on all three RP2350
boards states its level explicitly today, so nothing changed underneath them,
but a new board that omits it will get HIGH rather than LOW.

The 9V rail was observed on in that state, and no relay command was observed to
change it. That observation does not establish active-low polarity: the
schematic connects GPIO18 directly to the MP4334 EN input with a pull-down, an
active-HIGH circuit. The GPIO and U6 EN pin were not metered during the test,
so the actual logic level is unknown.

Not yet resolved: changing `RELAY2` should change GPIO18 and U6 EN, and that has
not been seen. Note the initial-level fix does not settle this because
`AP_Relay::init()` drives the pin to `RELAY2_DEFAULT` immediately afterwards.
Either the GCS addressed a different instance, the stored relay parameters
overrode the expected state, the pad was not actually driving, or the rail was
being powered by another path.

The decisive test is to meter GPIO18 and U6 EN while changing `RELAY2`, then
meter the 9V output. Do not change `RELAY2_INVERTED` based only on the output
rail. Check `RELAY3`/5V at the same time, and note `MAV_CMD_DO_SET_RELAY` is
0-indexed, so RELAY2 is instance 1 - a GCS that numbers its relays from 1 will
be one out, and instance 0 is rejected outright because `RELAY1_FUNCTION` is
0.

The 5V rail is the one the fix visibly changes: `PA19 BEC_5V_EN OUTPUT HIGH`
now really is high for the window between board init and `AP_Relay::init()`,
where before it was low throughout.

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

## DCM drifts in flight and blocks the next arm

DCM is 15-65 degrees wrong by the end of every flight. It is fine on the
ground - `ErrRP` 0.0019 in log70 at t=37.7 s, pitch -3.24 against the EKF's
-3.40 - and starts diverging within a second of arming, before there is any
vibration to blame (`VIBE` was 0.007 m/s/s at t=40.5 s when `ErrRP` had already
reached 0.64).

| | worst in-flight error | prearm reports afterwards |
|--------|-----------------------------|-------------------------------------|
| log62 | not measured | 51 deg |
| log70 | pitch +24.5, roll -13.5 | 24 -> 16 -> 11 deg over 60 s |
| log69 | roll -64.8, pitch +24.6 | 58 -> 41 -> 29 -> 20 deg over 90 s |

The EKF is the correct one throughout: the vehicle held position within 1 m in
log70 with roll and pitch tracking demand to 0.3 deg, `XKF4.FS` was 0 in both
flights, and both IMU health flags stayed set.

**Copter never flies on DCM, so this is not a safety fallback issue.** The
fallback block in `AP_AHRS::_active_EKF_type()` is gated on
`_vehicle_class == FIXED_WING || GROUND`; Copter is neither, so none of the
GPS-loss or `const_pos_mode` paths apply. The only residual route is
`ekf3_estimates.filter_faults != 0`, which leaves `ret` at
`fallback_active_EKF_type()` = DCM - a hard-fault path, and `FS` has been 0 in
every flight so far.

What it does cost is the next arm. The DCM roll/pitch consistency check at
`AP_AHRS.cpp:1829` is gated on `!always_use_EKF() || (total_ekf_cores == 1)`.
Copter sets `FLAG_ALWAYS_USE_EKF` so the first clause is false, but this board
has one EKF core (`EK3_IMU_MASK` 1, "alloc 1 cores"), so the check is live. The
threshold is `ATTITUDE_CHECK_THRESH_ROLL_PITCH_RAD` = 10 deg, and log69 was
still at 20 deg when the log ended - a lockout of over 95 s. There is no way to
switch the check off short of `ARMING_CHECK`, because a single IMU means no
second lane. The yaw half is gated on `!always_use_EKF()` so it never fires,
which is why only "Roll/Pitch inconsistent" ever appears.

Mechanism, hypothesis not diagnosis. The timing points at `use_fast_gains()`,
which is just `!hal.util->get_soft_armed()`: pre-arm DCM's P gain is 8x, which
force-slaves it to the accelerometer and hides whatever the error is, and
arming removes that. The magnitude points at the GPS term - `_error_rp` pegs at
exactly 1.0 whenever `GA_b . GA_e < 0` (`AP_AHRS_DCM.cpp:981-983`), more than
90 degrees apart, which a 25 degree attitude error alone cannot produce. The
earth reference is
`GA_e = (0,0,-1) + (velocity - _last_velocity) * AHRS_GPS_GAIN / (_ra_deltat * g)`
at line 923-928, so a `_ra_deltat` that reads short would inflate the GPS
acceleration term and tip `GA_e` past 90 degrees. GPS `SAcc` is 0.16-0.80 m/s,
so there is real velocity noise for a wrong scale factor to amplify.

Cheapest discriminating test: one flight with `AHRS_GPS_GAIN` 0. That drops the
GPS term entirely and `GA_e` becomes pure (0,0,-1). If `ErrRP` returns to near
zero and DCM stops drifting it is the GPS/`_ra_deltat` path; if not, look at
the accel path. Note DCM degrading through 150 s of inverted 800 deg/s flight
is close to expected on its own - it is log70's 25 deg after ordinary +/-30 deg
Loiter that is harder to excuse.

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

The harmonic notch is now on and RPM-referenced - see the notch section above.
It was off for the whole PWM era, correctly: the motor fundamental sits around
180-190 Hz and shows in the accel, but the oscillations chased during tuning
were all sub-15 Hz control modes, which a notch cannot touch. That is still
true, and the notch has not moved the sub-30 Hz residual. What it does is
remove 34 dB of motor-band content from the gyro the rate loop sees, which is
worth having on its own terms. Bidirectional DShot supplies the eRPM, so the
throttle-based fallback (`INS_HNTCH_MODE` 1) is no longer needed.

### AUTOTUNE, first attempt

Started in log62 and did not finish. It spent all 193 s of the run on roll -
Rate D Up, Rate D Down, Rate P Up, Angle P Down - and was still in Angle P Up
when the flight ended. Pitch and yaw were never reached.

Nothing was saved. There is no `AUTOTUNE_SUCCESS` event and no `Saved gains`
message, and the only parameters written after takeoff are `STAT_*` and
`MOT_THST_HOVER`. The flying gains are still the hand-derived ones above.

What it did produce is a direction. `ATUN` on the roll axis converged toward
`RP` 0.075 and `RD` 0.00105, against the hand tune's 0.060 and 0.00080, with
`SP` left at 4.5. So autotune wants somewhat more rate P and slightly more rate
D than the hand tune - worth knowing, but do not hand-enter those: they are a
partial result from a run that never validated itself.

The run was slow because of pilot input, not because of the vehicle. There are
over fifty `AutoTune: pilot overrides active` messages across the 193 s, each
one suspending the test. Roll alone should not need three minutes. Budget a
longer flight, hold position hands-off between twitches, and expect to need
pitch and yaw after it.

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

## NeoPixel, and how the PIO blocks are divided up

The serial LED output on GPIO2 is driven by the NeoPixel half of
`RCOutput_pico.cpp` from **PIO1**, and the block choice is forced rather than
preferred:

| block | owner | SMs | instructions | GPIOBASE |
|-------|--------------------------------|-----|--------------|----------|
| PIO0 | PIOUART0 (SM0-1), PIOUART1 (SM2-3) | 4/4 | UART programs | 16 |
| PIO1 | NeoPixel (SM0) | 1/4 | 4/32 | 0 |
| PIO2 | DShot | 4/4 | 13 or 29 of 32 | 0 |

PIO1 was free because `SERIAL_ORDER` only instantiates PIOUART0 and PIOUART1,
and the driver table in `PIOUART.cpp` puts both of those on PIO0. The two PIO1
entries, PIOUART2 and PIOUART3, are never built on this board.

Sharing PIO2 with DShot was never an option, for two independent reasons. All
four state machines are in use, one per motor. And the bidirectional program is
29 of the 32 instruction slots, against the 4 the WS2812 program needs. Either
one alone rules it out.

The GPIOBASE column is the other half of it. A WS2812 pin below GPIO16 needs a
GPIO0-31 window, which the PIOUART blocks cannot offer - they are shifted to 16
so PIOUART0 can reach GPIO42/43. So even a free state machine on PIO0 would not
have been usable for this pin.

Correcting an earlier reading of the schematic: there is **no LED fitted on the
board**. The section headed "WS2812 LED" on page 2 contains only connector
**J2**, a 3-pin JST-SH compatible right-angle header with two shield pins to
ground. `RGB_LED` leaves GPIO2, passes through R82 (27 ohm) and arrives at J2
pin 3; pin 1 is +5V and pin 2 is ground, taken from C74's ground node. The
vendor GPIO sheet calls GPIO2 "the onboard RGB LED", which is what the first
version of this note and the README repeated, and it is wrong in the same way
the sheet is wrong about the ESC order and the regulator enables.

The practical consequence is that `NTF_LED_LEN` is however many LEDs are on the
strip somebody plugs in, not 1.

The program is the four-instruction ws2812 from pico-examples, by way of
Betaflight's `light_ws2811strip_pico.c`, with T1/T2/T3 of 3/3/4 giving ten PIO
cycles per bit. At the 800 kHz carrier that wants an 8 MHz PIO clock, so CLKDIV
is 225/8 = 28.125, which lands exactly on the 16.8 fixed point format as
28 + 32/256. Nothing here needs a fractional-divider apology the way the
bidirectional DShot decode does.

Two things differ deliberately from Betaflight:

- **No DMA.** Betaflight allocates a DMA channel per strip. This port feeds the
  TX FIFO directly, as the DShot driver does, for the reason recorded in the DMA
  note below - `dmaChannelAllocI()` with a specific id has no fallback, and that
  is what silently killed the GPS. Joining the RX half onto TX gives an eight
  word FIFO, so a chain of eight needs no refill at all and a longer one blocks
  the LED thread for about 30 us per LED beyond that.
- **Streamed, not buffered.** The frame is pushed a word at a time
  (`send_begin`/`send_word`/`send_end`) rather than packed into an array first.
  `AP_SERIALLED_MAX_LEDS` is 128, so a buffer would have put 512 bytes on the
  LED thread stack, and stack headroom on this board is already something
  `MAIN_STACK` has had to be raised for.

The pin is declared `PWM(5)` in the hwdef purely to get a channel index and a
rate group; slice 1 never drives it, exactly as slices 3 and 4 never drive the
motor pins under DShot. It lands in its own group, so NeoPixel mode on output 5
puts no rate constraint on PWM 1-4.

ProfiLED is refused at `set_group_mode` rather than silently treated as a
NeoPixel. It needs a second program, a 25-bit frame and a separate clock pin,
none of which exist here.

### Where the bring-up got to

Flown-on-the-bench state: the output mode is now correct and the LED still does
not light. Nothing has yet been seen on a scope or a meter.

One real bug found and fixed on the way. `mode_requires_dma()` is true for LED
protocols as well as DShot, and the RP2350 exemption in `set_output_mode()`
only cleared it for DShot - so a NeoPixel request still demanded a UP DMA
channel this chip never allocates, was rewritten to `MODE_PWM_NORMAL`, and
never reached the PIO path. The symptom was the startup banner reporting `PWM`
on output 5. That is DShot's item 1 above repeating itself, because the
exemption's comment asserted serial LED still needed a DMA and that stopped
being true the moment this driver landed.

Note the mode switch is **lazy**: `set_serial_led_num_LEDs()` only sets
`grp->led_mode`, and `current_mode` does not change until the first colour
write reaches `set_serial_led_rgb_data()`. So the startup banner can
legitimately read `PWM` even when configured correctly - it is emitted from
`AP_Vehicle.cpp` before any LED data exists. Judge the mode from a later banner
request, not the boot one.

Next time, in order:

1. **Meter the 5V on J2 pin 1.** This is the first thing to check and the most
   likely answer. That pin is fed from the switched peripheral rail whose
   enable polarity is still unresolved - see the OUTPUT HIGH/LOW section
   above. If the rail is off the strip has no power at all and no amount of
   correct data will light it. Nothing downstream is worth debugging until
   this reads 5 V.
2. Confirm the three parameters actually took: `SERVO5_FUNCTION` 120,
   `NTF_LED_TYPES` with bit 8 set, `NTF_LED_LEN` matching the strip.
   `SERVO5_FUNCTION` is in `defaults.parm`, which only applies on a parameter
   reset - an existing board keeps whatever it had stored.
3. Scope GPIO2. A WS2812 frame is unmistakable: 24 bits per LED at 800 kHz,
   1.25 us a bit. Silence means the state machine is not running or the pad is
   not routed; a waveform means the problem is downstream of this port.
4. If the pad is silent, suspect FUNCSEL before the program. PIO1 is FUNCSEL 7
   on RP2350, and the DShot bring-up lost a day to exactly this - FUNCSEL 11
   routed those pads to the aux UART, the state machines ran, and nothing
   reached the pin.
5. If the waveform is there but the colours are wrong, it is byte order rather
   than timing: `SERVO5_FUNCTION` 121 selects `MODE_NEOPIXELRGB` and the driver
   implements both orders.

The PC sampler is no help here - a state machine that never starts costs no CPU
and shows up as absence. The counters-over-SWD approach in `PROFILING.md` is
the right instrument if it comes to that.

## DShot parameters: the two that cost a day

Both of these presented identically - the ESCs repeating part of their arming
tone and never arming - and neither is obvious from the parameter name.

**`SERVO_DSHOT_ESC` must be 0 on this board.** It was set to 3 (BLHeli32 with
Extended DShot Telemetry). EDT enables itself by sending DShot command 13
repeatedly, and DShot commands 1-5 are the beep commands, so a command stream
the ESCs mishandle sounds exactly like a failed arm. It also showed up in the
numbers: with EDT on, the rcout thread issued about 170 more sends per second
than push() asked for, and the send/wake/signal counters would not reconcile.
With it off they balance exactly. Bidirectional eRPM does not need EDT - plain
bidir DShot600 returns RPM on its own.

**`SERVO_DSHOT_RATE` is a multiple of the rate loop, not of `SCHED_LOOP_RATE`.**
`ArduCopter/rate_thread.cpp` calls `set_dshot_rate(rate, attitude_rate)`, where
`attitude_rate` is `raw_gyro_rate / FSTRATE_DIV`. With a 4 kHz gyro:

| `SERVO_DSHOT_RATE` | `FSTRATE_DIV` 2 | `FSTRATE_DIV` 4 |
|--------------------|-----------------|-----------------|
| 0                  | 1 kHz fixed     | 1 kHz fixed     |
| 1                  | 2 kHz           | 1 kHz           |
| 2                  | 4 kHz           | 2 kHz           |

`SCHED_LOOP_RATE` is 200 here, so "1" looks like it should mean something slow
and does not. There is no setting between 0 and the rate-loop rate.

Beware that `SRV_Channels.cpp` also calls `set_dshot_rate()`, but with
`AP::scheduler().get_loop_rate_hz()` - 200, not `attitude_rate`. At 200 the
`while (drate < 800)` bump loop runs and yields `_dshot_rate` 4, where the rate
thread's call yields 1. The two disagree and whichever ran last wins, which also
decides whether the virtual timer gets armed. `SERVO_DSHOT_RATE` 0 is immune
because both callers then take the same early return. This looks like an
upstream bug on any board running the fast rate thread; it has not been raised.

At `_dshot_rate` 1 the virtual timer is deliberately not armed - the code
assumes push() provides the tick. Nothing guarantees that. A watchdog timer at
twice the period was tried and rejected: it bounded the gap at `FSTRATE_DIV` 4
but not at 2, because the stalls are not missed pushes at all (below).

## eRPM is correct, but `ESC.Err` cannot measure the error rate

The eRPM scale is verified against an independent sensor. Across log51 the
accel vibration peak tracks the eRPM-derived fundamental with correlation
+0.963 and a best-fit slope of 1.011, implying 14.2 poles against the
`SERVO_BLH_POLES` 14 that is set. Per-channel correlation between eRPM and
`RCOU` is +0.983 to +0.994, with only 0.2-0.6% repeated consecutive values, so
the decode is live rather than a stale register. That is enough to trust the
telemetry to drive a notch.

What is *not* available is the frame error rate. `ESC.Err` reads exactly
0.0000 on all four ESCs in every bdshot flight, and that is structural rather
than a perfect link. On RP2350 the decode runs through `RCOutput.cpp`, in the
`is_bidir_dshot_enabled()` branch that calls `RCOutput_pico::read_telemetry()`
- and neither the success nor the failure path touches the counters.
`_bdshot.erpm_clean_frames[]` and `_bdshot.erpm_errors[]` are only incremented
in `RCOutput_bdshot.cpp`, inside a test on `group.dshot_state` being
`RECV_COMPLETE` or `RECV_FAILED`. This port never enters either state, because
the DShot state fix returns the group straight to `IDLE`. So
`get_erpm_error_rate()` evaluates `0 / (1 + 0 + 0)` forever.

The plan recorded earlier - read `erpm_clean_frames[]` against `erpm_errors[]`
before letting telemetry drive a notch - therefore cannot be run as written.
Counting the `read_telemetry()` false returns in the RP2350 branch is a few
lines and would make it measurable. This matters more now than it did before
the notch was enabled, not less: if telemetry goes stale the notch falls back
toward the `INS_HNTCH_FREQ` floor rather than tracking, and nothing currently
reports that happening.

## The harmonic notch works, and needs to be per-motor

Flown in log52 as `INS_HNTCH_MODE` 3 (ESC RPM), `FREQ` 40, `BW` 10, `HMNCS` 1,
`OPTS` 22 - which decodes as TripleNotch + LoopRateUpdate + DynamicHarmonic
against the `Options` enum in `libraries/Filter/HarmonicNotchFilter.h`.

Within one flight the pre- and post-filter batch samples share the same
vibration input, so post/pre is a clean measure of what the filter chain
removes. Comparing that ratio between the notch-off and notch-on flights
isolates the notch from the 75 Hz `INS_GYRO_FILTER` low-pass:

| gyro rms, 150-240 Hz | roll | pitch |
|-------------------------------|-----------------|-----------------|
| notch off (log51), LPF only | 0.132 (-17.6 dB) | 0.137 (-17.2 dB) |
| notch on (log52), LPF + notch | 0.003 (-51.6 dB) | 0.003 (-52.0 dB) |
| notch contribution | -34.0 dB | -34.8 dB |

In absolute terms the post-filter motor band falls from 0.67 to 0.014 deg/s on
roll and 1.13 to 0.022 on pitch. The pre-filter motor band was comparable
between the two flights (5.08 to 5.37 roll, 8.24 to 8.91 pitch), so the
cross-flight step is not confounded by a change in vibration input.

Per-motor is not optional here. The four motors span 164-212 Hz, and `FTN`
shows `NDn` 4 with the centres landing within about 1.5% of each motor's own
eRPM fundamental (184.3/162.9/208.7/185.9 Hz against 186.8/164.3/211.7/188.6).
A single notch cannot cover a 48 Hz spread. Note the notch is constant-Q, so
`BW` 10 at a 40 Hz base is roughly 48 Hz wide at 190 Hz, tripled by the
TripleNotch option - the four overlap into a continuous stopband over about
150-240 Hz. That is a lot of filtering, but it is matched to the motor spread
rather than excessive, and it cost nothing measurable: `Dmod` stayed at 1.0
and D-output rms was 0.0034 roll / 0.0026 pitch against 0.0036 / 0.0043 with
the notch off.

Do not read the actual/demand ratio across these two flights. It moved 1.00 to
1.08, but demand amplitude fell about 40% at the same time, which is exactly
the trap recorded in the tune section. The absolute 11-12 Hz residual is
0.72 deg/s against 0.88 - essentially unchanged.

## The 17% diagonal RPM split is yaw trim, not a DShot fault

Mean ESC RPM splits cleanly by rotation pair and reproduces across flights:

| | M1 | M2 | M3 | M4 | {M3,M4} / {M1,M2} |
|-------|-------|-------|-------|-------|-------------------|
| log70 | 10753 | 9881 | 13041 | 11356 | 1.18 |
| log69 | 11950 | 10336 | 14148 | 11935 | 1.17 |

It looks alarming and it is not. Recording the reasoning because the obvious
suspicion - that the PIO is sending one diagonal hot - is wrong, and it would
be easy to re-open.

**At equal commands the motors are equal.** In log69 at t=2.61-2.79 s all four
outputs sit on the `MOT_SPIN_MIN` floor within 3 PWM of each other
(`RCOU` 1080/1082/1083/1081) and the ESCs report 4377/4342/4385/4428 RPM, a 2%
spread, with one sample at 0.6%. A per-channel scale error anywhere in mixer ->
PIO -> ESC -> motor would show at every RPM. It does not.

**The split is in the command, not just the telemetry.** `RCOU` is the mixer's
output, computed long before anything reaches `RCOutput_pico`, and it carries
the same split in the same direction (log70 1250/1227/1318/1266). That
direction is the tell: a PIO sending channel 3 hot would over-spin it, and the
controller would compensate by commanding it *down*. High command and high RPM
together is what a faithful output chain being asked for more looks like.

**The size matches the logged yaw output exactly.** In log69's acro window
`RATE.AOut` is 0.1400 and `RATE.YOut` is -0.0128, and the quad X yaw mixer
factors are +/-1. With `MOT_THST_EXPO` 0.49, hover thrust is
`0.51*0.140 + 0.49*0.140^2` = 0.0810, so the diagonals sit at 0.0938 and
0.0682 - a predicted thrust ratio of 1.375. Measured, summing RPM^2 per
diagonal: 342.6e6 / 249.6e6 = **1.3725**. Agreement to 0.2%. log70 predicts
1.52 against a measured 1.40, cruder because that window has more throttle
variation.

So the yaw controller is using 1.3% of its authority. It shows up as a 17% RPM
split only because this airframe hovers at about 8% thrust, which makes the
yaw term about 16% of hover thrust, and RPM goes as the square root. On an
ordinary 2:1 quad hovering at 50% thrust the same trim would be a ~1%
difference and invisible.

Physically that is under a degree of consistent in-plane motor mount twist
(order of magnitude - it scales with prop geometry). The other candidate is
prop pitch mismatch between the CW and CCW sets, which is easier to get wrong
than usual on a props-out build because the assignment is inverted from the
normal convention. If it ever needs settling, swap the two prop sets between
diagonals and re-fly: `PIDY.I` is -0.0115 in log70 and -0.0122 in log69, so a
sign flip would be unambiguous. It costs nothing and is not growing between
flights, so this is a note rather than a task.

## Flash writes stop motor output for up to 17 ms

Measured with a counter on the interval between `dshot_send_groups()` calls:
nine gaps over 5 ms, worst 17.6 ms, against a 500 us period. A virtual timer
cannot cover them, which is the tell - the whole of core1 is stopped, not just
waiting on an event. That is `rpEflBeforeXipOff()` parking core1 for a boot-flash
write; `rp2350_xip_park_max_us` had already been seen at 5298 us.

All the observed gaps were during boot and arming, and they persist unchanged
with the DShot parameters correct, so they are not what caused the arming-tone
problem. They matter anyway: 17 ms with no DShot to any motor is a real hole.
`AP_STORAGE_NO_WRITE_WHILE_ARMED` defers parameter writes while armed and
dataflash logging goes to the SD card over SPI rather than boot flash, so
nothing should write boot flash in the air.

Three bdshot flights now support that. `RTDT.dtMax` is a max-since-last-log at
10 Hz, so it bounds any stall the rate loop actually saw. Across the armed
window it never exceeds 1.05 ms in log51, 1.1 ms in log52 and 1.3 ms in log53
against a 500 us period. The only outliers in any of the three sit outside the
armed window entirely: 5.97 ms at t=43.757 in log51 against a disarm at 43.7,
which is the deferred storage flush landing exactly where predicted, and
5.45 ms at t=116.06 in log52, 28 s after disarm. log53 has none at all. That
bounds the stall rather than counting parks, so reading
`rp2350_xip_park_count` directly is still the cleaner confirmation, but no
in-flight park has shown up in the timing.

## AP_RCOUT_USE_32BIT_TIME

Set in `hwdef.dat`. No other ArduPilot board defines it, so this is its first
use anywhere and it is worth treating as unproven. It makes `rcout_timer_t`
32-bit and `rcout_micros()` resolve to `micros()`, which took the `micros64`
veneer off core1's dshot path - it had been 1.2% of samples.

`micros()` wraps at 71.6 minutes of uptime. `AP_HAL::timeout_remaining()` is
unsigned delta subtraction so it is wrap-safe, but two sites subtracted
`last_dmar_send_us` from `AP_HAL::micros64()` directly, which only agrees while
`rcout_timer_t` is 64-bit; past the wrap they compared a full clock against a
truncated stamp and always read "safe to send". Both now go through
`rcout_micros()`. Nothing caught it at compile time because they used raw
subtraction rather than the helper, whose static_asserts enforce matching types.

Until someone soaks the board past 71 minutes of uptime arming and disarming,
reboot before flying.

## The SD write path is CPU-starved, not card-limited

Retracted: this section used to conclude that "the sink is saturated, not
contended" and that the card had a ~100 KB/s ceiling. That is wrong. The card
delivers 91 KB/s when core0 is idle enough and 18 KB/s when it is not, in the
same flight, on the same file. The ceiling is core0 CPU, and the earlier
logs only ever sampled one load condition.

log69 settles it, because ACRO and LOITER run at very different core0 loads
inside one flight:

| Phase | `PM.Load` | `DSF.Bytes` per second | samples |
|--------------------------|-----------|------------------------|---------|
| ACRO, 7.5-152 s | 56-60% | 72538-91659 | 41 |
| LOITER, 174 s | 79% | 17922 | 1 |
| disarmed, 185-262 s | 68-70% | 46996-54795 | 10 |

log70 agrees on its own numbers - 20-28 KB/s armed at 78-81% load, 42-59 KB/s
disarmed at 66-71%, and 81-91 KB/s after the RC link dropped at t=323 s and
load fell to 62-64%. That last step is the cleanest single data point: nothing
changed but the RC processing going away, load fell 4.9 points, and throughput
went up 68%.

Delivered fraction follows: ACRO writes about 1480 msg/s against 4874 dropped
(23% through), LOITER 330 against 7896 (4%). Only one `DSF` record survived
the entire 28 s LOITER segment, which is itself the evidence.

### Why: round trips, not bandwidth

`PM.SPIC` counts SPI transactions, so this is measurable. Across log69's acro
window `SPIC` runs at 4807/s *for every bus combined*, while the card takes
79 KB/s = 154 sectors/s. Even if every transaction on the board were the SD
card that is 31 per sector, and the IMU on SPI0 at 4 kHz must account for most
of the 4807, which bounds the SD share at roughly 5-8 per sector. So
`mmc_wait_idle` is *not* spinning on a busy card, and bus utilisation is about
2.8%. The path is idle nearly all the time.

What costs is that each of those transactions is a full thread round trip.
`mmcSequentialWrite()` in `hal_mmc_spi.c` frames every 512-byte block in
software - `spiSend(2)` prologue, `spiSend(512)` data, `spiIgnore(2)` CRC,
`spiReceive(1)` response, then `mmc_wait_idle()` - so a 4 KB chunk is about 40
SPI transactions of which 8 carry data. Every one goes through
`SPIDevice::do_transfer()`: CS assert, `osalSysLock`, `bouncebuffer_setup`, two
RP DMA channels programmed, `osalThreadSuspendTimeoutS`, DMA ISR, thread
resume, `bouncebuffer_finish`, CS restore. A 1-byte poll is 0.36 us of wire
time wrapped in a thread suspend and resume.

`log_io` runs at priority 59 (see `STAK`) against `APM_MAIN_PRIORITY` 180, so
every one of those resumes waits for the main loop to yield. Dividing measured
throughput by 40 round trips per 4 KB gives an effective cost per round trip of
about 1.3 ms at 58% load and 5.7 ms at 79%. Wire time for the whole 4 KB is
only 1.46 ms, so essentially all of it is scheduling latency.

This is exactly what the ST SDMMC path avoids: there a multi-block write is one
DMA of N x 512 with the busy state signalled by the peripheral, so the io
thread is scheduled once per 4 KB instead of forty times. There is no
equivalent to port, because RP2350 has no SD host controller at all - see
below.

### What is already in place, and is not the problem

- The ArduPilot MMC-SPI work is present in the submodule: `hal_mmc: correct
  MMC driver, add support for SPI hooks and read/write timeouts`, `mmc_spi:
  added bus acquire hooks`, `fatfs_bindings: add support for op retries`.
  These are what `spiSendHook`/`spiReceiveHook`/`spiAcquireBusHook` in
  `sdcard.cpp` plug into.
- FatFs already streams whole chunks: `hwdef/common/ffconf.h` has
  `FF_FS_TINY 0`, so it does direct multi-sector transfers rather than
  windowing each sector, and `AP_FATFS_MIN_IO_SIZE` is 4096. A logger chunk
  therefore reaches `blkWrite(&MMCD1, sector, buf, 8)` and becomes one CMD25.
- SPI is not board-wide polled; only Durandal sets `HAL_SPI_USE_POLLED`.
- It is not the de-overclock. `SPIDevice.cpp` fixes `SSPCPSR` at 2 and varies
  `SCR` only, giving 22.50 MHz at 225 MHz against 23.44 MHz at the old
  375 MHz - 4%. Raw bus is 2.81 MB/s, so there is 20-100x of headroom above
  what is being achieved.
- Moving the SD off the rate core was tried in log53 (`HAL_CORE_SPI1` 1 to 0)
  and changed nothing. Note that test ran in Stabilize, where core0 was never
  loaded, so it did not test the mechanism above. It is worth re-running now
  that the mechanism is understood.

### Plan: collapse the round trips

Agreed approach, in order.

**(a) One full-duplex exchange per block.** Prebuild
`[0xFF][0xFC][512 data][2 CRC][1 resp slot]` - 517 bytes - in a staging buffer
and issue a single `spiExchange`, then read the response token out of the RX
side. `spi_lld_exchange()` already does that in one DMA pair. Takes 40 round
trips per 4 KB down to 8. Costs one 512-byte SRAM to SRAM memcpy per block,
about 1 us against 180 us of wire time, so the copy does not matter. Order of
magnitude, this should take loaded throughput from 18 KB/s to around 90 KB/s -
above the current *unloaded* best.

**(b) ISR-driven multi-block state machine.** The real ST equivalent, if (a)
is not enough. `mmc_write()` posts the whole 8-block job and the SPI DMA
completion callback advances to the next block without involving the thread,
so `log_io` is scheduled once per 4 KB. The obstacle is that `do_transfer()`
unconditionally does `osalThreadSuspendTimeoutS`; it needs a completion-callback
path alongside. For the inter-block busy wait, note the card holds DO low
continuously while busy with CS asserted, so the ISR can read the MISO pad
(PA28) directly instead of clocking bytes.

**(c) PIO.** A state machine that frames blocks and handles busy autonomously,
so the whole chunk goes out on one kick. Only if (a) and (b) fall short.

Two things ruled out while planning this. **DMA chaining** would be the elegant
way to do (a) with no memcpy, but the ChibiOS RP DMA driver does not expose it
and `rp_dma.h` explicitly forces `CHAIN_TO` to self citing errata RP2350-E5 -
read that erratum before building on it. And **4-bit SDIO over PIO**, which is
the usual RP2xxx route to multi-MB/s, is not available on this board: the hwdef
routes only `PA30 SPI1_SCK`, `PA31 SPI1_MOSI`, `PA28 SPI1_MISO`,
`PA29 SDCARD_CS`, so DAT1/DAT2 are not wired. Worth raising for the next spin -
DAT1/DAT2 on GPIOs contiguous with DAT0 so one PIO instruction can shift four
bits, plus pull-ups on DAT0-3 and CMD.

### The write buffer is still short

`LOG_FILE_BUFSIZE` is 80 but `DSF.FMx` never exceeds about 5.1 KB in any
flight, so the allocation is around 5 KB. `AP_Logger_File::Init()` steps the
request down 10% at a time until `ByteBuffer::set_size()` succeeds, and that
needs one contiguous `calloc`. `PM.Mem` reports about 69 KB free in flight, so
the memory exists later and the failure is at init - ordering or fragmentation.
Init prints `AP_Logger: reduced buffer N/M` and `AP_Logger_File: buffer size=N`
through `DEV_PRINTF`, which reaches the USB console but never the log, so one
boot with the console attached gives the real number.

This matters more than the earlier note credited. `io_timer()` writes at most
`_writebuf_chunk` (4096) but takes `nbytes = MIN(nbytes, size)` where `size` is
the contiguous run from `_writebuf.readptr()`. On a ~5 KB ring that run is
frequently well under 4096, so writes fragment into sub-chunk pieces - more
round trips per byte, which multiplies straight into (a).

### RP2350 has no SD host controller

Worth stating plainly so nobody goes looking. RP2350 in any variant has no
SDIO/SDMMC block; the peripheral set is 2x UART, 2x SPI (PL022), 2x I2C, PWM,
USB 1.1, ADC, 3x PIO, HSTX. RP2350B differs from RP2350A only in package and
pin count. ChibiOS reflects it: there is no `SDCv1`/`SDMMCv1` under
`ports/RP/LLD/`, and this board builds with `HAL_USE_SDC FALSE` and
`HAL_USE_MMC_SPI TRUE`. The only two routes to a card are SPI mode via the
PL022, which is what is in use, and 4-bit SDIO bit-banged in PIO, which this
board is not wired for.

One card-choice caveat while on SPI: SPI mode is mandatory for SDSC and SDHC
but optional for SDXC, and some large cards implement it poorly. Benchmark on a
32 GB SDHC card rather than a 128 GB+ SDXC one.

## Logging setup for tuning work

Note the bitmask below asks for about 330 KB/s and the card delivers 18-91
depending on core0 load - see the section above. In a position-controlled
flight expect roughly 95% of it to be dropped and `RATE` to land near 48 Hz
rather than 1 kHz; in acro, closer to 23% through. It is still the right
setting for tuning work, but the sample rate is not what it claims and anything
spectral should come from the `ISBH`/`ISBD` batch samples.

Those mostly survive: log69 has 648 `ISBD` across 21 blocks against 672
expected, so 96% arrive and a few blocks are incomplete. `ISBH.smp_cnt` is
1024, so a block is 32 `ISBD` records with `seqno` 0-31 - check for a full set
before trusting an FFT of any one block.

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
around 19 MB for a 40 s flight, but the tested card does not keep up; expect
ordinary messages to be dropped as described above.

## Battery failsafe

`BATT_LOW_VOLT` shipped at 21.6 V, which on 6S is 3.60 V/cell and fires a Land
in mid-discharge on a healthy pack. It did exactly that on flight 17. For 6S
use 21.0 (3.5 V/cell) low and 19.8 (3.3 V/cell) critical. Sag compensation
(`BATT_FS_VOLTSRC` 1) is the better answer but is useless until the current
scale above is fixed.

This is now the most likely thing to cut a flight short, and it has been
ignored twice. Still 21.6 low and 21.0 critical as of log70. Minimum pack
voltage by flight: log62 21.79 V (0.19 V of margin), log69 **21.81 V**
(0.21 V), log70 23.07 V. log69 also tripped `BATT_ARM_VOLT` 22.1 on the
post-flight prearm, so the pack really was getting low - but at 3.64 V/cell it
was not flat, the threshold is simply set for the wrong chemistry state.
Reported consumption is invalid in all three because the current input does not
measure load. Change it before flying again, not after.

Sag compensation stays off regardless, because the current channel does not
work at all - see the next section.

## The current sense is not measuring current

`BAT.Curr` reads about 7 A whatever the vehicle is doing, including with the
motors stopped. This is not a scale or an offset problem and no parameter
value repairs it. Stop treating it as a calibration task.

### What log62 shows

Correlation of `BAT.Curr` against every plausible driver, over the armed
window:

| driver | r |
|------------------|----------------|
| `MCU.MTemp` | +0.575 |
| `BARO.Temp` | +0.390 |
| `BAT.Volt` | +0.322 |
| `RCOU.C1`-`C4` | -0.27 to -0.36 |
| `ESC.RPMmean` | -0.085 |
| `CTUN.ThO` | -0.085 |

No correlation with load at all, and the sign against motor output is
negative - backwards for a current sensor.

Correlation on its own would prove little here, since die temperature, pack
voltage and elapsed time all drift monotonically through a flight and will
correlate with each other. What settles it is two windows where the throttle
is pinned and the reading moves anyway:

- **Motors off, 40.7 to 50.7 s.** `CTUN.ThO` is 0.000 throughout, and the
  reading ramps 6.97 to 7.47 A while `MCU.MTemp` ramps 32.76 to 33.69 degC.
- **Constant hover, 51.7 to 68.7 s.** `ThO` is flat at about 0.12, and the
  reading *decays* 8.35 to 7.42 A while `MTemp` falls 33.4 to 32.3 degC - the
  board cooling in prop wash.

A real sensor at flat throttle on a sagging pack drifts slightly up. It never
falls 12% in 17 s. The channel tracks die temperature.

Two further facts. It reads 5.4 to 7.4 A with the motors physically stopped.
And ArduPilot's internal resistance estimator, which is fed from this channel,
produces 0.002 to 0.137 ohm - a 68x spread, which is what regressing a real
voltage sag against a fake current gives you.

An offset cannot rescue it either. Setting `BATT_AMP_OFFSET` to the motors-off
level (about 0.11 V) leaves hover at 1.84 A, roughly 41 W on a 6S quad turning
11500 RPM. So the gain is wrong too, or there is no signal to scale.

### The schematic is correct - do not chase the front end

Retracted: an earlier reading of this, taken from the prose in this file rather
than from the schematic, held that the 82.5k pulldown put too much source
impedance in front of the ADC, and that inter-channel charge sharing might be
bleeding the voltage channel into this one. Both are wrong. The page 3 circuit
is:

```
ESC connector (CUR) -- BAT_CURRENT --[ 120R 1% ]-- CURRENT_SENSE -- GPIO47/ADC7
                                                        |
                                          C34 100nF ----+---- R51 82.5k 1%
                                                        |
                                                       GND
```

That is a textbook ADC front end:

- 120R with 100nF is a 13.3 kHz low-pass, correct anti-aliasing for this
  signal.
- C34 is a charge reservoir about 10000x the RP2350 sample-and-hold
  capacitance, so the converter settles trivially and cross-channel charge
  sharing cannot survive it.
- With the ESC driving, source impedance at the pin is 120 || 82.5k, about
  120 ohm. Ideal.
- 82.5k/(82.5k+120) is 0.9985, hence the 1:1 in the README.

The 82.5k is a pulldown in *parallel* with the ESC output, not in series with
it. It only becomes the source impedance when nothing is driving the line,
which is the fault being diagnosed rather than a defect in the design. There
is nothing to raise with the board designers on impedance grounds.

Also stale, and previously recorded here as the leading suspicion: that the pad
was never configured for analog use. `adcRPGpioInit()` in the ChibiOS RP ADC
LLD sets FUNCSEL 31 and clears PUE, PDE and IE, and `rp2350_board_init()` calls
it for every pin in `HAL_RP_ADC_GPIOS`, which the generated header gives as
40, 46, 47. The pad is configured. The comment in `board_rp2350.c` records that
the list *used* to be hardcoded to Laurel's GPIO40/41/42, which is where that
suspicion came from; it has been fixed.

### What is left

With 82.5k *and* 100nF to ground, an ideal undriven node would sit at 0 V, and
the capacitor means this is not a fast sampling artifact. The RP2350 datasheet
allows up to 1 uA pin input leakage, which can develop 82.5 mV across the
pulldown and account for about 4.1 A at the nominal 50 A/V scale. The observed
0.1456 V requires **1.76 uA** in total, so specified pad leakage can explain a
substantial part but not necessarily all of it. The remaining candidates are:

1. **The ESC is driving it**, and its sense output idles near 0.145 V with
   essentially no gain - shunt not connected, sense amp unpopulated, or an ESC
   variant with no current sense at all.
2. **Nothing is driving it and the ADC pad or board is leaking** into the
   82.5k. The required current is above the datasheet's pin-leakage maximum on
   its own, so board contamination, a damaged pad or another current path
   would also be needed if the full 0.1456 V remains with the ESC disconnected.

### The CUR/TEL wiring hypothesis

The ESC connector runs `CUR`, `TEL`, `DS1`, `DS2`, `DS3`, `DS4`, with `CUR` and
`TEL` on adjacent pins at the same pitch. A one-pin error is easy to make, and
an ESC harness whose own pin order differs from this board's would do it
without any mistake at the soldering iron.

It cannot be a shifted connector: all four DShot lines work and bidirectional
eRPM decodes at +0.983 to +0.994 per-channel correlation against `RCOU`. Only a
discrete swap of those two wires is possible.

Whether it explains the reading depends on how the ESC drives its telemetry
pin, and the data already rules out half the cases:

- **Push-pull, idling high** (typical BLHeli_32 / AM32 UART TX): the ADC would
  sit near 3.3 V, which at `BATT_AMP_PERVLT` 50 reads about 165 A. The observed
  7.3 A rules this out.
- **High-Z except during a burst**: the 82.5k pulls the node to 0 V and what is
  left is pad leakage. Consistent with everything above - and note this is a
  *mechanism* for case 1/2, not a competing explanation. "Nothing is driving
  the node" and "the thing wired to it drives only occasionally" are the same
  electrical situation.

If it is the swap, the real current output is landing on the FC `TEL` pin,
which routes to GPIO5 and can only reach UART1 RX - a port the GPS owns. That
would mean a static mid-level DC on a digital input pad, and it would mean
current sense is recoverable by swapping two wires with no board change.

### Tests, cheapest first

1. **Read `RSSI_ADC` on GPIO40.** That is the AN1 spare pad. If it also sits
   near 0.1 V and tracks die temperature, the leakage floor is a property of
   the chip or board and case 2 is confirmed without touching the ESC.
2. **Make the ESC transmit telemetry**, on the bench with props off. This is
   the sharp one, because it looks for a positive result rather than an
   absence: if that pin is really `TEL`, the current reading goes noisy or
   jumps the moment telemetry starts. A DC analog level cannot do that. Note
   the `SERVO_DSHOT_ESC` warning in the DShot section before setting it.
3. **Scope the pad.** A 115200 baud burst against a flat analog level is
   unmistakable, and answers it in seconds if a scope is to hand.
4. **Unplug the ESC current lead and meter the pad.** With this circuit an
   undriven node must read about 0 V. If it still sits at 0.145 V, the leakage
   is in the MCU pad.
5. **Confirm what the ESC actually outputs on that pin**, and its mV/A.
   `BATT_AMP_PERVLT` 50 assumes 20 mV/A. If the output can exceed 3.3 V, see
   the protection note below.

### One thing worth asking the designers

Not a defect, but a question for the next revision. The 120R series has no
clamp diode, so the circuit assumes the ESC never drives that pin above 3.3 V.
If an ESC drives it from a 5 V rail, the pad's ESD diode has to absorb
(5 - 3.6)/120, about 12 mA, which is above what those clamps are typically
rated to carry continuously. That matters twice over: it is a robustness
question on the next spin, and a pad stressed that way is one way to arrive at
case 2's leaky input. Establish what the ESC outputs before raising it.

## Before the next flight

The DShot path, the SRAM relocations, the timer commits and the harmonic notch
have all now flown (logs 51-53); position control has flown (log70: Loiter,
119 s armed); and full-rate acro has flown (log69: 153 s at 800 deg/s,
inverted, 2.75 g).

1. Build a flight image. `AP_RP2350_PC_SAMPLER_ENABLED` and
   `AP_RP2350_DEBUG_REPORT_ENABLED` must both be 0 in `hwdef.dat`; they are
   committed as 0, so only an uncommitted edit can turn them on. Check
   `git diff` on `hwdef.dat` is empty before flying.
2. Set `BATT_LOW_VOLT` off 21.6 V. It fired a Land mid-discharge on flight 17
   and is *still* 21.6 with `BATT_CRT_VOLT` 21.0 as of log70. Use 21.0 low and
   19.8 critical for 6S. log62 came within 0.19 V, log69 within 0.21 V. Do this
   one first; it has now been deferred twice.
3. Confirm `rp2350_xip_park_count` stops advancing once armed, per the flash
   section above. The timing evidence says no in-flight park, but the counter
   has still not been read directly. log69 has one unexplained 54.9 ms `PM.MaxT`
   in the window containing the arm transition, most likely the new log file
   being created plus the 1388-record parameter dump, since `LOG_DISARMED` 2
   starts a fresh log at arm. The `RTDT` record covering it was dropped, so
   whether core1 was parked is unknown - this is the natural test case.
4. Reboot shortly before arming, until the 71 minute wrap has been soaked.
5. Position modes are available now, but nothing beyond Loiter has been flown.
   RTL, Auto and the GPS failsafe paths are all still untested on this board -
   fly them deliberately before relying on one to recover the vehicle.
6. Expect a re-arm delay of 1-2 minutes after an aggressive flight while the
   DCM consistency check decays below 10 deg. Not a fault; see the DCM section.

## Next steps

1. **Collapse the SD write into one exchange per block**, option (a) in the SD
   section. Prebuild the framed 517-byte block and issue a single
   `spiExchange`, checking the response token from the RX buffer. This is the
   agreed next piece of work. Measure it the same way it was diagnosed: `PM.Load`
   against `DSF.Bytes` across an acro segment and a Loiter segment in one
   flight, which is what made the mechanism visible in the first place.
2. Fix the battery failsafe thresholds, per above. Deferred twice now.
3. Find out why the current channel reads die temperature instead of current,
   per the section above. Work the test list there in order; it is a wiring or
   sensor question, not a calibration one. Only once a reading responds to load
   is there any point setting `BATT_AMP_OFFSET` or checking the scale against
   what the charger puts back.
4. Read the `AP_Logger_File: buffer size=` line off the USB console and fix the
   5 KB write buffer. Independent of item 1 and it multiplies into it, because a
   short ring fragments every write below the 4 KB chunk.
5. Make the bidirectional telemetry error rate measurable, by counting the
   `read_telemetry()` false returns in the RP2350 branch. `ESC.Err` is a
   hardcoded 0 on this port, so the notch is now being driven by telemetry
   whose frame loss nobody can see.
6. Establish whether flash page programs now succeed first time or only on the
   retry after a failed verify. Harmless either way for correctness, but a
   retry every time means the underlying QSPI defect is still there and double
   the flash wear. One counter on the `memcmp` mismatch answers it.
7. Finish AUTOTUNE. Roll got most of the way in log62 without saving; pitch and
   yaw are untouched. See the autotune section - the run needs a longer flight
   and less stick than it got.
8. Try `AHRS_GPS_GAIN` 0 for one flight to discriminate the DCM drift, per the
   DCM section. Low priority: nothing flies on DCM, the only cost is the re-arm
   delay.
9. Attack core0's flash share, starting with the veneers - see the veneer
   section in `PROFILING.md`. Core1 is done and needs nothing further. Note
   core0 load is now known to gate SD throughput, so this is worth more than it
   looked.
10. Bring up SERIAL1 and SERIAL4. SERIAL2 and SERIAL3 are confirmed.
11. Re-check the QMI flash timing if this revision fits a different flash part.
    `RP_QMI_CLKDIV 3` / `RP_QMI_RXDELAY 2` were characterised on the v1 part.
12. Re-measure glat before acting on it, per the retraction above.
13. For the next board spin: route microSD DAT1/DAT2, ideally on GPIOs
    contiguous with DAT0, plus pull-ups on DAT0-3 and CMD. That is the only
    route to SDIO-class throughput on RP2350, which has no SD host controller.
