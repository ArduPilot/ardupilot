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

log96 is the newest and the first on a different airframe: 274 s on an iFlight
Nazgul Evoque F5, 49 s Loiter then 177 s ACRO then 48 s Loiter. It is also the
first flight on an actual ICM-56686 rather than the ICM42688P samples every
earlier log used. Nothing in the port needed attention; see the log96 section.

log97 is the follow-up with `MASK_LOG_ATTITUDE_FAST` on, and it is the first
flight to put the microSD write path under a real offered rate. It held 155 KB/s
with zero drops through 101 s of acro and then lost about 30,000 messages in
30 s of Loiter - see the log97 section, which finds the mechanism is buffer
headroom rather than the card.

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
| IMU | Working; both fitted parts flown, ICM42688P and ICM-56686 |
| Barometer | DPS368 detected on I2C0 at 0x76 |
| microSD logging | 155 KB/s clean in acro; Loiter overruns the buffer, log97 |
| `spi_fail` prearm | Fixed twice: SD init left SPI1 stopped; late transfers also raised it |
| SPI timeout recovery | Fixed: RP `spi_lld_abort()` was an empty stub, upstream too |
| Corrupt log filenames | Open. One byte directory shift; not overrun, not XIP, not the abort |
| SPI teardown | Was cycling the peripheral per transaction; 3.8% recovered |
| Parameter storage | Working; needs both the sector-bound and write-verify fixes |
| RC input | CRSF/ELRS on SERIAL3, 333 Hz link, 199 Hz telemetry |
| GPS | log69 8-13 sats HDop 1.1-2.2; log70 11-16 sats HDop 0.8-1.3 |
| Serial ports | SERIAL2/3 confirmed on hardware; SERIAL1/4 untested |
| Battery voltage | Multiplier measured, 11.1 |
| Battery current | Gain unstable; zero is board-side thermal leakage, see below |
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
| DCM backup AHRS | 89 deg roll after log96, and starts before motors spin |
| Tune | Hand tune below; AUTOTUNE started, roll only, unsaved |
| Serial LED (J2) | Working; colours correct since the PULL_THRESH fix |
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

### ICM-56686: driver written, datasheet checked

Supported since `AP_InertialSensor: add ICM-56686 support`. DS-000563 rev 1.0
confirmed almost all of the Betaflight-derived notes below, with one exception
recorded in place. Two things the notes did not have:

- the difference from the ICM-45686 is a uniform **+4 shift** of the register
  block from `PWR_MGMT0` upwards. `WHO_AM_I` at 0x72 and the IREG window at
  0x7C are common to both, so the driver maps addresses at runtime through
  `reg456()` rather than duplicating the map.
- device ID is **0x08**, against 0xE9 for the ICM-45686, at the same address.

The FIFO is structurally identical - same header bits, same 20 byte high
resolution packet - so ArduPilot's existing parser and accumulator are reused
unchanged. `SREG_CTRL` is the one register the 456xy path does not already
write: it resets to 0x0A, meaning 20 bit sensor registers and big endian, and
the endianness applies to FIFO data too, so it must be cleared before the first
sample is read.

The notes below are kept for the register-level detail.

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

- Retracted: this said ArduPilot's ICM-456xy setup writes `0x0 << 4` intending
  2000 dps and would give a silent 2x gyro scale error. It does not. The
  comment at that write reads "4000dps range" and the driver sets
  `GYRO_SCALE_4000DPS`, so both parts agree that `FS_SEL` 0 is 4000 dps.
  DS-000563 rev 1.0 confirms the encoding. There is no trap here.
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

## The PIO UARTs, measured against Betaflight's

Betaflight runs the same hardware from `src/platform/PICO/uart`, and has had
longer to harden it. Reading the two side by side found four things worth
taking, and a few where this driver was already ahead.

### What was wrong here

**No framing check, and a resync that did not resynchronise.** The standard
receive program never looked at the stop bit; the SBUS one did, but recovered
with a jump straight back to `wait 0 pin, 0` - which a line that is *already*
low satisfies immediately. So a break, an unplugged transmitter, or a
receiver powered after the flight controller produced a continuous stream of
0x00 at full baud rate.

That is not hypothetical here. SERIAL3 is the RC input, SBUS idles low, and
`INOVER` inverts it - so with no receiver attached the pad pull-up makes the
state machine see a permanently low line. Roughly **8300 zero bytes a second**
into the RC parser, with the interrupt cost to match, whenever RC was
unplugged. Both programs now validate the stop bit and wait for the line to
return to idle, which is what pico-examples `uart_rx.pio` does and Betaflight
uses unmodified.

**Both FIFOs were half wasted.** Each state machine uses one direction, so
`FJOIN_TX` and `FJOIN_RX` give eight entries instead of four for nothing.

**`_write()` spun on the FIFO** for up to 20 ms a byte, on a core where that
time is not spare - because `_drain_tx_fifo()` existed and *nothing called
it*, so a write with no follow-up would otherwise have sat in the ring for
ever. The ST path settles the design: `UARTDriver::_write()` takes its mutex,
writes what fits, returns a possibly short count and never blocks. Here the
transmit interrupt does the draining, as in Betaflight - `_write()` primes the
FIFO once so a lone write still leaves immediately, arms TXNFULL, and the
interrupt disarms it when the ring empties. Leaving it armed fires
continuously, since TXNFULL is true whenever the FIFO has room.

**`tx_pending()` looked only at the FIFO**, which was nearly right until
`_write()` started queueing and then reported nothing pending with a full
ring. It now covers the ring, the FIFO and the state machine, which is only
finished once it is back at the blocking pull.

### What this driver already did better

- RX pin gets a pull-up **and** a Schmitt trigger; Betaflight only pulls up.
- The receive FIFO is read as an 8-bit access at `RXF+3`, the datasheet's
  method, rather than a 32-bit read and a shift.
- There is a dedicated 8E2 SBUS program with a parity skip; Betaflight has
  stock 8N1 only.
- The transmit program gets its stop bit from one instruction,
  `pull block side 1 [7]`, where Betaflight needs a `nop` as well.
- Driving the registers directly rather than through pico-sdk is what fits
  three programs into 24 of the 32 instruction slots.

### What it costs, measured

With RC connected and running, on core0:

| | |
|---|---|
| Bytes received | 41,890 in 20 s |
| Framing errors | 0 |
| Overruns | 0 |
| Interrupts | 2117/s, one per byte |
| Time in the handler | 3.18 us mean, **0.675% of core0** |
| Worst case | 92 us |

`RXNEMPTY` has no watermark, so one interrupt per byte is inherent and
Betaflight behaves the same. The deeper FIFO does not reduce the count - it
absorbs the tail. The worst case is around five byte-times at 420 kbaud,
which the old four deep FIFO would have been on the edge of losing.

With the receiver unplugged the same counters showed 13 framing errors and no
bytes, which is the discrimination worth having: the program flags a broken
line and stays quiet on a good one.

### Still not done

`OPTION_TXINV` is implemented but nothing on this board selects it, so the
inverted transmit path is untested.

The diagnostics are behind `AP_PIOUART_DEBUG_ENABLED`, off by default.

## Half duplex on a PIO UART

SmartAudio was the first thing to ask a PIO UART for a single-wire link, and
the driver had no half-duplex support at all - no `OPTION_HDPLEX` handling, and
`set_options()` inherited from the base class, which accepts nothing and
returns false. `AP_SmartAudio::init()` ignores that return, so the port
silently stayed full duplex: the request went out on the TX pin and the
receiver went on listening to an RX pin the VTX is not connected to.

Betaflight now has this, in `src/platform/PICO/uart/uart_tx_program.c`, and it
is worth reading before touching ours - the driver here follows it. The first
version of this work did not have it to read and drove the turnaround from the
CPU, which was worse in every way; what follows is the second version.

### The program owns the line

    .mov_status txfifo < 1
    .wrap_target
 0: set    pindirs, 0             ; release the line (half duplex only)
 1: pull   block                  ; idle here
 2: set    pindirs, 1 side 0  [6] ; take the line AND assert the start bit
 3: set    x, 7                   ; start bit continues (7 + 1 = 8 cycles)
 4: out    pins, 1                ; data bit, LSB first
 5: jmp    x--, 4             [6] ; 8 cycles per bit
 6: jmp    !y, 8       side 1 [5] ; stop bit 1; y == 0 means one stop bit
 7: nop                side 1 [7] ; stop bit 2
 8: mov    x, status              ; X = ~0 iff the TX FIFO is empty
 9: jmp    !x, 1                  ; more queued: hold the line, skip the release
    .wrap

Three things make this better than driving it from the CPU:

- **The state machine can see that its FIFO is empty.** That was the whole
  reason for going to the CPU in the first place - a program only discovers an
  empty FIFO by stalling on the pull, and a stalled program cannot act. `mov x,
  status` with `EXECCTRL.STATUS_SEL`/`STATUS_N` set to "TX level < 1" makes it
  visible without stalling. Everything else follows from that.
- **The release is exact.** One instruction after the last stop bit, and only
  when there is nothing left to send. No timer process, no polling from
  `available()`, and no bit-time fudge factor.
- **One program serves both modes.** Full duplex runs it with `SET_COUNT` of 0,
  which turns both `set pindirs` into no-ops - the side-set on those
  instructions still applies, only the pindirs write is dropped - so the pin
  stays driven throughout and there is no second program to keep in step.

Y holds the number of *extra* stop bits, loaded through `SMx_INSTR` while the
machine is stopped. Nothing in the program writes Y (`nop` is `mov y, y`,
chosen for exactly that) and `SM_RESTART` does not clear X/Y, so it survives
until the next `_begin()`.

The transmit program grew from 5 words to 10, which pushed the two receive
programs from offsets 5 and 14 to 10 and 19. Their jump targets are absolute,
so they were relocated with a script rather than by hand: shift the low 5 bits
of every opcode whose top three bits are 000, leave everything else alone.
Running that backwards over the old words reproduces the pico-examples original
byte for byte, which is the check worth doing before trusting the output.

### Two things the protocol needs that we were not doing

Both came out of reading Betaflight's version, and both are in ArduPilot's own
SmartAudio driver too - we were simply ignoring what it asked for.

- **Two stop bits.** `AP_SmartAudio::init()` calls `set_stop_bits(2)` unless
  `VTX_SA_ONE_STOP_BIT` is set, and Betaflight opens the port with
  `SERIAL_STOPBITS_2`. `set_stop_bits()` was a no-op here and the program sent
  one. That is now Y.
- **A pull-down, not a pull-up.** `AP_SmartAudio::init()` asks for
  `OPTION_PULLDOWN_TX | OPTION_PULLDOWN_RX`; Betaflight has a dedicated
  `SERIAL_PULL_SMARTAUDIO`, commented "the SA protocol usually requires
  pulldowns"; and `UARTDriver::set_pushpull()` applies exactly that on the ST
  path. The first version here overrode all of that with a pull-up and
  documented the override as deliberate, on the reasoning that a released line
  should idle high. Three independent implementations disagreeing with you is
  not a tie.

### The echo, which is where the two projects differ

On a shared pin the receiver hears everything the transmitter says. Betaflight
passes that echo up ("Everything transmitted is echoed to the rx, and the echo
is deliberately passed up") and lets the protocol layer deal with it. ArduPilot
does not: `UARTDriver::_rx_timer_tick()` drops received bytes while
`hd_tx_active`, and `AP_SmartAudio::read_response()` depends on that - it
matches the first byte against `0xAA` and the second against `0x55`, which is
also how our own request starts, so an echo is parsed as a reply.

So the HAL drops it here, like the ST path. The flag is set when `_write()`
queues something and cleared once `tx_pending()` goes false, from `_flush()` -
which `send_request()` calls immediately after writing - and from
`_available()`. The receive FIFO is purged before the flag clears and with the
interrupt held off, or the last echoed byte, which arrives at its stop bit just
as the transmitter finishes, gets committed by an interrupt landing in between.

### What the bench said

Read back over SWD with SmartAudio running on SERIAL3 (PIOUART0, PIO0 SM0/SM1,
GPIO42).

Connecting does **not** reset the board - `init` reads a running PIO block
straight away, and `ocd_process_reset_inner` in the log is init processing, not
a reset. What does reset it is `flash.sh`, which ends in `reset run`. Sampling
in the first few seconds after that reads a PIO block that is still all zeroes,
which is a good way to convince yourself nothing works. The settle in these
scripts is for that case only.

| | half duplex (GPIO42) | full duplex (GPIO16) |
|---|---|---|
| `PINCTRL` SET_COUNT | 1 | 0 - turnaround disarmed |
| `EXECCTRL` | wrap 0..9, `STATUS_N` 1, SIDE_EN | same |
| TX `ADDR` idle | 1 - parked on the pull | 1 |
| `DBG_PADOE` | 0 idle, 1 while sending | 1 always |
| pad | `PDE`, as asked for | no pull |

The receive machine parks at word 16, the `wait 1 pin` resync, because a
pulled-down line with nobody driving it looks exactly like the held-low case
that resync exists for. That is correct, and it is also how a reply gets
noticed: a VTX enabling its transmitter raises the line to idle before its
first start bit, which releases the park.

### The transmitter is provably correct

The receiver on a shared pin is a free loopback, so a debug build that records
every byte it pops - before half duplex drops the echo - reads back what
actually went onto the wire. `AP_PIOUART_DEBUG_ENABLED` fills
`pio_uart_dbg_rx_trace`, and it gave:

    00 aa 55 03 00 9f   aa 55 03 00 9f

which is SmartAudio GET_SETTINGS: sync `aa 55`, command `03`, length `00`,
CRC `9f` - recomputed independently as CRC-8 with poly 0xD5 over the frame, and
it matches. Every other counter agrees with the design: the first frame's
leading `00` was received and the second's was not (at boot the pad still had a
pull-up, so the first sacrificial byte had a real falling edge; after that it is
eaten, which is its job), `write_bytes` 12 against `rx_bytes` 11 accounts for
exactly that one byte, and there is one framing error per frame-end where the
line is released and falls.

Use this before theorising about framing. It turns "we believe the timing is
right" into a readback of the wire, and it needs no scope.

### The Betaflight A/B, and where it left the fault

Betaflight was flashed onto the same board and configured for SmartAudio on the
same pin, and got no reply either. The registers show it doing exactly what we
do - PIO1 with `GPIOBASE` 16, receive machine listening on the transmit pin,
`STATUS_N` of 1 for TX-FIFO-empty, pull-down pad, two stop bits, and the same
prepended `0x00` (`UART_TRAIT_BIDIR_PP_PREPEND` is 1 for PICO, with a comment
describing the missing-start-edge mechanism in the same terms). It was fully
configured for this board, not a bare target: PIO0 driving GPIO6-9 for DShot,
PIO2 driving GPIO21-22 for its own PIO OSD.

Two independently written implementations, one of them field-proven, agreeing on
the configuration and both silent, puts the fault outside the flight controller
firmware.

### Probing a pad for whether anything is on it

The useful technique from this session. Hand the pin to SIO as a pure input with
the output enable cleared, then alternate the internal pull-up and pull-down and
count. An unloaded pad follows the pull exactly; a pad with something on it does
not. Always include a known-unconnected pin and a known-connected one in the
same run, or the numbers mean nothing.

| pin | pull-up | pull-down | reading |
|---|---|---|---|
| GPIO17 (nothing attached) | 100/100 | 0/100 | follows the pull |
| GPIO20 (nothing attached) | 100/100 | 0/100 | follows the pull |
| GPIO23 (camera sync) | 94/100 | 90/100 | externally driven |
| GPIO16 (VTX SmartAudio) | 200/200 | 0/200 | follows the pull |

GPIO23 is what a connected pin looks like on this board, and GPIO16 does not
look like it. That is not proof of an open circuit - an idle CMOS pin is high
impedance and would read the same - but it is the strongest thing available
without a meter.

Two readings on the way here were wrong and are recorded so they are not
repeated. The first, on GPIO42, took a line collapsing under a pull-down as
evidence that nothing was connected; with SmartAudio's own pull configuration
that is the expected idle state. The second inferred "the PIO has released the
line" from `DBG_PADOE` bit 0 on a block with `GPIOBASE` 16, without checking
whether that register is windowed by `GPIOBASE`, and reported an external
pull-down on GPIO16 that the controlled test above says is not there. Both
mistakes have the same shape: a single measurement with no control, read as
proof.

### Status, and the next step

The driver is done and its behaviour is verified on hardware. What is not
resolved is whether this VTX ever answers SmartAudio at all, and that cannot be
settled from this board - the next step is the same VTX on an ST flight
controller with a known-good half-duplex UART. If it answers there, the fault is
on this board between the pad and the connector; if it does not, the VTX is not
speaking SmartAudio and none of this was ever going to work.

Still untested on this port as a result: receiving anything at all in half
duplex. Every transmit path is confirmed; the receive path is confirmed only in
that it correctly reads back our own echo and parks on a held-low line.

## Standing check: when you add to a hot path, look at where it landed

This board has been bitten by the same thing three times now, and it is
invisible in every source diff that causes it. Adding a line to an interrupt
handler puts that code wherever the linker feels like putting it, and the
default is XIP flash - so a handler that somebody deliberately relocated ends
up reached through, or extended by, code that is not.

**After changing anything that runs at kHz rates, read back where it is:**

```
arm-none-eabi-nm -C build/RPI_UAVFC/bin/arducopter | grep -i <function>
```

`0x10......` is flash. `0x20......` is SRAM: `0x2008xxxx` is Scratch X and
`0x2009xxxx` Scratch Y, the per-core banks. The three registries under
`hwdef/common/` decide it, and a function that is not in one of them is in
flash however hot it is.

The three times, so nobody argues the principle again:

- **Bidirectional DShot.** Took core1's flash share from 1.8% to 65.6% and
  evicted core0 out of the shared 16 KB cache hard enough to put CRSF into
  continuous failsafe. Recorded in the Scratch Y registry.
- **The OSD line interrupt.** The notes had already argued at length that the
  font must not live in flash, and the driver's own code was in flash the
  whole time. 14 kHz on core1. Underruns went from 5 per 10 s to none when it
  moved.
- **The PIO UART handler.** `_service_rx_fifo` had been put in Scratch X on
  its own, with a comment saying flash is the worst place for the RC drain.
  But the vector, the dispatcher, and later an error poll and a transmit
  drain, were all still in flash - four fifths of the handler, 2100 times a
  second, on the core the bank exists to protect.

The pattern in all three is the same: somebody relocated the expensive
function, and later work arrived either side of it. Relocation is a property
of the whole path, not of the one function whose name is in the registry.

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

log96 adds a worse case and one observation that the hypothesis above does not
cover. Roll walked from +1 deg to -108 deg through the acro segment and was
still -83 deg with the vehicle flat and stationary 15 s after landing,
recovering at about 0.25 deg/s. The prearm reported 89 deg.

The observation is that **`ErrRP` was already 0.63-0.75 before the motors ever
spun** - 1.5 to 4.0 s, throttle zero, `VIBE.VibeZ` 0.03 to 2.2, DCM roll still
correct at 2.5 deg. That is the window where `use_fast_gains()` is true and
DCM's P gain is 8x, so the fast-gains story does not explain it: the error term
is already large while the gain meant to hide it is still applied. log70 had
`ErrRP` 0.0019 on the ground, so this differs by airframe as well as by flight.

That makes it cheap to chase. It reproduces disarmed on the bench with the props
off, so `GA_e`, `_ra_deltat` and the GPS velocity term can all be instrumented
without flying.

One correction to the framing above. "Copter never flies on DCM, so this is not
a safety fallback issue" is right about the code path and wrong about the risk.
The `filter_faults != 0` route is still live, and a backup AHRS holding a 90 deg
roll error means taking it is not a degraded mode, it is an immediate flip. The
re-arm delay is the cost that shows up; it is not the cost that matters.

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

### Lit, and the colours were wrong: a borrowed constant off by five bits

Working now. The strip lights and the pattern is correct. Between the list
above and here the LEDs came on but every colour was wrong, which read as a
timing or byte-order fault and was neither.

`PIOUART.h` had `PIO_SHIFTCTRL_PULL_THRESH_LSB` as 20 against the true 25, and
`PUSH_THRESH_LSB` as 26 against 20. The authority is
`PIO_SM_SHIFTCTRL_*_THRESH_Pos` in the ChibiOS RP PIO header, in this same
tree: PUSH_THRESH is 24:20 and PULL_THRESH 29:25.

So `WS2812_BITS_PER_LED << PULL_THRESH_LSB` put 24 into PUSH_THRESH and left
PULL_THRESH at 0, which PIO reads as 32. Autopull refilled every 32 bits rather
than every 24, so each word clocked out G, R, B **and the zero low byte**. The
strip takes 24 bits per LED, so everything past the first LED shifted one byte:

| | bytes taken | shows |
|------|-------------|--------------------------------|
| LED1 | G1 R1 B1 | correct |
| LED2 | 00 G2 R2 | green 0, red G2, blue R2 |
| LED3 | B2 00 G3 | green B2, red 0, blue G3 |
| LED4 | R3 B3 00 | green R3, red B3, blue 0 |

The tell nobody used for a while: **LED1 is always right under this fault.** A
strip whose first LED tracks the requested colour while the rest do not is a
frame-alignment bug, not timing and not byte order.

Why it survived: PIOUART defines both constants and uses neither. Its transmit
program has an explicit `pull block` and its receive program an explicit
`push`, so no threshold is ever written - `SHIFTCTRL` there is just
`OUT_SHIFTDIR | FJOIN_TX`. `PUSH_THRESH_LSB` had no user at all and
`PULL_THRESH_LSB` had exactly one, in the NeoPixel path. The gotcha above about
this file borrowing PIO constants from `PIOUART.h` was written about the build
guards; it applies to the values too, and a constant its owner never exercises
is worth checking rather than trusting.

Read back over SWD, `PIO1` at `0x50300000`, NeoPixel on SM0:

| register | address | expected |
|-------------|--------------|--------------------------------------------|
| `CLKDIV` | `0x503000c8` | `0x001c2000`, 28 + 32/256, exactly 8.0 MHz |
| `EXECCTRL` | `0x503000cc` | `0x00003000`, wrap 0..3 |
| `SHIFTCTRL` | `0x503000d0` | `0x70020000`. `0x41820000` is the old bug |
| `INSTR` | `0x503000d8` | `0x6321` when idle, parked on the `out` |
| `PINCTRL` | `0x503000dc` | `0x24000840`, GPIO2 for SET and SIDESET |

`INSTR_MEM` reads back as zeros because it is write-only on this part. That is
not a missing program - `INSTR` showing `0x6321` is the proof it loaded.

Two things left in this path, neither of which caused the above.
`serial_led_send()` breaks out of the LED loop when `neopixel_send_word()`
times out and then still calls `send_end()`, so a partial frame goes to the
strip. And `neopixel_send_end()` stamps the completion time from
`WS2812_BITS_PER_LED * 8U`, a hardcoded eight LEDs, so a chain longer than that
gets a reset gap shorter than it should be.

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

### log96/97: the scale verified on this airframe, against the accel spectrum

The scale check above is log51, on the earlier airframe and the earlier ESC.
Repeated on both 2026-09-09 flights, because every quantitative claim about the
current pin is regressed against eRPM and nothing had ever checked the
regressor here.

The batch sampler's pre-filter accel blocks owe nothing to the ESC, so the
motor peak in their spectrum is an independent measurement of shaft speed.
Strongest peak in 40-700 Hz per block, keeping blocks where it lands within
20% of the eRPM fundamental, fitted through the origin:

| | slope | r | implied poles |
|-------|-------|-------|---------------|
| log96 | 1.027 | 0.988 | 14.37 |
| log97 | 1.011 | 0.996 | 14.16 |

against `SERVO_BLH_POLES` 14. That reproduces log51's 1.011 and 14.2 poles on a
different airframe and a different part, so eRPM is sound here to a few percent.

Beyond the notch, it settles one thing about the current work: the 26% gain
difference between the two flights is not an eRPM artifact. The model goes as
RPM cubed, so a 26% slope change needs an 8.7% scale error, and the measured
difference between the flights is 1.5% - worth about 4.5%. The instability is
in the pin.

**The pre/post-filter trap.** `INS_LOG_BAT_OPT` 4 logs pre- and post-filter
blocks and tells them apart in the ISBH `instance` field, not by IMU. There is
one IMU on this board: instance 0 is pre-filter, instance 1 post. The
post-filter accel carries essentially nothing above 150 Hz - band energy ratio
0.008 against 15448 for pre-filter - so an FFT that pools both finds noise
peaks and returns a plausible wrong answer. The first run of this check did
exactly that and gave slope 0.89, r 0.95, 12.5 poles. Select on `instance`
before believing any spectrum out of this log.

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

## log96: first flight on the ICM-56686, and the timing holds

274 s - 49 s Loiter, 177 s ACRO, 48 s Loiter - on an iFlight Nazgul Evoque F5
(`FRAME_TYPE` 18 BF_X_REV, 6S 1500 mAh, about 700 g AUW). Different airframe
from log69/log70 and a different IMU part, so nothing about vibration, the
tune or the current sense carries across from those.

The port itself needed nothing.

| | log96 |
|---------------------------|-------------------------------------------|
| `PM.NLon` | 0 after boot |
| `PM.MaxT` armed | 5029-5503 us against a 5000 us budget |
| `PM.Mem` | flat 56856, no leak |
| `PM.InE` / `ErrL` / `ErC` | 0 / 0 / 0 |
| `RTDT.dtMax` armed | mean 1.0 ms, P95 1.2 ms, worst 1.4 ms |
| Rate tracking | `RDes` 820.3 against `R` 812.3 at the peak |
| `RSSI.RXLQ` | min 97, mean 99.86 |

`dtMax` never left the 1.4 ms envelope log69 set, through 177 s of acro. The
one 5.25 ms sample is at 259.96 s, a second after disarm.

The 54.9 ms `PM.MaxT` log69 saw in its arm window did not recur, and
`LOG_DISARMED` is 0 here where it was 2 in log69. That is the control the log69
note asked for: no log rotation at arm, no spike, so the new-file plus
parameter-dump explanation stands.

### The IMU is the other part

`INS_ACC_ID` 4456706 has top byte `0x44` = `DEVTYPE_INS_ICM56686`. Every
earlier log reads `0x34`, the ICM42688P. So this is the first flight of the
ICM-56686 driver described above, and it needs nothing:

- at rest after disarm, |g| 9.6135 m/s/s, gyro (-0.0004, 0.0003, -0.0001) rad/s
  which is 0.02 deg/s, temperature 24.8 degC
- `IMU.EG` and `IMU.EA` 0, ODR flat at 3208 Hz for the whole flight
- `ISBH.mul` 104 on the accel blocks, which is `INT16_MAX/(32 g)` - the 32 g
  high-resolution configuration read back from the data rather than from the
  register write
- the harmonic notch tracked 63-400 Hz with a mean centre of 228-246 Hz against
  an ESC-RPM fundamental of 233 Hz

The 2% gravity shortfall is worth a fresh accel calibration - `INS_ACC1_CALTEMP`
is 28 degC against 24.8 flown - but the EKF absorbs about half of it as
`XKF2.AZ` 0.11 and nothing downstream noticed.

Batch-sampler headroom is fine, so FFT work on this log is valid: peak |ISBD|
is 18443 against the 32767 rail. Worth checking on any log used for notch work,
because the gyro blocks are scaled at `mul` 938, which is 2000 dps, on a part
running at 4000 dps - a hard enough manoeuvre would clip the log without
clipping the sensor.

### Logging: no drops, but the path was not stressed

`DSF.Dp` is cumulative for the file - `_dropped` is cleared only at log start,
`AP_Logger_Backend.cpp:101` - so read it as a total, not a rate. It reaches 473
in the first 5 s, which is the boot parameter dump, and then does not move for
274 s. Zero drops in flight, against log34's 18% and the half to three quarters
this board used to lose before the io_size fix.

`FMx` 81830, so the 80 KB buffer allocated. `FMn` never fell below 42.4 KB, so
the ring stayed more than half empty all flight.

That is not the measurement the SD section asked for, though. `LOG_BITMASK` is
180222 with bit 0 clear, so `MASK_LOG_ATTITUDE_FAST` is off and the rate-loop
streams logged at 10 Hz rather than 507. The offered rate was about 50 KB/s
against the 250 KB/s the path delivers. **The single-exchange write has still
never been measured under load.** Set the bit before the flight meant to answer
it.

The A/B that is here is worth keeping anyway, because it separates two things
this file had coupled:

| | Loiter-1 | ACRO | Loiter-2 |
|-------------|-----------|-----------|-----------|
| `DSF.Bytes` | 52.6 KB/s | 49.4 KB/s | 52.0 KB/s |
| `PM.Load` | 67.1% | 51.5% | 67.1% |

Logged bytes move 6% across the flight while load moves 15.6 points, so the
Loiter/ACRO load difference is position control, not logging.

### The PIO UARTs do not appear in the UART log message

Only instances 1 and 2 are in `UART`: SERIAL1 (MSP DisplayPort, 5378 B/s tx)
and SERIAL2 (GPS, 685 B/s rx), both hardware ports. `PIOUART` overrides none of
`get_total_tx_bytes`, `get_total_rx_bytes` or `get_total_dropped_rx_bytes`, so
it inherits the base class returning 0, and `log_stats()` skips any port that
has never seen data (`AP_HAL/UARTDriver.cpp:204`).

So `RxDp`, the count of received bytes the driver never processed, is
unreadable on SERIAL3 - the RC input, and the port the new framing and resync
work exists to protect. Three counters would fix it and would put the
framing-error discrimination in the flight log rather than on a bench probe.

Mind the decimation trap when reading this message: the instances alternate
strictly, so any even `--decimate` stride shows one port and silently hides the
other.

### Accel clipping, on this airframe

`VIBE.Clip` runs 2 at boot, 6 at 107.9 s and 12 at 181.0 s, against the 29.5 g
limit this part carries (`AP_InertialSensor_Invensensev3.cpp:1243`). `VibeZ`
averages 34 m/s/s through acro and peaks at 111. EKF velocity and attitude in
those windows are contaminated in the usual way.

log69 had no clipping and peaked at 33.9, but that was a different airframe, so
this is not a regression - it is what this one does.

## log97: the write path measured under load, and it runs out of buffer

The flight open item 4 asked for. `LOG_BITMASK` 180223 - bit 0 set, so
`MASK_LOG_ATTITUDE_FAST` is on - and 153 s split 102 s ACRO then 30 s Loiter.
22.4 MB against log96's 13.2 MB in twice the time.

**Through the acro segment the path is clean.** 155 KB/s delivered, `DSF.Dp`
frozen at 1711 for 101 consecutive seconds, buffer free oscillating 35-70 KB.
That is the single-exchange write doing its job and it is the first flight to
show it under a real offered rate.

**One second after the mode change to Loiter it collapses.** Mode goes to Loiter
at 106.1 s; the first drops land at 107.2 s, and by 136.3 s `Dp` has gone
1711 -> 31695. It recovers the instant the motors stop. The cause is core0 load,
not the card and not the write path - see below.

| | ACRO 20-100 s | Loiter 108-134 s |
|--------------------|---------------|------------------|
| `DSF.Bytes` | 154.9 KB/s | 139.0 KB/s |
| `DSF.Dp` | 1711, flat | 5517 -> 27084 |
| `DSF.FMn` | 33-37 KB | ~970 bytes |
| `DSF.FMx` | ~70 KB | ~33,730 bytes |
| `PM.Load` | 59% | 77% |

### It is core0 CPU, not buffer headroom

Retracted: this section first said Loiter "costs about 34 KB of buffer
headroom" and recommended halving io_size. Both are wrong, and the first is not
even coherent - nothing allocates, the ring is a fixed allocation, and a ring
only fills because drain fell below offer. The 34 KB was the ring being full,
which is the symptom.

What the ring is doing. `critical_message_reserved_space()` is a flat 1024 bytes
(`AP_Logger_Backend.h:227`) and non-critical messages are refused below it
(`AP_Logger_File.cpp:473`). `FMn` sits at 834-995 through Loiter, so the floor
being hit is that reserve, not zero. `FMx - FMn` of 32,758 is then the
saturation signature rather than a cause: once saturated the writer clears
exactly one io_size between sync stalls and the producers refill all of it
during the next stall.

The measurement that matters is drain capacity against core0 load, and two
regimes give it directly. When the ring is full, delivered *is* capacity; when
it is not, delivered is only what was offered.

| | ring state | delivered | `PM.Load` |
|--------------------|-------------------|-----------|-----------|
| ACRO, armed | `FMn` 34-36 KB | 156-160 KB/s (offer, not capacity) | 59% |
| Loiter, armed | `FMn` ~970 B, at reserve | **141-147 KB/s = capacity** | 77% |
| after disarm | draining a backlog | **195-198 KB/s = capacity** | 65% |

**Same board, same card, same file: 12 points of core0 load costs about 40% of
microSD throughput.** That is the whole of it, and it is the core0 starvation
this file has claimed all along - measured this time rather than inferred.

Loiter's own logging is not the explanation and can be dismissed with a number.
`PSCN`, `PSCE` and `PSCD` log at 7/s each, 21 messages/s, about 1.1 KB/s. Across
all 77 message types the *logged* rate actually falls in Loiter, 2850/s to
2579/s, because messages are being lost. Whatever Loiter costs, it is not extra
log volume.

One thing does not reconcile. `DSF.Dp` climbs at about 1791/s while the
all-types logged rate falls by only 271/s, so taking `Dp` as a count of
distinct messages implies an offered rate that rose 53%.

Checked since, and **in flight `Dp` is a count of distinct rejected messages,
not re-offers.** `_writing_startup_messages` is true only inside
`_startup_messagewriter->process()` (`AP_Logger_Backend.cpp:139`), and that
writer stops once `finished()`. Everything logged in flight therefore takes the
`else` branch at `AP_Logger_File.cpp:473`, and vehicle logging is
fire-and-forget - nothing re-offers a rejected message. The one place re-offers
do get counted is the boot FMT phase, where `_writing_startup_messages` is true
but `fmt_done()` is false, so the no-count branch is skipped while the writer
retries. That is the 473 in the first five seconds, and it is why boot `Dp`
reads differently from in-flight `Dp`.

So the offered rate really did rise, and the explanation this file gave for
dismissing Loiter's own logging does not hold: "`PSC*` is 21 messages/s, about
1.1 KB/s" is a count taken *from the file*, which is post-drop. Under
saturation the logged rate of any stream understates what was offered by
whatever fraction is being refused, so it cannot be used to bound what that
stream costs. Position control adds streams that ACRO does not have, and how
much they offer is still unmeasured - the logged rate cannot answer it.

Retracted: "try io_size 16384 first". It goes the wrong way. The sweep in the
next section has throughput *falling* as io_size falls - 211.4 KB/s at 16384
against 265.2 at 32768 - so halving it lowers capacity, which is exactly the
term that is already short. Smaller io_size buys a shallower sawtooth, and a
shallower sawtooth is worth nothing under sustained saturation.

What is left is the two ends of drain versus offer:

- Cut the offered rate. `LOG_FILE_RATEMAX` 67 rather than 100, which the
  rate-limiter section already prescribes when `DSF.Dp` climbs, or clear
  `MASK_LOG_ATTITUDE_FAST` again as log96 flew it.
- Cut core0 load. This is now the item with a measured price on it: the veneer
  work in `PROFILING.md` is worth roughly 3 KB/s of log bandwidth per point of
  core0 load recovered.

### Everything else in the flight is clean

`PM.NLon` 0, `Mem` flat at 56856, `InE`/`ErrL`/`ErC` all 0, `RTDT.dtMax` worst
1.7 ms armed. `VIBE.Clip` stayed at its boot value of 2 for the whole flight -
no accel clipping at all, against log96's 10 events, at similar vibration
levels.

## Why core0 load costs log bandwidth: the mechanism

Not board specific. This is stock ArduPilot behaviour and it is why the same
effect shows up on STM32; only the size of one term differs. Recorded here
because log97 is where it was finally measured rather than asserted.

Three things multiply, and only the first is scheduling.

**1. The logging IO thread is the lowest-priority thread in the system.**
`Scheduler.h`, and ChibiOS numbers priorities upwards:

| thread | priority |
|-----------------------|----------|
| monitor | 183 |
| timer, rcout | 181 |
| main loop | 180 |
| rcin | 177 |
| UART, LED, net | 60 |
| storage | 59 |
| **IO, which owns logging** | **58** |
| scripting | LOWPRIO |

Everything preempts it, the UART threads included. It runs on what the main
loop leaves and nothing schedules it otherwise: `Scheduler::_io_thread()` is a
plain `delay_microseconds(1000)` then `_run_io()` loop.

**2. The write is CPU work, not a DMA wait.** This is the term that turns
scheduling latency into lost throughput, and the one that is easy to miss. If
`io_timer()` merely kicked a DMA and slept, being descheduled would cost
latency and almost no bandwidth - the transfer would proceed while the thread
was off CPU. It does not. The measurement in the SD section below is 631 us of
wall time per 512 byte block against 275 us of actual clocking, so 56% of every
transaction is CPU-side overhead executed in that priority-58 thread. Leftover
CPU therefore converts more or less directly into bytes per second.

The arithmetic checks out on log97, which has two regimes where the ring was
saturated or draining and delivered throughput is therefore capacity:

| | `PM.Load` | CPU left | capacity |
|-------------------|-----------|----------|-----------|
| after disarm | 65.4% | 34.6% | 196 KB/s |
| Loiter, armed | 77.6% | 22.4% | 144 KB/s |

Residual-CPU prediction 0.224/0.346 = 0.65 against an observed 144/196 = 0.73.
Right size, right direction. The gap is expected: `PM.Load` is
`load_average() * 1000` (`AP_Scheduler.cpp:474`) and measures only the main
scheduler, not the timer, rcout, rcin and UART threads that also preempt IO.

**3. It writes in bursts, and one burst is always ineligible.**
`_writebuf_chunk` is `HAL_LOGGER_WRITE_CHUNK_SIZE` (`AP_Logger_File.h:116`),
which on FATFS is `AP_Filesystem_FATFS::get_io_size()` - 32768 here, from
`hwdef.dat:389`. Then `io_timer()` does

```c
if (nbytes < _writebuf_chunk && tnow - _last_write_time < 2000UL) {
    return;   // write in chunk-sized chunks, or at least once per 2 s
}
```

so nothing is written until `_writebuf_chunk` has accumulated.

Retracted: this paragraph said `_writebuf_chunk` is 32768 here, that the
32,758 byte sawtooth is therefore the chunk rather than the f_sync interval,
that 32 KB of the 80 KB buffer sits permanently below the write threshold, and
that each burst is about 64 block writes. All four are wrong. **The chunk is
4096.** It is captured at construction from `get_io_size()`, every backend is
constructed before any `Init()` runs, and on FATFS nothing mounts the card
until the first filesystem access - so it takes the pre-mount default of
`AP_FATFS_MIN_IO_SIZE`. `BRD_SD_SLOWDOWN` is 0 here, so the one early-mount
path in `AP_Vehicle::setup()` is not taken either.

The sweep in "Most of the writes were metadata" proves it without a probe,
because blocks-per-call is fixed by the chunk and the metadata ratio:

| io_size | measured | if data writes are 8 blocks | if 64 blocks |
|---------|----------|-----------------------------|--------------|
| 4096 | 2.40 | (8 + 4.01)/5.01 = **2.40** | 13.4 |
| 32768 | 5.52 | (8 + 0.55)/1.55 = **5.52** | 41.6 |

Both points land exactly on 4096 byte data writes, and the single-sector shares
agree too - 4.01/5.01 = 80.0% against 80.1% measured, 0.55/1.55 = 35.5% against
35.4%. So the io_size sweep never moved the chunk; it only ever moved the
f_sync interval. The 32,758 byte sawtooth is the sync interval after all, which
is what the "wrong explanation fitted" caveat was hedging against.

What follows: only 4 KB sits below the write threshold, not 32 KB, so the
usable slack is nearer 76 KB; each burst is 8 block writes, not 64; and the
per-call FATFS and CMD25 cost is paid eight times per sync rather than once.

### Why STM32 shows it too

Terms 1 and 3 are identical - same scheduler, same `io_timer()`. Term 2 is
weaker but not absent. SDIO/SDMMC moves the data by DMA, but the IO thread
still pays for FATFS bookkeeping, cluster and sector management, and `f_sync`'s
FAT, directory and FSINFO writes. And the one that is easy to overlook: on the
SDC path the bounce buffer is `io_size` (`sdcard.cpp:95`), so every 32 KB write
carries a 32 KB `memcpy` in that same thread. That is a CPU-per-byte term on
H7 as well.

So the slope is shallower there, not flat. Same mechanism, smaller coefficient.

### What follows for fixes

- Raising `LOG_FILE_BUFSIZE` does not change capacity. It only lengthens the
  overload burst that can be absorbed before messages are lost.
- Lowering io_size lowers capacity, because there are fewer bytes per unit of
  per-transaction overhead. Still the wrong direction, see the log97 section.
- The only two levers are less offered rate and more free CPU. That is what
  puts a price on the core0 flash work: about 3 KB/s of log bandwidth per point
  of main-loop load recovered.

### `_writebuf_chunk`: answered, and it was 4096 (fixed)

This asked which value the chunk took at boot. The answer is 4096 on every
boot, not just after a mount retry - see the retraction above for the
arithmetic that settles it from the existing sweep.

The ordering is the cause and it is not board specific. `AP_Logger::init()`
constructs every backend (`AP_Logger.cpp:273`), and `AP_Logger_File::probe()`
only calls `new`. All the `Init()` calls come afterwards, in a second loop
(`:286`), and `Init()` is the first thing to touch the filesystem, which is
what mounts the card and raises `io_size`. So the constructor always reads the
pre-mount default. Any FATFS board that raises `AP_FATFS_MAX_IO_SIZE` has the
same gap, H7 included.

Fixed by refreshing the chunk at the end of `Init()`, once the mount has
happened, bounded to half the allocated buffer so the threshold stays
reachable - `io_timer()` will not write until the chunk has accumulated, so a
chunk larger than the buffer would leave only the 2 second timeout driving
writes. That bound also caps it below the `uint16_t` wrap the old note warned
about.

**Measured, and reverted.** It does nothing, because the write size was never
the chunk's to set. See the next section.

Retracted with it: an estimate that the change was worth "up to 26%", reasoned
from the 21% of time spent outside `mmc_write`. The reasoning was fine and the
premise was wrong.

### The write size is capped by the FAT cluster size

This is the thing neither the chunk nor io_size can move, and it explains why
every data write in every measurement in this file has been exactly 8 blocks.

`ff.c:4013`, in `f_write`:

```c
cc = btw / SS(fs);                  /* sectors remaining to write */
if (cc > 0) {
    if (csect + cc > fs->csize) {   /* Clip at cluster boundary */
        cc = fs->csize - csect;
    }
    disk_write(fs->pdrv, wbuff, sect, cc);
```

FATFS clips every multi-sector transfer at the cluster boundary and does not
coalesce across clusters even when they are contiguous. The read path at
`:3898` does the same. Read off the mounted card - `FatFs` holds the object
pointer, `csize` is at offset 10 - this card is **`csize` 8, so 4096 byte
clusters**, alongside `fs_type` 3 and `n_fats` 2. So `disk_write` can never be
handed more than 8 sectors, whatever `f_write` is given.

The A/B, one boot, f_sync interval held at 32768 throughout by giving the chunk
its own SWD-pokeable override rather than sweeping io_size, which would have
moved both terms at once:

| | chunk 4096 | chunk 32768 |
|-----------------|------------|-------------|
| blocks / call | 5.53 | 5.53 |
| n histogram | 1:454 8:835 | 1:442 8:809 |
| card blocks | 179.9 KB/s | 164.9 KB/s |
| us/block exchange | 594 | 585 |
| us/block idle | 409 | 403 |

Identical histograms and identical blocks per call: bin 9, which is "9 blocks
or more", stayed empty in both. The 8% throughput difference is not a gain
going the other way, it is the same write pattern taking slightly longer -
with a 32 KB chunk the writer waits for 32 KB and then issues eight
cluster-capped writes back to back rather than spreading them.

**Check the histogram before believing any throughput number here.** If bin 9
is empty the write size did not change, and whatever the KB/s did is something
else.

What follows. Raising the chunk is necessary but not sufficient; to get larger
writes you need a larger cluster size *and* a chunk big enough to fill it, and
neither alone does anything. Sizing the gain from this measurement: inside
`mmc_write` the per-block terms are 587 + 400 = 987 us against 1434 us/block
total, so roughly 31% is per-call overhead. Removing seven of eight data calls
recovers part of that, order 10%, not the 26% claimed above. That is a
reformat of the card, so it is not free, and it is worth less than the core0
work - which log97 prices at 40% of throughput for 12 points of load.

## The SD write path is CPU-starved, not card-limited

Retracted a second time, and this one moved the number. The dominant cost was
neither CPU nor the card: it was the filesystem syncing every 4 KB. See "Most
of the writes were metadata" below, which took 115 KB/s to 265 with a one line
change. What follows is still the right analysis of what is left.


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

Agreed approach, in order. (a) is implemented, see the section after this one.
(b) and (c) remain.

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

Note this is not the same as prebuilding one 4 KB frame and clocking it blind,
which the busy distribution rules out outright - see the section on that below.
(b) still waits per block; it only moves the wait off the thread. The same
measurement is the argument for it, since 91.4% of blocks are ready with no
wait at all.

**(b) was attempted and backed out.** It wedges the SPI bus on hardware. Read
the section below before starting again - two of the three things that broke
are not obvious from the code, and one of them invalidates the design as
originally sketched above.

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

### (a) as built

`mmcSequentialWrite()` stages the whole frame and clocks it in one exchange:

```
[0xFF][0xFC][512 data][2 dummy CRC][1 response slot][256 busy bytes]
```

773 bytes, 275 us of wire time. The exchange is in place, so afterwards the
staging buffer holds what the card sent back rather than what went out: the
data response token at offset 516, then the busy window. That makes the path
observable over SWD with no instrumentation - read the buffer and you have the
last block's response and how long the card stayed busy.

It is opt-in. `MMCD1.wbuffer` comes out of `mmcObjectInit()` as NULL and NULL
selects the original four-transfer path, so nothing else using `hal_mmc_spi.c`
changes behaviour. `sdcard.cpp` allocates the buffer and the driver falls back
on its own if that fails.

Use the in-place `transfer_fullduplex(uint8_t *, uint32_t)` overload, not the
two-pointer one: the latter stages through a `uint8_t buf[len]` variable length
array on the caller's stack, which here would put 773 bytes on `log_io`.

The busy window is the part worth tuning, and `mmc_wait_idle()` was the part
worth fixing first. It polled a byte at a time, and every poll is a DMA setup
and a thread suspend - exactly the cost the staged frame exists to avoid, paid
sixteen times over for sixteen byte-times of wire. It now reads
`MMC_BUFFER_SIZE` and scans. Overshooting the moment the card goes idle is
free: the extra bytes clock against an idle bus, and a gap before the next
token is allowed.

Measured over 5932 block writes on the bench with `LOG_DISARMED` 1, counted
only on the block write path so that command traffic - which calls
`mmc_wait_idle()` with the card already idle - cannot flatter it. The card
finished inside the 256-byte window **91.4%** of the time. The 8.6% that
overran it distribute as:

| wait past the window | share | cumulative |
|---|---|---|
| under 16 us | 58.7% | 58.7% |
| 16-63 us | 26.4% | 85.2% |
| 64-255 us | 2.2% | 87.3% |
| 512 us - 2 ms | 5.4% | 92.7% |
| 2-10 ms | 7.3% | 100% |

Mean 406 us, worst 10.3 ms. The empty band between 256 us and 512 us is an
artefact of the 500 us sleep, which quantises anything it catches, so the true
spread there hides inside the 512 us bucket.

Two conclusions. The window is already the right size - widening it to 512
bytes would add 91 us of wire time to *every* block to save about 25 us of
polling on 8.6% of them. And no further poll phase is worth adding: two reads
land before the first sleep, covering everything out to 256 us, so the ceiling
on a longer poll is the 1.7% in the 512 us bucket.

The rest of the tail is the card erasing internally - real work, not overhead.
This supersedes an earlier reading of the same distribution as "about 40% of
blocks run past 91 us", which came from fifteen hand-counted samples and was
wrong.

Throughput is not yet measured. `PM.SPIC` cannot settle it, because fewer
transfers per block and more blocks per second move it the same way - it sat at
8000 to 10400 a second either side of the change. The figure to take is
`PM.Load` against `DSF.Bytes` across an acro segment and a Loiter segment, the
same way the problem was diagnosed in the first place.

### The `idle` shortcut silently lost whole writes

The staged frame originally read the last byte of the busy window and, if it
was 0xFF, returned success without polling at all. That is not evidence. A
card that has finished programming and a card that has not yet pulled MISO low
read back identically, and on the second the next data token goes into a busy
card, which discards the block and reports nothing. `f_write()` then returns
success, `AP_Logger` advances `_write_offset`, and the data is gone.

Logs 78 and 81 are what that looks like from the outside. Both begin with a
run of zero bytes - 2,985,984 and 6,230,016, each an exact multiple of 4096 -
followed by one contiguous run of real data containing no FMT records at all.
The preamble and the first minutes of each flight were written, acknowledged
and never landed; the file kept its full length because the offset had moved.
Log 82 from the same session is dense from byte 0. Logs 79 and 80 were zero
length, which is the same failure with nothing surviving.

Two things make this worth remembering. A dropped message is not this - drops
show up in `DSF.Dp` and leave the file dense - so the two symptoms need
separating before diagnosing. And `AP_Logger_File::io_timer()` only advances
the offset on a successful write, so a *failed* write cannot produce a hole;
only a write that returns success without committing can, which narrows the
search to below `f_write()` immediately.

`LOG_FILE_DSRMROT` 1 makes this much more likely to bite, because it closes
the log and opens a new one on every disarm: flush, FAT update, then 184 FMT
plus 195 FMTU plus around 1400 PARM records, over 100 KB, as fast as the
buffer can feed it. That is the heaviest SD moment of a flight and it happens
at the exact instant `spi_fail` is reported.

### 4 KB cannot go out as one exchange

Worth recording because the STM32 path makes it look obvious. There the logger
writes 4 KB, FATFS passes it through, and SDMMC streams the whole multi-block
payload on one DMA with the card's busy handled in hardware on DAT0. The
natural question is why SPI mode does not prebuild the same 4 KB as eight
framed blocks and clock it in one exchange.

Because in SPI mode busy is in-band on MISO. Concatenating eight blocks means
guessing each inter-block busy length in advance, and guessing short is the
`idle` shortcut failure again, eight times per write. At the measured 8.6%
per-block overrun rate:

    P(all eight fit their window) = 0.914^8 = 0.49

Half of all 4 KB frames would contain a block that ran long, and every one of
those would have to be detected and rewritten whole. Against a saving of about
3% - seven thread suspends out of a 2.2 ms write - it is not close.

This does not rule out (b). That design still waits per block, it only moves
the waiting off the thread, and the measurement argues for it: 91.4% of blocks
need no wait at all, so an ISR-driven walk would usually run to completion
without rescheduling `log_io`.

### (b) ISR-driven chaining: attempted, backed out

Built and run on hardware three times, wedged the board three times, reverted.
The mechanics of chaining work; what it exposes underneath does not. Recorded
in enough detail that the next attempt starts from here rather than from the
sketch in the plan above.

**Busy cannot be watched out of band.** The design rested on reading the MISO
pad instead of clocking idle bytes - `IO_BANK0 GPIO[n].STATUS` bit 17
(`INFROMPAD`) reports the pad level whatever the pin is muxed to, so a register
read looked like a free substitute for a DMA transfer. It is not. A card in SPI
mode releases DO **in response to clock edges**: clocking is not how busy is
observed, it is what lets busy end. With nothing clocking, the chain sat on
block 0 for the full 500 ms timeout, 2142 re-polls deep, on every run. This is
the one real asymmetry with SDMMC, where DAT0 is genuinely free-running, and it
is why the 4 KB payload-only staging that works there cannot work here.

The evidence was in hand beforehand and was misread. MISO measured low with the
card mounted and the select deasserted, which was written off as unexplained
but irrelevant. It was the whole answer.

**The validation that passed was not a validation.** A correlation counter
compared the pad against the clocked busy byte and returned 20288 samples with
zero disagreement. It sampled after `transfer_fullduplex()` returned, when the
card is idle 91.4% of the time, so it was overwhelmingly comparing two ways of
saying "not busy". The gap was noticed and an attempt was made to close it by
sampling the frame buffer over SWD and finding 0x00 in 38% of reads - but those
were asynchronous reads at random moments, which show busy exists, not that the
counters ever sampled one. The check that would have caught this is a counter
for "the clocked byte said busy", required to be non-zero. Any future
correlation test needs that counter before its result means anything.

**A chained failure must close the multi-block write.** The chain leaves the
card mid-CMD25 with the select asserted and the driver in `BLK_WRITING`. The
per-block path gets its cleanup for free, because `mmcSequentialWrite()`
unselects and drops to `BLK_READY` on its own error path; a chain that breaks
out of `mmc_write()` skips `mmcStopSequentialWrite()` and every later transfer
fails. Symptom is `MMCD1` stuck in `BLK_STOP` or `BLK_CONNECTING` with
`sdcard_running` still 1.

**What was not solved, and is the reason it is parked.** Even with the busy
clocked from the ISR and the cleanup in place, roughly one run in a dozen never
completes. The thread times out, and because a transfer is left outstanding the
driver stays in `SPI_ACTIVE` - after which `do_transfer()` and the
`mmcConnect()` of the retry that is meant to recover it both block forever.
That is a stuck thread and an internal error, and only a power cycle clears it.
`poll-limit failures` stayed 0 throughout, so the ISR was not spinning on busy:
a transfer simply stopped completing.

The untested suspicion, and where to start: DMA IRQs on this part are per core.
`INTE1` carries the SPI channels, so their completion services on core1, while
`log_io` waits on core0. Chaining means arming the next transfer from core1's
interrupt while the owning thread sleeps on core0 - the per-block path never
does that, and it is the one structural difference between the design that
works and the one that does not. Snapshot `chain.idx`, `polls`, `polling` and
`spip->state` at the timeout, and read the SPI1 DMA channel `BUSY`,
`READ_ERROR`, `WRITE_ERROR` and `TRANS_COUNT` while it is wedged, before
changing anything.

Three ChibiOS details worth keeping whatever the next attempt looks like. The
`end_cb` in `SPIConfig` is available because this board builds SPIv1 - the
`HAL_LLD_SELECT_SPI_V2` switch is undefined, and the comment in `SPIDevice.cpp`
about v2 not having `end_cb` does not apply here. `_spi_isr_code()` calls the
callback *before* it takes the lock, and virtual timer callbacks also run
outside the critical section, so both must lock for themselves.
`SPI_SUPPORTS_CIRCULAR` is FALSE on RP, so there is no `spiAbort()` to cancel a
transfer that has gone missing - which is precisely why an outstanding transfer
is unrecoverable and the bus stays dead.

### Most of the writes were metadata

`AP_Logger_File::io_timer()` syncs whenever a write lands on an
`AP_Filesystem_FATFS` `io_size` boundary (`AP_Logger_File.cpp:1040`), and
`io_size` is `AP_FATFS_MIN_IO_SIZE`, 4096. Each `f_sync` writes the directory
entry, the FSINFO sector and one sector per FAT copy - and this card has
`n_fats` 2. Four single sector writes per 4 KB of log, each paying a whole
CMD25 and a card program cycle for 512 bytes.

Measured over 20 s, disarmed, by counting the `n` passed to `disk_write`:

| blocks per `disk_write` | calls | share |
|---|---|---|
| 1 | 1510 | 80.0% |
| 8 | 376 | 20.0% |

4.01 metadata writes per data write, exactly what the mechanism predicts. A
third of every block reaching the card was filesystem overhead.

`AP_Filesystem_FATFS::set_io_size()` already existed for this - and the SDC
path in the same file already called it. `chibios.h` has carried

```
// 32k gives huge performance improvements on boards that can cope
#define AP_FATFS_MAX_IO_SIZE 32768
```

since it was added for STM32H7, gated on
`defined(STM32H7) && HAL_MEM_CLASS >= HAL_MEM_CLASS_1000`. Everywhere else
`AP_FATFS_MAX_IO_SIZE` falls back to 4096, and the MMC-SPI path never made the
call at all, so nothing outside H7 has ever used it. **H7 has been syncing
every 32 KB all along; this is the same value on the other path.**

It is cheaper here than there. On SDC the bounce buffer is `io_size` and
`HAL_LOGGING_FILE_BUFSIZE` gives up 28 KB to pay for it; on MMC-SPI the
staging buffer is one block, `MMC_WRITE_FRAME_SIZE`, so the log buffer is
untouched and the change costs no memory.

Setting it to 32768 at mount:

| | 4 KB sync | 32 KB sync |
|---|---|---|
| delivered | 115 KB/s | **265 KB/s** |
| blocks per `disk_write` | 2.40 | 5.52 |
| single sector share | 80.0% | 35.4% |
| metadata per data write | 4.01 | 0.55 |
| card busy per block | 1175 us | 413 us |
| wire per block | 578 us | 631 us |
| time inside `mmc_write` | 54% | 79% |

Card busy fell 2.8x because each metadata write had been paying its own
program cycle.

The whole curve, swept by writing `io_size` over SWD with no reflash - it is a
static in `.data`, so the trade can be explored on a running board:

| `io_size` | throughput | blocks/call | single sector share | tail at risk |
|---|---|---|---|---|
| 4096 | 118.9 KB/s | 2.40 | 80.1% | 34 ms |
| 8192 | 158.1 KB/s | 3.31 | 67.0% | 52 ms |
| 16384 | 211.4 KB/s | 4.42 | 51.1% | 78 ms |
| 32768 | 265.2 KB/s | 5.50 | 35.6% | 124 ms |

There is no knee - each doubling buys about a third more throughput and costs
about half as much again in exposure, so the value is a judgement rather than
an optimum.

**What the exposure actually is.** The file data is written as it goes; what
lags is the directory entry, so a hard power cut leaves up to `io_size` of log
on the card but outside the recorded file length. `LOG_FILE_DSRMROT` is set
here and disarm closes the file, and crash detection disarms - so a crash the
flight controller survives syncs everything. The exposure is battery ejection
or a severed lead, not a crash as such. Weigh it against what the 4096 setting
was costing: at the offered rate this board logs at, half to three quarters of
every message was being dropped for the whole flight. Note what that means for the older conclusion: the card was
never as slow as it looked, it was being asked for eight times too many
program operations.

### Sizing the offered rate to match

Fixing the write path was only half of it: with `MASK_LOG_ATTITUDE_FAST` set
the rate loop offers PIDR, PIDP, PIDY, PIDA and RATE at
`calc_gyro_decimation(2, 1000)` = every second iteration of a 2027 Hz loop,
so 1013 Hz - 4 x 52 + 63 bytes each, 268 KB/s, before anything else logs at
all. log34 still dropped 18% of every message with the io_size fix in.

`LOG_FILE_RATEMAX` is the knob, and it does not mean what it says.
`AP_Logger_RateLimiter::should_log()` caches its decision per message id per
scheduler tick (`AP::scheduler().ticks()`), so it gates whole ticks rather
than individual messages. At `SCHED_LOOP_RATE` 200 a tick is 5 ms, so:

| `LOG_FILE_RATEMAX` | ticks passed | effective stream rate |
|---|---|---|
| >= 200 | every one | no limiting at all |
| 100 | every 2nd | ~507 Hz |
| 50 | every 4th | ~253 Hz |
| 25 | every 8th | ~127 Hz |

Measured, with `defaults.parm` settling on 100:

| `LOG_FILE_RATEMAX` | delivered | time inside `mmc_write` |
|---|---|---|
| 0 (none) | 250 KB/s | 78-82%, saturated |
| 100 | 176 KB/s | **52.6%** |
| 50 | 108.8 KB/s | 30.6% |

Falling throughput is the success condition: the write path stops being
saturated and only does the work it is asked for. 100 keeps real headroom
while leaving the rate loop streams at 507 Hz, which is twelve times Nyquist
for the 15-40 Hz band `rate_band.py` works in; `rate_response.py` does not
believe anything above 10 Hz regardless, because demand and response share a
noise source. The 1013 Hz was oversampled by about 5x for anything downstream
of it. Drop to 67 if a flight log shows `DSF.Dp` climbing.

**Card blocks are not logger bytes, and confusing them invents a flight
penalty that is not there.** The sweep figures above count `mmc_wr_blocks`;
`DSF.Bytes` counts what the logger handed over. At `io_size` 4096 a third of
the blocks were metadata, so 119 KB/s of card blocks is about 79 KB/s of
logger data - which is log25's 77. At 32768 metadata is 6% of blocks, so
265 becomes 248, which is log34's 250. Both logs agree with the bench sweep
once that is accounted for, and both were bench arm/disarm cycles rather than
flights (`VIBE` 0.008-0.10, `RATE.R` about zero), so nothing here measures a
real flight load at all.

### What the ceiling actually is

Read from the hardware rather than assumed. `SSPCR0` gives SCR 4 and `SSPCPSR`
2, so SPI1 runs at 225/(2 x 5) = 22.5 MHz, and only the card is on that bus.
`MMC_WRITE_FRAME_SIZE` is 773 bytes per 512 of data - the 256 byte in-frame
busy window is 33% of it.

| | |
|---|---|
| Raw wire | 2.81 MB/s |
| With the 773 byte frame | 1.86 MB/s |
| At measured wire + card busy | 490 KB/s |
| Inside `mmc_write` only | 335 KB/s |
| Delivered | 265 KB/s |

**Wire is now the dominant term and it is mostly not wire.** 631 us per block
against 275 us of actual clocking is 356 us of transaction overhead, 56% of
every exchange - the round trip cost this section has always described, now
the largest single item rather than the fourth. That is what plan (b) and (c)
are for.

For scale: H7 SDMMC at 4 bit does 12.5-25 MB/s, so SPI mode costs 7-13x before
any software is involved. That part is not recoverable on this silicon - see
"RP2350 has no SD host controller" below.

### Measuring it again

Set `MMC_USE_WRITE_STATS TRUE` in the hwdef to build the counters in; they are
off by default and the flight build carries none of them. They live in the
ChibiOS fork's `hal_mmc_spi.c`: `mmc_wr_blocks`,
`mmc_wr_calls`, `mmc_wr_us_exchange`, `mmc_wr_us_idle`, `mmc_wr_us_call`,
`mmc_wr_n_hist[10]`, and the `mmc_wait_idle` outcome counters. Read over SWD.

Two traps found the hard way. A counter for "blocks that needed no wait"
measured nothing, because a 16 byte poll cannot take zero microseconds at 1 us
resolution - the earlier claim that 91.4% of blocks are ready with no wait is
not what that measured and is not established. And the `mmc_wait_idle`
outcome counters catch every caller, not just the per block ones, so they
total more than `mmc_wr_blocks`; the proportions are usable, the absolute
counts are not per block.

**Flashing wedges the card.** A reset landing inside a CMD25 leaves the card
in a multi block write that survives the reset, and it will not answer CMD0
again until it loses power. `MMCD1.state` 1 and `SDC_FS.fs_type` 0 with
`sdcard_retry_interval_ms` at its 30000 ceiling is that state. Power cycle
between flashes when the card is being written.

### The write buffer is still short - no longer true

`DSF.FMx` reaches 81830 in a 2026-09-08 log, so the 80 KB allocation now
succeeds and this section is kept only so the reasoning is not repeated. What
follows was written when it did not.

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

## The SPI peripheral was torn down on every transaction

`SPIDevice::acquire_bus()` ran `stop_peripheral()` then `start_peripheral()`
every time it asserted CS. On RP2350 that is not a register write: `spiStart()`
holds `osalSysLock()` across `spi_lld_start()`, which frees and reallocates both
DMA channels, so the whole cycle runs inside the global cross-core spinlock.

It only skipped the cycle when CS was already held. The SD card is the one path
that qualifies - the MMC driver holds CS across a whole multi-block sequence -
so logging never paid it. Everything else did, once per transfer.

Measured on the bench, then again after fixing it:

| | before | after |
|--------------------------|----------|----------|
| SPI0 teardowns | 6635/s | 334/s |
| core1 CPU in teardown | 3.97% | 0.36% |
| core0 spinlock stall | 0.865% | 0.639% |
| core1 spinlock stall | 0.603% | 0.576% |
| worst stall, core0/core1 | 9/11 us | 10/12 us |

About 3.8% of a core, 3.6 of it on core1 where the rate loop lives. SPI transfer
rate held at about 6850/s across both, so the workload is comparable.

`SPIBus::apply_config()` now compares `SSPCR0`, `SSPCPSR` and the chip select
against what is already programmed and cycles the hardware only on a real
change. Note the chip select fields had to move into it: they were previously
written before any comparison could see them, which would have masked a genuine
device switch on a shared bus and driven the wrong CS.

Two things this did **not** buy. Worst-case cross-core stalls are unchanged at
10-12 us, so this is a CPU win and not a latency one - the ~14000 blocks/s each
core sees are mostly collisions with short scheduler critical sections, not with
SPI. And it does nothing for SD throughput, because that path already skipped
the cycle.

The residual 334/s costs 10.66 us each, up from 5.72. The survivors are genuine
reconfigurations - the Invensense driver alternating between low-speed register
access and high-speed FIFO reads changes `SSPCR0`. Removing those too needs a
reconfigure-in-place path in the RP SPI LLD, since changing `SSPCR0` needs SSE
cleared but does *not* need the DMA channels freed. That is 0.36%, so it is
only worth doing if the rate loop needs the headroom.

Instrumentation for repeating any of this: `AP_RP2350_SPI_CYCLE_STATS_ENABLED`
counts and times the cycles per bus, and `PORT_SPINLOCK_STATS` in the ChibiOS
SMP port records per-core spinlock contention. Both off by default.

Retracted: `dmaChannelFreeI()` ends with a whole-block
`rp_peripheral_reset(RESETS_ALLREG_DMA)` when the last channel goes, and this
file previously suspected it was firing at transaction rate. It is not. Measured
zero over the entire session, because I2C and UART always hold channels so the
mask never reaches zero.

## `spi_fail`: the microSD bus was being left switched off (fixed)

`AP_InternalError` bit 14, 0x4000, is raised in exactly one place: the
`MSG_TIMEOUT` path in `SPIDevice::do_transfer()`, when an SPI transfer does not
complete within 20 ms plus 32 us per byte and the scheduler is not in an
expected delay. The flag is sticky, so a single event blocks arming for the
rest of that boot with `Internal errors 0x4000 l:<line>`.

Cause: `sdcard_init()` stopped SPID1 by calling ChibiOS `spiStop()` directly,
to force the following `mmcConnect()` to restart it from core0 so its DMA
interrupts landed there. The premise was wrong. `hal_mmc_spi.c` redirects
`spiStart` to `spiStartHook`, which only sets the bus speed, so nothing ever
restarted the peripheral. And because the stop bypassed
`SPIBus::stop_peripheral()`, `spi_started` stayed true, so `start_peripheral()`
early-returned from then on.

The bus was therefore left with its DMA channels freed and the peripheral in
reset while the software believed it was running. Every transfer was armed
against nothing and timed out. The filesystem layer retries on error and each
retry ran the same code, so one failure became a burst: measured at exactly 60
timeouts per `sdcard_init()`, tracking it one for one.

Fixed by stopping the bus through the SPIBus, which keeps the flag and the
hardware in step so the next `acquire_bus()` starts the peripheral again.
Verified with per-bus counters: SPI1 timeouts went from 60 per reinit to zero
across a minute in which `sdcard_init` still ran three times, and
`internal_errors` stayed 0.

Retracted: this file previously said the fault fired "once or twice per boot,
always early", and that card init at 400 kHz was the likely cause. Both wrong.
It is not boot-related at all - it scales with SD activity, because it is the
retry path that triggers it. The earlier reading came from watching an idle
board; with `LOG_DISARMED` 1 it runs continuously and the rate is far higher.

Also retracted: the suspicion that `HAL_CORE_SPI1` moving from core1 to core0
was implicated because it was the only SPI-path difference from
`rp2350-v5-dual-core-baseline-rebase`. It was a real difference but not this
fault. Keep `HAL_CORE_SPI1` 0.

### What the diagnosis cost, and what actually worked

Two wrong fixes went in first, both plausible and both useless against this:

- a bus-semaphore interlock around `sdcard_init()`, on the theory that a reinit
  was racing an in-flight transfer. There is no race; the bus is simply left
  off. The interlock is still there and is defensible on its own terms, since
  `sdcard_init()` does reconfigure a shared bus, but it fixed nothing.
- propagating DMA allocation failure out of `spi_lld_start()`, on the theory
  that channels were exhausted. `spi_start_fail_count` reads zero, so they
  never were. That change is still worth having: the allocations are guarded
  only by `osalDbgAssert`, which flight builds compile out, so a real failure
  would fall through and dereference a null channel. Same class of bug as the
  UART RX one recorded above.

What settled it was a one-shot snapshot of the hardware taken at the first
timeout, recording the driver state, both DMA channel pointers, `SSPSR`,
`SSPCR1`, `SSPDMACR`, `cs_forced`, the calling thread and - the field that gave
it away - the SPIBus `spi_started` flag. `spi_started` true with null DMA
channels and `SSPCR1` zero is only reachable if something stopped the bus
without going through `stop_peripheral()`, which points straight at the caller.
That contradiction was visible in the first snapshot and it still took two
wrong turns to act on it. Read the state, not the theory.

Per-bus counters mattered too: the failing bus was SPI1 throughout, and SPI0
never recorded a single timeout. An inference from teardown rates had pointed
at SPI0 and was simply wrong.

### A late transfer was raising it too (fixed)

Separate from the cause above, and only visible once the abort work gave a way
to tell the two cases apart. `do_transfer()` raised `spi_fail` on any
`MSG_TIMEOUT`, but the driver state at that moment says whether the transfer
was actually lost. If the DMA ISR lands between the timeout expiring and the
state being checked, the transfer completed on its own: the data is intact and
the bus needs no recovery. It was late, not failed - and raising a sticky
error that blocks arming for the rest of the boot over a transfer that
finished a few microseconds after an arbitrary deadline is a false positive.

Measured with the timeout shortened to 1500 us so both cases occur. A
spontaneous late completion now increments `spi_late_count` and leaves
`internal_errors` at 0, where the same event on the previous build latched
0x4000. Clearing `SSPCR1.SSE` over SWD to abandon a transfer for real still
raises `spi_fail`, still increments the abort count, and the bus recovers with
no cascade. `ret` stays false in both cases, so callers still retry.

This should not fire at all at the production 20 ms timeout. It is a
robustness fix, not a live bug - but it is the same shape as the fault above,
where one event poisoned a whole boot.

## Corrupt log filenames: three mechanisms ruled out, cause still open

Since around 8 August some logs on the card come back named `0000012B.IN`
rather than `00000012.BIN`: a digit short, with the dot one place to the
right.

**It is a one byte left shift of the directory sector.** A FAT short name
entry stores the name as 11 bytes with no dot, and byte 11 immediately after
it is the attribute byte, `AM_ARC` = 0x20, which is also ASCII space. Read the
entry one byte late and `00000012` + `BIN` + 0x20 becomes base `0000012B` and
extension `IN ` - exactly the reported name, and the trailing space is what
makes the extension read as two characters rather than looking mangled. A
right shift gives `0000001.2BI`, which is obviously broken and is not what is
seen. The digit being short is the confirmation: the leading `0` falls off the
front.

That much is arithmetic. Everything below is what the shift turned out not to
be.

**Not an SSP receive overrun.** The RP SPI driver never reads `SSPRIS` or
writes `SSPICR`, so `RORRIS` is a sticky raw status bit that accumulates from
boot - which means it can be read over SWD on unmodified firmware, no rebuild
needed. It is zero on both buses across every build tried, over hours of
`LOG_DISARMED` logging and across arm/disarm cycles. A per-driver software
counter added alongside it agrees. The receive FIFO also never once showed
`RFF` set across 251 samples taken while the bus was busy, so the RX side has
comfortable margin in steady state.

**Not an XIP park stalling a read.** Before the `dummytx` fix, `spi_lld_receive()`
sourced its transmit DMA from `.rodata`, so every microSD sector read fetched
its operand from XIP flash - and a directory update is a read-modify-write, so
that read happens immediately before the write that lands the name. The theory
was that a park stalls the read past its timeout. Tested by building two
images differing only in whether `dummytx` is `const`, with the SPI timeout
shortened to 1500 us so that a park-scale delay would show. Both produced one
spontaneous timeout each, at 447 s and 375 s respectively. Indistinguishable.
Refuted.

**Not the missing abort.** That hole was real and is now fixed (see the commit
and the section above), but `internal_errors` is 0 in normal operation, so the
path is not being taken.

Numbers worth keeping from the exercise:

| Measurement | Value |
|---|---|
| XIP park duration, worst seen | 3634 us |
| XIP park rate, idle | about one per 11 s |
| SPI timeout budget, 512 byte transfer | 36384 us (20000 + len * 32) |
| microSD bus duty under `LOG_DISARMED` | 6-9% |
| SPI transfers per second, both buses | about 6200 |

The park worst case is an order of magnitude inside the production timeout,
which is why none of this registers in a normal build.

**Where to look next.** The shift is not entering through the SPI hardware.
Three independent hardware-level measures - the sticky overrun flag, the
software overrun counter, and the internal error state - are all clean while
the corruption keeps happening. That points at the layers above: the
bouncebuffer copy in `SPIDevice::do_transfer()`, and what the card itself does
with a directory sector whose write was interrupted. Note the corruption
correlates with arm and disarm, which is when the directory entry is written
at all - the log file is created on arm and its size updated on close - so a
fault only has two chances per flight to express itself as a bad name, against
many thousands of data block writes. A low rate at the transport layer would
still show up this way.

**Reading the counters.** The per-driver counters and `rp2350_xip_park_count`
are plain globals, so a snapshot is `nm` for the symbol and `mdw` over SWD.
The trap is that every address moves on a rebuild: check the flashed image
matches the ELF (dump 256 bytes from `0x10020000` and compare against the
`.bin`) before believing any symbol-derived value, or you get neighbouring
words that look plausible. `internal_errors` is reached through
`hal.util->persistent_data` and reads as garbage until the `hal` pointer is
written during static init, so anything sampled in the first second of boot is
meaningless.

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

`BAT.Curr` reads a steady value whatever the vehicle is doing, including with
the motors stopped - about 7 A in the flight logs, about 30 A on the bench with
a 6S pack connected. This is not a scale or an offset problem and no parameter
value repairs it. Stop treating it as a calibration task.

The cause is now established: the ESC drives the pin and its output carries no
current information. The leakage and wiring-swap explanations are both dead.
See "Resolved" below before reading the older reasoning.

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

### Resolved: the ESC drives the pin and it has no usable gain

Case 1, settled on the bench with the debug probe attached. Measurements were
taken by reading `ChibiOS::AnalogIn::samples[0]`, the ADC DMA landing buffer,
over the OpenOCD telnet port with `mdw` while the board ran. That is a
non-halting read; a GDB attach halts both cores and wedges any SPI transfer in
flight, which then needs a `reset run` to recover. The buffer is
`ADC_DMA_BUF_DEPTH` (8) slots of `num_grp_channels` conversions, interleaved.
Note the group is **four** channels, not three: pins 0, 6 and 7 plus the MCU
temperature sensor, because `HAL_WITH_MCU_MONITORING` is 1.

**Pad leakage cannot produce the level.** The pin sits at 0.60 to 0.63 V with a
6S pack connected, which needs 7.6 uA through the 82.5k pulldown. The datasheet
allows 1 uA. An order of magnitude short, so the current arrives from outside
the chip.

Retracted: that paragraph used to end "so something low-impedance is driving
the pin", and it does not follow. The arithmetic bounds the source's *current*,
not its impedance. A 350k path to 3.3 V on the ESC side sources 7.6 uA and
holds the node at 0.63 V against 82.5k - a high-impedance source, and one a
stronger pulldown would fix. The vendor has since raised the 82.5k as too weak,
and nothing in this section refutes that. Test 6 below settles it in one
reading.

**There is no chip-wide leakage floor.** The AN1 spare pad on GPIO40 reads
0.0365 V and holds it across reboots and thermal drift, against 0.6 V on the
current pin. Note AN1's external network is not known to match this one's, so
this is suggestive rather than a controlled comparison.

**The pin really is wired to the ESC.** Spread within one 8-sample ADC burst is
1 to 2 counts with the motors stopped and 4 to 33 counts with them spinning.
That is ESC switching noise arriving on the pad, which needs a real wire. No
control was taken on AN1 with the motors spinning, so this does not separate
noise coming down the CUR wire from supply or reference noise common to every
channel.

**It still does not respond to load.** Motors spinning, props off, the level
stays at 0.61 to 0.64 V while the pack sags only 23.18 to 23.06 V. Temperature
accounts for nearly all of the motors-off to motors-on step: 51.0 to 52.0 degC
predicts 0.6115 V against 0.6129 V measured. The residual is about 17 mV,
roughly 0.85 A, the same size as the switching noise now on the pad and what
rectification of it would produce. Props-off draw is only a couple of amps, so
the bench alone cannot exclude a weak response - log62's negative correlation
against `RCOU` across a whole flight can, and does.

**The temperature coefficient is real.** Two independent measurements agree:
log62's motors-off window gives 0.5 A over 0.93 degC, and a bench reboot gives
30 mV over 2.4 degC. That is 0.54 and 0.63 A/degC, about 12 mV/degC at the pin.
AN1 did not move across the same reboot, so it is not ADC drift.

So the ESC's sense output is powered, connected, temperature-dependent and
carries no current information. Shunt not fitted, sense amp unpopulated, or an
ESC variant with no current sense - which of those cannot be settled from the
flight controller.

The idle level is not a fixed artifact. This file previously recorded 0.1456 V,
about 7.3 A; with a 23 V pack connected it sits near 0.60 V, about 30 A. Quote
the conditions with any future reading.

### The CUR/TEL wiring hypothesis (disproved)

Retracted: there is no swap. GPIO5, the FC `TEL` pin, was probed on the bench
with the pad's output driver disabled - `OD` 1, so the test can never contend
with whatever is out there - and the internal pulls toggled. It reads high with
the pull-up and low with the pull-down, so it follows a 50k pull in both
directions and nothing is driving it. An analog sense output is low-impedance
and would hold the pin against that pull.

That holds whichever way the harness is wired. If `TEL` carries a wire, the
ESC's current output is not on it. If `TEL` is unwired, a swap would put the
ESC's telemetry on GPIO47 - but GPIO47 carries a steady 0.6 V with 2 counts of
noise when the motors are stopped, which is not a UART line. The one case the
pull test cannot separate is a telemetry TX that tri-states when idle, and that
does not matter here: a tri-stated UART floats, an analog output does not.

To repeat it, from the shipped `rp2350.h`: `PADS_BANK0` is 0x40038000 with
GPIO n at +0x04+4n, `IO_BANK0` is 0x40028000 with GPIO n status at +8n and
`INFROMPAD` in bit 17. GPIO5 is unclaimed by the hwdef, so poking it disturbs
nothing. Restore the original pad register afterwards.

The pin-order reasoning below is kept because it is still the right argument to
make about any future harness.

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

1 to 4 have been run. 6 is new, is the cheapest thing on the list, and is the
one to do next. 5 remains the only test that can close the hardware question,
and it is not a flight controller question.

1. Done. **Read `RSSI_ADC` on GPIO40.** 0.0365 V against 0.6 V on the current
   pin, so there is no leakage floor and case 2 is dead.
2. Superseded. **Make the ESC transmit telemetry.** The GPIO5 pull test
   answered the swap question directly and without arming. Still worth doing
   if the ESC's telemetry behaviour is ever in doubt - and note the
   `SERVO_DSHOT_ESC` warning in the DShot section before setting it.
3. **Scope the pad.** Not needed for the swap any more, but still the fastest
   way to see what the ESC actually puts out and the only way to see the
   switching noise properly.
4. Effectively answered. **Unplug the ESC current lead and meter the pad.** The
   level needs 7.6 uA against a 1 uA leakage maximum, so the ESC is driving it.
   Metering with the lead off would confirm it directly and is still the
   cleanest single check if the ESC is ever off the airframe.
5. **Confirm what the ESC actually outputs on that pin**, and its mV/A. The
   only test that can close the hardware question. `BATT_AMP_PERVLT` 50 assumes
   20 mV/A. If the output can exceed 3.3 V, see the protection note below.
6. **Parallel a known resistor onto the pin and read the level again.** This
   measures the source impedance directly, which is the thing every argument
   above was inferring. Tack a resistor from the pin to ground and re-read
   `ChibiOS::AnalogIn::samples[0]` over SWD as in the section above - no
   reflash, no arming. With 10k added:

   | source | 0.63 V becomes |
   |---|---|
   | high-Z, Rs about 350k | 0.082 V, down 7.7x |
   | driven, Rs under 1k | 0.624 V, down 1% |

   A factor of seven against one percent. Two readings with two different
   resistors solve the Thevenin pair outright and give Rs and the open-circuit
   voltage rather than another inference. Take it twice, motors stopped and at
   a steady throttle: an output that can source but not sink looks
   low-impedance under load and high-impedance at rest, and that case is the
   one where strengthening the pulldown would cost gain on the airframe that
   currently works. Do it in the same bench session as the drift test in open
   item 1 - same rig, and between them they separate source impedance,
   voltage coupling and thermal drift.

### One thing worth asking the designers

Not a defect, but a question for the next revision. The 120R series has no
clamp diode, so the circuit assumes the ESC never drives that pin above 3.3 V.
If an ESC drives it from a 5 V rail, the pad's ESD diode has to absorb
(5 - 3.6)/120, about 12 mA, which is above what those clamps are typically
rated to carry continuously. That is a robustness question for the next spin.
It is no longer part of the current-sense diagnosis - that pad is not damaged,
since it is being driven and AN1 reads a clean 0.0365 V - so raise it on its
own merits once the ESC's output is known.

### log96: on the EVO 5 the pin does respond to throttle

Everything above was measured on the earlier airframe. On the iFlight Nazgul
Evoque F5 the pin behaves differently, and it is a different ESC, so the "no
usable gain" conclusion must not be carried across without re-measuring.

**Detrended, the reading tracks throttle almost perfectly.** Correlating levels
reproduces the old result - r = 0.219 against `MOTB.ThrOut` across the armed
window, which reads as no coupling at all. Correlating 1 s *differences*, which
removes the slow drift this section already warns about, gives **r = 0.925** and
a slope of 9.6 A per unit throttle at `BATT_AMP_PERVLT` 10. The fast signal is
real and it is large. Note what that means for the log62 table above: those
correlations were taken on levels, which is the same trap, and the constant
throttle windows are what carried that conclusion rather than the r values.

**The gain is real and standard.** Regressing the pin against the RPM model of
the next section, on 0.5 s differences, gives r = 0.967 and an incremental gain
of **68.4 A per pin-volt** - 14.6 mV/A, an entirely ordinary ESC shunt scale.
`BATT_AMP_PERVLT` 10 is about 7x too low, which is most of the under-reading on
its own. Set it to 68: the shape and the peaks become correct even while the DC
level is not, and it costs nothing.

**The zero is what is broken, and badly.** With the motors stopped
the pin reads 5.33 A before arming at 25.44 V, and 1.90 A eight seconds after
landing at 22.82 V. True current at both points is a few hundred milliamps. The
in-flight mean is 2.71 A - the reading under load is *half* what it reads at
rest with the motors off. At the same throttle, 0.105 to 0.112, the reading was
5.02 A at t=10 s and 1.90 A at t=250 s: 3.1 A of drift at constant load, still
moving at the end. At the correct gain of 68 that zero wanders 0.343 V, about
**23.5 A across one flight**, against a mean current near 9.5 A.

Held-out tests settle whether any fixed calibration survives. Fit gain and
offset on the first half of the flight and score the second: R^2 -1.21, worse
than predicting the mean, forecasting 17.6 A where the reference says 10.8.
Fit 200-230 s and test 230-258 s - thirty seconds later, fully warm - and it is
still R^2 -0.20. Adding pack voltage as a second term gets the whole-flight fit
to R^2 0.515 and the held-out fit to nothing. So no constant `BATT_AMP_PERVLT`
and `BATT_AMP_OFFSET` pair calibrates this pin for absolute current.

What it *is* good for is anything that needs changes rather than levels: peaks,
transients, per-second deltas, "that manoeuvre pulled a lot". Those are sound
at r = 0.967. Absolute amps, mAh, `RemPct` and any mAh-based failsafe are not.

Whether that drift is thermal, voltage-coupled or a settling transient is not
established. A three-term fit over the flight gives
`I = 0.634*V + 7.95*Thr - 13.34` at R^2 0.571, but inside the 10-48 s hover
window, with throttle pinned at 0.105, the implied voltage slope is 4.8 A/V
against the global fit's 0.63. Those are inconsistent, so the confound is not
resolved, and it decides whether this is recoverable. If the drift is
voltage-coupled a Lua script can subtract it using the pack voltage the FC
already measures, and the real sensor gives real current. If it is thermal it
cannot: nothing on the flight controller sees ESC die temperature, and
`ESC.Temp` is 0 on this port.

It is a bench question, not a flight question, and a cheap one. Props off,
disarmed, hold the pack voltage steady from a bench supply and log the pad
against time for ten minutes, then step the voltage and watch the pad. That
separates the two terms, which a flight never can - pack voltage and elapsed
time fall together in every log.

**What the true current is.** Two independent estimates agree:

- Pack accounting. Resting 25.42 V before arming and 22.82 V after landing on a
  6S 1500 mAh pack is roughly 40-47% of a charge over 253 s armed, so 8.5 to
  10.0 A mean.
- Momentum theory. About 700 g AUW on four 5 inch props, figure of merit 0.55
  and drivetrain efficiency 0.78, gives about 4.8 A at hover.

Reported mean was 2.71 A, so the reading is low by roughly 3.5x - and most of
what it does report is the offset.

### log97: the gain is not stable between flights either

log96 was flown at `BATT_AMP_PERVLT` 10 and log97 at 68.4, so the two can be
compared in pin volts. The comparison has to avoid the circularity that any
assumed true current introduces: calibrate a reference to a flight mean and the
fitted gain is proportional to whatever mean was assumed, so two flights will
always "disagree" by exactly the ratio of two guesses.

The reference-free test is the slope of pin volts against `SUM (RPM/1000)^3`.
The airframe's RPM-to-current physics is the same in both flights, so a stable
sensor must give the same slope, and no current estimate enters:

| | log96 | log97 |
|-------------------------------|-----------|-----------|
| d(pin V) / d(sum kRPM^3) | 1.236e-05 | 9.132e-06 |
| corr of those differences | 0.967 | 0.954 |

A ratio of 0.74 - **26% of gain difference between two flights on the same
hardware a day apart**, with nothing assumed. So `BATT_AMP_PERVLT` 68.4, fitted
to log96, is already about 25% low for log97, which implies about 92.

The level rows are worse. Binned by `SUM (RPM/1000)^3`, log96's pin reads
0.3011 V at 4000-6000 and 0.2385 V at 8000-12000 - *lower* at higher RPM, which
is backwards for a current sensor - while log97 reads 0.2096, 0.2441, 0.3564
across the same bins, monotonic and the right way up. Same pin, same airframe,
opposite behaviour.

Conclusion, now on two flights: the pin tracks the *shape* of current well
(r 0.95-0.97 on differences) but neither its zero nor its gain is stable enough
to calibrate. Setting `BATT_AMP_PERVLT` was worth trying and the answer is that
it does not hold. Treat the pin as a relative indicator only.

### The zero is thermal, and it tracks the flight controller's die temperature

This file used to say the voltage-against-thermal question could not be settled
from a flight, because pack voltage and elapsed time fall together in every
log. True of voltage and time. Not true of voltage and *board temperature*, and
log96 is the counter-example: the board was at 45.4 degC when the motors
started, prop wash cooled it to 31.8 degC by disarm, and it soaked back to
34.6 degC afterwards. Temperature reverses twice while the pack only falls and
then plateaus.

Motors-off samples only, both flights pooled, n = 316 over 32.3-45.4 degC:

| model | R2 | slope |
|--------------------|-------|----------------------------------------|
| pin ~ `MCU.MTemp` | 0.996 | +28.4 mV/degC |
| pin ~ pack voltage | 0.842 | +117 mV/V |
| pin ~ both | 0.996 | the V term flips negative and adds nothing |

`corr(MTemp, pack V)` is 0.928 across those samples, so the R2 gap is
suggestive rather than conclusive on its own. What carries it is a matched
pair: the two flights' post-disarm windows overlap in temperature and sit
0.84 V apart in pack voltage.

| | MCU | pack | pin |
|-------|---------|----------|----------|
| log96 | 33.92 C | 22.824 V | 200.4 mV |
| log97 | 33.93 C | 23.663 V | 199.9 mV |

0.5 mV of difference, against the 98 mV a voltage-coupled offset predicts.

**And the temperature that predicts it is on the flight controller.** Open item
1 assumed thermal drift would be uncorrectable because nothing here sees ESC die
temperature. `MCU.MTemp` is not standing in for the ESC - through the flight the
ESC is heating while the board cools in prop wash, and the pin follows the
board. So a correction is available from something the FC already logs.

### The standing offset is on the board, not in the ESC

Three things put it there, and together they make the vendor's pulldown
question the leading explanation rather than a dead end.

**It is there with the motors stopped.** 530 mV before arming in log96 and
496 mV in log97, which is 67% and 94% of the entire in-flight swing. Rectified
switching noise cannot produce a level that exists when nothing is switching,
so that mechanism is dead for the offset whatever it does to the gain.

**It needs more current than the pad is allowed to leak.** 2.0 uA at 32.3 degC
rising to 6.5 uA at 45.4 degC through the 82.5k, against a 1 uA pad maximum -
the same few microamps the earlier airframe showed.

**The same offset appears on two different ESCs.** 0.50-0.53 V here against
0.60-0.63 V on the earlier airframe, a different ESC on a different aircraft.
What those two share is this board.

Read with the thermal result that is a few microamps of temperature-dependent
leakage sourced into a node whose only DC return is the 82.5k, which is the
failure a stronger pulldown fixes. At 10k the same current gives 20-65 mV where
82.5k gives 166-538 mV, and a low-impedance ESC output is untouched.

Two things not established. The tempco does not match between airframes -
28.4 mV/degC here against about 12.5 mV/degC on the bench earlier, same sign
and a factor of two apart. And the shape is open: if the rise is exponential
the doubling constant is 7.9 degC, which is the junction-leakage band, but over
a 13 degC span linear and exponential fit equally well at R2 0.9958. A wider
sweep settles it and is worth doing while test 6 is set up.

Worth checking before theorising further: whether RP2350 erratum E9, the
documented GPIO input leakage whose published workaround is a stronger external
pull-down, applies to a pad in analog mode with IE cleared. Read the erratum
text rather than trusting this sentence - it is a lead, not a finding.

### log97: sag compensation makes the failsafe worse, not better

`BATT_FS_VOLTSRC` was changed to 1 for log97. Do not leave it there while the
current reading is broken.

`AP_BattMonitor_Backend.cpp:133` computes
`voltage_resting_estimate = voltage + current_amps * resistance`, and
`:249` selects it when `BATT_FS_VOLTSRC` is 1. The current carries a +34 A
offset at rest falling to +15 A by landing, and `BAT.Res` learned 0.044 ohm, so
the estimate is inflated by roughly **0.7 to 1.5 V** for the whole flight. In
log97 `BAT.VoltR` averaged 24.27 V against a raw 23.73 V, and at the worst
moment read 23.82 V while raw was 22.56 V.

`BATT_FS_LOW_ACT` is 0, so nothing happens on the low threshold. But
`BATT_CRT_VOLT` is 19.8 with `BATT_FS_CRT_ACT` 1, Land - and that now fires
roughly 0.7 V late, at about 19.1 V raw, which is 3.19 V/cell on 6S. The
resistance estimate itself is fine, since it is fitted to a slope and a constant
offset cancels; it is the estimate's use of the absolute current that does the
damage.

Set `BATT_FS_VOLTSRC` back to 0 until current is trustworthy. Sag compensation
is the right setting *once* it is, and the wrong one now.

### Synthesising current from thrust

Current can be computed rather than measured, and log96 calibrates it.

`BATT_MONITOR` 25, "Synthetic Current and Analog Voltage", is **not** the route.
`AP_BattMonitor_Synthetic_Current::read()` uses
`SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)`, and nothing in
ArduCopter ever calls `set_output_scaled()` for `k_throttle` - only
`AP_MotorsTailsitter` does. `get_output_scaled()` returns the stored
`output_scaled`, which stays 0 (`SRV_Channel_aux.cpp:625`), so on a multirotor
that backend reports a flat `BATT_AMP_OFFSET`. The parameter documentation does
not say it is fixed-wing only; check it before suggesting it to anyone.

The route that works is `BATT_MONITOR` 29, Scripting, with a Lua script feeding
`battery:handle_scripting()`. Every binding needed exists:
`esc_telem:get_rpm(i)` for the four motor RPMs, and `BattMonitorScript_State`
with `current_amps`,
`consumed_mah`, `voltage` and `capacity_remaining_pct`. There is no Lua binding
for `AP_Motors::get_throttle()`, so RPM is the available input - which is the
better one anyway.

**The model.** Rotor power goes as thrust^1.5 and thrust goes as RPM^2, so
current goes as the sum of RPM^3 across the motors. Calibrated against log96's
flight mean:

    I = 0.5 + 9.0e-4 * SUM_i (RPM_i / 1000)^3     amps

The 0.5 A is the flight controller, VTX and receiver. The constant follows from
the pack accounting above and nothing else.

**It self-validates.** Fitted only to the flight mean, the model independently
predicts 4.95 A at hover against the 4.8 A momentum theory gives from the
airframe mass. Nothing tied those two together. It also correlates 0.9944 with
the throttle form `0.5 + 129 * ThrOut^1.5`, so the choice between them is about
which input Lua can read, not fidelity - but RPM wins on merit too, because it
sees per-motor asymmetry during a manoeuvre and throttle does not. Peak over the
flight was 37 A on the RPM model against 25 A from throttle alone, and that gap
is the asymmetry.

**The weak term is the pack accounting**, since mean current scales linearly
with the assumed state-of-charge swing: 7.5 A at 35%, 10.0 A at 47%. Settle it
with a charger rather than another log - fly a pack, note what the charger puts
back in, and scale the constant by `charger_mah / script_reported_mah`. One
flight fixes it to a few percent.

This is the fallback, not the first move. The pin's gain is sound and only its
zero is not, so try `BATT_AMP_PERVLT` 68 first and see what the bench says about
the drift. Reach for the model if the drift turns out to be thermal.

Note also that no failsafe depends on current here: `BATT_LOW_MAH` and
`BATT_CRT_MAH` are 0 and `BATT_FS_VOLTSRC` is 0, so the wrong reading costs
`CurrTot` and `RemPct` and nothing else. There is no safety reason to drop to
`BATT_MONITOR` 3 and give up the signal.

## Open items

Ordered by what is being worked on, not by severity. Items that block a flight
are marked. Every claim behind these is in the section named.

### 1. Current sensing (in progress)

The pin's gain is real and standard - 68.4 A per pin-volt, 14.6 mV/A - and it
tracks changes in current at r = 0.967. Its zero drifts about 23.5 A over a
flight, which is what makes absolute amps and mAh unusable while leaving peaks
and transients sound. See "log96: on the EVO 5 the pin does respond to
throttle". The plan, in order:

Note first that nothing unsafe depends on this. `BATT_LOW_MAH` and
`BATT_CRT_MAH` are both 0, so no mAh failsafe is armed, and `BATT_FS_VOLTSRC` is
0 so the failsafes that are armed run on voltage. The wrong current corrupts
`CurrTot` and `RemPct` and nothing else. Do not switch to `BATT_MONITOR` 3 to
"make it safe" - that throws away a signal which is 97% right about changes.

- [x] Set `BATT_AMP_PERVLT` 68. Done for log97, and the answer is that it does
      not hold: the reference-free gain moved 26% between the two flights, so
      68.4 is already about 25% low. Keep it - the shape is still useful - but
      stop expecting a fixed value to be right. eRPM is not the cause: it is
      verified to 1.5% between those flights, worth 4.5% of gain against the 26%
      seen.
- [ ] Set `BATT_FS_VOLTSRC` back to 0. At 1 the failsafe uses
      `voltage + current * resistance`, and a +15 to +34 A current offset with
      `BAT.Res` 0.044 inflates it by 0.7-1.5 V, so the `BATT_CRT_VOLT` 19.8 Land
      fires about 0.7 V late. Sag compensation is right once current is, and
      wrong now.
- [x] Decide whether the drift is voltage-coupled or thermal. Answered from
      log96/97 without a bench run: it is thermal, and it tracks `MCU.MTemp` at
      R2 0.996 and +28.4 mV/degC. log96 breaks the confound this file said no
      flight could break, because prop wash cools the board through the flight
      while the pack falls. See "The zero is thermal".
- [ ] Test 6, the parallel resistor, now with a temperature sweep rather than at
      one temperature. The offset is 2.0-6.5 uA into the 82.5k with the motors
      stopped, it appears on two different ESCs, and it follows the board rather
      than the ESC - so measure the source impedance and find out whether the
      leakage is on the board. Do it with the ESC lead off as well: if the level
      and its tempco survive that, the ESC is not involved at all.
- [ ] Correct the zero against `MCU.MTemp` in Lua rather than falling back to
      the RPM model. Thermal was assumed uncorrectable because nothing sees ESC
      die temperature; the predictor turns out to be the FC's own. Keep the RPM
      model (`BATT_MONITOR` 29, `esc_telem:get_rpm()`,
      `I = 0.5 + 9.0e-4 * SUM (RPM/1000)^3`, through
      `battery:handle_scripting()`) as the fallback if the correction does not
      hold across flights.
- [ ] Either way, calibrate the absolute scale against a charger: fly a pack,
      note the mAh put back in, scale by `charger_mah / reported_mah`. That is
      the only weak term in the model and one flight settles it.
- [ ] Ask the ESC vendor whether the shunt and sense amp are fitted on this
      variant. Still the cheapest way to close the hardware question.

### 2. Battery failsafe thresholds (blocks flight)

`BATT_LOW_VOLT` is 21.6 and `BATT_CRT_VOLT` 21.0, unchanged for three flights.
log96 reached 21.13 V at 160.4 s - below the low threshold and 0.13 V above
critical. It survived only because the dip lasted 0.2 s against a
`BATT_LOW_TIMER` of 10 s. Set 21.0 and 19.8 for 6S. Deferred three times now;
the margin has gone from 0.19 V to 0.13 V.

### 3. DCM roll/pitch divergence

89 deg after log96, worse each flight, and now known to start **before the
motors spin**. Reproduces on the bench disarmed, so instrument `GA_e`,
`_ra_deltat` and the GPS velocity term there rather than flying for it. The
`AHRS_GPS_GAIN` 0 flight is still the cheapest discriminator if a flight is
wanted. See the DCM section - and note the risk framing there has been
corrected.

### 4. core0 load now has a measured price in log bandwidth

Answered by log97, and the answer is CPU. Drain capacity is 141-147 KB/s at
`PM.Load` 77% and 195-198 KB/s at 65% - **about 40% of microSD throughput for
12 points of core0 load**, same board, same card, same file. Loiter's own
logging is not the cause: `PSC*` is 21 messages/s, about 1.1 KB/s.

This promotes the core0 flash work from a tidy-up to the main lever.

- [ ] `LOG_FILE_RATEMAX` 67, or clear `MASK_LOG_ATTITUDE_FAST`, as the immediate
      way to stop losing messages while the load work is done.
- [ ] Attack core0's flash share - the veneers in `PROFILING.md`. Now worth
      roughly 3 KB/s of log bandwidth per point of load recovered.
- [x] Check whether `_dropped` counts distinct messages or re-offers. Answered:
      in flight it counts distinct rejected messages, and nothing re-offers.
      Only the boot FMT phase counts retries. So the offered rate genuinely
      rose in Loiter, and the "`PSC*` is 21 messages/s" dismissal is circular -
      that count is post-drop. See the mechanism section.
- [ ] Measure what position control actually offers, which the logged rate
      cannot tell you. Needs a counter at the offer site, or a bench run with
      LOG_BITMASK varied.
- [ ] Do not shrink io_size. The sweep has throughput falling with it, and
      capacity is the term that is short.
- [x] Confirm which value `_writebuf_chunk` took. Answered: 4096, on every
      boot rather than only after a mount retry, because every backend is
      constructed before any `Init()` mounts the card. Fixed, measured, and
      **reverted** - the write size is set by the 4096 byte FAT cluster, not by
      the chunk, so raising it changed nothing. See "The write size is capped by
      the FAT cluster size".
- [ ] Optional, and worth less than the core0 work: reformat the card with a
      larger cluster and re-land the chunk change with it. Neither alone does
      anything. Order 10% by the measured per-call share, against 40% for the
      12 points of core0 load log97 priced. Destroys the card contents.

### 5. PIO UART statistics

`PIOUART` implements none of `get_total_tx_bytes`, `get_total_rx_bytes` or
`get_total_dropped_rx_bytes`, so SERIAL3 and SERIAL4 never appear in the `UART`
log message and `RxDp` is unreadable on the RC port. Three counters.

### 6. Bidirectional DShot error rate

`ESC.Err` is a hardcoded 0 on this port, confirmed again in log96 across all
four instances. Count the `read_telemetry()` false returns in the RP2350 branch.
The notch is being driven by telemetry whose frame loss nobody can see.

### 7. Half duplex receive, and the VTX

Every transmit path is confirmed byte for byte; nothing has ever replied.
Next step is unchanged - the same VTX on an ST flight controller with a
known-good half-duplex UART. Until then `SERIAL4_PROTOCOL` stays -1 and
`VTX_ENABLE` 0, which is how log96 flew.

### 8. Standing checks before each flight

- [ ] `git diff` on `hwdef.dat` empty, so `AP_RP2350_PC_SAMPLER_ENABLED` and
      `AP_RP2350_DEBUG_REPORT_ENABLED` are both 0.
- [ ] `rp2350_xip_park_count` read over SWD before arming and after landing.
      Still only ever read disarmed; whether it stops at arm is still open.
- [ ] `SPID0/1.rxoverruns`, `SPID0/1.aborts` and `spi_late_count` after the
      flight. All four should be zero and none has yet fired outside deliberate
      injection.
- [ ] Reboot shortly before arming, until the 71 minute wrap has been soaked.
- [ ] Expect a re-arm delay after an aggressive flight while DCM decays below
      10 deg. log96 was still 89 deg out at the end, so this is now minutes.

### 9. Not yet flown

RTL, Auto and the GPS failsafe paths remain untested on this board. Loiter and
acro are flown. Fly them deliberately before relying on one to recover the
vehicle.

### 10. Longer-lived

- Fresh accel calibration - log96 reads |g| 2% low at 24.8 degC against a
  28 degC cal temperature.
- Finish AUTOTUNE. Roll got most of the way in log62 without saving; pitch and
  yaw are untouched.
- Attack core0's flash share, starting with the veneers - see `PROFILING.md`.
  Core1 is done.
- Find the corrupt log filename cause. Three transport-level mechanisms are
  ruled out, so start above the SPI layer: the bouncebuffer copy in
  `SPIDevice::do_transfer()`, and what the card does with a directory sector
  whose write was interrupted.
- Establish whether flash page programs succeed first time or only on the retry
  after a failed verify. One counter on the `memcmp` mismatch answers it.
- Bring up SERIAL1 and SERIAL4. SERIAL2 and SERIAL3 are confirmed.
- Re-check the QMI flash timing if this revision fits a different flash part.
- For the next board spin: route microSD DAT1/DAT2 contiguous with DAT0, plus
  pull-ups on DAT0-3 and CMD. The only route to SDIO-class throughput on an
  RP2350, which has no SD host controller.
