# ArduPilot under Renode

Runs unmodified ArduPilot STM32 firmware in the [Renode](https://renode.io)
emulator. STM32F405 and STM32H743 boards are assembled from their normal
`hwdef.dat` and `hwdef-bl.dat`, following the target-generation pattern used by
the AM32 Renode harness. This is a decision log as much as a README.

## Status

`run.py` invokes ArduPilot's production ChibiOS hwdef compiler on every run and
generates the board REPL and RESC below `build/<board>/renode/generated/` in
under a second. There are no checked-in KakuteF4 or BlitzWingH743 Renode board
descriptions. The application and bootloader hwdefs remain authoritative for
the MCU, flash offset, serial order, buses, sensors, chip selects, DMA, system
timer, and SDMMC presence. The bootloader is not emulated yet; compiling its
hwdef makes its MCU and load offset available now rather than introducing a
second copy of that information.

`Tools/renode/run.py --list` shows the F405/H743 hwdef targets accepted by the
generator. Acceptance means the board configuration can be translated; actual
boot progress still depends on whether the shared MCU base and sensor models
cover that board's peripherals.

A stock `build/KakuteF4/bin/arducopter` ELF boots to the **main vehicle
loop**: parameter load from emulated flash storage, BMP280 barometer
calibration (100647.7 Pa from the model's datasheet-example values),
ICM20689 probe and gyro calibration, RCOut init, **EKF3 active with tilt
alignment complete**, and prearm checks running with the expected
fresh-board residual ("Check frame class and type"). MAVLink works over
the USART6 pty: heartbeat, statustexts, parameter reads, and the
vehicle's own save-notify broadcasts.

A stock CubeOrange ArduCopter image also boots and emits MAVLink. Its generated
platform includes the FMU's USART6 IOMCU peer, both MS5611 barometers, the
ICM20602, ICM20948, and ICM20649 IMUs, and SPI FRAM. The IOMCU firmware CRC and
the exact IMU WHOAMI values come from the compiled hwdef/ROMFS output. This is
important for CubeOrange because the `icm20948` SPI name is validated as an
ICM20649. No CubeOrange-specific REPL, RESC, or parameter file is needed.

The parameter-set round trip is verified end to end: MAV_SYSID set over
MAVLink, handled, saved to the emulated flash, and the save-notify ACK
received back (note the parameter is MAV_SYSID on master, not the old
SYSID_THISMAV - the firmware answers a set of an unknown name with
PARAM_ERROR, which a checker watching only PARAM_VALUE will never see).
Serial transport is a TCP socket terminal on port 5762: an earlier pty
terminal went silently deaf on the inbound side after sustained
bidirectional traffic (bytes reached neither the UART model's fifo nor
its DMA, while outbound kept flowing - diagnosed by injecting bytes at
the model, which traversed fine). What looked like TX starvation from
outside was that inbound deafness all along.

On the way up, with **stock** Renode 1.16.1 models: ChibiOS clock init
(RCC + PWR VOSRDY + FLASH ACR), the TIM4 tickless system tick, thread
switching, `Shared_DMA::init`, `AnalogIn::init` (ADC absent, fails
soft), `Scheduler::init`, `AP_Vehicle::setup()`, parameter storage
genuinely erasing and programming flash sectors 2/3 through the stock
`STM32F4_FlashController`, and `Copter::init_ardupilot()`.

Two small custom pieces made that possible; both compile at Renode runtime
(`include @foo.cs` in the resc), no Renode fork:

- `peripherals/common/AP_NVIC_RettobaseFix.cs` — **the** ChibiOS-on-Renode
  blocker. Renode's NVIC (≤1.16.1 and current master) leaves ICSR.RETTOBASE
  (bit 11) a tag that reads 0. ChibiOS `_port_irq_epilogue()` requires
  RETTOBASE=1 before arranging a preemptive context switch on exception
  return, so every interrupt-driven thread wakeup is silently dropped; the
  first casualty is the 1.5ms `chThdSleep` in `usb_initialise()`, which never
  wakes while the emulation looks healthy (CPU idles, virtual time runs).
  Diagnosed by hooking `chSchReadyI` (fired) vs `SVC_Handler` (never fired).
  The fix installs an after-read hook on ICSR via the public register API
  that ORs in bit 11 while at most one exception is active (the NVIC's
  private `activeIRQs` stack, read once by reflection). Worth upstreaming.
- `peripherals/common/AP_DWT.cs` — CYCCNT following virtual time at the CPU
  clock. Renode leaves the DWT unimplemented, so `chSysPolledDelayX()` spins
  forever on a counter that reads 0. First hit in the ChibiOS OTG driver's
  PHY delays inside `usbStart()`.

Plus `peripherals/stm32/AP_STM32_IWDG.cs`, the AM32 IWDG stub verbatim
(accepts kicks, never fires — the stock model really resets, which turns
every pause-at-a-breakpoint into a reboot).

## Files

- `gen_board.py` — runs the same `ChibiOSHWDef` compiler used by
  `./waf configure`, then translates its expanded configuration and generated
  `hwdef.h` DMA assignments into a board overlay and launch script.
- `platforms/stm32f405_base.repl` — vendored from Renode 1.16.1
  `stm32f4.repl`, edited for the F405 as ArduPilot uses it (real 128K SRAM +
  64K CCM, usart6, bus-correct timer frequencies, IWDG stub, RETTOBASE fix,
  DWT, narrow OTG tags, no network SVD fetch). Edit rationale in the file.
- `scripts/ardupilot_f405.resc` — common boot script: loads the custom .cs,
  `VectorTableOffset 0x08010000` **plus a `macro reset` re-asserting it**
  (Renode restores VTOR=0 on NVIC_SystemReset; nothing is mapped there),
  loads persistent storage sectors 2/3 (fresh STM32 flash must read 0xFF),
  `PerformanceInMips 125`, 1ms global quantum.
- `data/STM32F405.svd.gz` — vendored so headless CI never touches the network.

Generated board descriptions remain below `build/<board>/renode/`. Persistent
emulated media instead defaults to `renode/<board>/` at the repository root;
see Running below.

## Renode model bugs found (beyond RETTOBASE and the DWT)

- **`STM32_Timer` never fires a compare whose CCR value is 0.** TIM4 is the
  ChibiOS tickless system timer; about one alarm in 65536 lands exactly on
  the 16-bit wrap, ChibiOS writes CCR1=0, the alarm never fires, and every
  thread sleeps forever while virtual time keeps running - the emulation
  looks alive (CPU idles, timers count) but MAVLink, I2C, everything stops.
  Bites minutes into any run. Diagnosed from the frozen state (CCR1=0,
  DIER=CC1IE, CC1IF never set), proven with a synthetic register test, and
  double-proven by resurrecting a frozen machine live (writing CCR1=1 from
  the monitor woke the whole system). Fixed with one line in the board resc:
  a `SetHookBeforePeripheralWrite` on the tick timer nudging value 0 to 1.
- **Stock `STM32SPI` accepts only one child and never calls
  `FinishTransmission`**, while real boards put several devices on one bus.
  Generated platforms therefore put an active-low `AP_SPIMultiplexer` between
  each SPI controller and its hwdef devices. Chip-select GPIOs select a mux
  address, and CS deassertion frames the child transaction. When hwdef lists
  alternative sensor drivers at the same physical bus/CS location, only the
  first declaration is instantiated.
- The stock `STM32F4_I2C` cannot run ChibiOS I2Cv1 at all (no DMA request
  output; SR2.BUSY never set so the EV5/EV6/EV8_2 decodes fail; ADDR
  cleared on any SR2 read instead of the SR1-then-SR2 sequence, and the
  ChibiOS ISR reads SR2 *first*). `AP_STM32F4_I2C.cs` is a fork with those
  fixed and an RxDmaRequest pump keyed off the CR2.LAST write, which
  ChibiOS conveniently performs right after enabling the RX stream.
- Stock `STM32DMA` keeps a stream's internal memory offset until NDTR
  reaches zero, corrupting ArduPilot's partial-harvest UART RX pattern
  (IDLE interrupt, disable, rewrite NDTR, re-enable). `AP_STM32DMA_Fixup.cs`
  resets the offset on NDTR writes, which is what real hardware does.

## Bring-up traps found so far

- Monitor `echo` needs its argument quoted: `echo "=== x ==="`. Unquoted
  `===` is a syntax error that aborts the script *after* the emulation ran,
  which presents as a hang (renode sits at the prompt, `quit` never runs).
- A blanket `Tag <OTG_FS range> 0x80000000` makes GINTSTS read
  WKUPINT-pending and the OTG ISR storms. Tag only GRSTCTL with bit 31
  (AHBIDL) set; everything else in the block must read 0.
- The stock `STM32_Timer` SR is fine for ChibiOS's `SR = ~sr` clear idiom on
  its *defined* flags; the garbage that appears in undefined bits (5,7,8,
  13-15) is cosmetic — ChibiOS masks SR against DIER.
- 8s of virtual time in the config_error loop costs ~3.5min wall
  (~27x realtime) at quantum 1ms / 125 MIPS. The idle-heavy early boot is
  far faster (WFI sleeps skip time).

## Performance notes (perf-profiled, AM32-harness methodology)

The measurements in this section use unthrottled execution; pass
`--unthrottled` when reproducing them.

Profile of the main-loop phase (perf, JIT symbols via
DOTNET_PerfMapEnabled=1 DOTNET_EnableWriteXorExecute=0): the guest CPU
emulation (tlib) is ~1% - identical to the AM32 finding that the
expensive part is Renode's managed machinery, not the ARM emulation.
The emulation thread's top cost was Misc.IndexOf + MulticastDelegate/
Delegate.Equals at ~49%: BaseClockSource.GetClockEntry looks up clock
entries *by delegate* with a linear scan, and ArduPilot reads TIM4's
counter (micros()) ~213k times per simulated second, paying the scan
every read. Each STM32_Timer contributes five LimitTimer entries to
that list whether used or not, so trimming unused timers from the
platform bought 33.6x -> 25.7x steady and 325s -> 216s boot wall time.

The firmware only executes ~25M guest instructions per simulated second
(~20% of the 125 MIPS budget - ChibiOS WFI-idles the rest), so the cost
is per-event/per-access framework overhead, not guest code. Global
quantum 1ms vs 10ms is neutral (matches AM32). Silencing log output is
also neutral: log entries cost at the producer regardless of level, and
the logging thread busy-spins its dequeue loop on its own core (~29% of
process samples, but not on the emulation thread's critical path).

Those levers grew into a six-patch stack against renode v1.16.1,
maintained on the renode `pr-arudpilot-am32-perf` performance branch
shared by the ArduPilot and AM32 harnesses - install renode from that
branch (its `perf_patches/apply.sh` applies the stack idempotently,
and `perf_lessons.md` carries the combined findings and measurement
discipline). The stack:

- 0001: BaseClockSource clock-entry lookup via a lazily rebuilt,
  self-healing dictionary instead of a delegate-equality linear scan,
  plus integer arithmetic in the tick advance handlers
- 0002: AM32-workload clock and bus hot paths
- 0003: the console reader's EOF busy-spin
- 0004: bus access cache and the advance path
- 0005: per-access locks and allocations fast paths
- 0006: clock entries updated in place

To build: clone renode at the performance branch, run
`git submodule update --init --recursive`, `./perf_patches/apply.sh`,
then ./build.sh --net with a .NET SDK on PATH. The branch defaults the build to net10.0 -
the newer runtime is worth ~1.35x on this workload - so prefer the
.NET 10 SDK; `-F net8.0` still works on hosts that only have 8.
Delete stale obj/bin directories when switching SDKs. Pin the result
with the gitignored `Tools/renode/renode` symlink.

Measured result stack on the KakuteF4 boot benchmark: 33.6x realtime
(stock portable, net8) -> 25.7x (platform trim) -> 8.55x (patches
0001 only, net8) -> **5.1x realtime, boot-to-main-loop 44s wall**
(full six-patch stack on net10.0). Simulation results stay
bit-identical across the stack.

A documented dead end: replacing TIM4 with a model that computes CNT
arithmetically from elapsed virtual time (making micros() reads free)
made everything ~3x *slower*: the machine-level elapsed time only
advances at quantum granularity for the running CPU, so every
microsecond-precision wait in the firmware turned into a quantum-long
busy spin that kept the CPU from ever idling. The CPU's
executed-instruction count and its TimeHandle.TotalElapsedTime were
both tried as replacements and also fail (all three primitives are
quantum-granular or lock-heavy from a peripheral's point of view; each
variant measured ~3x slower than the stock timer with the patches). A
cheap intra-quantum virtual-time primitive would make this the single
biggest remaining win - the clock-source machinery behind the
~213k/sim-s counter reads is still roughly a third of the emulation
thread.

Remaining known costs after the patches: BaseClockSource.Update's
per-entry sweeps, the SystemBus per-access fixed cost, the logging
thread's busy-spin dequeue (its own core, off the critical path), and
tb_invalidate_phys_page_range on flash param writes (storage lives in
the same MappedMemory the code executes from).

## Running

```sh
./waf configure --board KakuteF4 && ./waf copter
Tools/renode/run.py KakuteF4
```

That drops into the interactive Renode monitor with the firmware
running and the first hardware UART in `SERIAL_ORDER` served on
tcp:localhost:5762
(`mavproxy.py --master tcp:localhost:5762`; the socket terminal serves
its first client only - restart to reconnect). `--port N` gives a
telnet monitor instead of the console, `--exec` appends monitor
commands, `--serial N` selects another `SERIAL_ORDER` entry, `--elf` overrides
the firmware, and renode itself is found from `--renode`, `$RENODE` or PATH.
The selected firmware UART must still have the desired ArduPilot serial
protocol configured; exposing a UART does not alter firmware parameters.

State survives both firmware resets and separate `run.py` invocations. By
default it is stored in `renode/<board>/`; use `--state-dir DIR` to select an
exact alternative directory. Depending on the hwdef, this contains:

- `sdcard.img`: a newly created 256 MiB FAT32 microSD image, attached to
  Renode in persistent mode.
- `storage.img`: the two STM32 flash sectors selected by
  `STORAGE_FLASH_PAGE`, used for parameters, missions, and other AP_HAL
  storage on flash-backed boards.
- `fram.img`: the SPI RAMTRON/FRAM contents on boards with
  `HAL_WITH_RAMTRON` and a `SPIDEV ramtron` entry. Writes are committed on the
  real chip-select deassertion forwarded by the generated SPI multiplexer.
- `crashlog.img`: the linker-defined CrashCatcher flash region, when present.

`crashlog.img` is created eagerly as erased flash backing; its presence does
not mean that the firmware crashed. An untouched image contains only `0xFF`.

Existing images are reused and are never silently resized. Move or remove a
board's state directory to return it to erased/factory state. The SD image is a
normal host image suitable for `mtools` or loopback mounting while Renode is
stopped. Do not mount the same filesystem read-write on the host while the
firmware is running.

CubeOrange FRAM persistence was checked end to end by setting `FRAME_CLASS=1`,
issuing a firmware reboot, stopping Renode, and starting a new `run.py` process
with the same state directory. Both rebooted instances reported the saved value.

Virtual time is paced at wall-clock speed by default. Renode sleeps only when
the emulation gets ahead, so a workload that cannot sustain real time is not
slowed further. Use `--unthrottled` to run as fast as possible for benchmarks
or automated regression tests.

Works against a stock Renode 1.16.1, but the performance patches in
`patches/` are worth ~4x - see the performance notes for the build
recipe.

### STM32H743 BLHeli/Shared_DMA reproduction

BlitzWingH743 ArduPlane can causally reproduce issue #33926. The test creates a
stale BLHeli motor mapping after initialization, then enters the normal
connection-test and RCOutput serial-DMA path:

```sh
./waf configure --board BlitzWingH743 --enable-APJ_TOOL_PARAMETERS \
    --default-parameters=Tools/renode/params/pr33933.parm
./waf plane
Tools/renode/run.py BlitzWingH743 --vehicle arduplane \
    --reproduce-pr33933 vulnerable
```

The vulnerable run naturally selects a PWM group whose DMA handle was never
initialized and pauses after CrashCatcher writes its dump. Nothing writes or
clears a DMA field. Use `--reproduce-pr33933 fixed` with the AP_BLHeli fix; it
pauses when the stale mapping is rejected before RCOutput. The older
`--reproduce-pr33933 guarded` mode remains available for testing PR #33933's
allocator guard. `--reproduce-pr33933 valid` is the positive control that keeps
the configured output mapping and must reach serial DMA with a non-null handle.
See [`h743_port.md`](h743_port.md) for the root cause, symbolized stack, fix,
and test results.

## Next

- Expand the shared F405/H743 MCU bases and sensor translation table as other
  generated boards expose missing peripherals.
- Add bootloader-in-the-loop execution, using the already compiled
  `hwdef-bl.dat` output.
- Add an AM32-style result-producing CI runner for boot and firmware regression
  scenarios.
