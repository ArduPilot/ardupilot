# ArduPilot under Renode

Runs unmodified ArduPilot STM32 firmware in the [Renode](https://renode.io)
emulator. Supported STM32 F1, F3, F4, F7, G4, H7, and L4 boards are assembled
from their normal `hwdef.dat` and `hwdef-bl.dat`, following the
target-generation pattern used by the AM32 Renode harness. This is a decision
log as much as a README.

## Status

`run.py` invokes ArduPilot's production ChibiOS hwdef compiler on every run and
generates the board REPL and RESC below `build/<board>/renode/generated/` in
under a second. There are no checked-in KakuteF4 or BlitzWingH743 Renode board
descriptions. The application and bootloader hwdefs remain authoritative for
the MCU, flash offset, serial order, buses, sensors, chip selects, DMA, system
timer, and SDMMC presence. The bootloader is not emulated yet; compiling its
hwdef makes its MCU and load offset available now rather than introducing a
second copy of that information.

`Tools/renode/run.py --list` shows the hwdef targets accepted by the generator.
Support is MCU-family driven rather than maintained as a board allowlist. The
current generation audit covers 397 targets, including all 109 STM32
AP_Periph hwdefs. AP_Periph can run without a separate bootloader hwdef because
Renode loads its application ELF directly. The AP_Periph targets by exact MCU
class are:

| MCU hwdef | Targets | Shared platform |
|---|---:|---|
| STM32F103xB / F105xC | 10 / 1 | F1 / connectivity-line F1 |
| STM32F303xC | 10 | `stm32f303_base.repl` |
| STM32F405xx / F407xx / CKS32F407xx / F412Rx / F427xx | 6 / 2 / 1 / 12 / 3 | F4 platforms |
| STM32F732xx | 1 | F767-compatible platform |
| STM32G441xx / G474xx / G491xx | 1 / 10 / 5 | G4 platform |
| STM32H723xx / H743xx / H757xx | 1 / 5 / 4 | H7 platform |
| STM32L431xx / L476xx / L496xx | 35 / 1 / 1 | `stm32l4_base.repl` |

Acceptance means the expanded board configuration translates to generated
REPL/RESC. Representative platform loads, firmware builds, and boots are tested
separately. CubeBlack/F427 and Pixhawk4/F767 mount their SD media and emit an
ArduCopter MAVLink heartbeat; CubeOrangePlus/H757 also emits a heartbeat.
Existing F405, H743, and Pixhawk6X boot results are described below.

A stock HolybroG4_GPS AP_Periph image boots on the G474 platform, obtains a
dynamic DroneCAN node ID, and emits a stationary 3D GNSS Fix2 from its modeled
u-blox receiver on both CAN interfaces. The generator reads
`HAL_PERIPH_GPS_PORT_DEFAULT` and `SERIAL_ORDER` from the expanded hwdef, so the
receiver attaches to USART3 without a board-specific serial override. CAN1 and
CAN2 connect to `mcast:0` and `mcast:1` by default for AP_Periph runs.

Other AP_Periph devices are selected from the same expanded hwdef and processed
defaults. Supported airspeed types attach to the selected I2C bus, serial
rangefinders attach to their configured UART, and analog battery voltage and
current inputs receive deterministic 24 V / 10 A ADC samples. L4 and H7 use an
ADC-v3 model with the board's generated DMA assignment; second-battery analog
pins are populated when present. Non-analog battery backends do not create a
spurious ADC.

The sensor translation covers flight-controller hwdefs in the supported
families. SPI and I2C probes include the Invensense generations, ADIS1647x,
ADIS16607, BMI055/088/160/270, LSM9DS0, LSM6DSV, and SCHA63T IMUs, plus
BMP085/280/388/581, DPS280/310, ICP201XX, LPS2XH, MS5611, and SPL06
barometers. Identity, configuration, stationary sample/FIFO behavior, chip
select framing, and alternative drivers at a shared physical location are
modeled; these are boot/test sensor models, not physical-motion simulation.

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

A stock Pixhawk6X ArduCopter image boots from its alternate 0x24000000 AXI SRAM
layout, mounts its microSD through SDMMC2, detects the Holybro FMUv6 sensor
variant, and reaches the main vehicle loop. Its generated platform includes
both FDCAN controllers, the LAN8742A-compatible RMII Ethernet PHY, the IOMCU,
SPI FRAM, ICM42688/ICM42670/ICM20649 IMUs, and two BMP388 barometers. An
ArduPlane image with networking enabled responds to host-side ARP and ICMP over
a TAP and exchanges MAVLink through an emulated UDP server.

CAN1 and CAN2 can be connected to the same UDP multicast transport used
by ArduPilot SITL, DroneCAN tools, and the AM32 Renode harness. The bridge
supports classic CAN and CAN FD and maps the two interfaces independently to
`mcast:0` and `mcast:1`. H743/H757 boards with an `ETH1` hwdef receive the
Renode Synopsys DWC QoS MAC and PHY; `run.py` can attach that MAC to a host
TAP for an isolated network or an existing host LAN bridge.

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
- `platforms/stm32f427_base.repl` — extends the F405 platform with the second
  flash MiB, the upper SRAM bank, UART7/8, SPI4/5/6, and timer12.
- `platforms/stm32f767_base.repl` and `scripts/ardupilot_f767.resc` — F767
  memory, serial/SPI/I2C, bxCAN, SDMMC1/2, DMA, UART-IDLE, IOMCU, and OTG reset
  support. The Renode F746 base currently obtains its STM32F7x6 SVD on first
  use.
- `peripherals/common/AP_CANMcast.cs` — bidirectional classic CAN/CAN FD
  bridge using the ArduPilot `mcast:N` UDP framing.
- `peripherals/stm32/AP_STM32H7_Ethernet.cs` — preserves the receive-buffer
  address that STM32H7 hardware leaves in RDES0 when the DWC DMA writes receive
  status, and maps STM32's no-source-replacement setting onto the model's MAC0
  replacement setting. This allows ChibiOS to reuse descriptors without the
  model rejecting its valid source-address configuration.
- `peripherals/stm32/AP_STM32F_SDMMC_DmaPump.cs` — restores the F4/F7 SDIO and
  SDMMC DMA request/enable ordering expected by ChibiOS, using each board's
  compiled DMA stream assignment.
- `peripherals/sensors/` — identity, register, stationary-sample, and FIFO
  models for the IMU and barometer families listed in Status.
- `platforms/stm32h743_base.repl` — shared H743 MCU platform, including
  FDCAN1/2 and both SDMMC controller locations. H757 uses the same peripheral
  map while retaining its different application RAM layout from `hwdef.dat`.
  Generated overlays select SDMMC1 or SDMMC2 from the board hwdef.
- `platforms/stm32g474_base.repl` and `scripts/ardupilot_g474.resc` — G474
  memory, DMA/DMAMUX, UART, flash storage, and fixed-message-RAM FDCAN support
  used by HolybroG4_GPS. `peripherals/sensors/AP_UBlox.cs` supplies its
  deterministic serial GNSS input.
- `platforms/stm32l4_base.repl`, `platforms/stm32f303_base.repl`, and their
  scripts — L431/L476/L496 and F303 AP_Periph memory and peripheral platforms.
- `platforms/stm32f103_base.repl` and `scripts/ardupilot_f103.resc` — shared
  F103/F105 platform with channel DMA, I2Cv1, and internal-flash storage.
- `peripherals/sensors/AP_Airspeed.cs` — deterministic MS4525, ASP5033, and
  AUAV differential-pressure models plus the AUAV barometer variant.
- `peripherals/stm32/AP_STM32_ADC_v3.cs` — L4/H7 ADC sequencing and DMA requests
  for generated analog battery inputs.

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
- **Stock `STM32SPI` accepts only one child**, while real boards put several
  devices on one bus. Generated platforms therefore put an active-low
  `AP_SPIMultiplexer` between each SPI controller and its hwdef devices.
  Chip-select GPIOs select a mux address. H7 finite transfers explicitly frame
  sensor transactions; buses containing SPI FRAM retain CS-deassertion framing
  because RAMTRON commands span several controller transfers. When hwdef lists
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
- The host terminal can inject a complete TCP write at one virtual-time point,
  overflowing the H7 UART's 64-byte ChibiOS RX bounce-buffer handoff. Generated
  H743 UART endpoints use `AP_UARTPacer.cs` to supply host bytes over time.
  `AP_STM32F7_USART_Idle.cs` then raises IDLE only after a real input gap,
  instead of treating every DMA read of RDR as an idle line.
- Renode's STM32F7x6 SVD gives OTG `GRSTCTL` the wrong reset value for the
  ChibiOS reset loop (`0x20000000` instead of AHB-idle). The F767 platform maps
  `AP_STM32_OTG_Stub.cs` over the disconnected USB block; it reports AHB idle
  and self-clearing reset/flush requests so boot can continue. MAVLink testing
  still uses the selected hardware UART.
- The stock F4/F7 `STM32FSDMMC` emits read-DMA edges before ChibiOS has enabled
  the board-selected stream, while `STM32DMA` performs writes immediately when
  that stream is enabled, before CMD24 arms the card write path. The generated
  `AP_STM32F_SDMMC_DmaPump.cs` wiring replays read requests and defers only
  SDMMC-bound write enables until the data command is ready. It also clears the
  completed stream enable bit as the hardware does. This is required for both
  CubeBlack SDIO and Pixhawk4 SDMMC to finish mounting and enter the vehicle
  loop.
- The generic Synopsys DWC QoS model replaces every word of an RX descriptor
  with write-back status. STM32H7 preserves RDES0, and ChibiOS initializes its
  buffer address only once before later returning descriptors by rewriting
  RDES3. Without `AP_STM32H7_Ethernet.cs`, the first frame consumes a valid
  buffer and subsequent TAP traffic leaves the DMA in receive-buffer-
  unavailable state. The model also logs valid STM32 SARC=0 transmissions as
  reserved; the helper selects MAC0 replacement using the same address already
  programmed by the firmware.

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

Those levers grew into a patch stack against renode v1.16.1,
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
- 0007: bounded reverse history and faster instruction replay

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
(full performance stack on net10.0). Simulation results stay
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

For AP_Periph targets, `run.py` selects the AP_Periph ELF, opens each CAN
multicast bus, and attaches supported hwdef-selected sensors automatically.
For example:

```sh
./waf configure --board HolybroG4_GPS
./waf AP_Periph
Tools/renode/run.py HolybroG4_GPS
```

### GDB

Build with debug information and pass `--gdb`:

```sh
./waf configure --board CubeBlack --debug
./waf copter
Tools/renode/run.py CubeBlack --gdb
```

Renode starts the machine when GDB attaches; no `start` command is needed in
the monitor. Use GDB's normal interrupt, continue, stepping, and breakpoint
commands. If no terminal emulator is available, `--no-xterm` prints the
generated GDB attach script to run in another terminal.

`--gdb` places a ChibiOS-aware GDB remote adapter in front of Renode's server.
It reads the firmware's `ch_debug` memory signature instead of embedding a
particular ChibiOS structure layout. Once `chSysInit()` has initialized the
registry, `info threads`, `thread N`, and `thread apply all bt` show the guest
threads and their saved Cortex-M contexts. Before that point GDB shows the
single physical CPU. `--gdb-port` selects the public adapter port (3333 by
default); `--renode-gdb-port` can override its private connection to Renode.

Pass `--reverse-debug` together with `--gdb` to enable reverse execution.
Renode then takes periodic snapshots, so GDB can use `reverse-step`,
`reverse-stepi`, and `reverse-continue` (or `rs`, `rsi`, and `rc`).
`--reverse-gdb-limit` limits retained history by guest instruction count and
defaults to 1000; zero selects Renode's unlimited history. The bounded mode
requires the ArduPilot Renode branch described in the performance section.
It retains the snapshots inside the requested window plus one older checkpoint
needed to replay its oldest instruction, so the effective range can exceed the
limit by one snapshot interval.

Reverse execution is single-core. `rs` steps back one source line, which can
require multiple restore/replay cycles; use `rsi` to step back exactly one
machine instruction. `run.py` gives GDB the original DWARF ELF
but loads a generated `gdb-runtime.elf` with only DWARF removed into Renode,
avoiding snapshots containing all debug metadata. Reverse-debug launches also
use a separate 16 MiB FAT16 scratch SD image instead of serializing the normal
256 MiB persistent card into every snapshot, and use a global error-only
Renode log level because Renode 1.16.1 cannot restore snapshots containing
per-peripheral log overrides.

### CAN

Pass `--can` to connect generated CAN1 and CAN2 peripherals to separate
ArduPilot multicast buses:

```sh
./waf configure --board CubeOrange
./waf copter
Tools/renode/run.py CubeOrange --can
```

Configure two ArduPilot CAN drivers through MAVProxy, then reboot:

```text
param set CAN_P1_DRIVER 1
param set CAN_D1_PROTOCOL 1
param set CAN_P2_DRIVER 2
param set CAN_D2_PROTOCOL 1
reboot
```

CAN1 is then available to DroneCAN tooling as `mcast:0`, and CAN2 as
`mcast:1`. The bus numbers follow the multicast convention rather than the
one-based ArduPilot peripheral names. The bridge is closed unless `--can` is
passed. AP_Periph targets are the exception: their CAN buses open by default
because CAN is their primary transport.

### Ethernet

Pixhawk6X is the reference Ethernet target. Enable networking and DHCP on the
firmware, then reboot:

```text
param set NET_ENABLE 1
reboot
param set NET_DHCP 1
reboot
```

`NET_DHCP` and the address parameters are hidden until networking has been
enabled and rebooted once. A static configuration instead uses
`NET_DHCP=0`, `NET_IPADDR0` through `NET_IPADDR3`, `NET_NETMASK`, and
`NET_GWADDR0` through `NET_GWADDR3`.

For example, expose MAVLink as a UDP server after networking is active:

```text
param set NET_P1_TYPE 2
reboot
param set NET_P1_PROTOCOL 2
param set NET_P1_PORT 14550
reboot
```

The second reboot is required because the remaining `NET_P1_*` parameters
are hidden until the port type is enabled. Other network port types and
protocols work normally; Renode does not override ArduPilot parameters.

`--ethernet-tap` attaches the emulated MAC to a named host TAP. For access
to the physical LAN, create a persistent TAP owned by the current user and
enslave it to an existing Linux bridge that already contains the LAN
interface:

```sh
sudo ip tuntap add dev tap-renode mode tap user "$USER"
sudo ip link set tap-renode master br0
sudo ip link set tap-renode up
Tools/renode/run.py Pixhawk6X --ethernet-tap tap-renode
```

Creating or reconfiguring the host bridge is intentionally outside
`run.py`: moving a live Ethernet interface into a bridge can interrupt the
host connection and requires administrator policy. The TAP can instead be
left on an isolated bridge for controlled testing. Bridging exposes the
emulated autopilot directly to the local network, so use the same firewall
and network-port precautions as for physical flight-controller hardware.

On a directly addressed TAP, verify the link with `ping <NET_IPADDR>` before
testing the configured UDP or TCP port. Initial gyro calibration can take
roughly 20 seconds in a paced Pixhawk6X run; serial MAVLink should continue
during calibration, and Ethernet should respond after the firmware reports its
address.

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

- Turn the all-board generation audit into CI and add representative automated
  heartbeat checks for each MCU/sensor group.
- Replace the remaining generic stationary sensor identities with richer data
  paths where a firmware test needs live samples.
- Add bootloader-in-the-loop execution, using the already compiled
  `hwdef-bl.dat` output.
- Add an AM32-style result-producing CI runner for boot and firmware regression
  scenarios.
