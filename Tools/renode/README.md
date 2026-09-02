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
timer, and SDMMC presence. `run.py --bootloader` loads an ArduPilot bootloader
before the application and starts from its vector table.

`Tools/renode/run.py --list` shows the hwdef targets accepted by the generator.
Support is MCU-family driven rather than maintained as a board allowlist. The
current generation audit covers 401 targets, including all 109 STM32
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
Serial transport defaults to a TCP socket terminal on port 5762 and can use a
Unix domain socket instead. An earlier pty terminal went silently deaf on the
inbound side after sustained
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

- `launch.py` — tabbed graphical target/device chooser, lifecycle manager and
  live Renode speed, PC and MIPS status panel. It delegates board construction
  and execution to `run.py`.
- `device_emulator.py` — host-side device models for external transports,
  initially a stationary DroneCAN airspeed node.
- `gen_board.py` — runs the same `ChibiOSHWDef` compiler used by
  `./waf configure`, then translates its expanded configuration and generated
  `hwdef.h` DMA assignments into a board overlay and launch script.
- `platforms/stm32f405_base.repl` — vendored from Renode 1.16.1
  `stm32f4.repl`, edited for the F405 as ArduPilot uses it (real 128K SRAM +
  64K CCM, usart6, bus-correct timer frequencies, IWDG stub, RETTOBASE fix,
  DWT, narrow OTG tags, no implicit Renode SVD fetch). Edit rationale in the
  file.
- `scripts/ardupilot_f405.resc` — common boot script: loads the custom .cs,
  `VectorTableOffset 0x08010000` **plus a `macro reset` re-asserting it**
  (Renode restores VTOR=0 on NVIC_SystemReset; nothing is mapped there),
  loads persistent internal flash (fresh STM32 flash must read 0xFF),
  `PerformanceInMips 125`, 1ms global quantum.
- `renode_data.py` — downloads the SVD matching the exact MCU selected by the
  compiled `hwdef.dat`, verifies its fixed size and SHA-256, and atomically
  caches it outside the git tree. Assets have named roles so future categories
  such as bootloaders do not depend on list ordering.
- `renode_firmware.py` — selects an exact board/vehicle ELF from ArduPilot's
  official firmware manifest, validates its host, size, and ARM ELF identity,
  then atomically installs it in a content-checked cache.
- `fat_image.py` — creates sparse FAT16/FAT32 images and extracts files using
  the platform-independent pyfatfs library. It validates the generated
  geometry and explicitly enables sparse files on Windows.
- `extract_logs.py` — extracts every DataFlash BIN log from a stopped Renode
  SD image without requiring host FAT utilities.
- `build_bundle.py` — assembles a source-checkout-independent runtime tree for
  installers. It includes the Renode Python tools and models, the production
  hwdef compiler with only its required textual inputs, IOMCU firmware and
  bootloaders, and a supplied native physics sidecar. Board photographs,
  build sources, tests, and other repository content are excluded.
- `THIRD_PARTY_NOTICES.md` — copyright and licence terms for the vendored
  Renode platform/model sources and remotely hosted STMicroelectronics SVD
  files.
- `platforms/stm32f427_base.repl` — extends the F405 platform with the second
  flash MiB, the upper SRAM bank, UART7/8, SPI4/5/6, and timer12.
- `platforms/stm32f767_base.repl` and `scripts/ardupilot_f767.resc` — F767
  memory, serial/SPI/I2C, bxCAN, SDMMC1/2, DMA, UART-IDLE, IOMCU, and OTG reset
  support. `run.py` supplies the cached F732 or F767 SVD selected by the
  board's exact MCU type.
- `peripherals/common/AP_CANMcast.cs` — bidirectional classic CAN/CAN FD
  bridge using the ArduPilot `mcast:N` UDP framing.
- `peripherals/common/AP_Hotplug.cs` — monitor command used by the launcher to
  safely unregister a peripheral and its external UART connection while the
  emulated machine is running.
- `peripherals/stm32/AP_STM32*_RCC.cs` — register-driven STM32 clock trees for
  the F1/F3, F4/F7, H7, and L4/G4 families. Firmware oscillator, PLL,
  prescaler, and peripheral-mux writes determine UART, timer, CAN, SPI, I2C,
  ADC, and other kernel clocks instead of relying on fixed platform values.
  Generated board overlays supply `OSCILLATOR_HZ` from the compiled hwdef.
- `peripherals/common/AP_Sigrok.cs` — continuous TCP logic-analyser stream for
  the `renode-la` libsigrok driver. It reconstructs UART and SPI pin edges from
  Renode's byte-level peripheral transactions and samples chip selects from
  their generated GPIO routes.
- `peripherals/common/AP_GPIOStimulus.cs` — on-demand pulse and quadrature
  sources addressed by ArduPilot's logical `GPIO(n)` numbers. They feed the
  physical pads selected by the hwdef and therefore traverse the normal STM32
  SYSCFG/EXTI path used by AP_RPM and AP_WheelEncoder.
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
the firmware (`--firmware` is an alias), and renode itself is found from
`--renode`, `$RENODE` or PATH. Custom application firmware may be APJ, raw BIN,
Intel HEX, or ELF. HEX support uses the Python `intelhex` package; GDB requires
ELF firmware.
The selected firmware UART must still have the desired ArduPilot serial
protocol configured; exposing a UART does not alter firmware parameters.

Pass `--uds` (or select **UDS** in the graphical launcher) to avoid allocating
a TCP port for the selected UART. The endpoint follows the SITL convention from
[ArduPilot PR #34253](https://github.com/ArduPilot/ardupilot/pull/34253) and is
named `APM-UDS-serialN` below the board state directory, where `N` is the
selected `SERIAL_ORDER` index. `run.py` prints the exact MAVProxy endpoint, for
example:

```sh
Tools/renode/run.py CubeOrangePlus --uds
mavproxy.py --master uds:$PWD/renode/CubeOrangePlus/APM-UDS-serial1
```

Inactive socket files left by a terminated Renode process are recovered safely.
An advisory lock held for the complete Renode run prevents another launcher
from replacing the endpoint between its creation and listen, while non-socket
paths are always preserved. The Renode monitor, GDB, USB/IP, sigrok, physics,
and launcher control endpoint remain on TCP because those integrations do not
currently provide compatible Unix-socket transports. The graphical launcher
automatically chooses another free monitor port when its preferred port is
occupied or still in TCP `TIME_WAIT` after a previous run.

### Graphical launcher

`launch.py` provides a board and firmware chooser around `run.py`:

```sh
Tools/renode/launch.py --renode ~/project/UAV/renode/renode
```

The **Target** tab discovers supported boards and their built ELF images,
selects a matching bootloader when one is available, and exposes CPU pinning,
real IOMCU, UDS, USB/DFU, CAN bus and Ethernet TAP options. The **Config** tab
expands the selected board's production `hwdef.dat` and lists its
`SERIAL_ORDER`, `I2C_ORDER`, and `CAN_ORDER` ports. Devices can be attached
directly to those logical ArduPilot ports; the physical MCU peripheral is shown
alongside each one. Firmware parameters must still enable the corresponding
driver and port.

The Config tab remains enabled after Start. Attach and Remove make live Renode
configuration changes, which permits testing firmware discovery and runtime
initialisation without restarting the flight controller. A UART is
point-to-point and therefore accepts one device. An I2C bus accepts multiple
devices as long as their addresses do not collide, and a CAN port accepts
multiple nodes with unique node IDs. The address or node ID and connection
state are shown for every attachment. The same attachment list is retained for
the next launch. Each attachment has a gear button for changing its simulated
configuration. Every device can be switched off without deleting it; this
hot-unplugs the peripheral while Renode is running and permits it to be
re-enabled later. Rangefinders expose their ArduPilot orientation and physics
input, compasses expose their simulated sensor orientation, and DroneCAN nodes
can use either an automatic or fixed node ID.

Pressing **Start** or **Quit** writes the complete Target, Config, and Physics
selection to `launch-settings.json` in the directory from which the launcher
was started. The launcher restores that file the next time it is started from
the same directory, allowing separate working directories to retain different
configurations. Runtime-only state such as generated device names and transient
connection status is not saved.

The initial device catalog provides u-blox GPS and Benewake/LightWare
rangefinders on UARTs; IST8310 compass and MS4525, AUAV, and ASP5033 airspeed
sensors on I2C; and a DroneCAN airspeed node on CAN. UART and I2C models run
inside Renode. Host-side DroneCAN nodes use the same multicast transport as
other DroneCAN tooling. They are owned by the graphical launcher for live
attach/remove, or by `run.py` when it is invoked directly, and stop with
Renode. The catalog and attachment format retain the bus type separately,
allowing timer outputs, GPIO interrupt sources, device-specific options, and
physics inputs to be added without changing the Config tab layout.

Start and Stop own the complete Renode and USB-helper process groups, and Quit
performs the same cleanup before closing.
The runtime panel obtains status directly from Renode's monitor and shows the
current PC, configured MIPS, actually executed MIPS, and virtual-time speedup.
The target for paced execution is `1.00x realtime`; a lower value means the
host is not keeping up with the selected model and workload.

### Standalone physics sidecar

The first reusable ArduPilot physics backend is built as a separate SITL tool:

```sh
./waf configure --board sitl
./waf --targets tool/renode-physics
build/sitl/tool/renode-physics --model quad --physics-port 9002
```

It uses the existing `libraries/SITL` model factory and dynamics directly,
then exchanges timestamped actuator and sensor state with Renode over the
localhost lockstep protocol. The executable embeds the model and location data
it needs, so an installed copy does not need an ArduPilot source checkout at
runtime. A Windows installer can package the corresponding Cygwin executable
and runtime alongside Renode. The launcher integration and model/location
controls are available in the launcher's **Physics** tab. It initially offers
quadcopter, fixed-wing plane, rover, and quadplane models, plus latitude,
longitude, altitude, heading, and exchange-rate controls. Physics can start
with Renode or be connected and disconnected while firmware is running. This
makes flight-controller resets and runtime sensor initialisation testable
without resetting the physical vehicle. Stop the sidecar before choosing a new
model or location; reconnecting the bridge preserves its current state. The
requested protocol rate must not exceed the selected model's native update
rate.

The launcher finds `renode-physics` beside `launch.py`, under its `bin/`
directory, in a local SITL build, or on `PATH`; `--physics-binary PATH`
overrides discovery. This order lets a Windows installer place the Cygwin
executable and its runtime DLLs alongside the launcher without exposing a
source-tree path.

### Installer runtime bundle

Build the native sidecar for the installer target, then assemble a runtime
tree with:

```sh
Tools/renode/build_bundle.py build/renode-runtime \
    --physics-binary build/sitl/tool/renode-physics
```

The output preserves the small portion of the repository layout used by the
launcher (`Tools/renode`, `libraries/.../hwdef`, and their runtime data), so
the normal hwdef compiler remains authoritative without requiring an
ArduPilot checkout. `bundle.json` records every installed file's size and
SHA-256 for release tooling. The builder refuses to replace an existing
output directory and writes through a temporary sibling, preventing a failed
build from leaving a partial installer tree.
Concurrent builders coordinate through a sibling advisory lock; the operating
system releases ownership even if a build process is killed.

Run `Tools/renode/launch.py` from the resulting tree and select a firmware ELF
from any location. A platform installer must additionally provide Python 3,
PySide6, pymavlink, pyfatfs 1.1.0, intelhex, Renode, and the native sidecar's
runtime libraries. On Windows those can be installed beside this tree; no
build tools, native FAT utilities, or source checkout are used at runtime. The
platform-specific installer wrapper is a separate release stage.

An installed bundle can obtain a firmware ELF without build tools or a source
tree. For example:

```sh
Tools/renode/renode_firmware.py CubeBlack --vehicle Copter --channel latest
```

The catalog distinguishes the moving `stable` and `latest` aliases from
versioned archives and requires the URL's board directory to match exactly,
so variants such as `CubeBlack-heli` cannot be selected accidentally. The
download is restricted to `firmware.ardupilot.org`, bounded in size, and must
be a 32-bit little-endian ARM ELF. The server manifest's Git SHA selects the
cache directory; locally recorded size and SHA-256 values detect corruption
before reuse. The manifest does not currently publish a content hash for the
ELF, so HTTPS and the official host authenticate the initial download.
The same operation is available in the Target tab: choose the vehicle and
`stable` or `latest`, then click **Download Firmware**. Downloading runs in the
background, and the verified cached ELF becomes the selected firmware when it
finishes.

The **Download Renode** button checks
`https://firmware.ardupilot.org/Tools/Renode/latest.json` on every use and
selects the portable package matching the host architecture. Downloads are
size and SHA-256 checked, extracted atomically, and cached under
`~/.cache/ardupilot/renode/`. A cached executable is reused only when its
recorded source revision and package hash still match the current server
manifest. Use `--renode-cache DIR` to select another cache directory. The
launcher also checks for an update in the background when it opens and changes
the button to **Update Renode** when the managed cache is stale.

`run.py` also downloads the matching SVD for every supported MCU type from
`https://firmware.ardupilot.org/Tools/Renode/data/SVD/`. Selection uses the
exact `MCU` value from the compiled `hwdef.dat`, independently of which Renode
platform model the MCU shares. The files are validated against sizes and
SHA-256 hashes recorded in `renode_data.py`, installed atomically, and reused from
`~/.cache/ardupilot/renode/data/SVD/`. The launcher's `--renode-cache DIR`
places them in `DIR/data/SVD`; direct `run.py` users can select another data
root with `--data-cache DIR` or `RENODE_DATA_CACHE`. Set
`RENODE_DATA_BASE_URL` to use another download mirror. The typed layout allows
future server jobs to populate separate categories such as `data/bootloaders/`
without mixing them with SVDs.

When USB is selected, the launcher starts `usbip_attach.py` as the current
user. Perform its one-time udev setup before the first launch:

```sh
sudo Tools/renode/usbip_attach.py --install-rules
```

For automated UI tests, `--control-port N` enables a localhost command socket.
Run `Tools/renode/launch.py --help` for its board, firmware, option, lifecycle,
and JSON status commands.

Use `--bootloader` to execute a bootloader before the selected application:

```sh
Tools/renode/run.py CubeOrange \
    --bootloader Tools/bootloaders/CubeOrange_bl.bin
```

The bootloader can be an ELF, a raw BIN based at `0x08000000`, or an Intel HEX
whose embedded addresses select its flash location. The application still
comes from the normal build output, `--firmware`, or its legacy `--elf` alias.
Both images are overlaid into `flash.img` on every launch, and resets return to
the bootloader vector table.

By default the bootloader follows its normal timeout and starts the
application. Pass `--hold-bootloader` to request the same one-shot bootloader
hold used by an application-requested reboot, keeping the upload interface
available until the uploader commands a reboot.

State survives both firmware resets and separate `run.py` invocations. By
default it is stored in `renode/<board>/`; use `--state-dir DIR` to select an
exact alternative directory. Depending on the hwdef, this contains:

- `sdcard.img`: a newly created sparse 512 MiB FAT32 microSD image with 4 KiB
  clusters, attached to Renode in persistent mode.
- `flash.img`: the complete internal MCU flash, sized from `BOARD_FLASH_SIZE`
  (2 MiB on Pixhawk6X). The current firmware's file-backed flash contents are
  overlaid into the image on every launch, while parameters, missions, crash
  dumps, and other firmware-written flash data persist.
- `fram.img`: the SPI RAMTRON/FRAM contents on boards with
  `HAL_WITH_RAMTRON` and a `SPIDEV ramtron` entry. Writes are committed on the
  real chip-select deassertion forwarded by the generated SPI multiplexer.
- `mcu_id.txt`: the persistent random 96-bit STM32 unique-device ID, encoded
  as 24 hexadecimal characters. It is created on the first launch and used by
  firmware logs, bootloader identification, and USB serial-number descriptors,
  so separate state directories appear as separate physical boards.

When creating `flash.img` for the first time, `run.py` imports existing
`storage.img` and `crashlog.img` state at their flash addresses and leaves the
old files in place as backups. A legacy crash-log image is aligned to the end
of flash, so it remains at the address used by the firmware build that created
it even when the current linker's crash-log boundary has moved.

ELF gaps in the executable flash overlay retain the erased value `0xFF`. This
matters for firmware-identity checks: flattening a non-file-backed alignment
section as zeroes makes an SD crashdump's runtime flash CRC differ from the
matching ELF.

`test_crashdump.py` exercises an H7 microSD crashdump end to end using a fresh
card image. It waits for MAVLink, sends ArduPilot's failure-creation HardFault
command, requires the rebooted firmware to report the retained dump, extracts
`APM/CrashDump.DAT`, and runs the `crashdump_info.py` found beside the selected
ELF:

```sh
Tools/renode/test_crashdump.py Pixhawk6X --vehicle arduplane \
    --elf /path/to/crashdump-branch/build/Pixhawk6X/bin/arduplane
```

Use `--fault lockup` to stop the main loop and exercise the scheduler monitor
thread, watchdog, and its pre-watchdog crashdump path:

```sh
Tools/renode/test_crashdump.py Pixhawk6X --fault lockup \
    --elf /path/to/build/Pixhawk6X/bin/arducopter
```

For a firmware built with `--enable-CRASHDUMP_FLASH` and
`--disable-CRASHDUMP_FATFS`, select the internal-flash backend. The test reads
the crash-log range from the ELF, extracts it from the persistent full-flash
image, and checks the CrashCatcher signature and recorded length:

```sh
Tools/renode/test_crashdump.py Pixhawk6X --backend flash \
    --vehicle arduplane --elf /path/to/build/Pixhawk6X/bin/arduplane
```

The flash and lockup tests enable `run.py --watchdog`, which makes the
independent watchdog reset the emulated machine when it expires and models the
fault address used by the scheduler monitor thread. It is opt-in so pausing a
normal `--gdb` session does not unexpectedly reset the target.

Existing images are reused and are never silently resized. Move or remove a
board's state directory to return it to erased/factory state. The SD image is a
normal host image suitable for FAT image tools or loopback mounting while
Renode is stopped. `run.py` requires pyfatfs 1.1.0 to create a new image. Do
not mount the same filesystem read-write on the host while the firmware is
running.

Extract all DataFlash logs after stopping Renode with:

```sh
Tools/renode/extract_logs.py renode/CubeOrangePlus/sdcard.img \
    --output-dir extracted-logs
```

`--output-dir` defaults to the current directory.

CubeOrange FRAM persistence was checked end to end by setting `FRAME_CLASS=1`,
issuing a firmware reboot, stopping Renode, and starting a new `run.py` process
with the same state directory. Both rebooted instances reported the saved value.

Virtual time is paced at wall-clock speed by default. Renode sleeps only when
the emulation gets ahead, so a workload that cannot sustain real time is not
slowed further. Use `--unthrottled` to run as fast as possible for benchmarks
or automated regression tests.

On a hybrid host, `--cpusel N` pins only Renode's emulated MCU CPU thread to
host CPU N. Renode's logging, socket, monitor and runtime helper threads remain
available to the normal scheduler, so they do not contend with the emulated CPU
on the selected core. For example:

```sh
Tools/renode/run.py CubeOrange --cpusel 6
```

Use `--num-imus N` to limit sensor emulation to the first N distinct IMUs in
the board's hwdef. Barometers and other sensors are unaffected. This is useful
for reducing the peripheral workload on boards with several redundant IMUs:

```sh
Tools/renode/run.py CubeOrange --num-imus 1
```

Basic emulation works against stock Renode 1.16.1. The recommended packages
downloaded by `tests/fetch_renode.sh` include the performance patch stack
(worth roughly 4x) and USB/IP support used for firmware re-enumeration.

For AP_Periph targets, `run.py` selects the AP_Periph ELF, opens each CAN
multicast bus, and attaches supported hwdef-selected sensors automatically.
For example:

```sh
./waf configure --board HolybroG4_GPS
./waf AP_Periph
Tools/renode/run.py HolybroG4_GPS
```

### Real IOMCU

H7 flight controllers with an `IOMCU_UART` can run the real STM32F100 IOMCU
bootloader and firmware in a second Renode machine:

```sh
Tools/renode/run.py CubeOrangePlus --real-iomcu \
    --bootloader Tools/bootloaders/CubeOrangePlus_bl.bin
```

The generated FMU UART and the F100 USART2 are connected through a paced UART
hub. Renode runs the two CPUs with deterministic serial execution and a 1.5 ms
global quantum; this prevents concurrent native CPU execution from racing
exception entry without excessive synchronization overhead. Larger quanta can
make the 1.5 Mbaud FMU/IOMCU link unreliable. The F100 model supplies the
24 MHz Cube oscillator, USART IDLE timing, DMA receive request, and flash page
erase behavior needed by the unmodified bootloader and application.

The default IOMCU bootloader is `Tools/bootloaders/iomcu_bl.bin`; override it
with `--iomcu-bootloader`. The application comes from the FMU ELF's embedded
`io_firmware.bin`. Its complete 64 KiB flash persists as `iomcu-flash.img` in
the board state directory. `--iomcu-force-update` erases only the application
region before startup, causing the FMU to upload its embedded image through
the real IOMCU bootloader. Do not leave that option enabled on every run if the
persisted result is what you want to test.

Once MAVProxy is connected to the selected MAVLink UART, these commands use
the normal FMU-to-IOMCU register path:

```text
arm safetyoff
arm safetyon
```

The FMU's normal `IOMC` DataFlash message records the real IOMCU status once
logging is active. Set `LOG_DISARMED=1` when collecting it without arming the
vehicle. Real-IOMCU mode currently requires an STM32H7 FMU; the synthetic
IOMCU remains the default for faster general board testing.

### USB/IP

On Linux, `--usb` exports the firmware-driven STM32F4 or H7 USB controller
through Renode's USB/IP server. This includes F4 boards such as CubeBlack.
Install the distribution's `usbip` userspace tools, then start Renode and the
vhci helper in separate terminals:

```sh
Tools/renode/run.py CubeOrange --usb \
    --bootloader Tools/bootloaders/CubeOrange_bl.bin \
    --hold-bootloader
Tools/renode/usbip_attach.py
```

The helper does the USB/IP import itself and hands the socket to
`vhci_hcd` through sysfs, so the distribution's usbip tool is not
needed. Root is only required for the kernel's attach/detach files;
run once

```sh
sudo Tools/renode/usbip_attach.py --install-rules
```

to load `vhci_hcd` at boot and make them group-writable for `dialout`,
after which the helper runs as a normal user. (Without the rules it
still works run as root.) To undo the setup, remove
`/etc/udev/rules.d/99-vhci-user.rules` and
`/etc/modules-load.d/vhci-hcd.conf`. It attaches export `1-0` and monitors it for
firmware USB disconnects. It automatically reattaches after bootloader,
application, or runtime USB re-enumeration so Linux obtains the new
descriptors. Control-C or SIGTERM exits cleanly and detaches the vhci
port this helper attached. Pass the same `--port N` to the helper when
using `run.py --usbip-port N`.

Select the launcher's **DFU** checkbox, or pass `--usb --dfu`, to expose an
STM32 factory-ROM-compatible DfuSe device (`0483:df11`) before Renode starts.
The DFU client writes the board's persistent `flash.img`. After a successful
download manifests, the DFU endpoint disconnects, Renode starts from the
programmed image, and the USB helper reconnects to the firmware-driven USB
device on the same USB/IP port. The selected firmware supplies the matching
board model in this mode but is not overlaid onto flash.

The virtual controller then appears in `dmesg`, as `/dev/ttyACM*`, and
under `/dev/serial/by-id/`. Both bootloader and application descriptors use
the persistent ID from `mcu_id.txt`. Firmware can therefore be uploaded
through the same host path used for hardware, for example:

```sh
Tools/scripts/uploader.py \
    --port /dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange-BL_*-if00 \
    build/CubeOrange/bin/arducopter.apj
```

Use paced execution for uploading. With `--unthrottled`, guest time can pass
the bootloader's receive timeout while the host is between separate USB
writes. The Renode build script applies
`patches/usbip-device-state.patch`, which reports full-speed USB and makes a
firmware soft-disconnect close the current USB/IP client while keeping the
export listener available.

Renode's USB/IP server listens on all host interfaces. Keep its TCP port
firewalled from untrusted networks.

### Sigrok/PulseView

Pass `--sigrok` to expose a continuous logic-analyser stream on TCP port 4242:

```sh
Tools/renode/run.py Pixhawk6X --sigrok
pulseview -d renode-la:conn=tcp/127.0.0.1/4242
```

The capture contains the selected main MAVLink UART's TX and RX, then SCK,
MOSI, MISO, every chip-select line belonging to the first SPI bus, and every
pin with a `GPIO(n)` assignment in the compiled hwdef. This includes inactive
`ALT(n)` pin configurations so relay assignments can be inspected after a
runtime board configuration change. PulseView receives physical pin, hwdef
signal, and logical GPIO names from Renode. It also tells Renode which channels
are enabled, so disabled channels do not generate or transmit samples. The
default sample rate is 10 MHz; use
`--sigrok-sample-rate HZ` and `--sigrok-port PORT` to change it. UART and SPI
wire edges are reconstructed from Renode's byte-level models, while chip-select
and general GPIO levels come directly from the generated GPIO fan-out.

Use `--sigrok-channels` to advertise only matching channels. It accepts a
comma-separated list of case-insensitive shell wildcards matched against
peripheral names, hwdef signal labels, logical GPIO names, and physical pin
numbers. `UARTn` and `USARTn` are treated as equivalent. For example:

```text
Tools/renode/run.py CubeOrange --sigrok \
    --sigrok-channels 'UART2,SPI*,PB4'
```

Every generated hwdef `GPIO(n)` is also connected to an on-demand external
stimulus. For example, these monitor commands generate a 25 Hz RPM input on
GPIO 1, then replace it with a 10-cycle/s quadrature wheel encoder on GPIOs 1
and 2:

```text
sysbus.gpioStimulus StartPulse 1 25
sysbus.gpioStimulus StartQuadrature 1 2 10 false
sysbus.gpioStimulus StopAll
```

`Set GPIO LEVEL` injects an individual level and `Stop GPIO` stops one source.
The firmware still owns pin mode and EXTI edge selection: AP_RPM configures its
pin as an input with a rising-edge interrupt, while AP_WheelEncoder configures
both inputs for both-edge interrupts. A stimulus on an output, analog, or
alternate-function pin is retained externally and becomes visible immediately
if firmware changes that pad to an input. No periodic Renode clock entry exists
until `StartPulse` or `StartQuadrature` is called.

### GDB

Build with debug information and pass `--gdb`:

```sh
./waf configure --board CubeBlack -g
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
512 MiB persistent card into every snapshot, and use a global error-only
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
because CAN is their primary transport. `--can-base N` moves CAN1 to
`mcast:N` and following interfaces to successive buses; the default remains
zero. This is primarily useful for parallel automated tests.

### Automated tests

`test_all.py` discovers supported firmware already present under
`build/*/bin`. Its optional wildcard matches either a board or a
`board/vehicle` name. Quote shell wildcards so the script receives them:

```sh
Tools/renode/test_all.py --parallel 4
Tools/renode/test_all.py 'HolybroG4*' --parallel 2
Tools/renode/test_all.py 'CubeBlack/arducopter'
```

Flight-controller tests wait for a MAVProxy-detected heartbeat and then run
`param ftp`, requiring a complete non-empty parameter download. AP_Periph
tests allocate a DroneCAN node ID and issue indexed parameter GetSet requests
until the target returns the end of its table. Each worker has private flash,
SD, Renode configuration, TCP ports, and (for AP_Periph) multicast CAN buses.
Use `--list` to show the builds selected by a pattern and `--timeout` to adjust
the per-test limit.

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

## Next

- Turn the all-board generation audit into CI and add representative automated
  heartbeat checks for each MCU/sensor group.
- Replace the remaining generic stationary sensor identities with richer data
  paths where a firmware test needs live samples.
- Add an AM32-style result-producing CI runner for boot and firmware regression
  scenarios.
