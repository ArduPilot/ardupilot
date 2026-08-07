# H743 port progress notes

Primary target: run BlitzWingH743 ArduPlane under Renode and deterministically
reproduce the firmware bug behind issue #33926 / PR #33933. The completed
reproduction follows the normal BLHeli connection-test and RCOutput paths: a
stale BLHeli motor mapping selects an unconfigured PWM group and its null DMA
handle. The vulnerable firmware hardfaults and the AP_BLHeli fix rejects the
invalid mapping before entering RCOutput. This demonstrates that the Renode
harness can reproduce and test hard STM32H743 firmware bugs at the peripheral
and DMA level.

A naturally generated CrashCatcher dump and a backtrace matching the reporter's
dump would be valuable corroborating evidence, but dump generation or extraction
must not block the primary bug reproduction.

Reference backtrace (reporter's crash_dump.bin + stable-4.7.0 ELF, via
gdb_crashdump.sh): PC=0 <- Shared_DMA::lock_core <- RCOutput::
serial_write_bytes <- AP_BLHeli::BL_SendBuf <- BL_ConnectEx <-
blheli_process_command <- process_input <- GCS_MAVLINK::update_receive.

## Stages (each committed, plans and commits codex-reviewed)

1. Platform + boot: vendored stm32h743 repl, minimal AP_STM32H7_PWR/RCC
   models covering the proven clock-init polls, boot the stock arduplane
   ELF (app-only, VTOR 0x08060000) to ChibiOS threads and AP_Vehicle::setup.
2. Config + MAVLink + storage: apj_tool-embedded defaults (needs
   --enable-APJ_TOOL_PARAMETERS at configure), minimal DPS310 (the baro
   gates gcs().setup_uarts in Plane), USART3 + DMAMUX wiring, TCP serial,
   heartbeat + param round-trip and persistence; run.py --vehicle.
3. Sensors + main loop: ICM42688 (Invensensev3) model, main loop and EKF3
   update path active, useful prearm residuals. Fixed-wing EKF bootstrap
   remains at its intentional 3D-GPS-lock gate until a GPS is modeled.
3.5 Causal issue #33926 reproduction (complete): configure output 5 as
   DShot300, let AP_BLHeli cache it, then make its live lookup fail and use the
   firmware's BLHeli connection test. The vulnerable build silently falls back
   to output 1, selects an unconfigured PWM group, and hardfaults through its
   null DMA handle. No DMA state is injected.
4. Timer update DMA (complete): synthesize the TIM1/3/4/5 update requests that
   Renode's stock STM32 timer omits. A real BLHeli serial byte now completes
   through the firmware's DMA IRQ instead of its timeout/cancel path.
4.5 Root fix (complete): make AP_BLHeli channel lookup fallible, reject missing
   or stale mappings at every consumer, and keep valid mappings on the real
   serial-DMA path.
5. Later: bootloader-in-the-loop, USB OTG device model, proper ADCv4, CI.

## Generated board configuration

There are no BlitzWingH743-specific Renode REPL, RESC, or baseline parameter
files. `run.py` invokes `gen_board.py`, which runs the production
`ChibiOSHWDef` compiler over both `hwdef.dat` and `hwdef-bl.dat` on every
launch. It consumes the compiler's expanded configuration and resolved
`hwdef.h` DMA definitions, then writes disposable REPL/RESC files below
`build/BlitzWingH743/renode/generated/`.

The application hwdef supplies the devices, buses, pins, DMA assignments,
serial order, system timer, storage, and application offset. Although the
bootloader is not run yet, its hwdef is compiled and checked for the same MCU
while its load offset is retained independently. Thus the normal board hwdefs
remain the only board-level source of truth, and adding another F405/H743 target
requires no Renode board file when its peripherals are already modeled.

## Board facts (BlitzWingH743 = thin include of BlitzH743Pro)

- 8MHz HSE, 480MHz (VOS1 + SYSCFG ODEN overdrive), HCLK 240MHz, timer
  clocks 240MHz. App at 0x08060000 (384K reserve), len 1664K.
- Storage: AP_FlashStorage on 2x128K sectors at 0x08020000 (must read 0xFF
  when erased). Crash log: unused app-flash tail, derived from each ELF
  (`0x081d6f20..0x08200000` for the vulnerable reproduction build) - it must
  read all-0xFF or
  CrashCatcher silently no-ops; one dump per flash, never refill on reset.
- Tick timer: TIM12 (16-bit) - the CCR==0 nudge hook moves here.
- Main RAM: 0x30000000 D2 SRAM1/2 (matches reporter's SP 0x30001ed0);
  ram0 len 256K. App enables SRAM1-3 clocks and ITCM/DTCM itself -
  app-only boot needs nothing from the bootloader (verified in
  hal_lld.c stm32_clock_init + board.c __early_init; confirmed by codex).
- SERIAL_ORDER OTG1 USART1(MSP) USART2(RCIN) USART3(none) UART4(GPS) EMPTY
  USART6 UART7 UART8(ESCtelem) OTG2 - no default MAVLink UART; SERIAL3/
  USART3 is DMA-capable and can be exposed with `run.py --serial 3`.
- IMU: single Invensensev3 (ICM42688-class) SPI1 CS PC15, no DRDY, fast
  sampling. Baro DPS310/SPL06 on I2C2@0x76; H7 hwdefs exclude I2C DMA
  entirely (interrupt-mode I2Cv2 -> stock STM32F7_I2C should serve).
  OSD MAX7456 on SPI2. Logging via SDMMC (no card = fail soft; the IO
  thread retries the crash-dump copy every 5s, non-blocking).
- DShot: TIM3/TIM5/TIM4 groups (BIDIR) + TIM1 (LED strip), TIM15 NODMA.
- Resolved DMA (build/BlitzWingH743/hwdef.h): SPI1 RX/TX = DMA1 s3/s4,
  USART1_RX DMA1 s5, USART3 per hwdef.h, TIM3/5/4_UP = DMA1 s0/s1/s2,
  ADC1 = DMA2 s1, SPI2_RX = DMA2 s7 shared with TIM1_UP, SPI2_TX DMA2 s4.
- DMAMUX1 request numbers (ChibiOS STM32H7xx/stm32_dmamux.h): ADC1=9,
  TIM1_UP=15, TIM3_UP=27, TIM4_UP=32, SPI1_RX/TX=37/38, SPI2_RX/TX=39/40,
  USART1_RX/TX=41/42, USART2_RX=43, USART3_RX/TX=45/46, TIM5_UP=59,
  UART4_RX/TX=63/64. Renode wiring: peripheral request GPIO ->
  `dmamux1@<request>` (stock repl uses spi4 -> dmamux1@83 = SPI4_RX).

## Renode H7 base (from the shared perf branch)

platforms/cpus/stm32h743.repl is complete and mostly exact: STM32H7_EXTI,
STM32H7_FlashController (dual bank, 32-byte programming), STM32DMA (stream
layout correct for H7), STM32_DMAMUX, STM32F7_USART (= H7 UART layout),
STM32F7_I2C (= I2Cv2), STM32H7_SPI (SPIv2 FIFO), MCAN, STM32HSDMMC,
STM32H7_QuadSPI. Gaps we cover: no PWR model (was Tag hacks), STM32H7_RCC
lacks most ENR/RSTR/CCIPR/DxCFGR registers, ADCv4 poor fit (fail-soft
initially), no USB OTG (TCP serial instead), SPI1/2 not declared,
systickFrequency 96MHz (board 480MHz), timers declared 250MHz (board 240).

## Clock-init polls the minimal PWR/RCC models must satisfy

PWR: CSR1.ACTVOSRDY (bit13), D3CR.VOSRDY (bit13) after SYSCFG PWRCR ODEN
write (no poll on ODEN itself). RCC: HSION->HSIRDY, CFGR SW/SWS mirror
(HSI first, PLL1 after), HSEON->HSERDY, HSI48ON->HSI48RDY,
PLL1/2/3ON->RDY (polled together), plus plain-storage readback for
HSICFGR/CSICFGR/CSR/PLLCKSELR/PLLCFGR/PLLxDIVR+FRACR/D1-3CFGR/CCIPRs/
BDCR/RSR/all ENR+RSTR (hal_lld_init mass-resets everything after clock
init, and enables SRAM1-3 via AHB2ENR). FLASH ACR latency readback is the
stock H7 flash controller's job.

## Stage 1 status (complete)

Boot of the stock `build/BlitzWingH743/bin/arduplane` ELF reaches
`Plane::init_ardupilot()`, after `AP_Vehicle::setup()` and a successful
FAT32 mount. Passing on the way, all with the models in this tree:

- `stm32_clock_init` returns: `AP_STM32H7_PWR` + `AP_STM32H7_RCC`
  satisfy every unbounded poll (PWR CSR1.ACTVOSRDY and D3CR.VOSRDY
  after the SYSCFG PWRCR ODEN write; RCC HSI/HSE/HSI48/PLL1-3 ready and
  the CFGR SW->SWS mirror, first to HSI then to PLL1_P).
- `chSysInit` and `main`: threads run, so the RETTOBASE and DWT fixes
  carry over to the H7 unchanged.
- `usb_lld_start`: OTG core reset falls through. Note ChibiOS OTG1 on
  this board is the **FS core at 0x40080000**, not the HS core at
  0x40040000 - read out of the stalled CPU's r6, after tagging the
  wrong one first. Only GRSTCTL reads AHBIDL set; everything else in
  the block must read 0 or the OTG ISR storms (F405 lesson).
- `AP_Vehicle::setup()`, then `sdcard_retry()`.
- SDMMC card identification, FAT sector reads, and mount; execution
  leaves `sdcard_retry()` and enters `Plane::init_ardupilot()`.

### SDMMC IDMA resolution

An SD card is not optional for boot here: with no card the ChibiOS
driver spins in `sdc_lld_send_cmd_none` on CMDSENT, which the model
only raises when a card answers. With a card attached
(`machine SdCardFromFile`, persistent FAT32-formatted image),
identification completes but the mount blocks: Renode's `STM32HSDMMC`
leaves the IDMA registers as tags, and the ChibiOS H7 driver uses IDMA
exclusively, so no data ever moves and DATAEND never fires.

`AP_STM32H7_SDMMC.cs` implements IDMA as a pump between the base
model's FIFO paths and system memory. The base model defers command
data preparation to the next synchronized state; the wrapper queues
its pump at the same boundary after observing both IDMA enable and
DCTRL.DTEN. It queues from both CMD and DCTRL because ChibiOS writes
DCTRL after the command for block I/O but before the command for short
card-register reads.

A second issue appeared after sector 0: Renode's image-backed SDCard
left a single-block CMD17 in DATA forever, so the sector-1 preflight
poll issued CMD13 indefinitely. The wrapper now gives CMD17 its
implicit DATA-to-TRAN completion after fetching the block; multi-block
CMD18 still uses the driver's explicit CMD12. The FAT mount then
completes and Plane initialization begins.

`run.py` creates and supplies a persistent 256 MiB FAT32 image. It defaults to
`renode/<board>/sdcard.img`, or to the directory selected with `--state-dir`,
and is directly accessible with host image tools while Renode is stopped.

## Stage 2 status (complete)

For the historical MAVLink bring-up, the BlitzWing image was configured with a
temporary embedded `SERIAL3_PROTOCOL=2` default and built with:

```sh
./waf configure --board BlitzWingH743 --enable-APJ_TOOL_PARAMETERS \
    --default-parameters=/path/to/temporary-mavlink-defaults.parm
./waf plane
Tools/renode/run.py BlitzWingH743 --vehicle arduplane --serial 3
```

That board-specific bring-up parameter file is no longer part of the harness;
the generator never changes firmware parameters.

`apj_tool.py --show` reports the embedded `SERIAL3_PROTOCOL 2` default.
The DPS310 model on I2C2 address 0x76 supplies a checked product ID,
stable calibration coefficients, data-ready status, and 100000Pa/25C
samples. Plane passes the barometer gate and starts USART3 MAVLink;
the TCP endpoint emits heartbeats from the unmodified ArduPlane ELF.

Inbound traffic required one additional H7 model correction. Renode's
`STM32F7_USART` transfers incoming bytes through DMAMUX/DMA correctly,
but models ISR.IDLE and CR1.IDLEIE only as tags. ChibiOS relies on the
IDLE interrupt to stop DMA and harvest a partially filled 64-byte bounce
buffer, so a normal MAVLink request remained invisible until the whole
buffer filled. `AP_STM32F7_USART_Idle` synthesizes IDLE after DMA empties
the USART FIFO and clears it through ICR.IDLECF. The existing DMA fixup
also resets the model's private stream offset when ChibiOS rewrites NDTR,
matching the hardware pointer reload for each new bounce buffer.

A single live pymavlink connection proved the complete path:

```text
embedded SERIAL3_PROTOCOL=2
set MAV_SYSID=42
requested reboot
post-reboot heartbeat system=42
persisted MAV_SYSID=42
```

This covers UART TX/RX, MAVLink parsing and replies, AP_FlashStorage
writes, the Renode reset macro's VTOR restoration, and storage survival
across an NVIC firmware reboot.

## Stage 3 status (complete)

`AP_ICM42688` models the board's SPI1 device at CS PC15. It supplies
the ICM-42688-P ID, five banks of configuration-register storage, FIFO
flush behavior, and 1kHz 16-byte little-endian FIFO records. The sample
is a stable level bench condition: sensor Z is +1g and becomes -1g after
the board's `ROTATION_ROLL_180`; angular rates are zero and temperature
is 25C. FIFO count is in records as selected by the driver's
`INTF_CONFIG0` setup.

The stock H7 SPI model always forces master mode but logged an error each
time ChibiOS restored a cached CFG2 value with MASTER clear. SPI1 runs at
the IMU callback rate, so those false diagnostics dominated runtime.
Board hooks preserve CFG2.MASTER on SPI1 and SPI2 writes, matching the
model's effective behavior and keeping the long calibration run usable.

After normal gyro calibration, the firmware reaches standby and reports:

```text
SYS_STATUS gyro/accel health bits set, errors_comm=0
RAW_IMU accel=(0,0,-999) gyro=(0,0,0) temp=2500
SCALED_PRESSURE press_abs=1000.0 temp=2500
VIBRATION vibration=(0,0,0) clipping=(0,0,0)
```

EKF3 is allocated and its update/status path runs. Its status remains
`EKF_UNINITIALIZED` (0x0400) by design: for Plane,
`NavEKF3_core::InitialiseFilterBootstrap()` refuses bootstrap while
`assume_zero_sideslip()` is true and GPS status is below 3D lock. GPS
and compass are explicitly disabled in the Renode defaults because
neither is modeled yet; changing EKF source parameters cannot bypass
this fixed-wing safety gate.

A normal MAVLink arm request is rejected (`MAV_RESULT_FAILED`) and
produces useful remaining prearm work rather than an IMU-health error:

```text
Arm: 3D Accel calibration needed
Arm: AHRS: waiting for home
Arm: Battery 1 unhealthy
Arm: Waiting for RC
```

This is far enough for the GCS receive/BLHeli reproduction path. A
future GPS model can move EKF3 past its fixed-wing bootstrap gate, but
is not required for the Shared_DMA fault.

## Stage 3.5 status: causal issue #33926 reproduction complete

The reporter's stable-4.7.0 dump exposes the complete failure state. AP_BLHeli
was processing motor index 2 with `SERVO_BLH_MASK=0`, a boot-time
`motor_map={4,5,6}`, and `motor_mask=0x70`. The live Motor3 servo-function
lookup failed. `blheli_chan_to_output_chan()` ignored that failure and returned
its initialized fallback value, channel 0.

That mismatch explains the apparently corrupt DMA state without a DMA lifetime
bug. `serial_setup_output(channel=0, mask=0x70)` selects PWM group 0 as
`serial_group`, but its setup loop configures only groups intersecting mask
0x70. Groups 1 and 2 receive DMA handles while group 0 remains in PWM mode with
a null handle. The function nevertheless returns success. The first
`serial_write_bytes()` then calls `group0.dma_handle->lock()` and reaches
`Shared_DMA::lock_core()` through the null handle's `allocate` functor.

The Renode regression creates this inconsistency causally rather than writing a
DMA field. It configures output 5 as DShot300, waits until AP_BLHeli has cached
that output, changes the live BLHeli mask so Motor1 has no dynamic servo
assignment, and starts the normal firmware connection test. This is a compact
same-boot surrogate for the reporter's repeated mask and servo-function edits:
it exercises the same unchecked lookup and naturally creates the same null
PWM-group handle.

Build and run the vulnerable case with:

```sh
./waf configure --board BlitzWingH743 --enable-APJ_TOOL_PARAMETERS \
    --default-parameters=Tools/renode/params/pr33933.parm
./waf plane
Tools/renode/run.py BlitzWingH743 --vehicle arduplane \
    --reproduce-pr33933 vulnerable
```

The launcher reads symbol and parameter metadata from the selected ELF, so
addresses and object offsets are not tied to one build. It arms all hooks before
starting the MCU, waits for BLHeli initialization, applies the mapping change in
RAM, and pauses after the expected result. It only observes the selected PWM
group's DMA handle; it never changes it.

Observed vulnerable result:

```text
PR33933: changed SERVO_BLH_MASK from 16 to 0 after BLHeli init
PR33933: selected pwm_group DMA handle=0x0
PR33933: vulnerable build entered CrashCatcher
PR33933: CrashCatcher finished writing the dump
```

The vulnerable run naturally produced a CrashCatcher dump. CrashDebug
symbolized the relevant stack as:

```text
Functor<void, Shared_DMA*>::operator()       (null method pointer, r3=0)
Shared_DMA::lock_core
RCOutput::serial_write_bytes
AP_BLHeli::BL_SendBuf
AP_BLHeli::BL_ConnectEx
AP_BLHeli::run_connection_test
Plane::one_second_loop
```

That is the same core `Shared_DMA -> RCOutput -> AP_BLHeli` failure path as the
reporter's dump, although this run enters it through the built-in connection
test rather than MAVLink passthrough and Renode records the faulting call-site
PC rather than PC=0.

## Stage 4 status: timer update DMA complete

Renode's stock `STM32_Timer` schedules the programmed PSC/ARR overflows but
treats `TIMx_DIER.UDE` as an inert tag. `AP_STM32_Timer_UpdateDMA` subscribes to
the timer's overflow event and pulses the corresponding H743 DMAMUX request
while UDE is enabled. The board wires TIM1/3/4/5 to requests 15/27/32/59.

A no-fault BLHeli connection-test run reached `serial_write_byte()` and then
`RCOutput::dma_up_irq_callback()` without reaching `dma_cancel()`. This proves
that a timer update now advances NDTR, completes the transfer, and raises the
real firmware DMA-completion path. The stock timer still logs the DMAR write at
offset 0x4c as unhandled; modeling the individual pulse values is unnecessary
for transfer completion and will matter only when an ESC line model is added.

## Stage 4.5 status: root fix tested

`AP_BLHeli::get_output_channel()` now returns failure when the requested motor
does not exist, its servo-function lookup fails, or the resolved output is not
in the mask configured during initialization. All BLHeli serial, MSP motor, and
ESC-voltage consumers handle that failure instead of operating on output 1.
The mask check also prevents a live servo-function move from driving an output
that was never initialized for BLHeli; the existing BLHeli mask and output-type
parameters already require a reboot.

The same Renode stimulus against the fixed firmware stops before RCOutput:

```sh
Tools/renode/run.py BlitzWingH743 --vehicle arduplane \
    --reproduce-pr33933 fixed
```

```text
PR33933: changed SERVO_BLH_MASK from 16 to 0 after BLHeli init
PR33933: fixed build rejected stale motor mapping
```

A control mode leaves the mask and mapping valid:

```sh
Tools/renode/run.py BlitzWingH743 --vehicle arduplane \
    --reproduce-pr33933 valid
```

It reached the normal BLHeli serial path with `DMA handle=0x3002BA38`, proving
the fix does not block a configured DShot output. The fixed BlitzWingH743
ArduPlane image also completes its normal waf build.

The hwdef-generated harness was retested with a fresh image on 2026-08-07. The
fixed case again rejected the stale mapping before RCOutput, and the valid case
reached serial DMA with a non-null handle (`0x3002C9C8`). A fresh generated
KakuteF4 platform also reached `Copter::init_ardupilot()`.

## CubeOrange boot and MAVLink

CubeOrange originally stopped before `Copter::init_ardupilot()` with
`Config Error: Failed to update IO firmware`. The application hwdef declares
`IOMCU_UART USART6`, but the generated platform had no USART6 peer. After 32
register-read timeouts the FMU entered the IO bootloader uploader, failed to
synchronize, and called the fatal configuration-error loop. USART2 was already
the correct first hardware UART in `SERIAL_ORDER`; it never transmitted because
GCS initialization had not been reached.

`AP_IOMCU` now provides the normal IO register protocol as a USART child. The
generator reads the board's `io_firmware.bin` ROMFS entry, applies the hwdef
`AP_IOMCU_FW_FLASH_SIZE` (or its normal 0xF000 default), and computes the same
CRC32 including erased-flash padding as the FMU. This makes the model generic
for other IOMCU boards and leaves the bootloader uploader available as a real
failure path rather than bypassing the check in firmware.

The next CubeOrange gate was `HAL_VALIDATE_BOARD`. Its hwdef requires two
MS5611s and three different InvenSense IMUs. Shared models now cover MS5611 on
SPI and I2C, legacy InvenSense with a hwdef-derived WHOAMI, and banked
InvenSense-v2 with FIFO samples. The generator also creates an active-low SPI
multiplexer per bus because Renode's STM32 SPI controller accepts only one
child. Multiple hwdef sensor alternatives at one physical bus/CS location are
collapsed to the first declaration; devices on distinct chip selects are all
instantiated. WHOAMI values are taken from `HAL_VALIDATE_BOARD`, which handles
CubeOrange's internal `icm20948` device name that is physically an ICM20649.

A fresh CubeOrange run produced a decoded MAVLink heartbeat on SERIAL1/USART2
(system 1). Its 322752-byte `crashlog.img` remained entirely 0xFF. The file is
created eagerly from the ELF linker region as persistent erased flash, so its
existence alone is not evidence of a crash. MatekH743 was then rerun as a
regression and also produced a decoded system-1 heartbeat with the generated
SPI multiplexers and dual-bus MS5611 model.

CubeOrange parameter writes initially worked only until reboot. The RAMTRON
model deliberately ignored `FinishTransmission()` when connected directly to
Renode's STM32 SPI controller because the controller called it at hardware
TSIZE boundaries rather than chip-select deassertion. With the generated SPI
multiplexer those intermediate callbacks are suppressed and the only forwarded
finish is the real CS edge, so ignoring it left WREN/write transactions open and
`fram.img` erased. Ending and saving the FRAM transaction on that forwarded
finish fixed the problem. A MAVLink test set `FRAME_CLASS` from 0 to 1, observed
1 after a firmware reboot, then stopped and restarted Renode with the same state
directory and observed 1 again.

## Log

- 2026-08-07: fixed generated CubeOrange startup without adding board-specific
  files. Added a hwdef/ROMFS-derived IOMCU peer, MS5611 and InvenSense-v2
  models, exact validation WHOAMI selection, and per-bus SPI chip-select
  multiplexers. CubeOrange and MatekH743 both emitted decoded MAVLink
  heartbeats; CubeOrange's erased crash-log image remained untouched. Fixed
  RAMTRON transaction completion through the mux and verified a `FRAME_CLASS`
  update across both firmware reboot and a separate Renode process.
- 2026-08-07: replaced the KakuteF4 and BlitzWingH743 board REPL/RESC/parameter
  copies with run-time generation from the production application and
  bootloader hwdef compiler. All 232 STM32F405/STM32H743 targets with both
  hwdefs generate; actual peripheral coverage remains incremental. Fresh F405
  and H743 builds booted through the generated platforms, and the PR #33933
  fixed/valid controls passed.
- 2026-08-07: board state moved out of the generated build tree and made
  persistent across Renode processes. `run.py` defaults to
  `renode/<board>/`, creates 256 MiB FAT32 SD images, writes mapped STM32
  storage/crash sectors back on pause and exit, and generates an SPI
  RAMTRON model from `HAL_WITH_RAMTRON` plus `SPIDEV ramtron`. A real
  CUAV-Nora ArduPlane build initialized the generated FRAM and wrote its
  parameter header; MatekH743 wrote its storage backup into the host SD
  image and it remained present after a fresh Renode invocation.

- 2026-08-06: plan reviewed by codex (S4 lever corrections: BDMASK is
  boot-only, reboots clear locks[], ic_dma_fail not trivially reachable -
  same-boot poisoning required; apj_tool needs --enable-APJ_TOOL_PARAMETERS;
  DPS310 moved into stage 2; timer-DMA + CrashCatcher smoke test added as
  stage 3.5). BlitzWingH743 plane builds; reference backtrace extracted
  from the reporter's dump with gdb_crashdump.sh.
- 2026-08-06: stage 1 to `sdcard_retry` (above). run.py grew `--vehicle`
  and derives the erased-flash images from the build: storage sectors
  from hwdef.h `STORAGE_FLASH_PAGE` plus the family sector map, and the
  crash-log region from the ELF's `__crash_log_base__/__crash_log_end__`
  (0x081D4D20..0x08200000 for this build, matching the linker map).
- 2026-08-06: stage 1 complete. SDMMC IDMA pumps at the deferred data
  boundary, CMD17 returns the image-backed card to TRAN, FAT32 mounts,
  and the stock ELF reaches `Plane::init_ardupilot()`. run.py now
  supplies the required blank FAT32 card image automatically.
- 2026-08-06: stage 2 complete. An embedded SERIAL3 MAVLink default and
  DPS310 model take Plane through GCS UART setup. USART IDLE and DMA
  pointer-reload shims make partial RX buffers visible to ChibiOS. A
  live MAVLink test read the embedded default, changed MAV_SYSID to 42,
  rebooted the firmware, received a new system-42 heartbeat, and read
  the persisted value back from AP_FlashStorage.
- 2026-08-06: stage 3 complete. The ICM42688 model passes the
  Invensensev3 probe and checked-register/FIFO path. After calibration,
  Plane reaches standby with healthy gyro/accel bits, exact level/still
  RAW_IMU data, zero communication errors, and useful prearm residuals.
  EKF3 is running but correctly reports uninitialized at Plane's explicit
  3D-GPS-lock bootstrap gate; GPS is outside this stage and unnecessary
  for the target Shared_DMA reproduction.
- 2026-08-06: clarified the primary outcome: reproduce issue #33926 with a
  deterministic same-boot stimulus. CrashCatcher extraction is useful
  corroboration but is not the success criterion.
- 2026-08-06: an initial A/B tested the PR's claimed non-null owner/null
  deallocator state: vulnerable firmware entered CrashCatcher and the guarded
  firmware reported `dma_fail`. Later dump analysis below superseded that state
  as the reproduction target.
- 2026-08-06: reporter dump re-analysis corrected the injected state. The
  selected group's `dma_handle` is null and stream 0 has no lock owner; the
  null call is the handle's `allocate` functor, not a registered deallocator.
  The exact-state A/B now produces CrashCatcher on vulnerable firmware and
  `flow_of_control` on firmware with the PR allocator guard. Timer update-DMA
  requests also complete a real serial transfer through `dma_up_irq_callback`.
- 2026-08-06: the reporter's AP_BLHeli state identified the producer. A failed
  live Motor3 lookup silently fell back to output 1 while the cached BLHeli mask
  covered outputs 5-7. RCOutput selected the unconfigured output-1 group but
  initialized only mask-intersecting groups, naturally leaving the selected
  handle null. The Renode test now creates that mapping inconsistency without
  DMA injection. The AP_BLHeli fix rejects it and retains the valid serial-DMA
  path.
