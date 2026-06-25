# ANY_THREAD_ANY_CORE — RP2350 Per-Thread Core Assignment Strategy

## Goal
Find a multicore strategy that optimises our end result, main loop speed, and scheduler general performance.
Enable A/B performance testing of different thread-to-core assignments by changing
a single `#define` per thread, with the IRQ routing following automatically.

## ChibiOS SMP Facts

**No dynamic migration.** Each thread has an `owner` field (type `os_instance_t *`)
set at creation time and never changed. There is no global ready list, no work
stealing, and no load balancing. A thread created on Core0 stays on Core0 forever.

**Per-core ready lists.** Each core schedules only threads it owns. When an ISR on
Core0 wakes a thread owned by Core1, ChibiOS calls `chSysNotifyInstance(tp->owner)`
to send an IPI to Core1, which then reschedules locally.

**Virtual timers are core-local.** A `chVTSet()` call runs the callback on the core
that called it.

## The HAL Rule: Core that Starts, Serves

When a ChibiOS HAL driver is started (e.g. `spiStart()`, `i2cStart()`, `sdStart()`),
it configures the peripheral's IRQ on the NVIC of the **calling core**. Whichever
core calls `spiStart()` owns the SPI IRQs permanently.

This means: if a thread pinned to Core1 calls `spiStart()`, the SPI DMA completion
IRQ fires on Core1 — no IPI needed to wake the SPI thread. Thread and IRQ are
co-located automatically.

## The Design

### One config header

```c
// libraries/AP_HAL_ChibiOS/hwdef/common/rp2350_core_affinity.h

#define HAL_CORE_SPI0     0
#define HAL_CORE_SPI1     0
#define HAL_CORE_I2C0     0
#define HAL_CORE_UART     0
#define HAL_CORE_USB      0
#define HAL_CORE_RCIN     0
#define HAL_CORE_RCOUT    0
#define HAL_CORE_STORAGE  0
#define HAL_CORE_RATE     1
#define HAL_CORE_EKF      1
```

Change one line. The thread creation and IRQ routing both follow.

### Thread creation

```cpp
// Scheduler.cpp — uniform pattern
thread_create_pinned_to_core(spi_thread,     "SPI0",    stack, prio, arg, HAL_CORE_SPI0);
thread_create_pinned_to_core(rate_thread,    "rate",    stack, prio, arg, HAL_CORE_RATE);
thread_create_pinned_to_core(storage_thread, "storage", stack, prio, arg, HAL_CORE_STORAGE);
// etc.
```

### IRQ routing

No manual NVIC code required. The rule is: **the driver start call must happen from
within the thread that owns the peripheral**, not from global HAL init on Core0.

When the thread runs its first transaction on its assigned core, `spiStart()` /
`i2cStart()` / `sdStart()` runs on that core, and ChibiOS HAL routes the IRQ there
automatically.

## Current Driver Status

| Driver | Start call location | Status |
|---|---|---|
| SPI | `SPIBus::start_peripheral()`, called per-transaction from SPI thread | **Ready** — already lazy, no changes needed |
| I2C | `i2cStart()` in I2CDevice bus thread | Verify call site is inside thread, not Core0 init |
| UART | `sdStart()` from `UARTDriver::_begin()` | Verify `_begin()` runs on the UART thread's core |
| USB/OTG | `sduObjectInit()` / USB driver start | Likely Core0 global init — needs deferral |
| Storage/SD | `sdcStart()` | Verify call site |
| rcin/rcout | PWM LLD start | Likely Core0 global init — needs deferral |

## Why the SPI Driver Already Works

`SPIBus::dma_allocate()` contains the comment "nothing to do as we call `spiStart()`
on each transaction." The `start_peripheral()` → `spiStart()` call happens lazily
inside the SPI bus thread when the first transaction is acquired. Since the SPI thread
is already pinned to its assigned core, `spiStart()` runs on that core and the DMA
IRQ is routed there automatically.

Moving SPI to Core1 is therefore a 1-line change today.

## Interesting Experiments

```c
// Baseline (current)
#define HAL_CORE_SPI0  0
#define HAL_CORE_RATE  1

// Does SPI on Core1 help main loop Hz?
#define HAL_CORE_SPI0  1   // removes SPI DMA IRQ load from Core0

// Does rate on Core0 hurt? (regression baseline)
#define HAL_CORE_RATE  0

// Everything I/O on Core1, leaving Core0 for scheduler only
#define HAL_CORE_SPI0     1
#define HAL_CORE_I2C0     1
#define HAL_CORE_STORAGE  1
```

## IRQ Contention Note

Core0 is currently saturated (~100% load) primarily from SPI DMA IRQs firing at 1 kHz
for the IMU, plus USB CDC and timer callbacks. These IRQs fire on Core0 because Core0
runs global HAL init which calls the driver start functions. Moving a driver's start
call into its thread eliminates that IRQ from Core0's NVIC entirely — the gain is not
just thread scheduling, it is the raw ISR execution overhead disappearing from Core0.

## What This Is Not

This is not dynamic load balancing. ChibiOS does not migrate threads. The assignment
is static for the lifetime of a boot. Changing `HAL_CORE_xxx` requires a rebuild and
reflash. The benefit is rapid experimental iteration across boots, not runtime
adaptivity.

---

# SMP Stability Work — Bugs Found and Fixed

## Summary of Current State (branch: rp2350-v5-etc-dual-core)

| Item | Status |
|---|---|
| Core1 boots and runs ChibiOS | ✓ Working |
| Rate thread on Core1 @ ~988 Hz | ✓ Working |
| EKF thread on Core1 @ ~164–494 Hz (adaptive) | ✓ Working (relocated from Core0) |
| c1_vtable in SRAM9 (256-byte aligned) | ✓ Fixed |
| XIP lockout protocol (IBUSERR #2) | ✓ Implemented |
| XIP lockout deadlock (IRQ26 priority) | ✓ Fixed |
| Log_Write_GSF restored (was commented out) | ✓ Done (yawEstimator null-check in place) |
| main loop rate (target 400 Hz) | **360–367 Hz** (Config E: SPI on Core1 + DCM/8 + ekf_decim_min=2) |
| `main=700+Hz` INTERNAL ERROR crash | ✓ Fixed (root cause: c1_vtable bank conflict) |

---

## Bug 1: Core1 IBUSERR — `c1_vtable` in Striped SRAM

### Symptom
After ~5s of boot, `Perf: main=670Hz` appears (or similar 500–700 Hz value).  
GDB shows Core1 halted in `c1_sram_fault_handler` spinning `while(1){}`.  
WATCHDOG SCRATCH registers:
```
SCRATCH[1] = 0xBB000035  (Core1 fully booted)
SCRATCH[2] = 0xC1FA0001  (fault handler fired)
SCRATCH[3] = 0x00000100  (CFSR = IBUSERR, BusFault bit 8)
```

### Why `main=700+Hz` means INTERNAL ERROR, not performance
When ArduPilot raises an `AP_InternalError` (e.g. `flow_of_control`, `invalid_arg`),
the scheduler enters a fast-spinning empty loop. The main loop counter increments
rapidly with zero real work — hence very high Hz readings. A settled healthy rate is
500–600 Hz (post EKF -O2 and semaphore optimisations); target is 400 Hz.
Never interpret 700+ Hz as "running fast" — diagnose the internal error first.

### Root Cause
`c1_vtable` was declared as a 64-entry uint32_t array in striped SRAM (was at
0x20036B00 — striped bank 0). When Core1's TIMER0_ALARM1 interrupt fired (1 kHz),
Core1 fetched the exception vector from `c1_vtable[17]` (Vector44). If Core0 was
simultaneously accessing the same striped SRAM bank 0, the RP2350 bus fabric
returned DECERR → IBUSERR → BusFault → Core1 dead.

### Fix
Moved `c1_vtable` to **SRAM9 (0x20081000)** — Core1's dedicated I-CODE bus ("Scratch
Y"). SRAM9 is non-striped and physically separate from the main SRAM0-7 banks.
Zero bank-conflict risk for Core1 instruction/vector fetches.

```c
// Before:
static volatile uint32_t c1_vtable[64] __attribute__((aligned(256)));

// After (in c1_main.c):
#define c1_vtable ((volatile uint32_t *)0x20081000U)
```

**256-byte alignment:** VTOR requires 256-byte alignment. `0x20081000 & 0xFF = 0` —
alignment is guaranteed by the address itself, no `__attribute__((aligned(256)))` needed.

**SRAM9 layout:** Total 4 KB (0x20081000–0x20081FFF).  
First 256 bytes (64 × 4) = c1_vtable.  
Remaining 3840 bytes = available for future use.  
The `ram5` linker section / `rp2350_scratchy_sections.ld` covers SRAM9 but is
currently empty — the vtable is placed there via hard-coded address, not the linker.

### Verification
After fix, GDB confirms:
```
c1_fault_info[7] (VTOR at fault) = 0x20081000   ← SRAM9 ✓
c1_vtable[17] = 0x1015FFF5   (Vector44 handler, Thumb bit set) ✓
SCRATCH[2] = 0x00000000   (no fault) ✓
```

---

## Bug 2: Secondary IBUSERR — XIP Flash Disabled During Parameter Write

### Symptom
After the c1_vtable fix, IBUSERR persists on first boot. GDB shows:
```
SCRATCH[3] = 0x00000100  (CFSR = IBUSERR again)
stacked PC  = 0x1015FFF6  (inside Vector44, in FLASH)
```
Core1 was INSIDE the Vector44 handler, at its second instruction
(`bl __stats_increase_irq` at a flash address), when XIP became unavailable.

### Root Cause
On every flash erase/program, the RP2350 QMI controller temporarily disables XIP
(execute-in-place). Any instruction fetch by Core1 from flash during that window
returns DECERR → IBUSERR.

The flash writes come from **AP_FlashStorage** (parameter storage), which calls:
```
AP_Param::load_all()
  → stm32_flash_erasepage() / stm32_flash_write()
    → efl_lld_start_erase_sector() / efl_lld_program()
      → rp_flash_exit_xip()   ← XIP DISABLED
      → [erase/program]
      → rp_flash_enter_xip()  ← XIP RE-ENABLED
```

**AP_Logger is NOT the cause.** Laurel defaults have `LOG_BACKEND_TYPE 0`, disabling
all AP_Logger backends. AP_Logger_Flash uses an external SPI flash chip anyway; it
does not write to the internal XIP flash. The internal flash writes are exclusively
from AP_FlashStorage (parameter storage).

### The Hook Point
The EFL driver provides weak no-op hooks bracketing every XIP-off operation:
```c
// modules/ChibiOS/os/hal/ports/RP/LLD/EFLv1/rp_efl_lld.c
CC_WEAK void rpEflBeforeXipOff(void) {}
CC_WEAK void rpEflAfterXipOn(void) {}
```
These are called for every erase AND every page program — including the parameter
write at boot and every subsequent `AP_Param::save()` call.

---

## Fix 2: XIP Lockout Protocol

### Design
Core0 parks Core1 in SRAM before disabling XIP, then releases it after XIP is
restored. Uses RP2350 SIO doorbell (IRQ26, VectorA8) as the signal mechanism.

```
Core0  rpEflBeforeXipOff():
         c1_xip_lock = 1
         SIO->DOORBELL_OUT_SET = 1    ← rings Core1's bell (IRQ26)
         spin until c1_xip_lock == 2  (≤10ms timeout)
       [flash operation: XIP disabled → erase/program → XIP restored]
       rpEflAfterXipOn():
         c1_xip_lock = 0

Core1  c1_xip_lockout_handler()  [.ramtext, runs from SRAM]:
         SIO->DOORBELL_IN_CLR = 0xFF  ← de-assert IRQ line
         save NVIC ISER0/ISER1
         NVIC_ICER0 = NVIC_ICER1 = 0xFFFFFFFF   ← disable ALL Core1 IRQs
         c1_xip_lock = 2             ← ack: Core1 parked
         spin while c1_xip_lock != 0  [SRAM only — no flash fetches]
         restore NVIC ISER0/ISER1
         return via EXC_RETURN       ← hardware restores pre-IRQ state
```

### Key Implementation Details

**`c1_xip_lockout_handler` is a "fast interrupt"** (ChibiOS terminology):
- Priority 0 (default) — fires even during ChibiOS BASEPRI kernel lock
- No `OSAL_IRQ_PROLOGUE/EPILOGUE` — does not call any ChibiOS API
- Entire function in `.ramtext` — safe to execute with XIP disabled
- Installed at `c1_vtable[42]` = VectorA8 = SIO_BELL = IRQ26

**SIO doorbell register addresses (SIO base = 0xD0000000, banked per-core):**
```
SIO->DOORBELL_OUT_SET  0xD0000180  Core0 write → rings Core1's bell
SIO->DOORBELL_IN_CLR   0xD000018C  Core1 write → clears Core1's pending bells
```
(SIO struct layout: CPUID + GPIO + FIFO + SPINLOCK_ST + resvd + INTERP[2] + SPINLOCK[32]
 = 0x180 offset to DOORBELL_OUT_SET)

**`c1_xip_lock_ready` guard:** Core1 sets this to 1 after enabling IRQ26, which
happens immediately after `chInstanceObjectInit()`. If Core0 calls
`rpEflBeforeXipOff()` before Core1 has armed the protocol (very early boot),
the function returns immediately without attempting the lockout. This is safe
because Core1's ChibiOS tick (TIMER0_ALARM1) is not yet active during that window.

**Files changed:**
- `libraries/AP_HAL_ChibiOS/hwdef/common/board_rp2350.c` — defines `c1_xip_lock`,
  `c1_xip_lock_ready`, `rpEflBeforeXipOff()`, `rpEflAfterXipOn()` (all inside
  `#if defined(RP_CORE1_START) && RP_CORE1_START == TRUE`)
- `libraries/AP_HAL_ChibiOS/hwdef/Laurel/c1_main.c` — adds NVIC_ISER0/ISER1 and
  SIO_DOORBELL_IN_CLR macros; adds `c1_xip_lockout_handler` in `.ramtext`; installs
  it in `c1_vtable[42]`; enables IRQ26 and sets `c1_xip_lock_ready = 1` after
  `chInstanceObjectInit()`.

**IRQ26 priority (deadlock fix):** See Bug 3 section below. IRQ26 is set to
`CORTEX_MINIMUM_PRIORITY` so it is masked by ChibiOS BASEPRI during kernel locks.

**Symmetry note:** The current protocol only handles Core0→Core1 lockout (Core0
writing to flash, Core1 parked). If Core1 ever writes to flash, a symmetric
Core1→Core0 lockout would be needed: `rpEflBeforeXipOff()` would detect which core
it's running on via `SIO->CPUID`, ring the other core's bell, and the other core's
lockout handler would park Core0. Core1 does not currently write to flash, so this
is deferred.

### Rate Thread Latency Impact
Flash sector erase takes ~30–50 ms. During this window, Core1 is parked and its
ChibiOS tick (TIMER0_ALARM1) does not fire. The rate thread misses ~30–50 tick
periods. The tick timer re-arms immediately when Core1 resumes and TIMER0_ALARM1
is re-enabled. In practice: parameter writes happen only at first boot (or when the
user saves params via GCS), not during flight. The rate thread gap at boot is
acceptable.

**Future option — rate thread immune to XIP lockout:** A thread whose ENTIRE call
tree executes from SRAM (no flash code anywhere in the call path) would not need to
be parked during XIP-off and could continue running at 1 kHz even during flash
erases. The ArduPilot rate thread as currently written calls deep AP stack in flash,
so this is not feasible without a very large SRAM footprint. A minimal bare-metal
rate loop (MMIO reads + MMIO writes only) could achieve this.

---

## EKF Thread Relocation to Core1

The EKF thread (`AP_NavEKF3`) was moved from Core0 to Core1 as part of this work
(`thread_create_pinned_to_core` with `HAL_CORE_EKF = 1`). Rationale: EKF is a slow,
compute-heavy background thread. Moving it off Core0 frees Core0 for the main loop.

**Observed result:** Core0 main loop rate improved from ~140 Hz (Core0-only) to
296–305 Hz settled (all bugs fixed, EKF on Core1). Still below the 400 Hz target.
Core0 is still CPU-saturated by the flight-control fast loop (60.9%).

**`Log_Write_GSF` status:** Was temporarily commented out due to a crash with a
possibly-null `yawEstimator` pointer. Now restored with null-guard `if (yawEstimator
== nullptr) { return; }` in place. No crash observed. EKF adaptive decimation
(~164 Hz) keeps Core1 load at 65–68%, which reduces SMP spinlock contention vs the
previous full-rate EKF.

---

## Bug 3: XIP Lockout Deadlock — IRQ Priority 0 vs ChibiOS Spinlock

### Symptom
After the XIP lockout protocol was implemented, GDB shows both cores stuck:
```
Core0: port_spinlock_take() ← spinning, never returns
Core1: c1_xip_lockout_handler+86 ← spinning at "while (c1_xip_lock != 0)"
c1_xip_lock = 2  (Core1 has acknowledged, is parked)
```
Board is alive but Core0 cannot proceed — complete freeze. No MAVLink output.

### Root Cause: Priority-0 IRQ fires mid-ChibiOS critical section
The lockout handler was installed at IRQ26 with default priority 0 (the highest
Cortex-M33 priority — not maskable by BASEPRI). This allowed it to preempt Core1
even during ChibiOS kernel locks.

Deadlock sequence:
1. Core1 holds the ChibiOS SMP spinlock (`SIO->SPINLOCK[PORT_SPINLOCK_NUMBER]`)
   during a kernel lock
2. Core0 calls `rpEflBeforeXipOff()`: sets `c1_xip_lock=1`, rings Core1's doorbell
3. IRQ26 fires on Core1 at priority 0 — **fires even though BASEPRI is set** —
   preempting Core1 mid-critical-section
4. `c1_xip_lockout_handler`: disables all NVIC IRQs, sets `c1_xip_lock=2`, spins
5. **Core1 still holds the SMP spinlock** — the handler never released it
6. Core0 receives the ack (`c1_xip_lock=2`), then calls `chSysLock()` →
   `port_spinlock_take()` → spins forever waiting for a spinlock Core1 abandoned
7. **Deadlock**

### Fix: Set IRQ26 priority to `CORTEX_MINIMUM_PRIORITY`
RP2350 NVIC configuration:
- `CORTEX_PRIORITY_BITS = 4` → 16 priority levels (0 = highest, 15 = lowest)
- `CORTEX_MINIMUM_PRIORITY = 15` → 8-bit IPR byte = `15 << 4 = 0xF0`
- `CORTEX_BASEPRI_KERNEL = 0x20` (SMP mode: `CORTEX_PRIO_MASK(2)`)
- BASEPRI = 0x20 masks IRQs with priority byte ≥ 0x20 → our IRQ at 0xF0 is masked

Result: IRQ26 fires only when Core1 is NOT in a kernel lock → Core1 does not hold
the spinlock when the handler runs. Pending doorbell fires within microseconds after
Core1 exits the critical section (well within the 10 ms ack timeout).

```c
// In __c1_cpu_init(), after installing c1_vtable[42]:
// NVIC_IPR6 covers IRQs 24-27. IRQ26 is bits[23:16] of IPR6.
volatile uint32_t *nvic_ipr6 = (volatile uint32_t *)0xE000E418U;
const uint32_t min_prio = CORTEX_PRIO_MASK(CORTEX_MINIMUM_PRIORITY); /* 0xF0 */
*nvic_ipr6 = (*nvic_ipr6 & ~(0xFFU << 16U)) | (min_prio << 16U);
```

### Verified working
After fix, settled output:
```
Perf: main=234Hz rate=987Hz core0load:100% core1load:77%
C1: rate=988Hz ekf=247Hz ekf_dur=760us ekf_duty=41% decim=2
```
No crash. No INTERNAL ERROR. Core1 parked and unparked correctly during parameter
writes at boot.

---

## SRAM Map — Key Regions for Core1

| Region | Address | Size | Contents |
|---|---|---|---|
| Striped SRAM 0–7 | 0x20000000 – 0x2007FFFF | 512 KB | Main heap, stacks, .bss, .data |
| SRAM8 ("Scratch X") | 0x20080000 | 4 KB | Core0 dedicated I-CODE bus (`ram4`) |
| SRAM9 ("Scratch Y") | 0x20081000 | 4 KB | Core1 dedicated I-CODE bus (`ram5`) |
| `.ramtext` | 0x20001xxx | varies | SRAM-resident functions (fault handler, lockout handler) |

**SRAM9 allocation:**
```
0x20081000 – 0x200810FF  c1_vtable (256 bytes, 64 × uint32_t)
0x20081100 – 0x20081FFF  Available (3840 bytes)
```
The `.ramtext` section (striped SRAM bank 1 at ~0x20001314) holds:
- `c1_sram_fault_handler` — saves diagnostics on BusFault/HardFault
- `c1_xip_lockout_handler` — parks Core1 during flash writes
- `rp_flash_exit_xip`, `rp_flash_enter_xip`, and the full EFL flash driver
  (all already in `.ramtext` by the ChibiOS EFL driver)

---

## ChibiOS Tick Timer Assignment (RP2350 SMP)

| Core | Tick source | IRQ | ChibiOS priority | Vector |
|---|---|---|---|---|
| Core0 | TIMER0_ALARM0 | IRQ0 | 2 | Vector40 |
| Core1 | TIMER0_ALARM1 | IRQ1 | 2 | Vector44 |

TIMER0 is a free-running 64-bit microsecond counter. Each core's alarm fires when
`TIMER0->TIMERAWL` reaches the alarm value; the handler re-arms it for the next tick.
`TIMER0->TIMERAWL` is safe to read from either core without coordination (unlike
`TIMELR` which latches `TIMEH`).

---

## WATCHDOG SCRATCH Registers — Diagnostic Breadcrumbs

All survive SYSRESETREQ (warm reset). OpenOCD resets the chip when it detects a
lockup, wiping SRAM — but these persist.

| Register | Address | Written by | Meaning |
|---|---|---|---|
| SCRATCH[0] | 0x400D800C | `_unhandled_exception` | VTOR at exception time |
| SCRATCH[1] | 0x400D8010 | `c1_main()` | Core1 boot milestones (0xBB0000xx) |
| SCRATCH[2] | 0x400D8014 | `c1_sram_fault_handler` | 0xC1FA0001 if Core1 faulted |
| SCRATCH[3] | 0x400D8018 | `c1_sram_fault_handler` | CFSR value at fault |
| SCRATCH[4] | 0x400D801C | `c1_sram_fault_handler` | HFSR value at fault |
| SCRATCH[5] | 0x400D8020 | `c1_sram_fault_handler` | VTOR value at fault |

**Core1 boot milestones (SCRATCH[1]):**
```
0xBB000003  c1_main() entered
0xBB000031  after chSysWaitSystemState
0xBB000032  after chInstanceObjectInit  (TIMER0_ALARM1 enabled HERE)
0xBB000033  after VTOR capture
0xBB000034  about to chSysUnlock
0xBB000035  after chSysUnlock — Core1 fully operational
```

**Decoding CFSR = 0x00000100:** BFSR bit 8 = IBUSERR — instruction fetch bus error.
Most likely cause: Core1 fetched from XIP flash while XIP was disabled.

---

## c1_fault_info[] — Live SRAM Diagnostics

```c
// readable via GDB: x/8wx &c1_fault_info   or   p c1_fault_info
c1_fault_info[0]  CFSR
c1_fault_info[1]  HFSR
c1_fault_info[2]  MMFAR
c1_fault_info[3]  SFSR
c1_fault_info[4]  SFAR
c1_fault_info[5]  PSP at fault
c1_fault_info[6]  MSP at fault  (also used for c1_vtable[41] snapshot)
c1_fault_info[7]  VTOR at fault (also used as boot snapshot after chInstanceObjectInit)
```

---

## GDB Diagnostic Commands

### Start OpenOCD (use ports in 55xxx or 57xxx range to avoid auto-connect scripts)
```bash
~/openocd-pico/openocd \
  -s ~/openocd-pico/scripts \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg \
  -c "adapter speed 5000" \
  -c "gdb_port 55000; tcl_port 55001; telnet_port 55002" &
```

### GDB — halt without reset (CRITICAL: always --nx to suppress .gdbinit)
```bash
gdb-multiarch --nx build/Laurel/bin/arducopter
(gdb) target extended-remote :55000
(gdb) monitor halt
```

### Key inspection commands
```gdb
# Core1 fault state
x/8wx &c1_fault_info
p/x c1_boot_stage
p/x c1_xip_lock
p/x c1_xip_lock_ready

# WATCHDOG SCRATCH registers
x/6wx 0x400D800C

# Core1 vector table (in SRAM9)
x/64wx 0x20081000
# Vector44 = TIMER0_ALARM1 = c1_vtable[17]:
x/wx 0x20081044

# c1_vtable[42] = VectorA8 = SIO_BELL lockout handler:
x/wx 0x200810A8

# Core0/Core1 VTOR
# (connect to rp2350.dap.core1 for Core1's PPB registers)
info threads
thread 2         # switch to Core1
x/wx 0xE000ED08  # SCB->VTOR

# Stack frame at fault (MSP - 32)
x/8wx <msp_value - 32>
# stacked frame: R0 R1 R2 R3 R12 LR PC xPSR

# AP_InternalError bitmask — find address from ELF symbol
# (AP::internalerror() has no symbol in current context; use nm instead)
arm-none-eabi-nm build/Laurel/bin/arducopter | grep -i "internal_error\|internalerror"
# then: x/wx <addr>
```

### Decode CFSR
```
CFSR = 0x00000100  → BFSR.IBUSERR  (bus error on instruction fetch from XIP flash)
CFSR = 0x00000200  → BFSR.PRECISERR (precise data bus error, BFAR valid)
CFSR = 0x00020000  → UFSR.INVSTATE  (invalid EPSR.T bit — Thumb/ARM mismatch)
CFSR = 0x00010000  → UFSR.UNDEFINSTR (undefined instruction)
```

---

## Performance Observations

| Config | SPI | EKF | main Hz | core0 | core1 | EKF rate | Notes |
|---|---|---|---|---|---|---|---|
| Single-core | 0 | 0 | ~140 Hz | 100% | — | — | Core0 only |
| Core1 + rate | 0 | 0 | ~180 Hz | 100% | low | — | EKF still Core0 |
| A (baseline) | 0 | 1 | **296–302 Hz** | 100% | 68% | 164 Hz | Stable after t+75s. ekf_duty=28%, decim=6. |
| A+DCM/8+floor | 0 | 1 | **309–319 Hz** | 100% | 68% | 164 Hz | DCM backup at /8 rate; ekf_decim_min=2. EKF still decim=2 (→164 Hz). +14 Hz over A. |
| B (experiment) | 1 | 1 | **335 Hz** | 100% | 100% | ~100 Hz | No floor; Core1 saturated; EKF decimates far down. |
| D (experiment) | 0 | 1 | **225 Hz** | 100% | 85% | — | USB+UART+I2C+RCIN on Core1 → WORSE (SMP contention). |
| **E (current best)** | **1** | **1** | **360–367 Hz** | 100% | 97% | **61 Hz** | SPI on Core1 + DCM/8 + ekf_decim_min=2. Adaptive pushes decim to 8. EKF at 61 Hz (acceptable). +54 Hz over A. |
| Target | — | — | 400 Hz | <100% | — | — | Not yet achieved. Gap: ~35 Hz. |

**EKF adaptive decimation — 75-second convergence:** Config A shows a two-phase
boot profile. The EKF adaptive decimation takes ~75 s to fully converge:

```
t=0–75s:    ekf=247Hz, ekf_duty=40%, core1load=76%, main=245–251Hz  (transient)
t=75–∞:     ekf=164Hz, ekf_duty=28%, core1load=68%, main=296–302Hz  (settled)
```

**DO NOT benchmark before t=90s.** The 246 Hz readings in the first 75 s are real
but transient — the EKF is running at 247 Hz (decim=4) and Core1 SMP spinlock
contention is higher. After the EKF decimates to 164 Hz (decim=6), Core1 load drops
from 76% to 68%, SMP contention falls, and the main loop improves to 296–302 Hz.

The transition is sharp and reproducible (seen on multiple boots). The trigger appears
to be the EKF adaptive algorithm finally deciding ekf_duty=40% is too high and
halving the EKF rate, dropping duty to 28% and settling there.

**Core0 saturation:** Core0 at 100% is dominated by flight-control scheduler tasks
(see tasks.txt analysis below). Moving SPI0 to Core1 improves main from 296 Hz to
335 Hz (Config B/C) but saturates Core1 at 100% — the SPI bus thread + rate thread +
DMA IRQs together consume Core1 fully, causing EKF adaptive decimation (decim=4–5,
EKF drops to ~80–100 Hz).

**Config D failure — SMP spinlock contention is the fundamental bottleneck:** Moving
USB+UART+I2C+RCIN to Core1 (in addition to rate+EKF) increased Core1 load from 65%
to 85% and dropped main from 296 Hz to 225 Hz. Root cause: more threads on Core1 =
more ChibiOS context switches on Core1 = more `chSysLock/chSysUnlock` calls = more
SMP hardware spinlock operations. Every spinlock acquire/release on Core1 stalls
Core0 if Core0 is simultaneously in a lock. The USB driver is not lightweight in
terms of IRQ frequency — USB CDC generates frequent endpoint polling interrupts even
at low MAVLink bandwidth. Moving IRQ sources to Core1 reduced Core0 preemption
slightly but increased ChibiOS SMP overhead by more than it saved.

**Lesson:** Minimise threads on Core1, not maximise. Core1 should run the fewest
possible threads (currently: rate + EKF). Each additional Core1 thread adds spinlock
contention that penalises both cores.

**Current stable config (Config A):** `HAL_CORE_SPI0=0, HAL_CORE_EKF=1` → 296 Hz,
Core1 65–68% loaded. No saturation on either core. Flashed and confirmed.

**FSTRATE settings (do not change):**
```
FSTRATE_ENABLE = 1   (rate thread active)
FSTRATE_DIV    = 1   (rate thread at 1 kHz — same as IMU ODR)
SCHED_LOOP_RATE = 400  (Core0 main loop target — never lower this)
```

---

## Open Issues / Next Steps

| Item | Status |
|---|---|
| ~~Re-enable Log_Write_GSF~~ | ✓ DONE — null-guard in place, no crash |
| ~~GCS update rate reduction (50Hz → 25Hz on RP2350)~~ | ✓ DONE |
| Main loop 400 Hz target not yet met (best: 360–367 Hz Config E, gap ~35 Hz) | **Open — see BUGS.md PERF-002** |
| Lock-free EKF result sharing to eliminate `_rsem` stall in `read_AHRS` (~+40 Hz) | **Open — see BUGS.md PERF-003** |
| `InertialSensor::update` cross-core latency 405 µs → 156 µs target (~+18 Hz) | **Open — see BUGS.md PERF-004** |
| `AP_InternalError` bitmask not preserved across WD reset (invisible post-reboot) | **Open — see BUGS.md DIAG-001** |
| Symmetric XIP lockout (Core1 → Core0) needed if Core1 ever writes flash | **Deferred — see BUGS.md ARCH-001** |
| Rate thread XIP-lockout-immune (full SRAM call chain) to avoid tick gaps at boot | **Research — see BUGS.md ARCH-002** |

---

## Key Files

| File | Purpose |
|---|---|
| `hwdef/Laurel/c1_main.c` | Core1 startup, c1_vtable in SRAM9, fault handler, XIP lockout handler |
| `hwdef/common/board_rp2350.c` | Core0 board init, `rpEflBeforeXipOff/AfterXipOn`, XIP lockout state vars |
| `hwdef/common/flash.c` | `stm32_flash_erasepage` / `stm32_flash_write` — calls EFL driver |
| `modules/ChibiOS/os/hal/ports/RP/LLD/EFLv1/rp_efl_lld.c` | EFL driver: `rpEflBeforeXipOff/AfterXipOn` weak hooks |
| `modules/ChibiOS/os/hal/ports/RP/RP2350/hal_efl_lld.c` | RP2350-specific flash: `rp_flash_exit_xip`, `rp_flash_enter_xip` (.ramtext) |
| `modules/ChibiOS/os/hal/ports/RP/LLD/TIMERv1/hal_st_lld.c` | ChibiOS tick ISR handlers (Vector40=Core0, Vector44=Core1) |
| `hwdef/common/rp2350_core_affinity.h` | Per-thread core assignment `#define`s |
| `hwdef/Laurel/hwdef.dat` | Board config — `RP_CORE1_START TRUE`, `SCHED_LOOP_RATE 400`, `FSTRATE_*` |
| `AP_NavEKF3/AP_NavEKF3_Logging.cpp` | `Log_Write_GSF` restored — null-guard in place |
| `AP_NavEKF3/AP_NavEKF3_Logging.cpp` | `Log_Write_GSF` null-guard in place |



