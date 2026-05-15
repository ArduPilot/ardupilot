---
name: pico2-hardware
description: "Raspberry Pi Pico2 / RP2350 hardware interaction for the ArduPilot port. Use when: flashing firmware via SWD or UF2; starting or restarting OpenOCD; connecting GDB to live hardware; live inspecting memory/registers; diagnosing USB CDC serial silence; adding DEV_PRINTF/print statements to trace boot crashes; diagnosing UART/GPIO FUNCSEL issues; halting vs resetting the board; using --nx with gdb; reading /dev/ttyACM* output; recovering from lost comms."
argument-hint: "What do you need to do? (flash / gdb / openocd / uart-debug / monitor-serial / print-debug)"
---

# Pico2 / RP2350 Hardware Skill

## Repository Safety

`git worktree` commands are approval-gated.

Before running any `git worktree add`, `git worktree remove`, `git worktree move`, `git worktree prune`, or any equivalent worktree-management command, the agent must first tell the human:
- the exact directory/path that will be created, removed, or changed
- the branch or commit that worktree will use
- the reason for using a worktree instead of the current checkout

Do not run the command until the user/operator/admin explicitly approves it.

## Hardware Setup

Two Pico2W boards are used:
- **Debugger** (labeled): flashed with `debugprobe_on_pico2.uf2` — provides CMSIS-DAP SWD + UART bridge
- **Target**: runs ArduPilot firmware

### Wiring (debugger → target)

Pin numbering conventions — always specify which system you mean:
- **GPIO N** (or **GP N**): logical RP2350 GPIO number — used in firmware/hwdef
- **board pin N**: physical header pin on Pico2W PCB (1–20 left column, 21–40 right column, USB at top, component side facing you)
- **RP2350 package pin N**: bare die pad (QFN-80), only relevant for carrier board PCB design

| Debugger GPIO (board pin) | Target signal | Target GPIO (board pin) | Notes |
|---|---|---|---|
| GPIO2 (board pin 4) | SWCLK | 3-pin debug header | SWD clock |
| GPIO3 (board pin 5) | SWDIO | 3-pin debug header | SWD data |
| GND (board pin 3) | GND | GND | Common ground — **mandatory** |
| GPIO4 (board pin 6) / UART0RX | console RX←TX | GPIO1 (board pin 2) | UART bridge |
| GPIO5 (board pin 7) / UART0TX | console TX→RX | GPIO0 (board pin 1) | UART bridge |

The target's SWD 3-pin debug header (centre of board, left→right) is: SWCLK / GND / SWDIO.

---

## OpenOCD

### Start (always background this)

```bash
~/openocd-pico/openocd \
  -c "gdb_port 50000" \
  -c "tcl_port 50001" \
  -c "telnet_port 50002" \
  -s ~/openocd-pico/scripts \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg \
  -c "adapter speed 5000" &
```

### Good output (means it's working)

```
Info : SWD DPIDR 0x4c013477
Info : [rp2350.dap.core0] Cortex-M33 r1p0 processor detected
Info : Listening on port 50000 for gdb connections
```

### "Error connecting DP: cannot read IDR"
Target is not powered. Plug in the target USB cable.

### Lost comms / hung OpenOCD
```bash
pkill -f openocd          # kill existing process
# re-run the start command above
```

---

## Flash Workflows

There are **two separate flash paths** — bootloader (one-time) and app firmware (everyday update).

Before any step that requires touching the target board, the agent must stop and
explicitly prompt the human for that action. Do not assume the human will unplug,
re-plug, or hold BOOTSEL without being asked.

---

### 1. Bootloader — one-time BOOTSEL flash

The bootloader is built once and loaded via BOOTSEL/UF2. It is NOT updated by `--upload`.

```bash
# Build the bootloader
./waf configure --board=Pico2 --bootloader --debug
./waf bootloader -j12
```

Then ask the human:
> "Please hold the BOOTSEL button on the Pico2 while plugging in the USB cable, then release. Tell me when the device appears as a mass-storage drive."

Do not start the bootloader upload until the human confirms the board is in BOOTSEL mode.

Once in BOOTSEL mode, flash via `--upload` (triggers picotool UF2 path automatically):
```bash
./waf configure --board=Pico2 --bootloader
./waf bootloader --upload
```

---

### 2. App Firmware — `./waf copter --upload` (everyday path)

This uses `uploader.py` which speaks the ArduPilot MAVLink bootloader protocol over USB CDC.
**Requires the AP_Bootloader to already be installed (see above).**

**IMPORTANT: This path requires a human to unplug and re-plug the USB cable.**

Before running `--upload`, always tell the human:
> "Please unplug the Pico2 USB cable and plug it back in now."

Do not run `uploader.py` or `./waf ... --upload` until the human confirms they have done the re-plug.

Wait for `/dev/ttyACM*` to re-appear, then run:
```bash
./waf copter --upload
# or manually:
python3 Tools/scripts/uploader.py \
    --port /dev/ttyACM1,/dev/ttyACM0 \
    build/Pico2/bin/arducopter.apj
```

Expected output:
```
Found board bd,0 bootloader rev 5 on /dev/ttyACM1
Bootloader Protocol: 5
ChipDes:
  family: RP2350
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.
EXIT: 0
```

The full flash takes about 60–90 seconds. Use `timeout 120` if calling manually.

**Port locking error** (`[Errno 11] Could not exclusively lock port`): another process (mavproxy, cat, etc.) has the port open. Kill it first.

---

### 3. App Firmware via SWD (developer/debug path — no unplug required)

Use this when OpenOCD is already running and you don't want to disturb the running system,
or when the AP_Bootloader is not installed.

```bash
gdb-multiarch --nx --batch \
  -ex "target extended-remote :50000" \
  -ex "mon halt" \
  -ex "mon reset halt" \
  -ex "load build/Pico2/bin/arducopter" \
  -ex "mon reset run" \
  -ex "quit" \
  build/Pico2/bin/arducopter
```

Expected: `Transfer rate: ~113 KB/sec` — ~15 seconds for a 1.4 MB image.

Always flash the `.elf` / no-extension ELF (not `.uf2` or `.bin`).

**Note:** `arm-none-eabi-gdb` is broken on Ubuntu 24.04 (missing libncurses.so.5). Use `gdb-multiarch` instead.

---

## GDB Live Diagnosis

### CRITICAL: always use `--nx`

`/home/buzz/ardupilot/.gdbinit` contains `mon reset halt`.  
**This resets the board on every standard GDB connect**, masking the real live state.  
Always use `--nx` for diagnostics:

```bash
gdb-multiarch --nx build/Pico2/bin/arducopter
# then manually:
(gdb) target extended-remote :50000
(gdb) mon halt          # halt without reset
```

### Halt without reset via telnet (no GDB needed)

```bash
nc localhost 50002      # openocd telnet
> halt
> targets              # show state
```

### Useful GDB inspection commands

```gdb
# General state
info threads
bt                          # backtrace current thread

# Memory: read a struct field
p serial0Driver._writebuf.available()
p serial1Driver._device_initialised
p serial1Driver.uart_thread_ctx

# Read raw memory address
x/4wx 0x2000db18

# Show USB CDC state
p SDU1.state              # SDU_READY = good
p SDU1.config->usbp->state  # USB_ACTIVE = good
```

---

## USB CDC Serial Debugging (print statements)

**USB CDC serial WORKS** on Pico2 and is the primary debugging tool for boot crashes.
The device emits both MAVLink frames and plain-text `DEV_PRINTF` / `hal.console->printf` output.
The port changes ACM number on every reboot (ttyACM1 → ttyACM2 → etc.), so use the stable
symlink `/dev/serial/by-id/usb-ArduPilot_Pico2_*-if00` instead.

### Read boot output across reboots

```python
import serial, time, glob, os

BY_ID = '/dev/serial/by-id/usb-ArduPilot_Pico2_9EE4ECE8FA06028D6EAD2FF9-if00'

def find_port():
    if os.path.exists(BY_ID):
        return BY_ID
    ports = sorted(glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else None

all_data = b''
t_end = time.time() + 30          # monitor for 30 s across reboots
while time.time() < t_end:
    port = find_port()
    if not port:
        time.sleep(0.1); continue
    try:
        s = serial.Serial(port, 115200, timeout=0.3)
        t_close = time.time() + 5  # read one cycle then reconnect
        while time.time() < t_close and time.time() < t_end:
            d = s.read(256)
            if d:
                all_data += d
                print(d.decode('utf-8', errors='replace'), end='', flush=True)
        s.close()
    except Exception as e:
        time.sleep(0.2)
```

### Boot sequence visible on USB serial

During normal startup the following text appears in this order:
```
\n                          # ChibiOS/ArduPilot boot marker
<MAVLink binary frames>     # HEARTBEAT etc. (binary, not text)
No Compass backends available
Init Gyro                   # AP_InertialSensor gyro calibration starts
<\n>                        # calibration loop DEV_PRINTF("\n") at end
ArduCopter Vx.x.x ...       # version banner = successful full boot
```

If the device reboots after `Init Gyro` but before the version banner, the crash is
happening inside `AP_InertialSensor::_init_gyro()` or something it calls (e.g. IMU
driver update loop, SPI transaction, scheduler, DMA).

### Add print statements to narrow down a crash

Use `DEV_PRINTF` for debug prints in ArduPilot C++ code (compiled out on final builds,
always emits to hal.console in debug builds). Use `hal.console->printf` when you need
it to always emit regardless of build type.

**In C++ (libraries or vehicle code):**
```cpp
// Breadcrumb prints — add around the suspected crash site
DEV_PRINTF("STAGE A\n");
// ... suspect code ...
DEV_PRINTF("STAGE B val=%d\n", (int)some_value);
```

**In ChibiOS HAL (AP_HAL_ChibiOS/):**
```cpp
// Use AP_HAL::panic() for fatal errors — prints to console then halts+resets
AP_HAL::panic("PIOUART: SM%u failed to start\n", (unsigned)sm_idx);

// Use chprintf to the SD1 (debugprobe UART) if USB isn't up yet
// (only available early in boot via hal_debug_printf)
```

**After adding prints, always:**
1. `./waf configure --board=Pico2 --debug && ./waf copter -j12`
2. Flash via SWD (OpenOCD telnet `program ... verify reset`)
3. Monitor USB output with the Python script above — port changes each reboot

### Example: tracing a boot-time crash

The `**` that appeared after `Init Gyro` in the Pico2 boot crash was the watchdog
firing inside `AP_InertialSensor::_init_gyro()`. To narrow it down:

```cpp
// In AP_InertialSensor.cpp, _init_gyro():
DEV_PRINTF("Init Gyro A\n");   // before loop
for (int16_t j = 0; ...) {
    DEV_PRINTF("Init Gyro B j=%d\n", j);  // each calibration iteration
    update();   // <-- if crash is here, B never prints for j > first
}
DEV_PRINTF("Init Gyro done\n");
```

Then monitor USB — last print before reboot identifies the exact line.

---

## Serial / USB Monitor

ArduPilot maps:
| Port | Device | GPIO |
|---|---|---|
| SERIAL0 / USB | `/dev/ttyACM0` or `ttyACM1` | USB connector |
| SERIAL1 | UART0 | TX=GP12, RX=GP13 |
| SERIAL2 | UART1 | TX=GP10, RX=GP11 |
| SERIAL3 | PIOUART0 | TX=GP14, RX=GP17 |

```bash
# Detect USB CDC port (stable symlink — survives reboots)
ls /dev/serial/by-id/usb-ArduPilot_Pico2*

# Quick health check — should see "Init ArduCopter" text
timeout 5 cat /dev/ttyACM1 | head -c 200 | xxd

# MAVLink stream (human readable)
mavproxy.py --master=/dev/ttyACM1 --baudrate=115200
```

---

## MAVLink SERIAL_CONTROL — Correct Flag Values

**Pymavlink constants** (confirmed from `pymavlink.dialects.v20.common`):

| Flag | Value | Meaning |
|---|---|---|
| `REPLY` | 1 | FC-originated response — **if set, FC returns immediately without acting** |
| `RESPOND` | 2 | FC echoes received data back |
| `EXCLUSIVE` | 4 | Lock port exclusively |
| `BLOCKING` | 8 | Block waiting for data |
| `MULTI` | 16 | Multi-packet response |

**Common mistake:** `EXCLUSIVE=1<<0=1` sets the REPLY bit → `handle_serial_control()` hits early return at line ~40 of `GCS_MAVLink/GCS_serial_control.cpp`:
```cpp
if (packet.flags & SERIAL_CONTROL_FLAG_REPLY) { return; }
```
This means `begin()` is **never called** on the target UART — completely silent failure.

**Correct flags for open+respond**: `EXCLUSIVE | RESPOND = 4 | 2 = 6`

**Correct device IDs** (from `common.xml`):
```python
SERIAL_CONTROL_SERIAL0 = 100   # SERIAL0/USB
SERIAL_CONTROL_SERIAL1 = 101   # SERIAL1/UART0
SERIAL_CONTROL_SERIAL2 = 102   # SERIAL2/UART1
```

---

## Build Commands

```bash
cd /home/buzz/ardupilot

# Configure (first time or after hwdef changes)
./waf configure --board=Pico2 --debug

# Build copter
./waf copter -j12

# Bootloader only
./waf configure --board=Pico2 --debug --bootloader
./waf bootloader -j12
```

Build output: `build/Pico2/bin/arducopter` (ELF, ~1.5 MB)

---

## UART GPIO FUNCSEL Fix (UARTDriver.cpp)

RP2350 requires explicit `FUNCSEL=2` (UART) on GPIO pads before calling `sioStart()`.
Without it, the pad stays in default GPIO mode and UART is silent.

Fix location: `libraries/AP_HAL_ChibiOS/UARTDriver.cpp`, in `_begin()`, SIO path:
```cpp
// Set GPIO function to UART (FUNCSEL=2) before sioStart — RP2350 requirement
if (sdef.tx_line != 0) {
    palSetLineMode(sdef.tx_line, PAL_MODE_ALTERNATE_UART);
}
if (sdef.rx_line != 0) {
    palSetLineMode(sdef.rx_line, PAL_MODE_ALTERNATE_UART);
    palLineSetPushPull(sdef.rx_line, PAL_PUSHPULL_PULLUP);
}
sioStart((SIODriver*)sdef.serial, &siocfg);
```

---

## Memory Map (confirmed addresses)

| Symbol | Address |
|---|---|
| `serial0Driver` (SERIAL0/USB) | `0x2000d9a0` |
| `serial1Driver` (SERIAL1/UART0) | `0x2000db18` |
| `serial2Driver` (SERIAL2/UART1) | `0x2000dc90` |
| `_serial_tab` (ROM) | `0x1017c890` |

---

## Test Script

`Tools/debug/test_uart_pico2.py` — MAVLink SERIAL_CONTROL exerciser.

Usage:
```bash
source zephyrproject/.venv/bin/activate
python3 Tools/debug/test_uart_pico2.py --port /dev/ttyACM1
python3 Tools/debug/test_uart_pico2.py --port /dev/ttyACM1 --loopback  # needs GPIO loopback wire
```

Loopback wiring for hardware verification:
- UART0: GP12 → GP13
- UART1: GP10 → GP11

---

## Common Pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| USB CDC goes silent after SERIAL_CONTROL | Wrong REPLY flag set (bit0=1) triggers early return | Use `flags = EXCLUSIVE\|RESPOND = 6` |
| GDB connect resets board unexpectedly | `.gdbinit` has `mon reset halt` | Use `gdb --nx` for live diagnosis |
| OpenOCD "cannot read IDR" | Target not powered | Plug in target USB |
| UART completely silent | FUNCSEL not set to UART before sioStart | Add `palSetLineMode(ALTERNATE_UART)` in `_begin()` |
| `serial1Driver._device_initialised = false` | `begin()` was never called | SERIAL_CONTROL flags wrong, or device ID wrong |
| No heartbeats on USB after test | Write buffer empty (not a freeze) | Board is running fine; check MAVLink connection |
| Device reboots after "Init Gyro" | Crash inside `_init_gyro()` / IMU driver | Add `DEV_PRINTF` breadcrumbs around each calibration step; monitor USB serial |
| ACM number changes each reboot | USB re-enumeration on every boot | Use `/dev/serial/by-id/usb-ArduPilot_Pico2_*-if00` stable symlink |
| USB print output mixed with MAVLink binary | Both share SERIAL0/USB | Use `xxd` or Python `serial` to view raw; text prints are readable between binary frames |
