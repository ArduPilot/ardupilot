---
name: flash-debug-hardware
description: "Flashing and debugging Laurel or Pico2 (RP2350) hardware for the ArduPilot port. Use when: flashing app firmware via OpenOCD SWD or UF2; starting or restarting OpenOCD; connecting GDB to live hardware; live inspecting memory/registers; diagnosing USB CDC serial silence; adding DEV_PRINTF/print statements to trace boot crashes; halting vs resetting the board; reading /dev/ttyACM* output; recovering from lost comms; choosing between Laurel and Pico2 flash paths."
argument-hint: "Board and action? (laurel-flash / pico2-flash / openocd / gdb / uart-debug / monitor-serial / print-debug)"
---

# Flashing and Debugging Laurel / Pico2 (RP2350) Hardware

## Repository Safety

`git worktree` commands are approval-gated.

Before running any `git worktree add`, `git worktree remove`, `git worktree move`, `git worktree prune`, or equivalent, the agent must first tell the human:
- the exact directory/path that will be created, removed, or changed
- the branch or commit that worktree will use
- the reason for using a worktree instead of the current checkout

Do not run the command until the user explicitly approves it.

---

## Board Identity

| Board | Build target | USB serial symlink |
|---|---|---|
| **Laurel** | `--board=Laurel` | `/dev/serial/by-id/usb-ArduPilot_Laurel_B8CE48E2E19D881E67A96B1B-if00` |
| **Pico2** | `--board=Pico2` | `/dev/serial/by-id/usb-ArduPilot_Pico2_*-if00` |

---

## Hardware Setup (Pico2 as debugger)

Two Pico2W boards are used:
- **Debugger** (labeled): flashed with `debugprobe_on_pico2.uf2` — provides CMSIS-DAP SWD + UART bridge
- **Target**: runs ArduPilot firmware (Pico2 or Laurel carrier board)

### Wiring (debugger → target)

Pin numbering conventions — always specify which system you mean:
- **GPIO N** (or **GP N**): logical RP2350 GPIO number — used in firmware/hwdef
- **board pin N**: physical header pin on Pico2W PCB (1–20 left column, 21–40 right column, USB at top, component side facing you)

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

### Binary and scripts location

```bash
~/openocd-pico/openocd          # binary
~/openocd-pico/scripts/         # config scripts
```

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
pkill -f openocd
# re-run the start command above
```

---

## Flash Workflows

### Laurel — App Firmware via OpenOCD SWD (preferred / always works)

**IMPORTANT:** Flash the `.bin` file at `0x10010000` (app offset = `FLASH_RESERVE_START_KB 64`).
Do **NOT** use `arducopter_with_bl.hex` — it contains segments at the STM32 address 0x08000000
that cause verify failures on RP2350.

Do **NOT** use `uploader.py` / `--upload` for Laurel. It sends a MAVLink reboot-to-bootloader
command and then waits for the board to re-enumerate as a bootloader device. If the board does not
respond automatically, it hangs forever printing "If the board does not respond, unplug and re-plug
the USB connector." — which requires a human to physically press the reset button on the target.
That is often not possible (board mounted in vehicle, user remote, etc.).
If `uploader.py` is already stuck, use OpenOCD to reset the target, then flash via SWD.

```bash
~/openocd-pico/openocd \
  -s ~/openocd-pico/scripts \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg \
  -c "adapter speed 5000; program build/Laurel/bin/arducopter.bin verify reset exit 0x10010000"
```

Expected output ends with:
```
** Programming Finished **
** Verify Started **
** Verified OK **
** Resetting Target **
shutdown command invoked
```

### Pico2 — Bootloader (one-time BOOTSEL flash)

The bootloader is built once and loaded via BOOTSEL/UF2. It is NOT updated by `--upload`.

```bash
./waf configure --board=Pico2 --bootloader --debug
./waf bootloader -j12
```

Then ask the human:
> "Please hold the BOOTSEL button on the Pico2 while plugging in the USB cable, then release. Tell me when the device appears as a mass-storage drive."

Do not start the bootloader upload until the human confirms the board is in BOOTSEL mode.

Once in BOOTSEL mode:
```bash
./waf configure --board=Pico2 --bootloader
./waf bootloader --upload
```

### Pico2 — App Firmware via `--upload` (MAVLink bootloader path)

**Requires AP_Bootloader already installed. Requires a human to unplug and re-plug the USB cable.**

Before running, tell the human:
> "Please unplug the Pico2 USB cable and plug it back in now."

Do not run until the human confirms the re-plug.

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
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.
EXIT: 0
```

Full flash takes ~60–90 seconds. Use `timeout 120` if calling manually.

**Port locking error** (`[Errno 11] Could not exclusively lock port`): another process has the port. Kill it first.

### Pico2 — App Firmware via SWD (no unplug required, debug path)

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

Always flash the ELF (no extension), not `.uf2` or `.bin`, when using GDB load.

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
info threads
bt                          # backtrace current thread

p serial0Driver._writebuf.available()
p serial1Driver._device_initialised
p serial1Driver.uart_thread_ctx

x/4wx 0x2000db18            # read raw memory

p SDU1.state                # SDU_READY = good
p SDU1.config->usbp->state  # USB_ACTIVE = good
```

---

## USB CDC Serial Debugging (print statements)

USB CDC serial WORKS on both boards and is the primary debugging tool for boot crashes.
The device emits both MAVLink frames and plain-text `DEV_PRINTF` output.
The ACM number changes on every reboot — use the stable symlinks above.

### Read boot output across reboots

```python
import serial, time, glob, os

BY_ID = '/dev/serial/by-id/usb-ArduPilot_Laurel_B8CE48E2E19D881E67A96B1B-if00'
# For Pico2: BY_ID = '/dev/serial/by-id/usb-ArduPilot_Pico2_9EE4ECE8FA06028D6EAD2FF9-if00'

def find_port():
    if os.path.exists(BY_ID):
        return BY_ID
    ports = sorted(glob.glob('/dev/ttyACM*'))
    return ports[0] if ports else None

all_data = b''
t_end = time.time() + 30
while time.time() < t_end:
    port = find_port()
    if not port:
        time.sleep(0.1); continue
    try:
        s = serial.Serial(port, 115200, timeout=0.3)
        t_close = time.time() + 5
        while time.time() < t_close and time.time() < t_end:
            d = s.read(256)
            if d:
                all_data += d
                print(d.decode('utf-8', errors='replace'), end='', flush=True)
        s.close()
    except Exception as e:
        time.sleep(0.2)
```

### Add print statements to narrow down a crash

```cpp
// ArduPilot C++ — compiled out in release, always emits in debug
DEV_PRINTF("STAGE A\n");
DEV_PRINTF("STAGE B val=%d\n", (int)some_value);

// Always emits regardless of build type
hal.console->printf("msg\n");

// Fatal — prints then halts+resets
AP_HAL::panic("PIOUART: SM%u failed to start\n", (unsigned)sm_idx);
```

After adding prints: rebuild → flash via SWD → monitor USB output.

---

## Build Commands

```bash
cd /home/buzz/ardupilot

# Laurel
./waf configure --board=Laurel
./waf copter -j12
# output: build/Laurel/bin/arducopter.bin

# Pico2
./waf configure --board=Pico2 --debug
./waf copter -j12
# output: build/Pico2/bin/arducopter (ELF)
```

---

## Runtime Profiling — tasks.txt / threads.txt

### Hands-off tasks.txt fetch (reboot + double-fetch, no user interaction)

```bash
timeout 30 mavproxy.py \
  --master=/dev/serial/by-id/usb-ArduPilot_Laurel_B8CE48E2E19D881E67A96B1B-if00 \
  --cmd="reboot" \
  --cmd="repeat add 10 ftp get @SYS/tasks.txt -"
```

What happens:
- Board reboots immediately
- After 10s: **first fetch** — all MIN/MAX/AVG/OVR/TOT fields are zero (discard this)
- After 20s: **second fetch** — real accumulated data (use this)
- After 30s: `timeout` closes the connection

**Why two fetches?** The first `ftp get` allocates the `_task_info` buffer inside
`AP_Scheduler::task_info()` and prints it before any scheduler tick has written into it.
All fields are zero. The second fetch 10s later has real data.

Always discard the first result. Never analyse the first fetch output.

### threads.txt (thread stack high-water marks)

```bash
# one-shot, no reboot needed — stack marks accumulate from boot
timeout 15 mavproxy.py \
  --master=/dev/serial/by-id/usb-ArduPilot_Laurel_B8CE48E2E19D881E67A96B1B-if00 \
  --cmd="ftp get @SYS/threads.txt -"
```

### freemem (actual runtime heap after full init)

```bash
timeout 15 mavproxy.py \
  --master=/dev/serial/by-id/usb-ArduPilot_Laurel_B8CE48E2E19D881E67A96B1B-if00 \
  --cmd="ftp get @SYS/free_memory.txt -"
```

This is the definitive heap number — more accurate than the static heap budget script,
which cannot see runtime allocations (EKF state matrices, MAVLink FTP session buffers,
expanding strings, file I/O buffers, etc.).

---

## MAVLink SERIAL_CONTROL — Correct Flag Values

| Flag | Value | Meaning |
|---|---|---|
| `REPLY` | 1 | FC-originated response — **if set, FC returns immediately without acting** |
| `RESPOND` | 2 | FC echoes received data back |
| `EXCLUSIVE` | 4 | Lock port exclusively |
| `BLOCKING` | 8 | Block waiting for data |
| `MULTI` | 16 | Multi-packet response |

**Common mistake:** `EXCLUSIVE=1<<0=1` sets the REPLY bit → `handle_serial_control()` early-returns, `begin()` never called on target UART — completely silent failure.

**Correct flags for open+respond**: `EXCLUSIVE | RESPOND = 4 | 2 = 6`

**Device IDs:**
```python
SERIAL_CONTROL_SERIAL0 = 100
SERIAL_CONTROL_SERIAL1 = 101
SERIAL_CONTROL_SERIAL2 = 102
```

---

## Common Pitfalls

| Symptom | Cause | Fix |
|---|---|---|
| `uploader.py` loops "If the board does not respond, unplug..." forever | Requires human to physically press reset on target — often not possible (board mounted, user remote) | Use OpenOCD SWD flash instead; no physical access needed |
| OpenOCD verify fails at 0x08000000 | Used `_with_bl.hex` which has STM32 address segments | Use `.bin` at `0x10010000` for Laurel |
| GDB connect resets board unexpectedly | `.gdbinit` has `mon reset halt` | Use `gdb --nx` for live diagnosis |
| OpenOCD "cannot read IDR" | Target not powered | Plug in target USB |
| UART completely silent | FUNCSEL not set to UART before `sioStart` | Add `palSetLineMode(ALTERNATE_UART)` in `_begin()` |
| USB CDC goes silent after SERIAL_CONTROL | Wrong REPLY flag set (bit0=1) triggers early return | Use `flags = EXCLUSIVE\|RESPOND = 6` |
| ACM number changes each reboot | USB re-enumeration on every boot | Use `/dev/serial/by-id/usb-ArduPilot_*-if00` stable symlink |
| `arm-none-eabi-gdb` crashes on Ubuntu 24.04 | Missing libncurses.so.5 | Use `gdb-multiarch` instead |
| Device reboots after "Init Gyro" | Crash inside `_init_gyro()` / IMU driver | Add `DEV_PRINTF` breadcrumbs; monitor USB serial |
