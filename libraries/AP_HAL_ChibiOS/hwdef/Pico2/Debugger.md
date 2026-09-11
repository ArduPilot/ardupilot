# Pico2 / RP2350 SWD Debugger Setup

For full step-by-step instructions see the Claude skill:
`.claude/commands/pico2-hardware.md`

This file is a quick-reference summary.

---

## Hardware

Two Pico2W boards required:
- **Debugger**: flashed with `debugprobe_on_pico2.uf2` (provides CMSIS-DAP SWD + UART bridge)
  - Download: https://github.com/raspberrypi/debugprobe/releases
- **Target**: runs ArduPilot firmware

### Wiring (debugger → target)

| Debugger (board pin / GPIO) | Target signal | Target (board pin / GPIO) |
|-----------------------------|---------------|---------------------------|
| pin 3 / GND                 | GND           | GND — mandatory           |
| pin 4 / GPIO2               | SWCLK         | 3-pin debug header (left) |
| pin 5 / GPIO3               | SWDIO         | 3-pin debug header (right)|
| pin 6 / GPIO4 (UART0 RX)   | console RX←TX | pin 2 / GPIO1             |
| pin 7 / GPIO5 (UART0 TX)   | console TX→RX | pin 1 / GPIO0             |

Target 3-pin debug header centre of board: **SWCLK / GND / SWDIO** (left→right).

---

## OpenOCD

The generic mechanics - launching OpenOCD, the WSL/Windows case, GDB, halting,
recovery - are in the `/swd-debug` skill. Only the RP2350 specifics are here.

RP2350 needs a pico-aware OpenOCD; the stock distro package is too old. Raspberry
Pi publish builds at https://github.com/raspberrypi/pico-sdk-tools/releases. On
Linux also install `libhidapi-hidraw0`.

The configs are `interface/cmsis-dap.cfg` and `target/rp2350.cfg`, and the probe
enumerates as VID:PID `0x2e8a:0x000c`.

```bash
OOCD=<path to openocd>            # do not assume a path, check first
$OOCD -c "gdb port 50000" -c "tcl port 50001" -c "telnet port 50002" \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000"
```

A working attach looks like this - note both cores are examined:

```
Info : SWD DPIDR 0x4c013477
Info : [rp2350.cm0] Cortex-M33 r1p0 processor detected
Info : [rp2350.cm1] Cortex-M33 r1p0 processor detected
Info : Listening on port 50000 for gdb connections
```

`Error: cannot read IDR` means the target is unpowered - the probe powers itself,
the target does not.

---

## Flash via SWD (Laurel/Pico2)

```bash
$OOCD -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000" \
  -c "init; reset halt" \
  -c "flash write_image erase build/Laurel/bin/arducopter.bin 0x10010000" \
  -c "reset run; exit"
```

Flash `.bin` to `0x10010000` (FLASH_RESERVE_START_KB=64 → app starts at 64 KB offset).
Do **not** use `_with_bl.hex`.

---

## GDB Live Diagnosis

**Always use `--nx`** — `.gdbinit` contains `mon reset halt` which resets the board on connect.

```bash
gdb-multiarch --nx build/Pico2/bin/arducopter
```
```gdb
(gdb) target extended-remote :50000
(gdb) mon halt
(gdb) info threads
(gdb) bt
```

Note: `arm-none-eabi-gdb` is broken on Ubuntu 24.04 (missing libncurses.so.5).
Use `gdb-multiarch` instead.

---

## Build

```bash
./waf configure --board=Laurel   # or --board=Pico2
./waf copter -j$(nproc)
```

See `Laurel/README.md` or `Pico2/README.md` for board-specific details.
