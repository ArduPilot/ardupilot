# F1 IOMCU bootloader

This ChibiOS bootloader replaces the libopencm3 `px4io_bl` target for
STM32F100xB and STM32F103xB IOMCUs. CubeRedSecondary and its H7 IOMCU continue
to use `Tools/AP_Bootloader`.

## Build

From the repository root:

```sh
./waf configure --board iomcu --bootloader
./waf bootloader
```

The binary is `build/iomcu/bin/AP_Bootloader.bin`; the ELF is
`build/iomcu/bootloader/AP_Bootloader`. Use `--debug-symbols` for SWD
debugging without changing optimization. The linker enforces the existing
4 KiB reservation. Waf enables LTO for this target to fit ChibiOS and the
protocol into that space. Fat LTO objects allow the ChibiOS archive to be
indexed by the existing `arm-none-eabi-ar` tool.

Build only the `iomcu` bootloader target. The same binary serves all F100
and F103 IOMCUs, including 8 MHz and 24 MHz crystal variants and both DShot
and ordinary firmware. It uses HSI/2 multiplied by six, giving 24 MHz without
depending on the external crystal. Its RAM layout fits the F100's 8 KiB;
application stack pointers are accepted across the combined F100/F103 RAM
range, up to 20 KiB. On larger flash parts the exposed application area
remains 60 KiB, matching the existing FMU uploader and firmware CRC.

`Tools/scripts/build_bootloaders.py iomcu` builds and copies these artifacts
into `Tools/bootloaders/iomcu_bl.*`, replacing the existing files. A full run
of that script also includes `iomcu`. Building with Waf alone leaves the
tracked bootloader binaries unchanged.

## Compatibility

- Bootloader occupies `0x08000000..0x08000fff`; firmware starts at
  `0x08001000` and ends before `0x08010000`.
- USART2 uses PA2/PA3 at 115200 baud, 8N1. UART I/O is polled to avoid the
  space cost of serial queues; ChibiOS provides startup, clocks, GPIO,
  system time and flash operations.
- The legacy protocol reports revision 5, board ID 10, board revision 0,
  and a 61440-byte application area. Supported commands are GET_SYNC,
  GET_DEVICE (including vector-area reads), CHIP_ERASE, PROG_MULTI,
  GET_CRC and BOOT. Programming accepts aligned blocks up to 252 bytes,
  including the FMU uploader's 248-byte blocks.
- CRC32 uses polynomial `0xedb88320`, seed zero and no final XOR, over
  the entire application area, including erased padding. The deferred
  first word participates in this CRC.
- The first application word remains erased until BOOT, following an
  upload and CRC request. The host compares the CRC before requesting
  BOOT. Resetting during programming therefore leaves an unbootable image.
  A flash failure requires another erase; repeated erase discards any
  pending first word.
- BOOT accepts the FMU's 200 ms pause before its command terminator. The
  loader waits for UART transmission to finish before handing over.
  Handoff validates the stack pointer and Thumb reset vector, disables
  interrupts, restores HSI and peripheral reset state, and sets VTOR/MSP.

No change to `AP_IOMCU::upload_fw()` or `AP_IOMCU_CHIBIOS_BOOTLOADER` is
needed. That latter flag selects the different handshake used by the
normal ArduPilot bootloader on the H7 IOMCU.

## Safety button

PB5 is an active-high, floating input relying on the board's external
pull resistor, as in `px4io_bl`. It is sampled at startup. A high sample
latches an indefinite bootloader wait, even after the button is released.
An explicit BOOT command can still launch firmware while the button is
held. This preserves recovery through the FMU uploader.

With PB5 low, the loader waits 200 ms before attempting to boot. A valid
protocol command cancels this timeout. An invalid application stays in
the loader. PB15 blinks while idle. PWM outputs remain inputs throughout
bootloader operation.

## Tests and installation

Run the native protocol tests (requires a host C++ compiler):

```sh
python3 -m unittest discover -s libraries/AP_IOMCU/bootloader/tests -v
```

These compile the production parser with an emulated UART and flash,
using undefined-behavior sanitization. They exercise the FMU upload
sequence, CRC against Python's zlib, deferred commit, reset during an
upload, repeated and failed erase, failed programming, malformed packets,
bounds, and timeout cancellation.

Install through the IOMCU's SWD connector, using an MCU-appropriate
OpenOCD target. Back up the whole IOMCU flash before replacing its first
4 KiB. For the F103 IOMCU on Pixhawk6X, with an ST-Link:

```sh
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
  -c init -c 'reset halt' \
  -c 'dump_image /tmp/iomcu-backup.bin 0x08000000 0x10000' \
  -c 'flash write_image erase build/iomcu/bin/AP_Bootloader.bin 0x08000000 bin' \
  -c 'verify_image build/iomcu/bin/AP_Bootloader.bin 0x08000000 bin' \
  -c 'reset run' -c shutdown
```

The FMU's normal IOMCU update protocol writes only the application; it
cannot install this bootloader.
