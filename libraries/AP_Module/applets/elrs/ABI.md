# Experimental SX1280 native callback ABI v1

The callback is a trusted native function registered through `sx1280_register`.
It uses an opaque 4096-byte, 8-byte-aligned state, a 128-byte input, and a
256-byte output. Integers are little-endian; times are wrapping microseconds.
The return value and output byte 0 must both be 1. Buffers must not be retained.

Initialization runs on the module worker. Subsequent callbacks run on the
radio worker at SPI priority + 1. A lifetime mutex pins module code/RW data during each
call; unload removes the callback before reclaiming memory. Main-thread UART
work exchanges one outstanding request/reply, never invokes module code. UART operations run outside the lifetime mutex. Stream
work is deferred when less than 500 us remains before the radio wakeup.

## Input

| Offset | Meaning |
| --- | --- |
| 0 | Operation: 0 initialize, 1 radio event, 2 stream exchange |
| 4 | Current host time, u32 |
| 20..31 (initialize) | UID1 u32, UID2 u32, model u16, binding request u8, version u8 |
| 1 (radio) | Event: 0 none, 1 RX, 2 TX complete |
| 2 (radio) | Hardware healthy |
| 3 (radio) | Result: 0 none, 1 reset succeeded, 2 hardware failure, 3 TX deadline expired, 4 TX started |
| 8, 12 (radio) | IRQ timestamp and IRQ overrun counter, u32 |
| 16..19 (radio) | RSSI i8, raw SNR i8, RX length, button state |
| 36 (radio) | Actual TX start completion timestamp, u32; valid with result 4 |
| 20..35 (radio) | Raw received packet, at most 16 bytes |
| 1..3 (stream) | Downstream length, previously accepted upstream bytes, flush completed |
| 8, 12 (stream) | UART TX and RX free space, u32 |
| 20..81 (stream) | Downstream bytes, at most 62 |
| 84 (stream) | Generation of the previous stream response, u32 |

## Output

| Offset | Meaning |
| --- | --- |
| 1 | Actions: reset=1, configure=2, RX=4, TX=8, standby=16 |
| 2 | Receiver error code |
| 3 | Flags: binding=1, learned UID=2, LED=4, stream active=8, flush=16, accept downstream=32, consume binding request=64 |
| 4 | Wakeup delay relative to **input time**, u32; host deducts callback/SPI elapsed time |
| 8, 12, 16 | Frequency register u32, sync word u32, CRC seed u16 |
| 18..24 | Packet length, FLRC flag, SF encoding, coding, preamble, inverted IQ, power dBm i8 |
| 25, 26..41 | TX length and packet |
| 42, 43, 44..105 | Downstream consumed flag, upstream length and bytes |
| 106..111 | Learned UID |
| 112, 113 | Channel count and link quality scaled to 0..255 |
| 116, 120, 124 | Frame sequence, channel validity deadline, last input timestamp (u32) |
| 128, 132 | Failsafe repeat interval and timeout (u32) |
| 136..167 | Sixteen PWM channels, u16 |
| 168, 172, 176 | TX requested, TX completed, TX timeout counters (u32) |
| 180..235 | Fourteen tracker counters (same order as `tracker.rs`) |
| 240 | Absolute TX deadline, u32 |
| 244 | Transport generation, u32 |
| 248..250 | Rate wire ID, LQ percent, tracker state |
| 251 | Error-text length, 0..62; when error byte 2 is nonzero, bytes 44..105 contain text and upstream length byte 43 must be zero |

The host rejects invalid masks, packet/channel lengths and wait values. UART
acknowledgements are generation-tagged so stale bytes cannot enter a new radio
session. The AP_Periph publication sink retains the last module-declared
failsafe timeout and latches expiry even if the module exits or time wraps.
This ABI is experimental; firmware and applet are built and flashed together.


## Module-owned configuration and persistence

`periph.sx1280_configure(word0, word1, word2)` stages 12 opaque little-endian
initialization bytes for the calling module before callback registration. Only
that owner can replace the staged data or install the callback; active callbacks
cannot be reconfigured. Runtime cleanup revokes staged configuration and callback.
`periph.sx1280_snapshot(offset)` copies a coherent 16-byte window from the
256-byte report, under the callback lifetime lock, for the registered owner.
Offsets above 240 and other callers are rejected. These SDK calls run on the
module worker, never inside the RF callback. Previous import slots are retained.

The Rust worker reads binding completion and persists the UID using AP_Param;
the host RC sink interprets only generic maintenance/error status, channel values,
sequence and validity/failsafe timing fields. Bit 0 of report byte 3 requests
maintenance mode; the ELRS applet uses it during binding.

Report error byte 2 reserves 255 for host callback detachment. After an active
owner exits, the host clears the payload and retains this error until a new
callback is successfully installed. Unrelated cleanup does not clear it.

Error text is supplied by the applet and forwarded as a literal CAN log message.
The host reserves error 7 for worker creation failure and 255 for callback detachment.
The ELRS applet reports configuration (1), receiver start (2), radio initialization (3),
radio operation/retry (8), and exhausted recovery (9) errors. CAN logs are emitted
on error-code changes, with an INFO message when the error clears.
