# Plan: analog OSD on RPI_UAVFC, ported from Betaflight's PICO OSD

Status: **phases 0 to 3 done and committed, and running on a camera.** An
NTSC camera and VTX give a steady overlay; PAL is still untested for want of a
PAL camera. Phase 4 is part done - the SPI counters are measured and clean,
the flight numbers are not. Source is `../betaflight/src/platform/PICO/osd/`.

The board has the analog front end already - camera and VTX ports - and the
hwdef reserves GPIO21 `OSD_W`, GPIO22 `OSD_EN`, GPIO23 `OSD_SYNC`, which is the
right shape for a genlocked overlay. Betaflight's PIO program counts incoming
hsyncs (`firstDisplayLine`, `skipHsyncs`) and has a separate sync-measuring
program, so this is a true MAX7456-style overlay rather than standalone video
generation.

## Scope: MAX7456 emulation only

AP_OSD already renders elements into a character buffer and already ships
MAX7456 fonts in ROMFS, in the same 12x18 2bpp format Betaflight uses. So the
only thing missing is the back half: character buffer -> font blit ->
framebuffer -> PIO scan-out.

| File | Lines | Take? |
|---|---|---|
| `osd_pico.c` | 965 | Yes, the scan-out, PIO and DMA setup |
| `osd_tx.pio` / `.pio.h` | 258 / 194 | Yes, unmodified if possible |
| `osd_pico_internal.h` | 171 | Yes, trimmed |
| `font_betaflight.c` | 4903 | No - use AP_OSD's ROMFS fonts |
| `osd_elements_pico.c` | 806 | No - AP_OSD renders elements |
| `osd_element_ah.c` | 133 | No - same |
| `fb_osd_pico.c` | 316 | No - pixel-mode API, not needed for char cells |

That drops roughly 6100 of the 24300 lines outright, and the font file is the
largest single item.

## The resource budget

Three scarce resources, all of which have to be traded at boot.

### PIO: does not fit alongside NeoPixel

Measured from the generated header:

| Program | Instructions | Concurrent? |
|---|---|---|
| `osd_tx_pal` | 31 | one of these |
| `osd_tx_ntsc` | 31 | one of these |
| `osd_count_sync` | 25 | no - swapped in and out, `osd_pico.c:327/369` |

So the peak is **31 instructions and 1 state machine**.

Current allocation, from DEVELOPMENT.md:

| Block | Owner | SMs free | Instr free | GPIOBASE |
|---|---|---|---|---|
| PIO0 | PIOUART0, PIOUART1 | 0/4 | UART programs | 16 |
| PIO1 | NeoPixel | 3/4 | 28 | 0 |
| PIO2 | DShot | 0/4 | 3 | 0 |

**31 > 28.** PIO1 is the only block with a free state machine and the only one
with GPIOBASE 0, which GPIO21/22/23 require. PIO0 and PIO2 have no state
machines at all. So OSD and NeoPixel cannot coexist, and OSD needs the block to
itself: 31 of 32 slots, leaving one.

Dropping NeoPixel is close to free on this board. There is no LED fitted - J2
is a bare 3-pin header - so the loss only affects a user who has plugged a strip
in.

### RAM: 53 KB, and it must come off the heap

`osd_pico.c` declares three static framebuffers of
`PICO_OSD_BUF_LENGTH` = 92 x 288 = 26,496 B each: `osdBufferBackground`,
`osdBuffer1W`, `osdBuffer2W`. 79.5 KB as written.

`osdBufferBackground` supports Betaflight's static/dynamic element split, which
AP_OSD has no concept of. Drop it and the cost is **53 KB** for the A/B pair,
which double-buffers against the scan-out DMA and should be kept.

These must be **heap allocated at OSD init, not static**. A boot-time selectable
feature with static buffers pays the cost whether or not it is selected, which
defeats the point. RP2350 has no data cache so ordinary SRAM is DMA-capable and
no bounce buffer is needed. Current free heap is about 230 KB.

**As built there is no field buffer at all.** Lines are generated from the
character buffer as the DMA asks for them, so the cost is two 92 byte line
buffers rather than 26,496 bytes, and they are members rather than heap.
Measured: the OSD went from 43,424 bytes to 17,352, of which 13,824 is the
font.

That matters more than the plan assumed, because free heap when AP_OSD
initialises is about 119 KB, not the 230 KB quoted below - that figure is the
heap section, and the vehicle has already taken most of it by then.

The cost is a DMA completion interrupt per line, 15.6 kHz on PAL, serviced on
core1. It fires about two thirds of the way through the line being displayed,
since the transfer completes once all 23 words are in the FIFO rather than
once they are clocked out, leaving roughly 30 us to point the channel at the
next line and build the one after. Rendering a line is about 90 table lookups,
call it 2 us.

### DMA: two channels, claimed the way UARTDriver already does

One channel as built, not the two Betaflight uses: the second serves the
background buffer, which went with it. Claim with
`RP_DMA_CHANNEL_ID_ANY` and a real failure path, which is the pattern
`UARTDriver.cpp:547/712` and the ADC driver already use, and the documented fix
for the incident where a fixed-id allocation silently killed the GPS.

Note the pool is tighter than it was: since the SPI config cache landed, SPI0
and SPI1 hold four channels permanently rather than freeing them per
transaction.

## What actually has to be written

`osd_pico.c` is not a file to copy in. It has **92 pico-sdk call sites** -
`pio_add_program`, `pio_claim_unused_sm`, `sm_config_set_*`,
`dma_channel_configure`, `irq_set_exclusive_handler` and so on - and this port
does not use pico-sdk at all. `RCOutput_pico.cpp:670` writes `INSTR_MEM`
directly and DMA goes through ChibiOS `dmaChannelAllocI`.

The algorithm ports across intact: the scan-out state machine, vsync handling,
the A/B swap, the incremental renderer. The setup and teardown layer is a
rewrite against ChibiOS and bare registers. `RCOutput_pico.cpp` already does
exactly this by hand for both WS2812 and DShot, so there is a local pattern to
follow rather than a design to invent.

Budget phase 1 accordingly - it is the longest phase, not a quick checkpoint.

## The font is a specified transform, not an assumption

AP_OSD's ROMFS fonts are `NVM_RAM_SIZE * 256` = 54 x 256 = 13,824 bytes
(`AP_OSD_MAX7456.cpp:31,170`), which is the same geometry as Betaflight's
`fontData[18*3*256]`: 12x18, 2bpp, 54 bytes per character. Only the encoding
differs, and `mcm2h.py` specifies it:

| MCM (MAX7456) | Meaning | PICO framebuffer |
|---|---|---|
| `00` | black | `01` |
| `01` | transparent | `00` |
| `10` | white | `11` |
| `11` | transparent | `00` |

then reverse the 8 bits of each byte, because MCM stores pixels left to right
as high to low while the PIO shifts LSB first for wire order.

Applied once at load over 13.8 KB, so about ten lines. `OSD_FONT` and the rest
of AP_OSD's font machinery then work unchanged, and `font_betaflight.c` - the
largest file in the source tree at 4903 lines - is not needed.

## Design decisions

**Renderer runs on core1, not core0.** Core0 is at 45-47% and its load is the
documented gate on SD throughput; core1 is at 36-40% with a single
flight-critical thread. The renderer is already time-sliced
(`OSD_DRAWSCREEN_TIME_LIMIT_US` 20), which is the right shape for consuming
slack. Pin it below the rate thread so the rate thread always preempts it. This
is deliberately the opposite of reducing core1 rates to pay for the feature.

Consequence to accept: core1 is parked during XIP-off flash writes, so the
renderer stalls for up to about 3.6 ms.

**That was written for the field buffer and does not survive the per-line
renderer.** With a field buffer the scan-out DMA coasts on memory the CPU is
not touching and a park really is invisible. Per line, the DMA needs the
interrupt every 63.5 us, so a park starves it outright: measured at 2 to 4 ms,
which is 30 to 60 lines against the 17 us the FIFO holds. This is the price of
the 26 KB, and it is not free. What the driver does about it is under "the
state machine consumes what it is not given" below.

The renderer is a thread again, as this section always intended - see "blocks,
not lines" below. The per-line rework had put it in the completion interrupt,
where no thread priority could preempt it.

**Boot-time selection, not live switching.** PIO instruction memory can be
reprogrammed at runtime - Betaflight does it - but switching between LED and
video output while armed has no use case and would mean quiescing a DMA-fed
state machine in flight.

**All three resources fail the same way.** Acquire early, all or nothing, and
send a GCS message naming what was given up. The two worst debugging sessions on
this board, the GPS DMA and the `spi_fail` prearm, were both a resource quietly
not being there while the software believed it was.

**Vsync and DMA IRQs are enabled from core1.** RP2350's NVIC is per core, so an
interrupt fires on whichever core enabled it. The renderer is on core1, so the
vsync and DMA completion interrupts have to be enabled there too or the buffer
swap lands on one core and the render on the other. ChibiOS has the vectors -
`RP_PIO1_IRQ_0_HANDLER` is `Vector84`, IRQ 17 - but nothing in ArduPilot uses a
PIO interrupt today, so this is first use. Note the `HAL_CORE_SPI1`
investigation was about exactly this class of problem.

**Parameter.** `OSD_TYPE` currently runs 0-5 (`AP_OSD.h:549`). Add 6 for this
backend. That touches `AP_OSD.h`, the parameter metadata and the backend
factory - shared AP_OSD code rather than anything board specific, so it is
upstream visible and wants to be clean. NeoPixel yields when `OSD_TYPE` selects it; the decision has to be made
from the parameter before either driver initialises, because NeoPixel currently
initialises first and first-come would mean OSD never wins.

## Phases

Each phase ends somewhere testable, so a failure is attributable.

**0. Resource broker. Done**, but pulled forward into phase 1 rather than done
first: the technical risk was all in the translation, so proving the scan-out
configured correctly came first and the arbiter followed once it was provably
needed. `RP2350_pio1.{h,cpp}`. Verified both ways - OSD_TYPE 5 leaves PIO1
untouched, OSD_TYPE 6 hands it over.

Knows all three claimants. PIOUART2 and PIOUART3 are fixed to PIO1 in the
config table, so a board with more than two PIO UARTs would have put them on
top of whichever of the OSD and the LED driver had won it. Wiring UARTs to
PIO1 is a property of the board, not a runtime choice, so it settles the
question at build time and outranks OSD_TYPE. Not reachable here - RPI_UAVFC
has two, both on PIO0.

**1. Scan-out bring-up. Done** except the camera-dependent part.
`OSD_pico.{h,cpp}`. Every register reads back as intended: CLKDIV exactly
0x00030000, OUT base 21 count 2, IN base and JMP pin 23, wrap 0-30, FIFO joined
and shifting right. The state machine parks at instruction 2 waiting for sync,
which is correct with no video present.

Signed off on a camera since: 14027 lines/s, which is 234 x 59.94 exactly, and
the field runs to line 233. *Remaining: the same on PAL, which needs a PAL
camera. NTSC is detected rather than configured, so PAL costs no code, only a
test.*

**2. Character blitter. Done**, and verified without a camera - see below.

**3. AP_OSD backend. Done.** `OSD_PICO = 6` in AP_OSD's enum, the `@Values`
metadata and the factory case, then a backend implementing the five pure
virtuals: `write`, `init`, `flush`, `is_compatible_with_backend_type`,
`get_backend_type`. It inherits `load_font_data()`, which falls back from the
microSD to ROMFS, so doing it retires both the font scaffolding in the bring-up
thread and the magic 6 sitting in the broker. *Checkpoint: OSD_TYPE 6 gives a
working OSD; OSD_TYPE 5 gives NeoPixel back and the framebuffer's heap back.*

**4. Measurement. Part done.** The SPI counters and the internal error count
are measured and clean with the OSD running; the flight numbers are not. Below.

## What the build taught us

**The ASCII code is the glyph index.** `AP_OSD_MAX7456::write()` is
`frame[y][x] = *text` with no translation, and AP_OSD's ROMFS fonts are laid
out to match, so index 65 renders an A. There is no character map to port.

**The geometry divides exactly.** A 12 pixel cell at two bits per pixel is 3
bytes, a cell row is 18 lines, and 16 rows is the 288 line PAL field. So a blit
is a byte aligned 3 by 18 copy with no shifting anywhere, and 30 cells is 90 of
the 92 bytes in a line.

**The font conversion is one lookup per byte.** A 256 byte table, not a
transformed 13.8 KB copy of the font. The pair values look transposed because
reversing the whole byte afterwards fixes the pixel order and the order within
each pair at the same time.

**Verification needs no camera.** The framebuffer is SRAM at a known address,
so rendering a known string and dumping it over SWD - then decoding it under
the framebuffer's own pixel rules - tests the font transform, the cell
geometry, the line stride and the dirty flush in one go. This is how phase 2
was signed off, and it is how phase 3 should be too.

Two mistakes worth not repeating:

- **`is_system_initialized()` is the wrong gate for anything wanting a
  parameter.** It only goes true at the very end of `AP_Vehicle::setup()`, long
  after `init_ardupilot()` has set up the output groups. Using it made NeoPixel
  always lose PIO1 whatever OSD_TYPE said, and RCOutput takes a refusal as
  permanent. `AP_Param::initialised()` is the right one: `load_parameters()` is
  `AP_Vehicle.cpp:342`, `init_ardupilot()` is `:449`.
- **The IO process table is eight entries shared with the whole vehicle, and
  `register_io_process()` drops the ninth silently.** A one-shot init has no
  business taking a permanent slot. Use a thread.

## What the camera taught us

Nothing in the line pipeline had ever executed before the first camera was
plugged in, and all three defects were in it.

**`dmaChannelAllocI()` does not enable the channel interrupt.** It enables the
NVIC vector; the channel's bit in `INTE0`/`INTE1` is `dmaChannelEnableInterruptX()`,
a separate call. Without it the transfers run and nothing is ever told, so the
scan-out emitted one line per field. `UARTDriver.cpp:742` had it right all
along.

**A completed channel cannot be re-armed by setting `EN`.** `serve_interrupt()`
writes `CTRL_TRIG = READ_ERROR|WRITE_ERROR` before calling back, which takes
`DATA_SIZE`, `INCR_READ` and `TREQ_SEL` with it. Re-arming with `EN` alone
restarted the channel byte-wide, non-incrementing and paced off PIO0's DREQ -
a screenful of repeated garbage. The mode has to be rewritten per transfer,
which is what `UARTDriver.cpp:1336` does. Diagnosed by reading `CTRL_TRIG` back
over SWD as `0x0400a001` and finding `TREQ_SEL` zeroed; no amount of source
reading would have found it as fast.

**The state machine consumes what it is not given.** The line loop is `pull
noblock`, so a word the FIFO could not supply is not waited for - the OSR is
reused and the loop moves on. Two consequences. Starvation paints the `X`
register, which holds the remaining line count, rather than going transparent.
And because each line consumes exactly 23 words either way, every missed word
displaces the rest of the field by 16 pixels - 1.33 character cells - with no
way to find the phase again until the next field. "Shifted right by a few
characters" is the visible signature of two or three missed words. The driver
now blanks the remainder of a field once the FIFO has run dry, so it reads as a
flicker rather than as corruption, and flushes the FIFO at every field start so
a short field cannot displace the next one.

**The line interrupt must not run from XIP.** The eight word FIFO is 17 us of
slack and a cache miss can exceed it. Every OSD function was in flash while the
ChibiOS half of the path, `Vector6C` and `serve_interrupt`, was already in
SRAM. Moving the six functions above it into RAMFUNC2 took FIFO underruns from
5 per 10 s to none. This is the same failure the Scratch Y registry records for
bidirectional DShot, and it is the reason the font was kept out of flash - the
argument applied to the code too, which nobody noticed until it was measured.

**PAL and NTSC really do differ in one instruction.** `set y, hshiftA [hshiftC]`
is 26/[12] against 25/[8], encoding to `0xEC5A` and `0xE859`; `firstDisplayLine`
differs but `skipHsyncs` works out to 16 for both. So the standard is a single
word plus a reseed of the line count, which the program takes from ISR through
`mov x, isr`. `init()` counts fields for 400 ms and picks - 20 against 24 is a
wide margin - and falls back to the caller's choice when no camera is present.
Betaflight's separate sync-measuring program is not needed for this.

**What is left is the XIP parks.** Underruns now correlate 1:1 with
`rp2350_xip_park_count`, at roughly 3 per 40 s and 2 to 4 ms each. No code can
run on core1 during one. They also freeze the rate loop, which
`board_rp2350.c` already flags as a suspect for glat jitter, so this is a board
issue the OSD happens to make visible rather than an OSD issue.

## Blocks, not lines

The line pipeline was rewritten once more after the camera bring-up, because
5.79% of core1 was being spent somewhere no thread could preempt.

**The DMA transfer is a word stream, not a sequence of lines.** The state
machine pulls 23 words per line and waits for hsync between them; nothing in
the DMA knows where a line ends. Re-arming per line followed from rendering
per line, not from anything the hardware wanted. So a transfer can carry any
number of lines, and the completion interrupt rate is ours to choose.

Nine lines per block - half a character cell - and three buffers: one being
scanned out and up to two rendered and waiting.

| | per line | per block of 9 |
|---|---|---|
| Completion interrupt | 14 kHz | 1.56 kHz |
| Non-preemptible core1 time | 5.79% | 0.79% |
| Worst case an IMU completion waits | 30 us | 26 to 37 us |
| Renderer's deadline | 17 us | 1144 us |
| Buffers | 184 B | 2484 B |

**Why nine and not eight.** The cell is 18 lines, so a block of 9 covers its
top or bottom half and never straddles two rows of characters - which lets
`render_block()` hoist the row lookup out of the line loop, and a
`static_assert` keeps it true. Nine also divides both fields exactly, 26
blocks on NTSC and 32 on PAL, where 8 left a 2 line runt at the bottom of an
NTSC field.

The hoist's own saving could not be measured. Render time is instrumented as
elapsed, and preemption dominates it - 71 us of elapsed against about 16 us
of work - so a few percent of inner loop is invisible. The case for 9 is the
exact division, the longer deadline and the enforceable invariant, not a
measured speedup.

**The drawing went back to a thread.** `OSD_c1` already existed on core1 - it
claimed the DMA channel, enabled the two vectors and then exited - so it just
stays alive as the renderer. At `PRIORITY_IO`, 58, the rate thread and the IMU
bus thread at 181 both preempt it, which is the whole point and which an
interrupt handler cannot offer at any priority.

**Why the IMU cares.** `HAL_CORE_SPI0` is 1, so the IMU bus is on core1, and
RP2350 has one DMA interrupt per core: the OSD holds channel 5 and SPI0 holds
6 and 7, all on IRQ 11. They cannot preempt each other, and ChibiOS's shared
handler walks channels in order, so the OSD is always served first and its
duration lands on the worst case for an IMU completion. Never the reverse.

**The vector's priority used to be set by whoever allocated first.**
`dmaChannelAllocI()` called `nvicEnableVector()` only on the first channel
taken for a core and silently discarded every later priority argument, so the
OSD getting there first turned SPI0's requested 2 into the OSD's 6 - and on
core0 the ADC's 3 sat where SPI wanted 2. It cost nothing on this board,
since nothing sits between 2 and 6, but the value depended on driver init
order, which is not something to leave under a flight critical path.

Fixed in the ChibiOS fork: the vector now takes the most urgent priority any
of its channels asked for. Both cores read 2. The consequence to know is that
the OSD's completion runs at 2 as well, level with the core1 tick rather than
below it - unavoidable when the two share a vector, and the alternative is
the IMU running at 6.

**Two block times, not one.** The renderer's work is 16 us; what it waits for
is to be scheduled at all, behind three core1 threads at priority 181. One
block time was close enough to that latency that it was missed 1% of the
time, about 17 events a second. A third buffer doubles the deadline and that
falls to 5 in 30 s. Buffer count, not block size, is the
lever for this: block size trades the deadline against the work in the same
proportion and changes nothing.

**Queue rules.** `produced` and `consumed` have a single writer each, so
nothing needs locking. The thread stops at two rendered, which with three
buffers always leaves the armed one alone, so the DMA can never read a buffer
being written. Each buffer carries the block it holds: a block sent blank is
never consumed, so without the tag the queue would be left one ahead of the
scan-out and put the wrong eight lines up, shifted, for the rest of the
field. The interrupt drops stale heads until one matches.

**A late block is not a desync, and conflating them cost a screen.** An
underrun consumes words that were never supplied, so the phase is gone and
the rest of the field has to be blanked. A late block is not that - the
filler armed in its place is exactly `block_words()` long, so the stream
stays in step and only those eight lines are affected. Treating both as a
desync blanked ~200 lines instead of 8, at 17 events a second, and that path
incremented no counter of its own - so the dominant artifact on screen was
invisible in the instrumentation while the counters that did exist looked
clean. Second time in this bring-up that a missing or ambiguous diagnostic
sent the search the wrong way.

**What the block size does and does not buy.** Doubling it doubles the work
as well as the deadline, so it buys no CPU margin. What it buys is tolerance
of a core1 XIP park: at 9 lines every park costs a field, at 32 five in six
were absorbed. Parks only happen on a flash write - and with parameter writes
disabled in flight that is a disarmed-only artifact, which is where this was
left. 18 line blocks are the next step inside the invariant if it ever
matters: one full character row, a 2288 us deadline, 4968 bytes.

## Rejected: keeping the font in XIP flash

`AP_ROMFS::find_decompress()` returns a direct flash pointer when
`HAL_ROMFS_UNCOMPRESSED` is set, so the driver could blit straight out of
flash and give back the font's 13,824 bytes - the bulk of what the OSD still
costs. Rejected on the numbers.

| | |
|---|---|
| RP2350 XIP cache | 16 KB (`RP_XIP_CACHE_SIZE`) |
| Font | 13,824 bytes, 84% of it |
| core0 non-idle samples in flash | 62.3% |
| core1 non-idle samples in flash | 1.8% |

The line renderer reads 90 bytes per line at 15.6 kHz, so 1.4 MB/s,
continuously, from the core1 interrupt. Two things compound.

Core1 is only 1.8% flash-resident, and that is not an accident - it is what
the `__RAMFUNC2__` relocation work bought. This would put its highest rate
interrupt on the flash bus on every line.

And 13.8 KB pulled repeatedly through a 16 KB cache shared with core0 evicts
core0's working set, on the core that is 62% flash-resident and whose load
already gates microSD throughput. Reading through the non-cached alias avoids
the eviction and replaces it with 1.4 MB/s of raw QSPI traffic, which stalls
core0's instruction fetch instead. Neither variant is worth 13.8 KB.

**If the RAM is ever needed**, cache only the glyphs in use rather than moving
the font. A screen typically touches 60-90 distinct characters, so 96 glyphs
is 5.2 KB plus a 256 byte index, saving about 8 KB - and the flash reads
happen in `flush()` on core0 at around 10 Hz rather than in the line interrupt
on core1, so core1 gains no flash traffic at all. It costs a miss path and the
population having to stay ahead of the renderer, so it is only worth it if
17 KB actually becomes a problem.

`rp2350_xip_cache_stats()` returns hit and access counters if this is ever
worth settling by measurement instead of argument.

## Acceptance criteria

Phase 4 is not "it looks right on screen". With OSD enabled, in flight:

| Measure | Requirement | How | Result |
|---|---|---|---|
| `SPID0/1.rxoverruns` | 0 | `sdsnap.sh` | 0, on the bench |
| `SPID0/1.aborts` | 0 | `sdsnap.sh` | 0, on the bench |
| `spi_late_count` | 0 | `sdsnap.sh` | 0, on the bench |
| `internal_errors` | 0 | `sdsnap.sh` | 0, on the bench |
| Rate loop `RTDT` dtMax | no worse than 1.4 ms | log | 1.164 ms, bench arm/disarm, no OSD_TYPE 0 baseline |
| Core1 idle | still positive | `PROFILING.md` method | 0.79% non-preemptible, render is preemptible thread time |
| SD throughput | no material drop | `PM.Load` against `DSF.Bytes`, per Next steps item 1 | not yet |
| Free heap | headroom left after 53 KB | boot message | 101 KB free after 17 KB taken |

Add two of the driver's own, which the bench run already satisfies:

| Measure | Requirement | How |
|---|---|---|
| `osd_pico_txstall` | only alongside `rp2350_xip_park_count` | `osdhw.sh` |
| `osd_pico_last_line` | 233 NTSC, 287 PAL | `osdhw.sh` |

`osdhw.sh` and `sdsnap.sh` both re-derive every address from the ELF and
refuse to report against firmware that ELF does not describe, so a stale build
cannot produce plausible-looking nonsense.

The SPI counters matter because this adds two permanently claimed DMA channels
and about 1.33 MB/s of continuous scan-out traffic (23 words x 288 lines x 50
fields) to a subsystem that was shown to have marginal transfers. That
instrumentation exists now; use it rather than assuming.

## The front end, checked against the schematic

Page 3 of `RP-010038-SD-1` (R2 Rev C). Confirms the port needs no adaptation.

**Pin mapping.** Net labels pair with MCU pin labels by x coordinate, pin pitch
3.0 pt, all three matching to 0.6 pt:

| Net | GPIO |
|---|---|
| `OSD_W` | 21 |
| `OSD_EN` | 22 |
| `OSD_SYNC` | 23 |

W and EN are adjacent with W first, which is the program's only hard pin
constraint - `sm_config_set_out_pins(&config, osd_w_gpio, 2)` and the matching
`pio_sm_set_consecutive_pindirs`. SYNC is independent, used as both the IN pin
base for `wait` and as `jmp` PIN. There is no side-set anywhere in the program,
and the pin bases are runtime variables, so `osd_tx.pio` goes across
unmodified. That matters: at 31 of 32 slots there is nowhere to put a change.

All three are below GPIO32, so GPIOBASE 0, which is what PIO1 has.

**Sync detector.** Camera -> R66 75R termination -> R63 680R + C58 470p
chroma/noise filter, fc about 500 kHz -> C59 1u coupling -> D7 clamp -> U9
TVL3231 comparator -> `OSD_SYNC`. Video on the + input, reference near ground
on -, so the output is high through active video and low on sync tips. The PIO
treats low as sync (`wait 0 PIN, 0`), so the polarity is right. Read off the
schematic, not measured.

**Overlay.** `OSD_W` drives a 3.9k / 12k / 1.4k divider giving VOH 1.0 V and
VOL 0.3 V into U8, a 74LVC1G3157 analog mux with `OSD_EN` on select: camera
video on B0, overlay level on B1. U10 TLV3541 buffers at gain 2, then R67 75R
to VID. The schematic prints the truth table:

| | OSD_EN | OSD_W |
|---|---|---|
| Black pixel | 1 | 0 |
| White pixel | 1 | 1 |
| Transparent | 0 | X |

That is bit for bit what Betaflight encodes - `osd_pico.c:678` has
`cols[4] = {0b00000000, 0b10101010, 0b11111111, 0b00000000}`, transparent,
black, white, transparent, with bit 0 to W and bit 1 to EN. Hardware and
framebuffer encoding agree with no translation layer.

**Clock.** The program wants a 75 MHz PIO clock; Betaflight gets there from
150 MHz with divider 2. At our 225 MHz the divider is exactly 3.0, so no
fractional divider and no apology needed.

## Open questions

1. ~~Pin assignment~~ **Checked, see below. No PIO change needed.**
2. ~~PAL/NTSC autodetect~~ **Closed: Betaflight drives this OSD on this
   hardware, which is better evidence than the schematic reading below.**
3. 31 of 32 instruction slots leaves one. Any modification to the PIO program
   has nowhere to go, so the program should be taken unmodified if at all
   possible.
4. Two contiguous 26.5 KB heap allocations get harder as the heap fragments.
   Allocate early; if it proves unreliable, a dedicated pool is the fallback.
5. Licensing: Betaflight is GPLv3 and so is ArduPilot, so the port is fine, but
   the files need their provenance recorded in the headers.
