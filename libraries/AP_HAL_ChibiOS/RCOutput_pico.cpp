/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * PIO-driven outputs for RP2350: DShot600 on PIO2, WS2812 / NeoPixel on PIO1.
 *
 * The DShot programs are assembled from Betaflight's src/platform/PICO/dshot.pio
 * (GPLv3), by Matthew Selby, Andy Piper and qqqlab, and the WS2812 program is
 * the four-instruction one from the Raspberry Pi pico-examples, reached by way
 * of Betaflight's src/platform/PICO/light_ws2811strip_pico.c (GPLv3). ArduPilot
 * has no pioasm in its build, so the assembled words are embedded below rather
 * than generated.
 */

#include "RCOutput_pico.h"

#if defined(RP2350) && (HAL_USE_PWM == TRUE) && (HAL_DSHOT_ENABLED || HAL_SERIALLED_ENABLED)

#include <AP_HAL/AP_HAL.h>
#include <hal.h>

// PIOUART.h carries the PIO register bit-field constants for this port; they
// are not in rp2350.h. Nothing else is taken from it.
#include "PIOUART.h"
#include "RCOutput.h"

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

/* --------------------------- shared PIO plumbing -------------------------- */

// Not in PIOUART.h because the UART programs have no use for it. RP2350
// SHIFTCTRL puts FJOIN_TX at bit 30 and FJOIN_RX at 31.
#define PIO_SHIFTCTRL_FJOIN_TX (1U << 30)
#define PIO_SHIFTCTRL_FJOIN_RX (1U << 31)

// GPIOBASE is marked __I in rp2350.h, so it has to be written through a raw
// pointer. Offset from the PIO base, as in PIOUART.cpp.
#define PIO_GPIOBASE_OFFSET 0x168U

// FUNCSEL on RP2350: PIO0 is 6, PIO1 is 7, PIO2 is 8. 11 is UART_AUX, which is
// where the DShot pads were being routed - the state machines ran but nothing
// reached the pin. PIO0 belongs to PIOUART, so only the other two appear here.
#define PIO1_FUNCSEL 7U
#define PIO2_FUNCSEL 8U

/* --------------------- DShot: block, programs, timing --------------------- */

#if HAL_DSHOT_ENABLED

/*
  DShot lives on PIO2. PIOUART only ever uses PIO0 and PIO1, so there is no
  arbitration to do - and more usefully, PIO2 has its own GPIOBASE. PIOUART
  sets GPIOBASE=16 on its blocks to reach GPIO16-47, which would put motor pins
  below GPIO16 out of range; here we leave it at 0 for a GPIO0-31 window.
 */
#define DSHOT_PIO         PIO2
#define DSHOT_PIO_RESET   RESETS_ALLREG_PIO2
#define DSHOT_PIO_FUNCSEL PIO2_FUNCSEL

/*
  Both programs load at offset 0, one at a time.

  dshot_600       13 instructions
  dshot_600_bidir 29 instructions

  42 words against the 32 available in a block, so they cannot both be
  resident. Every channel shares one direction, so the program is chosen when
  the output mode is set and the block is reloaded if the direction changes.
 */
static const uint16_t k_dshot600_pgm[] = {
    0xff00, //  0: set    pins, 0                [31]
    0xb442, //  1: nop                           [20]
    0x80a0, //  2: pull   block
    0x6050, //  3: out    y, 16
    0x6041, //  4: out    y, 1
    0x006a, //  5: jmp    !y, 10
    0xfa01, //  6: set    pins, 1                [26]
    0xe900, //  7: set    pins, 0                [9]
    0x00e4, //  8: jmp    !osre, 4
    0x0000, //  9: jmp    0
    0xec01, // 10: set    pins, 1                [12]
    0xf600, // 11: set    pins, 0                [22]
    0x01e4, // 12: jmp    !osre, 4               [1]
};
#define DSHOT600_WRAP_TARGET 0
#define DSHOT600_WRAP       12

static const uint16_t k_dshot600_bidir_pgm[] = {
    0x80a0, //  0: pull   block
    0xe081, //  1: set    pindirs, 1
    0x6050, //  2: out    y, 16
    0x6041, //  3: out    y, 1
    0xff00, //  4: set    pins, 0                [31]
    0x086b, //  5: jmp    !y, 11                 [8]
    0xbf42, //  6: nop                           [31]
    0xaa42, //  7: nop                           [10]
    0xff01, //  8: set    pins, 1                [31]
    0x07e3, //  9: jmp    !osre, 3               [7]
    0x000e, // 10: jmp    14
    0xff01, // 11: set    pins, 1                [31]
    0xbf42, // 12: nop                           [31]
    0x12e3, // 13: jmp    !osre, 3               [18]
    0xb442, // 14: nop                           [20]
    0xe080, // 15: set    pindirs, 0
    0x20a0, // 16: wait   1 pin, 0
    0xe023, // 17: set    x, 3
    0x3020, // 18: wait   0 pin, 0               [16]
    0xe05f, // 19: set    y, 31
    0x4e01, // 20: in     pins, 1                [14]
    0x0098, // 21: jmp    y--, 24
    0x0053, // 22: jmp    x--, 19
    0x0019, // 23: jmp    25
    0x0114, // 24: jmp    20                     [1]
    0xe081, // 25: set    pindirs, 1
    0xe001, // 26: set    pins, 1
    0xe044, // 27: set    y, 4
    0x1d7c, // 28: jmp    !y, 28                 [29]
};
#define DSHOT600_BIDIR_WRAP_TARGET 0
#define DSHOT600_BIDIR_WRAP       28

/*
  Points in the bidirectional program that write_frame() has to know about. The
  program is loaded at instruction 0, so SM.ADDR is an index into the array
  above.

    0        blocked on pull, ready for a frame
    1-14     transmitting, do not disturb
    15       hands the line back to the ESC
    16, 18   the two waits - where a silent ESC parks the machine
    19-24    sampling the response
    25-28    done, about to wrap
 */
#define DSHOT600_BIDIR_PC_PULL      0U
#define DSHOT600_BIDIR_PC_WAIT_ONE  16U
#define DSHOT600_BIDIR_PC_WAIT_ZERO 18U
#define DSHOT600_BIDIR_PC_COMPLETE  25U

/*
  Consecutive updates parked on a wait before the machine is restarted. The
  rcout thread runs at 1 kHz and a real ESC turnaround is tens of microseconds,
  so two in a row is already far past normal and this costs 2 ms of output on a
  channel whose ESC has gone quiet.
 */
#define DSHOT600_BIDIR_STALL_LIMIT  2U

/*
  Bit periods in PIO cycles, fixed by the programs above. These set the PIO
  clock: DShot600 is 1/600000 s per bit, so the non-bidirectional program wants
  24 MHz and the bidirectional one 75 MHz.
 */
#define DSHOT600_CYCLES_PER_BIT       40U
#define DSHOT600_BIDIR_CYCLES_PER_BIT 125U

#define DSHOT600_PIO_HZ       24000000U
#define DSHOT600_BIDIR_PIO_HZ 75000000U

/*
  The bidirectional decoder converts sample counts to bit times against a fixed
  75 MHz, so a fractional divider there would not just add jitter, it would put
  the decode on the wrong scale. Require the system clock to be an exact
  multiple. 225 MHz gives 3; Betaflight's 150 MHz gives 2.

  The non-bidirectional program has no such constraint - it only has to hold
  the line for the right fraction of a bit - so 24 MHz may be reached with a
  fractional divider (9.375 at 225 MHz, exactly representable in 16.8).
 */
static_assert((RP_CLK_SYS_FREQ % DSHOT600_BIDIR_PIO_HZ) == 0,
              "bidirectional DShot needs a system clock that is a multiple of 75MHz");

// 18 PIO cycles per sample at 75MHz against a 1/(5/4 * 600000) telemetry bit
#define SAMPLES_TO_BITS ((18.0f / 75.0f) / (1.0f / 0.75f))

#define TELEM_WORDS   4     // 4 x 32 samples
#define TELEM_MAX_EDGES 24  // 21 data bits plus the return to idle

#endif // HAL_DSHOT_ENABLED

/* -------------------- NeoPixel: block, program, timing -------------------- */

#if HAL_SERIALLED_ENABLED

/*
  NeoPixel lives on PIO1, and the block is forced rather than preferred.

  PIOUART only ever instantiates onto PIO0 and DShot owns PIO2 above, so PIO1
  is the one left. That is fortunate rather than incidental: an LED pin below
  GPIO16 needs GPIOBASE 0, which the PIOUART blocks cannot give because they
  are shifted to 16 to reach GPIO16-47, and the DShot block has neither a spare
  state machine nor room for the program - the bidirectional program alone is
  29 of the 32 instruction slots against the 4 this one needs.
 */
#define NEOP_PIO         PIO1
#define NEOP_PIO_RESET   RESETS_ALLREG_PIO1
#define NEOP_PIO_FUNCSEL PIO1_FUNCSEL

/*
  ws2812.pio, side-set on the LED pin:

    .side_set 1
    .wrap_target
    bitloop:
        out    x, 1        side 0 [T3 - 1]
        jmp    !x, do_zero side 1 [T1 - 1]
    do_one:
        jmp    bitloop     side 1 [T2 - 1]
    do_zero:
        nop                side 0 [T2 - 1]
    .wrap

  T1 3, T2 3, T3 4 - ten PIO cycles per bit, which is what sets the clock
  below. A one is high for T1+T2 and low for T3; a zero is high for T1 and low
  for T2+T3.
 */
static const uint16_t k_ws2812_pgm[] = {
    0x6321,   // 0: out    x, 1     side 0 [3]
    0x1223,   // 1: jmp    !x, 3    side 1 [2]
    0x1200,   // 2: jmp    0        side 1 [2]
    0xa242,   // 3: nop             side 0 [2]
};

#define WS2812_WRAP_TARGET 0U
#define WS2812_WRAP        3U

// ten PIO cycles a bit at the 800 kHz WS2812 carrier
#define WS2812_CYCLES_PER_BIT 10U
#define WS2812_CARRIER_HZ     800000U
#define WS2812_PIO_HZ         (WS2812_CARRIER_HZ * WS2812_CYCLES_PER_BIT)

// bits shifted per LED, and so the autopull threshold
#define WS2812_BITS_PER_LED 24U

/*
  A WS2812 latches its shift register when the line has been low for long
  enough. The datasheet asks for 50 us; parts in the wild want more, and the
  cost of being generous is invisible at LED update rates.
 */
#define WS2812_RESET_US 300U

// one LED is 24 bits at 800 kHz, so 30 us. Bound the wait well above that.
#define WS2812_SEND_TIMEOUT_US 20000U

#endif // HAL_SERIALLED_ENABLED

/* ------------------------------ class storage ----------------------------- */

#if HAL_DSHOT_ENABLED
bool     RCOutput_pico::_initialised;
bool     RCOutput_pico::_bidir;
uint32_t RCOutput_pico::_chan_mask;
uint8_t  RCOutput_pico::_gpio[RCOutput_pico::MAX_CHANNELS];
uint8_t  RCOutput_pico::_len_transition[4];
uint8_t  RCOutput_pico::_stall_count[RCOutput_pico::MAX_CHANNELS];
uint32_t RCOutput_pico::_stall_restarts[RCOutput_pico::MAX_CHANNELS];
#endif // HAL_DSHOT_ENABLED

#if HAL_SERIALLED_ENABLED
bool     RCOutput_pico::_neop_initialised;
uint32_t RCOutput_pico::_neop_chan_mask;
uint8_t  RCOutput_pico::_neop_gpio[RCOutput_pico::MAX_CHANNELS];
uint32_t RCOutput_pico::_neop_last_send_us[RCOutput_pico::MAX_CHANNELS];
#endif // HAL_SERIALLED_ENABLED

/* --------------------------- DShot implementation ------------------------- */

#if HAL_DSHOT_ENABLED
/*
  Load one of the two programs at offset 0, stopping every state machine first.
  Called again when the direction changes, which is why the instruction memory
  is rewritten rather than appended to.
 */
void RCOutput_pico::load_program(bool bidir)
{
    PIO_TypeDef *pio = DSHOT_PIO;

    rp_peripheral_unreset(DSHOT_PIO_RESET);

    pio->CTRL = 0U;   // stop all state machines before touching instr memory

    // GPIO0-31 window. PIOUART needs 16 on its blocks; we must not, because
    // the motor pins are below GPIO16 on this board.
    (*reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(pio) + PIO_GPIOBASE_OFFSET)) = 0U;

    const uint16_t *pgm = bidir ? k_dshot600_bidir_pgm : k_dshot600_pgm;
    const uint8_t   len = bidir ? ARRAY_SIZE(k_dshot600_bidir_pgm) : ARRAY_SIZE(k_dshot600_pgm);

    for (uint8_t i = 0; i < len; i++) {
        pio->INSTR_MEM[i] = pgm[i];
    }
}

bool RCOutput_pico::init(bool bidir)
{
    if (_initialised && _bidir == bidir) {
        return true;
    }

    load_program(bidir);

    _bidir = bidir;
    _initialised = true;

    // rebind anything already added, since the program moved underneath it
    for (uint8_t chan = 0; chan < MAX_CHANNELS; chan++) {
        if (_chan_mask & (1U << chan)) {
            start_sm(chan, _gpio[chan]);
        }
    }
    return true;
}

/*
  Configure one state machine for one motor pin.

  The two programs differ in more than timing. The non-bidirectional one only
  ever drives the pin, idles low, and wants a pull-down so a disarmed ESC sees
  a quiet line. The bidirectional one hands the pin back and forth between
  output and input, idles high, and wants a pull-up so the line is held while
  the ESC decides to answer.
 */
void RCOutput_pico::start_sm(uint8_t chan, uint8_t gpio)
{
    PIO_TypeDef *pio = DSHOT_PIO;
    const uint8_t sm = chan;

    pio->CTRL &= ~(1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    const uint32_t pio_hz = _bidir ? DSHOT600_BIDIR_PIO_HZ : DSHOT600_PIO_HZ;

    // CLKDIV is 16.8 fixed point. Compute in that scale so the fractional
    // non-bidirectional divider is not rounded away.
    const uint32_t div256 = (uint32_t)(((uint64_t)RP_CLK_SYS_FREQ * 256U) / pio_hz);
    pio->SM[sm].CLKDIV = ((div256 >> 8) << PIO_CLKDIV_INT_LSB)
                       | ((div256 & 0xFFU) << PIO_CLKDIV_FRAC_LSB);

    const uint32_t wrap_top = _bidir ? DSHOT600_BIDIR_WRAP : DSHOT600_WRAP;
    const uint32_t wrap_bot = _bidir ? DSHOT600_BIDIR_WRAP_TARGET : DSHOT600_WRAP_TARGET;

    pio->SM[sm].EXECCTRL = (wrap_top << PIO_EXECCTRL_WRAP_TOP_LSB)
                         | (wrap_bot << PIO_EXECCTRL_WRAP_BOT_LSB);

    /*
      Both shift directions are left, which is the cleared state of
      OUT_SHIFTDIR and IN_SHIFTDIR - the bits mean "shift right" when set. Left
      is what DShot wants: the frame goes out most significant bit first. The
      frame is written into the low half; discarding the initial empty high
      half shifts the frame into position for the bit loop.
     */
    uint32_t shiftctrl = 0U;
    if (_bidir) {
        // PUSH_THRESH of 0 means 32 bits, which is what turns the 128 sampled
        // line levels into four words with no push in the program
        shiftctrl |= PIO_SHIFTCTRL_AUTOPUSH;
    } else {
        // nothing is received, so give the whole FIFO to TX
        shiftctrl |= PIO_SHIFTCTRL_FJOIN_TX;
    }
    pio->SM[sm].SHIFTCTRL = shiftctrl;

    // GPIOBASE is 0 on this block, so PINCTRL fields are absolute GPIO numbers.
    pio->SM[sm].PINCTRL = ((uint32_t)gpio << PIO_PINCTRL_SET_BASE_LSB)
                        | (1U             << PIO_PINCTRL_SET_COUNT_LSB)
                        | ((uint32_t)gpio << PIO_PINCTRL_OUT_BASE_LSB)
                        | (1U             << PIO_PINCTRL_OUT_COUNT_LSB)
                        | ((uint32_t)gpio << PIO_PINCTRL_IN_BASE_LSB);

    /*
      Hand the pad to the PIO and set the idle pull for this direction. IE is
      required even for output-only: the pad register is written whole, and
      without it the bidirectional program's "wait 1 pin 0" never sees the line
      go high and the state machine stalls after the first frame. SCHMITT and a
      real drive strength likewise have to be asked for or they come out zero.
     */
    palSetLineMode(PAL_LINE(IOPORT1, gpio),
                   PAL_RP_IOCTRL_FUNCSEL(DSHOT_PIO_FUNCSEL) |
                   PAL_RP_PAD_IE |
                   PAL_RP_PAD_SCHMITT |
                   PAL_RP_PAD_DRIVE4 |
                   (_bidir ? PAL_RP_PAD_PUE : PAL_RP_PAD_PDE));

    // start at the top of the program with the pin driven
    pio->SM[sm].INSTR = 0xe081U;                       // set pindirs, 1
    pio->SM[sm].INSTR = _bidir ? 0xe001U : 0xe000U;    // set pins, idle level

    pio->CTRL |= (1U << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1U << (PIO_CTRL_SM_RESTART_LSB + sm));
    pio->CTRL |= (1U << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

bool RCOutput_pico::add_channel(uint8_t chan, uint8_t gpio)
{
    if (!_initialised || chan >= MAX_CHANNELS) {
        return false;
    }
    _gpio[chan] = gpio;
    _chan_mask |= (1U << chan);
    start_sm(chan, gpio);
    return true;
}

/*
  The frame goes in the LOW half of the word. Both programs open with
  "out y, 16" to throw away the top half, and the OSR shifts left, so that
  discard is what walks the frame up into the MSBs for the bit loop to send
  most significant bit first. Putting the frame in the high half instead feeds
  the discard with the frame itself and transmits sixteen zeros - a well formed
  DShot packet meaning throttle zero, which an ESC accepts and sits on.

  Dropping the frame when the FIFO is full is deliberate: a stale frame is worse
  than a missed one, and at DShot600 the state machine is always ready well
  inside a rate-loop period.
 */
void RCOutput_pico::write_frame(uint8_t chan, uint16_t frame)
{
    if (!_initialised || !(_chan_mask & (1U << chan))) {
        return;
    }
    PIO_TypeDef *pio = DSHOT_PIO;
    const uint8_t sm = chan;

    if (!_bidir) {
        // no receive phase, so there is nothing to collide with
        if (pio->FSTAT & (1U << (PIO_FSTAT_TXFULL_LSB + sm))) {
            return;
        }
        pio->TXF[sm] = (uint32_t)frame;
        return;
    }

    /*
      Bidirectional: the machine owns the line for a whole transmit-then-listen
      cycle, so a frame can only be handed over at the two points where it is
      not mid-cycle. Pushing anywhere else races the receive, and if the RX FIFO
      still holds an earlier response then autopush blocks part way through
      sampling - the telemetry comes back as one good word followed by three of
      all ones.
     */
    const uint32_t pc = pio->SM[sm].ADDR;

    if (pc == DSHOT600_BIDIR_PC_PULL || pc >= DSHOT600_BIDIR_PC_COMPLETE) {
        while (!(pio->FSTAT & (1U << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
            (void)pio->RXF[sm];
        }
        pio->TXF[sm] = (uint32_t)frame;
        _stall_count[chan] = 0;
        return;
    }

    if (pc == DSHOT600_BIDIR_PC_WAIT_ONE || pc == DSHOT600_BIDIR_PC_WAIT_ZERO) {
        /*
          Parked waiting for the ESC. An ESC that never answers leaves the
          second wait looking for a falling edge that will not come, and
          without this the channel stops driving for good.
         */
        if (++_stall_count[chan] >= DSHOT600_BIDIR_STALL_LIMIT) {
            restart_sm(chan, frame);
        }
        return;
    }

    // mid transmit or mid receive; let it finish
    _stall_count[chan] = 0;
}

/*
  Recover a state machine parked on a wait: stop it, flush both FIFOs, put it
  back at the top of the program and give it the frame it missed.
 */
void RCOutput_pico::restart_sm(uint8_t chan, uint16_t frame)
{
    PIO_TypeDef *pio = DSHOT_PIO;
    const uint8_t sm = chan;

    pio->CTRL &= ~(1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    // toggling the FIFO join flushes both halves; there is no direct flush
    pio->SM[sm].SHIFTCTRL ^= PIO_SHIFTCTRL_FJOIN_RX;
    pio->SM[sm].SHIFTCTRL ^= PIO_SHIFTCTRL_FJOIN_RX;

    pio->CTRL |= (1U << (PIO_CTRL_SM_RESTART_LSB + sm));
    pio->SM[sm].INSTR = 0x0000U;   // jmp 0, back to the pull

    pio->TXF[sm] = (uint32_t)frame;
    pio->CTRL |= (1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    _stall_count[chan] = 0;
    _stall_restarts[chan]++;
}

/*
  ----------------------------------------------------------------------------
  Bidirectional telemetry decode.

  The state machine samples the line 128 times at 18 PIO cycles a sample, so at
  75MHz and a 100 cycle telemetry bit that is 5.56 samples per bit. What comes
  back is therefore not bits but run lengths, and the decode is: find the edges,
  turn the gap between each pair into a run of 1, 2 or 3 GCR bits, reassemble
  the 20 bit GCR word, and map it back through the quintet table.

  Ported from Betaflight decodeOversampledTelemetry() in
  src/platform/PICO/dshot_bidir_pico.c, whose own comment credits ArduPilot for
  the run length approach.
  ----------------------------------------------------------------------------
 */

void RCOutput_pico::set_transitions(float bits_per_sample)
{
    uint8_t length = 0;
    float bits = 0.5f;
    uint8_t samples = 0;
    while (length < 4) {
        bits += bits_per_sample;
        samples++;
        if ((uint8_t)bits >= length + 1) {
            _len_transition[length] = samples;
            length++;
        }
    }
}

bool RCOutput_pico::read_telemetry(uint8_t chan, uint16_t &encoded)
{
    if (!_initialised || !_bidir || !(_chan_mask & (1U << chan))) {
        return false;
    }
    PIO_TypeDef *pio = DSHOT_PIO;
    const uint8_t sm = chan;

    // FLEVEL packs TX then RX per state machine, four bits each
    const uint8_t rx_level = (pio->FLEVEL >> (8U * sm + 4U)) & 0xFU;
    if (rx_level < TELEM_WORDS) {
        return false;   // response not complete yet
    }

    uint32_t buf[TELEM_WORDS];
    for (uint8_t i = 0; i < TELEM_WORDS; i++) {
        buf[i] = pio->RXF[sm];
    }
    // anything left is stale; the next response must start clean
    while (!(pio->FSTAT & (1U << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
        (void)pio->RXF[sm];
    }

    if (_len_transition[0] == 0) {
        set_transitions(SAMPLES_TO_BITS);
    }

    /*
      Walk the 128 samples as runs. clz on the inverted-or-not word gives the
      length of the current run directly, so each iteration costs one count and
      one shift rather than a scan.
     */
    uint8_t  diffs[TELEM_MAX_EDGES];
    uint8_t  edges = 0;
    bool     ones  = (buf[0] >> 31) != 0;
    uint32_t w0 = buf[0], w1 = buf[1], w2 = buf[2], w3 = buf[3];
    int16_t  remaining = 128;

    while (remaining > 0 && edges < TELEM_MAX_EDGES) {
        const uint32_t testword = ones ? ~w0 : w0;
        if (testword == 0) {
            break;  // rest of the window is one level, and clz(0) is undefined
        }
        const uint8_t run = __builtin_clz(testword);
        diffs[edges++] = run;

        const uint8_t comp = 32U - run;
        w0 = (w0 << run) | (w1 >> comp);
        w1 = (w1 << run) | (w2 >> comp);
        if (w2) {
            w2 = (w2 << run) | (w3 >> comp);
            w3 = w3 << run;
        }
        ones = !ones;
        remaining -= run;
    }

    // the trailing run of ones is the line going idle, not data
    if (!ones && edges > 0) {
        edges--;
    }
    if (edges < 2 || edges > 21) {
        return false;
    }

    // runs to GCR bits
    uint32_t core_gcr = 0;
    uint8_t  core_bits = 0;
    for (uint8_t i = 0; i < edges; i++) {
        const uint8_t d = diffs[i];
        uint8_t len;
        if (d < _len_transition[1]) {
            len = 1;
        } else if (d < _len_transition[2]) {
            len = 2;
        } else if (d < _len_transition[3]) {
            len = 3;
        } else {
            return false;   // GCR never produces a run of four or more
        }
        core_gcr = (core_gcr << len) | (1U << (len - 1U));
        core_bits += len;
        if (core_bits >= 21U) {
            break;
        }
    }

    if (core_bits > 21U) {
        return false;
    }
    /*
      Trailing 1 bits cannot be told apart from the idle line that follows, so
      they never produced an edge. Whatever is missing from 21 is that padding,
      put back here.
     */
    const uint8_t pad = 21U - core_bits;
    uint32_t gcr20 = core_gcr << pad;
    if (pad > 0) {
        gcr20 |= 1U << (pad - 1U);
    }

    /*
      From here the decode is identical to the timer path, so it is shared:
      quintet table, checksum, and dropping the checksum nibble.
     */
    const uint32_t erpm = RCOutput::bdshot_decode_gcr_erpm(gcr20);
    if (erpm == RCOutput::INVALID_ERPM) {
        return false;
    }
    encoded = uint16_t(erpm);
    return true;
}

#endif // HAL_DSHOT_ENABLED

/* -------------------------- NeoPixel implementation ----------------------- */

#if HAL_SERIALLED_ENABLED

bool RCOutput_pico::neopixel_init(void)
{
    if (_neop_initialised) {
        return true;
    }

    PIO_TypeDef *pio = NEOP_PIO;

    rp_peripheral_unreset(NEOP_PIO_RESET);

    pio->CTRL = 0U;   // stop all state machines before touching instr memory

    /*
      GPIO0-31 window. This is the reset value, but PIO1 is only unclaimed on
      boards that leave PIOUART off it, so say it rather than inherit it.
     */
    (*reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(pio) + PIO_GPIOBASE_OFFSET)) = 0U;

    for (uint8_t i = 0; i < ARRAY_SIZE(k_ws2812_pgm); i++) {
        pio->INSTR_MEM[i] = k_ws2812_pgm[i];
    }

    _neop_initialised = true;
    return true;
}

/*
  Configure one state machine for one LED pin.

  The pin is driven through side-set rather than OUT, which is why SIDESET_BASE
  and SET_BASE both point at it: side-set carries the waveform and SET is only
  used to establish the pin direction before the machine starts.
 */
void RCOutput_pico::neopixel_start_sm(uint8_t idx, uint8_t gpio)
{
    PIO_TypeDef *pio = NEOP_PIO;
    const uint8_t sm = idx;

    pio->CTRL &= ~(1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    // CLKDIV is 16.8 fixed point. At 225 MHz this is exactly 28 + 32/256, but
    // compute it so a board on another clock still lands on the right carrier.
    const uint32_t div256 = (uint32_t)(((uint64_t)RP_CLK_SYS_FREQ * 256U) / WS2812_PIO_HZ);
    pio->SM[sm].CLKDIV = ((div256 >> 8) << PIO_CLKDIV_INT_LSB)
                       | ((div256 & 0xFFU) << PIO_CLKDIV_FRAC_LSB);

    pio->SM[sm].EXECCTRL = (WS2812_WRAP        << PIO_EXECCTRL_WRAP_TOP_LSB)
                         | (WS2812_WRAP_TARGET << PIO_EXECCTRL_WRAP_BOT_LSB);

    /*
      Shift left, so the frame goes out most significant bit first and the 24
      colour bits belong in bits 31..8 of the word. OUT_SHIFTDIR cleared is
      left; the bit means "shift right" when set.

      Autopull at 24 bits keeps the bit loop fed with no pull in the program,
      and joining the RX half onto TX gives an eight word FIFO, which covers a
      chain of eight with no refill at all.
     */
    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_AUTOPULL
                          | PIO_SHIFTCTRL_FJOIN_TX
                          | (WS2812_BITS_PER_LED << PIO_SHIFTCTRL_PULL_THRESH_LSB);

    // GPIOBASE is 0 on this block, so PINCTRL fields are absolute GPIO numbers.
    pio->SM[sm].PINCTRL = ((uint32_t)gpio << PIO_PINCTRL_SET_BASE_LSB)
                        | (1U             << PIO_PINCTRL_SET_COUNT_LSB)
                        | ((uint32_t)gpio << PIO_PINCTRL_SIDESET_BASE_LSB)
                        | (1U             << PIO_PINCTRL_SIDESET_COUNT_LSB);

    /*
      Hand the pad to the PIO. A WS2812 line is push-pull output only, so it
      wants no pull of its own; the pull-down is there so the chain sees a
      quiet line rather than a floating one before the machine starts.
     */
    palSetLineMode(PAL_LINE(IOPORT1, gpio),
                   PAL_RP_IOCTRL_FUNCSEL(NEOP_PIO_FUNCSEL) |
                   PAL_RP_PAD_IE |
                   PAL_RP_PAD_SCHMITT |
                   PAL_RP_PAD_DRIVE4 |
                   PAL_RP_PAD_PDE);

    // drive the pin, idle low, and start at the top of the program
    pio->SM[sm].INSTR = 0xe081U;   // set pindirs, 1
    pio->SM[sm].INSTR = 0xe000U;   // set pins, 0

    pio->CTRL |= (1U << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1U << (PIO_CTRL_SM_RESTART_LSB + sm));
    pio->CTRL |= (1U << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

bool RCOutput_pico::neopixel_add_channel(uint8_t idx, uint8_t gpio)
{
    if (!_neop_initialised || idx >= MAX_CHANNELS) {
        return false;
    }
    _neop_gpio[idx] = gpio;
    _neop_chan_mask |= (1U << idx);
    _neop_last_send_us[idx] = AP_HAL::micros();
    neopixel_start_sm(idx, gpio);
    return true;
}

bool RCOutput_pico::neopixel_send_begin(uint8_t idx)
{
    if (!_neop_initialised || idx >= MAX_CHANNELS || !(_neop_chan_mask & (1U << idx))) {
        return false;
    }

    /*
      Refuse rather than truncate if the previous frame has not had its reset
      gap. Sending inside the gap does not fail cleanly - the chain treats the
      new frame as a continuation of the old one and the colours walk along the
      strip.

      The compare is signed on purpose. neopixel_send_end() stamps a time in
      the future, because words can still be queued in the FIFO when it
      returns, and an unsigned difference against a future stamp wraps to about
      4e9 - which passes a "have we waited long enough" test every time and
      disables the guard completely. Signed, a future stamp reads negative and
      waits.
     */
    if ((int32_t)(AP_HAL::micros() - _neop_last_send_us[idx]) < (int32_t)WS2812_RESET_US) {
        return false;
    }
    return true;
}

bool RCOutput_pico::neopixel_send_word(uint8_t idx, uint32_t word)
{
    PIO_TypeDef *pio = NEOP_PIO;
    const uint8_t sm = idx;
    const uint32_t start_us = AP_HAL::micros();

    while (pio->FSTAT & (1U << (PIO_FSTAT_TXFULL_LSB + sm))) {
        // FIFO full: a chain longer than the FIFO has to wait for the wire.
        if (AP_HAL::micros() - start_us > WS2812_SEND_TIMEOUT_US) {
            return false;
        }
        hal.scheduler->delay_microseconds(20);
    }
    pio->TXF[sm] = word;
    return true;
}

void RCOutput_pico::neopixel_send_end(uint8_t idx)
{
    if (idx >= MAX_CHANNELS) {
        return;
    }
    /*
      The reset gap has to be measured from the end of the frame on the wire,
      not from the last FIFO write - up to eight words can still be queued.
     */
    _neop_last_send_us[idx] = AP_HAL::micros()
                            + (WS2812_BITS_PER_LED * 8U * 1000000UL) / WS2812_CARRIER_HZ;
}

#endif // HAL_SERIALLED_ENABLED

/* ------------------- RCOutput methods, RP2350 definitions ----------------- */

/*
  These are RCOutput's own methods, not this file's PIO drivers. RCOutput.cpp
  guards its versions with #if !defined(RP2350) and these stand in, the same
  arrangement RCOutput_iofirmware.cpp uses for IOMCU_FW. It keeps whole-function
  platform differences out of the shared file rather than wrapping a hundred
  lines of STM32 timer code in #if/#else.
 */

bool RCOutput::setup_group_DMA(pwm_group &group, uint32_t bitrate, uint32_t bit_width, bool active_high,
                               const uint16_t buffer_length, rcout_timer_t pulse_time_us, bool at_least_freq)
{
    /*
      This sets up a timer-driven DMAR burst, which RP2350 has no equivalent
      of. DShot is driven from the PIO instead, see RCOutput_pico.cpp. The
      other two callers - serial LED output and serial ESC passthrough - are
      not supported on this chip either, so failing here is the honest answer
      rather than leaving them half-configured.
     */
    (void)group; (void)bitrate; (void)bit_width; (void)active_high;
    (void)buffer_length; (void)pulse_time_us; (void)at_least_freq;
    return false;
}

void RCOutput::send_pulses_DMAR(pwm_group &group, uint32_t buffer_length)
{
    (void)buffer_length;
    /*
      Nothing further to do: writing the packets above already handed them to
      the state machines, which clock them out on their own. Everything below
      is the timer/DMAR burst that RP2350 does not have.

      Straight back to IDLE, not SEND_COMPLETE. On a timer the DMA completion
      walks the state on and dma_unlock() eventually returns it to IDLE; here
      there is no completion event to do that, and is_dshot_send_allowed()
      rejects SEND_COMPLETE - so the group would send exactly one frame at boot
      and then be refused for good. The frame is on its way out of the state
      machine by the time we return, so the group really is idle.
     */
    group.dshot_state = DshotState::IDLE;
    return;
}

#if HAL_SERIALLED_ENABLED
bool RCOutput::serial_led_send(pwm_group &group)
{
    if (!group.serial_led_pending || !is_led_protocol(group.current_mode)) {
        return true;
    }
    /*
      Pack straight from the LED data into the PIO FIFO. There is no DMA buffer
      and no DMA lock to take: the state machine holds the timing and the LED
      thread is the only writer.
     */
    {
        WITH_SEMAPHORE(group.serial_led_mutex);

        group.serial_led_pending = false;
        group.prepared_send = false;

        const bool rgb_order = (group.current_mode == MODE_NEOPIXELRGB);

        for (uint8_t i = 0; i < HAL_PWM_GROUP_CHANNELS; i++) {
            if (group.chan[i] == CHAN_DISABLED || group.serial_led_data[i] == nullptr) {
                continue;
            }
            if (!RCOutput_pico::neopixel_send_begin(i)) {
                continue;
            }
            for (uint8_t led = 0; led < group.serial_nleds; led++) {
                const SerialLed &c = group.serial_led_data[i][led];
                const uint32_t w = rgb_order
                    ? RCOutput_pico::neopixel_pack_rgb(c.red, c.green, c.blue)
                    : RCOutput_pico::neopixel_pack_grb(c.red, c.green, c.blue);
                if (!RCOutput_pico::neopixel_send_word(i, w)) {
                    break;
                }
            }
            RCOutput_pico::neopixel_send_end(i);
        }
    }
    return true;
}
#endif // HAL_SERIALLED_ENABLED

#endif // defined(RP2350) && (HAL_USE_PWM == TRUE) && (HAL_DSHOT_ENABLED || HAL_SERIALLED_ENABLED)
