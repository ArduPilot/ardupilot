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
 * DShot600 for RP2350, driven from the PIO.
 *
 * The PIO programs are assembled from Betaflight's src/platform/PICO/dshot.pio
 * (GPLv3), by Matthew Selby, Andy Piper and qqqlab. ArduPilot has no pioasm in
 * its build, so the assembled words are embedded below rather than generated.
 */

#include "RCOutput_pico.h"

#if defined(RP2350) && HAL_DSHOT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <hal.h>

// PIOUART.h carries the PIO register bit-field constants for this port; they
// are not in rp2350.h. Nothing else is taken from it.
#include "PIOUART.h"
#include "RCOutput.h"

// Not in PIOUART.h because the UART programs have no use for it. RP2350
// SHIFTCTRL puts FJOIN_TX at bit 30 and FJOIN_RX at 31.
#define PIO_SHIFTCTRL_FJOIN_TX (1U << 30)

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

/*
  DShot lives on PIO2. PIOUART only ever uses PIO0 and PIO1, so there is no
  arbitration to do - and more usefully, PIO2 has its own GPIOBASE. PIOUART
  sets GPIOBASE=16 on its blocks to reach GPIO16-47, which would put motor pins
  below GPIO16 out of range; here we leave it at 0 for a GPIO0-31 window.
 */
#define DSHOT_PIO       PIO2
#define DSHOT_PIO_RESET RESETS_ALLREG_PIO2

// GPIOBASE is marked __I in rp2350.h, so it has to be written through a raw
// pointer. Offset from the PIO base, as in PIOUART.cpp.
#define PIO_GPIOBASE_OFFSET 0x168U

// FUNCSEL for PIO2 on RP2350. PIO0 is 6, PIO1 is 7, PIO2 is 11.
#define DSHOT_PIO_FUNCSEL 11U

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

bool     RCOutput_pico::_initialised;
bool     RCOutput_pico::_bidir;
uint32_t RCOutput_pico::_chan_mask;
uint8_t  RCOutput_pico::_gpio[RCOutput_pico::MAX_CHANNELS];
uint8_t  RCOutput_pico::_len_transition[4];

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
      is what DShot wants: the frame goes out most significant bit first, and
      the program discards the top 16 bits of the word before the bit loop, so
      the frame is written into the high half.
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

    // hand the pad to the PIO and set the idle pull for this direction
    palSetLineMode(PAL_LINE(IOPORT1, gpio),
                   PAL_RP_IOCTRL_FUNCSEL(DSHOT_PIO_FUNCSEL) |
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
  The frame goes into the top half of the word because the programs discard the
  first 16 bits before entering the bit loop. Dropping the frame when the FIFO
  is full is deliberate: a stale frame is worse than a missed one, and at
  DShot600 the state machine is always ready well inside a rate-loop period.
 */
void RCOutput_pico::write_frame(uint8_t chan, uint16_t frame)
{
    if (!_initialised || !(_chan_mask & (1U << chan))) {
        return;
    }
    PIO_TypeDef *pio = DSHOT_PIO;
    const uint8_t sm = chan;

    if (pio->FSTAT & (1U << (PIO_FSTAT_TXFULL_LSB + sm))) {
        return;
    }
    pio->TXF[sm] = ((uint32_t)frame) << 16;
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


// 18 PIO cycles per sample at 75MHz against a 1/(5/4 * 600000) telemetry bit
#define SAMPLES_TO_BITS ((18.0f / 75.0f) / (1.0f / 0.75f))

#define TELEM_WORDS   4     // 4 x 32 samples
#define TELEM_MAX_EDGES 24  // 21 data bits plus the return to idle

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

#endif // defined(RP2350) && HAL_DSHOT_ENABLED
