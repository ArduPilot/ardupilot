/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 * Analog OSD scan-out for RP2350. See OSD_pico.h for provenance.
 */

#include "OSD_pico.h"

#if AP_OSD_PICO_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "PIOUART.h"      // PIO register bit fields; not in rp2350.h
#include "RP2350_pio1.h"

using namespace ChibiOS;

extern const AP_HAL::HAL& hal;

volatile uint32_t OSD_pico::vsync_count;

// The OSD owns PIO1. PIO0 belongs to PIOUART and PIO2 to DShot, and both have
// all four state machines in use; PIO1 is also the only block left at
// GPIOBASE 0, which pins below GPIO32 require. NeoPixel is the other claimant
// and cannot coexist - the video program is 31 of the 32 instruction slots.
#define OSD_PIO         PIO1
#define OSD_PIO_RESET   RESETS_ALLREG_PIO1
#define OSD_PIO_FUNCSEL 7U

// GPIOBASE is marked __I in rp2350.h so it has to be written through a raw
// pointer, as PIOUART.cpp and RCOutput_pico.cpp both do.
#define PIO_GPIOBASE_OFFSET 0x168U

// Not in PIOUART.h: the UART programs never join the FIFO, and IN_COUNT is an
// RP2350 addition the UART programs do not use either. IN_COUNT lives in
// SHIFTCTRL at bits 4:0, where the value is the pin count and 0 means 32.
#define PIO_SHIFTCTRL_FJOIN_TX    (1U << 30)
#define PIO_SHIFTCTRL_IN_COUNT_LSB 0U

/*
  osd_tx_pal and osd_tx_ntsc, assembled from Betaflight's osd_tx.pio. 31
  instructions each, wrap_target 0, wrap 30. They differ in one word, index
  19, which carries the horizontal shift that centres the overlay: PAL sets
  y to 26 with a delay of 12, NTSC 25 with a delay of 8.

  The line loop reads 23 words per line and shifts two bits at a time to
  OSD_W and OSD_EN, so a word is 16 pixels and a line is 368.
 */
static const uint16_t k_osd_tx[] = {
    0xc000, 0x20a0, 0x2020, 0xec56, 0x1f84, 0x00c1, 0x20a0, 0xe030,
    0xe04f, 0x2920, 0x0e8a, 0x00c8, 0xe058, 0x1a8d, 0x00d0, 0x0008,
    0x0048, 0xa026, 0x2020, 0xec5a, 0x1d94, 0x00d7, 0x0000, 0xe056,
    0x8080, 0x6002, 0x06fc, 0x0098, 0x01f9, 0xe000, 0x0052,
};
#define OSD_TX_HSHIFT_WORD  19U
#define OSD_TX_HSHIFT_PAL   0xec5aU
#define OSD_TX_HSHIFT_NTSC  0xe859U
#define OSD_TX_WRAP_TARGET  0U
#define OSD_TX_WRAP         30U

// The program wants a 75 MHz PIO clock. At 225 MHz that is exactly 3, so
// unlike the bidirectional DShot decode there is no fractional divider here.
#define OSD_PIO_HZ 75000000U

bool OSD_pico::claim_pio(void)
{
    PIO_TypeDef *pio = OSD_PIO;

    if (!pio1_claim(PIO1Owner::OSD)) {
        return false;
    }

    rp_peripheral_unreset(OSD_PIO_RESET);

    pio->CTRL = 0U;   // stop all state machines before touching instr memory

    // GPIO0-31 window, which OSD_W, OSD_EN and OSD_SYNC all need. This is the
    // reset value but say it rather than inherit it.
    (*reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(pio) + PIO_GPIOBASE_OFFSET)) = 0U;

    for (uint8_t i = 0; i < ARRAY_SIZE(k_osd_tx); i++) {
        pio->INSTR_MEM[i] = k_osd_tx[i];
    }
    // patch the one word that differs between the two line standards
    pio->INSTR_MEM[OSD_TX_HSHIFT_WORD] = is_pal ? OSD_TX_HSHIFT_PAL : OSD_TX_HSHIFT_NTSC;

    sm = 0;
    return true;
}

void OSD_pico::configure_sm(void)
{
    PIO_TypeDef *pio = OSD_PIO;

    pio->CTRL &= ~(1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    // CLKDIV is 16.8 fixed point, and this one is an exact integer.
    const uint32_t div256 = (uint32_t)(((uint64_t)RP_CLK_SYS_FREQ * 256U) / OSD_PIO_HZ);
    pio->SM[sm].CLKDIV = ((div256 >> 8) << PIO_CLKDIV_INT_LSB)
                       | ((div256 & 0xFFU) << PIO_CLKDIV_FRAC_LSB);

    // "jmp PIN" tests the sync line, which is a separate field from the IN
    // base even though both point at the same pin here.
    pio->SM[sm].EXECCTRL = (OSD_TX_WRAP << PIO_EXECCTRL_WRAP_TOP_LSB)
                         | (OSD_TX_WRAP_TARGET << PIO_EXECCTRL_WRAP_BOT_LSB)
                         | (HAL_OSD_PICO_SYNC_PIN << PIO_EXECCTRL_JMP_PIN_LSB);

    /*
      Shift right, so the low two bits of each word reach the pins first and a
      word is consumed left to right across the screen. No autopull: the line
      loop pulls explicitly and uses !OSRE to find the end of a word. The
      whole FIFO goes to TX because nothing is received.
     */
    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_OUT_SHIFTDIR
                          | PIO_SHIFTCTRL_FJOIN_TX
                          | (1U << PIO_SHIFTCTRL_IN_COUNT_LSB);

    // GPIOBASE is 0 on this block, so these are absolute GPIO numbers. OUT and
    // SET both cover W and EN; IN is the sync pin.
    pio->SM[sm].PINCTRL = (HAL_OSD_PICO_W_PIN    << PIO_PINCTRL_OUT_BASE_LSB)
                        | (2U                    << PIO_PINCTRL_OUT_COUNT_LSB)
                        | (HAL_OSD_PICO_W_PIN    << PIO_PINCTRL_SET_BASE_LSB)
                        | (2U                    << PIO_PINCTRL_SET_COUNT_LSB)
                        | (HAL_OSD_PICO_SYNC_PIN << PIO_PINCTRL_IN_BASE_LSB);

    // Hand W and EN to the PIO. Both idle low, which the overlay mux reads as
    // transparent, so the camera passes through until the first pixel data.
    palSetLineMode(PAL_LINE(IOPORT1, HAL_OSD_PICO_W_PIN),
                   PAL_RP_IOCTRL_FUNCSEL(OSD_PIO_FUNCSEL) |
                   PAL_RP_PAD_IE | PAL_RP_PAD_DRIVE4);
    palSetLineMode(PAL_LINE(IOPORT1, HAL_OSD_PICO_EN_PIN),
                   PAL_RP_IOCTRL_FUNCSEL(OSD_PIO_FUNCSEL) |
                   PAL_RP_PAD_IE | PAL_RP_PAD_DRIVE4);

    // Sync stays a plain input. The comparator ahead of it drives a clean
    // logic level, so no pull is wanted; schmitt is free and costs nothing.
    palSetLineMode(PAL_LINE(IOPORT1, HAL_OSD_PICO_SYNC_PIN),
                   PAL_MODE_INPUT | PAL_RP_PAD_IE | PAL_RP_PAD_SCHMITT);

    pio->SM[sm].INSTR = 0xe083U;   // set pindirs, 3 - W and EN are outputs
    pio->SM[sm].INSTR = 0xe000U;   // set pins, 0 - transparent

    /*
      Seed the ISR with the display line count minus one. The program keeps it
      in X across a field and reloads from ISR at the start of each, so it has
      to be there before the state machine runs.
     */
    pio->SM[sm].INSTR = 0x80a0U;                   // pull block
    pio->TXF[sm] = lines - 1U;
    pio->SM[sm].INSTR = 0xa0c7U;                   // mov isr, osr

    pio->CTRL |= (1U << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1U << (PIO_CTRL_SM_RESTART_LSB + sm));
}

/*
  Build the MCM to framebuffer byte map.

  MCM packs four pixels per byte with the leftmost in the high pair and means
  00 black, 10 white, 01 and 11 transparent. The framebuffer wants the
  leftmost pixel in the low pair, because the PIO shifts right, and means bit
  0 OSD_W and bit 1 OSD_EN - which the overlay mux reads as 00 transparent,
  10 black, 11 white.

  Mapping each pair and then reversing the whole byte fixes the pixel order
  and the order within each pair in one step, which is why the pair values
  below look transposed. Straight from Betaflight's mcm2h.py; verified
  against font0.bin by rendering every row of several glyphs both ways.
 */
void OSD_pico::build_font_lut(void)
{
    static const uint8_t pair_map[4] = { 1, 0, 3, 0 };   // 00->01 01->00 10->11 11->00

    for (uint16_t b = 0; b < 256; b++) {
        uint8_t bits = 0;
        for (uint8_t i = 0; i < 4; i++) {
            bits = (bits << 2) | pair_map[(b >> (6 - 2 * i)) & 3];
        }
        uint8_t out = 0;
        for (uint8_t i = 0; i < 8; i++) {
            out = (out << 1) | ((bits >> i) & 1);
        }
        mcm_to_pico[b] = out;
    }
}

void OSD_pico::set_font(const uint8_t *mcm_font)
{
    font = mcm_font;
    build_font_lut();
}

void OSD_pico::write(uint8_t x, uint8_t y, const char *text)
{
    if (text == nullptr || y >= rows) {
        return;
    }
    while (x < OSD_PICO_COLS && *text != 0) {
        chars[y * OSD_PICO_COLS + x] = (uint8_t)*text;
        text++;
        x++;
    }
}

void OSD_pico::clear(void)
{
    memset(chars, ' ', sizeof(chars));
}


/*
  The interrupts reach the driver through this rather than being handed a
  this pointer. There is one scan-out, so a single instance pointer is
  honest about it.
 */
static OSD_pico *osd_instance;

// IRQ0_INTE: bits 0-3 are SMn_RXNEMPTY and 4-7 SMn_TXNFULL, so the four state
// machine IRQ flags start at 8. The program raises flag 0 at the top of a
// field.
#define PIO_INTE_SM_IRQ(n) (1U << (8U + (n)))

/*
  One step below the PIO UARTs, which run at 5, and well below SPI at 2 so a
  sensor transfer still preempts a line. A late line shows as one glitched
  row; a late RC character costs a frame of control input.

  The field flag sits one below the line completion. It has over a
  millisecond of slack - the program raises it, then waits out the vsync and
  skips sixteen hsyncs before its first pull - where a line has thirty
  microseconds. Ordering them also means a completion left pending from the
  end of a field is served before field_start() resets the line counter,
  rather than racing it.
 */
#ifndef OSD_PICO_IRQ_PRIO
#define OSD_PICO_IRQ_PRIO 6
#endif
#ifndef OSD_PICO_FIELD_IRQ_PRIO
#define OSD_PICO_FIELD_IRQ_PRIO (OSD_PICO_IRQ_PRIO + 1)
#endif


// A short last block: 234 NTSC lines is seven full blocks and one of ten.
uint16_t OSD_pico::block_words(uint16_t block) const
{
    const uint32_t first = (uint32_t)block * OSD_PICO_BLOCK_LINES;
    uint32_t n = (lines > first) ? (lines - first) : 0U;
    if (n > OSD_PICO_BLOCK_LINES) {
        n = OSD_PICO_BLOCK_LINES;
    }
    return (uint16_t)(n * OSD_PICO_LINE_WORDS);
}

/*
  Build one block.

  A 12 pixel cell at two bits per pixel is exactly 3 bytes and a cell is 18
  lines, so a character blit is a byte aligned 3 by 18 copy with no shifting.
  30 cells is 90 of the 92 bytes in a line; the last two pixel pairs are off
  the right of the grid and stay transparent.

  The block divides the cell, so it covers one half of one row of cells and
  the row lookup lifts out of the loop - see the static_assert on
  OSD_PICO_BLOCK_LINES.

  chars[] is read without locking. The OSD thread on core0 can change a cell
  mid field, which shows as that character changing a block early. One byte
  per cell, so there is nothing to tear.
 */
void OSD_pico::render_block(uint16_t block, uint32_t *dst)
{
    const uint32_t first = (uint32_t)block * OSD_PICO_BLOCK_LINES;
    const uint16_t words = block_words(block);
    if (words == 0U) {
        return;
    }
    if (font == nullptr || desynced) {
        memset(dst, 0, (size_t)words * sizeof(uint32_t));
        return;
    }

    const uint8_t *cells = &chars[(first / OSD_PICO_CELL_ROWS) * OSD_PICO_COLS];
    uint8_t row_in_cell = (uint8_t)(first % OSD_PICO_CELL_ROWS);

    for (uint16_t w = 0; w < words; w += OSD_PICO_LINE_WORDS) {
        uint8_t *out = (uint8_t *)(dst + w);
        for (uint8_t col = 0; col < OSD_PICO_COLS; col++) {
            const uint8_t *g = &font[cells[col] * OSD_PICO_GLYPH_BYTES
                                     + row_in_cell * OSD_PICO_CELL_BYTES];
            *out++ = mcm_to_pico[g[0]];
            *out++ = mcm_to_pico[g[1]];
            *out++ = mcm_to_pico[g[2]];
        }
        *out++ = 0;
        *out++ = 0;
        row_in_cell++;
    }
}

/*
  The mode is rewritten for every transfer rather than once at setup because
  serve_interrupt() clears CTRL_TRIG down to the error bits before calling
  back, taking DATA_SIZE, INCR_READ and TREQ_SEL with it. UARTDriver.cpp:1336
  reprograms it per transfer for the same reason.
 */
void OSD_pico::arm_block(uint16_t block, uint8_t buf)
{
    dmaChannelDisableX(dma);
    dmaChannelSetSourceX(dma, (uint32_t)line_buf[buf]);
    dmaChannelSetCounterX(dma, block_words(block));
    dmaChannelSetModeX(dma, DMA_CTRL_TRIG_DATA_SIZE_WORD |
                            DMA_CTRL_TRIG_INCR_READ |
                            DMA_CTRL_TRIG_TREQ_SEL(0x08U + sm) |
                            DMA_CTRL_TRIG_EN);
    dmaChannelEnableX(dma);
}

// Transparent filler for a block the renderer did not finish. One word read
// repeatedly, so it needs no buffer and cannot collide with the thread. Left
// in BSS rather than made const: the DMA would otherwise read it from XIP.
static uint32_t osd_blank_word;

void OSD_pico::arm_blank(uint16_t block)
{
    dmaChannelDisableX(dma);
    dmaChannelSetSourceX(dma, (uint32_t)&osd_blank_word);
    dmaChannelSetCounterX(dma, block_words(block));
    dmaChannelSetModeX(dma, DMA_CTRL_TRIG_DATA_SIZE_WORD |
                            DMA_CTRL_TRIG_TREQ_SEL(0x08U + sm) |
                            DMA_CTRL_TRIG_EN);
    dmaChannelEnableX(dma);
}

void OSD_pico::signal_render(void)
{
    chSysLockFromISR();
    if (render_ctx != nullptr) {
        chEvtSignalI(render_ctx, EVENT_MASK(0));
    }
    chSysUnlockFromISR();
}

/*
  Hand block `block` to the DMA off the head of the queue.

  The thread only ever writes buffers the DMA is not reading - it stops at two
  queued, which with three buffers always leaves the armed one alone - so the
  two never need to interlock beyond the produced and consumed counters, and
  those have a single writer each.
 */
void OSD_pico::advance_to(uint16_t block)
{
    dma_block = block;

    /*
      Drop a stale head. A block sent blank because the renderer was late is
      never consumed, so the queue is left holding it while the scan-out has
      moved on; taking it would put the wrong 8 lines on screen and shift
      everything after them. One drop puts the two back in step.
     */
    while (produced != consumed && buf_block[cons_idx] != block) {
        consumed++;
        cons_idx = (cons_idx + 1U < 3U) ? (uint8_t)(cons_idx + 1U) : 0U;
    }

    if (produced == consumed) {
        /*
          Nothing rendered in time. Send this block transparent, and only
          this block: the filler is exactly block_words() long so the stream
          stays in step with the line boundaries. This is not the desync an
          underrun causes, where words the FIFO never supplied were consumed
          anyway and the phase is gone.
         */
        arm_blank(block);
        signal_render();
        return;
    }

    arm_block(block, cons_idx);
    consumed++;
    cons_idx = (cons_idx + 1U < 3U) ? (uint8_t)(cons_idx + 1U) : 0U;
    signal_render();
}

/*
  A block has been handed to the FIFO. All this does now is start the next
  one and wake the renderer - the drawing itself happens in the thread, where
  the rate loop and the IMU can preempt it.
 */
void OSD_pico::block_complete(void)
{
    const uint16_t next = dma_block + 1U;
    if (next >= blocks) {
        return;   // field finished; the vsync interrupt starts the next one
    }

    PIO_TypeDef *pio = OSD_PIO;
    if ((pio->FDEBUG & PIO_FDEBUG_TXSTALL(sm)) != 0U) {
        pio->FDEBUG = PIO_FDEBUG_TXSTALL(sm);
        /*
          Words the FIFO could not supply were still consumed, so the rest of
          this field is displaced by 16 pixels for each one and there is no
          way to find the state machine's phase again before the next field.
          Blank what is left rather than draw it in the wrong place: the
          overlay drops out for a fraction of a field and the camera shows
          through, which reads as a flicker rather than as corruption.
         */
        desynced = true;
    }

    advance_to(next);
}

static void osd_dma_isr(void *p, uint32_t ct)
{
    (void)ct;
    ((OSD_pico *)p)->block_complete();
}

// Forward declaration suppresses -Wmissing-declarations, as PIOUART.cpp does
// for the vectors it owns.
extern "C" {
OSAL_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER);
}

/*
  Top of a field. The program signals here and then spends about 23 lines
  skipping hsyncs before its first pull, so there is over a millisecond to
  build the first two lines and start the channel.
 */
OSAL_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER)
{
    OSAL_IRQ_PROLOGUE();

    PIO_TypeDef *pio = OSD_PIO;

    if ((pio->IRQ0_INTS & PIO_INTE_SM_IRQ(0)) != 0U) {
        pio->IRQ = 1U;    // write one to clear the state machine flag

        OSD_pico *osd = osd_instance;
        if (osd != nullptr) {
            osd->field_start();
        }
        OSD_pico::vsync_count++;
    }

    OSAL_IRQ_EPILOGUE();
}

void OSD_pico::field_start(void)
{
    if (detecting) {
        return;   // the field rate is still being counted; nothing to scan out
    }

    PIO_TypeDef *pio = OSD_PIO;

    /*
      The state machine keeps pulling after the last armed block and stalls
      there every field, so clear that before the active window starts.
      Without it the flag stays latched from the first stall and every field
      after would blank itself.
     */
    pio->FDEBUG = PIO_FDEBUG_TXSTALL(sm);

    /*
      Drop whatever is still queued before line 0 goes in. A field cut short -
      video lost, or a standard that does not match what the camera sends -
      leaves words behind, and since the program pulls without blocking they
      would displace the whole of the next field rather than being noticed.
      Toggling FJOIN_TX is the only way to empty a PIO FIFO.
     */
    pio->SM[sm].SHIFTCTRL ^= PIO_SHIFTCTRL_FJOIN_TX;
    pio->SM[sm].SHIFTCTRL ^= PIO_SHIFTCTRL_FJOIN_TX;

    desynced = false;   // the flush puts word and line boundaries back together
    advance_to(0);
}

/*
  Swap the line standard on a scan-out that is already running. The two
  programs differ in the single instruction that sets the delay from hsync to
  the first pixel, so this is one word plus a reseed of the line count, which
  the program reads from ISR.
 */
void OSD_pico::set_standard(bool pal)
{
    PIO_TypeDef *pio = OSD_PIO;

    pio->IRQ0_INTE &= ~PIO_INTE_SM_IRQ(0);
    dmaChannelDisableX(dma);
    pio->CTRL &= ~(1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    is_pal = pal;
    lines = pal ? OSD_PICO_LINES_PAL : OSD_PICO_LINES_NTSC;
    rows = pal ? OSD_PICO_ROWS_PAL : OSD_PICO_ROWS_NTSC;
    blocks = (uint16_t)((lines + OSD_PICO_BLOCK_LINES - 1U) / OSD_PICO_BLOCK_LINES);

    pio->INSTR_MEM[OSD_TX_HSHIFT_WORD] = pal ? OSD_TX_HSHIFT_PAL
                                             : OSD_TX_HSHIFT_NTSC;
    // configure_sm() seeds the line count through the FIFO, so it has to be
    // empty or the seed lands behind stale pixel data
    pio->SM[sm].SHIFTCTRL ^= PIO_SHIFTCTRL_FJOIN_TX;
    pio->SM[sm].SHIFTCTRL ^= PIO_SHIFTCTRL_FJOIN_TX;

    configure_sm();

    pio->IRQ = 1U;
    pio->IRQ0_INTE |= PIO_INTE_SM_IRQ(0);
    pio->CTRL |= (1U << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

/*
  Claim the DMA channel, enable both interrupts, then stay as the renderer.

  Deliberately run on core1: dmaChannelAllocI() and nvicEnableVector() both
  act on the calling core's NVIC, so doing this from the OSD thread would put
  the completion interrupt on core0 - the core whose load already gates
  microSD throughput. Core1 carries the rate loop and has the headroom.

  It stays here afterwards rather than exiting, because a thread at
  PRIORITY_IO is preempted by the rate thread and the IMU bus thread, both at
  PRIORITY_RCOUT and PRIORITY_SPI. Drawing in the completion interrupt could
  not be.
 */
void OSD_pico::core1_thread(void)
{
    PIO_TypeDef *pio = OSD_PIO;

    /*
      Any free channel, never a fixed id. A fixed id has no fallback and the
      failure is silent, which is what took out the GPS once already.
     */
    osalSysLock();
    dma = dmaChannelAllocI(RP_DMA_CHANNEL_ID_ANY, OSD_PICO_IRQ_PRIO,
                           osd_dma_isr, this);
    osalSysUnlock();
    if (dma == nullptr) {
        hal.console->printf("OSD: no DMA channel\n");
        core1_failed = true;
        return;
    }

    // the destination survives a completion; the mode does not, so it is set
    // per transfer in arm_block() instead
    dmaChannelSetDestinationX(dma, (uint32_t)&pio->TXF[sm]);
    // dmaChannelAllocI() enables the NVIC vector but not the channel's bit in
    // INTE, so without this the transfers run and nothing is ever told
    dmaChannelEnableInterruptX(dma);

    osd_instance = this;

    pio->IRQ = 1U;
    pio->IRQ0_INTE |= PIO_INTE_SM_IRQ(0);
    nvicEnableVector(RP_PIO1_IRQ_0_NUMBER, OSD_PICO_FIELD_IRQ_PRIO);

    pio->CTRL |= (1U << (PIO_CTRL_SM_ENABLE_LSB + sm));

    /*
      Block 0 has to exist before the first field flag arrives, so build it
      here rather than waiting to be asked.
     */
    render_ctx = chThdGetSelfX();
    produced = 0;
    consumed = 0;
    prod_idx = 0;
    cons_idx = 0;
    next_render_block = 0;

    core1_ready = true;

    while (!thread_stop) {
        // keep two rendered and waiting; the third is whatever the DMA has
        while ((produced - consumed) < 2U && !thread_stop) {
            render_block(next_render_block, line_buf[prod_idx]);
            // the tag has to be visible before the count that publishes it
            buf_block[prod_idx] = next_render_block;
            produced++;

            prod_idx = (prod_idx + 1U < 3U) ? (uint8_t)(prod_idx + 1U) : 0U;
            next_render_block = (next_render_block + 1U < blocks)
                                ? (uint16_t)(next_render_block + 1U) : 0U;
        }
        // the timeout only matters if a field flag is ever missed; the work
        // is driven by the completion interrupt signalling here
        chEvtWaitAnyTimeout(EVENT_MASK(0), chTimeMS2I(100));
    }
}

bool OSD_pico::init(bool pal)
{
    if (initialised) {
        return true;
    }
    is_pal = pal;
    lines = pal ? OSD_PICO_LINES_PAL : OSD_PICO_LINES_NTSC;
    rows = pal ? OSD_PICO_ROWS_PAL : OSD_PICO_ROWS_NTSC;
    blocks = (uint16_t)((lines + OSD_PICO_BLOCK_LINES - 1U) / OSD_PICO_BLOCK_LINES);
    memset(chars, ' ', sizeof(chars));
    detecting = true;

    if (!claim_pio()) {
        return false;
    }
    configure_sm();

    if (!hal.scheduler->thread_create_pinned_to_core(
            FUNCTOR_BIND_MEMBER(&OSD_pico::core1_thread, void),
            "OSD_c1", 1024, AP_HAL::Scheduler::PRIORITY_IO, 0, 1)) {
        hal.console->printf("OSD: could not start core1 thread\n");
        release();
        return false;
    }

    // the thread only claims a channel and enables two vectors, so it is
    // quick; give it room without hanging the caller if it never runs
    for (uint16_t i = 0; i < 200 && !core1_ready && !core1_failed; i++) {
        hal.scheduler->delay(1);
    }
    if (!core1_ready) {
        hal.console->printf("OSD: core1 setup did not complete\n");
        release();
        return false;
    }

    /*
      Which standard the camera sends is measured, not configured. The field
      flag is raised on both, so counting fields for 400 ms separates 50 Hz
      from 60 Hz by four fields either side of the threshold. No camera means
      no fields and the caller's choice stands.
     */
    const uint32_t before = vsync_count;
    hal.scheduler->delay(400);
    const uint32_t fields = vsync_count - before;
    if (fields >= 8) {
        const bool measured_pal = fields < 22;   // 20 at 50 Hz, 24 at 60 Hz
        if (measured_pal != is_pal) {
            set_standard(measured_pal);
        }
    }
    detecting = false;

    initialised = true;
    hal.console->printf("OSD: pico %s on PIO1 sm%u, %u lines, %u fields in 400ms\n",
                        is_pal ? "PAL" : "NTSC", unsigned(sm),
                        unsigned(lines), unsigned(fields));
    return true;
}

void OSD_pico::release(void)
{
    thread_stop = true;
    OSD_PIO->IRQ0_INTE &= ~PIO_INTE_SM_IRQ(0);
    OSD_PIO->CTRL &= ~(1U << (PIO_CTRL_SM_ENABLE_LSB + sm));
    osd_instance = nullptr;
    if (dma != nullptr) {
        dmaChannelFree(dma);
        dma = nullptr;
    }
    initialised = false;
}

#endif  // AP_OSD_PICO_ENABLED
