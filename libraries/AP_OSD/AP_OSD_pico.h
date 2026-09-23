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
 * Analog OSD on RP2350: a MAX7456-style overlay without the MAX7456,
 * genlocked to the camera and scanned out by a PIO state machine.
 *
 * The PIO program and the video timing are from Betaflight's
 * src/platform/PICO/osd (GPLv3), by way of osd_tx.pio. ArduPilot has no
 * pioasm in its build and no pico-sdk, so the assembled words are embedded
 * in AP_OSD_pico.cpp and the setup is written against the registers
 * directly, as the HAL's RCOutput_pico.cpp does for WS2812 and DShot.
 *
 * See libraries/AP_HAL_ChibiOS/hwdef/RPI_UAVFC/OSD.md for the port plan and
 * the resource budget.
 */
#pragma once

#include "AP_OSD_config.h"
#include <AP_OSD/AP_OSD_Backend.h>

#ifndef AP_OSD_PICO_ENABLED
#define AP_OSD_PICO_ENABLED 0
#endif

#if AP_OSD_PICO_ENABLED

#include <hal.h>

// Overlay pins. GPIO21 OSD_W and GPIO22 OSD_EN must stay adjacent with W
// first: the program does "out PINS, 2" against a single OUT base, low bit to
// W and high bit to EN. GPIO23 OSD_SYNC is independent, used as both the IN
// base for WAIT and as the JMP pin.
#ifndef HAL_OSD_PICO_W_PIN
#define HAL_OSD_PICO_W_PIN 21U
#endif
#ifndef HAL_OSD_PICO_EN_PIN
#define HAL_OSD_PICO_EN_PIN 22U
#endif
#ifndef HAL_OSD_PICO_SYNC_PIN
#define HAL_OSD_PICO_SYNC_PIN 23U
#endif

// 23 words of 16 pixels at 2bpp is 368 pixels, which is the line length the
// PIO program is built around. Not adjustable without reassembling it.
#define OSD_PICO_LINE_WORDS 23U
#define OSD_PICO_LINE_BYTES (OSD_PICO_LINE_WORDS * 4U)   // 92
#define OSD_PICO_LINES_PAL  288U   // 18 rows of 16 char cells
#define OSD_PICO_LINES_NTSC 234U   // 18 rows of 13 char cells

/*
  A 12 pixel cell at two bits per pixel is exactly 3 bytes and a row of cells
  is exactly 18 lines, so a character blit is a byte aligned 3 by 18 copy with
  no shifting. 30 cells is 90 of the 92 bytes in a line.
 */
#define OSD_PICO_COLS        30U
#define OSD_PICO_ROWS_PAL    16U
#define OSD_PICO_ROWS_NTSC   13U
#define OSD_PICO_CELL_BYTES   3U
#define OSD_PICO_CELL_ROWS   18U
#define OSD_PICO_GLYPH_BYTES (OSD_PICO_CELL_BYTES * OSD_PICO_CELL_ROWS)  // 54
#define OSD_PICO_MAX_CELLS   (OSD_PICO_COLS * OSD_PICO_ROWS_PAL)         // 480

/*
  Lines handed to the DMA in one transfer. The state machine pulls 23 words
  per line and waits for hsync between them, so the transfer is only a word
  stream paced by the FIFO - nothing in the DMA knows where a line ends.
  Batching therefore costs nothing but buffer, and buys two things: the
  completion interrupt drops from one per line to one per block, 14 kHz to
  1.56 kHz over NTSC's 234 active lines a field, and the renderer gets
  two block times to work in - about a millisecond - rather than the seventeen
  microseconds the eight word FIFO holds.

  Nine lines is half a character cell, and has to divide the 18 line cell for
  render_block() to hoist the cell lookup: at multiples of 9 a block covers
  either the top or the bottom half of one row of cells and never straddles
  two. It also divides both fields exactly - 234 is 26 blocks and 288 is 32 -
  where 8 leaves a 2 line runt at the bottom of an NTSC field.

  A block is 572 us, near enough one rate thread cycle at 2 kHz, so the
  renderer does about a block per cycle. Larger blocks buy no CPU margin,
  since doubling the block doubles the work as well as the deadline; what
  they buy is tolerance of a core1 XIP park, and those only happen on a flash
  write.
 */
#define OSD_PICO_BLOCK_LINES 9U
#define OSD_PICO_BLOCK_WORDS (OSD_PICO_BLOCK_LINES * OSD_PICO_LINE_WORDS)

static_assert(OSD_PICO_CELL_ROWS % OSD_PICO_BLOCK_LINES == 0,
              "block must divide the character cell so it cannot span two rows");

class AP_OSD_pico : public AP_OSD_Backend
{
public:
    static AP_OSD_Backend *probe(AP_OSD &osd);

    // Loads the font, then claims PIO1, two GPIOs, one DMA channel and a
    // framebuffer. All or nothing: returns false with everything released.
    bool init() override;

    // Character cell access. Coordinates are cells, not pixels. Lines are
    // generated from the cells on the fly, so flush() has nothing to copy.
    void write(uint8_t x, uint8_t y, const char *text) override;
    void flush() override;
    void clear() override;

    /*
      Asks whether a second backend of the given type can run alongside this
      one, so it is false for the ones that do the same job. PICO is another
      bitmap character overlay, like MAX7456 and SITL, so those three are
      mutually exclusive; the MSP backends drive a different display and can
      coexist.
     */
    bool is_compatible_with_backend_type(AP_OSD::osd_types type) const override {
        switch (type) {
        case AP_OSD::osd_types::OSD_PICO:
        case AP_OSD::osd_types::OSD_MAX7456:
        case AP_OSD::osd_types::OSD_SITL:
            return false;
        case AP_OSD::osd_types::OSD_NONE:
        case AP_OSD::osd_types::OSD_TXONLY:
        case AP_OSD::osd_types::OSD_MSP:
        case AP_OSD::osd_types::OSD_MSP_DISPLAYPORT:
            return true;
        }
        return false;
    }

    AP_OSD::osd_types get_backend_type() const override {
        return AP_OSD::osd_types::OSD_PICO;
    }

    // fields scanned out since init, incremented by the vsync interrupt.
    // Static so the interrupt handler can reach it without the object.
    static volatile uint32_t vsync_count;
    // blocks sent transparent because the renderer was late, and fields cut
    // short by the FIFO running dry; both only ever count up
    static volatile uint32_t late_blocks;
    static volatile uint32_t desyncs;
    // blocks sent transparent without rendering because nothing on them is
    // visible
    static volatile uint32_t blank_blocks;

    // called from interrupt context only
    void field_start(void);
    void block_complete(void);

private:
    using AP_OSD_Backend::AP_OSD_Backend;

    // the hardware half of init(), PAL until a camera says otherwise
    bool start_scanout(bool pal);

    /*
      Takes the font in the 54 bytes per character MCM layout AP_OSD keeps
      in ROMFS. The glyphs are blitted straight out of it, so the pointer
      must stay valid.
     */
    void set_font(const uint8_t *mcm_font);

    // re-measure the field rate and switch standard if the camera's differs;
    // the switch blanks the overlay for a field or two
    void check_standard(void);

#if AP_RP2350_DEBUG_REPORT_ENABLED
    // field, late and desync counts over the last window, every 10 s
    void report_stats(void);
    uint32_t last_report_ms;
    uint32_t last_fields, last_late, last_blank, last_desyncs;
#endif

    void build_font_lut(void);
    void build_blank_table(void);
    // true when every cell the block covers shows nothing there
    bool block_is_blank(uint16_t block) const;
    bool claim_pio(void);
    void configure_sm(void);
    // swap the line standard on a running scan-out
    void set_standard(bool pal);
    // set_standard() from the core1 thread, with the queue emptied
    void apply_standard(bool pal);
    void release(void);
    /*
      Claims the DMA channel, enables both interrupts and then stays as the
      renderer. Pinned to core1 at PRIORITY_IO, which is 58 against the rate
      thread's and the IMU bus thread's 181, so both preempt it. That is the
      point of doing the drawing here rather than in the completion
      interrupt, where it could not be preempted by anything.
     */
    void core1_thread(void);
    void render_block(uint16_t block, uint32_t *dst);
    void arm_block(uint16_t block, uint8_t buf);
    void signal_render(void);
    void arm_blank(uint16_t block);
    void advance_to(uint16_t block);
    uint16_t block_words(uint16_t block) const;

    /*
      Three blocks, as a queue: one being read by the DMA, up to two rendered
      and waiting. That gives the renderer two block times rather than one -
      about a millisecond - which matters because the work is only 16 us and
      what it is really waiting for is to be scheduled at all, behind three
      core1 threads at priority 181.
     */
    uint32_t line_buf[3][OSD_PICO_BLOCK_WORDS];
    // which block each buffer holds, so a skipped one cannot silently shift
    // everything after it
    volatile uint16_t buf_block[3];
    // single writer each, so the difference needs no locking
    volatile uint32_t produced;
    volatile uint32_t consumed;
    uint8_t prod_idx;
    volatile uint8_t cons_idx;
    uint16_t next_render_block;
    // after a late block, where the renderer should pick up; the interrupt
    // writes it, and late_seen is the thread's record of which late block it
    // last acted on
    volatile uint16_t resync_block;
    uint32_t late_seen;
    volatile uint16_t dma_block;
    uint16_t blocks;
    thread_t *render_ctx;
    volatile bool thread_stop;
    uint32_t lines;
    const rp_dma_channel_t *dma;
    uint8_t sm;
    const uint8_t *font;
    // MCM byte to framebuffer byte. 256 bytes rather than transforming a
    // 13.8 KB font copy, so a cell blit is one lookup per byte.
    uint8_t mcm_to_pico[256];
    // one bit per character code for each block-sized part of a cell, set
    // when that part of the glyph is entirely transparent
    uint32_t glyph_blank[OSD_PICO_CELL_ROWS / OSD_PICO_BLOCK_LINES][8];
    uint8_t chars[OSD_PICO_MAX_CELLS];
    uint8_t rows;
    bool is_pal;
    bool initialised;
    // set while the field rate is being measured, so a field interrupt
    // arriving mid-reconfiguration does not arm the channel
    volatile bool detecting;
    // set when the FIFO has run dry, cleared at the next field
    volatile bool desynced;
    volatile bool core1_ready;
    volatile bool core1_failed;

    // check_standard()'s measuring window, and the standard it has asked the
    // core1 thread to apply: 0 none, 1 NTSC, 2 PAL
    uint32_t std_window_ms;
    uint32_t std_window_fields;
    uint8_t std_mismatches;
    volatile uint8_t pending_standard;
};

#endif  // AP_OSD_PICO_ENABLED
