/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO UART driver — 8N1, no-DMA, IRQ-driven RX.
 * WIP: skeleton — verify on real RP2350 hardware.
 */

#include "PIOUART.h"
#include "RP2350_pio1.h"

#if defined(HAL_HAVE_PIO_UARTS) && HAL_HAVE_PIO_UARTS > 0

#include <AP_HAL/AP_HAL.h>
#include <hal.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;

extern const AP_HAL::HAL &hal;

// GPIO funcsel for PIO0=6, PIO1=7
#define RP_GPIO_FUNCSEL_PIO0  6U
#define RP_GPIO_FUNCSEL_PIO1  7U

// Default RX ring-buffer size (TX goes direct to FIFO — no staging buffer needed)
static const uint16_t PIO_UART_RX_BUF = 512;
static const uint16_t PIO_UART_TX_BUF = 512;

// TX FIFO depth per state machine (not joined)
static const uint8_t PIO_TX_FIFO_DEPTH = 8U;   // FJOIN_TX, see _start_tx_sm()

// Extract TX fill level for SM sm from PIO FLEVEL register.
// FLEVEL layout: bits [sm*8+3:sm*8] = TX fill, bits [sm*8+7:sm*8+4] = RX fill
static inline uint32_t pio_tx_level(PIO_TypeDef *pio, uint8_t sm)
{
    return (pio->FLEVEL >> (sm * 8U)) & 0xFU;
}

// NVIC priority for PIO UART RX IRQ
#define PIO_UART_IRQ_PRIO  5

#define RP2350_PIOUART2_STAGE_BEGIN_ENTER      0x01U
#define RP2350_PIOUART2_STAGE_PROG_UPLOADED    0x02U
#define RP2350_PIOUART2_STAGE_GPIO_CONFIGURED  0x03U
#define RP2350_PIOUART2_STAGE_SMS_STARTED      0x04U
#define RP2350_PIOUART2_STAGE_IRQ_ENABLED      0x05U
#define RP2350_PIOUART2_STAGE_BEGIN_DONE       0x06U

#if defined(RP2350)
// Temporary RP2350 bring-up breadcrumb for PIOUART2 (SERIAL5) reboot-loop diagnosis.
// SCRATCH[0] is fastboot, SCRATCH[1] is bootloader handoff, SCRATCH[6] is watchdog, SCRATCH[7] is global reset-cause breadcrumb, so use SCRATCH[5] here.
#define RP2350_PIOUART_DIAG_SCRATCH_IDX        5U
#define RP2350_PIOUART2_DIAG_MAGIC             0x50320000U
#endif

// --------------------------------------------------------------------------- Static members ---------------------------------------------------------------------------

PIORXDriver *PIORXDriver::_instances[PIO_NUM_INSTANCES];
bool         PIORXDriver::_pgm_loaded[2];

/*
  Bring-up and fault instrumentation, read over SWD. Off by default: several
  of these live in the receive interrupt, which runs once per byte at 420
  kbaud, and the whole handler now sits in Scratch X where space is scarce.
  Set AP_PIOUART_DEBUG_ENABLED to 1 in the hwdef to build them in.

  What each is for is recorded in hwdef/RPI_UAVFC/DEVELOPMENT.md.
 */
#ifndef AP_PIOUART_DEBUG_ENABLED
#define AP_PIOUART_DEBUG_ENABLED 0
#endif

#if AP_PIOUART_DEBUG_ENABLED
#define PIOUART_DBG(...) do { __VA_ARGS__ } while (0)
#else
#define PIOUART_DBG(...) do { } while (0)
#endif

#if AP_PIOUART_DEBUG_ENABLED
volatile uint32_t pio_uart_dbg_begin_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_ctor_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_calls[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_bytes[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_rx_service_calls[PIO_NUM_INSTANCES];
/*
  Faults the driver previously could not see. RXSTALL latches when a blocking
  push met a full FIFO, so it counts bytes the state machine had to drop;
  before this the loss was silent. The framing flag is irq 4+sm raised by the
  RX programs when a stop bit was not where it should be.
 */
uint32_t pio_uart_rx_overrun_count[PIO_NUM_INSTANCES];
uint32_t pio_uart_rx_framing_count[PIO_NUM_INSTANCES];
// Time spent in the shared vector. RXNEMPTY has no watermark, so at 420 kbaud
// this fires once per byte - every 19 us inside a frame - and the question is
// what that costs the core it lands on.
uint32_t pio_uart_irq_us_total[PIO_NUM_INSTANCES];
uint32_t pio_uart_irq_us_max[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_rx_bytes[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_begin_reentry[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_irq_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_irq_max_drain[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_last_fstat[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_last_stage[PIO_NUM_INSTANCES];
/*
  Every byte the receiver pops, before half duplex drops its own echo. On a
  shared pin that makes the transmitter self-checking: if the wire carries what
  we meant to say, it lands here, which separates "our framing is wrong" from
  "the far end is not listening" without a scope.
 */
uint8_t  pio_uart_dbg_rx_trace[PIO_NUM_INSTANCES][64];
uint16_t pio_uart_dbg_rx_trace_ofs[PIO_NUM_INSTANCES];

static inline void pio_uart_debug_stage_mark(const uint8_t instance, const uint8_t stage, const uint8_t aux)
{
    const uint32_t marker = ((uint32_t)stage << 8) | aux;
    pio_uart_dbg_last_stage[instance] = marker;

#if defined(RP2350)
    if (instance == 2U) {
        // Persist across SYSRESETREQ to show how far SERIAL5 init progressed.
        WATCHDOG->SCRATCH[RP2350_PIOUART_DIAG_SCRATCH_IDX] = RP2350_PIOUART2_DIAG_MAGIC | marker;
    }
#endif
}
#else
static inline void pio_uart_debug_stage_mark(uint8_t, uint8_t, uint8_t) {}
#endif  // AP_PIOUART_DEBUG_ENABLED


// InstanceConfig: pin numbers from hwdef.h PIORXDRIVERn_TX/RX_PIN defines
const PIORXDriver::InstanceConfig PIORXDriver::_cfg_table[PIO_NUM_INSTANCES] = {
#if PIO_NUM_INSTANCES >= 1
    { PIO0, 0, 1, PIOUART0_TX_PIN, PIOUART0_RX_PIN, RP_PIO0_IRQ_0_NUMBER },
#endif
#if PIO_NUM_INSTANCES >= 2
    { PIO0, 2, 3, PIOUART1_TX_PIN, PIOUART1_RX_PIN, RP_PIO0_IRQ_1_NUMBER },
#endif
#if PIO_NUM_INSTANCES >= 3
    { PIO1, 0, 1, PIOUART2_TX_PIN, PIOUART2_RX_PIN, RP_PIO1_IRQ_0_NUMBER },
#endif
#if PIO_NUM_INSTANCES >= 4
    { PIO1, 2, 3, PIOUART3_TX_PIN, PIOUART3_RX_PIN, RP_PIO1_IRQ_1_NUMBER },
#endif
};

// --------------------------------------------------------------------------- ChibiOS IRQ handlers (C linkage, vector table entries) ---------------------------------------------------------------------------

// Forward declarations suppress -Wmissing-declarations on the IRQ handlers below.
// CH_IRQ_HANDLER(x) expands to 'extern "C" void x(void)'
extern "C" {
CH_IRQ_HANDLER(RP_PIO0_IRQ_0_HANDLER);
CH_IRQ_HANDLER(RP_PIO0_IRQ_1_HANDLER);
#if PIO_NUM_INSTANCES >= 3
CH_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER);
#endif
#if PIO_NUM_INSTANCES >= 4
CH_IRQ_HANDLER(RP_PIO1_IRQ_1_HANDLER);
#endif
} // extern "C" (declarations)

extern "C" {

CH_IRQ_HANDLER(RP_PIO0_IRQ_0_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio0_0();
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(RP_PIO0_IRQ_1_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio0_1();
    CH_IRQ_EPILOGUE();
}

/*
  Gated like the instance table above. Without this a board that instantiates
  no PIO1 UART still claims both PIO1 vectors, which collides with anything
  else that owns the block - the OSD scan-out is 31 of PIO1's 32 instruction
  slots and needs its own vsync interrupt.
 */
#if PIO_NUM_INSTANCES >= 3
CH_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio1_0();
    CH_IRQ_EPILOGUE();
}
#endif

#if PIO_NUM_INSTANCES >= 4
CH_IRQ_HANDLER(RP_PIO1_IRQ_1_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio1_1();
    CH_IRQ_EPILOGUE();
}
#endif

} // extern "C"

void PIORXDriver::_irq_pio0_0()
{
#if PIO_NUM_INSTANCES > 0
    if (_instances[0]) {
        PIOUART_DBG(pio_uart_dbg_irq_count[0]++;);
        _instances[0]->_service_irq();
    }
#endif
}

void PIORXDriver::_irq_pio0_1()
{
#if PIO_NUM_INSTANCES > 1
    if (_instances[1]) {
        PIOUART_DBG(pio_uart_dbg_irq_count[1]++;);
        _instances[1]->_service_irq();
    }
#endif
}

void PIORXDriver::_irq_pio1_0()
{
#if PIO_NUM_INSTANCES > 2
    if (_instances[2]) {
        PIOUART_DBG(pio_uart_dbg_irq_count[2]++;);
        _instances[2]->_service_irq();
    }
#endif
}

void PIORXDriver::_irq_pio1_1()
{
#if PIO_NUM_INSTANCES > 3
    if (_instances[3]) {
        PIOUART_DBG(pio_uart_dbg_irq_count[3]++;);
        _instances[3]->_service_irq();
    }
#endif
}

// --------------------------------------------------------------------------- Constructor ---------------------------------------------------------------------------

PIORXDriver::PIORXDriver(uint8_t instance)
    : _instance(instance)
    , _initialized(false)
    , _active_rxinv(false)
    , _active_hdplex(false)
    , _active_baud(0)
    , _hd_enabled(false)
    , _hd_echo_active(false)
    , _stop_bits(1)
    , _active_stop_bits(0)
    , _readbuf(nullptr)
    , _writebuf(nullptr)
    , _sbus_rx{{0}, 0, 0}
{
    if (instance < PIO_NUM_INSTANCES) {
        _instances[instance] = this;
        PIOUART_DBG(pio_uart_dbg_ctor_count[instance]++;);
    }
}

// --------------------------------------------------------------------------- Private helpers ---------------------------------------------------------------------------

void PIORXDriver::_calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div)
{
// Use the runtime-configured clk_sys value so PIO UART baud generation stays aligned with the actual RP2350 clock tree, even if the board overrides PLL/post-divider settings from the rp_clocks.h defaults.
    uint32_t sys_clk = rp_clock_get_hz(RP_CLK_SYS);
    if (sys_clk == 0U) {
        sys_clk = RP_CLK_SYS_FREQ;
    }
    const uint32_t divisor  = PIO_UART_CYCLES_PER_BIT * baud;
    int_div  = sys_clk / divisor;
    frac_div = ((sys_clk % divisor) * 256U + divisor / 2U) / divisor;
    if (int_div  < 1U)   { int_div  = 1U; }
    if (int_div  > 65535U){ int_div  = 65535U; }
    if (frac_div > 255U) { frac_div = 255U; }
}

void PIORXDriver::_configure_gpio(uint8_t pin, bool is_output)
{
    const uint32_t funcsel = (cfg().pio == PIO0) ? RP_GPIO_FUNCSEL_PIO0
                                                 : RP_GPIO_FUNCSEL_PIO1;
/*
  In half duplex one pad carries both directions, so it needs the input buffer
  and a pull as well as drive strength: the pull is what holds the line in the
  gaps, when neither end is driving it.

  Which way it pulls is the caller's business, not ours. SmartAudio asks for a
  pull-down and means it - Betaflight has a dedicated SERIAL_PULL_SMARTAUDIO
  for the same thing, commented "the SA protocol usually requires pulldowns".
  A pull-up here looks harmless and leaves the far end unable to talk.
 */
    const bool half_duplex = _hd_enabled && (pin == cfg().tx_pin);
    const bool drive = is_output && !half_duplex;
    const bool pulldown = half_duplex
        ? (option_is_set(Option::OPTION_PULLDOWN_TX) || option_is_set(Option::OPTION_PULLDOWN_RX))
        : option_is_set(Option::OPTION_PULLDOWN_RX);
    iomode_t mode;
    if (half_duplex) {
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_DRIVE4
             | PAL_RP_PAD_IE
             | (pulldown ? PAL_RP_PAD_PDE : PAL_RP_PAD_PUE)
             | PAL_RP_PAD_SCHMITT;
    } else if (is_output) {
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_DRIVE4
             | PAL_RP_PAD_IE
             | PAL_RP_PAD_SCHMITT;
    } else {
// RX pin should be a plain peripheral input with pull-up.
// Open-drain mode here can distort idle/high levels and produce framing noise on loopback or externally-driven UART lines.
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_IE
             | (pulldown ? PAL_RP_PAD_PDE : PAL_RP_PAD_PUE)
             | PAL_RP_PAD_SCHMITT;
    }
// RP2350B has 48 GPIOs split across two PAL ports: IOPORT1 (port 0) covers GPIO 0-31, IOPORT2 (port 1) covers GPIO 32-47.
// palSetPadMode() silently returns if pad >= PAL_IOPORTS_WIDTH (32), so we must use IOPORT2 with the local pad offset for extended-range pins.
    if (pin < 32U) {
        palSetPadMode(IOPORT1, pin, mode);
    } else {
        palSetPadMode(IOPORT2, pin - 32U, mode);
    }

// Keep GPIO direction sane even when routed to PIO function.
// RP2350: bring-up, explicitly setting SIO OE avoids silent TX pins staying as inputs when PIO pin-direction state is not latched yet.
    if (pin < 32U) {
        if (drive) {
            SIO->GPIO_OE_SET = (1u << pin);
        } else {
            SIO->GPIO_OE_CLR = (1u << pin);
        }
    } else {
        const uint32_t bit = 1u << (pin - 32U);
        if (drive) {
            SIO->GPIO_HI_OE_SET = bit;
        } else {
            SIO->GPIO_HI_OE_CLR = bit;
        }
    }

/*
  Inverted protocols are handled in the pad rather than the program, so one
  receive program serves both: INOVER on the way in, OUTOVER on the way out,
  each two bits of GPIOn_CTRL with 01 meaning invert. SBUS idles low, so the
  state machine sees ordinary UART levels once INOVER has flipped it.

  Both MUST be written after palSetPadMode(), which writes the whole
  IO_BANK0 GPIOn_CTRL register and would otherwise clear them.
 */
    const bool invert = half_duplex
        ? (option_is_set(Option::OPTION_TXINV) || option_is_set(Option::OPTION_RXINV))
        : (is_output ? option_is_set(Option::OPTION_TXINV)
                     : option_is_set(Option::OPTION_RXINV));
    if (invert) {
        volatile uint32_t *gpio_ctrl =
            reinterpret_cast<volatile uint32_t *>(0x40028004U + (uint32_t)pin * 8U);
        // OUTOVER is bits 9:8, INOVER bits 17:16. A half-duplex pad needs both,
        // which is also why either inversion option implies the other there -
        // the same convention the ST driver uses.
        uint32_t ctrl = *gpio_ctrl;
        if (half_duplex || is_output) {
            ctrl = (ctrl & ~(3U << 8U)) | (1U << 8U);
        }
        if (half_duplex || !is_output) {
            ctrl = (ctrl & ~(3U << 16U)) | (1U << 16U);
        }
        *gpio_ctrl = ctrl;
    }
}

bool PIORXDriver::_upload_programs()
{
    const uint8_t pio_idx = (cfg().pio == PIO0) ? 0U : 1U;
    if (_pgm_loaded[pio_idx]) {
        return true;
    }
    /*
      PIO1 is shared with the analog OSD scan-out and the LED driver, and all
      three claims are blind writes to INSTR_MEM with nothing in the hardware
      to detect a collision. Ask before writing.
     */
    if (pio_idx == 1U && !pio1_claim(PIO1Owner::PIOUART)) {
        return false;
    }
    PIO_TypeDef *const pio = cfg().pio;

// RP2350 keeps many peripherals asserted in reset until explicitly released.
// Ensure the selected PIO block is live before touching any PIO registers, otherwise register writes are ignored.
    rp_peripheral_unreset((pio_idx == 0U) ? RESETS_ALLREG_PIO0
                                           : RESETS_ALLREG_PIO1);

    pio->CTRL = 0U; // stop all SMs

    // RP2350 PIO PINCTRL BASE fields are 5-bit (0-31). With GPIOBASE=0 (default),
    // GPIO32+ is inaccessible — a 5-bit value truncates modulo 32, so GPIO34
    // becomes GPIO2. Set GPIOBASE=16 to shift the window to GPIO16-47, making
    // GPIO20/21 (PIOUART0, rel 4/5) and GPIO34/35 (PIOUART1, rel 18/19) both
    // reachable. Only 0 and 16 are valid per the RP2350 datasheet (bit 4 only).
    // rp2350.h incorrectly marks GPIOBASE as __I, so write via raw pointer.
    (*reinterpret_cast<volatile uint32_t *>(reinterpret_cast<uintptr_t>(pio) + 0x168U)) = 16U;

    // Start from a known IRQ mask state; RX polling is used during bring-up.
    pio->IRQ0_INTE = 0U;
    pio->IRQ1_INTE = 0U;

    for (uint8_t i = 0; i < PIO_UART_TX_PROG_LEN; i++) {
        pio->INSTR_MEM[PIO_UART_TX_PROG_OFFSET + i] = k_pio_uart_tx_pgm[i];
    }
    for (uint8_t i = 0; i < PIO_UART_RX_PROG_LEN; i++) {
        pio->INSTR_MEM[PIO_UART_RX_PROG_OFFSET + i] = k_pio_uart_rx_pgm[i];
    }
    for (uint8_t i = 0; i < PIO_UART_RX_SBUS_PROG_LEN; i++) {
        pio->INSTR_MEM[PIO_UART_RX_SBUS_PROG_OFFSET + i] = k_pio_uart_rx_sbus_pgm[i];
    }

    _pgm_loaded[pio_idx] = true;

    return true;
}

void PIORXDriver::_start_tx_sm(uint32_t int_div, uint32_t frac_div)
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm    = cfg().sm_tx;
    const uint8_t      tx_pin = cfg().tx_pin;

    pio->CTRL &= ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm));

    pio->SM[sm].CLKDIV = (int_div  << PIO_CLKDIV_INT_LSB)
                       | (frac_div << PIO_CLKDIV_FRAC_LSB);

    pio->SM[sm].EXECCTRL =
          ((uint32_t)(PIO_UART_TX_PROG_OFFSET + PIO_UART_TX_PROG_LEN - 1)
                       << PIO_EXECCTRL_WRAP_TOP_LSB)
        | ((uint32_t)PIO_UART_TX_PROG_OFFSET << PIO_EXECCTRL_WRAP_BOT_LSB)
        | PIO_EXECCTRL_STATUS_TX_EMPTY  // what 'mov x, status' reports
        | (1u << 30); // SIDE_EN: enable optional sideset bit used by uart_tx

    // this state machine only transmits, so the RX half of its FIFO is dead
    // weight - join it to get 8 entries instead of 4
    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_OUT_SHIFTDIR | PIO_SHIFTCTRL_FJOIN_TX;

    // PINCTRL BASE fields are 5-bit and GPIOBASE-relative (GPIOBASE=16 set in
    // _upload_programs). GPIO34 → rel 18, GPIO20 → rel 4.
    const uint8_t rel_tx = tx_pin - 16U;

    // SIDE_EN consumes one bit in Delay/Side-set, so one actual side-set data
    // bit requires SIDESET_COUNT=2 (enable+data).
    const uint32_t pinctrl_base =
          (2u              << PIO_PINCTRL_SIDESET_COUNT_LSB)
        | ((uint32_t)rel_tx << PIO_PINCTRL_SIDESET_BASE_LSB)
        | ((uint32_t)rel_tx << PIO_PINCTRL_OUT_BASE_LSB)
        | (1u               << PIO_PINCTRL_OUT_COUNT_LSB)
        | ((uint32_t)rel_tx << PIO_PINCTRL_SET_BASE_LSB);

    // Set up the pin while the state machine is stopped, with SET pointed at
    // it: idle level high, then the direction each mode starts in. Half duplex
    // starts released and the program takes the line per frame; full duplex
    // drives from here on and its two 'set pindirs' become no-ops below.
    pio->SM[sm].PINCTRL = pinctrl_base | (1u << PIO_PINCTRL_SET_COUNT_LSB);
    pio->SM[sm].INSTR = PIO_UART_INSTR_SET_PINS_1;
    pio->SM[sm].INSTR = PIO_UART_INSTR_SET_PINDIRS(_hd_enabled ? 0u : 1u);

    // A SET_COUNT of 0 in full duplex is what disarms the turnaround: the
    // side-set on those instructions still applies, only the pindirs write is
    // dropped, so one program serves both modes.
    pio->SM[sm].PINCTRL = pinctrl_base
        | ((_hd_enabled ? 1u : 0u) << PIO_PINCTRL_SET_COUNT_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

    // Y holds the number of *extra* stop bits and nothing in the program
    // writes it, so it survives until the next time through here. SM_RESTART
    // does not clear X/Y either, hence after the restart rather than before.
    pio->SM[sm].INSTR = PIO_UART_INSTR_SET_Y((_stop_bits >= 2) ? 1u : 0u);
    pio->SM[sm].INSTR = PIO_UART_INSTR_JMP(PIO_UART_TX_PROG_OFFSET);

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

/*
  Which receive program is loaded. Half duplex restarts the receiver every time
  the line is handed back, and after a restart the PC reads 0 - the transmit
  program - so it has to be pointed at the right entry point explicitly.
 */
uint32_t PIORXDriver::_rx_prog_offset() const
{
    return option_is_set(Option::OPTION_RXINV) ? PIO_UART_RX_SBUS_PROG_OFFSET
                                               : PIO_UART_RX_PROG_OFFSET;
}

void PIORXDriver::_start_rx_sm(uint32_t int_div, uint32_t frac_div)
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm    = cfg().sm_rx;
    // half duplex listens on the wire it transmits on
    const uint8_t      rx_pin = _hd_enabled ? cfg().tx_pin : cfg().rx_pin;

    // Choose between the standard 8N1/8N2 RX program and the SBUS 8E2 program
    // (which adds a parity-bit skip after the 8 data bits).
    const uint32_t rx_offset = _rx_prog_offset();
    const uint32_t rx_len = option_is_set(Option::OPTION_RXINV)
        ? PIO_UART_RX_SBUS_PROG_LEN
        : PIO_UART_RX_PROG_LEN;

    pio->CTRL &= ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm));

    pio->SM[sm].CLKDIV = (int_div  << PIO_CLKDIV_INT_LSB)
                       | (frac_div << PIO_CLKDIV_FRAC_LSB);

    // PINCTRL IN_BASE and EXECCTRL JMP_PIN are 5-bit and GPIOBASE-relative.
    // GPIO35 → rel 19, GPIO21 → rel 5.
    const uint8_t rel_rx = rx_pin - 16U;

    pio->SM[sm].EXECCTRL =
          ((uint32_t)(rx_offset + rx_len - 1) << PIO_EXECCTRL_WRAP_TOP_LSB)
        | ((uint32_t)rx_offset << PIO_EXECCTRL_WRAP_BOT_LSB)
        | ((uint32_t)rel_rx << PIO_EXECCTRL_JMP_PIN_LSB);

    // RX program uses explicit 'push noblock' after stop-bit validation,
    // so AUTOPUSH must remain disabled.
    // receive only, so join the TX half in: eight entries of slack against
    // interrupt latency rather than four
    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_IN_SHIFTDIR | PIO_SHIFTCTRL_FJOIN_RX;

    pio->SM[sm].PINCTRL = ((uint32_t)rel_rx << PIO_PINCTRL_IN_BASE_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

// Force the RX SM PC to the correct program entry point.
// After restart, SM PC defaults to 0 which runs the TX program (pull block) and stalls.
    const uint32_t jmp_rx_prog = (rx_offset & 0x1FU);
    pio->SM[sm].INSTR = jmp_rx_prog;

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

/*
  On a shared pin the receiver hears everything we send, and a SmartAudio
  request opens with the same two sync bytes as a SmartAudio reply - so
  `read_response()` would parse our own request straight back as an answer.
  Betaflight passes the echo up and lets its protocol layer deal with it;
  ArduPilot's convention is the other way round, and the ST driver drops it in
  the HAL behind `hd_tx_active`. Same thing here.

  The purge has to happen before the flag clears and with the interrupt held
  off, or the last echoed byte - which reaches the FIFO at its stop bit, right
  as the transmitter finishes - gets committed by an interrupt landing in
  between.
 */
void PIORXDriver::_hd_echo_check()
{
    if (!_hd_echo_active || tx_pending()) {
        return;
    }
    nvicDisableVector(cfg().irq_num);
    {
        PIO_TypeDef *const pio = cfg().pio;
        const uint8_t      sm  = cfg().sm_rx;
        while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
            (void)pio->RXF[sm];
        }
        _hd_echo_active = false;
    }
    nvicEnableVector(cfg().irq_num, PIO_UART_IRQ_PRIO);
}

void PIORXDriver::_enable_rx_irq()
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm_rx = cfg().sm_rx;
    const uint32_t rx_mask = PIO_INTE_RX_NOTEMPTY(sm_rx);

// Drop any stale pending data before enabling IRQ-driven RX.
// During bring-up, random FIFO residue can otherwise cause an immediate IRQ retrigger loop before the driver is fully initialized.
    while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm_rx)))) {
        (void)pio->RXF[sm_rx];
    }

    if (sm_rx <= 1U) {
// Enable exactly one RXNEMPTY source for this instance.
// Preserving previous bits can leave unrelated sources enabled, causing immediate interrupt storms during early init.
        pio->IRQ0_INTF = 0U;
        pio->IRQ0_INTE = rx_mask;
    } else {
        pio->IRQ1_INTF = 0U;
        pio->IRQ1_INTE = rx_mask;
    }

    nvicEnableVector(cfg().irq_num, PIO_UART_IRQ_PRIO);
}

/*
  Which of the two interrupt enable registers this instance owns. The vector
  is chosen per instance in the config table and the two PIO0 UARTs take one
  each, so the register follows the receive state machine index.
 */
volatile uint32_t *PIORXDriver::_inte_reg() const
{
    PIO_TypeDef *const pio = cfg().pio;
    return (cfg().sm_rx <= 1U) ? &pio->IRQ0_INTE : &pio->IRQ1_INTE;
}

/*
  Sticky fault flags, both write-one-to-clear.

  RXSTALL latches when a blocking push met a full FIFO - bytes the state
  machine had to drop. The framing flag is irq 4+sm from the receive
  programs, raised when a stop bit was not where it should be.

  Deliberately not inside the FIFO drain: a line held low raises framing
  errors and produces no bytes at all, so a check that only runs when data
  arrived would never see the fault it exists to report. Called from the
  interrupt and from _available(), which the protocol layers poll whether or
  not anything is being received.
 */
void PIORXDriver::_poll_pio_errors()
{
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_rx;

    if ((pio->FDEBUG & PIO_FDEBUG_RXSTALL(sm)) != 0U) {
        pio->FDEBUG = PIO_FDEBUG_RXSTALL(sm);
        PIOUART_DBG(pio_uart_rx_overrun_count[_instance]++;);
    }
    if ((pio->IRQ & PIO_IRQ_FRAMING_FLAG(sm)) != 0U) {
        pio->IRQ = PIO_IRQ_FRAMING_FLAG(sm);
        PIOUART_DBG(pio_uart_rx_framing_count[_instance]++;);
    }
}

// Both directions share one vector, so both are checked on every entry.
void PIORXDriver::_service_irq()
{
#if AP_PIOUART_DEBUG_ENABLED
    const uint32_t entry_us = TIMER0->TIMERAWL;
#endif

    _poll_pio_errors();
    _service_rx_fifo();
    _drain_tx_fifo();

#if AP_PIOUART_DEBUG_ENABLED
    const uint32_t spent = TIMER0->TIMERAWL - entry_us;
    pio_uart_irq_us_total[_instance] += spent;
    if (spent > pio_uart_irq_us_max[_instance]) {
        pio_uart_irq_us_max[_instance] = spent;
    }
#endif
}

/*
  Arm the transmit interrupt. _drain_tx_fifo() disarms it again once the ring
  is empty, so the source is only enabled while there is something to send -
  otherwise TXNFULL is true whenever the FIFO has room, which is almost
  always, and the interrupt never stops firing.
 */
void PIORXDriver::_enable_tx_irq()
{
    *_inte_reg() |= PIO_INTE_TX_NOTFULL(cfg().sm_tx);
}

// --------------------------------------------------------------------------- ISR: drain RX FIFO into ring buffer ---------------------------------------------------------------------------

void PIORXDriver::_service_rx_fifo()
{
    PIOUART_DBG(pio_uart_dbg_rx_service_calls[_instance]++;);

    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_rx;

    volatile uint8_t *const rxfifo_byte = ((volatile uint8_t *)&pio->RXF[sm]) + 3;
    uint32_t drained = 0;
    const bool sbus_sanitize = _active_rxinv && (_active_baud == 100000U);

// Always drain hardware FIFO if data is present.
// If this runs before normal initialization has completed, discarding bytes here prevents an IRQ retrigger storm that can starve the main loop.
    while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
// For right-shifted UART RX, the received byte is left-justified in RXF bits [31:24].
// The RP2350 datasheet's UART RX example reads the FIFO as an 8-bit access at RXF+3, which pops the FIFO and returns that upper byte directly.
        const uint8_t byte = *rxfifo_byte;
        PIOUART_DBG(
            pio_uart_dbg_rx_bytes[_instance]++;
            const uint16_t t = pio_uart_dbg_rx_trace_ofs[_instance];
            if (t < 64U) {
                pio_uart_dbg_rx_trace[_instance][t] = byte;
                pio_uart_dbg_rx_trace_ofs[_instance] = t + 1U;
            }
        );
        drained++;
        // still pop it, or the FIFO fills and RXSTALL latches
        if (_readbuf && _initialized && !_hd_echo_active) {
            if (!sbus_sanitize) {
                _readbuf->write(&byte, 1);
            } else {
// SBUS on PIOUART: assemble full frames and only forward valid 25-byte packets.
// This keeps framing garbage out of the upper protocol layer and improves failsafe stability.
                if (_sbus_rx.ofs == 0U && byte != 0x0FU) {
                    continue;
                }
                _sbus_rx.buf[_sbus_rx.ofs++] = byte;
                if (_sbus_rx.ofs == 25U) {
                    uint8_t flags = _sbus_rx.buf[23];
                    const uint8_t footer = _sbus_rx.buf[24];
                    const bool footer_ok = (footer == 0x00U) || (footer == 0x04U) ||
                                           (footer == 0x14U) || (footer == 0x24U) ||
                                           (footer == 0x34U);
                    const bool flags_ok = (flags & 0xF0U) == 0U;
                    if (footer_ok && flags_ok) {
// Debounce single-frame SBUS failsafe-flag spikes caused by occasional UART framing noise: require 3 consecutive flagged frames before forwarding FAILSAFE bit to upper layers.
                        if (flags & (1U << 3)) {
                            if (_sbus_rx.fs_count < 255U) {
                                _sbus_rx.fs_count++;
                            }
                            if (_sbus_rx.fs_count < 3U) {
                                flags &= ~(1U << 3);
                                _sbus_rx.buf[23] = flags;
                            }
                        } else {
                            _sbus_rx.fs_count = 0U;
                        }
                        _readbuf->write(_sbus_rx.buf, 25U);
                        _sbus_rx.ofs = 0U;
                    } else {
                        uint8_t new_ofs = 0U;
                        for (uint8_t i = 1; i < 25U; i++) {
                            if (_sbus_rx.buf[i] == 0x0FU) {
                                new_ofs = 25U - i;
                                memmove(_sbus_rx.buf, &_sbus_rx.buf[i], new_ofs);
                                break;
                            }
                        }
                        _sbus_rx.ofs = new_ofs;
                    }
                }
            }
        }
    }

    PIOUART_DBG(
        if (drained > pio_uart_dbg_irq_max_drain[_instance]) {
            pio_uart_dbg_irq_max_drain[_instance] = drained;
        });
    PIOUART_DBG(pio_uart_dbg_last_fstat[_instance] = pio->FSTAT;);
}

// --------------------------------------------------------------------------- AP_HAL::UARTDriver protected virtual overrides ---------------------------------------------------------------------------

void PIORXDriver::_begin(uint32_t b, uint16_t rxSpace, uint16_t txSpace)
{
    if (_instance >= PIO_NUM_INSTANCES) {
        return;
    }
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_BEGIN_ENTER, 0U);

    if (b == 0) {
        b = 57600;
    }

    _sbus_rx.ofs = 0;
    _sbus_rx.fs_count = 0;

// SERIAL_CONTROL commonly calls begin() repeatedly with unchanged parameters.
// Reinitializing PIO SMs on each packet disrupts RX/TX and can inject framing noise into loopback tests.
    const bool rxinv = option_is_set(Option::OPTION_RXINV);
    const bool hdplex = option_is_set(Option::OPTION_HDPLEX);
    if (_initialized && _active_baud == b && _active_rxinv == rxinv
        && _active_hdplex == hdplex && _active_stop_bits == _stop_bits) {
        PIOUART_DBG(pio_uart_dbg_begin_reentry[_instance]++;);
        return;
    }

    if (rxSpace == 0) {
        rxSpace = PIO_UART_RX_BUF;
    }

    if (_readbuf == nullptr) {
        _readbuf = new ByteBuffer(rxSpace);
        if (!_readbuf) {
            return;
        }
    }

    if (txSpace == 0) {
        txSpace = PIO_UART_TX_BUF;
    }
    if (_writebuf == nullptr) {
        _writebuf = new ByteBuffer(txSpace);
        if (!_writebuf) {
            return;
        }
    }

    if (!_upload_programs()) {
        // PIO1 belongs to something else; leaving the pins alone is the only
        // safe answer, and the broker has already said who won
        return;
    }
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_PROG_UPLOADED, 0U);

    _hd_enabled = hdplex;
    _hd_echo_active = false;

    _configure_gpio(cfg().tx_pin, true);
    if (!hdplex) {
        // in half duplex the receive pad is not ours; leave it as the hwdef
        // left it rather than routing a second pin to a state machine that is
        // now listening somewhere else
        _configure_gpio(cfg().rx_pin, false);
    }
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_GPIO_CONFIGURED, 0U);

    uint32_t int_div, frac_div;
    _calc_clkdiv(b, int_div, frac_div);

    _start_tx_sm(int_div, frac_div);
    _start_rx_sm(int_div, frac_div);
    PIOUART_DBG(pio_uart_dbg_last_fstat[_instance] = cfg().pio->FSTAT;);
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_SMS_STARTED, 0U);

// Start each session from a clean RX state.
// During clock/pin bring-up, the RX SM can capture transient bits
    {
        PIO_TypeDef *const pio = cfg().pio;
        const uint8_t sm = cfg().sm_rx;
        while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
            (void)pio->RXF[sm];
        }
        if (_readbuf) {
            _readbuf->clear();
        }
    }

    // Mark initialized before enabling RX IRQ so ISR writes can safely append
    // into the software ring buffer as soon as bytes start arriving.
    _initialized = true;
    _active_baud = b;
    _active_rxinv = rxinv;
    _active_hdplex = hdplex;
    _active_stop_bits = _stop_bits;
    _enable_rx_irq();
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_IRQ_ENABLED, 0U);
    PIOUART_DBG(pio_uart_dbg_begin_count[_instance]++;);
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_BEGIN_DONE, (uint8_t)(cfg().sm_rx & 0xFFU));
}

void PIORXDriver::_end()
{
    if (!_initialized) {
        return;
    }
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm_tx = cfg().sm_tx;
    const uint8_t      sm_rx = cfg().sm_rx;

    pio->CTRL &= ~((1u << (PIO_CTRL_SM_ENABLE_LSB + sm_tx))
                 | (1u << (PIO_CTRL_SM_ENABLE_LSB + sm_rx)));

    nvicDisableVector(cfg().irq_num);
    if (sm_rx <= 1U) {
        pio->IRQ0_INTE &= ~PIO_INTE_RX_NOTEMPTY(sm_rx);
    } else {
        pio->IRQ1_INTE &= ~PIO_INTE_RX_NOTEMPTY(sm_rx);
    }

    _initialized = false;
    _active_baud = 0;
    _active_rxinv = false;
    _active_hdplex = false;
    _active_stop_bits = 0;
    _hd_enabled = false;
    _hd_echo_active = false;
    _sbus_rx.ofs = 0;
    _sbus_rx.fs_count = 0;
}

void PIORXDriver::_flush()
{
    if (!_initialized) {
        return;
    }

    // Wait for the hardware FIFO/shift engine to become idle so callers that
    // require synchronous transmission semantics can force completion.
    const uint32_t start_ms = AP_HAL::millis();
    while (tx_pending() && (AP_HAL::millis() - start_ms) < 50U) {
        hal.scheduler->delay_microseconds(50);
    }
    _hd_echo_check();
}

uint32_t PIORXDriver::_available()
{
    if (_initialized) {
        _poll_pio_errors();
        _hd_echo_check();
    }
    if (!_initialized || !_readbuf) {
        return 0;
    }
    return _readbuf->available();
}

bool PIORXDriver::_discard_input()
{
    if (!_initialized || !_readbuf) {
        return false;
    }
// ByteBuffer::clear() (head=tail=0) is not ISR-safe
// the RX ISR also writes to _readbuf via _service_rx_fifo().
// this is the same purge sequence used in _begin().
    nvicDisableVector(cfg().irq_num);
    {
        PIO_TypeDef *const pio = cfg().pio;
        const uint8_t      sm  = cfg().sm_rx;
        while (!(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
            (void)pio->RXF[sm];
        }
        _readbuf->clear();
    }
    nvicEnableVector(cfg().irq_num, PIO_UART_IRQ_PRIO);
    return true;
}

ssize_t PIORXDriver::_read(uint8_t *buffer, uint16_t count)
{
    if (!_initialized || !_readbuf || !buffer) {
        return -1;
    }
    return (ssize_t)_readbuf->read(buffer, count);
}

void PIORXDriver::_drain_tx_fifo()
{
    if (!_writebuf || !_initialized) {
        return;
    }
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;

    // Move as many bytes as the TX FIFO has free slots
    uint8_t byte;
    while (pio_tx_level(pio, sm) < PIO_TX_FIFO_DEPTH
           && _writebuf->read_byte(&byte)) {
        pio->TXF[sm] = (uint32_t)byte;
    }

    if (_writebuf->available() == 0) {
        *_inte_reg() &= ~PIO_INTE_TX_NOTFULL(sm);
    }
}

/*
  Buffer and return, the way UARTDriver::_write() does on the ST path - it
  takes its mutex, writes what fits and returns a possibly short count, and
  every byte reaches the wire from the transmit thread. Nothing waits.

  This used to push straight into the FIFO and spin on it for up to 20 ms a
  byte, because _drain_tx_fifo() existed but nothing ever called it, so a
  write with no follow-up would have sat in the ring for ever. Priming the
  FIFO here and arming the interrupt covers that case without blocking: the
  first eight bytes go immediately and the interrupt carries the rest.
 */
size_t PIORXDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized || !_writebuf || buffer == nullptr || size == 0) {
        return 0;
    }

    PIOUART_DBG(pio_uart_dbg_write_calls[_instance]++;);

    size_t written;
    {
        WITH_SEMAPHORE(_write_mutex);
        written = _writebuf->write(buffer, size);
    }

    if (_hd_enabled && written > 0) {
        _hd_echo_active = true;
    }

    // Start it moving now rather than waiting for the first interrupt, then
    // let the interrupt finish the job.
    _drain_tx_fifo();
    if (_writebuf->available() > 0) {
        _enable_tx_irq();
    }

    PIOUART_DBG(pio_uart_dbg_write_bytes[_instance] += written;);

    return written;
}

// --------------------------------------------------------------------------- AP_HAL::UARTDriver public virtual overrides ---------------------------------------------------------------------------

uint32_t PIORXDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    // _write() blocks on FIFO drain internally, so we can always accept up to
    // the TX ring-buffer size. Returning only the 4-byte hardware FIFO depth
    // causes HAVE_PAYLOAD_SPACE to be permanently false for every MAVLink
    // message (which are ≥17 bytes), silently dropping all GCS output.
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t sm = cfg().sm_tx;
    const uint32_t level = pio_tx_level(pio, sm);
    if (level >= PIO_TX_FIFO_DEPTH) {
        return 0;
    }
    return PIO_UART_TX_BUF;
}

/*
  Three places a byte can still be outstanding, and the FIFO is only one of
  them. The ring matters now that _write() queues into it rather than pushing
  straight at the hardware, and the state machine matters because an empty
  FIFO says nothing about the byte currently being shifted: it is only
  finished once it has come back to the blocking pull, which is where it
  waits for work. Betaflight's isTxComplete_pio() tests the same two things.

  One bit of slack remains. Reaching the pull applies its side-set, so the
  stop bit starts there and runs for the eight cycles of the delay; a caller
  switching a half duplex line around the instant this returns false could
  clip it. Wait a bit time if that matters.
 */
bool PIORXDriver::tx_pending()
{
    if (!_initialized) {
        return false;
    }
    if (_writebuf != nullptr && _writebuf->available() > 0) {
        return true;
    }

    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;
    if (!(pio->FSTAT & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm)))) {
        return true;
    }
    return pio->SM[sm].ADDR != (PIO_UART_TX_PROG_OFFSET + 1U);
}

/*
  The base class accepts nothing, which would leave SmartAudio's request for
  half duplex silently ignored on a port that can do it. Inversion and the
  half-duplex turnaround both live in the pad and the state machine setup, so
  applying a change means going back through _begin(); the no-change guard
  there is defeated deliberately rather than reproducing the setup here.

  Pull direction is fixed by the pad configuration each mode needs, and there
  is no DMA on this path, so those options are accepted and do nothing.
 */
bool PIORXDriver::set_options(uint16_t options)
{
    const uint16_t supported = OPTION_RXINV | OPTION_TXINV | OPTION_HDPLEX
                             | OPTION_PULLDOWN_RX | OPTION_PULLUP_RX
                             | OPTION_PULLDOWN_TX | OPTION_PULLUP_TX
                             | OPTION_NODMA_RX | OPTION_NODMA_TX;
    const uint16_t changed = _last_options ^ options;
    _last_options = options;

    if (_initialized
        && (changed & (OPTION_RXINV | OPTION_TXINV | OPTION_HDPLEX)) != 0) {
        const uint32_t baud = _active_baud;
        _active_baud = 0;
        _begin(baud, 0, 0);
    }

    return (options & ~supported) == 0;
}

/*
  Stop bits reach the wire through Y in the transmit program, so a change only
  takes effect at the next _begin(). AP_SmartAudio sets this before it calls
  begin() from its own thread, which is the order that matters; a later change
  on a running port is picked up by the guard in _begin().
 */
void PIORXDriver::set_stop_bits(int n)
{
    _stop_bits = (n >= 2) ? 2 : 1;
}

bool PIORXDriver::wait_timeout(uint16_t n, uint32_t timeout_ms)
{
    const uint32_t start = AP_HAL::millis();
    while (AP_HAL::millis() - start < timeout_ms) {
        if (_available() >= n) {
            return true;
        }
        hal.scheduler->delay_microseconds(100);
    }
    return _available() >= n;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#endif // HAL_HAVE_PIO_UARTS
