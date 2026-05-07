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
static const uint8_t PIO_TX_FIFO_DEPTH = 4U;

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
#define RP2350_PIOUART2_STAGE_WRITE_TIMEOUT    0xE1U

#if defined(RP2350)
// Temporary RP2350 bring-up breadcrumb for PIOUART2 (SERIAL5) reboot-loop diagnosis.
// SCRATCH[0] is fastboot, SCRATCH[1] is bootloader handoff, SCRATCH[6] is watchdog, SCRATCH[7] is global reset-cause breadcrumb, so use SCRATCH[5] here.
#define RP2350_PIOUART_DIAG_SCRATCH_IDX        5U
#define RP2350_PIOUART2_DIAG_MAGIC             0x50320000U
#endif

// --------------------------------------------------------------------------- Static members ---------------------------------------------------------------------------

PIORXDriver *PIORXDriver::_instances[PIO_NUM_INSTANCES];
bool         PIORXDriver::_pgm_loaded[2];

// Bring-up debug counters read via SWD when diagnosing silent TX/RX paths.
volatile uint32_t pio_uart_dbg_begin_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_ctor_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_calls[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_bytes[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_rx_service_calls[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_rx_bytes[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_begin_reentry[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_irq_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_irq_max_drain[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_write_timeout_count[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_last_fstat[PIO_NUM_INSTANCES];
volatile uint32_t pio_uart_dbg_last_stage[PIO_NUM_INSTANCES];

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
CH_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER);
CH_IRQ_HANDLER(RP_PIO1_IRQ_1_HANDLER);
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

CH_IRQ_HANDLER(RP_PIO1_IRQ_0_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio1_0();
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(RP_PIO1_IRQ_1_HANDLER)
{
    CH_IRQ_PROLOGUE();
    PIORXDriver::_irq_pio1_1();
    CH_IRQ_EPILOGUE();
}

} // extern "C"

void PIORXDriver::_irq_pio0_0()
{
#if PIO_NUM_INSTANCES > 0
    if (_instances[0]) {
        pio_uart_dbg_irq_count[0]++;
        _instances[0]->_service_rx_fifo();
    }
#endif
}

void PIORXDriver::_irq_pio0_1()
{
#if PIO_NUM_INSTANCES > 1
    if (_instances[1]) {
        pio_uart_dbg_irq_count[1]++;
        _instances[1]->_service_rx_fifo();
    }
#endif
}

void PIORXDriver::_irq_pio1_0()
{
#if PIO_NUM_INSTANCES > 2
    if (_instances[2]) {
        pio_uart_dbg_irq_count[2]++;
        _instances[2]->_service_rx_fifo();
    }
#endif
}

void PIORXDriver::_irq_pio1_1()
{
#if PIO_NUM_INSTANCES > 3
    if (_instances[3]) {
        pio_uart_dbg_irq_count[3]++;
        _instances[3]->_service_rx_fifo();
    }
#endif
}

// --------------------------------------------------------------------------- Constructor ---------------------------------------------------------------------------

PIORXDriver::PIORXDriver(uint8_t instance)
    : _instance(instance)
    , _initialized(false)
    , _active_rxinv(false)
    , _active_baud(0)
    , _readbuf(nullptr)
    , _writebuf(nullptr)
    , _sbus_rx{{0}, 0, 0}
{
    if (instance < PIO_NUM_INSTANCES) {
        _instances[instance] = this;
        pio_uart_dbg_ctor_count[instance]++;
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
    iomode_t mode;
    if (is_output) {
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_DRIVE4
             | PAL_RP_PAD_IE
             | PAL_RP_PAD_SCHMITT;
    } else {
// RX pin should be a plain peripheral input with pull-up.
// Open-drain mode here can distort idle/high levels and produce framing noise on loopback or externally-driven UART lines.
        mode = PAL_RP_IOCTRL_FUNCSEL(funcsel)
             | PAL_RP_PAD_IE
             | PAL_RP_PAD_PUE
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
        if (is_output) {
            SIO->GPIO_OE_SET = (1u << pin);
        } else {
            SIO->GPIO_OE_CLR = (1u << pin);
        }
    } else {
        const uint32_t bit = 1u << (pin - 32U);
        if (is_output) {
            SIO->GPIO_HI_OE_SET = bit;
        } else {
            SIO->GPIO_HI_OE_CLR = bit;
        }
    }

// For inverted-UART protocols (SBUS, option OPTION_RXINV): set GPIO INOVER=01 so the PIO SM sees standard UART levels (idle HIGH) from the idle-LOW SBUS wire signal.
// MUST be written AFTER palSetPadMode() because that function writes the full IO_BANK0 GPIOx_CTRL register and would clear INOVER.
    if (!is_output && option_is_set(Option::OPTION_RXINV)) {
        volatile uint32_t *gpio_ctrl =
            reinterpret_cast<volatile uint32_t *>(0x40028004U + (uint32_t)pin * 8U);
        *gpio_ctrl = (*gpio_ctrl & ~(3U << 16)) | (1U << 16);
    }
}

void PIORXDriver::_upload_programs()
{
    const uint8_t pio_idx = (cfg().pio == PIO0) ? 0U : 1U;
    if (_pgm_loaded[pio_idx]) {
        return;
    }
    PIO_TypeDef *const pio = cfg().pio;

// RP2350 keeps many peripherals asserted in reset until explicitly released.
// Ensure the selected PIO block is live before touching any PIO registers, otherwise register writes are ignored.
    rp_peripheral_unreset((pio_idx == 0U) ? RESETS_ALLREG_PIO0
                                           : RESETS_ALLREG_PIO1);

    pio->CTRL = 0U; // stop all SMs
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
        | ((uint32_t)(PIO_UART_TX_PROG_OFFSET + 1U) << PIO_EXECCTRL_WRAP_BOT_LSB)
        | (1u << 30); // SIDE_EN: enable optional sideset bit used by uart_tx

    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_OUT_SHIFTDIR;

        pio->SM[sm].PINCTRL =
            // SIDE_EN consumes one bit in Delay/Side-set, so one actual
            // side-set data bit requires SIDESET_COUNT=2 (enable+data).
            (2u               << PIO_PINCTRL_SIDESET_COUNT_LSB)
                | ((uint32_t)tx_pin << PIO_PINCTRL_SIDESET_BASE_LSB)
                | ((uint32_t)tx_pin << PIO_PINCTRL_OUT_BASE_LSB)
                | (1u               << PIO_PINCTRL_OUT_COUNT_LSB)
                | ((uint32_t)tx_pin << PIO_PINCTRL_SET_BASE_LSB)
                | (1u               << PIO_PINCTRL_SET_COUNT_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
}

void PIORXDriver::_start_rx_sm(uint32_t int_div, uint32_t frac_div)
{
    PIO_TypeDef *const pio   = cfg().pio;
    const uint8_t      sm    = cfg().sm_rx;
    const uint8_t      rx_pin = cfg().rx_pin;

    // Choose between the standard 8N1/8N2 RX program and the SBUS 8E2 program
    // (which adds a parity-bit skip after the 8 data bits).
    const uint32_t rx_offset = option_is_set(Option::OPTION_RXINV)
        ? PIO_UART_RX_SBUS_PROG_OFFSET
        : PIO_UART_RX_PROG_OFFSET;
    const uint32_t rx_len = option_is_set(Option::OPTION_RXINV)
        ? PIO_UART_RX_SBUS_PROG_LEN
        : PIO_UART_RX_PROG_LEN;

    pio->CTRL &= ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm));

    pio->SM[sm].CLKDIV = (int_div  << PIO_CLKDIV_INT_LSB)
                       | (frac_div << PIO_CLKDIV_FRAC_LSB);

    pio->SM[sm].EXECCTRL =
          ((uint32_t)(rx_offset + rx_len - 1) << PIO_EXECCTRL_WRAP_TOP_LSB)
        | ((uint32_t)rx_offset << PIO_EXECCTRL_WRAP_BOT_LSB)
        | ((uint32_t)rx_pin << PIO_EXECCTRL_JMP_PIN_LSB);

    // RX program uses explicit 'push noblock' after stop-bit validation,
    // so AUTOPUSH must remain disabled.
    pio->SM[sm].SHIFTCTRL = PIO_SHIFTCTRL_IN_SHIFTDIR;

    pio->SM[sm].PINCTRL = ((uint32_t)rx_pin << PIO_PINCTRL_IN_BASE_LSB);

    pio->CTRL |= (1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm))
              |  (1u << (PIO_CTRL_SM_RESTART_LSB      + sm));

// Force the RX SM PC to the correct program entry point.
// After restart, SM PC defaults to 0 which runs the TX program (pull block) and stalls.
    const uint32_t jmp_rx_prog = (rx_offset & 0x1FU);
    pio->SM[sm].INSTR = jmp_rx_prog;

    pio->CTRL |= (1u << (PIO_CTRL_SM_ENABLE_LSB + sm));
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

// --------------------------------------------------------------------------- ISR: drain RX FIFO into ring buffer ---------------------------------------------------------------------------

void PIORXDriver::_service_rx_fifo()
{
    pio_uart_dbg_rx_service_calls[_instance]++;

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
        pio_uart_dbg_rx_bytes[_instance]++;
        drained++;
        if (_readbuf && _initialized) {
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

    if (drained > pio_uart_dbg_irq_max_drain[_instance]) {
        pio_uart_dbg_irq_max_drain[_instance] = drained;
    }
    pio_uart_dbg_last_fstat[_instance] = pio->FSTAT;
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
    if (_initialized && _active_baud == b && _active_rxinv == rxinv) {
        pio_uart_dbg_begin_reentry[_instance]++;
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

    _upload_programs();
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_PROG_UPLOADED, 0U);

    _configure_gpio(cfg().tx_pin, true);
    _configure_gpio(cfg().rx_pin, false);
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_GPIO_CONFIGURED, 0U);

    uint32_t int_div, frac_div;
    _calc_clkdiv(b, int_div, frac_div);

    _start_tx_sm(int_div, frac_div);
    _start_rx_sm(int_div, frac_div);
    pio_uart_dbg_last_fstat[_instance] = cfg().pio->FSTAT;
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
    _enable_rx_irq();
    pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_IRQ_ENABLED, 0U);
    pio_uart_dbg_begin_count[_instance]++;
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
}

uint32_t PIORXDriver::_available()
{
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
}

size_t PIORXDriver::_write(const uint8_t *buffer, size_t size)
{
    if (!_initialized || !buffer || size == 0) {
        return 0;
    }

    pio_uart_dbg_write_calls[_instance]++;

    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t sm = cfg().sm_tx;
    size_t written = 0;

// Direct FIFO writes avoid dependence on a periodic TX refill callback.
// This is important for SERIAL_CONTROL where one write() call may enqueue the full payload and no subsequent write occurs to trigger draining.
    for (size_t i = 0; i < size; i++) {
        const uint32_t wait_start_us = AP_HAL::micros();
        while (pio_tx_level(pio, sm) >= PIO_TX_FIFO_DEPTH) {
            if ((AP_HAL::micros() - wait_start_us) > 20000U) {
                pio_uart_dbg_write_timeout_count[_instance]++;
                pio_uart_debug_stage_mark(_instance, RP2350_PIOUART2_STAGE_WRITE_TIMEOUT,
                                          (uint8_t)(written & 0xFFU));
                return written;
            }
            hal.scheduler->delay_microseconds(20);
        }
        pio->TXF[sm] = (uint32_t)buffer[i];
        written++;
    }

    pio_uart_dbg_write_bytes[_instance] += written;

    return written;
}

// --------------------------------------------------------------------------- AP_HAL::UARTDriver public virtual overrides ---------------------------------------------------------------------------

uint32_t PIORXDriver::txspace()
{
    if (!_initialized) {
        return 0;
    }
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t sm = cfg().sm_tx;
    const uint32_t level = pio_tx_level(pio, sm);
    return (level < PIO_TX_FIFO_DEPTH) ? (PIO_TX_FIFO_DEPTH - level) : 0U;
}

bool PIORXDriver::tx_pending()
{
    if (!_initialized) {
        return false;
    }
    // Pending while the PIO TX FIFO has bytes not yet shifted out.
    PIO_TypeDef *const pio = cfg().pio;
    const uint8_t      sm  = cfg().sm_tx;
    return !(pio->FSTAT & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm)));
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
