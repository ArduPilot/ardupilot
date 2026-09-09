/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO-based pseudo-UART driver for ArduPilot/ChibiOS.
 *
 * Implements up to 3 additional serial ports via RP2350 PIO state machines:
 *   PIOUART0: PIO0 SM0 (TX) + SM1 (RX)  — RX IRQ via PIO0 IRQ0
 *   PIOUART1: PIO0 SM2 (TX) + SM3 (RX)  — RX IRQ via PIO0 IRQ1
 *   PIOUART2: PIO1 SM0 (TX) + SM1 (RX)  — RX IRQ via PIO1 IRQ0
 * 
 * TX: PIO side-set. RX: PIO IN + autopush at 8 bits → ISR ring buffer.
 * Baud clock: sys_clk / (8 cycles_per_bit * baud_rate).
 *
 * Instruction words derived from pico-sdk uart_tx.pio / uart_rx.pio.
 * Source in hwdef/Pico2/pico_pio_uart.pio.
 *
 * WIP: skeleton — verify on real RP2350 hardware.
 */
#pragma once

#include "AP_HAL_ChibiOS.h"

#if defined(HAL_HAVE_PIO_UARTS) && HAL_HAVE_PIO_UARTS > 0

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <hal.h>

// --------------------------------------------------------------------------- PIO UART protocol constants ---------------------------------------------------------------------------

#define PIO_UART_CYCLES_PER_BIT  8U

// Instruction memory layout (24 of 32 words used per PIO): [0..4] TX program
// (8N1 transmit, side-set) [5..13] Standard RX (8N1/8N2, stop-bit validation)
// [14..23] SBUS RX (8E2, parity-bit skip + stop-bit validation)
#define PIO_UART_TX_PROG_OFFSET       0U
#define PIO_UART_TX_PROG_LEN          5U
#define PIO_UART_RX_PROG_OFFSET       5U
#define PIO_UART_RX_PROG_LEN          9U
#define PIO_UART_RX_SBUS_PROG_OFFSET  14U
#define PIO_UART_RX_SBUS_PROG_LEN     10U

// --------------------------------------------------------------------------- Pre-assembled PIO UART programs ---------------------------------------------------------------------------

// TX: side_set 1 opt, 8 cycles/bit, loaded at offset 0 0xE081: set pindirs, 1
// one-time pin direction init 0x9FA0: pull block side 1 [7] 0xF727: set x, 7 side 0 [7] 0x6001: out pins, 1 0x0643: jmp x--, 3 [6]
static const uint16_t k_pio_uart_tx_pgm[PIO_UART_TX_PROG_LEN] = {
    0xE081u, 0x9FA0u, 0xF727u, 0x6001u, 0x0643u,
};

/*
  Standard 8N1/8N2 receiver, pre-relocated for offset 5.

  The stop bit is validated before the byte is pushed, and a framing error
  resynchronises by waiting for the line to return to idle rather than going
  straight back to hunting for a start bit. Without that wait a line held low
  - a break, an unplugged transmitter, a receiver powered before its source -
  satisfies "wait 0 pin" immediately and the state machine emits a continuous
  stream of 0x00 at full baud rate. Taken from pico-examples uart_rx.pio,
  which Betaflight also uses unmodified.

  irq 4 rel sets a flag the CPU can poll but that cannot raise an interrupt:
  PIO routes only flags 0-3 to INTE, so 4-7 are free for exactly this.
 */
static const uint16_t k_pio_uart_rx_pgm[PIO_UART_RX_PROG_LEN] = {
    0x2020u,  //  5: wait  0 pin, 0     start bit
    0xEA27u,  //  6: set   x, 7 [10]    delay to bit-0 centre
    0x4001u,  //  7: in    pins, 1
    0x0647u,  //  8: jmp   x--, 7 [6]   loop 8 times
    0x00CDu,  //  9: jmp   pin, 13      stop bit high - accept the byte
    0xC014u,  // 10: irq   nowait 4 rel framing error, pollable flag
    0x20A0u,  // 11: wait  1 pin, 0     resync: hold until the line is idle
    0x0005u,  // 12: jmp   5
    0x8020u,  // 13: push  block
};

/*
  SBUS receiver, 8E2, pre-relocated for offset 14. The wire is inverted by
  GPIO INOVER before it reaches the state machine, so levels here are ordinary
  UART levels: idle high, start low, stop high.

  Same framing-error resync as the standard program above - SBUS at 100 kbaud
  from an unpowered receiver is exactly the held-low case that produces an
  endless 0x00 stream without it.
 */
static_assert(PIO_UART_RX_SBUS_PROG_OFFSET == 14U,
    "SBUS RX pgm has hardcoded absolute targets (14, 16, 23) - update if offset changes");
static const uint16_t k_pio_uart_rx_sbus_pgm[PIO_UART_RX_SBUS_PROG_LEN] = {
    0x2020u,  // 14: wait  0 pin, 0     start bit
    0xEA27u,  // 15: set   x, 7 [10]    delay to bit-0 centre
    0x4001u,  // 16: in    pins, 1
    0x0650u,  // 17: jmp   x--, 16 [6]  loop 8 times
    0xA642u,  // 18: mov   y, y [6]     stall through the parity bit
    0x00D7u,  // 19: jmp   pin, 23      stop bit high - accept the byte
    0xC014u,  // 20: irq   nowait 4 rel framing error, pollable flag
    0x20A0u,  // 21: wait  1 pin, 0     resync: hold until the line is idle
    0x000Eu,  // 22: jmp   14
    0x8020u,  // 23: push  block
};

// --------------------------------------------------------------------------- PIO register bit-field constants ---------------------------------------------------------------------------

#define PIO_CTRL_SM_ENABLE_LSB        0u
#define PIO_CTRL_SM_RESTART_LSB       8u
#define PIO_CTRL_CLKDIV_RESTART_LSB  12u

// RP2350 PIO FSTAT layout (datasheet Table 983):
// RXFULL[3:0], RXEMPTY[11:8], TXFULL[19:16], TXEMPTY[27:24].
#define PIO_FSTAT_RXFULL_LSB          0u
#define PIO_FSTAT_RXEMPTY_LSB         8u
#define PIO_FSTAT_TXFULL_LSB         16u
#define PIO_FSTAT_TXEMPTY_LSB        24u

#define PIO_CLKDIV_FRAC_LSB           8u
#define PIO_CLKDIV_INT_LSB           16u

#define PIO_EXECCTRL_WRAP_BOT_LSB     7u
#define PIO_EXECCTRL_WRAP_TOP_LSB    12u
#define PIO_EXECCTRL_JMP_PIN_LSB     24u

#define PIO_SHIFTCTRL_AUTOPUSH       (1u << 16)
#define PIO_SHIFTCTRL_AUTOPULL       (1u << 17)
#define PIO_SHIFTCTRL_IN_SHIFTDIR    (1u << 18)
#define PIO_SHIFTCTRL_OUT_SHIFTDIR   (1u << 19)
// SHIFTCTRL is PUSH_THRESH 24:20 and PULL_THRESH 29:25, per
// PIO_SM_SHIFTCTRL_*_THRESH_Pos in ChibiOS rp_pio.h. Neither threshold is
// used by this driver - both UART programs pull and push explicitly - so
// these two were wrong for a long time without any UART noticing.
#define PIO_SHIFTCTRL_PUSH_THRESH_LSB 20u
// Each UART state machine uses one direction only, so the unused half of its
// FIFO can be given to the half in use: 8 entries instead of 4.
#ifndef PIO_SHIFTCTRL_FJOIN_TX
#define PIO_SHIFTCTRL_FJOIN_TX       (1u << 30)
#endif
#ifndef PIO_SHIFTCTRL_FJOIN_RX
#define PIO_SHIFTCTRL_FJOIN_RX       (1u << 31)
#endif
// PIO_FDEBUG_RXSTALL comes from rp_pio.h: bits 0-3, one per state machine,
// latching when a blocking push met a full FIFO. That is the RX overrun this
// driver previously could not see.
// irq 4 rel from the RX programs lands on flag 4+sm, outside the 0-3 the
// interrupt enable registers can reach.
#define PIO_IRQ_FRAMING_FLAG(sm)     (1u << (4u + (sm)))
#define PIO_SHIFTCTRL_PULL_THRESH_LSB 25u

#define PIO_PINCTRL_OUT_BASE_LSB      0u
#define PIO_PINCTRL_SET_BASE_LSB      5u
#define PIO_PINCTRL_SIDESET_BASE_LSB 10u
#define PIO_PINCTRL_IN_BASE_LSB      15u
#define PIO_PINCTRL_OUT_COUNT_LSB    20u
#define PIO_PINCTRL_SET_COUNT_LSB    26u
#define PIO_PINCTRL_SIDESET_COUNT_LSB 29u

// RX FIFO not-empty interrupt bit for SM sm (in IRQx_INTE/INTS on RP2350).
// RP2350 maps SM0..SM3 RXNEMPTY to bits 0..3.
#define PIO_INTE_RX_NOTEMPTY(sm)  (1u << (sm))
#define PIO_INTE_TX_NOTFULL(sm)   (1u << ((sm) + 4u))

#ifdef HAL_HAVE_PIO_UARTS
#define PIO_NUM_INSTANCES  HAL_HAVE_PIO_UARTS
#else
#define PIO_NUM_INSTANCES  4U
#endif

// --------------------------------------------------------------------------- PIORXDriver class.
// inherits AP_HAL::UARTDriver ---------------------------------------------------------------------------

namespace ChibiOS {

class PIORXDriver final : public AP_HAL::UARTDriver {
public:
    explicit PIORXDriver(uint8_t instance);
    CLASS_NO_COPY(PIORXDriver);

    // ---- Public virtual overrides required by AP_HAL::UARTDriver ----
    bool is_initialized() override { return _initialized; }
    bool tx_pending() override;
    uint32_t txspace() override;
    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override;

    // ---- ISR dispatch handlers ----
    static void _irq_pio0_0();
    static void _irq_pio0_1();
    static void _irq_pio1_0();
    static void _irq_pio1_1();

    // ---- ISR worker ----
    void _service_rx_fifo();

protected:
    // ---- Protected pure-virtual overrides from AP_HAL::UARTDriver ----
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    struct InstanceConfig {
        PIO_TypeDef  *pio;
        uint8_t       sm_tx;
        uint8_t       sm_rx;
        uint8_t       tx_pin;
        uint8_t       rx_pin;
        uint8_t       irq_num;
    };

    static const InstanceConfig _cfg_table[PIO_NUM_INSTANCES];
    static PIORXDriver          *_instances[PIO_NUM_INSTANCES];
    static bool                  _pgm_loaded[2];   // [0]=PIO0, [1]=PIO1

    const uint8_t _instance;
    bool  _initialized;
    bool  _active_rxinv;
    uint32_t _active_baud;
    // _write() only ever fills this and returns; the transmit interrupt
    // empties it. Nothing waits on the wire.
    HAL_Semaphore _write_mutex;
    ByteBuffer *_readbuf;
    ByteBuffer *_writebuf;

    struct {
        uint8_t buf[25];
        uint8_t ofs;
        uint8_t fs_count;
    } _sbus_rx;

    const InstanceConfig &cfg() const { return _cfg_table[_instance]; }

    // false when PIO1 is already owned by the OSD scan-out or the LED
    // driver; the caller must not go on to configure pins or start SMs
    bool _upload_programs();
    void _start_tx_sm(uint32_t clkdiv_int, uint32_t clkdiv_frac);
    void _start_rx_sm(uint32_t clkdiv_int, uint32_t clkdiv_frac);
    void _configure_gpio(uint8_t pin, bool is_output);
    void _enable_rx_irq();
    void _drain_tx_fifo();
    // one entry point for the shared PIO vector: errors, then RX, then TX
    void _service_irq();
    // sticky fault flags, polled where they will be seen even when no byte
    // has arrived - a line stuck low raises framing errors and nothing else
    void _poll_pio_errors();
    void _enable_tx_irq();
    volatile uint32_t *_inte_reg() const;

    static void _calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div);
};

} // namespace ChibiOS

#endif // HAL_HAVE_PIO_UARTS
