/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>

#include "Semaphores.h"

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
#include <zephyr/device.h>
struct uart_event;
#endif

namespace Zephyr {

class UARTDriver : public AP_HAL::UARTDriver {
public:
    explicit UARTDriver(uint8_t serial_num);

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;

    uint32_t get_baud_rate() const override
    {
        return _baudrate;
    }

    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

#if HAL_UART_STATS_ENABLED
    void uart_info(ExpandingString &str, StatsTracker &stats, const uint32_t dt_ms) override;
#endif

    /* AP_RCProtocol's serial_configs[] cycling reopens the port repeatedly, so
     * anything logged per-open floods the console. */
    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;
    bool set_options(uint16_t options) override;

    /* Hardware RTS/CTS flow control. RT1176's LPUART genuinely implements it, unlike
     * the generic path which only pretends to. */
    void set_flow_control(enum flow_control flow_control_setting) override;
    enum flow_control get_flow_control(void) override { return _flow_control; }

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void _end() override;
    void _flush() override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    static constexpr uint16_t DEFAULT_RX_BUF_SIZE = 2048;  // bytes
    static constexpr uint16_t DEFAULT_TX_BUF_SIZE = 1024;  // bytes

#ifdef __ZEPHYR__
    static void _irq_handler(const struct device *dev, void *user_data);
    void _drain_rx_fifo();
    void _fill_tx_fifo();
    /* Periodic fallback pump, registered once via register_timer_process. */
    void _tx_timer_tick();
    void _rx_timer_tick();
    /* Single registration point for both ticks above - see the comment on
       ZEPHYR_SCHED_MAX_TIMER_PROCS in Scheduler.h for why this must stay
       ONE registration per UART instance, not two. */
    void _uart_timer_tick() { _tx_timer_tick(); _rx_timer_tick(); }
    uint8_t _parity = 0;       // 0=none 1=odd 2=even, matches AP_HAL's own convention
    int _stop_bits = 1;
    enum flow_control _flow_control = FLOW_CONTROL_DISABLE;
    const struct device *_dev = nullptr;
    bool _lookup_warned = false;
    k_tid_t _owner_tid = nullptr;
    bool _uart_timer_registered = false;

    /* Async (eDMA) path, taken by instances whose devicetree node carries `dmas`. */
    static constexpr uint16_t RX_DMA_BUF_SIZE = 128;  // 2 allocated; ~3ms at 416kbaud
    static constexpr uint16_t TX_DMA_BUF_SIZE = 128;  // one staged chunk in flight
    static constexpr int32_t  RX_DMA_TIMEOUT_US = 1000;  // idle flush latency
    static void _async_cb(const struct device *dev, struct uart_event *evt, void *user_data);
    void _tx_dma_kick();
    bool _use_async = false;
    volatile bool _tx_dma_busy = false;

public:
    /* Per-port byte/event counters, reported on the console every 10 s. */
    volatile uint32_t _dbg_tx_queued = 0;    // bytes handed to _write()
    volatile uint32_t _dbg_tx_dma = 0;       // bytes handed to uart_tx()
    volatile uint32_t _dbg_tx_done = 0;      // bytes confirmed sent (TX_DONE)
    volatile uint32_t _dbg_tx_fail = 0;      // uart_tx() rejections
    volatile uint32_t _dbg_rx_bytes = 0;     // bytes received
    volatile uint32_t _dbg_rx_events = 0;    // RX_RDY events
private:
    volatile bool _rx_need_restart = false;
    volatile bool _rx_dma_next_is_1 = false;
    uint8_t *_rx_dma_buf[2] = {};   // __nocache pool allocations
    uint8_t *_tx_dma_buf = nullptr; // __nocache pool allocation
#endif

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    bool _initialized = false;
    uint8_t _serial_num;
    uint32_t _baudrate = 0;
    uint32_t _rx_dropped = 0;

    uint64_t _receive_timestamp[2] = {};
    uint8_t _receive_timestamp_idx = 0;

    void _receive_timestamp_update();
};

}  // namespace Zephyr
