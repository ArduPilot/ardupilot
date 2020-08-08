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
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */
#pragma once

#include <AP_HAL/utility/RingBuffer.h>

#include "AP_HAL_ChibiOS.h"
#include "shared_dma.h"
#include "Semaphores.h"

#define RX_BOUNCE_BUFSIZE 64U
#define TX_BOUNCE_BUFSIZE 64U

// enough for uartA to uartH, plus IOMCU
#define UART_MAX_DRIVERS 9

class ChibiOS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uint8_t serial_num);

    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    void flush() override;
    bool is_initialized() override;
    void set_blocking_writes(bool blocking) override;
    bool tx_pending() override;


    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    ssize_t read(uint8_t *buffer, uint16_t count) override;
    int16_t read_locked(uint32_t key) override;
    void _timer_tick(void) override;

    bool discard_input() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    // lock a port for exclusive use. Use a key of 0 to unlock
    bool lock_port(uint32_t write_key, uint32_t read_key) override;

    // control optional features
    bool set_options(uint16_t options) override;
    uint8_t get_options(void) const override;

    // write to a locked port. If port is locked and key is not correct then 0 is returned
    // and write is discarded
    size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key) override;

    struct SerialDef {
        BaseSequentialStream* serial;
        bool is_usb;
#ifndef HAL_UART_NODMA
        bool dma_rx;
        uint8_t dma_rx_stream_id;
        uint32_t dma_rx_channel_id;
        bool dma_tx;
        uint8_t dma_tx_stream_id;
        uint32_t dma_tx_channel_id;
#endif
        ioline_t tx_line;
        ioline_t rx_line;
        ioline_t rts_line;
        int8_t rxinv_gpio;
        uint8_t rxinv_polarity;
        int8_t txinv_gpio;
        uint8_t txinv_polarity;
        uint8_t get_index(void) const {
            return uint8_t(this - &_serial_tab[0]);
        }
    };

    bool wait_timeout(uint16_t n, uint32_t timeout_ms) override;

    void set_flow_control(enum flow_control flow_control) override;
    enum flow_control get_flow_control(void) override { return _flow_control; }

    // allow for low latency writes
    bool set_unbuffered_writes(bool on) override;

    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.

      A return value of zero means the HAL does not support this API
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    uint32_t bw_in_kilobytes_per_second() const override {
        if (sdef.is_usb) {
            return 200;
        }
        return _baudrate/(9*1024);
    }

private:
    const SerialDef &sdef;
    bool rx_dma_enabled;
    bool tx_dma_enabled;

    // thread used for all UARTs
    static thread_t *uart_thread_ctx;

    // table to find UARTDrivers from serial number, used for event handling
    static UARTDriver *uart_drivers[UART_MAX_DRIVERS];

    // index into uart_drivers table
    uint8_t serial_num;

    // key for a locked port
    uint32_t lock_write_key;
    uint32_t lock_read_key;

    uint32_t _baudrate;
    uint16_t tx_len;
#if HAL_USE_SERIAL == TRUE
    SerialConfig sercfg;
#endif
    const thread_t* _uart_owner_thd;

    struct {
        // thread waiting for data
        thread_t *thread_ctx;
        // number of bytes needed
        uint16_t n;
    } _wait;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
#ifndef HAL_UART_NODMA
    bool tx_bounce_buf_ready;
    volatile uint8_t rx_bounce_idx;
    uint8_t *rx_bounce_buf[2];
    uint8_t *tx_bounce_buf;
#endif
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    HAL_Semaphore _write_mutex;
#ifndef HAL_UART_NODMA
    const stm32_dma_stream_t* rxdma;
    const stm32_dma_stream_t* txdma;
#endif
    virtual_timer_t tx_timeout;
    bool _in_timer;
    bool _blocking_writes;
    bool _initialised;
    bool _device_initialised;
#ifndef HAL_UART_NODMA
    Shared_DMA *dma_handle;
#endif
    static const SerialDef _serial_tab[];

    // timestamp for receiving data on the UART, avoiding a lock
    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx;

    // handling of flow control
    enum flow_control _flow_control = FLOW_CONTROL_DISABLE;
    bool _rts_is_active;
    uint32_t _last_write_completed_us;
    uint32_t _first_write_started_us;
    uint32_t _total_written;

    // we remember config options from set_options to apply on sdStart()
    uint32_t _cr1_options;
    uint32_t _cr2_options;
    uint32_t _cr3_options;
    uint16_t _last_options;

    // half duplex control. After writing we throw away bytes for 4 byte widths to
    // prevent reading our own bytes back
#if CH_CFG_USE_EVENTS == TRUE
    bool half_duplex;
    event_listener_t hd_listener;
    bool hd_tx_active;
    void half_duplex_setup_tx(void);
#endif

    // set to true for unbuffered writes (low latency writes)
    bool unbuffered_writes;

#if CH_CFG_USE_EVENTS == TRUE
    // listener for parity error events
    event_listener_t ev_listener;
    bool parity_enabled;
#endif

#ifndef HAL_UART_NODMA
    static void rx_irq_cb(void* sd);
#endif
    static void rxbuff_full_irq(void* self, uint32_t flags);
    static void tx_complete(void* self, uint32_t flags);
    static void handle_tx_timeout(void *arg);

#ifndef HAL_UART_NODMA
    void dma_tx_allocate(Shared_DMA *ctx);
    void dma_tx_deallocate(Shared_DMA *ctx);
    void dma_rx_enable(void);
#endif
    void update_rts_line(void);

    void check_dma_tx_completion(void);
#ifndef HAL_UART_NODMA
    void write_pending_bytes_DMA(uint32_t n);
#endif
    void write_pending_bytes_NODMA(uint32_t n);
    void write_pending_bytes(void);

    void receive_timestamp_update(void);

    // set SERIALn_OPTIONS for pullup/pulldown
    void set_pushpull(uint16_t options);

    void thread_init();
    static void uart_thread(void *);
};

// access to usb init for stdio.cpp
void usb_initialise(void);
