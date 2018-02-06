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

#define RX_BOUNCE_BUFSIZE 128
#define TX_BOUNCE_BUFSIZE 64

#define UART_MAX_DRIVERS 7

class ChibiOS::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uint8_t serial_num);

    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();


    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;
    void _timer_tick(void) override;

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);
    struct SerialDef {
        BaseSequentialStream* serial;
        bool is_usb;
        bool dma_rx;
        uint8_t dma_rx_stream_id;
        uint32_t dma_rx_channel_id;
        bool dma_tx;
        uint8_t dma_tx_stream_id;
        uint32_t dma_tx_channel_id; 
        ioline_t rts_line;
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
    
private:
    bool tx_bounce_buf_ready;
    const SerialDef &sdef;

    // thread used for all UARTs
    static thread_t *uart_thread_ctx;

    // last time we ran the uart thread
    static uint32_t last_thread_run_us;
    
    // table to find UARTDrivers from serial number, used for event handling
    static UARTDriver *uart_drivers[UART_MAX_DRIVERS];

    // index into uart_drivers table
    uint8_t serial_num;
    
    uint32_t _baudrate;
    uint16_t tx_len;
    SerialConfig sercfg;
    const thread_t* _uart_owner_thd;

    struct {
        // thread waiting for data
        thread_t *thread_ctx;
        // number of bytes needed
        uint16_t n;
    } _wait;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    uint8_t rx_bounce_buf[RX_BOUNCE_BUFSIZE];
    uint8_t tx_bounce_buf[TX_BOUNCE_BUFSIZE];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    mutex_t _write_mutex;
    const stm32_dma_stream_t* rxdma;
    const stm32_dma_stream_t* txdma;
    bool _in_timer;
    bool _nonblocking_writes;
    bool _initialised;
    bool _device_initialised;
    bool _lock_rx_in_timer_tick = false;
    Shared_DMA *dma_handle;
    static const SerialDef _serial_tab[];

    // handling of flow control
    enum flow_control _flow_control = FLOW_CONTROL_DISABLE;
    bool _rts_is_active;
    uint32_t _last_write_completed_us;
    uint32_t _first_write_started_us;

    // set to true for unbuffered writes (low latency writes)
    bool unbuffered_writes;
    
    static void rx_irq_cb(void* sd);
    static void rxbuff_full_irq(void* self, uint32_t flags);
    static void tx_complete(void* self, uint32_t flags);

    void dma_tx_allocate(void);
    void dma_tx_deallocate(void);
    void update_rts_line(void);

    void write_pending_bytes_DMA(uint32_t n);
    void write_pending_bytes_NODMA(uint32_t n);
    void write_pending_bytes(void);

    void thread_init();
    static void uart_thread(void *);
};
