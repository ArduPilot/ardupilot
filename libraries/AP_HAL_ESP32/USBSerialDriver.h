#pragma once

#include <AP_HAL/UARTDriver.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/AP_HAL_ESP32.h>
#include <AP_HAL_ESP32/Semaphores.h>

#include "ardupilot_tusb.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace ESP32
{

class USBSerialDriver : public AP_HAL::UARTDriver
{
public:
    USBSerialDriver();
    virtual ~USBSerialDriver() = default;

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;
    void _timer_tick() override;
    uint32_t bw_in_bytes_per_second() const override;
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;
    uint32_t get_baud_rate() const override;
    uint32_t get_usb_baud() const override;
    uint8_t get_usb_parity() const override;
    void vprintf(const char *fmt, va_list ap) override;

    static bool is_usb_connected();

private:
    static constexpr size_t TX_BUF_SIZE = 1024;
    static constexpr size_t RX_BUF_SIZE = 1024;

    bool _initialized;
    uint8_t _buffer[64];
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    Semaphore _write_mutex;
    Semaphore _read_mutex;
    uint64_t _receive_timestamp[2] {};
    uint8_t _receive_timestamp_idx;
    uint32_t _baudrate;
    const tskTaskControlBlock *_uart_owner_thd;
    uint32_t _tx_tick_counter;
    uint32_t _write_calls;
    uint32_t _tx_bytes_queued;
    uint32_t _tx_bytes_sent_to_tusb;
    uint32_t _flush_failures;
    uint32_t _last_write_ms;
    uint32_t _last_progress_ms;
    uint32_t _last_diag_ms;
    uint32_t _last_flush_log_ms;
    esp_err_t _last_flush_ret;
    static USBSerialDriver *_singleton;
    static volatile bool _mounted;
    static volatile bool _port_open;

    void read_data();
    void write_data();
    void _receive_timestamp_update();
    void handle_rx();
    static void rx_callback(void *arg);
    static void line_state_callback(bool dtr, bool rts, void *arg);

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    bool _discard_input() override;
};

} // namespace ESP32
