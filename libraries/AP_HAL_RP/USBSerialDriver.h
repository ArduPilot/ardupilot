#pragma once

#include "AP_HAL_RP.h"
#include <AP_HAL/utility/RingBuffer.h>

class RP::USBSerialDriver : public AP_HAL::UARTDriver {
public:
    USBSerialDriver();

    bool is_initialized() override { return _initialized; }
    bool tx_pending() override;
    bool set_options(uint16_t options) override;
    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;
    uint32_t get_baud_rate() const override { return _baudrate; }

    uint32_t txspace() override;
    void vprintf(const char *fmt, va_list ap) override;
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    void _timer_tick() override;

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void _end() override;
    void _flush() override;

    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;

    uint32_t _available() override;
    bool _discard_input(void) override;

private:
    void _poll_rx();
    void _flush_tx();

    uint32_t _baudrate;
    bool _initialized;
    ByteBuffer _readbuf;
    ByteBuffer _writebuf;
    HAL_Semaphore _write_mutex;
};
