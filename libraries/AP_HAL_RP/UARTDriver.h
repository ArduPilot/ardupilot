#pragma once

#include "AP_HAL_RP.h"
#include <AP_HAL/utility/RingBuffer.h>
#include "hardware/uart.h"
#include <sys/types.h>
#include <cstdlib>
#include <cstdio>

class RP::UARTDriver : public AP_HAL::UARTDriver {
public:
    UARTDriver(uart_inst_t *uart_hw, uint8_t tx_pin, uint8_t rx_pin, uint8_t instance);

    bool is_initialized() override { return _initialized; }
    bool tx_pending() override;
    
    // Options and configuration
    bool set_options(uint16_t options) override;
    //uint16_t get_options(void) const override { return _last_options; }
    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;
    uint32_t get_baud_rate() const override { return _baudrate; }

    void handle_irq();

    uint32_t txspace() override;
    void vprintf(const char *fmt, va_list ap) override;

    uint64_t receive_time_constraint_us(uint16_t nbytes) override;

private:
    uart_inst_t *_uart;
    uint8_t _tx_pin, _rx_pin, _instance;
    uint32_t _baudrate;
    bool _initialized;
    
    ByteBuffer _readbuf;
    ByteBuffer _writebuf;

    HAL_Semaphore _write_mutex;    

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    void _end() override;
    void _flush() override;

    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override WARN_IF_UNUSED;

    uint32_t _available() override;
    bool _discard_input(void) override;

    void _timer_tick() override;
};
