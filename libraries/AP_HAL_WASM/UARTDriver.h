#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#include <stdint.h>
#include <stddef.h>

#include "AP_HAL_WASM_Namespace.h"

extern "C" {
    size_t ardupilot_serial0_write(const uint8_t *buf, size_t len);
    size_t ardupilot_serial0_read(uint8_t *buf, size_t max_len);
    size_t ardupilot_serial0_read_available(void);
}

/*
 * Ring-buffer UART driver for serial0 / MAVLink.
 *
 * TX is written by the autopilot and read by JS. RX is written by JS and read
 * by the autopilot. Each direction supports one producer thread and one
 * consumer thread.
 */
class HALWASM::UARTDriver : public AP_HAL::UARTDriver {
public:
    bool is_initialized() override { return _initialized; }
    bool tx_pending() override;
    uint32_t txspace() override;

    size_t js_write(const uint8_t *buf, size_t len);
    size_t js_read(uint8_t *buf, size_t max_len);
    size_t js_read_available() const;

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    void _end() override;
    void _flush() override;
    uint32_t _available() override;
    bool _discard_input() override;

private:
    static constexpr size_t BUF_SIZE = 16384;

    ByteBuffer _tx_buf{BUF_SIZE};
    ByteBuffer _rx_buf{BUF_SIZE};

    bool _initialized = false;
};
