#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL/Semaphores.h>

class UAVCAN_UARTDriver : public AP_HAL::UARTDriver {
public:
    UAVCAN_UARTDriver();

    /* Empty implementations of UARTDriver virtual methods */
    void begin(uint32_t b) override;
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;
    void end() override;
    bool is_initialized() override { return _initialised; }
    bool tx_pending() override;

    // unimplemented
    void flush() override {}
    void set_blocking_writes(bool blocking) override { (void)blocking; }

    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    uint32_t handle_inbound(const uint8_t *buffer, uint32_t size);
    int16_t fetch_for_outbound(void);
    uint32_t tx_available() { return _writebuf.available(); }

private:
    bool _initialised;
    AP_HAL::Semaphore *_write_mutex;
    AP_HAL::Semaphore *_read_mutex;

    // ring buffers
    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
};
