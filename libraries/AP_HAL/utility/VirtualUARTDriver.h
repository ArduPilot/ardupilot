#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_HAL_VIRTUAL_UART_ENABLED
#define AP_HAL_VIRTUAL_UART_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if AP_HAL_VIRTUAL_UART_ENABLED

#include <AP_HAL/UARTDriver.h>
#include "RingBuffer.h"

namespace AP_HAL
{

// A byte-stream UART whose transport is serviced by a software backend.
class VirtualUARTDriver : public UARTDriver
{
public:
    explicit VirtualUARTDriver(uint8_t instance);
    ~VirtualUARTDriver();

    // Instances are numbered independently of their SERIAL_ORDER positions.
    static VirtualUARTDriver *get_uart(uint8_t instance);

    // Backend operations are independent of the UART frontend's port lock.
    size_t read_tx(uint8_t *buffer, uint16_t count);
    size_t write_rx(const uint8_t *buffer, size_t size);
    uint32_t rxspace();

    bool is_initialized() override;
    bool tx_pending() override;
    uint32_t txspace() override;
    uint32_t get_baud_rate() const override
    {
        return _baudrate.load();
    }

private:
    static constexpr uint8_t MAX_INSTANCES = 2;
    static constexpr uint16_t DEFAULT_CAPACITY = 512;
    // Bound per-port memory use; an oversized begin leaves the port unchanged.
    static constexpr uint16_t MAX_CAPACITY = 4096;
    static VirtualUARTDriver *_instances[MAX_INSTANCES];

    void _begin(uint32_t baud, uint16_t rx_space, uint16_t tx_space) override;
    size_t _write(const uint8_t *buffer, size_t size) override;
    ssize_t _read(uint8_t *buffer, uint16_t count) override;
    void _end() override;
    // Waiting for the backend could deadlock its caller. Never discard data.
    void _flush() override {}
    uint32_t _available() override;
    bool _discard_input() override;

    static ByteBuffer *allocate_buffer(uint16_t capacity);
    static void replace_buffer(ByteBuffer *&current, ByteBuffer *replacement);

    HAL_Semaphore _sem;
    ByteBuffer *_rx = nullptr;
    ByteBuffer *_tx = nullptr;
    std::atomic<uint32_t> _baudrate{0};
    uint8_t _instance = 0;
};

} // namespace AP_HAL

#endif // AP_HAL_VIRTUAL_UART_ENABLED
