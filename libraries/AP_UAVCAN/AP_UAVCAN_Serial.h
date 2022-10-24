#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS && AP_SERIAL_EXTENSION_ENABLED
#include <uavcan/uavcan.hpp>
#include <AP_Common/Bitmask.h>
#include <StorageManager/StorageManager.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_UAVCAN;
class BroadcastCb;

class AP_UAVCAN_Serial : public AP_HAL::UARTDriver {
    friend class AP_SerialManager;
public:

    // set channel id
    AP_UAVCAN_Serial(uint8_t channel_id) :
        _channel_id(channel_id)
    {}

    /* Implementations of UARTDriver virtual methods */
    void begin(uint32_t b) override {
        begin(b, 0, 0);
    }
    void begin(uint32_t b, uint16_t rxS, uint16_t txS) override;

    void end() override;

    void flush() override {}

    void set_blocking_writes(bool blocking) override {}

    bool is_initialized() override {
        return _initialized;
    }

    bool tx_pending() override {
        return _writebuf.available() > 0;
    }

    /* Implementations of Stream virtual methods */
    uint32_t available() override {
        return _readbuf.available();
    }
    uint32_t txspace() override {
        return _writebuf.space();
    }
    int16_t read() override;
    ssize_t read(uint8_t *buffer, uint16_t count) override;

    bool discard_input() override {
        _readbuf.clear();
        return true;
    }

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    void handleBroadcast(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BroadcastCb& resp);

    static void trampoline_handleBroadcast(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BroadcastCb& resp);

private:

    void uavcan_loop(AP_SerialManager::SerialProtocol protocol_id);

    int _channel_id;

    bool _initialized = false;
    bool _connected = false; // true if a client has connected

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};

    AP_UAVCAN *_ap_uavcan;
    uint8_t _driver_index;
    uint32_t _baudrate;
    uint32_t _last_baudrate;
    uint32_t _last_serial_config_ms;
};
#endif
