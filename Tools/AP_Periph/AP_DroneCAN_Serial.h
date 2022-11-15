#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#if AP_SERIAL_EXTENSION_ENABLED || HAL_ENABLE_SERIAL_TUNNEL
#include <AP_SerialManager/AP_SerialManager.h>
#include <canard.h>
#include <dronecan_msgs.h>

class AP_DroneCAN_Serial : public AP_HAL::UARTDriver {
public:

    AP_DroneCAN_Serial(uint8_t channel_id) :
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
    bool handle_tunnel_broadcast(CanardInstance &ins, CanardRxTransfer &transfer, const uavcan_tunnel_Broadcast &msg);

    uint8_t get_channel_id() { return _channel_id; }
    void set_channel_id(uint8_t channel_id) {  _channel_id = channel_id; }
    uint32_t get_passthrough_baud() const override { return baudrate; }
    uint32_t set_passthrough_baud(uint32_t baud) { return baudrate = baud; }
    static AP_SerialManager::SerialProtocol tunnel_protocol_to_ap_protocol(uint8_t tunnel_protocol);

    void set_idle_time_us(const uint32_t idle_time_us) {
        _idle_time_us = idle_time_us;
    }

    void send_data();
private:
    int _channel_id;
    uint32_t _idle_time_us;
    uint8_t _protocol;
    bool _initialized = false;
    bool _connected = false; // true if a client has connected

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    uint32_t baudrate;
    uint64_t last_write_us;
    HAL_Semaphore send_sem;
};
#endif
