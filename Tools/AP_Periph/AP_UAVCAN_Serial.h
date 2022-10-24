#pragma once
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#if AP_SERIAL_EXTENSION_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>
#include <canard.h>
#include <dronecan_msgs.h>

class AP_UAVCAN_Serial : public AP_HAL::UARTDriver {
    friend class AP_Periph;
public:

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
    bool handle_tunnel_broadcast(CanardInstance &ins, CanardRxTransfer &transfer, const uavcan_tunnel_Broadcast &msg);

    int8_t get_target_node_id() { return _target_node_id; }
    void set_target_node_id(int8_t target_node_id) { _target_node_id = target_node_id; }

    uint8_t get_channel_id() { return _channel_id; }
    uint32_t get_usb_baud() const override { return baudrate; }
    uint32_t set_usb_baud(uint32_t baud) { return baudrate = baud; }
private:
    int _channel_id;
    uint8_t _protocol;

    int8_t _target_node_id = -1;
    bool _initialized = false;
    bool _connected = false; // true if a client has connected

    ByteBuffer _readbuf{0};
    ByteBuffer _writebuf{0};
    void send_data();
    uint32_t baudrate;
};
#endif
