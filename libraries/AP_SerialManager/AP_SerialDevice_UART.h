#pragma once

#include "AP_SerialDevice.h"

class AP_SerialDevice_UART : public AP_SerialDevice
{

public:

    struct UARTState {
        AP_Enum<Protocol> protocol;
        AP_Int32 baud;
        AP_Int16 options;
    };

    AP_SerialDevice_UART(struct UARTState &_state,
                         uint8_t _instance,
                         AP_HAL::UARTDriver &_uart) :
        AP_SerialDevice(_state.protocol, _instance),
        state{_state},
        uart{_uart},
        baud{map_baudrate(_state.baud)}
        { }

    void begin() override {
        begin(baud);
    }
    void begin_locked(uint32_t _baud, uint32_t key) override {
        uart.begin_locked(_baud, key);
    }

    void begin(uint32_t _baud) {
        // begin with default buffers (0 is magic):
        begin(_baud, bufsize_rx, bufsize_tx);
    }
    void end() override {
        uart.end();
    }

    AP_SerialDevice_UART *get_serialdevice_uart() override { return this; }

    bool should_forward_mavlink_telemetry() const override;

    static uint32_t map_baudrate(int32_t rate);

    uint32_t configured_baud() const {
        return map_baudrate(state.baud);
    }
    // returns baud we will connect at:
    uint32_t get_baud() const {
        return map_baudrate(baud);
    }

    AP_HAL::UARTDriver &get_uart() {
        // for those that need to get down-and-dirty with the uart
        // e.g. autobauding in GPS.  Will cause an internal error if
        // type is not Type::UART
        return uart;
    }

    size_t write(uint8_t c) override {
        return uart.write(c);
    }
    size_t write(const uint8_t *buffer, size_t size) override {
        return uart.write(buffer, size);
    }
    uint32_t available() override {
        return uart.available();
    }
    ssize_t read(uint8_t *buffer, uint16_t count) override {
        return uart.read(buffer, count);
    }
    int16_t read() override {
        return uart.read();
    }
    bool discard_input() override { // discard all bytes available for reading
        return uart.discard_input();
    }
    uint32_t txspace() override {
        return uart.txspace();
    }

    bool tx_pending() const override {
        return uart.tx_pending();
    }

    uint32_t bw_in_kilobytes_per_second() const override {
        return uart.bw_in_kilobytes_per_second();
    }

    uint64_t receive_time_constraint_us(uint16_t nbytes) override {
        return uart.receive_time_constraint_us(nbytes);
    }

    enum AP_HAL::UARTDriver::flow_control get_flow_control(void) {
        return uart.get_flow_control();
    }
    void set_flow_control(enum AP_HAL::UARTDriver::flow_control flow_control_setting) {
        return uart.set_flow_control(flow_control_setting);
    }

    bool set_unbuffered_writes(bool on) {
        return uart.set_unbuffered_writes(on);
    }
    void set_blocking_writes(bool blocking) {
        uart.set_blocking_writes(blocking);
    }

    void set_baud(uint32_t _baud) { baud = _baud; }

    void set_bufsize_rx(uint32_t size) { bufsize_rx = size; }
    void set_bufsize_tx(uint32_t size) { bufsize_tx = size; }

    void set_options() override;

private:

    void begin(uint32_t _baud, uint16_t rxSpace, uint16_t txSpace) {
        uart.begin(_baud, rxSpace, txSpace);
    }

    AP_HAL::UARTDriver &uart;
    UARTState &state;

    // baud to use to connect; initially that found in the configured
    // UARTState but can be overridden by a protocol driver
    uint32_t baud;
    uint32_t bufsize_rx;
    uint32_t bufsize_tx;

};
