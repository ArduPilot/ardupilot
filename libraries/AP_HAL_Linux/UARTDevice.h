#pragma once

#include "SerialDevice.h"
#include <AP_HAL/utility/Socket_native.h>

class UARTDevice: public SerialDevice {
public:
    UARTDevice(const char *device_path);
    virtual ~UARTDevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;
    virtual void set_flow_control(enum AP_HAL::UARTDriver::flow_control flow_control_setting) override;
    virtual AP_HAL::UARTDriver::flow_control get_flow_control(void) override
    {
        return _flow_control;
    }
    virtual void set_parity(int v) override;

private:
    void _disable_crlf();
    AP_HAL::UARTDriver::flow_control _flow_control = AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_DISABLE;

    int _fd = -1;
    const char *_device_path;
};
