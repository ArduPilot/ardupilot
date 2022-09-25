#pragma once

#include "SerialDevice.h"

class FIFODevice: public SerialDevice
{
public:
    FIFODevice(const char *rd_device, const char *wr_device);
    virtual ~FIFODevice();

    virtual bool open() override;
    virtual bool close() override;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) override;
    virtual ssize_t read(uint8_t *buf, uint16_t n) override;
    virtual void set_blocking(bool blocking) override;
    virtual void set_speed(uint32_t speed) override;
    virtual void set_flow_control(enum AP_HAL::UARTDriver::flow_control flow_control_setting) override;
    virtual AP_HAL::UARTDriver::flow_control get_flow_control(void) override
    {
        return AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_DISABLE;
    }
    virtual void set_parity(int v) override;

private:
    int _rd_fd = -1;
    int _wr_fd = -1;
    const char *_rd_device;
    const char *_wr_device;
};
