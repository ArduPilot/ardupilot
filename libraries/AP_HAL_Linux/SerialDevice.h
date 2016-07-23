#pragma once

#include <stdint.h>
#include <stdlib.h>

#include "AP_HAL_Linux.h"

class SerialDevice {
public: 
    virtual ~SerialDevice() {}

    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual ssize_t write(const uint8_t *buf, uint16_t n) = 0;
    virtual ssize_t read(uint8_t *buf, uint16_t n) = 0;
    virtual void set_blocking(bool blocking) = 0;
    virtual void set_speed(uint32_t speed) = 0;
    virtual AP_HAL::UARTDriver::flow_control get_flow_control(void) { return AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE; }
    virtual void set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting)
    {
        /* most devices simply igmore this setting */
    };
};
