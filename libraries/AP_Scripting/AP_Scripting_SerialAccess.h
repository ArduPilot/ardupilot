#pragma once

#include "AP_Scripting_config.h"
#include "AP_Scripting.h"

#include <AP_HAL/UARTDriver.h>

class AP_Scripting_SerialAccess {
public:
    /* Do not allow copies */
    CLASS_NO_COPY(AP_Scripting_SerialAccess);

    AP_Scripting_SerialAccess() {}

    void begin(uint32_t baud);
    void begin();

    void configure_parity(uint8_t parity);
    void set_stop_bits(uint8_t stop_bits);
    void set_unbuffered_writes(bool on);

    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    int16_t read(void);
    ssize_t read(uint8_t *buffer, uint16_t count);

    uint32_t available(void);

    void set_flow_control(enum AP_HAL::UARTDriver::flow_control fcs);

    AP_HAL::UARTDriver *stream;
#if AP_SCRIPTING_ENABLED
#if AP_SCRIPTING_SERIALDEVICE_ENABLED
    bool is_device_port;
#endif
#endif
};
