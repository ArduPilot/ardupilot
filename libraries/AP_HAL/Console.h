
#ifndef __AP_HAL_CONSOLE_DRIVER_H__
#define __AP_HAL_CONSOLE_DRIVER_H__

#include "AP_HAL_Namespace.h"

class AP_HAL::ConsoleDriver : public AP_HAL::BetterStream {
public:
    virtual void init(void*implspecific) = 0;
    virtual void backend_open() = 0;
    virtual void backend_close() = 0;
    virtual size_t backend_read(uint8_t *data, size_t len) = 0;
    virtual size_t backend_write(const uint8_t *data, size_t len) = 0;
};

#endif // __AP_HAL_CONSOLE_DRIVER_H__
