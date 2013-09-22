
#ifndef __AP_HAL_AVR_CONSOLE_DRIVER_H__
#define __AP_HAL_AVR_CONSOLE_DRIVER_H__

#include <stdlib.h>

#include <AP_HAL.h>
#include "AP_HAL_AVR_Namespace.h"

class AP_HAL_AVR::AVRConsoleDriver : public AP_HAL::ConsoleDriver {
public:
    void init(void* baseuartdriver);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    /* Implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c);

private:
    AP_HAL::UARTDriver* _base_uart;
};

#endif // __AP_HAL_AVR_CONSOLE_DRIVER_H__

