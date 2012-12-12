/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_AVR_SITL_CONSOLE_DRIVER_H__
#define __AP_HAL_AVR_SITL_CONSOLE_DRIVER_H__

#include <stdlib.h>

#include <AP_HAL.h>
#include "AP_HAL_AVR_SITL_Namespace.h"

class SITLConsoleDriver : public AP_HAL::ConsoleDriver {
public:
	SITLConsoleDriver();
    void init(void* baseuartdriver);
    void backend_open();
    void backend_close();
    size_t backend_read(uint8_t *data, size_t len);
    size_t backend_write(const uint8_t *data, size_t len);

    /* Implementations of BetterStream virtual methods */
    void print_P(const prog_char_t *s);
    void println_P(const prog_char_t *s);
    void printf(const char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));
    void _printf_P(const prog_char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));

    /* Implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();
    int16_t peek();

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c);

private:
    AP_HAL::UARTDriver* _base_uart;
};

#endif // __AP_HAL_AVR_CONSOLE_DRIVER_H__

