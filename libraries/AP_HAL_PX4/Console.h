
#ifndef __AP_HAL_PX4_CONSOLE_DRIVER_H__
#define __AP_HAL_PX4_CONSOLE_DRIVER_H__

#include <AP_HAL.h>
#include <AP_HAL_PX4_Namespace.h>

class AP_HAL_PX4::PX4ConsoleDriver : public AP_HAL::ConsoleDriver {
    PX4ConsoleDriver();
    void init(void*);
    void backend_open();
    void backend_close();
    int backend_read(uint8_t *data, int len);
    int backend_write(const uint8_t *data, int len);

    /* Implementations of BetterStream virtual methods */
    void print_P(const prog_char_t *s);
    void println_P(const prog_char_t *s);
    void printf(const char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));
    void _printf_P(const prog_char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));

    /* Implementations of Stream virtual methods */
    int available();
    int txspace();
    int read();
    int peek();

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c);

};

#endif // __AP_HAL_PX4_CONSOLE_DRIVER_H__
