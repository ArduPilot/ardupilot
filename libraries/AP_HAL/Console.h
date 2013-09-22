
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


    /* Implementations of BetterStream virtual methods. These are
     * provided by AP_HAL to ensure consistency between ports to
     * different boards
     */
    void print_P(const prog_char_t *s);
    void println_P(const prog_char_t *s);
    void printf(const char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));
    void _printf_P(const prog_char *s, ...)
            __attribute__ ((format(__printf__, 2, 3)));

    void vprintf(const char *s, va_list ap);
    void vprintf_P(const prog_char *s, va_list ap);
};

#endif // __AP_HAL_CONSOLE_DRIVER_H__
