
#include <limits.h>

#include "vprintf.h"

#include <AP_HAL.h>
#include "Console.h"
using namespace AP_HAL_AVR;

AVRConsoleDriver::AVRConsoleDriver() :
    _user_backend(false)
{}

// ConsoleDriver method implementations ///////////////////////////////////////
//
void AVRConsoleDriver::init(void* base_uart) {
    _base_uart = (AP_HAL::UARTDriver*) base_uart;
}


void AVRConsoleDriver::backend_open() {
    _user_backend = true;
}

void AVRConsoleDriver::backend_close() {
    _user_backend = false;
}

int AVRConsoleDriver::backend_read(uint8_t *data, int len) {
    return 0;
}

int AVRConsoleDriver::backend_write(const uint8_t *data, int len) {
    return 0;
}
// Print method implementations /////////////////////////////////////////

size_t AVRConsoleDriver::write(uint8_t c) {
    if (_user_backend) {
        return 1;
    } else {
        return _base_uart->write(c);
    }
}

// BetterStream method implementations /////////////////////////////////////////
void AVRConsoleDriver::print_P(const prog_char_t *s) {
        char    c;
        while ('\0' != (c = pgm_read_byte((const prog_char *)s++)))
                write(c);
}

void AVRConsoleDriver::println_P(const prog_char_t *s) {
        print_P(s);
        println();
}

void AVRConsoleDriver::printf(const char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf((AP_HAL::Print*)this, 0, fmt, ap);
        va_end(ap);
}

void AVRConsoleDriver::_printf_P(const prog_char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf((AP_HAL::Print*)this, 1, fmt, ap);
        va_end(ap);
}

// Stream method implementations /////////////////////////////////////////
int AVRConsoleDriver::available(void) {
    if (_user_backend) {
        return INT_MAX;
    } else {
        return _base_uart->available();
    }
}

int AVRConsoleDriver::txspace(void) {
    if (_user_backend) {
        return INT_MAX;
    } else {
        return _base_uart->txspace();
    }
}

int AVRConsoleDriver::read() {
    if (_user_backend) {
        return -1;
    } else {
        return _base_uart->read();
    }
}

int AVRConsoleDriver::peek() {
    if (_user_backend) {
        return -1;
    } else {
        return _base_uart->peek();
    }
}

