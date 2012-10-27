
#include <limits.h>
#include <stdarg.h>

#include <AP_HAL.h>
#include "Console.h"
using namespace AP_HAL_PX4;

PX4ConsoleDriver::PX4ConsoleDriver()
{}

// ConsoleDriver method implementations ///////////////////////////////////////
void PX4ConsoleDriver::init(void* base_uart) {
}


void PX4ConsoleDriver::backend_open() {
}

void PX4ConsoleDriver::backend_close() {
}

int PX4ConsoleDriver::backend_read(uint8_t *data, int len) {
    return 0;
}

int PX4ConsoleDriver::backend_write(const uint8_t *data, int len) {
    return 0;
}

// BetterStream method implementations /////////////////////////////////////////
void PX4ConsoleDriver::print_P(const prog_char_t *s) {
        char    c;
        while ('\0' != (c = *s++))
                write(c);
}

void PX4ConsoleDriver::println_P(const prog_char_t *s) {
        print_P(s);
        println();
}

void PX4ConsoleDriver::printf(const char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
//        vprintf((AP_HAL::Print*)this, 0, fmt, ap);
        va_end(ap);
}

void PX4ConsoleDriver::_printf_P(const prog_char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
//        vprintf((AP_HAL::Print*)this, 1, fmt, ap);
        va_end(ap);
}

// Stream method implementations /////////////////////////////////////////
int PX4ConsoleDriver::available(void) {
    return 0;
}

int PX4ConsoleDriver::txspace(void) {
    return 0;
}

int PX4ConsoleDriver::read() {
    return -1;
}

int PX4ConsoleDriver::peek() {
    return -1;
}

// Print method implementations /////////////////////////////////////////

size_t PX4ConsoleDriver::write(uint8_t c) {
    return 0;
}

