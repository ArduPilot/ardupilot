/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2

#include <limits.h>

#include "print_vprintf.h"

#include "Console.h"
using namespace AP_HAL_AVR;

// ConsoleDriver method implementations ///////////////////////////////////////
void AVRConsoleDriver::init(void* base_uart) {
    _base_uart = (AP_HAL::UARTDriver*) base_uart;
}


void AVRConsoleDriver::backend_open() {
}

void AVRConsoleDriver::backend_close() {
}

size_t AVRConsoleDriver::backend_read(uint8_t *data, size_t len) {
    return 0;
}

size_t AVRConsoleDriver::backend_write(const uint8_t *data, size_t len) {
    return 0;
}

// AVRConsoleDriver private method implementations ////////////////////////////

// BetterStream method implementations /////////////////////////////////////////
void AVRConsoleDriver::print_P(const prog_char_t *s) {
	_base_uart->print_P(s);
}

void AVRConsoleDriver::println_P(const prog_char_t *s) {
	_base_uart->println_P(s);
}

    
void AVRConsoleDriver::printf(const char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf(fmt, ap);
        va_end(ap);
}

void AVRConsoleDriver::_printf_P(const prog_char *fmt, ...) {
        va_list ap;
        va_start(ap, fmt);
        vprintf_P(fmt, ap);
        va_end(ap);
}

void AVRConsoleDriver::vprintf(const char *fmt, va_list ap){
    print_vprintf((AP_HAL::Print*)this, 0, fmt, ap); 
}

void AVRConsoleDriver::vprintf_P(const prog_char *fmt, va_list ap){
    print_vprintf((AP_HAL::Print*)this, 1, fmt, ap); 
}

// Stream method implementations /////////////////////////////////////////
int16_t AVRConsoleDriver::available(void) {
    return _base_uart->available();
}

int16_t AVRConsoleDriver::txspace(void) {
    return _base_uart->txspace();
}

int16_t AVRConsoleDriver::read() {
    return _base_uart->read();
}

// Print method implementations /////////////////////////////////////////

size_t AVRConsoleDriver::write(uint8_t c) {
    return _base_uart->write(c);
}

#endif // CONFIG_HAL_BOARD

