/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2

#include <limits.h>
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

