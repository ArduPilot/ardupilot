
#include "UARTDriver.h"

using namespace Empty;

EmptyUARTDriver::EmptyUARTDriver() {}

void EmptyUARTDriver::begin(uint32_t b) {}
void EmptyUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {}
void EmptyUARTDriver::end() {}
void EmptyUARTDriver::flush() {}
bool EmptyUARTDriver::is_initialized() { return false; }
void EmptyUARTDriver::set_blocking_writes(bool blocking) {}
bool EmptyUARTDriver::tx_pending() { return false; }

/* Empty implementations of BetterStream virtual methods */
void EmptyUARTDriver::print_P(const prog_char_t *pstr) {}
void EmptyUARTDriver::println_P(const prog_char_t *pstr) {}
void EmptyUARTDriver::printf(const char *pstr, ...) {}
void EmptyUARTDriver::_printf_P(const prog_char *pstr, ...) {}

/* Empty implementations of Stream virtual methods */
int16_t EmptyUARTDriver::available() { return 0; }
int16_t EmptyUARTDriver::txspace() { return 1; }
int16_t EmptyUARTDriver::read() { return -1; }
int16_t EmptyUARTDriver::peek() { return -1; }

/* Empty implementations of Print virtual methods */
size_t EmptyUARTDriver::write(uint8_t c) { return 0; }

