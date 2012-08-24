
#ifndef __AP_HAL_UART_DRIVER_H__
#define __AP_HAL_UART_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"

/* Pure virtual UARTDriver class */
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    virtual void begin(long baud) = 0;
	/// Extended port open method
	///
	/// Allows for both opening with specified buffer sizes, and re-opening
	/// to adjust a subset of the port's settings.
	///
	/// @note	Buffer sizes greater than ::_max_buffer_size will be rounded
	///			down.
	///
	/// @param	baud		Selects the speed that the port will be
	///						configured to.  If zero, the port speed is left
	///						unchanged.
	/// @param rxSpace		Sets the receive buffer size for the port.  If zero
	///						then the buffer size is left unchanged if the port
	///						is open, or set to ::_default_rx_buffer_size if it is
	///						currently closed.
	/// @param txSpace		Sets the transmit buffer size for the port.  If zero
	///						then the buffer size is left unchanged if the port
	///						is open, or set to ::_default_tx_buffer_size if it
	///						is currently closed.
	///
    virtual void begin(long baud,
                    unsigned int rxSpace,
                    unsigned int txSpace) = 0;
    virtual void end() = 0;
    virtual void flush() = 0;
    virtual bool is_initialized() = 0;
    virtual void set_blocking_writes(bool blocking) = 0;
    virtual bool tx_pending() = 0;
};

/* Concrete EmptyUARTDriver class provided for convenience */
class AP_HAL::EmptyUARTDriver : public AP_HAL::UARTDriver {
public:
    EmptyUARTDriver() {}
    /* Empty implementations of UARTDriver virtual methods */
    void begin(long b) {}
    void begin(long b, unsigned int rxS, unsigned int txS) {}
    void end() {}
    void flush() {}
    bool is_initialized() { return false; }
    void set_blocking_writes(bool blocking) {}
    bool tx_pending() { return false; }

    /* Empty implementations of BetterStream virtual methods */
    void print_P(const prog_char_t *pstr) {}
    void println_P(const prog_char_t *pstr) {}
    void printf(const char *pstr, ...) {}
    void _printf_P(const prog_char *pstr, ...) {}
    int txspace() { return 1; }

    /* Empty implementations of Stream virtual methods */
    int available() { return 0; }
    int read() { return -1; }
    int peek() { return -1; }

    /* Empty implementations of Print virtual methods */
    size_t write(uint8_t c) { return 0; }
};


#endif // __AP_HAL_UART_DRIVER_H__

