
#ifndef __AP_HAL_UART_DRIVER_H__
#define __AP_HAL_UART_DRIVER_H__

#include <stdint.h>

#include "AP_HAL_Namespace.h"
#include "utility/BetterStream.h"

/* Pure virtual UARTDriver class */
class AP_HAL::UARTDriver : public AP_HAL::BetterStream {
public:
    UARTDriver() {}
    virtual void begin(uint32_t baud) = 0;
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
    virtual void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) = 0;
    virtual void end() = 0;
    virtual void flush() = 0;
    virtual bool is_initialized() = 0;
    virtual void set_blocking_writes(bool blocking) = 0;
    virtual bool tx_pending() = 0;

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

#endif // __AP_HAL_UART_DRIVER_H__

