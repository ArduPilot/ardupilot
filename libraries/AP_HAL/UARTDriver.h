#pragma once

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

    // lock a port for exclusive use. Use a key of 0 to unlock
    virtual bool lock_port(uint32_t key) { return false; }

    // write to a locked port. If port is locked and key is not correct then 0 is returned
    // and write is discarded
    virtual size_t write_locked(const uint8_t *buffer, size_t size, uint32_t key) { return 0; }
    
    enum flow_control {
        FLOW_CONTROL_DISABLE=0, FLOW_CONTROL_ENABLE=1, FLOW_CONTROL_AUTO=2
    };
    virtual void set_flow_control(enum flow_control flow_control_setting) {};
    virtual enum flow_control get_flow_control(void) { return FLOW_CONTROL_DISABLE; }

    virtual void configure_parity(uint8_t v){};
    virtual void set_stop_bits(int n){};

    /* unbuffered writes bypass the ringbuffer and go straight to the
     * file descriptor
     */
    virtual bool set_unbuffered_writes(bool on){ return false; };

    /*
      wait for at least n bytes of incoming data, with timeout in
      milliseconds. Return true if n bytes are available, false if
      timeout
     */
    virtual bool wait_timeout(uint16_t n, uint32_t timeout_ms) { return false; }

    /*
     * Optional method to control the update of the motors. Derived classes
     * can implement it if their HAL layer requires.
     */
    virtual void _timer_tick(void) { }
};
