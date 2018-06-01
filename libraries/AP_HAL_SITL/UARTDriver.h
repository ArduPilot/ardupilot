/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_HAL_SITL_UART_DRIVER_H__
#define __AP_HAL_SITL_UART_DRIVER_H__
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>
#include <stdarg.h>
#include "AP_HAL_SITL_Namespace.h"

class HALSITL::SITLUARTDriver : public AP_HAL::UARTDriver {
public:
    friend class HALSITL::SITL_State;

    SITLUARTDriver(const uint8_t portNumber, SITL_State *sitlState) {
        _portNumber = portNumber;
        _rxSpace = _default_rx_buffer_size;
        _txSpace = _default_tx_buffer_size;
        _sitlState = sitlState;

        _fd = -1;
        _listen_fd = -1;
    }

    /* Implementations of UARTDriver virtual methods */
    void begin(uint32_t b) {
        begin(b, 0, 0);
    }
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized() {
        return true;
    }

    void set_blocking_writes(bool blocking)
    {
        _nonblocking_writes = !blocking;
    }

    bool tx_pending() {
        return false;
    }

    /* Implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    // file descriptor, exposed so SITL_State::loop_hook() can use it
    int _fd;

private:
    uint8_t _portNumber;
    bool _connected; // true if a client has connected
    int _listen_fd;  // socket we are listening on
    int _serial_port;
    static bool _console;
    bool _nonblocking_writes;
    uint16_t _rxSpace;
    uint16_t _txSpace;

    // IPv4 address of target for uartC
    const char *_tcp_client_addr;

    void _tcp_start_connection(bool wait_for_connection);
    void _tcp_start_client(const char *address);
    void _check_connection(void);
    static bool _select_check(int );
    static void _set_nonblocking(int );

    /// default receive buffer size
    static const uint16_t _default_rx_buffer_size = 128;

    /// default transmit buffer size
    static const uint16_t _default_tx_buffer_size = 16;

    /// maxium tx/rx buffer size
    /// @note if we could bring the max size down to 256, the mask and head/tail
    ///       pointers in the buffer could become uint8_t.
    ///
    static const uint16_t _max_buffer_size = 512;

    SITL_State *_sitlState;

};

#endif
#endif // __AP_HAL_SITL_UART_DRIVER_H__

