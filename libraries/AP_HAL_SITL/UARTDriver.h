#pragma once

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>
#include <stdarg.h>
#include "AP_HAL_SITL_Namespace.h"
#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/utility/RingBuffer.h>

class HALSITL::UARTDriver : public AP_HAL::UARTDriver {
public:
    friend class HALSITL::SITL_State;

    UARTDriver(const uint8_t portNumber, SITL_State *sitlState) {
        _portNumber = portNumber;
        _sitlState = sitlState;

        _fd = -1;
        _listen_fd = -1;
    }

    static UARTDriver *from(AP_HAL::UARTDriver *uart) {
        return static_cast<UARTDriver*>(uart);
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
    uint32_t available() override;
    uint32_t txspace() override;
    int16_t read() override;

    /* Implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

    // file descriptor, exposed so SITL_State::loop_hook() can use it
    int _fd;

    bool _unbuffered_writes;

    enum flow_control get_flow_control(void) { return FLOW_CONTROL_ENABLE; }

    void configure_parity(uint8_t v) override;
    void set_stop_bits(int n) override;
    bool set_unbuffered_writes(bool on) override;

    void _timer_tick(void);

    /*
      return timestamp estimate in microseconds for when the start of
      a nbytes packet arrived on the uart. This should be treated as a
      time constraint, not an exact time. It is guaranteed that the
      packet did not start being received after this time, but it
      could have been in a system buffer before the returned time.

      This takes account of the baudrate of the link. For transports
      that have no baudrate (such as USB) the time estimate may be
      less accurate.

      A return value of zero means the HAL does not support this API
     */
    uint64_t receive_time_constraint_us(uint16_t nbytes) override;
    
private:
    uint8_t _portNumber;
    bool _connected = false; // true if a client has connected
    bool _use_send_recv = false;
    int _listen_fd;  // socket we are listening on
    int _serial_port;
    static bool _console;
    bool _nonblocking_writes;
    ByteBuffer _readbuffer{16384};
    ByteBuffer _writebuffer{16384};

    const char *_uart_path;
    uint32_t _uart_baudrate;

    // IPv4 address of target for uartC
    const char *_tcp_client_addr;

    void _tcp_start_connection(uint16_t port, bool wait_for_connection);
    void _uart_start_connection(void);
    void _check_reconnect();
    void _tcp_start_client(const char *address, uint16_t port);
    void _check_connection(void);
    static bool _select_check(int );
    static void _set_nonblocking(int );
    bool set_speed(int speed);

    SITL_State *_sitlState;
    uint64_t _receive_timestamp;
};

#endif
