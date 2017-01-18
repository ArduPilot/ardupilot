#pragma once

#include <AP_HAL/AP_HAL.h>
#if defined(__CYGWIN__)

#include <stdint.h>
#include <stdarg.h>

#include "../CORE_URUS_NAMESPACE.h"
#include "../CoreUrusUARTDriver.h"

#include <AP_HAL/utility/Socket.h>
#include <AP_HAL/utility/RingBuffer.h>

class CLCoreUrusUARTDriver_Cygwin : public NSCORE_URUS::CLCoreUrusUARTDriver {
public:

    CLCoreUrusUARTDriver_Cygwin(const uint8_t portNumber, const bool console) {
        _portNumber = portNumber;
        _console = console;
        _fd = -1;
        _listen_fd = -1;
    }

    static CLCoreUrusUARTDriver *from(AP_HAL::UARTDriver *uart) {
        return static_cast<CLCoreUrusUARTDriver_Cygwin*>(uart);
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

    enum flow_control get_flow_control(void) { return FLOW_CONTROL_ENABLE; }

    void _timer_tick(void) override;

    /* Set scheduling status. */
    void set_scheduling_status(bool status) {
        _status_scheduling = status;
    }

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
    bool _use_rtscts;

    /* default configuration for uart driver */
    const char *path = "tcp:0:wait";
    uint16_t _base_port = 5760;

    /*  Is scheduling using and active?
        It's activated if we are using the AP_Scheduler.
    */
    bool _status_scheduling = false;
};

#endif // __CYGWIN__
