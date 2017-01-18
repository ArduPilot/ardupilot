/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//      Copyright (c) 2010 Michael Smith. All rights reserved.
//
#include <AP_HAL/AP_HAL.h>
#if defined(__CYGWIN__)

#include <limits.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <AP_Math/AP_Math.h>

#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <termios.h>

#include "../CORE_URUS_NAMESPACE.h"

#include "../CoreUrusUARTDriver.h"
#include "CoreUrusUARTDriver_Cygwin.h"

extern const NSCORE_URUS::CLCORE_URUS& _urus_core;

bool CLCoreUrusUARTDriver_Cygwin::_console;

/* CLCoreUrusUARTDriver_Cygwin method implementations */

void CLCoreUrusUARTDriver_Cygwin::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    /* parse type:args:flags string for path.
       For example:
         tcp:5760:wait    // tcp listen on port 5760
         tcp:0:wait       // tcp listen on use base_port + 0
         tcpclient:192.168.2.15:5762
         uart:/dev/ttyUSB0:57600
     */
    char *saveptr = nullptr;
    char *s = strdup(path);
    char *devtype = strtok_r(s, ":", &saveptr);
    char *args1 = strtok_r(nullptr, ":", &saveptr);
    char *args2 = strtok_r(nullptr, ":", &saveptr);
    if (strcmp(devtype, "tcp") == 0) {
        uint16_t port = atoi(args1);
        bool wait = (args2 && strcmp(args2, "wait") == 0);
        _tcp_start_connection(port, wait);
    } else if (strcmp(devtype, "tcpclient") == 0) {
        if (args2 == nullptr) {
            AP_HAL::panic("Invalid tcp client path: %s", path);
        }
        uint16_t port = atoi(args2);
        _tcp_start_client(args1, port);
    } else if (strcmp(devtype, "uart") == 0) {
        uint32_t baudrate = args2? atoi(args2) : baud;
        ::printf("UART connection %s:%u\n", args1, baudrate);
        _uart_path = strdup(args1);
        _uart_baudrate = baudrate;
        _uart_start_connection();
    } else {
        AP_HAL::panic("Invalid device path: %s", path);
    }
    free(s);

    _set_nonblocking(_fd);
}

void CLCoreUrusUARTDriver_Cygwin::end()
{
}

uint32_t CLCoreUrusUARTDriver_Cygwin::available(void)
{
    _check_connection();

    if (!_connected) {
        return 0;
    }

    return _readbuffer.available();
}

uint32_t CLCoreUrusUARTDriver_Cygwin::txspace(void)
{
    _check_connection();
    if (!_connected) {
        return 0;
    }
    return _writebuffer.space();
}

int16_t CLCoreUrusUARTDriver_Cygwin::read(void)
{
    if (available() <= 0) {
        return -1;
    }
    uint8_t c;
    _readbuffer.read(&c, 1);
    return c;
}

void CLCoreUrusUARTDriver_Cygwin::flush(void)
{
}

size_t CLCoreUrusUARTDriver_Cygwin::write(uint8_t c)
{
    if (txspace() <= 0) {
        return 0;
    }

    _writebuffer.write(&c, 1);

    /*  If we aren't using task manager (AP_Scheduler), then we
        process data byte each.
    */
    if (!_status_scheduling) {
        _timer_tick();
    }

    return 1;
}

size_t CLCoreUrusUARTDriver_Cygwin::write(const uint8_t *buffer, size_t size)
{
    if (txspace() <= (ssize_t)size) {
        size = txspace();
    }
    if (size <= 0) {
        return 0;
    }
    _writebuffer.write(buffer, size);
    return size;
}

    
/*
  start a TCP connection for the serial port. If wait_for_connection
  is true then block until a client connects
 */
void CLCoreUrusUARTDriver_Cygwin::_tcp_start_connection(uint16_t port, bool wait_for_connection)
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;

    if (_connected) {
        return;
    }

    _use_send_recv = true;

    if (_console) {
        // hack for console access
        _connected = true;
        _use_send_recv = false;
        _listen_fd = -1;
        _fd = 1;
        _writebuffer.set_size(5);
        _readbuffer.set_size(5);
        return;
    }

    if (_fd != -1) {
        close(_fd);
    }

    if (_listen_fd == -1) {
        memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
        sockaddr.sin_len = sizeof(sockaddr);
#endif
        if (port > 1000) {
            sockaddr.sin_port = htons(port);
        } else {
            sockaddr.sin_port = htons(_base_port + port);
        }
        sockaddr.sin_family = AF_INET;

        _listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (_listen_fd == -1) {
            fprintf(stderr, "socket failed - %s\n", strerror(errno));
            exit(1);
        }

        /* we want to be able to re-use ports quickly */
        setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        fprintf(stderr, "bind port %u for %u\n",
                (unsigned)ntohs(sockaddr.sin_port),
                (unsigned)_portNumber);

        ret = bind(_listen_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
        if (ret == -1) {
            fprintf(stderr, "bind failed on port %u - %s\n",
                    (unsigned)ntohs(sockaddr.sin_port),
                    strerror(errno));
            exit(1);
        }

        ret = listen(_listen_fd, 5);
        if (ret == -1) {
            fprintf(stderr, "listen failed - %s\n", strerror(errno));
            exit(1);
        }

        fprintf(stderr, "Serial port %u on TCP port %u\n", _portNumber,
                _base_port + _portNumber);
        fflush(stdout);
    }

    if (wait_for_connection) {
        fprintf(stdout, "Waiting for connection ....\n");
        fflush(stdout);
        _fd = accept(_listen_fd, nullptr, nullptr);
        if (_fd == -1) {
            fprintf(stderr, "accept() error - %s", strerror(errno));
            exit(1);
        }
        setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        _connected = true;
    }
}


/*
  start a TCP client connection for the serial port. 
 */
void CLCoreUrusUARTDriver_Cygwin::_tcp_start_client(const char *address, uint16_t port)
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;

    if (_connected) {
        return;
    }

    _use_send_recv = true;
    
    if (_fd != -1) {
        close(_fd);
    }

    memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);

    _fd = socket(AF_INET, SOCK_STREAM, 0);
    if (_fd == -1) {
        fprintf(stderr, "socket failed - %s\n", strerror(errno));
        exit(1);
    }

    /* we want to be able to re-use ports quickly */
    setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    ret = connect(_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "connect failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }

    setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    _connected = true;
}


/*
  start a UART connection for the serial port
 */
void CLCoreUrusUARTDriver_Cygwin::_uart_start_connection(void)
{
    struct termios t {};
    if (!_connected) {
        _fd = ::open(_uart_path, O_RDWR | O_CLOEXEC);
        if (_fd == -1) {
            return;
        }
        // use much smaller buffer sizes on real UARTs
        _writebuffer.set_size(1024);
        _readbuffer.set_size(512);
        ::printf("Opened %s\n", _uart_path);
    }

    if (_fd == -1) {
        AP_HAL::panic("Unable to open UART %s", _uart_path);
    }

    // set non-blocking
    int flags = fcntl(_fd, F_GETFL, 0);
    flags = flags | O_NONBLOCK;
    fcntl(_fd, F_SETFL, flags);

    // disable LF -> CR/LF
    tcgetattr(_fd, &t);
    t.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL | IXON | IXOFF);
    t.c_oflag &= ~(OPOST | ONLCR);
    t.c_lflag &= ~(ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
    t.c_cc[VMIN] = 0;
    if (_use_rtscts) {
        t.c_cflag |= CRTSCTS;
    }
    tcsetattr(_fd, TCSANOW, &t);

    // set baudrate
    tcgetattr(_fd, &t);
    cfsetspeed(&t, _uart_baudrate);
    tcsetattr(_fd, TCSANOW, &t);

    _connected = true;
    _use_send_recv = false;
}

/*
  see if a new connection is coming in
 */
void CLCoreUrusUARTDriver_Cygwin::_check_connection(void)
{
    if (_connected) {
        // we only want 1 connection at a time
        return;
    }
    if (_select_check(_listen_fd)) {
        _fd = accept(_listen_fd, nullptr, nullptr);
        if (_fd != -1) {
            int one = 1;
            _connected = true;
            setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
            fprintf(stdout, "New connection on serial port %u\n", _portNumber);
        }
    }
}

/*
  use select() to see if something is pending
 */
bool CLCoreUrusUARTDriver_Cygwin::_select_check(int fd)
{
    if (fd == -1) {
        return false;
    }
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // zero time means immediate return from select()
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    if (select(fd+1, &fds, nullptr, nullptr, &tv) == 1) {
        return true;
    }
    return false;
}

void CLCoreUrusUARTDriver_Cygwin::_set_nonblocking(int fd)
{
    unsigned rd_flags;
    unsigned wr_flags;
    int _wr_fd = fd;
    int _rd_fd = STDIN_FILENO;

    rd_flags  = fcntl(_rd_fd, F_GETFL, 0);
    wr_flags  = fcntl(_wr_fd, F_GETFL, 0);

    rd_flags = rd_flags | O_NONBLOCK;
    wr_flags = wr_flags | O_NONBLOCK;

    if (fcntl(_rd_fd, F_SETFL, rd_flags) < 0) {
        ::printf("Failed to set Console nonblocking %s\n", strerror(errno));
    }

    if (fcntl(_wr_fd, F_SETFL, wr_flags) < 0) {
        ::printf("Failed to set Console nonblocking %s\n",strerror(errno));
    }
}

void CLCoreUrusUARTDriver_Cygwin::_check_reconnect(void)
{
    if (!_uart_path) {
        return;
    }
    _uart_start_connection();
}

void CLCoreUrusUARTDriver_Cygwin::_timer_tick(void)
{
    if (!_connected) {
        _check_reconnect();
        return;
    }
    uint32_t navail;
    ssize_t nwritten;

    const uint8_t *readptr = _writebuffer.readptr(navail);

    if (readptr && navail > 0) {
        if (!_use_send_recv) {
            nwritten = ::write(_fd, readptr, navail);
            if (nwritten == -1 && errno != EAGAIN && _uart_path) {
                close(_fd);
                _fd = -1;
                _connected = false;
            }
        } else {
            nwritten = send(_fd, readptr, navail, MSG_DONTWAIT);
        }
        if (nwritten > 0) {
            _writebuffer.advance(nwritten);
        }
    }

    uint32_t space = _readbuffer.space();
    if (space == 0) {
        return;
    }
    
    char buf[space];
    ssize_t nread = 0;
    if (!_use_send_recv) {
        int fd = _console?0:_fd;
        nread = ::read(fd, buf, space);
        if (nread == -1 && errno != EAGAIN && _uart_path) {
            close(_fd);
            _fd = -1;
            _connected = false;
        }
    } else {
        if (_select_check(_fd)) {
            nread = recv(_fd, buf, space, MSG_DONTWAIT);
            if (nread <= 0) {
                // the socket has reached EOF
                close(_fd);
                _connected = false;
                fprintf(stdout, "Closed connection on serial port %u\n", _portNumber);
                fflush(stdout);
                return;
            }
        } else {
            nread = 0;
        }
    }
    if (nread > 0) {
        _readbuffer.write((uint8_t *)buf, nread);
    }
}

#endif // __CYGWIN__

