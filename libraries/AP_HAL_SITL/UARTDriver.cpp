// -*- Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

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
#include <sys/select.h>
#include <termios.h>

#include "UARTDriver.h"
#include "SITL_State.h"

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

bool SITLUARTDriver::_console;

/* UARTDriver method implementations */

void SITLUARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (txSpace != 0) {
        _txSpace = txSpace;
    }
    if (rxSpace != 0) {
        _rxSpace = rxSpace;
    }

    const char *path = _sitlState->_uart_path[_portNumber];

    if (strcmp(path, "GPS1") == 0) {
        /* gps */
        _connected = true;
        _fd = _sitlState->gps_pipe();
    } else if (strcmp(path, "GPS2") == 0) {
        /* 2nd gps */
        _connected = true;
        _fd = _sitlState->gps2_pipe();
    } else {
        /* parse type:args:flags string for path. 
           For example:
             tcp:5760:wait    // tcp listen on port 5760
             tcp:0:wait       // tcp listen on use base_port + 0
             tcpclient:192.168.2.15:5762
             uart:/dev/ttyUSB0:57600
         */
        char *saveptr = NULL;
        char *s = strdup(path);
        char *devtype = strtok_r(s, ":", &saveptr);
        char *args1 = strtok_r(NULL, ":", &saveptr);
        char *args2 = strtok_r(NULL, ":", &saveptr);
        if (strcmp(devtype, "tcp") == 0) {
            uint16_t port = atoi(args1);
            bool wait = (args2 && strcmp(args2, "wait") == 0);
            _tcp_start_connection(port, wait);
        } else if (strcmp(devtype, "tcpclient") == 0) {
            if (args2 == NULL) {
                AP_HAL::panic("Invalid tcp client path: %s", path);
            }
            uint16_t port = atoi(args2);
            _tcp_start_client(args1, port);
        } else if (strcmp(devtype, "uart") == 0) {
            uint32_t baudrate = args2? atoi(args2) : 57600;
            ::printf("UART connection %s:%u\n", args1, baudrate);
            _uart_start_connection(args1, baudrate);
        } else {
            AP_HAL::panic("Invalid device path: %s", path);
        }
        free(s);
    }
}

void SITLUARTDriver::end()
{
}

int16_t SITLUARTDriver::available(void)
{
    _check_connection();

    if (!_connected) {
        return 0;
    }

    if (_select_check(_fd)) {
#ifdef FIONREAD
        // use FIONREAD to get exact value if possible
        int num_ready;
        if (ioctl(_fd, FIONREAD, &num_ready) == 0) {
            if (num_ready > _rxSpace) {
                return _rxSpace;
            }
            if (num_ready == 0) {
                // EOF is reached
                fprintf(stdout, "Closed connection on serial port %u\n", _portNumber);
                close(_fd);
                _connected = false;
                return 0;
            }
            return num_ready;
        }
#endif
        return 1; // best we can do is say 1 byte available
    }
    return 0;
}



int16_t SITLUARTDriver::txspace(void)
{
    // always claim there is space available
    return _txSpace;
}

int16_t SITLUARTDriver::read(void)
{
    char c;

    if (available() <= 0) {
        return -1;
    }

    if (_portNumber == 1 || _portNumber == 4) {
        if (_sitlState->gps_read(_fd, &c, 1) == 1) {
            return (uint8_t)c;
        }
        return -1;
    }

    if (!_use_send_recv) {
        int fd = _console?0:_fd;
        if (::read(fd, &c, 1) != 1) {
            return -1;
        }
        return (uint8_t)c;
    }

    int n = recv(_fd, &c, 1, MSG_DONTWAIT);
    if (n <= 0) {
        // the socket has reached EOF
        close(_fd);
        _connected = false;
        fprintf(stdout, "Closed connection on serial port %u\n", _portNumber);
        fflush(stdout);
        return -1;
    }
    if (n == 1) {
        return (uint8_t)c;
    }
    return -1;
}

void SITLUARTDriver::flush(void)
{
}

size_t SITLUARTDriver::write(uint8_t c)
{
    int flags = 0;
    _check_connection();
    if (!_connected) {
        return 0;
    }
    if (_nonblocking_writes) {
        flags |= MSG_DONTWAIT;
    }
    if (!_use_send_recv) {
        return ::write(_fd, &c, 1);
    }
    return send(_fd, &c, 1, flags);
}

size_t SITLUARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}

/*
  start a TCP connection for the serial port. If wait_for_connection
  is true then block until a client connects
 */
void SITLUARTDriver::_tcp_start_connection(uint16_t port, bool wait_for_connection)
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
            sockaddr.sin_port = htons(_sitlState->base_port() + port);
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
                _sitlState->base_port() + _portNumber);
        fflush(stdout);
    }

    if (wait_for_connection) {
        fprintf(stdout, "Waiting for connection ....\n");
        fflush(stdout);
        _fd = accept(_listen_fd, NULL, NULL);
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
void SITLUARTDriver::_tcp_start_client(const char *address, uint16_t port)
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
    sockaddr.sin_port = port;
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
void SITLUARTDriver::_uart_start_connection(const char *path, uint32_t baudrate)
{
    struct termios t {};
    if (!_connected) {
        ::printf("Opening %s\n", path);
        _fd = ::open(path, O_RDWR | O_CLOEXEC);
    }

    if (_fd == -1) {
        AP_HAL::panic("Unable to open UART %s", path);
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
    tcsetattr(_fd, TCSANOW, &t);

    // set baudrate
    tcgetattr(_fd, &t);
    cfsetspeed(&t, baudrate);
    tcsetattr(_fd, TCSANOW, &t);

    _connected = true;
    _use_send_recv = false;
}

/*
  see if a new connection is coming in
 */
void SITLUARTDriver::_check_connection(void)
{
    if (_connected) {
        // we only want 1 connection at a time
        return;
    }
    if (_select_check(_listen_fd)) {
        _fd = accept(_listen_fd, NULL, NULL);
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
bool SITLUARTDriver::_select_check(int fd)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    // zero time means immediate return from select()
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    if (select(fd+1, &fds, NULL, NULL, &tv) == 1) {
        return true;
    }
    return false;
}

void SITLUARTDriver::_set_nonblocking(int fd)
{
    unsigned v = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, v | O_NONBLOCK);
}

#endif
