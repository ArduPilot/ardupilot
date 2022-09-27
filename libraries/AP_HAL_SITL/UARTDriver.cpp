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
#include <sys/time.h>

#include "UARTDriver.h"
#include "SITL_State.h"
#if HAL_GCS_ENABLED
#include <AP_HAL/utility/packetise.h>
#endif

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

bool UARTDriver::_console;

/* UARTDriver method implementations */

void UARTDriver::begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace)
{
    if (_portNumber >= ARRAY_SIZE(_sitlState->_uart_path)) {
        AP_HAL::panic("port number out of range; you may need to extend _sitlState->_uart_path");
    }

    const char *path = _sitlState->_uart_path[_portNumber];

    if (baud != 0) {
        _uart_baudrate = baud;
    }
    
    if (strcmp(path, "GPS1") == 0) {
        /* gps */
        _connected = true;
        _sim_serial_device = _sitlState->create_serial_sim("gps:1", "");
    } else if (strcmp(path, "GPS2") == 0) {
        /* 2nd gps */
        _connected = true;
        _sim_serial_device = _sitlState->create_serial_sim("gps:2", "");
    } else {
        /* parse type:args:flags string for path. 
           For example:
             tcp:5760:wait    // tcp listen on port 5760
             tcp:0:wait       // tcp listen on use base_port + 0
             tcpclient:192.168.2.15:5762
             udpclient:127.0.0.1
             udpclient:127.0.0.1:14550
             mcast:
             mcast:239.255.145.50:14550
             uart:/dev/ttyUSB0:57600
             sim:ParticleSensor_SDS021:
         */
        char *saveptr = nullptr;
        char *s = strdup(path);
        char *devtype = strtok_r(s, ":", &saveptr);
        char *args1 = strtok_r(nullptr, ":", &saveptr);
        char *args2 = strtok_r(nullptr, ":", &saveptr);
#if !defined(HAL_BUILD_AP_PERIPH)
        if (_portNumber == 2 && AP::sitl()->adsb_plane_count >= 0) {
            // this is ordinarily port 5762.  The ADSB simulation assumed
            // this port, so if enabled we assume we'll be doing ADSB...
            // add sanity check here that we're doing mavlink on this port?
            ::printf("SIM-ADSB connection on port %u\n", _portNumber);
            _connected = true;
            _sim_serial_device = _sitlState->create_serial_sim("adsb", nullptr);
        } else
#endif
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
        } else if (strcmp(devtype, "fifo") == 0) {
            if(strcmp(args1, "gps") == 0) {
                UNUSED_RESULT(asprintf(&args1, "/tmp/gps_fifo%d", (int)_sitlState->get_instance()));
            }
            ::printf("Reading FIFO file @ %s\n", args1);
            _fd = ::open(args1, O_RDONLY | O_NONBLOCK);
            if (_fd >= 0) {
                _connected = true;
            } else {
                ::printf("Failed Reading FIFO file @ %s\n", args1);       
            }
        } else if (strcmp(devtype, "sim") == 0) {
            if (!_connected) {
                ::printf("SIM connection %s:%s on port %u\n", args1, args2, _portNumber);
                _connected = true;
                _sim_serial_device = _sitlState->create_serial_sim(args1, args2);
            }
        } else if (strcmp(devtype, "udpclient") == 0) {
            // udp client connection
            const char *ip = args1;
            uint16_t port = args2?atoi(args2):14550;
            if (!_connected) {
                ::printf("UDP connection %s:%u\n", ip, port);
                _udp_start_client(ip, port);
            }
        } else if (strcmp(devtype, "mcast") == 0) {
            // udp multicast connection
            const char *ip = args1 && *args1?args1:mcast_ip_default;
            uint16_t port = args2?atoi(args2):mcast_port_default;
            if (!_connected) {
                ::printf("UDP multicast connection %s:%u\n", ip, port);
                _udp_start_multicast(ip, port);
            }
        } else if (strcmp(devtype,"none") == 0) {
            // skipping port
            ::printf("Skipping port %s\n", args1);
        } else {
            AP_HAL::panic("Invalid device path: %s", path);
        }
        free(s);
    }

    if (_sim_serial_device != nullptr) {
        _sim_serial_device->set_autopilot_baud(baud);
    }

    if (hal.console != this) { // don't clear USB buffers (allows early startup messages to escape)
        _readbuffer.clear();
        _writebuffer.clear();
    }

    _set_nonblocking(_fd);
}

void UARTDriver::end()
{
}

uint32_t UARTDriver::available(void)
{
    _check_connection();

    if (!_connected) {
        return 0;
    }

    return _readbuffer.available();
}

uint32_t UARTDriver::txspace(void)
{
    _check_connection();
    if (!_connected) {
        return 0;
    }
    return _writebuffer.space();
}

int16_t UARTDriver::read(void)
{
    uint8_t c;
    if (read(&c, 1) == 0) {
        return -1;
    }
    return c;
}

ssize_t UARTDriver::read(uint8_t *buffer, uint16_t count)
{
    return _readbuffer.read(buffer, count);
}

bool UARTDriver::discard_input(void)
{
    _readbuffer.clear();
    return true;
}

void UARTDriver::flush(void)
{
    // flush the write buffer - but don't fail and don't
    // infinitely-loop.  This is not a good definition of "flush", but
    // it was judged that we had to return from this function even if
    // we hadn't actually done our job.
    uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 1000) {
        if (_writebuffer.available() == 0) {
            break;
        }
        _timer_tick();
    }

    // ensure that the outbound TCP queue is also empty...
    start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 1000) {
        if (((HALSITL::UARTDriver*)hal.serial(0))->get_system_outqueue_length() == 0) {
            break;
        }
        usleep(1000);
    }
}

// size_t UARTDriver::write(uint8_t c)
// {
//     if (txspace() <= 0) {
//         return 0;
//     }
//     _writebuffer.write(&c, 1);
//     return 1;
// }

size_t UARTDriver::write(uint8_t c)
{
    return write(&c, 1);
}
size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    if (txspace() <= size) {
        size = txspace();
    }
    if (size <= 0) {
        return 0;
    }
    if (_unbuffered_writes) {
        const ssize_t nwritten = ::write(_fd, buffer, size);
        if (nwritten == -1 && errno != EAGAIN && _uart_path) {
            close(_fd);
            _fd = -1;
            _connected = false;
        }
        // these have no effect
        tcdrain(_fd);
    } else {
        _writebuffer.write(buffer, size);
    }
    return size;
}

    
/*
  start a TCP connection for the serial port. If wait_for_connection
  is true then block until a client connects
 */
void UARTDriver::_tcp_start_connection(uint16_t port, bool wait_for_connection)
{
    int one=1;
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
        memset(&_listen_sockaddr,0,sizeof(_listen_sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
        _listen_sockaddr.sin_len = sizeof(_listen_sockaddr);
#endif
        if (port > 1000) {
            _listen_sockaddr.sin_port = htons(port);
        } else {
            _listen_sockaddr.sin_port = htons(_sitlState->base_port() + port);
        }
        _listen_sockaddr.sin_family = AF_INET;

        _listen_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (_listen_fd == -1) {
            fprintf(stderr, "socket failed - %s\n", strerror(errno));
            exit(1);
        }
        ret = fcntl(_listen_fd, F_SETFD, FD_CLOEXEC);
        if (ret == -1) {
            fprintf(stderr, "fcntl failed on setting FD_CLOEXEC - %s\n", strerror(errno));
            exit(1);
        }

        /* we want to be able to re-use ports quickly */
        if (setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == -1) {
            fprintf(stderr, "setsockopt failed: %s\n", strerror(errno));
            exit(1);
        }

        fprintf(stderr, "bind port %u for %u\n",
                (unsigned)ntohs(_listen_sockaddr.sin_port),
                (unsigned)_portNumber);

        ret = bind(_listen_fd, (struct sockaddr *)&_listen_sockaddr, sizeof(_listen_sockaddr));
        if (ret == -1) {
            fprintf(stderr, "bind failed on port %u - %s\n",
                    (unsigned)ntohs(_listen_sockaddr.sin_port),
                    strerror(errno));
            exit(1);
        }

        ret = listen(_listen_fd, 5);
        if (ret == -1) {
            fprintf(stderr, "listen failed - %s\n", strerror(errno));
            exit(1);
        }

        fprintf(stderr, "Serial port %u on TCP port %u\n", _portNumber,
                (unsigned)ntohs(_listen_sockaddr.sin_port));
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
        fcntl(_fd, F_SETFD, FD_CLOEXEC);
        _connected = true;
        fprintf(stdout, "Connection on serial port %u\n", (unsigned)ntohs(_listen_sockaddr.sin_port));
    }
}


/*
  start a TCP client connection for the serial port. 
 */
void UARTDriver::_tcp_start_client(const char *address, uint16_t port)
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
    ret = fcntl(_fd, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        fprintf(stderr, "fcntl failed on setting FD_CLOEXEC - %s\n", strerror(errno));
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
    fcntl(_fd, F_SETFD, FD_CLOEXEC);
    _connected = true;
}


/*
  start a UDP client connection for the serial port.
 */
void UARTDriver::_udp_start_client(const char *address, uint16_t port)
{
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

    _fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_fd == -1) {
        fprintf(stderr, "socket failed - %s\n", strerror(errno));
        exit(1);
    }
    ret = fcntl(_fd, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        fprintf(stderr, "fcntl failed on setting FD_CLOEXEC - %s\n", strerror(errno));
        exit(1);
    }

    // try to setup for broadcast, this may fail if insufficient privileges
    int one = 1;
    setsockopt(_fd,SOL_SOCKET,SO_BROADCAST,(char *)&one,sizeof(one));

    ret = connect(_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "udp connect failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }

    _is_udp = true;
#if HAL_GCS_ENABLED
    _packetise = true;
#endif
    _connected = true;
}

/*
  start a UDP multicast connection
 */
void UARTDriver::_udp_start_multicast(const char *address, uint16_t port)
{
    if (_connected) {
        return;
    }

    // establish the listening port
    struct sockaddr_in sockaddr;
    int ret;

    memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);

    _mc_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (_mc_fd == -1) {
        fprintf(stderr, "socket failed - %s\n", strerror(errno));
        exit(1);
    }
    ret = fcntl(_mc_fd, F_SETFD, FD_CLOEXEC);
    if (ret == -1) {
        fprintf(stderr, "fcntl failed on setting FD_CLOEXEC - %s\n", strerror(errno));
        exit(1);
    }
    int one = 1;
    if (setsockopt(_mc_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == -1) {
        fprintf(stderr, "setsockopt failed: %s\n", strerror(errno));
        exit(1);
    }

    // close on exec, to allow reboot
    fcntl(_mc_fd, F_SETFD, FD_CLOEXEC);

    ret = bind(_mc_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == -1) {
        fprintf(stderr, "multicast bind failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }

    struct ip_mreq mreq {};
    mreq.imr_multiaddr.s_addr = inet_addr(address);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    ret = setsockopt(_mc_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
    if (ret == -1) {
        fprintf(stderr, "multicast membership add failed on port %u - %s\n",
                (unsigned)ntohs(sockaddr.sin_port),
                strerror(errno));
        exit(1);
    }

    // now start the outgoing connection as an ordinary UDP connection
    _udp_start_client(address, port);
}


/*
  start a UART connection for the serial port
 */
void UARTDriver::_uart_start_connection(void)
{
    struct termios t {};
    if (!_connected) {
        _fd = ::open(_uart_path, O_RDWR | O_CLOEXEC);
        if (_fd == -1) {
            static uint32_t last_error_print_ms;
            if (AP_HAL::millis() - last_error_print_ms > 5000) {
                ::printf("Failed to open (%s): %s\n", _uart_path, strerror(errno));
                last_error_print_ms = AP_HAL::millis();
            }
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
    if (_sitlState->use_rtscts()) {
        t.c_cflag |= CRTSCTS;
    }
    tcsetattr(_fd, TCSANOW, &t);

    // set baudrate
    if (_uart_baudrate != 0) {
        set_speed(_uart_baudrate);
    }

    _connected = true;
    _use_send_recv = false;
}

/*
  see if a new connection is coming in
 */
void UARTDriver::_check_connection(void)
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
            fcntl(_fd, F_SETFD, FD_CLOEXEC);
            fprintf(stdout, "New connection on serial port %u\n", _portNumber);
        }
    }
}

/*
  use select() to see if something is pending
 */
bool UARTDriver::_select_check(int fd)
{
    if (fd == -1) {
        return false;
    }
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
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
#endif
    return false;
}

void UARTDriver::_set_nonblocking(int fd)
{
    unsigned v = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, v | O_NONBLOCK);
}

bool UARTDriver::set_unbuffered_writes(bool on) {
    if (_fd == -1) {
        return false;
    }
    _unbuffered_writes = on;

    // this has no effect
    unsigned v = fcntl(_fd, F_GETFL, 0);
    v &= ~O_NONBLOCK;
#if defined(__APPLE__) && defined(__MACH__)
    fcntl(_fd, F_SETFL | F_NOCACHE, v | O_SYNC);
#else
    fcntl(_fd, F_SETFL, v | O_DIRECT | O_SYNC);
#endif
    return _unbuffered_writes;
}

void UARTDriver::_check_reconnect(void)
{
    if (!_uart_path) {
        return;
    }
    _uart_start_connection();
}

void UARTDriver::_timer_tick(void)
{
    if (!_connected) {
        _check_reconnect();
        return;
    }
    ssize_t nwritten;
    uint32_t max_bytes = 10000;
#if !defined(HAL_BUILD_AP_PERIPH)
    SITL::SIM *_sitl = AP::sitl();
    if (_sitl && _sitl->telem_baudlimit_enable) {
        // limit byte rate to configured baudrate
        uint32_t now = AP_HAL::micros();
        float dt = 1.0e-6 * (now - last_tick_us);
        max_bytes = _uart_baudrate * dt / 10;
        if (max_bytes == 0) {
            return;
        }
        last_tick_us = now;
    }
#endif
    if (_packetise) {
        uint16_t n = _writebuffer.available();
        n = MIN(n, max_bytes);
#if HAL_GCS_ENABLED
        if (n > 0) {
            n = mavlink_packetise(_writebuffer, n);
        }
#endif
        if (n > 0) {
            // keep as a single UDP packet
            uint8_t tmpbuf[n];
            _writebuffer.peekbytes(tmpbuf, n);
            ssize_t ret = send(_fd, tmpbuf, n, MSG_DONTWAIT);
            if (ret > 0) {
                _writebuffer.advance(ret);
            }
        }
    } else {
        uint32_t navail;
        const uint8_t *readptr = _writebuffer.readptr(navail);
        if (readptr && navail > 0) {
            navail = MIN(navail, max_bytes);
            if (_sim_serial_device != nullptr) {
                nwritten = _sim_serial_device->write_to_device((const char*)readptr, navail);
            } else if (!_use_send_recv) {
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
    }

    uint32_t space = _readbuffer.space();
    if (space == 0) {
        return;
    }
    space = MIN(space, max_bytes);
    
    char buf[space];
    ssize_t nread = 0;
    if (_mc_fd >= 0) {
        if (_select_check(_mc_fd)) {
            struct sockaddr_in from;
            socklen_t fromlen = sizeof(from);
            nread = recvfrom(_mc_fd, buf, space, MSG_DONTWAIT, (struct sockaddr *)&from, &fromlen);
            uint16_t port = ntohs(from.sin_port);
            if (_mc_myport == 0) {
                // get our own address, so we can recognise packets from ourself
                struct sockaddr_in myaddr;
                socklen_t myaddrlen;
                if (getsockname(_fd, (struct sockaddr *)&myaddr, &myaddrlen) == 0) {
                    _mc_myport = ntohs(myaddr.sin_port);
                }
            }
            if (_mc_myport == port) {
                // assume this is a packet from ourselves. This is not
                // entirely accurate, as it could be a packet from
                // another machine that has assigned the same port,
                // unfortunately we don't have a better way to detect
                // packets from ourselves
                nread = 0;
            }
        }
    } else if (_sim_serial_device != nullptr) {
        nread = _sim_serial_device->read_from_device(buf, space);
    } else if (!_use_send_recv) {
        if (!_select_check(_fd)) {
            return;
        }
        int fd = _console?0:_fd;
        nread = ::read(fd, buf, space);
        if (nread == -1 && errno != EAGAIN && _uart_path) {
            close(_fd);
            _fd = -1;
            _connected = false;
        }
    } else if (_select_check(_fd)) {
        nread = recv(_fd, buf, space, MSG_DONTWAIT);
        if (nread <= 0 && !_is_udp) {
            // the socket has reached EOF
            close(_fd);
            _fd = -1;
            _connected = false;
            fprintf(stdout, "Closed connection on serial port %u\n", _portNumber);
            fflush(stdout);
            return;
        }
    }
    if (nread > 0) {
        _readbuffer.write((uint8_t *)buf, nread);
        _receive_timestamp = AP_HAL::micros64();
    }
}

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
uint64_t UARTDriver::receive_time_constraint_us(uint16_t nbytes)
{
    uint64_t last_receive_us = _receive_timestamp;
    if (_uart_baudrate > 0) {
        // assume 10 bits per byte. 
        uint32_t transport_time_us = (1000000UL * 10UL / _uart_baudrate) * (nbytes+available());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

ssize_t UARTDriver::get_system_outqueue_length() const
{
    if (!_connected) {
        return 0;
    }

#if defined(__CYGWIN__) || defined(__CYGWIN64__) || defined(CYGWIN_BUILD)
    return 0;
#elif defined(__APPLE__) && defined(__MACH__)
    return 0;
#else
    int size;
    if (ioctl(_fd, TIOCOUTQ, &size) == -1) {
        // ::fprintf(stderr, "ioctl TIOCOUTQ failed: %m\n");
        return 0;
    }
    return size;
#endif
}

#endif // CONFIG_HAL_BOARD

