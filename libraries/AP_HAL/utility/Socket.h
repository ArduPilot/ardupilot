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
/*
  simple socket handling class for systems with BSD socket API
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Networking/AP_Networking_Config.h>
#if AP_NETWORKING_SOCKETS_ENABLED

#if HAL_OS_SOCKETS

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>
#elif AP_NETWORKING_BACKEND_CHIBIOS
#include <AP_Networking/AP_Networking_ChibiOS.h>
#include <lwip/sockets.h>
#endif

class SocketAPM {
public:
    SocketAPM(bool _datagram);
    SocketAPM(bool _datagram, int _fd);
    ~SocketAPM();

    bool connect(const char *address, uint16_t port);
    bool connect_timeout(const char *address, uint16_t port, uint32_t timeout_ms);
    bool bind(const char *address, uint16_t port);
    bool reuseaddress() const;
    bool set_blocking(bool blocking) const;
    bool set_cloexec() const;
    void set_broadcast(void) const;

    ssize_t send(const void *pkt, size_t size) const;
    ssize_t sendto(const void *buf, size_t size, const char *address, uint16_t port);
    ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

    // return the IP address and port of the last received packet
    void last_recv_address(const char *&ip_addr, uint16_t &port) const;

    // return the IP address and port of the last received packet, using caller supplied buffer
    const char *last_recv_address(char *ip_addr_buf, uint8_t buflen, uint16_t &port) const;

    // return true if there is pending data for input
    bool pollin(uint32_t timeout_ms);

    // return true if there is room for output data
    bool pollout(uint32_t timeout_ms);

    // start listening for new tcp connections
    bool listen(uint16_t backlog) const;

    // accept a new connection. Only valid for TCP connections after
    // listen has been used. A new socket is returned
    SocketAPM *accept(uint32_t timeout_ms);

    // get a FD suitable for read selection
    int get_read_fd(void) const {
        return fd_in != -1? fd_in : fd;
    }

    bool is_connected(void) const {
        return connected;
    }

private:
    bool datagram;
    struct sockaddr_in in_addr {};
    bool is_multicast_address(struct sockaddr_in &addr) const;

    int fd = -1;

    // fd_in is used for multicast UDP
    int fd_in = -1;

    bool connected;

    void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr);
};

#endif // AP_NETWORKING_SOCKETS_ENABLED
