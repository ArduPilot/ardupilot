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

#ifndef HAL_SOCKET_H
#define HAL_SOCKET_H

#include <AP_HAL.h>
#if HAL_OS_SOCKETS

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <netdb.h>

class SocketAPM {
public:
    SocketAPM(bool _datagram);
    SocketAPM(bool _datagram, int _fd);
    ~SocketAPM();

    bool connect(const char *hostname, uint16_t port);
    bool bind(const char *hostname, uint16_t port);
    void reuseaddress();
    void set_blocking(bool blocking);
    void set_broadcast(void);

    ssize_t send(const void *pkt, size_t size);
    ssize_t sendto(const void *buf, size_t size, const char *hostname, uint16_t port);
    ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

    // return the IP address and port of the last received packet
    bool last_recv_address(char *hostname, uint16_t *port);

    // return true if there is pending data for input
    bool pollin(uint32_t timeout_ms);

    // return true if there is room for output data
    bool pollout(uint32_t timeout_ms);

    // start listening for new tcp connections
    bool listen(uint16_t backlog);

    // accept a new connection. Only valid for TCP connections after
    // listen has been used. A new socket is returned
    SocketAPM *accept(uint32_t timeout_ms);

private:
    bool datagram;
    struct sockaddr_storage in_addr {};

    int fd = -1;

    struct addrinfo *get_address_info(const char *hostname, uint16_t port);
    struct addrinfo *select_address(struct addrinfo *address_list);
};

#endif // HAL_OS_SOCKETS
#endif // HAL_SOCKET_H
