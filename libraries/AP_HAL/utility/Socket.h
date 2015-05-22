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

class SocketAPM {
public:
    SocketAPM(bool _datagram);
    bool connect(const char *address, uint16_t port);
    bool bind(const char *address, uint16_t port);
    void reuseaddress();
    void set_blocking(bool blocking);

    ssize_t send(void *pkt, size_t size);
    ssize_t sendto(void *buf, size_t size, const char *address, uint16_t port);
    ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

private:
    bool datagram;
    int fd;

    void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr);
};

#endif // HAL_OS_SOCKETS
#endif // HAL_SOCKET_H
