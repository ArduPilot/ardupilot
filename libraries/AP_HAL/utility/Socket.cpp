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

#include <AP_HAL.h>
#if HAL_OS_SOCKETS

#include "Socket.h"

#include <cstdio>
#include <cstdlib>
#include <limits.h>

/*
  constructor
 */
SocketAPM::SocketAPM(bool _datagram) :
    SocketAPM(_datagram, 
              socket(AF_INET, _datagram?SOCK_DGRAM:SOCK_STREAM, 0))
{}

SocketAPM::SocketAPM(bool _datagram, int _fd) :
    datagram(_datagram),
    fd(_fd)
{
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    if (!datagram) {
        int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }
}

SocketAPM::~SocketAPM()
{
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

struct addrinfo *SocketAPM::get_address_info(const char *address, uint16_t port)
{
    char service[20] = {0};
    int service_length = sizeof(service);

    int ret;

    ret = snprintf(service, service_length, "%d", port);
    
    if (ret < 0 || ret == service_length) {
        return NULL;
    }

    struct addrinfo hints;
    struct addrinfo *address_info;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = datagram? SOCK_DGRAM: SOCK_STREAM;

    ret = getaddrinfo(address, service, &hints, &address_info);
    if (ret < 0) {
        return NULL;
    }

    return address_info;
}

struct addrinfo *SocketAPM::select_address(struct addrinfo *address_list)
{
    for (struct addrinfo *address = address_list; address != NULL; address = address->ai_next) {
        if (address->ai_family == AF_INET) {
            /* connect to first IPv4 address */
            return address; 
        } 
    }

    return NULL;
}

/*
  connect the socket
 */
bool SocketAPM::connect(const char *hostname, uint16_t port)
{
    struct addrinfo *address_list = get_address_info(hostname, port);

    if (address_list == NULL) {
        return false;
    }

    struct addrinfo *address = select_address(address_list);

    if (address == NULL) {
        goto errout;
    }

    if (::connect(fd, address->ai_addr, address->ai_addrlen) != 0) {
        goto errout;
    }

    freeaddrinfo(address_list);
    return true;

errout:
    freeaddrinfo(address_list);
    return false;
}

/*
  bind the socket
 */
bool SocketAPM::bind(const char *hostname, uint16_t port)
{
    struct addrinfo *address_list = get_address_info(hostname, port);

    if (address_list == NULL) {
        return false;
    }

    struct addrinfo *address = select_address(address_list);

    if (address == NULL) {
        goto errout;
    }

    if (::bind(fd, address->ai_addr, address->ai_addrlen) != 0) {
        goto errout;
    }

    freeaddrinfo(address_list);
    return true;

errout:
    freeaddrinfo(address_list);
    return false;
}


/*
  set SO_REUSEADDR
 */
void SocketAPM::reuseaddress(void)
{
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
}

/*
  set blocking state
 */
void SocketAPM::set_blocking(bool blocking)
{
    if (blocking) {
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) & ~O_NONBLOCK);
    } else {
        fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
    }
}

/*
  send some data
 */
ssize_t SocketAPM::send(const void *buf, size_t size)
{
    return ::send(fd, buf, size, 0);
}

/*
  send some data
 */
ssize_t SocketAPM::sendto(const void *buf, size_t size, const char *hostname, uint16_t port)
{
    int ret = -1;

    struct addrinfo *address_list;
    struct addrinfo *address;

    address_list = get_address_info(hostname, port);

    if (address_list == NULL) {
        goto errout_without_list;
    }

    address = select_address(address_list);

    if (address == NULL) {
        goto errout;
    }

    ret = ::sendto(fd, buf, size, 0, address->ai_addr, address->ai_addrlen);
    if (ret < 0) {
        goto errout;
    }

errout:
    freeaddrinfo(address_list);

errout_without_list:
    return ret;
}

/*
  receive some data
 */
ssize_t SocketAPM::recv(void *buf, size_t size, uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        return -1;
    }
    socklen_t len = sizeof(in_addr);
    return ::recvfrom(fd, buf, size, MSG_DONTWAIT, (sockaddr *)&in_addr, &len);
}

/*
  return the IP address and port of the last received packet
 */
bool SocketAPM::last_recv_address(char *hostname, uint16_t *port)
{
    const size_t SERVICE_NAME_MAX = 20;

    char service[SERVICE_NAME_MAX] = {0};

    if (int ret = getnameinfo((struct sockaddr *)&in_addr, sizeof(in_addr), hostname, HOST_NAME_MAX, 
                service, SERVICE_NAME_MAX, 0)) {
        fprintf(stderr, "getnameinfo: %s\n", gai_strerror(ret));
        return false;
    }

    char *endptr;

    *port = strtol(service, &endptr, 0);

    if (*endptr != '\0' || service == endptr) {
        return false;
    }

    return true;
}

void SocketAPM::set_broadcast(void)
{
    int one = 1;
    setsockopt(fd,SOL_SOCKET,SO_BROADCAST,(char *)&one,sizeof(one));
}

/*
  return true if there is pending data for input
 */
bool SocketAPM::pollin(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (select(fd+1, &fds, NULL, NULL, &tv) != 1) {
        return false;
    }
    return true;
}


/*
  return true if there is room for output data
 */
bool SocketAPM::pollout(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (select(fd+1, NULL, &fds, NULL, &tv) != 1) {
        return false;
    }
    return true;
}

/* 
   start listening for new tcp connections
 */
bool SocketAPM::listen(uint16_t backlog)
{
    return ::listen(fd, (int)backlog) == 0;
}

/*
  accept a new connection. Only valid for TCP connections after
  listen has been used. A new socket is returned
*/
SocketAPM *SocketAPM::accept(uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        return NULL;
    }

    int newfd = ::accept(fd, NULL, NULL);
    if (newfd == -1) {
        return NULL;
    }
    // turn off nagle for lower latency
    int one = 1;
    setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return new SocketAPM(false, newfd);
}

#endif // HAL_OS_SOCKETS
