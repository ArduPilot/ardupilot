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

#include <AP_HAL/AP_HAL.h>
#include <AP_Networking/AP_Networking_Config.h>
#if AP_NETWORKING_SOCKETS_ENABLED

#include "Socket.h"
#include <errno.h>

#if AP_NETWORKING_BACKEND_CHIBIOS
#define CALL_PREFIX(x) ::lwip_##x
#else
#define CALL_PREFIX(x) ::x
#endif

/*
  constructor
 */
SocketAPM::SocketAPM(bool _datagram) :
    SocketAPM(_datagram, 
              CALL_PREFIX(socket)(AF_INET, _datagram?SOCK_DGRAM:SOCK_STREAM, 0))
{}

SocketAPM::SocketAPM(bool _datagram, int _fd) :
    datagram(_datagram),
    fd(_fd)
{
#ifdef FD_CLOEXEC
    CALL_PREFIX(fcntl)(fd, F_SETFD, FD_CLOEXEC);
#endif
    if (!datagram) {
        int one = 1;
        CALL_PREFIX(setsockopt)(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    }
}

SocketAPM::~SocketAPM()
{
    if (fd != -1) {
        CALL_PREFIX(close)(fd);
        fd = -1;
    }
    if (fd_in != -1) {
        CALL_PREFIX(close)(fd_in);
        fd_in = -1;
    }
}

void SocketAPM::make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr)
{
    memset(&sockaddr, 0, sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(port);
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_addr.s_addr = inet_addr(address);
}

/*
  connect the socket
 */
bool SocketAPM::connect(const char *address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    int ret;
    int one = 1;
    make_sockaddr(address, port, sockaddr);

    if (datagram && is_multicast_address(sockaddr)) {
        /*
          connect fd_in as a multicast UDP socket
         */
        fd_in = CALL_PREFIX(socket)(AF_INET, SOCK_DGRAM, 0);
        if (fd_in == -1) {
            return false;
        }
        struct sockaddr_in sockaddr_mc = sockaddr;
        struct ip_mreq mreq {};
#ifdef FD_CLOEXEC
        CALL_PREFIX(fcntl)(fd_in, F_SETFD, FD_CLOEXEC);
#endif
        IGNORE_RETURN(CALL_PREFIX(setsockopt)(fd_in, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)));

#if defined(__CYGWIN__) || defined(__CYGWIN64__) || defined(CYGWIN_BUILD)
        /*
          on cygwin you need to bind to INADDR_ANY then use the multicast
          IP_ADD_MEMBERSHIP to get on the right address
        */
        sockaddr_mc.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
    
        ret = CALL_PREFIX(bind)(fd_in, (struct sockaddr *)&sockaddr_mc, sizeof(sockaddr));
        if (ret == -1) {
            goto fail_multi;
        }

        mreq.imr_multiaddr.s_addr = sockaddr.sin_addr.s_addr;
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);

        ret = CALL_PREFIX(setsockopt)(fd_in, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
        if (ret == -1) {
            goto fail_multi;
        }
    }

    if (datagram && sockaddr.sin_addr.s_addr == INADDR_BROADCAST) {
        // setup for bi-directional UDP broadcast
        set_broadcast();
        reuseaddress();
    }

    ret = CALL_PREFIX(connect)(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret != 0) {
        return false;
    }
    connected = true;

    if (datagram && sockaddr.sin_addr.s_addr == INADDR_BROADCAST) {
        // for bi-directional UDP broadcast we need 2 sockets
        struct sockaddr_in send_addr;
        socklen_t send_len = sizeof(send_addr);
        ret = CALL_PREFIX(getsockname)(fd, (struct sockaddr *)&send_addr, &send_len);
        fd_in = CALL_PREFIX(socket)(AF_INET, SOCK_DGRAM, 0);
        if (fd_in == -1) {
            goto fail_multi;
        }
        CALL_PREFIX(setsockopt)(fd_in, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        // 2nd socket needs to be bound to wildcard
        send_addr.sin_addr.s_addr = INADDR_ANY;
        ret = CALL_PREFIX(bind)(fd_in, (struct sockaddr *)&send_addr, sizeof(send_addr));
        if (ret == -1) {
            goto fail_multi;
        }
    }
    return true;

fail_multi:
    CALL_PREFIX(close)(fd_in);
    fd_in = -1;
    return false;
}

/*
  connect the socket with a timeout
 */
bool SocketAPM::connect_timeout(const char *address, uint16_t port, uint32_t timeout_ms)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    set_blocking(false);

    int ret = CALL_PREFIX(connect)(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (ret == 0) {
        // instant connect?
        return true;
    }
    if (errno != EINPROGRESS) {
        return false;
    }
    bool pollret = pollout(timeout_ms);
    if (!pollret) {
        return false;
    }
    int sock_error = 0;
    socklen_t len = sizeof(sock_error);
    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, (void*)&sock_error, &len) != 0) {
        return false;
    }
    connected = sock_error == 0;
    return connected;
}

/*
  bind the socket
 */
bool SocketAPM::bind(const char *address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);

    if (CALL_PREFIX(bind)(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
        return false;
    }
    return true;
}


/*
  set SO_REUSEADDR
 */
bool SocketAPM::reuseaddress(void) const
{
    int one = 1;
    return (CALL_PREFIX(setsockopt)(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != -1);
}

/*
  set blocking state
 */
bool SocketAPM::set_blocking(bool blocking) const
{
    int fcntl_ret;
    if (blocking) {
        fcntl_ret = CALL_PREFIX(fcntl)(fd, F_SETFL, CALL_PREFIX(fcntl)(fd, F_GETFL, 0) & ~O_NONBLOCK);
        if (fd_in != -1) {
            fcntl_ret |= CALL_PREFIX(fcntl)(fd_in, F_SETFL, CALL_PREFIX(fcntl)(fd_in, F_GETFL, 0) & ~O_NONBLOCK);
        }
    } else {
        fcntl_ret = CALL_PREFIX(fcntl)(fd, F_SETFL, CALL_PREFIX(fcntl)(fd, F_GETFL, 0) | O_NONBLOCK);
        if (fd_in != -1) {
            fcntl_ret |= CALL_PREFIX(fcntl)(fd_in, F_SETFL, CALL_PREFIX(fcntl)(fd_in, F_GETFL, 0) | O_NONBLOCK);
        }
    }
    return fcntl_ret != -1;
}

/*
  set cloexec state
 */
bool SocketAPM::set_cloexec() const
{
#ifdef FD_CLOEXEC
    return (CALL_PREFIX(fcntl)(fd, F_SETFD, FD_CLOEXEC) != -1);
#else
    return false;
#endif
}

/*
  send some data
 */
ssize_t SocketAPM::send(const void *buf, size_t size) const
{
    return CALL_PREFIX(send)(fd, buf, size, 0);
}

/*
  send some data
 */
ssize_t SocketAPM::sendto(const void *buf, size_t size, const char *address, uint16_t port)
{
    struct sockaddr_in sockaddr;
    make_sockaddr(address, port, sockaddr);
    return CALL_PREFIX(sendto)(fd, buf, size, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

/*
  receive some data
 */
ssize_t SocketAPM::recv(void *buf, size_t size, uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        errno = EWOULDBLOCK;
        return -1;
    }
    socklen_t len = sizeof(in_addr);
    int fin = get_read_fd();
    ssize_t ret;
    ret = CALL_PREFIX(recvfrom)(fin, buf, size, MSG_DONTWAIT, (sockaddr *)&in_addr, &len);
    if (ret <= 0) {
        return ret;
    }
    if (fd_in != -1) {
        /*
          for multicast check we are not receiving from ourselves
         */
        struct sockaddr_in send_addr;
        socklen_t send_len = sizeof(send_addr);
        if (CALL_PREFIX(getsockname)(fd, (struct sockaddr *)&send_addr, &send_len) != 0) {
            return -1;
        }
        if (in_addr.sin_port == send_addr.sin_port &&
            in_addr.sin_family == send_addr.sin_family &&
            in_addr.sin_addr.s_addr == send_addr.sin_addr.s_addr) {
            // discard packets from ourselves
            return -1;
        }
    }
    return ret;
}

/*
  return the IP address and port of the last received packet
 */
void SocketAPM::last_recv_address(const char *&ip_addr, uint16_t &port) const
{
    ip_addr = inet_ntoa(in_addr.sin_addr);
    port = ntohs(in_addr.sin_port);
}

/*
  return the IP address and port of the last received packet, using caller supplied buffer
 */
const char *SocketAPM::last_recv_address(char *ip_addr_buf, uint8_t buflen, uint16_t &port) const
{
    const char *ret = inet_ntop(AF_INET, (void*)&in_addr.sin_addr, ip_addr_buf, buflen);
    if (ret == nullptr) {
        return nullptr;
    }
    port = ntohs(in_addr.sin_port);
    return ret;
}

void SocketAPM::set_broadcast(void) const
{
    int one = 1;
    CALL_PREFIX(setsockopt)(fd,SOL_SOCKET,SO_BROADCAST,(char *)&one,sizeof(one));
}

/*
  return true if there is pending data for input
 */
bool SocketAPM::pollin(uint32_t timeout_ms)
{
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    int fin = get_read_fd();
    FD_SET(fin, &fds);

    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000UL;

    if (CALL_PREFIX(select)(fin+1, &fds, nullptr, nullptr, &tv) != 1) {
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

    if (CALL_PREFIX(select)(fd+1, nullptr, &fds, nullptr, &tv) != 1) {
        return false;
    }
    return true;
}

/* 
   start listening for new tcp connections
 */
bool SocketAPM::listen(uint16_t backlog) const
{
    return CALL_PREFIX(listen)(fd, (int)backlog) == 0;
}

/*
  accept a new connection. Only valid for TCP connections after
  listen has been used. A new socket is returned
*/
SocketAPM *SocketAPM::accept(uint32_t timeout_ms)
{
    if (!pollin(timeout_ms)) {
        return nullptr;
    }

    socklen_t len = sizeof(in_addr);
    int newfd = CALL_PREFIX(accept)(fd, (sockaddr *)&in_addr, &len);
    if (newfd == -1) {
        return nullptr;
    }
    return new SocketAPM(false, newfd);
}

/*
  return true if an address is in the multicast range
 */
bool SocketAPM::is_multicast_address(struct sockaddr_in &addr) const
{
    const uint32_t mc_lower = 0xE0000000; // 224.0.0.0
    const uint32_t mc_upper = 0xEFFFFFFF; // 239.255.255.255
    const uint32_t haddr = ntohl(addr.sin_addr.s_addr);
    return haddr >= mc_lower && haddr <= mc_upper;
}

#endif // AP_NETWORKING_BACKEND_ANY
