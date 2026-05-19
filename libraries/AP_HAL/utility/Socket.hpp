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

#if AP_NETWORKING_SOCKETS_ENABLED || defined(AP_SOCKET_NATIVE_ENABLED)

#ifndef SOCKET_CLASS_NAME
#error "Don't include Socket.hpp directly"
#endif

#define IP4_STR_LEN 16

class SOCKET_CLASS_NAME {
public:
    SOCKET_CLASS_NAME(bool _datagram);
    SOCKET_CLASS_NAME(bool _datagram, int _fd);
    ~SOCKET_CLASS_NAME();

    bool connect(const char *address, uint16_t port);
    bool connect_timeout(const char *address, uint16_t port, uint32_t timeout_ms);
    bool bind(const char *address, uint16_t port);
    bool reuseaddress() const;
    bool set_blocking(bool blocking) const;
    bool set_cloexec() const;
    void set_broadcast(void) const;

    ssize_t send(const void *pkt, size_t size) const;
    ssize_t sendto(const void *buf, size_t size, const char *address, uint16_t port);
    ssize_t sendto(const void *buf, size_t size, uint32_t address, uint16_t port);
    ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

    // return the IP address and port of the last received packet
    void last_recv_address(const char *&ip_addr, uint16_t &port) const;

    // return the IP address and port of the last received packet, using caller supplied buffer
    const char *last_recv_address(char *ip_addr_buf, uint8_t buflen, uint16_t &port) const;

    // return the IP address and port of the last received packet
    bool last_recv_address(uint32_t &ip_addr, uint16_t &port) const;

    // return true if there is pending data for input
    bool pollin(uint32_t timeout_ms);

    // return true if there is room for output data
    bool pollout(uint32_t timeout_ms);

    // start listening for new tcp connections
    bool listen(uint16_t backlog) const;

    // close socket
    void close(void);

    // accept a new connection. Only valid for TCP connections after
    // listen has been used. A new socket is returned
    SOCKET_CLASS_NAME *accept(uint32_t timeout_ms);

    // get a FD suitable for read selection
    int get_read_fd(void) const {
        return fd_in != -1? fd_in : fd;
    }

    // create a new socket with same fd, but new memory
    // the old socket gets fd of -1
    SOCKET_CLASS_NAME *duplicate(void);

    bool is_connected(void) const {
        return connected;
    }

    bool is_pending(void) const {
        return pending_connect;
    }
    
    // access to inet_ntop
    static const char *inet_addr_to_str(uint32_t addr, char *dst, uint16_t len);

    // access to inet_pton
    static uint32_t inet_str_to_addr(const char *ipstr);
    
private:
    bool datagram;
    // we avoid using struct sockaddr_in here to keep support for
    // mixing native sockets and lwip sockets in SITL
    uint32_t last_in_addr[4];
    bool is_multicast_address(struct sockaddr_in &addr) const;

    int fd = -1;

    // fd_in is used for multicast UDP
    int fd_in = -1;

    bool connected;

    bool pending_connect;

    void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr);
};

#endif // AP_NETWORKING_SOCKETS_ENABLED
