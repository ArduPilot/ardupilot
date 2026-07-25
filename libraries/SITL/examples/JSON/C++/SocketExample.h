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

#include <cstring>
#include <iostream>
#include <string>

#ifdef _WIN32
  //#pragma comment(lib, "Ws2_32.lib")
  //#define WIN32_LEAN_AND_MEAN
  //#include <Windows.h>  
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #include <stdlib.h>
  #include <stdio.h>
  #include <stdint.h>
  #include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#else
  #include <fcntl.h>
  #include <unistd.h>
  #include <sys/ioctl.h>
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <sys/select.h>
#endif

class SocketExample {
public:
    SocketExample();
    //SocketExample(bool _datagram, int _fd);
    ~SocketExample();

    bool connect(const char *address, uint16_t port);
    bool bind(const char *address, uint16_t port);
    bool reuseaddress() const;
    bool set_blocking(bool blocking) const;
    //bool set_cloexec() const;
    //void set_broadcast(void) const;

    ssize_t send(const void *pkt, size_t size) const;
    ssize_t sendto(const void *buf, size_t size, const char *address, uint16_t port);
    ssize_t recv(void *pkt, size_t size, uint32_t timeout_ms);

    // return the IP address and port of the last received packet
    void last_recv_address(const char *&ip_addr, uint16_t &port) const;

    // return true if there is pending data for input
    bool pollin(uint32_t timeout_ms);

    // return true if there is room for output data
    //bool pollout(uint32_t timeout_ms);

    // start listening for new tcp connections
    bool listen(uint16_t backlog) const;

    // accept a new connection. Only valid for TCP connections after
    // listen has been used. A new socket is returned
    //SocketExample *accept(uint32_t timeout_ms);

private:
    bool datagram;
    struct sockaddr_in in_addr {};

#ifdef _WIN32
    SOCKET fd;
#else
    int fd = -1;
#endif // _WIN32

    void make_sockaddr(const char *address, uint16_t port, struct sockaddr_in &sockaddr);

    //bool socket_init(bool& _datagram, int &_fd);
#ifdef _WIN32
    bool init_WSA();
#endif // _WIN32

};
