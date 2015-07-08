#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netdb.h>

#include "TCPServerDevice.h"

TCPServerDevice::TCPServerDevice(const char *ip, uint16_t port):
    _ip(ip),
    _port(port)
{
}

TCPServerDevice::~TCPServerDevice()
{

}

int TCPServerDevice::write(const uint8_t *buf, uint16_t n)
{
    return ::send(_net_fd, buf, n, 0);
}

int TCPServerDevice::read(uint8_t *buf, uint16_t n)
{
    return ::recv(_net_fd, buf, n, 0);
}

bool TCPServerDevice::open()
{
    int one=1;
    struct sockaddr_in sockaddr;
    int ret;
    int client_fd = -1;
    uint8_t portNumber = 0; /* connecto to _port + portNumber */

    memset(&sockaddr,0,sizeof(sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
    sockaddr.sin_len = sizeof(sockaddr);
#endif
    sockaddr.sin_port = htons(_port + portNumber);
    sockaddr.sin_family = AF_INET;

    if (strcmp(_ip, "*") == 0) {
        /* bind to all interfaces */
        sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        sockaddr.sin_addr.s_addr = inet_addr(_ip);
    }

    _listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (_listen_fd == -1) {
        ::printf("socket failed - %s\n", strerror(errno));
        exit(1);
    }

    /* we want to be able to re-use ports quickly */
    setsockopt(_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    ret = bind(_listen_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr));

    if (ret == -1) {
        ::printf("bind failed on port %u - %s\n",
                 (unsigned)ntohs(sockaddr.sin_port),
                 strerror(errno));
        exit(1);
    }

    ret = listen(_listen_fd, 5);
    if (ret == -1) {
        ::printf("listen failed - %s\n", strerror(errno));
        exit(1);
    }

    ::printf("Serial port %u on TCP port %u\n", portNumber, 
             _port + portNumber);
    ::fflush(stdout);

    ::printf("Waiting for connection ....\n");
    ::fflush(stdout);

    struct sockaddr_storage client_addr;
    socklen_t addr_size;

    client_fd = accept(_listen_fd, (struct sockaddr *) &client_addr, &addr_size);

    if (client_fd == -1) {
        ::printf("accept() error - %s", strerror(errno));
        exit(1);
    }

    struct sockaddr_in *sa4 = (struct sockaddr_in*) &client_addr;
    printf("%s connected\n", inet_ntoa(sa4->sin_addr) );

    setsockopt(client_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    /* always run the file descriptor non-blocking, and deal with                                         |
    *  blocking IO in the higher level calls */
    fcntl(client_fd, F_SETFL, fcntl(client_fd, F_GETFL, 0) | O_NONBLOCK);

    _net_fd = client_fd;

    return true;
}

bool TCPServerDevice::close()
{
    if (::close(_listen_fd) < 0) {
        perror("close");
        return false;
    }

    if (::close(_net_fd) < 0) {
        perror("close");
        return false;
    }

    return true;
}

void TCPServerDevice::set_nonblocking()
{
}

void TCPServerDevice::set_speed(uint32_t speed)
{

}

#endif
