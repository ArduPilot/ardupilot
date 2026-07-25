#include <uxr/client/profile/transport/ip/tcp/tcp_transport_posix.h>
#include "tcp_transport_internal.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <netdb.h>

#ifdef UCLIENT_PLATFORM_LINUX
static void sigpipe_handler(
        int fd)
{
    (void)fd;
}

#endif /* ifdef UCLIENT_PLATFORM_LINUX */

bool uxr_init_tcp_platform(
        struct uxrTCPPlatform* platform,
        uxrIpProtocol ip_protocol,
        const char* ip,
        const char* port)
{
    bool rv = false;

    switch (ip_protocol)
    {
        case UXR_IPv4:
            platform->poll_fd.fd = socket(AF_INET, SOCK_STREAM, 0);
            break;
        case UXR_IPv6:
            platform->poll_fd.fd = socket(AF_INET6, SOCK_STREAM, 0);
            break;
    }

    if (-1 != platform->poll_fd.fd)
    {
#ifdef UCLIENT_PLATFORM_LINUX
        signal(SIGPIPE, sigpipe_handler);
#endif /* ifdef UCLIENT_PLATFORM_LINUX */
        struct addrinfo hints;
        struct addrinfo* result;
        struct addrinfo* ptr;

        memset(&hints, 0, sizeof(hints));
        switch (ip_protocol)
        {
            case UXR_IPv4:
                hints.ai_family = AF_INET;
                break;
            case UXR_IPv6:
                hints.ai_family = AF_INET6;
                break;
        }
        hints.ai_socktype = SOCK_STREAM;

        if (0 == getaddrinfo(ip, port, &hints, &result))
        {
            for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
            {
                if (0 == connect(platform->poll_fd.fd, ptr->ai_addr, ptr->ai_addrlen))
                {
                    platform->poll_fd.events = POLLIN;
                    rv = true;
                    break;
                }
            }
        }
        freeaddrinfo(result);
    }
    return rv;
}

bool uxr_close_tcp_platform(
        struct uxrTCPPlatform* platform)
{
    return (-1 == platform->poll_fd.fd) ? true : (0 == close(platform->poll_fd.fd));
}

size_t uxr_write_tcp_data_platform(
        struct uxrTCPPlatform* platform,
        const uint8_t* buf,
        size_t len,
        uint8_t* errcode)
{
    size_t rv = 0;
    ssize_t bytes_sent = send(platform->poll_fd.fd, (void*)buf, len, 0);
    if (-1 != bytes_sent)
    {
        rv = (size_t)bytes_sent;
        *errcode = 0;
    }
    else
    {
        *errcode = 1;
    }
    return rv;
}

size_t uxr_read_tcp_data_platform(
        struct uxrTCPPlatform* platform,
        uint8_t* buf,
        size_t len,
        int timeout,
        uint8_t* errcode)
{
    size_t rv = 0;
    int poll_rv = poll(&platform->poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
        ssize_t bytes_received = recv(platform->poll_fd.fd, (void*)buf, len, 0);
        if (-1 != bytes_received)
        {
            rv = (size_t)bytes_received;
            *errcode = 0;
        }
        else
        {
            *errcode = 1;
        }
    }
    else
    {
        *errcode = (0 == poll_rv) ? 0 : 1;
    }
    return rv;
}

void uxr_disconnect_tcp_platform(
        struct uxrTCPPlatform* platform)
{
    close(platform->poll_fd.fd);
    platform->poll_fd.fd = -1;
}
