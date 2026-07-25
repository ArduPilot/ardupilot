#include "udp_transport_datagram_internal.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

bool uxr_init_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    int fd = socket(PF_INET, SOCK_DGRAM, 0);
    transport->fd = fd;

    return fd != -1;
}

bool uxr_close_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    return (0 == close(transport->fd));
}

bool uxr_udp_send_datagram_to(
        uxrUDPTransportDatagram* transport,
        const uint8_t* buf,
        size_t len,
        const TransportLocator* locator)
{
    bool rv = true;
    switch (locator->format)
    {
        case ADDRESS_FORMAT_MEDIUM:
        {
            struct sockaddr_in remote_addr;
            memcpy(&remote_addr.sin_addr, locator->_.medium_locator.address, sizeof(remote_addr.sin_addr));
            remote_addr.sin_family = AF_INET;
            remote_addr.sin_port = htons(locator->_.medium_locator.locator_port);

            ssize_t bytes_sent = sendto(transport->fd, (const void*)buf, len, 0,
                            (struct sockaddr*)&remote_addr, sizeof(remote_addr));
            if (0 > bytes_sent)
            {
                rv = false;
            }
            break;
        }
        default:
            rv = false;
            break;
    }
    return rv;
}

bool uxr_udp_recv_datagram(
        uxrUDPTransportDatagram* transport,
        uint8_t** buf,
        size_t* len,
        int timeout)
{
    bool rv = false;

    timeout = (timeout <= 0) ? 1 : timeout;

    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    if (0 != setsockopt(transport->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)))
    {
        return false;
    }

    ssize_t bytes_received = recv(transport->fd, (void*)transport->buffer, sizeof(transport->buffer), 0);
    if (-1 != bytes_received)
    {
        *len = (size_t)bytes_received;
        *buf = transport->buffer;
        rv = true;
    }

    return rv;
}

void uxr_bytes_to_ip(
        const uint8_t* bytes,
        char* ip)
{
    struct in_addr addr;
    addr.s_addr = (in_addr_t)(*bytes + (*(bytes + 1) << 8) + (*(bytes + 2) << 16) + (*(bytes + 3) << 24));
    char* internal_ip = inet_ntoa(addr);
    strcpy(ip, internal_ip);
}
