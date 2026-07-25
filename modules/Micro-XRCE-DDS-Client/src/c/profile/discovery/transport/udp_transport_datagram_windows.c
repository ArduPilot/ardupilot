#include "udp_transport_datagram_internal.h"

#include <WS2tcpip.h>
#include <string.h>

bool uxr_init_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    bool rv = false;

    /* WSA initialization. */
    WSADATA wsa_data;
    if (0 != WSAStartup(MAKEWORD(2, 2), &wsa_data))
    {
        return false;
    }

    /* Socket initialization. */
    transport->poll_fd.fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (INVALID_SOCKET != transport->poll_fd.fd)
    {
        transport->poll_fd.events = POLLIN;
        rv = true;
    }
    return rv;
}

bool uxr_close_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    bool rv = (INVALID_SOCKET == transport->poll_fd.fd) ? true : (0 == closesocket(transport->poll_fd.fd));
    return (0 == WSACleanup()) && rv;
}

bool uxr_udp_send_datagram_to(
        uxrUDPTransportDatagram* transport,
        const uint8_t* buf,
        size_t len,
        const TransportLocator* locator)
{
    bool rv = false;
    switch (locator->format)
    {
        case ADDRESS_FORMAT_MEDIUM:
        {
            struct sockaddr_in remote_addr;
            memcpy(&remote_addr.sin_addr, locator->_.medium_locator.address, sizeof(remote_addr.sin_addr));
            remote_addr.sin_family = AF_INET;
            remote_addr.sin_port = htons(locator->_.medium_locator.locator_port);

            int bytes_sent = sendto(transport->poll_fd.fd, (const char*)buf, (int)len, 0,
                            (struct sockaddr*)&remote_addr, sizeof(remote_addr));
            rv = (SOCKET_ERROR != bytes_sent);
            break;
        }
        default:
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

    int poll_rv = WSAPoll(&transport->poll_fd, 1, timeout);
    if (0 < poll_rv)
    {
        int bytes_received = recv(transport->poll_fd.fd, (char*)transport->buffer, (int)sizeof(transport->buffer), 0);
        if ( bytes_received)
        {
            *len = (size_t)bytes_received;
            *buf = transport->buffer;
            rv = true;
        }
    }
    else if (0 == poll_rv)
    {
        WSASetLastError(WSAETIMEDOUT);
    }

    return rv;
}

void uxr_bytes_to_ip(
        const uint8_t* bytes,
        char* ip)
{
    struct in_addr addr;
    addr.s_addr = (unsigned long)(*bytes + (*(bytes + 1) << 8) + (*(bytes + 2) << 16) + (*(bytes + 3) << 24));
    inet_ntop(AF_INET, &(addr.s_addr), ip, INET_ADDRSTRLEN);
}
