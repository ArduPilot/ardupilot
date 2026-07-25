// Copyright 2018 eSOL Co.,Ltd.
// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "udp_transport_datagram_internal.h"
#include "arpa/inet.h"
#include "sys/socket.h"
#include "netinet/in.h"

#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

bool uxr_init_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    bool rv = false;

    transport->fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (-1 != transport->fd)
    {
        /* Poll setup. */
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wsign-conversion"
        FD_ZERO(&transport->select_fd);
        FD_SET(transport->fd, &transport->select_fd);
        #pragma GCC diagnostic pop
        rv = true;
    }

    return rv;
}

bool uxr_close_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    (void) close(transport->fd);

    return true;
}

bool uxr_udp_send_datagram_to(
        uxrUDPTransportDatagram* transport,
        const uint8_t* buf,
        size_t len,
        const TransportLocator* locator)
{
    bool rv = true;

    struct sockaddr_in remote_addr;
    memcpy(&remote_addr.sin_addr.s_addr, &locator->_.medium_locator.address, sizeof(locator->_.medium_locator.address));
    if (0 != remote_addr.sin_addr.s_addr)
    {
        remote_addr.sin_family = AF_INET;
        remote_addr.sin_port = htons(locator->_.medium_locator.locator_port);

        int32_t bytes_sent = sendto(transport->fd, (void*)buf, len, 0,
                        (struct sockaddr*)&remote_addr, sizeof(remote_addr));

        if (0 >= bytes_sent)
        {
            rv = false;
        }
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

    struct timeval tv;
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;

    fd_set fds = transport->select_fd;
    int32_t poll_rv = select(transport->fd + 1, &fds, NULL, NULL, &tv);
    if (0 < poll_rv)
    {
        int32_t bytes_received = recvfrom(transport->fd, (void*)transport->buffer, sizeof(transport->buffer),
                        0, NULL, NULL);
        if (0 < bytes_received)
        {
            *len = (size_t)bytes_received;
            *buf = transport->buffer;
            rv = true;
        }
    }
    else if (0 == poll_rv)
    {
        errno = ETIME;
    }

    return rv;
}

void uxr_bytes_to_ip(
        const uint8_t* bytes,
        char* ip)
{
    uint32_t addr;
    addr = (uint32_t)(*bytes + (*(bytes + 1) << 8) + (*(bytes + 2) << 16) + (*(bytes + 3) << 24));
    struct in_addr saddr;
    saddr.s_addr = addr;
    char* res = inet_ntoa(saddr);
    strncpy(ip, res, 16); // 16 == MAX_IPv4_LEN
}
