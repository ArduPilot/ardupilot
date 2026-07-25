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
#include "FreeRTOS.h"
#include "FreeRTOS_Sockets.h"

#include <unistd.h>
#include <errno.h>
#include <string.h>

bool uxr_init_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    bool rv = false;

    transport->fd = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);

    if (FREERTOS_INVALID_SOCKET != transport->fd)
    {
        transport->poll_fd = FreeRTOS_CreateSocketSet();

        if (NULL != transport->poll_fd)
        {
            /* FreeRTOS_FD_SET() is a void function. */
            FreeRTOS_FD_SET(transport->fd, transport->poll_fd, eSELECT_READ);
            rv = true;
        }
    }

    return rv;
}

bool uxr_close_udp_transport_datagram(
        uxrUDPTransportDatagram* transport)
{
    /* FreeRTOS_closesocket() always returns 0. */
    (void) FreeRTOS_closesocket(transport->fd);

    if (NULL != transport->poll_fd)
    {
        (void) FreeRTOS_DeleteSocketSet(transport->poll_fd);
    }

    return true;
}

bool uxr_udp_send_datagram_to(
        uxrUDPTransportDatagram* transport,
        const uint8_t* buf,
        size_t len,
        const TransportLocator* locator)
{
    bool rv = true;

    struct freertos_sockaddr remote_addr;
    memcpy(&remote_addr.sin_addr, &locator->_.medium_locator.address, sizeof(locator->_.medium_locator.address));
    if (0 != remote_addr.sin_addr)
    {
        remote_addr.sin_family = FREERTOS_AF_INET;
        remote_addr.sin_port = FreeRTOS_htons(locator->_.medium_locator.locator_port);

        int32_t bytes_sent = FreeRTOS_sendto(transport->fd, (void*)buf, len, 0,
                        (struct freertos_sockaddr*)&remote_addr, sizeof(remote_addr));

        /* FreeRTOS_sendto() returns 0 if an error or timeout occurred. */
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

    BaseType_t poll_rv = FreeRTOS_select(transport->poll_fd, pdMS_TO_TICKS(timeout));
    if (0 < poll_rv)
    {
        int32_t bytes_received = FreeRTOS_recvfrom(transport->fd, (void*)transport->buffer, sizeof(transport->buffer),
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
    FreeRTOS_inet_ntoa(addr, ip);
}
